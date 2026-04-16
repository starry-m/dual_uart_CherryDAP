# 使用HPM5301 debug HPM6E80 && GW1NR-LV9

## 硬件引脚分配

|引脚|功能|
|-|-|
|PB08|LED_G|
|PB09|LED_B|
|PB10|MCU_TMS|
|PB11|MCU_TCK|
|PB12|MCU_TDO|
|PB13|MCU_TDI|
|PB14|MCU_TRST|
|PB15|MCU_SRST|
|PA02|MSC_UF2|
|PA04|FPGA_TMS|
|PA05|FPGA_TCK|
|PA06|FPGA_TDO|
|PA07|FPGA_TDI|
|PA08|MCU_UART_TX|
|PA09|MCU_UART_RX|
|PA30|MODE_SWITCH|

## 软件适配

- MCU：现有基于cherryusb方案的cherrydap hslink方案完全满足，只需修改引脚即可。
- FPGA：参考https://github.com/sipeed/RV-Debugger-BL702/tree/main/firmware/app/usb2uartjtag
因为这个也是使用的cherryusb协议栈实现模拟ft2232方案（已实现）

---

## FPGA JTAG 实现详解

### 整体思路

目标：让 HPM5301 能够给板载 GW1NR-LV9 FPGA 下载比特流文件。

高云（Gowin）Programmer 软件使用 FTDI D2XX 驱动与 FT2232H 芯片通信，通过 **MPSSE（Multi-Protocol Synchronous Serial Engine）** 协议来驱动 JTAG 引脚。因此我们的方案是：

1. 在 USB 层面，让 HPM5301 **模拟一个 FT2232H 双通道设备**（VID=0x0403, PID=0x6010）
2. 在协议层面，实现 **FTDI vendor request** 处理（EEPROM 模拟、波特率、MPSSE 模式切换等）
3. 在数据层面，实现 **MPSSE 命令解析引擎**，将 MPSSE 命令翻译为 GPIO bit-bang 操作
4. 通过 PA30 按键在 **DAP 模式**和 **FT2232 模式**之间切换，切换时进行 USB 重新枚举

整个数据流如下：

```
高云Programmer → D2XX驱动 → USB Bulk传输 → HPM5301 USB端点回调
    → MPSSE引擎解析命令 → GPIO bit-bang驱动JTAG引脚 → GW1NR FPGA
    → TDO回读 → MPSSE引擎组装响应 → USB Bulk回传 → D2XX驱动 → Programmer
```

### 文件结构与职责

| 文件 | 层次 | 职责 |
|-|-|-|
| `fpga_jtag_io.h` | 硬件层 | GPIO引脚定义、初始化、bit-bang宏 |
| `fpga_jtag_mpsse.h/c` | 协议层 | MPSSE命令状态机，JTAG时序生成 |
| `usbd_ftdi.h/c` | USB控制层 | FTDI vendor request处理，设备配置 |
| `fpga_jtag_main.c` | 集成层 | USB描述符、端点回调、数据流调度 |
| `mode_switch.h/c` | 应用层 | PA30按键消抖与模式切换 |

下面按从底层到上层的顺序详细介绍。

---

### 第一层：GPIO 硬件操作（fpga_jtag_io.h）

这是最底层，负责用 GPIO 模拟 JTAG 四线接口：

```
PA04 → TMS（Test Mode Select，输出）
PA05 → TCK（Test Clock，输出）
PA06 → TDO（Test Data Out，输入，带上拉）
PA07 → TDI（Test Data In，输出）
```

#### 初始化

`fpga_jtag_gpio_init()` 完成以下操作：

1. **配置 IOC**：所有引脚设为 GPIO 功能（ALT0），中等驱动强度
2. **TDO 上拉**：FPGA 在某些状态下会将 TDO 三态，不上拉会导致读到随机值
3. **配置 GPIOM**：将引脚控制器设为 `gpiom_core0_fast`，映射到 **FGPIO**（Fast GPIO）
4. **方向设置**：TMS/TCK/TDI 为输出，TDO 为输入
5. **初始电平**：全部拉低

#### 为什么用 FGPIO？

HPM5301 有两套 GPIO 外设：普通 GPIO 和 FGPIO。FGPIO 通过 CPU 的 fast I/O 总线直接访问，不走 AHB 总线，单次操作只需 1 个时钟周期。对 JTAG bit-bang 来说速度至关重要——每传输一个 bit 需要操作 TCK 两次 + 设置 TDI + 读取 TDO。

#### Fence 指令

每个 GPIO 操作宏后面都加了 `__asm volatile("fence io, io")`。这是 RISC-V 的 I/O 排序屏障——确保前一个 GPIO 写入在下一个操作之前真正到达外设寄存器。没有 fence 的话，CPU 流水线可能会重排 I/O 操作，导致时序错乱（比如 TDI 还没稳定就拉高了 TCK）。

#### 延时控制

```c
#define FPGA_JTAG_DELAY()
    do {
        __asm volatile("fence io, io");
        volatile uint32_t _cnt = 5;
        while (_cnt--) { __asm volatile(""); }
    } while(0)
```

用一个简单的忙等循环来控制 TCK 的半周期宽度。`volatile` 防止编译器优化掉循环。

---

### 第二层：MPSSE 协议引擎（fpga_jtag_mpsse.c）

这是整个实现的核心——一个 **18 状态的状态机**，将 USB 收到的 MPSSE 命令字节流翻译为 GPIO 操作。

#### MPSSE 协议简介

MPSSE 是 FTDI 芯片内置的硬件引擎，原本用于在 FT2232H 内部直接驱动 SPI/I2C/JTAG 等同步串行协议。我们用软件模拟它。

MPSSE 命令是一个紧凑的二进制协议：

```
[命令字节] [参数字节...] [数据字节...]
```

命令字节的各个 bit 有特定含义：

| Bit | 含义 |
|-|-|
| Bit 0 | 写入时钟边沿（0=上升沿，1=下降沿） |
| Bit 1 | 位序（0=MSB first，1=LSB first） |
| Bit 2 | 读TDO（0=不读，1=读） |
| Bit 3 | 写TDI（0=不写，1=写） |
| Bit 4 | TMS模式（0=普通数据，1=TMS输出） |
| Bit 5 | 字节/位（0=字节传输，1=位传输） |

常见的命令例子：

- `0x19`：写字节，LSB first，下降沿时钟（最常用的 JTAG 写命令）
- `0x39`：同时读写字节，LSB first
- `0x6B`：TMS 位输出（用于 TAP 状态机导航）
- `0x80`：设置 ADBUS 引脚电平（直接控制 TCK/TDI/TMS）
- `0x81`：读取 ADBUS 引脚状态（读取 TDO）

#### 状态机结构

```
MPSSE_IDLE → 解析命令字节，分派到对应状态
    ├─ 字节传输: → RCV_LENGTH_L → RCV_LENGTH_H → TRANSMIT_BYTE/TRANSMIT_BYTE_MSB
    ├─ 位传输:   → RCV_LENGTH → TRANSMIT_BIT/TRANSMIT_BIT_MSB
    ├─ TMS输出:  → RCV_LENGTH → TMS_OUT
    ├─ GPIO设置: → NO_OP_1 → NO_OP_2
    ├─ 只读:     → RCV_LENGTH_L/H → READ_BYTE_LSB/MSB 或 READ_BIT_LSB/MSB
    ├─ 空时钟:   → CLOCK_BITS 或 RCV_LENGTH_L/H → CLOCK_BYTES
    └─ 错误:     → ERROR（回复 0xFA + 错误命令字节）
```

#### 缓冲区设计

- **RX 缓冲区**（512 字节线性 buffer）：存储从 USB 收到的 MPSSE 命令数据。用 `jtag_rx_pos` / `jtag_rx_len` 管理消费进度
- **TX 环形缓冲区**（2KB `chry_ringbuffer`）：存储需要回传给主机的 TDO 数据。用环形缓冲区是因为产生和消费可能不同步

为什么 RX 用线性 buffer 而不是环形？因为 MPSSE 命令需要按位置随机访问（`jtag_rx_buffer[jtag_rx_pos]`），而且每次 USB 包都是完整处理完才接收下一包。

#### 字节传输核心逻辑（以 LSB first 为例）

```c
case MPSSE_TRANSMIT_BYTE:
    data = jtag_rx_buffer[jtag_rx_pos];  // 取一字节数据
    usb_tx_data = 0;

    for (uint32_t i = 8; i; i--) {
        FPGA_TCK_LOW();                  // TCK 下降沿

        if (data & 0x01)                 // 设置 TDI
            FPGA_TDI_HIGH();
        else
            FPGA_TDI_LOW();

        data >>= 1;                      // LSB first: 右移
        usb_tx_data >>= 1;

        FPGA_TCK_HIGH();                 // TCK 上升沿（采样点）

        if (FPGA_TDO_READ())             // 读取 TDO
            usb_tx_data |= 0x80;
    }

    FPGA_TCK_LOW();                      // 最终拉低 TCK

    if (jtag_cmd == 0x39 || jtag_cmd == 0x3d)  // 读写命令才回传
        jtag_write(usb_tx_data);

    if (mpsse_longlen == 0)              // 长度递减到0，回到 IDLE
        mpsse_status = MPSSE_IDLE;
    mpsse_longlen--;
    jtag_rx_pos++;                       // 消费一字节数据
```

关键点：
- **长度是 `长度-1` 编码**：MPSSE 协议中 length=0 表示传输 1 字节，所以实际传输 `longlen+1` 字节
- **时钟边沿**：TDI 在下降沿设置，TDO 在上升沿采样，这是标准 JTAG 时序
- **只写命令不回传**：`0x19` 只写不读，不调用 `jtag_write()`；`0x39` 同时读写，才写入 TX 缓冲区

#### 0x80 Set Data Bits Low：GPIO 直接控制

```c
case MPSSE_NO_OP_1:
    if (jtag_cmd == 0x80) {
        uint8_t val = jtag_rx_buffer[jtag_rx_pos];
        if (val & 0x01) FPGA_TCK_HIGH(); else FPGA_TCK_LOW();  // Bit 0 = TCK
        if (val & 0x02) FPGA_TDI_HIGH(); else FPGA_TDI_LOW();  // Bit 1 = TDI
        if (val & 0x08) FPGA_TMS_HIGH(); else FPGA_TMS_LOW();  // Bit 3 = TMS
    }
```

Gowin Programmer 在发送 MPSSE 传输命令之前，会先用 `0x80` 设置引脚初始状态。如果这个命令不实际操作 GPIO，TAP 状态机无法正确导航，IDCODE 读取会失败。

引脚映射遵循 FT2232H ADBUS 定义：Bit0=TCK, Bit1=TDI, Bit2=TDO(输入), Bit3=TMS。

#### 0x81 Read Data Bits Low：读取引脚状态

```c
case 0x81:
    usb_tx_data = 0;
    if (FPGA_TDO_READ()) usb_tx_data |= 0x04;  // TDO 在 Bit 2
    jtag_write(usb_tx_data);
```

返回 TDO 的真实电平状态，放在 bit 2（对应 ADBUS2/TDO）。Programmer 用这个命令确认 FPGA 是否在线。

#### 跨包命令处理

MPSSE 命令可能跨越 USB 包边界。例如一个传输 300 字节的命令，参数和数据可能分布在多个 64 字节的 USB 包中。

状态机通过 `jtag_received_flag` 实现跨包处理：
- 每次 USB 回调收到数据，调用 `fpga_mpsse_feed()` 将数据复制到 RX buffer 并置位标志
- `fpga_mpsse_process()` 消费数据，当 `jtag_rx_pos >= jtag_rx_len` 时清除标志
- 下次调用时如果没有新数据且状态不是 IDLE，对于**只读/空时钟状态**（不需要消费数据的状态）仍然可以继续执行

#### 空时钟命令（0x8E / 0x8F）

Gowin Programmer 在编程结束后会发送大量空时钟来完成配置：

```c
case MPSSE_CLOCK_BITS:   // 0x8E: 空转 (shortlen+1) 个 TCK 周期
    do {
        FPGA_TCK_LOW();
        FPGA_TCK_HIGH();
    } while ((mpsse_shortlen--) > 0);

case MPSSE_CLOCK_BYTES:  // 0x8F: 空转 8*(longlen+1) 个 TCK 周期
    for (uint32_t i = 8; i; i--) {
        FPGA_TCK_LOW();
        FPGA_TCK_HIGH();
    }
```

这些命令不传输数据，只产生时钟——FPGA 在接收完比特流后，需要额外的时钟周期来完成内部配置过程。

---

### 第三层：FTDI Vendor Request 处理（usbd_ftdi.c）

Windows 上的 D2XX 驱动通过 USB 控制传输（EP0 vendor request）来配置 FT2232H。我们需要对每个请求给出正确响应，否则驱动会报错。

#### EEPROM 模拟

```c
const uint16_t ftdi_eeprom_info[] = {
    0x0800, 0x0403, 0x6010, 0x0500, ...
};
```

D2XX 驱动在枚举时会读取 FTDI EEPROM（`SIO_READ_EEPROM_REQUEST`，bRequest=0x90），确认芯片型号。这个数组模拟了 FT2232H 的 EEPROM 内容，包含 VID/PID、芯片类型、字符串描述符偏移等信息。如果不提供正确的 EEPROM 数据，D2XX 会拒绝设备。

#### 关键 Vendor Request

| bRequest | 名称 | 说明 |
|-|-|-|
| 0x00 | SIO_RESET | 复位设备 / 清空缓冲区 |
| 0x03 | SET_BAUDRATE | 设置波特率（Channel B UART 用） |
| 0x09 | SET_LATENCY_TIMER | 设置延迟定时器（控制心跳频率） |
| 0x0A | GET_LATENCY_TIMER | 读取延迟定时器 |
| 0x0B | SET_BITMODE | 设置工作模式（关键：mode=0x02 为 MPSSE） |
| 0x90 | READ_EEPROM | 读取 EEPROM 数据 |

#### SIO_RESET 的三种操作

```
wValue=0: 完全复位 — 重置延迟定时器、重初始化 MPSSE、重置 TX 状态、丢弃 RX 数据
wValue=1: 清空读缓冲（device→host） — 清空 MPSSE TX 环形缓冲区
wValue=2: 清空写缓冲（host→device） — 丢弃待处理的 RX 数据
```

这里有个重要的细节：`fpga_reset_tx_state()` 只重置 `fpga_tx_idle` 标志位，**不能**清除 `fpga_rx_ready`。因为 OUT 端点回调在设置 `fpga_rx_ready=true` 时已经停止了 OUT EP 的读取，如果我们清除了 `fpga_rx_ready` 但不重新启动 OUT EP 读取，端点会永久死锁。所以完全复位时单独用 `fpga_discard_rx()` 来安全地丢弃数据并重新 arm OUT EP。

#### SET_BITMODE 与动态通道路由

```c
case SIO_SET_BITMODE_REQUEST:
    if (((setup->wValue >> 8) & 0xFF) == 0x02) {
        mpsse_port = port;   // 记录哪个端口进入 MPSSE 模式
    }
```

这是一个关键的发现：**D2XX 驱动不一定把 MPSSE 分配给 Channel A**。根据 EEPROM 配置和驱动策略，D2XX 可能在 Channel A（port=1）或 Channel B（port=2）上启用 MPSSE。

`mpsse_port` 变量记录了当前哪个端口处于 MPSSE 模式。后续所有 USB 端点回调和数据处理都根据这个值动态路由——如果 MPSSE 在 Channel B，那么 Channel B 的 OUT 回调要把数据送给 MPSSE 引擎，Channel B 的 IN 端点要发送 MPSSE 响应，而 Channel A 变成空闲的。

#### 延迟定时器（Latency Timer）

D2XX 用延迟定时器控制 IN 端点的"心跳"频率。当没有数据要回传时，设备仍需按延迟定时器间隔发送 **modem status 包**（2字节：`{0x01, 0x60}`），告诉驱动"我还活着"。如果不发心跳，D2XX 会认为设备无响应。

#### 未知请求的处理

```c
default:
    /* Accept all unknown vendor requests to avoid stalling EP0 */
    break;
```

D2XX 驱动会发送一些非标准的 vendor request。如果我们对不认识的请求返回 STALL，Windows 设备管理器会显示 Code 10 错误。所以对所有未知请求都返回成功（空响应）。

---

### 第四层：USB 设备与数据流调度（fpga_jtag_main.c）

这一层把 USB 描述符、端点回调、MPSSE 引擎和 UART 桥接整合在一起。

#### USB 描述符设计

模拟的 FT2232H 使用以下 USB 布局：

```
Device Descriptor
├── VID: 0x0403 (FTDI)
├── PID: 0x6010 (FT2232H)
├── bcdDevice: 0x0500 (FT2232H 版本号)
└── bcdUSB: 2.0

Configuration Descriptor
├── Interface 0 (Channel A): Vendor Class (0xFF/0xFF/0xFF)
│   ├── EP 0x81 IN  (Bulk, 64B)  ← JTAG 响应
│   └── EP 0x02 OUT (Bulk, 64B)  ← JTAG 命令
│
└── Interface 1 (Channel B): Vendor Class (0xFF/0xFF/0xFF)
    ├── EP 0x83 IN  (Bulk, 64B)  ← UART RX / MPSSE 响应
    └── EP 0x04 OUT (Bulk, 64B)  ← UART TX / MPSSE 命令
```

注意端点编号的规则：**IN 端点用奇数（0x81, 0x83），OUT 端点用偶数（0x02, 0x04）**。这不是随意选的——D2XX 驱动期望 FT2232H 的端点布局完全匹配，否则无法正确识别通道。

同时注意描述符中**每个接口的端点顺序是 IN 在前、OUT 在后**。这也是 FT2232H 的原始布局，D2XX 期望这个顺序。

#### CherryUSB 高级描述符模式

由于 FT2232 模式和 DAP 模式使用完全不同的 USB 描述符（不同的 VID/PID/接口布局），不能用 CherryUSB 的静态描述符。使用 `CONFIG_USBDEV_ADVANCE_DESC` 模式，通过回调函数动态返回描述符：

```c
static const struct usb_descriptor fpga_usb_descriptor = {
    .device_descriptor_callback = fpga_device_descriptor_cb,
    .config_descriptor_callback = fpga_config_descriptor_cb,
    ...
};
```

模式切换时先 `usbd_deinitialize()` 销毁旧设备，再用新描述符 `usbd_initialize()` 重新枚举。

#### 端点回调与动态路由

数据流的核心调度逻辑在 OUT 端点回调中：

```c
// Channel A OUT 回调
void fpga_jtag_out_callback(...) {
    if (ftdi_get_mpsse_port() == 2) {
        // MPSSE 在 Channel B，Channel A OUT 数据无用，直接重新 arm
        usbd_ep_start_read(0, FPGA_JTAG_OUT_EP, ...);
        return;
    }
    // MPSSE 在 Channel A，接收数据
    fpga_rx_len = nbytes;
    fpga_rx_ready = true;
}

// Channel B OUT 回调
void ftdi_chb_out_callback(...) {
    if (ftdi_get_mpsse_port() == 2) {
        // MPSSE 在 Channel B，将数据送入 MPSSE 引擎
        memcpy(fpga_ep_rx_buf, ftdi_chb_rx_buf, nbytes);
        fpga_rx_ready = true;
        return;
    }
    // 普通 UART 模式，数据送入 UART TX 环形缓冲区
    chry_ringbuffer_write(&g_usbrx, ftdi_chb_rx_buf, nbytes);
}
```

这种设计的关键洞察：**无论 D2XX 选择哪个通道做 MPSSE，我们都能正确处理**。

#### 主处理循环（fpga_jtag_process）

```c
void fpga_jtag_process(void) {
    // 1. 根据 mpsse_port 选择正确的端点
    uint8_t in_ep  = (mp == 2) ? FTDI_CHB_IN_EP  : FPGA_JTAG_IN_EP;
    uint8_t out_ep = (mp == 2) ? FTDI_CHB_OUT_EP  : FPGA_JTAG_OUT_EP;

    // 2. 如果有新 USB 数据，喂给 MPSSE 引擎并处理
    if (fpga_rx_ready) {
        fpga_mpsse_feed(fpga_ep_rx_buf, fpga_rx_len);
        fpga_rx_ready = false;
        while (fpga_mpsse_is_busy()) fpga_mpsse_process();
        usbd_ep_start_read(0, out_ep, ...);  // 重新 arm OUT EP
    }

    // 3. 继续处理不需要新数据的只读操作
    while (fpga_mpsse_is_busy()) fpga_mpsse_process();

    // 4. 发送响应数据（带 2 字节 modem status 头）
    if (fpga_tx_idle) {
        uint32_t tx_len = fpga_mpsse_read_tx(fpga_tx_buffer + 2, ...);
        if (tx_len > 0) {
            fpga_tx_buffer[0] = 0x01;  // modem status
            fpga_tx_buffer[1] = 0x60;  // line status
            usbd_ep_start_write(0, in_ep, fpga_tx_buffer, tx_len + 2);
        } else if (latency timer expired) {
            // 无数据时发送心跳
            usbd_ep_start_write(0, in_ep, {0x01, 0x60}, 2);
        }
    }
}
```

注意 **每个 IN 包都带 2 字节 modem status 头**（`0x01, 0x60`）。这是 FTDI 协议的强制要求——D2XX 驱动会剥离这两字节，如果不带头部，驱动会把数据解析错误。

#### Clear Halt 恢复

D2XX 驱动在某些情况下（例如流控制或错误恢复）会发送 Clear Halt 请求来重置端点。CherryUSB 内部会清除 STALL 状态和重置 toggle bit，但**不会取消正在进行的 DMA 传输**。

为了处理可能的传输丢失，我们：

1. 在 CherryUSB 的 `usbd_core.c` 中添加了 weak 回调 `usbd_event_clear_halt_handler()`
2. 在回调中重置对应端点的 `tx_idle` 标志
3. 在主循环中加了 **2ms 超时恢复**：如果 IN 传输已经发起但超过 2ms 没有完成回调，视为丢失，重新标记 idle

```c
if (!fpga_tx_idle && (millis() - cha_tx_start_time) > 2) {
    fpga_tx_idle = true;  // 超时恢复
}
```

#### UART 桥接（fpga_uart_handle）

Channel B 在非 MPSSE 模式下作为 UART 桥接：

- **USB → UART**：Channel B OUT 数据 → `g_usbrx` 环形缓冲区 → DMA 发送到 UART TX
- **UART → USB**：UART RX → `g_uartrx` 环形缓冲区 → 加 modem status 头 → Channel B IN 发送

FTDI 的波特率设置（SIO_SET_BAUDRATE）使用特殊的分频器编码：

```c
baudrate = 48MHz / divisor    // divisor 包含整数部分和 3-bit 小数部分
```

当 Channel B 的波特率或数据格式被修改时，通过 `ftdi_uart_config_poll()` 通知 UART 外设重新配置。

#### 自检功能（fpga_jtag_self_test）

切换到 FPGA 模式时，在 MPSSE 引擎启动之前，直接用 GPIO bit-bang 读取 FPGA 的 JTAG IDCODE：

```
TLR（5个TCK+TMS=1）→ RTI → Select-DR → Capture-DR → Shift-DR → 读32位 → Exit
```

这绕过了 MPSSE，直接验证硬件连接。GW1NR-9C 的 IDCODE 是 `0x1100581B`。如果读到全 1 说明 TDO 没有下拉（FPGA 不在），全 0 说明短路。

---

### 第五层：模式切换（mode_switch.c）

#### 按键消抖

PA30 配置为输入，下拉电阻，按下为高电平。消抖逻辑：

```
按键按下 → 记录变化时间 → 等待 50ms → 确认稳定 → 触发切换
按键释放 → 清除标志，等待下一次按下
```

只在稳定后的**上升沿**触发一次，避免重复触发。

#### main.cpp 中的模式切换

```cpp
while (true) {
    mode_switch_poll();

    if (mode_switch_changed()) {
        if (mode_switch_get() == MODE_FPGA_JTAG) {
            chry_dap_deinit(0);           // 关闭 DAP USB 设备
            fpga_jtag_init();             // 初始化 GPIO + MPSSE + 自检
            fpga_usb_init(0, HPM_USB0_BASE);  // 启动 FT2232 USB 设备
        } else {
            fpga_usb_deinit(0);           // 关闭 FT2232 USB 设备
            chry_dap_init(0, HPM_USB0_BASE);  // 启动 DAP USB 设备
        }
    }

    if (mode == MODE_DAP) {
        chry_dap_handle();
        chry_dap_usb2uart_handle();
    } else {
        fpga_jtag_process();    // MPSSE 数据处理
        fpga_uart_handle();     // Channel B UART 桥接
    }
}
```

切换的本质是 **USB 重新枚举**：先完全拆除旧设备（`deinit`），再用新的描述符注册并初始化新设备（`init`）。从主机角度看，就像拔掉了一个设备、插上了另一个。

LED 指示：绿灯 = DAP 模式，蓝灯 = FPGA JTAG 模式。

---

### 调试过程中的关键问题与解决

| 问题 | 现象 | 根因 | 解决方案 |
|-|-|-|-|
| Windows Code 10 | 设备管理器报错 | EP0 对未知 vendor request 返回 STALL | 改为对所有未知请求返回成功 |
| 设备管理器挂起 | 打开设备属性卡死 | IN EP 没有按延迟定时器发送心跳 | 加入 latency timer 心跳机制 |
| SRAM 下载卡在 0% | Programmer 没有进度 | D2XX 将 MPSSE 分配给 Channel B，代码只处理 Channel A | 动态通道路由（`mpsse_port`） |
| "Device not found" | 读 IDCODE 失败 | `0x80` 命令没有实际操作 GPIO | 让 `0x80` 真正设置 TCK/TDI/TMS |
| OUT EP 死锁 | 设备不再接收数据 | SIO_RESET 时清除 `fpga_rx_ready` 但未重新 arm OUT EP | 分离 `fpga_reset_tx_state` 和 `fpga_discard_rx` |

---

### 编译开关

通过 CMake 选项 `CONFIG_CHERRYDAP_USE_FPGA_JTAG=1` 启用 FPGA JTAG 功能。关闭时所有 FPGA 相关代码不会被编译，固件行为与原始 CherryDAP 完全一致。

