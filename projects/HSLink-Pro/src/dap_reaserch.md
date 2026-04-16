# CherryDAP 中 CMSIS-DAP 的实现详解

本文档解析 CherryDAP 工程中 CMSIS-DAP 调试探针的完整实现，从协议规范到代码逻辑，帮助你理解一个调试器是如何工作的。

---

## 目录

1. [CMSIS-DAP 是什么](#1-cmsis-dap-是什么)
2. [整体架构](#2-整体架构)
3. [USB 传输层：dap_main.c](#3-usb-传输层dap_mainc)
4. [命令处理核心：DAP.c](#4-命令处理核心dapc)
5. [SWD 物理层：SW_DP.c](#5-swd-物理层sw_dpc)
6. [JTAG 物理层：JTAG_DP.c](#6-jtag-物理层jtag_dpc)
7. [硬件适配层：DAP_config.h](#7-硬件适配层dap_configh)
8. [HSLink-Pro 的特有优化](#8-hslink-pro-的特有优化)
9. [一次完整调试会话的数据流](#9-一次完整调试会话的数据流)

---

## 1. CMSIS-DAP 是什么

CMSIS-DAP 是 ARM 定义的一个**调试探针固件标准**。它规定了：

- 调试器（如 OpenOCD、pyOCD、Keil）通过 USB 发送什么格式的命令
- 探针固件如何解析这些命令、操作 SWD/JTAG 引脚
- 探针固件如何把结果返回给调试器

本质上，CMSIS-DAP 是一个 **USB 到 SWD/JTAG 的协议转换器**：

```
调试器软件 (OpenOCD/Keil)
    ↓ USB Bulk 传输
CMSIS-DAP 固件 (HPM5301)
    ↓ GPIO bit-bang
目标 MCU 的 Debug Access Port (DAP)
    ↓ ARM Debug Interface (ADI)
目标 MCU 内核 (读写寄存器/内存/Flash)
```

### v1 vs v2

- **CMSIS-DAP v1**：使用 USB HID 接口，兼容性好但速度慢（HID 轮询间隔限制）
- **CMSIS-DAP v2**：使用 **WinUSB Bulk** 接口，速度快很多（CherryDAP 使用此版本）

CherryDAP 的固件版本标识为 `"2.1.1"`。

---

## 2. 整体架构

CherryDAP 的分层结构：

```
┌─────────────────────────────────────────────────┐
│  dap_main.c — USB 传输层                         │
│  USB 描述符、端点回调、请求/响应缓冲区管理          │
│  CDC UART 桥接、WinUSB/WebUSB/BOS 描述符          │
├─────────────────────────────────────────────────┤
│  DAP.c — CMSIS-DAP 命令处理核心                   │
│  命令分派、DAP_Transfer、DAP_Info                  │
│  队列命令（QueueCommands/ExecuteCommands）         │
├───────────────────┬─────────────────────────────┤
│  SW_DP.c          │  JTAG_DP.c                   │
│  SWD 协议时序      │  JTAG 协议时序                │
│  SWD_Transfer()   │  JTAG_Transfer()             │
│  SWJ_Sequence()   │  JTAG_IR() / JTAG_Sequence() │
├───────────────────┴─────────────────────────────┤
│  DAP_config.h — 硬件抽象层                        │
│  PIN_SWCLK_TCK_SET/CLR, PIN_SWDIO_TMS_SET/CLR   │
│  PORT_SWD_SETUP(), PORT_JTAG_SETUP()             │
│  时钟配置、包大小、能力声明                         │
└─────────────────────────────────────────────────┘
```

### 文件清单

| 文件 | 位置 | 职责 |
|-|-|-|
| `dap_main.h/c` | 工程根目录 | USB 设备初始化、端点回调、DAP 命令调度循环、CDC UART |
| `DAP/Include/DAP.h` | DAP 库 | 协议常量定义（命令 ID、状态码、寄存器地址）、`DAP_Data_t` 结构体 |
| `DAP/Source/DAP.c` | DAP 库 | 命令解析与分派、Transfer 逻辑、Info/Connect/Clock 等 |
| `DAP/Source/SW_DP.c` | DAP 库 | SWD 物理层：bit-bang 时序、SWD_Transfer()、SWJ_Sequence() |
| `DAP/Source/JTAG_DP.c` | DAP 库 | JTAG 物理层：bit-bang 时序、JTAG_Transfer()、JTAG_IR() |
| `DAP/Config/DAP_config.h` | DAP 库 | 硬件抽象层模板（引脚操作、时钟参数、能力配置） |
| `projects/HSLink-Pro/src/DAP_config.h` | 项目级 | HPM5301 引脚实现覆盖（FGPIO 操作） |
| `projects/HSLink-Pro/src/SW_DP/` | 项目级 | SWD 端口初始化（IO 模式 / SPI 硬件加速模式） |
| `projects/HSLink-Pro/src/JTAG_DP/` | 项目级 | JTAG 端口初始化（IO 模式 / SPI 硬件加速模式） |

---

## 3. USB 传输层：dap_main.c

### 3.1 USB 设备描述符

CherryDAP 枚举为一个 **USB 2.1 复合设备**（VID=0x0D28, PID=0x0204）：

```
Device Descriptor (USB 2.1, Class 0xEF/0x02/0x01 = IAD)
│
└── Configuration Descriptor
    ├── Interface 0: CMSIS-DAP v2 (Vendor Class 0xFF)
    │   ├── EP 0x81 IN  (Bulk) ← DAP 响应
    │   └── EP 0x02 OUT (Bulk) ← DAP 命令
    │
    ├── Interface 1+2: CDC ACM (串口桥接)
    │   ├── EP 0x85 IN  (Interrupt) ← CDC 通知
    │   ├── EP 0x83 IN  (Bulk) ← UART RX 数据
    │   └── EP 0x04 OUT (Bulk) ← UART TX 数据
    │
    ├── Interface 3: Custom HID (可选, CONFIG_CHERRYDAP_USE_CUSTOM_HID)
    │   ├── EP 0x88 IN  (Interrupt)
    │   └── EP 0x09 OUT (Interrupt)
    │
    ├── Interface 4: MSC (可选, CONFIG_CHERRYDAP_USE_MSC)
    │
    └── Interface 5: WebUSB (可选)
```

关键设计选择：

- **Interface 0 使用 Vendor Class (0xFF)**，而不是 HID——这是 CMSIS-DAP v2 的核心区别。WinUSB Bulk 传输没有 HID 的轮询间隔限制，吞吐量高得多
- **WinUSB 自动安装**：通过 BOS 描述符中的 MS OS 2.0 兼容 ID 描述符，Windows 自动加载 WinUSB 驱动，无需 .inf 文件
- **WebUSB**：支持浏览器直接访问调试器（用于 Web 调试工具）

### 3.2 请求/响应缓冲区管理

这是 CherryDAP 性能优化的关键——**多缓冲流水线**：

```c
#define DAP_PACKET_COUNT  8   // 8 个包的缓冲深度
#define DAP_PACKET_SIZE   512 // 每包 512 字节（USB HS）

static uint8_t USB_Request [DAP_PACKET_COUNT][DAP_PACKET_SIZE];  // 请求环形缓冲
static uint8_t USB_Response[DAP_PACKET_COUNT][DAP_PACKET_SIZE];  // 响应环形缓冲
```

用生产者-消费者模型管理：

```
USB OUT 中断（生产者）              主循环（消费者）
    │                                 │
    ▼                                 ▼
USB_Request[IndexI] ← 新包      USB_Request[IndexO] → DAP_ExecuteCommand()
USB_RequestIndexI++                                     │
USB_RequestCountI++              USB_Response[IndexI] ← 结果
    │                            USB_ResponseIndexI++
    │                                 │
    │                            USB IN 中断 ← USB_Response[IndexO]
    ▼                            USB_ResponseIndexO++
如果缓冲区满 → USB_RequestIdle=1（停止接收）
缓冲区有空位 → 重新 arm OUT EP
```

#### 为什么用 8 个缓冲区？

调试器（如 OpenOCD）通常会**一次发送多个 DAP 命令**，不等上一个完成就发下一个。8 个缓冲区允许 USB 接收和命令处理同时进行——当 CPU 在处理第 N 个命令时，USB 硬件可以在后台接收第 N+1 到 N+7 个命令。这就是为什么 CMSIS-DAP v2 比 v1 快得多。

### 3.3 端点回调

#### OUT 回调（收到主机命令）

```c
void dap_out_callback(uint8_t busid, uint8_t ep, uint32_t nbytes) {
    if (USB_Request[USB_RequestIndexI][0] == ID_DAP_TransferAbort) {
        DAP_TransferAbort = 1U;  // 特殊：中止命令直接设标志，不入队
    } else {
        USB_RequestIndexI++;     // 入队
        USB_RequestCountI++;
    }
    // 如果缓冲区还有空间，继续接收下一包
    if (count != DAP_PACKET_COUNT) {
        usbd_ep_start_read(0, DAP_OUT_EP, USB_Request[USB_RequestIndexI], DAP_PACKET_SIZE);
    } else {
        USB_RequestIdle = 1U;    // 缓冲区满，暂停接收
    }
}
```

`DAP_TransferAbort` 是一个特殊机制：中止命令不需要排队等待前面的命令执行完，而是直接设置一个全局标志，正在执行的 Transfer 命令会检查这个标志并提前退出。

#### IN 回调（数据发送完成）

```c
void dap_in_callback(uint8_t busid, uint8_t ep, uint32_t nbytes) {
    if (USB_ResponseCountI != USB_ResponseCountO) {
        // 还有响应待发送，立即启动下一次 IN 传输
        usbd_ep_start_write(0, DAP_IN_EP, USB_Response[USB_ResponseIndexO], ...);
        USB_ResponseIndexO++;
        USB_ResponseCountO++;
    } else {
        USB_ResponseIdle = 1U;   // 无更多响应，等待新数据
    }
}
```

### 3.4 主循环调度（chry_dap_handle）

```c
void chry_dap_handle(void) {
    while (USB_RequestCountI != USB_RequestCountO) {
        // 1. 处理 QueueCommands：将多个待执行命令标记为 ExecuteCommands
        n = USB_RequestIndexO;
        while (USB_Request[n][0] == ID_DAP_QueueCommands) {
            USB_Request[n][0] = ID_DAP_ExecuteCommands;
            n++;
        }

        // 2. 执行 DAP 命令
        USB_RespSize[USB_ResponseIndexI] =
            DAP_ExecuteCommand(USB_Request[USB_RequestIndexO],
                               USB_Response[USB_ResponseIndexI]);

        // 3. 更新请求索引
        USB_RequestIndexO++;
        USB_RequestCountO++;

        // 4. 如果 OUT EP 空闲，重新 arm
        if (USB_RequestIdle && count != DAP_PACKET_COUNT) {
            USB_RequestIdle = 0U;
            usbd_ep_start_read(0, DAP_OUT_EP, ...);
        }

        // 5. 更新响应索引，启动 IN 传输
        USB_ResponseIndexI++;
        USB_ResponseCountI++;
        if (USB_ResponseIdle && has_pending_response) {
            USB_ResponseIdle = 0U;
            usbd_ep_start_write(0, DAP_IN_EP, ...);
        }
    }
}
```

#### QueueCommands 机制

`ID_DAP_QueueCommands (0x7E)` 是 CMSIS-DAP v2 的原子命令批处理：

1. 调试器发送多个 `QueueCommands` 包（格式与普通命令相同，但命令 ID 不同）
2. 最后发送一个 `ExecuteCommands (0x7F)` 包
3. 固件在收到 `ExecuteCommands` 后，一次性处理所有排队的命令

在 CherryDAP 的实现中，主循环检测到排队的 `QueueCommands`，将它们的命令 ID 改写为 `ExecuteCommands`，然后依次执行。`DAP_ExecuteCommand()` 检测到 `ExecuteCommands` 后会解析包内的多个子命令并逐一处理。

### 3.5 CDC UART 桥接

除了 DAP 功能，固件还实现了 **USB CDC ACM 虚拟串口 → 物理 UART** 的桥接：

```
USB CDC OUT → g_usbrx 环形缓冲区 → DMA 发送到 UART TX
UART RX    → g_uartrx 环形缓冲区 → USB CDC IN
```

使用 `chry_ringbuffer` 的 `linear_read_setup/done` 接口实现零拷贝 DMA 传输——直接把环形缓冲区中连续的线性区域指针传给 DMA，避免额外的 memcpy。

---

## 4. 命令处理核心：DAP.c

`DAP.c` 是 ARM 官方 CMSIS-DAP 参考实现代码，CherryDAP 直接使用。

### 4.1 DAP_ProcessCommand：命令分派

```c
uint32_t DAP_ProcessCommand(const uint8_t *request, uint8_t *response) {
    // Vendor 命令（0x80-0x9F）走自定义处理
    if (*request >= ID_DAP_Vendor0 && *request <= ID_DAP_Vendor31)
        return DAP_ProcessVendorCommand(request, response);

    *response++ = *request;  // 响应的第一字节 = 命令 ID（echo back）

    switch (*request++) {
        case ID_DAP_Info:              // 0x00 - 查询设备信息
        case ID_DAP_HostStatus:        // 0x01 - 设置 LED 状态
        case ID_DAP_Connect:           // 0x02 - 连接（选择 SWD/JTAG）
        case ID_DAP_Disconnect:        // 0x03 - 断开
        case ID_DAP_TransferConfigure: // 0x04 - 配置传输参数
        case ID_DAP_Transfer:          // 0x05 - 读写寄存器（核心）
        case ID_DAP_TransferBlock:     // 0x06 - 批量读写
        case ID_DAP_SWJ_Pins:          // 0x10 - 直接控制引脚
        case ID_DAP_SWJ_Clock:         // 0x11 - 设置时钟频率
        case ID_DAP_SWJ_Sequence:      // 0x12 - 发送 SWJ 序列
        case ID_DAP_SWD_Configure:     // 0x13 - SWD 配置
        case ID_DAP_JTAG_Sequence:     // 0x14 - JTAG 序列
        case ID_DAP_JTAG_Configure:    // 0x15 - JTAG 扫描链配置
        case ID_DAP_JTAG_IDCODE:       // 0x16 - 读 JTAG IDCODE
        // ... SWO、UART 等命令
    }
}
```

返回值设计巧妙：**高 16 位 = 消耗的请求字节数，低 16 位 = 产生的响应字节数**。这让 `DAP_ExecuteCommand` 能在单个 USB 包中处理多个命令。

### 4.2 DAP_Info：设备信息查询

调试器连接时首先发送 `DAP_Info` 查询探针能力：

```c
case DAP_ID_CAPABILITIES:
    info[0] = ((DAP_SWD  != 0)  ? (1U << 0) : 0U) |   // 支持 SWD
              ((DAP_JTAG != 0)  ? (1U << 1) : 0U) |   // 支持 JTAG
              ((SWO_UART != 0)  ? (1U << 2) : 0U) |   // 支持 SWO UART
              (1U << 4) |                               // 支持原子命令
              ((TIMESTAMP_CLOCK != 0U) ? (1U << 5) : 0U) | // 支持时间戳
              ((DAP_UART != 0U) ? (1U << 7) : 0U);    // 支持 UART
```

HSLink-Pro 的能力：SWD + JTAG + SWO UART + 原子命令 + 时间戳 + UART。

其他重要查询：
- `DAP_ID_PACKET_SIZE` → 512（USB HS bulk 包大小）
- `DAP_ID_PACKET_COUNT` → 8（缓冲深度）
- `DAP_ID_DAP_FW_VER` → "2.1.1"

### 4.3 DAP_Connect：选择调试端口

```c
static uint32_t DAP_Connect(const uint8_t *request, uint8_t *response) {
    if (*request == DAP_PORT_AUTODETECT)
        port = DAP_DEFAULT_PORT;  // 默认 SWD
    else
        port = *request;

    switch (port) {
        case DAP_PORT_SWD:
            DAP_Data.debug_port = DAP_PORT_SWD;
            PORT_SWD_SETUP();      // 初始化 SWD 引脚
            break;
        case DAP_PORT_JTAG:
            DAP_Data.debug_port = DAP_PORT_JTAG;
            PORT_JTAG_SETUP();     // 初始化 JTAG 引脚
            break;
    }
}
```

`PORT_SWD_SETUP()` / `PORT_JTAG_SETUP()` 是硬件抽象函数，由 `DAP_config.h` 或平台代码实现。

### 4.4 DAP_SWJ_Clock：时钟速率设置

```c
void Set_Clock_Delay(uint32_t clock) {
    if (clock >= MAX_SWJ_CLOCK(DELAY_FAST_CYCLES)) {
        DAP_Data.fast_clock = 1U;     // 快速模式：不插入延迟
        DAP_Data.clock_delay = 1U;
    } else {
        DAP_Data.fast_clock = 0U;
        delay = (CPU_CLOCK / 2) / clock;   // 计算需要的延迟周期数
        delay -= IO_PORT_WRITE_CYCLES;
        delay /= DELAY_SLOW_CYCLES;
        DAP_Data.clock_delay = delay;
    }
}
```

公式推导：

$$f_{SWCLK} = \frac{f_{CPU} / 2}{IO\_CYCLES + delay \times DELAY\_CYCLES}$$

- `CPU_CLOCK = 100MHz`
- `IO_PORT_WRITE_CYCLES = 2`（GPIO 写入消耗的周期）
- `DELAY_SLOW_CYCLES = 3`（每次延迟循环消耗的周期）
- 快速模式最大频率：`100MHz / (2 × 2) = 25MHz`

### 4.5 DAP_Transfer：寄存器读写（最核心的命令）

这是调试器最频繁使用的命令，用于读写目标 MCU 的 Debug Port（DP）和 Access Port（AP）寄存器。

请求格式：
```
[0x05] [DAP Index] [Transfer Count] [Transfer Request 1] [Data 1...] [Request 2] ...
```

Transfer Request 字节的各位含义：

| Bit | 名称 | 含义 |
|-|-|-|
| 0 | APnDP | 0=DP 寄存器, 1=AP 寄存器 |
| 1 | RnW | 0=写, 1=读 |
| 2 | A2 | 寄存器地址 bit 2 |
| 3 | A3 | 寄存器地址 bit 3 |
| 4 | Match Value | 1=读到匹配值才返回 |
| 5 | Match Mask | 1=设置匹配掩码 |
| 7 | Timestamp | 1=记录时间戳 |

#### SWD Transfer 的实现流程

```c
static uint32_t DAP_SWD_Transfer(const uint8_t *request, uint8_t *response) {
    for (每个 transfer 请求) {
        if (读请求) {
            if (AP 寄存器) {
                // AP 读是 posted：本次读返回的是上次的数据
                // 需要额外一次 DP_RDBUFF 读来获取最后一次数据
                if (!post_read) {
                    SWD_Transfer(request_value, NULL);  // Post 读
                    post_read = 1;
                }
            } else {
                // DP 读：直接返回数据
                SWD_Transfer(request_value, &data);
                *response++ = data;  // 存入响应
            }
        } else {
            // 写请求
            data = *(request...);
            SWD_Transfer(request_value, &data);
        }
        // WAIT 响应 → 自动重试（retry_count 次）
    }
    // 最后处理 posted read 的尾巴
    if (post_read) {
        SWD_Transfer(DP_RDBUFF | RnW, &data);  // 取回最后一次 AP 读
    }
}
```

**posted read 机制**是 ARM Debug Interface 的一个重要特性：AP 寄存器读是延迟返回的。第一次读 AP 寄存器只是发出请求，真正的数据在下一次读操作时返回。代码通过 `post_read` 标志跟踪这个状态。

---

## 5. SWD 物理层：SW_DP.c

SWD（Serial Wire Debug）只用两根线：SWCLK（时钟）和 SWDIO（双向数据）。

### 5.1 基本时钟宏

```c
#define SW_CLOCK_CYCLE()        \
    PIN_SWCLK_CLR();            \   // SWCLK ↓
    PIN_DELAY();                \   // 等待半周期
    PIN_SWCLK_SET();            \   // SWCLK ↑
    PIN_DELAY()                     // 等待半周期

#define SW_WRITE_BIT(bit)       \
    PIN_SWDIO_OUT(bit);         \   // 设置 SWDIO
    PIN_SWCLK_CLR();            \   // SWCLK ↓
    PIN_DELAY();                \
    PIN_SWCLK_SET();            \   // SWCLK ↑（目标在上升沿采样）
    PIN_DELAY()

#define SW_READ_BIT(bit)        \
    PIN_SWCLK_CLR();            \   // SWCLK ↓（目标在下降沿改变 SWDIO）
    PIN_DELAY();                \
    bit = PIN_SWDIO_IN();       \   // 读取 SWDIO
    PIN_SWCLK_SET();            \   // SWCLK ↑
    PIN_DELAY()
```

### 5.2 SWD_Transfer：一次完整的 SWD 事务

SWD 协议的一次传输包含三个阶段：

```
 Host驱动SWDIO       目标驱动SWDIO      Host或目标驱动
┌──────────────┐  ┌───────────┐  ┌─────────────────┐
│ Packet Request│→│ACK Response│→│  Data Transfer   │
│ 8 bits       │  │ 3 bits    │  │ 32+1 bits        │
└──────────────┘  └───────────┘  └─────────────────┘
  Start(1)          ACK[0:2]       DATA[0:31] + Parity
  APnDP
  RnW
  A[2:3]
  Parity
  Stop(0)
  Park(1)
```

代码实现（用宏生成两个版本：Fast 和 Slow）：

```c
#define SWD_TransferFunction(speed)
static uint8_t SWD_Transfer##speed(uint32_t request, uint32_t *data) {
    // === 阶段1：发送 Packet Request（8 bit，Host 驱动 SWDIO）===
    SW_WRITE_BIT(1U);                     // Start Bit = 1
    SW_WRITE_BIT(request >> 0);           // APnDP
    SW_WRITE_BIT(request >> 1);           // RnW
    SW_WRITE_BIT(request >> 2);           // A2
    SW_WRITE_BIT(request >> 3);           // A3
    SW_WRITE_BIT(parity);                 // 奇偶校验
    SW_WRITE_BIT(0U);                     // Stop Bit = 0
    SW_WRITE_BIT(1U);                     // Park Bit = 1

    // === Turnaround：SWDIO 方向翻转（Host → Target）===
    PIN_SWDIO_OUT_DISABLE();              // SWDIO 切为输入
    for (n = turnaround; n; n--)
        SW_CLOCK_CYCLE();                 // 等待 turnaround 周期

    // === 阶段2：读取 ACK（3 bit，Target 驱动 SWDIO）===
    SW_READ_BIT(bit); ack  = bit << 0;   // ACK[0]
    SW_READ_BIT(bit); ack |= bit << 1;   // ACK[1]
    SW_READ_BIT(bit); ack |= bit << 2;   // ACK[2]

    if (ack == DAP_TRANSFER_OK) {
        if (request & DAP_TRANSFER_RnW) {
            // === 阶段3a：读数据（32+1 bit，Target 驱动 SWDIO）===
            for (n = 32; n; n--) {
                SW_READ_BIT(bit);         // 读 DATA[0:31]
                val = (val >> 1) | (bit << 31);
                parity += bit;
            }
            SW_READ_BIT(bit);             // 读 Parity
            if ((parity ^ bit) & 1U)
                ack = DAP_TRANSFER_ERROR; // 校验错误

            // Turnaround：切回输出
            PIN_SWDIO_OUT_ENABLE();
        } else {
            // === Turnaround：切回输出 ===
            PIN_SWDIO_OUT_ENABLE();

            // === 阶段3b：写数据（32+1 bit，Host 驱动 SWDIO）===
            for (n = 32; n; n--) {
                SW_WRITE_BIT(val);        // 写 DATA[0:31]
                val >>= 1;
                parity += val;
            }
            SW_WRITE_BIT(parity);         // 写 Parity
        }

        // Idle cycles（可配置，保持 SWDIO=0）
        for (n = idle_cycles; n; n--)
            SW_CLOCK_CYCLE();
    }

    if (ack == WAIT || ack == FAULT) {
        // 错误处理：dummy 时钟清空数据阶段
        ...
    }
}
```

#### 快速 vs 慢速版本

```c
#define PIN_DELAY() PIN_DELAY_FAST()
SWD_TransferFunction(Fast)     // 生成 SWD_TransferFast()

#define PIN_DELAY() PIN_DELAY_SLOW(DAP_Data.clock_delay)
SWD_TransferFunction(Slow)     // 生成 SWD_TransferSlow()

uint8_t SWD_Transfer(uint32_t request, uint32_t *data) {
    if (DAP_Data.fast_clock)
        return SWD_TransferFast(request, data);
    else
        return SWD_TransferSlow(request, data);
}
```

用 C 宏生成两份代码，快速版本完全省略延迟调用，编译器可以更好地优化。运行时根据 `fast_clock` 标志选择。

### 5.3 SWJ_Sequence：切换协议

SWD 和 JTAG 共享 SWCLK/TCK 和 SWDIO/TMS 两根引脚。目标 MCU 上电默认为 JTAG 模式，需要通过特定序列切换到 SWD：

```
发送 50+ 个 1（TMS 高）→ JTAG 进入 Test-Logic-Reset
发送 0x9E, 0xE7 的 16-bit 切换序列
发送 50+ 个 1（确认切换）
发送 2 个 0（idle）
```

`SWJ_Sequence()` 函数负责发送任意长度的位序列：

```c
void SWJ_Sequence(uint32_t count, const uint8_t *data) {
    while (count--) {
        if (val & 1U)
            PIN_SWDIO_TMS_SET();
        else
            PIN_SWDIO_TMS_CLR();
        SW_CLOCK_CYCLE();
        val >>= 1;
    }
}
```

---

## 6. JTAG 物理层：JTAG_DP.c

JTAG 使用四根线：TCK（时钟）、TMS（模式选择）、TDI（数据输入）、TDO（数据输出），加上可选的 nTRST。

### 6.1 JTAG TAP 状态机

JTAG 的核心是一个 **16 状态的 TAP（Test Access Port）状态机**，TMS 信号在 TCK 上升沿采样来驱动状态转移：

```
                    TMS=1
              ┌─────────────┐
              ▼             │
         Test-Logic-Reset ──┘
              │ TMS=0
              ▼
          Run-Test/Idle ◄───────────────┐
              │ TMS=1                   │
              ▼                         │
         Select-DR-Scan                 │
          │TMS=0    │TMS=1              │
          ▼         ▼                   │
     Capture-DR  Select-IR-Scan        │
          │         │TMS=0    │TMS=1    │
          ▼         ▼         │         │
      Shift-DR  Capture-IR   ...       │
          │         │                   │
          ▼         ▼                   │
      Exit1-DR  Shift-IR               │
          │         │                   │
          ▼         ▼                   │
     Update-DR  ...                    │
          │                             │
          └─────────────────────────────┘
```

代码中的关键操作：

- **导航到 Shift-DR**（用于数据传输）：`TMS=1,0,0` → Select-DR → Capture-DR → Shift-DR
- **导航到 Shift-IR**（用于选择指令）：`TMS=1,1,0,0` → Select-DR → Select-IR → Capture-IR → Shift-IR
- **退出**：`TMS=1` → Exit → Update → Idle

### 6.2 JTAG_IR：加载指令寄存器

```c
static void JTAG_IR_##speed(uint32_t ir) {
    PIN_TMS_SET();
    JTAG_CYCLE_TCK();          // Select-DR-Scan
    JTAG_CYCLE_TCK();          // Select-IR-Scan
    PIN_TMS_CLR();
    JTAG_CYCLE_TCK();          // Capture-IR
    JTAG_CYCLE_TCK();          // Shift-IR

    // 发送 bypass bits（扫描链中该设备之前的设备）
    PIN_TDI_OUT(1U);
    for (n = ir_before[index]; n; n--)
        JTAG_CYCLE_TCK();

    // 发送 IR 数据（除最后一位）
    for (n = ir_length[index] - 1; n; n--) {
        JTAG_CYCLE_TDI(ir);
        ir >>= 1;
    }

    // 发送 bypass bits（扫描链中该设备之后的设备）+ 最后一位 + Exit1-IR
    ...

    JTAG_CYCLE_TCK();          // Update-IR
    PIN_TMS_CLR();
    JTAG_CYCLE_TCK();          // Idle
}
```

#### 扫描链处理

JTAG 可以有多个设备级联在一条链上。`DAP_Data.jtag_dev` 记录了链上每个设备的 IR 长度和前后的 bypass 位数。`JTAG_IR()` 在目标设备的 IR 之前和之后发送 1（bypass），确保其他设备处于 bypass 模式。

### 6.3 JTAG_Transfer：数据寄存器读写

```c
static uint8_t JTAG_Transfer##speed(uint32_t request, uint32_t *data) {
    // 导航到 Shift-DR
    PIN_TMS_SET();
    JTAG_CYCLE_TCK();          // Select-DR-Scan
    PIN_TMS_CLR();
    JTAG_CYCLE_TCK();          // Capture-DR
    JTAG_CYCLE_TCK();          // Shift-DR

    // Bypass：跳过链上前面的设备
    for (n = index; n; n--)
        JTAG_CYCLE_TCK();

    // 发送 3 位请求（RnW, A2, A3），同时读取 3 位 ACK
    JTAG_CYCLE_TDIO(request >> 1, bit);  ack = bit << 1;   // RnW → ACK[0]
    JTAG_CYCLE_TDIO(request >> 2, bit);  ack |= bit << 0;  // A2  → ACK[1]
    JTAG_CYCLE_TDIO(request >> 3, bit);  ack |= bit << 2;  // A3  → ACK[2]

    if (ack == OK) {
        if (读) {
            for (n = 31; n; n--)
                JTAG_CYCLE_TDO(bit);   // 读 D0-D30
            // 最后一位 + Exit1-DR（带 TMS=1）
        } else {
            for (n = 31; n; n--)
                JTAG_CYCLE_TDI(val);   // 写 D0-D30
            // 最后一位 + Exit1-DR
        }
    }

    // Update-DR → Idle
    JTAG_CYCLE_TCK();
    PIN_TMS_CLR();
    JTAG_CYCLE_TCK();
}
```

注意 JTAG 和 SWD 的一个关键区别：**JTAG 是 TDI/TDO 同时传输的**——在发送请求位的同时就能读到 ACK 应答。而 SWD 的 SWDIO 是半双工的，需要 turnaround 周期切换方向。

---

## 7. 硬件适配层：DAP_config.h

`DAP_config.h` 是整个移植的关键——它定义了所有与硬件相关的参数和引脚操作。

### 7.1 全局参数

```c
#define CPU_CLOCK            100000000U   // HPM5301 CPU 时钟 100MHz
#define IO_PORT_WRITE_CYCLES 2U           // GPIO 写操作消耗的 CPU 周期
#define DAP_SWD              1            // 支持 SWD
#define DAP_JTAG             1            // 支持 JTAG
#define DAP_JTAG_DEV_CNT     8U           // 最大 JTAG 设备数
#define DAP_DEFAULT_PORT     1U           // 默认 SWD 模式
#define DAP_PACKET_SIZE      512U         // USB HS，每包 512 字节
#define DAP_PACKET_COUNT     8U           // 8 个缓冲包
#define SWO_UART             1            // 支持 SWO
#define TIMESTAMP_CLOCK      100000000U   // 时间戳时钟 = CPU 时钟
```

### 7.2 引脚定义（HSLink-Pro 项目级覆盖）

```c
#define PIN_GPIO    HPM_FGPIO      // 使用 Fast GPIO
#define PIN_TCK     IOC_PAD_PB11
#define PIN_TMS     IOC_PAD_PB10
#define PIN_TDI     IOC_PAD_PB13
#define PIN_TDO     IOC_PAD_PB12
#define PIN_TRST    IOC_PAD_PB14
#define PIN_SRST    IOC_PAD_PB15
```

### 7.3 引脚操作函数

每个引脚操作都是 `__STATIC_FORCEINLINE`（强制内联），确保编译后直接展开为寄存器操作：

```c
__STATIC_FORCEINLINE void PIN_SWCLK_TCK_SET(void) {
    gpio_write_pin(PIN_GPIO, GPIO_GET_PORT_INDEX(PIN_TCK),
                   GPIO_GET_PIN_INDEX(PIN_TCK), true);
    __asm volatile("fence io, io");   // RISC-V I/O 排序屏障
}

__STATIC_FORCEINLINE void PIN_SWCLK_TCK_CLR(void) {
    gpio_write_pin(PIN_GPIO, GPIO_GET_PORT_INDEX(PIN_TCK),
                   GPIO_GET_PIN_INDEX(PIN_TCK), false);
    __asm volatile("fence io, io");
}
```

SWDIO 方向切换需要特殊处理，因为 SWD 的 SWDIO 是双向的：

```c
__STATIC_FORCEINLINE void PIN_SWDIO_OUT_ENABLE(void) {
    gpio_set_pin_output(PIN_GPIO, ..., PIN_TMS);  // 切为输出
}

__STATIC_FORCEINLINE void PIN_SWDIO_OUT_DISABLE(void) {
    gpio_set_pin_input(PIN_GPIO, ..., PIN_TMS);   // 切为输入
}
```

---

## 8. HSLink-Pro 的特有优化

### 8.1 SPI 硬件加速模式

HSLink-Pro 不仅支持标准的 GPIO bit-bang，还实现了 **SPI 硬件加速模式**：

```c
void PORT_SWD_SETUP(void) {
    if (HSLink_Setting.swd_port_mode == PORT_MODE_SPI)
        SPI_PORT_SWD_SETUP();     // 用 SPI 外设驱动 SWCLK/SWDIO
    else
        IO_PORT_SWD_SETUP();      // 传统 GPIO bit-bang
}
```

SPI 模式下，利用 SPI 外设的移位寄存器来生成时钟和数据，比逐位 GPIO 操作快得多。TCK 连接到 SPI_CLK，TDI/SWDIO 连接到 SPI_MOSI/MISO。这是 HSLink-Pro 高速调试的关键。

### 8.2 FGPIO + IO Fence

HPM5301 的 FGPIO（Fast GPIO）通过 CPU 的快速 I/O 总线直接访问 GPIO 寄存器，比走 AHB 总线的普通 GPIO 快。配合 `fence io, io` 屏障确保操作顺序，在 100MHz CPU 时钟下可以实现很高的 bit-bang 频率。

### 8.3 USB High-Speed + 大包 + 深缓冲

- **USB HS (480Mbps)**：比 Full-Speed 快 40 倍
- **512 字节包**：一个 USB 事务可以传输大量 DAP 命令
- **8 包深缓冲**：USB 接收和命令处理全速流水线运行

这三者配合使得 HSLink-Pro 的调试速度远超普通 CMSIS-DAP 调试器。

---

## 9. 一次完整调试会话的数据流

以 OpenOCD 连接一个 Cortex-M MCU 为例：

```
=== 1. 设备发现 ===
OpenOCD → DAP_Info(CAPABILITIES)
       ← SWD=1, JTAG=1, Packet=512, Count=8

=== 2. 连接 ===
OpenOCD → DAP_Connect(SWD)
          固件调用 PORT_SWD_SETUP()，配置 SWCLK/SWDIO 为输出
       ← DAP_PORT_SWD

=== 3. 设置时钟 ===
OpenOCD → DAP_SWJ_Clock(10000000)   // 10MHz
          固件计算 clock_delay，选择 Fast 或 Slow 路径
       ← DAP_OK

=== 4. SWD 切换序列 ===
OpenOCD → DAP_SWJ_Sequence(51, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03})
          固件在 SWDIO 上发送 51 个 1
       ← DAP_OK
OpenOCD → DAP_SWJ_Sequence(16, {0x9E, 0xE7})
          JTAG→SWD 切换码
       ← DAP_OK
OpenOCD → DAP_SWJ_Sequence(51, {0xFF...})  // 再次 50+ 个 1
       ← DAP_OK

=== 5. 配置传输参数 ===
OpenOCD → DAP_TransferConfigure(idle=0, retry=100, match_retry=0)
       ← DAP_OK
OpenOCD → DAP_SWD_Configure(turnaround=1, data_phase=0)
       ← DAP_OK

=== 6. 读目标 IDCODE ===
OpenOCD → DAP_Transfer(1, [读 DP_IDCODE])
          固件执行 SWD_Transfer(DP_IDCODE | RnW, &data)：
            → SWDIO 发送: 1(Start) 0(DP) 1(Read) 0(A2=0) 0(A3=0) 1(Parity) 0(Stop) 1(Park)
            → Turnaround
            ← 3-bit ACK = OK
            ← 32-bit DATA = 0x2BA01477 (Cortex-M IDCODE)
            ← 1-bit Parity
       ← [OK, 0x2BA01477]

=== 7. 读写内存 (通过 AP) ===
OpenOCD → DAP_Transfer([
    写 DP_SELECT(AP=0, Bank=0),     // 选择 MEM-AP
    写 AP_CSW(Size=Word, AddrInc),  // 配置 AP
    写 AP_TAR(0x20000000),          // 设置目标地址
    读 AP_DRW,                       // 读数据（posted）
    读 AP_DRW,                       // 读数据（返回上次的值 + post 新读）
    ...
    读 DP_RDBUFF                     // 获取最后一次 posted read 的结果
])
```

每一步的 `SWD_Transfer()` 都会在物理线上产生约 46 个 SWCLK 周期（8 request + 1 turnaround + 3 ack + 1 turnaround + 32 data + 1 parity）。在 10MHz 时钟下，单次传输约 4.6μs。一个 512 字节的 USB 包可以携带约 100 次传输请求，实现很高的吞吐量。

---

## 附：代码目录速查

```
CherryDAP/
├── dap_main.h/c              ← USB 层：描述符、端点、缓冲区、调度
├── DAP/
│   ├── Config/DAP_config.h   ← 硬件抽象模板（引脚操作桩函数）
│   ├── Include/DAP.h         ← 协议常量、DAP_Data_t、函数声明
│   └── Source/
│       ├── DAP.c             ← 命令分派、Transfer、Info、Connect
│       ├── SW_DP.c           ← SWD bit-bang 时序
│       ├── JTAG_DP.c         ← JTAG bit-bang 时序
│       ├── SWO.c             ← Serial Wire Output 追踪
│       ├── UART.c            ← DAP UART 通信
│       └── DAP_vendor.c      ← 厂商自定义命令
├── CherryUSB/                ← USB 协议栈
├── CherryRB/                 ← 环形缓冲区库
└── projects/HSLink-Pro/src/
    ├── DAP_config.h          ← HPM5301 引脚实现（覆盖模板）
    ├── SW_DP/                ← SWD 初始化（IO/SPI 双模式）
    ├── JTAG_DP/              ← JTAG 初始化（IO/SPI 双模式）
    └── main.cpp              ← 主循环入口
```