# 使用HPM5301 debug HPM6E80 && gn1nr-lv9

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
|PA30|MODE_SWTICH|

## 软件适配

- MCU：现有基于cherryusb方案的cherrydap hslink方案完全满足，只需修改引脚即可。
- FPGA：参考https://github.com/sipeed/RV-Debugger-BL702/tree/main/firmware/app/usb2uartjtag
因为这个也是使用的cherryusb协议栈实现模拟ft2232方案（已实现）

### FPGA JTAG实现说明

在现有CMSIS-DAP复合USB设备基础上，新增一个Vendor Class接口用于FPGA JTAG编程：

- 通过`CONFIG_CHERRYDAP_USE_FPGA_JTAG=1`编译开关启用
- 新增USB接口模拟FT2232 MPSSE协议
- 使用PA04-PA07引脚进行FPGA JTAG bit-bang操作
- 支持MPSSE命令集：字节传输(LSB/MSB)、位传输、TMS输出
- FTDI vendor request兼容（EEPROM读取、波特率设置、延迟定时器等）
- PA30按键一键切换DAP/FT2232模式（下拉，按下高电平）

#### 模式切换

- 默认启动为DAP模式（MCU调试）
- 按一下PA30按键切换到FT2232 FPGA JTAG模式
- 再按一下切回DAP模式
- CDC UART桥接在两种模式下都保持工作

#### 新增文件

| 文件 | 说明 |
|-|-|
| FPGA_JTAG/fpga_jtag_io.h | FPGA JTAG引脚定义和GPIO操作 |
| FPGA_JTAG/fpga_jtag_mpsse.h/c | MPSSE协议状态机实现 |
| FPGA_JTAG/usbd_ftdi.h/c | FTDI USB接口模拟 |
| FPGA_JTAG/fpga_jtag_main.c | USB端点与MPSSE引擎集成 |
| mode_switch.h/c | PA30按键模式切换（带消抖） |

