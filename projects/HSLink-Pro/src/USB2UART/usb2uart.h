#ifndef HSLINK_PRO_USB2UART_H
#define HSLINK_PRO_USB2UART_H

#define PIN_UART_TX IOC_PAD_PA08
#define PIN_UART_RX IOC_PAD_PA09

#define PIN_UART_2ND_TX IOC_PAD_PA07
#define PIN_UART_2ND_RX IOC_PAD_PA06

#ifdef __cplusplus
extern "C"
{
#endif

void uartx_io_init(void);

void uartx_preinit(void);

void usb2uart_handler(void);

#if CONFIG_CHERRYDAP_DUAL_UART

void uartx_2nd_preinit(void);

void usb2uart_2nd_handler(void);
#endif

#ifdef __cplusplus
}
#endif

#endif //HSLINK_PRO_USB2UART_H
