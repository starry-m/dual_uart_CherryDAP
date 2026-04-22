/*
 * FTDI USB Interface Emulation for FPGA JTAG
 * Standalone FT2232-compatible USB device for FPGA programming tools
 */

#ifndef USBD_FTDI_H
#define USBD_FTDI_H

#include <stdint.h>
#include <stdbool.h>
#include "usbd_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* FPGA JTAG Endpoints - Channel A (JTAG/MPSSE) */
#define FPGA_JTAG_IN_EP  0x81
#define FPGA_JTAG_OUT_EP 0x02

/* Channel B (UART bridge) */
#define FTDI_CHB_IN_EP   0x83
#define FTDI_CHB_OUT_EP  0x04

/* FTDI EEPROM emulation data (FT2232 compatible) */
extern const uint16_t ftdi_eeprom_info[];

/* Initialize FPGA JTAG GPIO and MPSSE engine (call once at startup) */
void fpga_jtag_init(void);

/* Process FPGA JTAG MPSSE commands - call in main loop when in FPGA mode */
void fpga_jtag_process(void);

/* Process Channel B UART bridge - call in main loop when in FPGA mode */
void fpga_uart_handle(void);

/* Initialize standalone FPGA/FT2232 USB device */
void fpga_usb_init(uint8_t busid, uint32_t reg_base);

/* Deinitialize FPGA USB device */
void fpga_usb_deinit(uint8_t busid);

/* USB callbacks for FPGA JTAG endpoints */
void fpga_jtag_out_callback(uint8_t busid, uint8_t ep, uint32_t nbytes);
void fpga_jtag_in_callback(uint8_t busid, uint8_t ep, uint32_t nbytes);

/* FTDI vendor request handler (handles both Channel A and B via wIndex) */
int ftdi_vendor_request_handler(uint8_t busid, struct usb_setup_packet *setup,
                                uint8_t **data, uint32_t *len);

/* Poll for FTDI UART config changes from Channel B vendor requests */
struct cdc_line_coding;
bool ftdi_uart_config_poll(struct cdc_line_coding *lc);

/* Get latency timer values (ms) */
uint8_t ftdi_get_latency_timer_a(void);
uint8_t ftdi_get_latency_timer_b(void);

/* Get which port is currently in MPSSE mode (0=none, 1=ChA, 2=ChB) */
uint8_t ftdi_get_mpsse_port(void);

/* Reset endpoint TX state (called on SIO_RESET / clear halt recovery) */
void fpga_reset_tx_state(void);

/* Port-specific TX idle control (for port-aware SIO_RESET) */
void fpga_tx_idle_set(bool idle);
void fpga_chb_tx_idle_set(bool idle);

/* Discard pending RX data and re-arm OUT endpoint (for SIO_RESET full reset) */
void fpga_discard_rx(void);

#ifdef __cplusplus
}
#endif

#endif /* USBD_FTDI_H */
