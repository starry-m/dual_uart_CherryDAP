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

/* FPGA JTAG Endpoints (standalone device, not composite with DAP) */
#define FPGA_JTAG_IN_EP  0x81
#define FPGA_JTAG_OUT_EP 0x02

/* FTDI EEPROM emulation data (FT2232 compatible) */
extern const uint16_t ftdi_eeprom_info[];

/* Initialize FPGA JTAG GPIO and MPSSE engine (call once at startup) */
void fpga_jtag_init(void);

/* Process FPGA JTAG MPSSE commands - call in main loop when in FPGA mode */
void fpga_jtag_process(void);

/* Initialize standalone FPGA/FT2232 USB device (with CDC UART bridge) */
void fpga_usb_init(uint8_t busid, uint32_t reg_base);

/* Deinitialize FPGA USB device */
void fpga_usb_deinit(uint8_t busid);

/* USB callbacks for FPGA JTAG endpoints */
void fpga_jtag_out_callback(uint8_t busid, uint8_t ep, uint32_t nbytes);
void fpga_jtag_in_callback(uint8_t busid, uint8_t ep, uint32_t nbytes);

/* FTDI vendor request handler for the FPGA interface */
int ftdi_vendor_request_handler(uint8_t busid, struct usb_setup_packet *setup,
                                uint8_t **data, uint32_t *len);

#ifdef __cplusplus
}
#endif

#endif /* USBD_FTDI_H */
