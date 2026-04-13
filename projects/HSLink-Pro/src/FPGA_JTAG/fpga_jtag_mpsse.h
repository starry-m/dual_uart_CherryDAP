/*
 * MPSSE JTAG Protocol Definitions
 */

#ifndef FPGA_JTAG_MPSSE_H
#define FPGA_JTAG_MPSSE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MPSSE State Machine States */
#define MPSSE_IDLE              0
#define MPSSE_RCV_LENGTH_L      1
#define MPSSE_RCV_LENGTH_H      2
#define MPSSE_TRANSMIT_BYTE     3
#define MPSSE_RCV_LENGTH        4
#define MPSSE_TRANSMIT_BIT      5
#define MPSSE_ERROR             6
#define MPSSE_TRANSMIT_BIT_MSB  7
#define MPSSE_TMS_OUT           8
#define MPSSE_NO_OP_1           9
#define MPSSE_NO_OP_2           10
#define MPSSE_TRANSMIT_BYTE_MSB 11
#define MPSSE_READ_BYTE_LSB     12
#define MPSSE_READ_BYTE_MSB     13
#define MPSSE_READ_BIT_LSB      14
#define MPSSE_READ_BIT_MSB      15

/* JTAG buffer sizes */
#define FPGA_JTAG_TX_BUFFER_SIZE (2 * 1024)
#define FPGA_JTAG_RX_BUFFER_SIZE (512)

/* Initialize MPSSE engine */
void fpga_mpsse_init(void);

/* Feed received USB data to MPSSE engine */
void fpga_mpsse_feed(uint8_t *data, uint32_t len);

/* Process MPSSE commands - returns true if there's data to send back */
bool fpga_mpsse_process(void);

/* Get TX ring buffer for sending data back to host */
uint32_t fpga_mpsse_read_tx(uint8_t *buf, uint32_t max_len);

/* Check if MPSSE is currently processing data */
bool fpga_mpsse_is_busy(void);

#ifdef __cplusplus
}
#endif

#endif /* FPGA_JTAG_MPSSE_H */
