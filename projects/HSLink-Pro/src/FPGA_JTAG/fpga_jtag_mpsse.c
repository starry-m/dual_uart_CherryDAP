/*
 * MPSSE JTAG Protocol Engine for FPGA Programming
 * Based on RV-Debugger-BL702 jtag_process implementation
 * Adapted for HPM5301 GPIO and CherryUSB
 */

#include "fpga_jtag_mpsse.h"
#include "fpga_jtag_io.h"
#include "chry_ringbuffer.h"
#include <string.h>

/* TX ring buffer for JTAG response data */
static uint8_t jtag_tx_buffer[FPGA_JTAG_TX_BUFFER_SIZE];
static chry_ringbuffer_t jtag_tx_rb;

/* RX buffer for incoming MPSSE commands */
static uint8_t jtag_rx_buffer[FPGA_JTAG_RX_BUFFER_SIZE];
static uint32_t jtag_rx_len = 0;
static volatile uint32_t jtag_rx_pos = 0;

/* MPSSE state machine variables */
static uint32_t mpsse_longlen = 0;
static uint32_t mpsse_shortlen = 0;
static uint32_t mpsse_status = MPSSE_IDLE;
static uint32_t jtag_cmd = 0;
static volatile bool jtag_received_flag = false;

static inline void jtag_write(uint8_t data)
{
    chry_ringbuffer_write_byte(&jtag_tx_rb, data);
}

void fpga_mpsse_init(void)
{
    memset(jtag_tx_buffer, 0, sizeof(jtag_tx_buffer));
    chry_ringbuffer_init(&jtag_tx_rb, jtag_tx_buffer, FPGA_JTAG_TX_BUFFER_SIZE);
    mpsse_status = MPSSE_IDLE;
    jtag_received_flag = false;
    jtag_rx_pos = 0;
    jtag_rx_len = 0;
}

void fpga_mpsse_feed(uint8_t *data, uint32_t len)
{
    if (len > FPGA_JTAG_RX_BUFFER_SIZE) {
        len = FPGA_JTAG_RX_BUFFER_SIZE;
    }
    memcpy(jtag_rx_buffer, data, len);
    jtag_rx_len = len;
    jtag_rx_pos = 0;
    jtag_received_flag = true;
}

bool fpga_mpsse_is_busy(void)
{
    return jtag_received_flag;
}

uint32_t fpga_mpsse_read_tx(uint8_t *buf, uint32_t max_len)
{
    return chry_ringbuffer_read(&jtag_tx_rb, buf, max_len);
}

bool fpga_mpsse_process(void)
{
    uint32_t usb_tx_data = 0;
    uint32_t data = 0;

    if (!jtag_received_flag) {
        return chry_ringbuffer_get_used(&jtag_tx_rb) > 0;
    }

    switch (mpsse_status) {
        case MPSSE_IDLE:
            jtag_cmd = jtag_rx_buffer[jtag_rx_pos];

            switch (jtag_cmd) {
                case 0x80:
                case 0x82: /* Fake Bit-bang mode set */
                    mpsse_status = MPSSE_NO_OP_1;
                    jtag_rx_pos++;
                    break;

                case 0x81:
                case 0x83: /* Fake read pins */
                    usb_tx_data = jtag_rx_buffer[jtag_rx_pos] - 0x80;
                    jtag_write(usb_tx_data);
                    jtag_rx_pos++;
                    break;

                case 0x84:
                case 0x85: /* Loopback */
                    jtag_rx_pos++;
                    break;

                case 0x86: /* Set clock divisor - skip 2 bytes */
                    mpsse_status = MPSSE_NO_OP_1;
                    jtag_rx_pos++;
                    break;

                case 0x87: /* Flush buffer immediately */
                    jtag_rx_pos++;
                    break;

                /* FT2232H specific commands (single byte, no parameters) */
                case 0x8A: /* Disable Clock Divide by 5 (60MHz) */
                case 0x8B: /* Enable Clock Divide by 5 (12MHz) */
                case 0x8C: /* Enable 3-phase data clocking */
                case 0x8D: /* Disable 3-phase data clocking */
                case 0x96: /* Disable adaptive clocking */
                case 0x97: /* Enable adaptive clocking */
                    jtag_rx_pos++;
                    break;

                /* Byte transfer commands (LSB first) */
                case 0x19: /* Clock Data Bytes Out on -ve clock edge LSB first (no read) */
                case 0x1d: /* Clock Data Bytes Out on -ve clock edge LSB first (no read) ??? */
                case 0x39: /* Clock Data Bytes In and Out LSB first */
                case 0x3d:
                /* Byte transfer commands (MSB first) */
                case 0x11: /* Clock Data Bytes Out on +ve clock edge MSB first */
                case 0x15:
                case 0x31: /* Clock Data Bytes In and Out MSB first */
                case 0x35:
                /* Read-only byte commands */
                case 0x28: /* Clock Data Bytes In on +ve clock edge LSB first */
                case 0x2c: /* Clock Data Bytes In on -ve clock edge LSB first */
                case 0x20: /* Clock Data Bytes In on +ve clock edge MSB first */
                case 0x24: /* Clock Data Bytes In on -ve clock edge MSB first */
                    mpsse_status = MPSSE_RCV_LENGTH_L;
                    jtag_rx_pos++;
                    break;

                /* Bit transfer commands */
                case 0x6b: /* Clock Data to TMS pin (no read) */
                case 0x6f: /* Clock Data to TMS pin with read */
                case 0x4b: /* Clock Data to TMS pin (no read) */
                case 0x4f: /* Clock Data to TMS pin with read */
                case 0x3b: /* Clock Data Bits In and Out LSB first */
                case 0x3f:
                case 0x1b: /* Clock Data Bits Out on -ve clock edge LSB first */
                case 0x1f:
                case 0x13: /* Clock Data Bits Out on +ve clock edge MSB first */
                case 0x17:
                /* Read-only bit commands */
                case 0x2a: /* Clock Data Bits In on +ve clock edge LSB first */
                case 0x2e: /* Clock Data Bits In on -ve clock edge LSB first */
                case 0x22: /* Clock Data Bits In on +ve clock edge MSB first */
                case 0x26: /* Clock Data Bits In on -ve clock edge MSB first */
                    mpsse_status = MPSSE_RCV_LENGTH;
                    jtag_rx_pos++;
                    break;

                default:
                    usb_tx_data = 0xFA; /* Bad command response */
                    jtag_write(usb_tx_data);
                    mpsse_status = MPSSE_ERROR;
                    break;
            }
            break;

        case MPSSE_RCV_LENGTH_L: /* Receive length low byte */
            mpsse_longlen = jtag_rx_buffer[jtag_rx_pos];
            mpsse_status = MPSSE_RCV_LENGTH_H;
            jtag_rx_pos++;
            break;

        case MPSSE_RCV_LENGTH_H: /* Receive length high byte */
            mpsse_longlen |= (jtag_rx_buffer[jtag_rx_pos] << 8) & 0xff00;
            jtag_rx_pos++;

            if (jtag_cmd == 0x20 || jtag_cmd == 0x24) {
                mpsse_status = MPSSE_READ_BYTE_MSB;
            } else if (jtag_cmd == 0x28 || jtag_cmd == 0x2c) {
                mpsse_status = MPSSE_READ_BYTE_LSB;
            } else if (jtag_cmd == 0x11 || jtag_cmd == 0x31 || jtag_cmd == 0x15 || jtag_cmd == 0x35) {
                mpsse_status = MPSSE_TRANSMIT_BYTE_MSB;
            } else {
                mpsse_status = MPSSE_TRANSMIT_BYTE;
            }
            break;

        case MPSSE_TRANSMIT_BYTE: /* LSB first byte transfer */
            data = jtag_rx_buffer[jtag_rx_pos];
            usb_tx_data = 0;

            for (uint32_t i = 8; i; i--) {
                FPGA_TCK_LOW();

                if (data & 0x01) {
                    FPGA_TDI_HIGH();
                } else {
                    FPGA_TDI_LOW();
                }

                data >>= 1;
                usb_tx_data >>= 1;

                FPGA_TCK_HIGH();

                if (FPGA_TDO_READ()) {
                    usb_tx_data |= 0x80;
                }
            }

            FPGA_TCK_LOW();

            /* Write-read commands: 0x39, 0x3d */
            if (jtag_cmd == 0x39 || jtag_cmd == 0x3d) {
                jtag_write(usb_tx_data);
            }

            if (mpsse_longlen == 0) {
                mpsse_status = MPSSE_IDLE;
            }

            mpsse_longlen--;
            jtag_rx_pos++;
            break;

        case MPSSE_TRANSMIT_BYTE_MSB: /* MSB first byte transfer */
            data = jtag_rx_buffer[jtag_rx_pos];
            usb_tx_data = 0;

            for (uint32_t i = 8; i; i--) {
                FPGA_TCK_LOW();

                if (data & 0x80) {
                    FPGA_TDI_HIGH();
                } else {
                    FPGA_TDI_LOW();
                }

                data <<= 1;
                usb_tx_data <<= 1;

                FPGA_TCK_HIGH();

                if (FPGA_TDO_READ()) {
                    usb_tx_data |= 0x01;
                }
            }

            FPGA_TCK_LOW();

            /* Write-read commands: 0x31, 0x35 */
            if (jtag_cmd == 0x31 || jtag_cmd == 0x35) {
                jtag_write(usb_tx_data);
            }

            if (mpsse_longlen == 0) {
                mpsse_status = MPSSE_IDLE;
            }

            jtag_rx_pos++;
            mpsse_longlen--;
            break;

        case MPSSE_RCV_LENGTH: /* Receive length for bit commands */
            mpsse_shortlen = jtag_rx_buffer[jtag_rx_pos];

            if (jtag_cmd == 0x6b || jtag_cmd == 0x4b || jtag_cmd == 0x6f || jtag_cmd == 0x4f) {
                mpsse_status = MPSSE_TMS_OUT;
            } else if (jtag_cmd == 0x22 || jtag_cmd == 0x26) {
                mpsse_status = MPSSE_READ_BIT_MSB;
            } else if (jtag_cmd == 0x2a || jtag_cmd == 0x2e) {
                mpsse_status = MPSSE_READ_BIT_LSB;
            } else if (jtag_cmd == 0x13 || jtag_cmd == 0x17) {
                mpsse_status = MPSSE_TRANSMIT_BIT_MSB;
            } else {
                mpsse_status = MPSSE_TRANSMIT_BIT;
            }

            jtag_rx_pos++;
            break;

        case MPSSE_TRANSMIT_BIT: /* LSB first bit transfer */
            data = jtag_rx_buffer[jtag_rx_pos];
            usb_tx_data = 0;

            do {
                FPGA_TCK_LOW();

                if (data & 0x01) {
                    FPGA_TDI_HIGH();
                } else {
                    FPGA_TDI_LOW();
                }

                data >>= 1;
                usb_tx_data >>= 1;

                FPGA_TCK_HIGH();

                if (FPGA_TDO_READ()) {
                    usb_tx_data |= 0x80;
                }
            } while ((mpsse_shortlen--) > 0);

            FPGA_TCK_LOW();

            /* Write-read commands: 0x3b, 0x3f */
            if (jtag_cmd == 0x3b || jtag_cmd == 0x3f) {
                jtag_write(usb_tx_data);
            }

            mpsse_status = MPSSE_IDLE;
            jtag_rx_pos++;
            break;

        case MPSSE_TRANSMIT_BIT_MSB: /* MSB first bit transfer */
            data = jtag_rx_buffer[jtag_rx_pos];

            do {
                FPGA_TCK_LOW();

                if (data & 0x80) {
                    FPGA_TDI_HIGH();
                } else {
                    FPGA_TDI_LOW();
                }

                data <<= 1;

                FPGA_TCK_HIGH();
            } while ((mpsse_shortlen--) > 0);

            FPGA_TCK_LOW();

            mpsse_status = MPSSE_IDLE;
            jtag_rx_pos++;
            break;

        case MPSSE_ERROR:
            usb_tx_data = jtag_rx_buffer[jtag_rx_pos];
            jtag_write(usb_tx_data);
            mpsse_status = MPSSE_IDLE;
            jtag_rx_pos++;
            break;

        case MPSSE_TMS_OUT: /* TMS bit-bang with optional TDO read */
            data = jtag_rx_buffer[jtag_rx_pos];

            /* Set TDI from bit 7 */
            if (data & 0x80) {
                FPGA_TDI_HIGH();
            } else {
                FPGA_TDI_LOW();
            }

            usb_tx_data = 0;

            do {
                FPGA_TCK_LOW();

                if (data & 0x01) {
                    FPGA_TMS_HIGH();
                } else {
                    FPGA_TMS_LOW();
                }

                data >>= 1;
                usb_tx_data >>= 1;

                FPGA_TCK_HIGH();

                if (FPGA_TDO_READ()) {
                    usb_tx_data |= 0x80;
                }
            } while ((mpsse_shortlen--) > 0);

            FPGA_TCK_LOW();

            /* Read-back commands: 0x6b, 0x6f */
            if (jtag_cmd == 0x6b || jtag_cmd == 0x6f) {
                jtag_write(usb_tx_data);
            }

            mpsse_status = MPSSE_IDLE;
            jtag_rx_pos++;
            break;

        case MPSSE_NO_OP_1:
            jtag_rx_pos++;
            mpsse_status = MPSSE_NO_OP_2;
            break;

        case MPSSE_NO_OP_2:
            mpsse_status = MPSSE_IDLE;
            jtag_rx_pos++;
            break;

        case MPSSE_READ_BYTE_LSB: /* Read-only byte, LSB first, no data consumed */
            usb_tx_data = 0;
            for (uint32_t i = 8; i; i--) {
                FPGA_TCK_LOW();
                usb_tx_data >>= 1;
                FPGA_TCK_HIGH();
                if (FPGA_TDO_READ()) {
                    usb_tx_data |= 0x80;
                }
            }
            FPGA_TCK_LOW();
            jtag_write(usb_tx_data);

            if (mpsse_longlen == 0) {
                mpsse_status = MPSSE_IDLE;
            }
            mpsse_longlen--;
            break;

        case MPSSE_READ_BYTE_MSB: /* Read-only byte, MSB first */
            usb_tx_data = 0;
            for (uint32_t i = 8; i; i--) {
                FPGA_TCK_LOW();
                usb_tx_data <<= 1;
                FPGA_TCK_HIGH();
                if (FPGA_TDO_READ()) {
                    usb_tx_data |= 0x01;
                }
            }
            FPGA_TCK_LOW();
            jtag_write(usb_tx_data);

            if (mpsse_longlen == 0) {
                mpsse_status = MPSSE_IDLE;
            }
            mpsse_longlen--;
            break;

        case MPSSE_READ_BIT_LSB: /* Read-only bits, LSB first */
            usb_tx_data = 0;
            do {
                FPGA_TCK_LOW();
                usb_tx_data >>= 1;
                FPGA_TCK_HIGH();
                if (FPGA_TDO_READ()) {
                    usb_tx_data |= 0x80;
                }
            } while ((mpsse_shortlen--) > 0);
            FPGA_TCK_LOW();
            jtag_write(usb_tx_data);
            mpsse_status = MPSSE_IDLE;
            /* No jtag_rx_pos++ — read-only, no data byte to consume */
            break;

        case MPSSE_READ_BIT_MSB: /* Read-only bits, MSB first */
            usb_tx_data = 0;
            do {
                FPGA_TCK_LOW();
                usb_tx_data <<= 1;
                FPGA_TCK_HIGH();
                if (FPGA_TDO_READ()) {
                    usb_tx_data |= 0x01;
                }
            } while ((mpsse_shortlen--) > 0);
            FPGA_TCK_LOW();
            jtag_write(usb_tx_data);
            mpsse_status = MPSSE_IDLE;
            /* No jtag_rx_pos++ — read-only, no data byte to consume */
            break;

        default:
            mpsse_status = MPSSE_IDLE;
            break;
    }

    /* Check if all data consumed */
    if (jtag_rx_pos >= jtag_rx_len) {
        jtag_received_flag = false;
    }

    return chry_ringbuffer_get_used(&jtag_tx_rb) > 0;
}
