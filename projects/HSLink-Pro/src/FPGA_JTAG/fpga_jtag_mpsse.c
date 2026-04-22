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

/* MPSSE clock rate emulation
 * FT2232H: freq = base / ((1 + divisor) * 2)
 *   base = 60MHz (div5 off, 0x8A) or 12MHz (div5 on, 0x8B)
 * We emulate the timing by adding delays between TCK edges. */
static bool mpsse_div5_enabled = true;       /* 0x8B: divide-by-5 enabled (12MHz base) */
static uint16_t mpsse_divisor = 0;           /* 0x86: clock divisor */
static uint32_t mpsse_half_clk_delay = 5;    /* delay loop count per half-clock */

static void mpsse_update_clock(void)
{
    uint32_t base = mpsse_div5_enabled ? 12000000U : 60000000U;
    uint32_t freq = base / ((1 + mpsse_divisor) * 2);
    if (freq == 0) freq = 1;
    /* Half-period in CPU cycles: CPU_CLOCK / (2 * freq)
     * Each delay loop iteration ~3 cycles; GPIO overhead ~5 cycles */
    uint32_t half_period = 100000000U / (2 * freq);
    mpsse_half_clk_delay = (half_period > 5) ? (half_period - 5) / 3 : 0;
}

static inline void mpsse_clk_wait(void)
{
    volatile uint32_t cnt = mpsse_half_clk_delay;
    while (cnt--) { __asm volatile(""); }
}

/* TCK macros with clock-rate-controlled delay for MPSSE engine */
#define JTAG_TCK_LOW()   do { FPGA_TCK_LOW();  mpsse_clk_wait(); } while(0)
#define JTAG_TCK_HIGH()  do { FPGA_TCK_HIGH(); mpsse_clk_wait(); } while(0)

static inline void jtag_write(uint8_t data)
{
    chry_ringbuffer_write_byte(&jtag_tx_rb, data);
}

void fpga_mpsse_purge_tx(void)
{
    chry_ringbuffer_reset(&jtag_tx_rb);
}

void fpga_mpsse_init(void)
{
    memset(jtag_tx_buffer, 0, sizeof(jtag_tx_buffer));
    chry_ringbuffer_init(&jtag_tx_rb, jtag_tx_buffer, FPGA_JTAG_TX_BUFFER_SIZE);
    mpsse_status = MPSSE_IDLE;
    jtag_received_flag = false;
    jtag_rx_pos = 0;
    jtag_rx_len = 0;
    mpsse_div5_enabled = true;
    mpsse_divisor = 0;
    mpsse_update_clock();
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
    return jtag_received_flag ||
           mpsse_status == MPSSE_READ_BYTE_LSB ||
           mpsse_status == MPSSE_READ_BYTE_MSB ||
           mpsse_status == MPSSE_READ_BIT_LSB ||
           mpsse_status == MPSSE_READ_BIT_MSB ||
           mpsse_status == MPSSE_CLOCK_BITS ||
           mpsse_status == MPSSE_CLOCK_BYTES;
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
        /* Allow data-less states to continue without new USB data */
        if (mpsse_status != MPSSE_READ_BYTE_LSB &&
            mpsse_status != MPSSE_READ_BYTE_MSB &&
            mpsse_status != MPSSE_READ_BIT_LSB &&
            mpsse_status != MPSSE_READ_BIT_MSB &&
            mpsse_status != MPSSE_CLOCK_BITS &&
            mpsse_status != MPSSE_CLOCK_BYTES) {
            return chry_ringbuffer_get_used(&jtag_tx_rb) > 0;
        }
    }

    switch (mpsse_status) {
        case MPSSE_IDLE:
            jtag_cmd = jtag_rx_buffer[jtag_rx_pos];

            switch (jtag_cmd) {
                case 0x80: /* Set Data Bits Low Byte (ADBUS) */
                case 0x82: /* Set Data Bits High Byte (ACBUS) */
                    mpsse_status = MPSSE_NO_OP_1;
                    jtag_rx_pos++;
                    break;

                case 0x81: /* Read Data Bits Low Byte (ADBUS) */
                    usb_tx_data = 0;
                    if (FPGA_TDO_READ()) usb_tx_data |= 0x04;
                    /* Return TDO on bit 2, other bits as idle state */
                    jtag_write(usb_tx_data);
                    jtag_rx_pos++;
                    break;

                case 0x83: /* Read Data Bits High Byte (ACBUS) */
                    jtag_write(0x00); /* No high-byte pins connected */
                    jtag_rx_pos++;
                    break;

                case 0x84:
                case 0x85: /* Loopback */
                    jtag_rx_pos++;
                    break;

                case 0x86: /* Set clock divisor - 2 parameter bytes */
                    mpsse_status = MPSSE_NO_OP_1;
                    jtag_rx_pos++;
                    break;

                case 0x87: /* Flush buffer immediately */
                    jtag_rx_pos++;
                    break;

                /* FT2232H specific commands (single byte, no parameters) */
                case 0x8A: /* Disable Clock Divide by 5 (60MHz base) */
                    mpsse_div5_enabled = false;
                    mpsse_update_clock();
                    jtag_rx_pos++;
                    break;
                case 0x8B: /* Enable Clock Divide by 5 (12MHz base) */
                    mpsse_div5_enabled = true;
                    mpsse_update_clock();
                    jtag_rx_pos++;
                    break;
                case 0x8C: /* Enable 3-phase data clocking */
                case 0x8D: /* Disable 3-phase data clocking */
                case 0x96: /* Disable adaptive clocking */
                case 0x97: /* Enable adaptive clocking */
                    jtag_rx_pos++;
                    break;

                case 0x8E: /* Clock For n bits with no data transfer */
                    mpsse_status = MPSSE_RCV_LENGTH;
                    jtag_rx_pos++;
                    break;

                case 0x8F: /* Clock For n x 8 bits with no data transfer */
                case 0x9C: /* Clock For n x 8 bits or until GPIOL1 is low */
                case 0x9D: /* Clock For n x 8 bits or until GPIOL1 is high */
                    mpsse_status = MPSSE_RCV_LENGTH_L;
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

            if (jtag_cmd == 0x8F || jtag_cmd == 0x9C || jtag_cmd == 0x9D) {
                mpsse_status = MPSSE_CLOCK_BYTES;
            } else if (jtag_cmd == 0x20 || jtag_cmd == 0x24) {
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
                JTAG_TCK_LOW();

                if (data & 0x01) {
                    FPGA_TDI_HIGH();
                } else {
                    FPGA_TDI_LOW();
                }

                data >>= 1;
                usb_tx_data >>= 1;

                JTAG_TCK_HIGH();

                if (FPGA_TDO_READ()) {
                    usb_tx_data |= 0x80;
                }
            }

            JTAG_TCK_LOW();

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
                JTAG_TCK_LOW();

                if (data & 0x80) {
                    FPGA_TDI_HIGH();
                } else {
                    FPGA_TDI_LOW();
                }

                data <<= 1;
                usb_tx_data <<= 1;

                JTAG_TCK_HIGH();

                if (FPGA_TDO_READ()) {
                    usb_tx_data |= 0x01;
                }
            }

            JTAG_TCK_LOW();

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

            if (jtag_cmd == 0x8E) {
                mpsse_status = MPSSE_CLOCK_BITS;
            } else if (jtag_cmd == 0x6b || jtag_cmd == 0x4b || jtag_cmd == 0x6f || jtag_cmd == 0x4f) {
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
                JTAG_TCK_LOW();

                if (data & 0x01) {
                    FPGA_TDI_HIGH();
                } else {
                    FPGA_TDI_LOW();
                }

                data >>= 1;
                usb_tx_data >>= 1;

                JTAG_TCK_HIGH();

                if (FPGA_TDO_READ()) {
                    usb_tx_data |= 0x80;
                }
            } while ((mpsse_shortlen--) > 0);

            JTAG_TCK_LOW();

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
                JTAG_TCK_LOW();

                if (data & 0x80) {
                    FPGA_TDI_HIGH();
                } else {
                    FPGA_TDI_LOW();
                }

                data <<= 1;

                JTAG_TCK_HIGH();
            } while ((mpsse_shortlen--) > 0);

            JTAG_TCK_LOW();

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
                JTAG_TCK_LOW();

                if (data & 0x01) {
                    FPGA_TMS_HIGH();
                } else {
                    FPGA_TMS_LOW();
                }

                data >>= 1;
                usb_tx_data >>= 1;

                JTAG_TCK_HIGH();

                if (FPGA_TDO_READ()) {
                    usb_tx_data |= 0x80;
                }
            } while ((mpsse_shortlen--) > 0);

            JTAG_TCK_LOW();

            /* Read-back commands: 0x6b, 0x6f */
            if (jtag_cmd == 0x6b || jtag_cmd == 0x6f) {
                jtag_write(usb_tx_data);
            }

            mpsse_status = MPSSE_IDLE;
            jtag_rx_pos++;
            break;

        case MPSSE_NO_OP_1:
            if (jtag_cmd == 0x80) {
                /* Set Data Bits Low Byte: apply GPIO output values
                 * Bit 0 = TCK, Bit 1 = TDI, Bit 2 = TDO(in), Bit 3 = TMS */
                uint8_t val = jtag_rx_buffer[jtag_rx_pos];
                if (val & 0x01) FPGA_TCK_HIGH(); else FPGA_TCK_LOW();
                if (val & 0x02) FPGA_TDI_HIGH(); else FPGA_TDI_LOW();
                if (val & 0x08) FPGA_TMS_HIGH(); else FPGA_TMS_LOW();
            } else if (jtag_cmd == 0x86) {
                /* Clock divisor low byte */
                mpsse_divisor = jtag_rx_buffer[jtag_rx_pos];
            }
            jtag_rx_pos++;
            mpsse_status = MPSSE_NO_OP_2;
            break;

        case MPSSE_NO_OP_2:
            if (jtag_cmd == 0x86) {
                /* Clock divisor high byte - recalculate clock delay */
                mpsse_divisor |= (uint16_t)(jtag_rx_buffer[jtag_rx_pos]) << 8;
                mpsse_update_clock();
            }
            mpsse_status = MPSSE_IDLE;
            jtag_rx_pos++;
            break;

        case MPSSE_READ_BYTE_LSB: /* Read-only byte, LSB first, no data consumed */
            usb_tx_data = 0;
            for (uint32_t i = 8; i; i--) {
                JTAG_TCK_LOW();
                usb_tx_data >>= 1;
                JTAG_TCK_HIGH();
                if (FPGA_TDO_READ()) {
                    usb_tx_data |= 0x80;
                }
            }
            JTAG_TCK_LOW();
            jtag_write(usb_tx_data);

            if (mpsse_longlen == 0) {
                mpsse_status = MPSSE_IDLE;
            }
            mpsse_longlen--;
            break;

        case MPSSE_READ_BYTE_MSB: /* Read-only byte, MSB first */
            usb_tx_data = 0;
            for (uint32_t i = 8; i; i--) {
                JTAG_TCK_LOW();
                usb_tx_data <<= 1;
                JTAG_TCK_HIGH();
                if (FPGA_TDO_READ()) {
                    usb_tx_data |= 0x01;
                }
            }
            JTAG_TCK_LOW();
            jtag_write(usb_tx_data);

            if (mpsse_longlen == 0) {
                mpsse_status = MPSSE_IDLE;
            }
            mpsse_longlen--;
            break;

        case MPSSE_READ_BIT_LSB: /* Read-only bits, LSB first */
            usb_tx_data = 0;
            do {
                JTAG_TCK_LOW();
                usb_tx_data >>= 1;
                JTAG_TCK_HIGH();
                if (FPGA_TDO_READ()) {
                    usb_tx_data |= 0x80;
                }
            } while ((mpsse_shortlen--) > 0);
            JTAG_TCK_LOW();
            jtag_write(usb_tx_data);
            mpsse_status = MPSSE_IDLE;
            /* No jtag_rx_pos++ — read-only, no data byte to consume */
            break;

        case MPSSE_READ_BIT_MSB: /* Read-only bits, MSB first */
            usb_tx_data = 0;
            do {
                JTAG_TCK_LOW();
                usb_tx_data <<= 1;
                JTAG_TCK_HIGH();
                if (FPGA_TDO_READ()) {
                    usb_tx_data |= 0x01;
                }
            } while ((mpsse_shortlen--) > 0);
            JTAG_TCK_LOW();
            jtag_write(usb_tx_data);
            mpsse_status = MPSSE_IDLE;
            /* No jtag_rx_pos++ — read-only, no data byte to consume */
            break;

        case MPSSE_CLOCK_BITS: /* Clock TCK for (shortlen+1) cycles, no data */
            do {
                JTAG_TCK_LOW();
                JTAG_TCK_HIGH();
            } while ((mpsse_shortlen--) > 0);
            JTAG_TCK_LOW();
            mpsse_status = MPSSE_IDLE;
            /* No jtag_rx_pos++ — no data byte to consume */
            break;

        case MPSSE_CLOCK_BYTES: /* Clock 8 TCK cycles per iteration, (longlen+1) iterations */
            for (uint32_t i = 8; i; i--) {
                JTAG_TCK_LOW();
                JTAG_TCK_HIGH();
            }
            JTAG_TCK_LOW();
            if (mpsse_longlen == 0) {
                mpsse_status = MPSSE_IDLE;
            }
            mpsse_longlen--;
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
