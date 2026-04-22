/*
 * FTDI USB Interface Emulation for FPGA JTAG
 * Based on RV-Debugger-BL702 usbd_ftdi implementation
 * Adapted for CherryUSB v2 API and HPM5301
 */

#include "usbd_ftdi.h"
#include "fpga_jtag_io.h"
#include "fpga_jtag_mpsse.h"
#include "chry_ringbuffer.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include <string.h>

/* FTDI EEPROM emulation data - FT2232 compatible */
const uint16_t ftdi_eeprom_info[] = {
    0x0800, 0x0403, 0x6010, 0x0500, 0x3280, 0x0000, 0x0200, 0x1096,
    0x1aa6, 0x0000, 0x0046, 0x0310, 0x004f, 0x0070, 0x0065, 0x006e,
    0x002d, 0x0045, 0x0043, 0x031a, 0x0055, 0x0053, 0x0042, 0x0020,
    0x0044, 0x0065, 0x0062, 0x0075, 0x0067, 0x0067, 0x0065, 0x0072,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x1027
};

/* FTDI SIO Requests */
#define SIO_RESET_REQUEST             0x00
#define SIO_SET_MODEM_CTRL_REQUEST    0x01
#define SIO_SET_FLOW_CTRL_REQUEST     0x02
#define SIO_SET_BAUDRATE_REQUEST      0x03
#define SIO_SET_DATA_REQUEST          0x04
#define SIO_POLL_MODEM_STATUS_REQUEST 0x05
#define SIO_SET_EVENT_CHAR_REQUEST    0x06
#define SIO_SET_ERROR_CHAR_REQUEST    0x07
#define SIO_SET_LATENCY_TIMER_REQUEST 0x09
#define SIO_GET_LATENCY_TIMER_REQUEST 0x0A
#define SIO_SET_BITMODE_REQUEST       0x0B
#define SIO_READ_PINS_REQUEST         0x0C
#define SIO_READ_EEPROM_REQUEST       0x90
#define SIO_WRITE_EEPROM_REQUEST      0x91
#define SIO_ERASE_EEPROM_REQUEST      0x92

static uint8_t latency_timer_a = 0x10;
static uint8_t latency_timer_b = 0x10;

/* Which port is in MPSSE mode: 0=none, 1=Channel A, 2=Channel B */
static uint8_t mpsse_port = 0;

/* Modem status response: consistent value for both control and bulk transfers */
static uint8_t ftdi_modem_status[2] = { 0x01, 0x60 }; /* no signals, THRE+TEMT */

/* SIO_READ_PINS response */
static uint8_t ftdi_pin_state = 0x00;

/* Channel B UART configuration state */
static volatile bool ftdi_uart_cfg_pending = false;
static uint32_t ftdi_uart_baudrate = 115200;
static uint8_t ftdi_uart_databits = 8;
static uint8_t ftdi_uart_parity = 0;
static uint8_t ftdi_uart_stopbits = 0;

static void ftdi_set_baudrate(uint32_t itdf_divisor, uint32_t *actual_baudrate)
{
#define FTDI_USB_CLK 48000000
    uint8_t frac[] = {0, 8, 4, 2, 6, 10, 12, 14};
    int divisor = itdf_divisor & 0x3fff;
    int baudrate;

    divisor <<= 4;
    divisor |= frac[(itdf_divisor >> 14) & 0x07];

    if (itdf_divisor == 0x01) {
        baudrate = 2000000;
    } else if (itdf_divisor == 0x00) {
        baudrate = 3000000;
    } else {
        baudrate = FTDI_USB_CLK / divisor;
    }
    *actual_baudrate = baudrate;
}

int ftdi_vendor_request_handler(uint8_t busid, struct usb_setup_packet *setup,
                                uint8_t **data, uint32_t *len)
{
    static uint32_t actual_baudrate = 1200;
    /* FTDI uses wIndex low byte for port: 1=Channel A, 2=Channel B */
    uint8_t port = setup->wIndex & 0xFF;

    switch (setup->bRequest) {
        case SIO_READ_EEPROM_REQUEST:
            if (setup->wIndex < sizeof(ftdi_eeprom_info) / sizeof(uint16_t)) {
                *data = (uint8_t *)&ftdi_eeprom_info[setup->wIndex];
                *len = 2;
            }
            break;

        case SIO_RESET_REQUEST:
            switch (setup->wValue) {
                case 0: /* SIO_RESET_SIO: reset specific port */
                    if (port <= 1) latency_timer_a = 0x10;
                    if (port == 2) latency_timer_b = 0x10;
                    if (port == mpsse_port) {
                        /* Only reset MPSSE when the MPSSE port itself is being reset */
                        fpga_mpsse_init();
                        fpga_tx_idle_set(true);
                        fpga_discard_rx();
                    } else if (port == 2 && mpsse_port != 2) {
                        /* Non-MPSSE Channel B reset */
                        fpga_chb_tx_idle_set(true);
                    }
                    break;
                case 1: /* SIO_RESET_PURGE_RX: purge device→host buffer for specific port */
                    if (port == mpsse_port) {
                        fpga_mpsse_purge_tx();
                        fpga_tx_idle_set(true);
                    } else if (port == 2 && mpsse_port != 2) {
                        fpga_chb_tx_idle_set(true);
                    }
                    break;
                case 2: /* SIO_RESET_PURGE_TX: purge host→device buffer for specific port */
                    if (port == mpsse_port) {
                        fpga_discard_rx();
                    }
                    break;
                default:
                    break;
            }
            break;

        case SIO_SET_MODEM_CTRL_REQUEST:
            /* DTR/RTS control - not used for JTAG */
            break;

        case SIO_SET_FLOW_CTRL_REQUEST:
            break;

        case SIO_SET_BAUDRATE_REQUEST: {
            uint8_t baudrate_high = (setup->wIndex >> 8);
            ftdi_set_baudrate(setup->wValue | (baudrate_high << 16), &actual_baudrate);
            /* Channel B baudrate → configure UART */
            if (port == 2 && actual_baudrate > 0) {
                ftdi_uart_baudrate = actual_baudrate;
                ftdi_uart_cfg_pending = true;
            }
            break;
        }

        case SIO_SET_DATA_REQUEST:
            /* Channel B data format → configure UART */
            if (port == 2) {
                ftdi_uart_databits = setup->wValue & 0xFF;
                ftdi_uart_parity = (setup->wValue >> 8) & 0x07;
                ftdi_uart_stopbits = (setup->wValue >> 11) & 0x03;
                ftdi_uart_cfg_pending = true;
            }
            break;

        case SIO_POLL_MODEM_STATUS_REQUEST:
            *data = ftdi_modem_status;
            *len = 2;
            break;

        case SIO_SET_EVENT_CHAR_REQUEST:
            break;

        case SIO_SET_ERROR_CHAR_REQUEST:
            break;

        case SIO_SET_LATENCY_TIMER_REQUEST:
            if (port == 1)
                latency_timer_a = setup->wValue & 0xFF;
            else
                latency_timer_b = setup->wValue & 0xFF;
            break;

        case SIO_GET_LATENCY_TIMER_REQUEST:
            if (port == 1)
                *data = &latency_timer_a;
            else
                *data = &latency_timer_b;
            *len = 1;
            break;

        case SIO_SET_BITMODE_REQUEST:
            /* Bitbang/MPSSE mode setting */
            if (((setup->wValue >> 8) & 0xFF) == 0x02) {
                mpsse_port = port;  /* This port is now in MPSSE mode */
            } else if (port == mpsse_port) {
                mpsse_port = 0;  /* This port left MPSSE mode */
            }
            break;

        case SIO_READ_PINS_REQUEST:
            *data = &ftdi_pin_state;
            *len = 1;
            break;

        default:
            /* Accept all unknown vendor requests to avoid stalling EP0 */
            break;
    }

    return 0;
}

bool ftdi_uart_config_poll(struct cdc_line_coding *lc)
{
    if (!ftdi_uart_cfg_pending) {
        return false;
    }
    ftdi_uart_cfg_pending = false;
    lc->dwDTERate = ftdi_uart_baudrate;
    lc->bDataBits = ftdi_uart_databits;
    /* FTDI parity: 0=N,1=O,2=E,3=M,4=S — same encoding as CDC */
    lc->bParityType = ftdi_uart_parity;
    /* FTDI stop: 0=1,1=1.5,2=2 — same encoding as CDC */
    lc->bCharFormat = ftdi_uart_stopbits;
    return true;
}

uint8_t ftdi_get_latency_timer_a(void)
{
    return latency_timer_a;
}

uint8_t ftdi_get_latency_timer_b(void)
{
    return latency_timer_b;
}

uint8_t ftdi_get_mpsse_port(void)
{
    return mpsse_port;
}
