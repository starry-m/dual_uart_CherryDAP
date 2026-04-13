/*
 * FPGA JTAG Standalone USB Device
 * FT2232H-compatible dual-channel layout:
 *   Channel A (Interface 0): JTAG/MPSSE  - EP 0x81 IN / 0x02 OUT
 *   Channel B (Interface 1): UART bridge - EP 0x83 IN / 0x04 OUT
 * Mutually exclusive with DAP mode - switches via USB re-enumeration
 */

#include "usbd_ftdi.h"
#include "fpga_jtag_mpsse.h"
#include "fpga_jtag_io.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "dap_main.h"
#include <string.h>

/* ========== FT2232H USB Descriptor Definitions ========== */

/* FT2232H compatible VID/PID */
#define FTDI_VID    0x0403
#define FTDI_PID    0x6010

/* Dual-channel: 2 vendor interfaces, each with 2 bulk endpoints */
#define FPGA_CHANNEL_SIZE    (9 + 7 + 7)
#define FPGA_USB_CONFIG_SIZE (9 + FPGA_CHANNEL_SIZE * 2)
#define FPGA_INTF_NUM        2

static const uint8_t fpga_device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, FTDI_VID, FTDI_PID, 0x0500, 0x01),
};

static const uint8_t fpga_config_descriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(FPGA_USB_CONFIG_SIZE, FPGA_INTF_NUM, 0x01,
                               USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    /* Interface 0: Channel A - JTAG/MPSSE (vendor class) */
    USB_INTERFACE_DESCRIPTOR_INIT(0x00, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0x02),
    USB_ENDPOINT_DESCRIPTOR_INIT(FPGA_JTAG_OUT_EP, USB_ENDPOINT_TYPE_BULK, DAP_PACKET_SIZE, 0x00),
    USB_ENDPOINT_DESCRIPTOR_INIT(FPGA_JTAG_IN_EP, USB_ENDPOINT_TYPE_BULK, DAP_PACKET_SIZE, 0x00),
    /* Interface 1: Channel B - UART (vendor class, same as real FT2232H) */
    USB_INTERFACE_DESCRIPTOR_INIT(0x01, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0x02),
    USB_ENDPOINT_DESCRIPTOR_INIT(FTDI_CHB_OUT_EP, USB_ENDPOINT_TYPE_BULK, DAP_PACKET_SIZE, 0x00),
    USB_ENDPOINT_DESCRIPTOR_INIT(FTDI_CHB_IN_EP, USB_ENDPOINT_TYPE_BULK, DAP_PACKET_SIZE, 0x00),
};

static const uint8_t fpga_device_quality_descriptor[] = {
    USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, 0x01),
};

/* FT2232H string descriptors */
static const char *fpga_string_descriptors[] = {
    (char[]){0x09, 0x04},       /* Langid */
    "FTDI",                      /* Manufacturer */
    "FT2232H",                   /* Product */
};

static const uint8_t *fpga_device_descriptor_cb(uint8_t speed)
{
    (void)speed;
    return fpga_device_descriptor;
}

static const uint8_t *fpga_config_descriptor_cb(uint8_t speed)
{
    (void)speed;
    return fpga_config_descriptor;
}

static const uint8_t *fpga_quality_descriptor_cb(uint8_t speed)
{
    (void)speed;
    return fpga_device_quality_descriptor;
}

static const uint8_t *fpga_other_speed_cb(uint8_t speed)
{
    (void)speed;
    return fpga_config_descriptor;
}

static const char *fpga_string_descriptor_cb(uint8_t speed, uint8_t index)
{
    (void)speed;
    if (index == 3) {
        return serial_number_dynamic;
    }
    if (index >= sizeof(fpga_string_descriptors) / sizeof(char *)) {
        return NULL;
    }
    return fpga_string_descriptors[index];
}

static const struct usb_descriptor fpga_usb_descriptor = {
    .device_descriptor_callback = fpga_device_descriptor_cb,
    .config_descriptor_callback = fpga_config_descriptor_cb,
    .device_quality_descriptor_callback = fpga_quality_descriptor_cb,
    .other_speed_descriptor_callback = fpga_other_speed_cb,
    .string_descriptor_callback = fpga_string_descriptor_cb,
    .bos_descriptor = NULL,
    .msosv2_descriptor = NULL,
    .webusb_url_descriptor = NULL,
};

/* ========== Channel A: JTAG/MPSSE ========== */

#define FPGA_JTAG_PACKET_SIZE DAP_PACKET_SIZE
#define FTDI_HEADER_SIZE 2

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t fpga_ep_rx_buf[FPGA_JTAG_PACKET_SIZE];
static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t fpga_tx_buffer[FPGA_JTAG_PACKET_SIZE];
static volatile bool fpga_rx_ready = false;
static volatile bool fpga_tx_idle = true;
static volatile uint32_t fpga_rx_len = 0;

void fpga_jtag_out_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    if (nbytes > 0 && !fpga_rx_ready) {
        fpga_rx_len = nbytes;
        fpga_rx_ready = true;
    } else {
        usbd_ep_start_read(0, FPGA_JTAG_OUT_EP, fpga_ep_rx_buf, FPGA_JTAG_PACKET_SIZE);
    }
}

void fpga_jtag_in_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    fpga_tx_idle = true;
}

/* ========== Channel B: UART Bridge ========== */

static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t ftdi_chb_rx_buf[DAP_PACKET_SIZE];
static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t ftdi_chb_tx_buf[DAP_PACKET_SIZE];
static volatile bool ftdi_chb_tx_idle = true;
static volatile bool ftdi_chb_rx_idle = false;

static void ftdi_chb_out_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    chry_ringbuffer_write(&g_usbrx, ftdi_chb_rx_buf, nbytes);
    if (chry_ringbuffer_get_free(&g_usbrx) >= DAP_PACKET_SIZE) {
        usbd_ep_start_read(0, FTDI_CHB_OUT_EP, ftdi_chb_rx_buf, DAP_PACKET_SIZE);
    } else {
        ftdi_chb_rx_idle = true;
    }
}

static void ftdi_chb_in_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    ftdi_chb_tx_idle = true;
}

/* ========== Endpoint / Interface Structures ========== */

/* Channel A */
static struct usbd_endpoint fpga_out_ep = {
    .ep_addr = FPGA_JTAG_OUT_EP,
    .ep_cb = fpga_jtag_out_callback
};

static struct usbd_endpoint fpga_in_ep = {
    .ep_addr = FPGA_JTAG_IN_EP,
    .ep_cb = fpga_jtag_in_callback
};

static struct usbd_interface fpga_jtag_intf;

/* Channel B */
static struct usbd_endpoint chb_out_ep = {
    .ep_addr = FTDI_CHB_OUT_EP,
    .ep_cb = ftdi_chb_out_callback
};

static struct usbd_endpoint chb_in_ep = {
    .ep_addr = FTDI_CHB_IN_EP,
    .ep_cb = ftdi_chb_in_callback
};

static struct usbd_interface ftdi_chb_intf;

/* ========== FPGA Mode USB Event Handler ========== */

static void fpga_usb_event_handler(uint8_t busid, uint8_t event)
{
    (void)busid;
    switch (event) {
        case USBD_EVENT_RESET:
            fpga_rx_ready = false;
            fpga_tx_idle = true;
            ftdi_chb_tx_idle = true;
            ftdi_chb_rx_idle = false;
            uarttx_idle_flag = 1;
            break;
        case USBD_EVENT_CONFIGURED:
            /* Arm Channel A OUT for JTAG */
            usbd_ep_start_read(0, FPGA_JTAG_OUT_EP, fpga_ep_rx_buf, FPGA_JTAG_PACKET_SIZE);
            /* Arm Channel B OUT for UART */
            usbd_ep_start_read(0, FTDI_CHB_OUT_EP, ftdi_chb_rx_buf, DAP_PACKET_SIZE);
            break;
        default:
            break;
    }
}

/* ========== Public API ========== */

void fpga_jtag_init(void)
{
    fpga_jtag_gpio_init();
    fpga_mpsse_init();
    fpga_rx_ready = false;
    fpga_tx_idle = true;
}

void fpga_usb_init(uint8_t busid, uint32_t reg_base)
{
    usbd_desc_register(busid, &fpga_usb_descriptor);

    /* Channel A: JTAG/MPSSE (vendor handler processes both channels via wIndex) */
    fpga_jtag_intf.vendor_handler = ftdi_vendor_request_handler;
    usbd_add_interface(busid, &fpga_jtag_intf);
    usbd_add_endpoint(busid, &fpga_out_ep);
    usbd_add_endpoint(busid, &fpga_in_ep);

    /* Channel B: UART (no vendor_handler — Channel A handles all vendor requests) */
    usbd_add_interface(busid, &ftdi_chb_intf);
    usbd_add_endpoint(busid, &chb_out_ep);
    usbd_add_endpoint(busid, &chb_in_ep);

    /* Reset UART bridge state */
    ftdi_chb_tx_idle = true;
    ftdi_chb_rx_idle = false;
    uarttx_idle_flag = 1;

    usbd_initialize(busid, reg_base, fpga_usb_event_handler);
}

void fpga_usb_deinit(uint8_t busid)
{
    usbd_deinitialize(busid);
}

void fpga_jtag_process(void)
{
    /* Process received USB data through MPSSE engine */
    if (fpga_rx_ready) {
        fpga_mpsse_feed(fpga_ep_rx_buf, fpga_rx_len);
        fpga_rx_ready = false;

        /* Process all MPSSE commands in this packet */
        while (fpga_mpsse_is_busy()) {
            fpga_mpsse_process();
        }

        /* Ready for next USB packet */
        usbd_ep_start_read(0, FPGA_JTAG_OUT_EP, fpga_ep_rx_buf, FPGA_JTAG_PACKET_SIZE);
    }

    /* Send response data back to host if available */
    if (fpga_tx_idle) {
        uint32_t tx_len = fpga_mpsse_read_tx(fpga_tx_buffer + FTDI_HEADER_SIZE,
                                              FPGA_JTAG_PACKET_SIZE - FTDI_HEADER_SIZE);
        if (tx_len > 0) {
            /* Prepend FTDI modem status header */
            fpga_tx_buffer[0] = 0x01;
            fpga_tx_buffer[1] = 0x60;
            fpga_tx_idle = false;
            usbd_ep_start_write(0, FPGA_JTAG_IN_EP, fpga_tx_buffer, tx_len + FTDI_HEADER_SIZE);
        }
    }
}

void fpga_uart_handle(void)
{
    uint32_t size;
    uint8_t *buffer;

    /* Check if FTDI UART config changed (Channel B SIO_SET_BAUDRATE/SET_DATA) */
    struct cdc_line_coding lc;
    if (ftdi_uart_config_poll(&lc)) {
        chry_dap_usb2uart_uart_config_callback(&lc);
        uarttx_idle_flag = 1;
        ftdi_chb_tx_idle = true;
    }

    /* UART RX → USB TX (Channel B IN with FTDI header) */
    if (ftdi_chb_tx_idle && chry_ringbuffer_get_used(&g_uartrx)) {
        buffer = chry_ringbuffer_linear_read_setup(&g_uartrx, &size);
        if (size > DAP_PACKET_SIZE - FTDI_HEADER_SIZE) {
            size = DAP_PACKET_SIZE - FTDI_HEADER_SIZE;
        }
        ftdi_chb_tx_buf[0] = 0x01; /* modem status */
        ftdi_chb_tx_buf[1] = 0x60; /* line status: THRE + TEMT */
        memcpy(&ftdi_chb_tx_buf[FTDI_HEADER_SIZE], buffer, size);
        chry_ringbuffer_linear_read_done(&g_uartrx, size);
        ftdi_chb_tx_idle = false;
        usbd_ep_start_write(0, FTDI_CHB_IN_EP, ftdi_chb_tx_buf, size + FTDI_HEADER_SIZE);
    }

    /* USB RX → UART TX (Channel B OUT → DMA) */
    if (uarttx_idle_flag && chry_ringbuffer_get_used(&g_usbrx)) {
        uarttx_idle_flag = 0;
        buffer = chry_ringbuffer_linear_read_setup(&g_usbrx, &size);
        chry_dap_usb2uart_uart_send_bydma(buffer, size);
    }

    /* Re-arm Channel B OUT if ring buffer has space */
    if (ftdi_chb_rx_idle && chry_ringbuffer_get_free(&g_usbrx) >= DAP_PACKET_SIZE) {
        ftdi_chb_rx_idle = false;
        usbd_ep_start_read(0, FTDI_CHB_OUT_EP, ftdi_chb_rx_buf, DAP_PACKET_SIZE);
    }
}
