#include "HSLink_Pro_expansion.h"
#include "LED.h"
#include "board.h"
#include "dap_main.h"
#include "setting.h"
#include "usb2uart.h"
#include <elog.h>
#include <hid_comm.h>
#include <hpm_dma_mgr.h>
#include <hpm_ewdg_drv.h>
#include <hpm_gpio_drv.h>
#include <hpm_gpiom_drv.h>
#include <hpm_romapi.h>
#include "MultiTimer.h"

#if CONFIG_CHERRYDAP_USE_FPGA_JTAG
#include "FPGA_JTAG/usbd_ftdi.h"
#include "mode_switch.h"
#endif

static void serial_number_init(void) {
#define OTP_CHIP_UUID_IDX_START (88U)
#define OTP_CHIP_UUID_IDX_END   (91U)
    uint32_t uuid_words[4];

    uint32_t word_idx = 0;
    for (uint32_t i = OTP_CHIP_UUID_IDX_START; i <= OTP_CHIP_UUID_IDX_END; i++) {
        uuid_words[word_idx++] = ROM_API_TABLE_ROOT->otp_driver_if->read_from_shadow(i);
    }

    sprintf(serial_number_dynamic, "%08X%08X%08X%08X", uuid_words[0], uuid_words[1], uuid_words[2], uuid_words[3]);
    printf("Serial number: %s\n", serial_number_dynamic);
}

static void EWDG_Init() {
    clock_add_to_group(clock_watchdog0, 0);
    ewdg_config_t config;
    ewdg_get_default_config(HPM_EWDG0, &config);

    config.enable_watchdog = true;
    config.int_rst_config.enable_timeout_reset = true;
    config.ctrl_config.use_lowlevel_timeout = false;
    config.ctrl_config.cnt_clk_sel = ewdg_cnt_clk_src_ext_osc_clk;

    /* Set the EWDG reset timeout to 5 second */
    config.cnt_src_freq = 32768;
    config.ctrl_config.timeout_reset_us = 5 * 1000 * 1000;

    /* Initialize the WDG */
    hpm_stat_t status = ewdg_init(HPM_EWDG0, &config);
    if (status != status_success) {
        printf(" EWDG initialization failed, error_code=%d\n", status);
    }
}

[[noreturn]] // make compiler happy
int main() {
    board_init();
    EWDG_Init();
    board_delay_ms(500);
    serial_number_init();
    board_init_usb(HPM_USB0);
    dma_mgr_init();

    Setting_Init();

    multiTimerInstall(millis);  // warning: timer cb all called in isr, and timer gap should align to 5ms
    board_timer_create(5, []() {multiTimerYield();});

    HSP_Init();
    intc_set_irq_priority(CONFIG_HPM_USBD_IRQn, 5);
    uartx_preinit();

#if CONFIG_CHERRYDAP_USE_FPGA_JTAG
    fpga_jtag_init();
    mode_switch_init();
#endif

    /* Default: start in DAP mode */
    chry_dap_init(0, HPM_USB0_BASE);
    board_init_led_pins();
#if CONFIG_CHERRYDAP_USE_FPGA_JTAG
    /* DAP mode: green LED on, blue LED off */
    board_led_write(0, BOARD_LED_ON_LEVEL);
    board_led_write(1, BOARD_LED_OFF_LEVEL);
#endif

    // led.SetNeoPixel(neopixel);

    bl_setting.app_version.major = CONFIG_BUILD_VERSION_MAJOR;
    bl_setting.app_version.minor = CONFIG_BUILD_VERSION_MINOR;
    bl_setting.app_version.patch = CONFIG_BUILD_VERSION_PATCH;

    bl_setting.fail_cnt = 0;

    while (true) {
        ewdg_refresh(HPM_EWDG0);

#if CONFIG_CHERRYDAP_USE_FPGA_JTAG
        mode_switch_poll();

        if (mode_switch_changed()) {
            if (mode_switch_get() == MODE_FPGA_JTAG) {
                chry_dap_deinit(0);
                fpga_jtag_init();  /* Re-init GPIO + MPSSE + self-test */
                fpga_usb_init(0, HPM_USB0_BASE);
                /* FPGA JTAG mode: blue LED on, green LED off */
                board_led_write(0, BOARD_LED_OFF_LEVEL);
                board_led_write(1, BOARD_LED_ON_LEVEL);
            } else {
                fpga_usb_deinit(0);
                chry_dap_init(0, HPM_USB0_BASE);
                /* DAP mode: green LED on, blue LED off */
                board_led_write(0, BOARD_LED_ON_LEVEL);
                board_led_write(1, BOARD_LED_OFF_LEVEL);
            }
        }

        if (mode_switch_get() == MODE_DAP) {
            chry_dap_handle();
            chry_dap_usb2uart_handle();
#if CONFIG_CHERRYDAP_USE_CUSTOM_HID
            HID_Handle();
#endif
        } else {
            fpga_jtag_process();
            fpga_uart_handle();
        }
#else
        chry_dap_handle();
        chry_dap_usb2uart_handle();
#endif

        usb2uart_handler();
        // HSP_Loop();
        // led.Handle();
    }
}
