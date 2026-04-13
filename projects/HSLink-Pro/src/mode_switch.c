/*
 * Mode Switch Implementation
 * Button on PA30 (MODE_SWITCH), pull-down, active high
 * Short press toggles between DAP and FT2232 FPGA JTAG mode
 */

#include "mode_switch.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
//#include "hpm_ioc_drv.h"
#include "board.h"
#include "led_extern.h"
#include <stdio.h>

#ifndef MODE_SWITCH_PIN
#define MODE_SWITCH_PIN IOC_PAD_PA30
#endif

#define MODE_SW_GPIO       HPM_FGPIO
#define MODE_SW_GPIOM      gpiom_core0_fast

/* Debounce parameters */
#define DEBOUNCE_MS        50
#define LONG_PRESS_MS      1000  /* Reserved for future use */

static WorkMode_t current_mode = MODE_DAP;
static volatile bool mode_changed_flag = false;

/* Button state for debounce */
static bool last_button_state = false;
static uint64_t last_change_time = 0;
static bool button_stable = false;
static bool press_handled = false;

static void mode_switch_gpio_init(void)
{
    /* Configure PA30 as GPIO input with pull-down */
    HPM_IOC->PAD[MODE_SWITCH_PIN].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0);
    HPM_IOC->PAD[MODE_SWITCH_PIN].PAD_CTL = IOC_PAD_PAD_CTL_PRS_SET(2) |
                                              IOC_PAD_PAD_CTL_PE_SET(1) |
                                              IOC_PAD_PAD_CTL_PS_SET(0); /* Pull-down */

    gpiom_set_pin_controller(HPM_GPIOM,
                             GPIO_GET_PORT_INDEX(MODE_SWITCH_PIN),
                             GPIO_GET_PIN_INDEX(MODE_SWITCH_PIN),
                             MODE_SW_GPIOM);

    gpio_set_pin_input(MODE_SW_GPIO,
                       GPIO_GET_PORT_INDEX(MODE_SWITCH_PIN),
                       GPIO_GET_PIN_INDEX(MODE_SWITCH_PIN));
}

static inline bool read_button(void)
{
    return gpio_read_pin(MODE_SW_GPIO,
                         GPIO_GET_PORT_INDEX(MODE_SWITCH_PIN),
                         GPIO_GET_PIN_INDEX(MODE_SWITCH_PIN)) != 0;
}

void mode_switch_init(void)
{
    mode_switch_gpio_init();
    current_mode = MODE_DAP;
    mode_changed_flag = false;
    last_button_state = false;
    button_stable = false;
    press_handled = false;
    last_change_time = millis();
    printf("Mode switch: DAP mode (default)\n");
}

void mode_switch_poll(void)
{
    bool raw = read_button();
    uint64_t now = millis();

    /* Detect state change for debounce */
    if (raw != last_button_state) {
        last_button_state = raw;
        last_change_time = now;
        return;
    }

    /* Wait for debounce period */
    if ((now - last_change_time) < DEBOUNCE_MS) {
        return;
    }

    /* Stable state determined */
    bool pressed = raw; /* Active high */

    if (pressed && !press_handled) {
        /* Button just pressed (rising edge after debounce) */
        press_handled = true;

        /* Toggle mode */
        if (current_mode == MODE_DAP) {
            current_mode = MODE_FPGA_JTAG;
            printf("Mode switch: FT2232 FPGA JTAG mode\n");
        } else {
            current_mode = MODE_DAP;
            printf("Mode switch: DAP mode\n");
        }
        mode_changed_flag = true;
    } else if (!pressed) {
        /* Button released */
        press_handled = false;
    }
}

WorkMode_t mode_switch_get(void)
{
    return current_mode;
}

bool mode_switch_changed(void)
{
    if (mode_changed_flag) {
        mode_changed_flag = false;
        return true;
    }
    return false;
}
