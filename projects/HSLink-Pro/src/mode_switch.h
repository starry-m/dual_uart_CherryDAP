/*
 * Mode Switch - Toggle between DAP mode and FT2232 FPGA JTAG mode
 * Button on PA30, pull-down, active high (pressed = high)
 */

#ifndef MODE_SWITCH_H
#define MODE_SWITCH_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MODE_DAP = 0,       /* CMSIS-DAP mode for MCU debug */
    MODE_FPGA_JTAG,     /* FT2232 MPSSE mode for FPGA JTAG */
} WorkMode_t;

/* Initialize mode switch button GPIO and state */
void mode_switch_init(void);

/* Poll button state and handle debounce - call in main loop */
void mode_switch_poll(void);

/* Get current working mode */
WorkMode_t mode_switch_get(void);

/* Check if mode just changed (cleared after read) */
bool mode_switch_changed(void);

#ifdef __cplusplus
}
#endif

#endif /* MODE_SWITCH_H */
