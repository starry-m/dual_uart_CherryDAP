/*
 * FPGA JTAG I/O Pin Configuration for HSLink-Pro
 * Target: HPM5301
 * FPGA JTAG Pins: PA04(TMS), PA05(TCK), PA06(TDO), PA07(TDI)
 */

#ifndef FPGA_JTAG_IO_H
#define FPGA_JTAG_IO_H

#include <stdint.h>
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
//#include "hpm_ioc_drv.h"
#include "hpm_soc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* FPGA JTAG Pin Definitions */
#ifndef FPGA_PIN_TMS
#define FPGA_PIN_TMS    IOC_PAD_PA04
#endif

#ifndef FPGA_PIN_TCK
#define FPGA_PIN_TCK    IOC_PAD_PA05
#endif

#ifndef FPGA_PIN_TDO
#define FPGA_PIN_TDO    IOC_PAD_PA06
#endif

#ifndef FPGA_PIN_TDI
#define FPGA_PIN_TDI    IOC_PAD_PA07
#endif

#define FPGA_GPIO       HPM_FGPIO
#define FPGA_GPIOM      gpiom_core0_fast

/* Initialize FPGA JTAG GPIO pins */
static inline void fpga_jtag_gpio_init(void)
{
    /* Configure all JTAG pins as GPIO function */
    HPM_IOC->PAD[FPGA_PIN_TMS].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0);
    HPM_IOC->PAD[FPGA_PIN_TCK].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0);
    HPM_IOC->PAD[FPGA_PIN_TDI].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0);
    HPM_IOC->PAD[FPGA_PIN_TDO].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0);

    /* Set pad control: medium drive strength */
    HPM_IOC->PAD[FPGA_PIN_TMS].PAD_CTL = IOC_PAD_PAD_CTL_PRS_SET(2);
    HPM_IOC->PAD[FPGA_PIN_TCK].PAD_CTL = IOC_PAD_PAD_CTL_PRS_SET(2);
    HPM_IOC->PAD[FPGA_PIN_TDI].PAD_CTL = IOC_PAD_PAD_CTL_PRS_SET(2);
    /* TDO: enable pull-up to avoid floating when FPGA tri-states */
    HPM_IOC->PAD[FPGA_PIN_TDO].PAD_CTL = IOC_PAD_PAD_CTL_PRS_SET(2) |
                                           IOC_PAD_PAD_CTL_PE_SET(1) |
                                           IOC_PAD_PAD_CTL_PS_SET(1);

    /* Configure GPIOM for fast GPIO access */
    gpiom_set_pin_controller(HPM_GPIOM, GPIO_GET_PORT_INDEX(FPGA_PIN_TMS), GPIO_GET_PIN_INDEX(FPGA_PIN_TMS), FPGA_GPIOM);
    gpiom_set_pin_controller(HPM_GPIOM, GPIO_GET_PORT_INDEX(FPGA_PIN_TCK), GPIO_GET_PIN_INDEX(FPGA_PIN_TCK), FPGA_GPIOM);
    gpiom_set_pin_controller(HPM_GPIOM, GPIO_GET_PORT_INDEX(FPGA_PIN_TDI), GPIO_GET_PIN_INDEX(FPGA_PIN_TDI), FPGA_GPIOM);
    gpiom_set_pin_controller(HPM_GPIOM, GPIO_GET_PORT_INDEX(FPGA_PIN_TDO), GPIO_GET_PIN_INDEX(FPGA_PIN_TDO), FPGA_GPIOM);

    /* TMS, TCK, TDI as output; TDO as input */
    gpio_set_pin_output(FPGA_GPIO, GPIO_GET_PORT_INDEX(FPGA_PIN_TMS), GPIO_GET_PIN_INDEX(FPGA_PIN_TMS));
    gpio_set_pin_output(FPGA_GPIO, GPIO_GET_PORT_INDEX(FPGA_PIN_TCK), GPIO_GET_PIN_INDEX(FPGA_PIN_TCK));
    gpio_set_pin_output(FPGA_GPIO, GPIO_GET_PORT_INDEX(FPGA_PIN_TDI), GPIO_GET_PIN_INDEX(FPGA_PIN_TDI));
    gpio_set_pin_input(FPGA_GPIO, GPIO_GET_PORT_INDEX(FPGA_PIN_TDO), GPIO_GET_PIN_INDEX(FPGA_PIN_TDO));

    /* Set initial state: TMS low, TCK low, TDI low */
    gpio_write_pin(FPGA_GPIO, GPIO_GET_PORT_INDEX(FPGA_PIN_TMS), GPIO_GET_PIN_INDEX(FPGA_PIN_TMS), 0);
    gpio_write_pin(FPGA_GPIO, GPIO_GET_PORT_INDEX(FPGA_PIN_TCK), GPIO_GET_PIN_INDEX(FPGA_PIN_TCK), 0);
    gpio_write_pin(FPGA_GPIO, GPIO_GET_PORT_INDEX(FPGA_PIN_TDI), GPIO_GET_PIN_INDEX(FPGA_PIN_TDI), 0);
}

/* Delay for JTAG clock timing, same as PIN_DELAY_SLOW(DAP_Data.clock_delay) */
#define FPGA_JTAG_DELAY()                          \
    do {                                            \
        __asm volatile("fence io, io");             \
        volatile uint32_t _cnt = 5; \
        while (_cnt--) { __asm volatile(""); }      \
    } while(0)

/* Fast pin access macros (with fence io, io for RISC-V I/O ordering) */
#define FPGA_TMS_HIGH()   do { gpio_write_pin(FPGA_GPIO, GPIO_GET_PORT_INDEX(FPGA_PIN_TMS), GPIO_GET_PIN_INDEX(FPGA_PIN_TMS), 1); __asm volatile("fence io, io"); } while(0)
#define FPGA_TMS_LOW()    do { gpio_write_pin(FPGA_GPIO, GPIO_GET_PORT_INDEX(FPGA_PIN_TMS), GPIO_GET_PIN_INDEX(FPGA_PIN_TMS), 0); __asm volatile("fence io, io"); } while(0)
#define FPGA_TCK_HIGH()   do { gpio_write_pin(FPGA_GPIO, GPIO_GET_PORT_INDEX(FPGA_PIN_TCK), GPIO_GET_PIN_INDEX(FPGA_PIN_TCK), 1); __asm volatile("fence io, io"); } while(0)
#define FPGA_TCK_LOW()    do { gpio_write_pin(FPGA_GPIO, GPIO_GET_PORT_INDEX(FPGA_PIN_TCK), GPIO_GET_PIN_INDEX(FPGA_PIN_TCK), 0); __asm volatile("fence io, io"); } while(0)
#define FPGA_TDI_HIGH()   do { gpio_write_pin(FPGA_GPIO, GPIO_GET_PORT_INDEX(FPGA_PIN_TDI), GPIO_GET_PIN_INDEX(FPGA_PIN_TDI), 1); __asm volatile("fence io, io"); } while(0)
#define FPGA_TDI_LOW()    do { gpio_write_pin(FPGA_GPIO, GPIO_GET_PORT_INDEX(FPGA_PIN_TDI), GPIO_GET_PIN_INDEX(FPGA_PIN_TDI), 0); __asm volatile("fence io, io"); } while(0)
#define FPGA_TDO_READ()   ({ uint32_t _sta = gpio_read_pin(FPGA_GPIO, GPIO_GET_PORT_INDEX(FPGA_PIN_TDO), GPIO_GET_PIN_INDEX(FPGA_PIN_TDO)); __asm volatile("fence io, io"); _sta; })

#ifdef __cplusplus
}
#endif

#endif /* FPGA_JTAG_IO_H */
