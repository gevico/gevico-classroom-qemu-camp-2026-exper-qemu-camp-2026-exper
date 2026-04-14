#ifndef HW_GPIO_G233_GPIOCTRL_H
#define HW_GPIO_G233_GPIOCTRL_H

/*
 * Minimal educational GPIO controller skeleton for the G233 board.
 *
 * Scope of this device:
 * - expose one MMIO page to the guest CPU
 * - expose one aggregated IRQ output to the board IRQ chip/PLIC
 * - expose 32 per-pin GPIO outputs to the rest of the board
 * - accept 32 per-pin GPIO inputs from the rest of the board
 *
 * Intentionally out of scope for the first step:
 * - full pinctrl/pinmux modelling
 * - pad electrical features
 * - board-specific consumers such as keys, LEDs, headers
 *
 * The board model is responsible for:
 * 1. mapping this device into the system address space
 * 2. wiring its aggregated IRQ to the chosen PLIC source
 * 3. connecting pin-level GPIO lines to other devices if needed
 */

#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_G233_GPIOCTRL "g233-gpioctrl"
OBJECT_DECLARE_SIMPLE_TYPE(G233GPIOCtrlState, G233_GPIOCTRL)

#define G233_GPIOCTRL_NUM_PINS      32
#define G233_GPIOCTRL_MMIO_SIZE     0x1000

/* Guest-visible register offsets */
#define G233_GPIO_REG_DIR           0x00
#define G233_GPIO_REG_OUT           0x04
#define G233_GPIO_REG_IN            0x08
#define G233_GPIO_REG_IE            0x0c
#define G233_GPIO_REG_IS            0x10
#define G233_GPIO_REG_TRIG          0x14
#define G233_GPIO_REG_POL           0x18

typedef struct G233GPIOCtrlState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    /*
     * Aggregated interrupt line towards the board interrupt controller.
     * GPIO_IS bits are OR-ed, then filtered by GPIO_IE, and finally exposed
     * on this single wire.
     */
    qemu_irq irq;

    /*
     * Per-pin external wiring.
     * - pin_out[n]: level driven out when GPIO_DIR[n] = 1
     * - external_inputs: current levels sampled from board-side producers
     */
    qemu_irq pin_out[G233_GPIOCTRL_NUM_PINS];

    /* Register bank defined by the teaching document. */
    uint32_t dir;
    uint32_t out;
    uint32_t ie;
    uint32_t is;
    uint32_t trig;
    uint32_t pol;

    /*
     * Internal line-state bookkeeping.
     *
     * visible_inputs:
     *   current raw external input levels presented by the rest of the board
     *
     * last_pin_level:
     *   full pin snapshot after direction muxing; useful for later edge
     *   detection logic
     *
     * last_output_level:
     *   cached driven outputs, so only changed pins call qemu_set_irq()
     */
    uint32_t visible_inputs; // input
    uint32_t last_pin_level;
    uint32_t last_output_level;
} G233GPIOCtrlState;

#endif /* HW_GPIO_G233_GPIOCTRL_H */
