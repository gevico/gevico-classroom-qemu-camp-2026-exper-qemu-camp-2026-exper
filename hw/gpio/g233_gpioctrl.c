#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "migration/vmstate.h"
#include "hw/gpio/g233_gpioctrl.h"

/*
 * Return the logical level currently seen on each GPIO pin.
 *
 * Output pins read back the output latch.
 * Input pins sample the last externally provided level.
 */
static uint32_t g233_gpioctrl_get_pin_level(G233GPIOCtrlState *s)
{
    uint32_t output_level = s->out & s->dir;
    uint32_t input_level = s->visible_inputs & ~s->dir;

    return output_level | input_level;
}

/*
 * Drive board-facing GPIO wires for pins currently configured as output.
 *
 * This is intentionally separate from interrupt evaluation because output
 * routing and interrupt semantics evolve independently while the model is
 * being built out.
 */
static void g233_gpioctrl_update_outputs(G233GPIOCtrlState *s)
{
    uint32_t new_output_level = s->out & s->dir;
    uint32_t changed = s->last_output_level ^ new_output_level;
    int i;

    s->last_output_level = new_output_level;

    for (i = 0; i < G233_GPIOCTRL_NUM_PINS; i++) {
        uint32_t mask = 1u << i;

        if (changed & mask) {
            qemu_set_irq(s->pin_out[i], (new_output_level & mask) != 0);
        }
    }
}

/* Recompute the single IRQ line exported to the board interrupt controller. */
static void g233_gpioctrl_update_irq(G233GPIOCtrlState *s)
{
    qemu_set_irq(s->irq, (s->is & s->ie) != 0);
}

/*
 * Recompute GPIO_IS from the previous and current logical pin levels.
 *
 * This function intentionally leaves the policy unimplemented so the user can
 * fill in the trigger semantics step by step. The surrounding framework is
 * already complete: MMIO, board wiring, pin muxing through DIR, W1C on IS, and
 * aggregated IRQ fan-in all work.
 *
 * Existing qtests define the expected contract:
 * - TRIG=0, POL=1: rising edge
 * - TRIG=0, POL=0: falling edge
 * - TRIG=1, POL=1: high level
 * - TRIG=1, POL=0: low level
 * - new pending bits are only latched when IE says the pin is enabled
 * - the tests drive GPIO_OUT on output pins, so use the post-DIR logical
 *   pin level rather than only raw external inputs
 */
static void g233_gpioctrl_update_pending(G233GPIOCtrlState *s,
                                         uint32_t old_pin_level,
                                         uint32_t new_pin_level)
{
    int i;

    for (i = 0; i < G233_GPIOCTRL_NUM_PINS; i++) {
        uint32_t mask = 1u << i;
        bool old_level = (old_pin_level & mask) != 0;
        bool new_level = (new_pin_level & mask) != 0;
        bool is_level = (s->trig & mask) != 0; // edge tarigger should be false.
        bool is_high_or_rising = (s->pol & mask) != 0; // hign level should be true.

        /*
         * TODO(user):
         * Decide whether this pin should assert/clear GPIO_IS here.
         * Use old_level/new_level/is_level/is_high_or_rising plus IE.
         */

        bool high_edge_come = !old_level && new_level;
        bool low_edge_come = old_level && !new_level;
        bool interrupt_triggers = (is_level && is_high_or_rising && new_level) 
        || (is_level && !is_high_or_rising && !new_level) 
        || (!is_level && is_high_or_rising && high_edge_come) 
        || (!is_level && !is_high_or_rising && low_edge_come);

        if((s->ie & mask) && interrupt_triggers) {
            s->is |= mask;
        }else if(is_level){
            s->is &= ~mask; /* clear existing bits */
        }
        


    }
}

/*
 * Central state update hook.
 *
 * The state machine is split into three steps:
 * 1. derive the logical pin level after DIR muxing
 * 2. update pending bits in GPIO_IS
 * 3. drive outputs and refresh the aggregated IRQ wire
 *   for this teaching model
 */
static void g233_gpioctrl_update_state(G233GPIOCtrlState *s)
{
    uint32_t old_pin_level = s->last_pin_level;
    uint32_t new_pin_level = g233_gpioctrl_get_pin_level(s);


    g233_gpioctrl_update_pending(s, old_pin_level, new_pin_level);
    s->last_pin_level = new_pin_level;

    g233_gpioctrl_update_outputs(s);
    /* send interrupt */
    g233_gpioctrl_update_irq(s);
}

/*
 * Board-side input callback.
 *
 * Other devices can drive GPIO input pins through qdev GPIO connections.
 * Only pins configured as input consume this value; output pins read back
 * the GPIO_OUT latch through g233_gpioctrl_get_pin_level().
 */
static void g233_gpioctrl_set_input(void *opaque, int pin, int level)
{
    G233GPIOCtrlState *s = opaque;
    uint32_t mask;

    if (pin < 0 || pin >= G233_GPIOCTRL_NUM_PINS) {
        return;
    }

    /* outer chip change the value */
    mask = 1u << pin;
    if((mask & s->dir) != 0) {
        s->visible_inputs &= ~mask;
        if (level)
        {
            s->visible_inputs |= mask;
        }
        return;
    }

    g233_gpioctrl_update_state(s);
}

static uint64_t g233_gpioctrl_read(void *opaque, hwaddr offset, unsigned size)
{
    G233GPIOCtrlState *s = opaque;

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unsupported read size %u at 0x%" HWADDR_PRIx "\n",
                      TYPE_G233_GPIOCTRL, size, offset);
        return 0;
    }

    switch (offset) {
    case G233_GPIO_REG_DIR:
        return s->dir;
    case G233_GPIO_REG_OUT:
        return s->out;
    case G233_GPIO_REG_IN:
        return g233_gpioctrl_get_pin_level(s);
    case G233_GPIO_REG_IE:
        return s->ie;
    case G233_GPIO_REG_IS:
        return s->is;
    case G233_GPIO_REG_TRIG:
        return s->trig;
    case G233_GPIO_REG_POL:
        return s->pol;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                      TYPE_G233_GPIOCTRL, offset);
        return 0;
    }
}

static void g233_gpioctrl_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    G233GPIOCtrlState *s = opaque;
    uint32_t data = value;

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unsupported write size %u at 0x%" HWADDR_PRIx "\n",
                      TYPE_G233_GPIOCTRL, size, offset);
        return;
    }

    switch (offset) {
    case G233_GPIO_REG_DIR:
        s->dir = data;
        break;
    case G233_GPIO_REG_OUT:
        s->out = data;
        break;
    case G233_GPIO_REG_IE:
        s->ie = data;
        break;
    case G233_GPIO_REG_IS:
        /* Write-1-to-clear, as required by the teaching document. */
        s->is &= ~data;
        break;
    case G233_GPIO_REG_TRIG:
        s->trig = data;
        break;
    case G233_GPIO_REG_POL:
        s->pol = data;
        break;
    case G233_GPIO_REG_IN:
        /*
         * Read-only register from the guest point of view.
         * Ignore writes, but keep the access visible in guest error logs.
         */
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write to read-only GPIO_IN ignored\n",
                      TYPE_G233_GPIOCTRL);
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write offset 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n",
                      TYPE_G233_GPIOCTRL, offset, value);
        return;
    }

    g233_gpioctrl_update_state(s);
}

static const MemoryRegionOps g233_gpioctrl_ops = {
    .read = g233_gpioctrl_read,
    .write = g233_gpioctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void g233_gpioctrl_reset(DeviceState *dev)
{
    G233GPIOCtrlState *s = G233_GPIOCTRL(dev);

    s->dir = 0;
    s->out = 0;
    s->ie = 0;
    s->is = 0;
    s->trig = 0;
    s->pol = 0;
    s->visible_inputs = 0;
    s->last_pin_level = 0;
    s->last_output_level = 0;

    g233_gpioctrl_update_state(s);
}

static void g233_gpioctrl_init(Object *obj)
{
    G233GPIOCtrlState *s = G233_GPIOCTRL(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &g233_gpioctrl_ops, s,
                          TYPE_G233_GPIOCTRL, G233_GPIOCTRL_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    qdev_init_gpio_in(dev, g233_gpioctrl_set_input, G233_GPIOCTRL_NUM_PINS);
    qdev_init_gpio_out(dev, s->pin_out, G233_GPIOCTRL_NUM_PINS);
}

static const VMStateDescription vmstate_g233_gpioctrl = {
    .name = TYPE_G233_GPIOCTRL,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(dir, G233GPIOCtrlState),
        VMSTATE_UINT32(out, G233GPIOCtrlState),
        VMSTATE_UINT32(ie, G233GPIOCtrlState),
        VMSTATE_UINT32(is, G233GPIOCtrlState),
        VMSTATE_UINT32(trig, G233GPIOCtrlState),
        VMSTATE_UINT32(pol, G233GPIOCtrlState),
        VMSTATE_UINT32(visible_inputs, G233GPIOCtrlState),
        VMSTATE_UINT32(last_pin_level, G233GPIOCtrlState),
        VMSTATE_UINT32(last_output_level, G233GPIOCtrlState),
        VMSTATE_END_OF_LIST()
    },
};

static void g233_gpioctrl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_g233_gpioctrl;
    device_class_set_legacy_reset(dc, g233_gpioctrl_reset);
}

static const TypeInfo g233_gpioctrl_info = {
    .name = TYPE_G233_GPIOCTRL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(G233GPIOCtrlState),
    .instance_init = g233_gpioctrl_init,
    .class_init = g233_gpioctrl_class_init,
};

static void g233_gpioctrl_register_types(void)
{
    type_register_static(&g233_gpioctrl_info);
}

type_init(g233_gpioctrl_register_types)
