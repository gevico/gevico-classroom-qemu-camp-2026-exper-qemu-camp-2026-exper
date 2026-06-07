/*
 * GEVICO I2C GPIO controller with Rust backend
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "qemu/module.h"

#define TYPE_I2C_GPIO "i2c-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(I2CGpioState, I2C_GPIO)

typedef struct I2CGpioState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    void *rust_state;
} I2CGpioState;

extern void *i2c_gpio_rust_new(DeviceState *parent);
extern void i2c_gpio_rust_free(void *state);
extern void i2c_gpio_rust_reset(void *state);
extern uint32_t i2c_gpio_rust_read(void *state, uint64_t offset);
extern void i2c_gpio_rust_write(void *state, uint64_t offset, uint32_t value);

static uint64_t i2c_gpio_read(void *opaque, hwaddr addr, unsigned size)
{
    I2CGpioState *s = opaque;

    if (size != 4) {
        return 0;
    }

    return i2c_gpio_rust_read(s->rust_state, addr);
}

static void i2c_gpio_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    I2CGpioState *s = opaque;

    if (size != 4) {
        return;
    }

    i2c_gpio_rust_write(s->rust_state, addr, (uint32_t)val);
}

static const MemoryRegionOps i2c_gpio_ops = {
    .read = i2c_gpio_read,
    .write = i2c_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void i2c_gpio_reset_hold(Object *obj, ResetType type)
{
    I2CGpioState *s = I2C_GPIO(obj);

    i2c_gpio_rust_reset(s->rust_state);
}

static void i2c_gpio_realize(DeviceState *dev, Error **errp)
{
    I2CGpioState *s = I2C_GPIO(dev);

    s->rust_state = i2c_gpio_rust_new(dev);
    if (!s->rust_state) {
        error_setg(errp, "failed to initialize Rust i2c-gpio state");
    }
}

static void i2c_gpio_finalize(Object *obj)
{
    I2CGpioState *s = I2C_GPIO(obj);

    i2c_gpio_rust_free(s->rust_state);
    s->rust_state = NULL;
}

static void i2c_gpio_init(Object *obj)
{
    I2CGpioState *s = I2C_GPIO(obj);

    memory_region_init_io(&s->iomem, obj, &i2c_gpio_ops, s, TYPE_I2C_GPIO, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void i2c_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = i2c_gpio_realize;
    rc->phases.hold = i2c_gpio_reset_hold;
}

static const TypeInfo i2c_gpio_info = {
    .name = TYPE_I2C_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(I2CGpioState),
    .instance_init = i2c_gpio_init,
    .instance_finalize = i2c_gpio_finalize,
    .class_init = i2c_gpio_class_init,
};

static void i2c_gpio_register_types(void)
{
    type_register_static(&i2c_gpio_info);
}

type_init(i2c_gpio_register_types)