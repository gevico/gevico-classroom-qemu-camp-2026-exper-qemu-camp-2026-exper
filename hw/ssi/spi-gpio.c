/*
 * GEVICO SPI GPIO controller with Rust backend
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "qemu/module.h"

#define TYPE_SPI_GPIO "spi-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(SPIGpioState, SPI_GPIO)

typedef struct SPIGpioState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    void *rust_state;
} SPIGpioState;

extern void *spi_gpio_rust_new(DeviceState *parent);
extern void spi_gpio_rust_free(void *state);
extern void spi_gpio_rust_reset(void *state);
extern uint32_t spi_gpio_rust_read(void *state, uint64_t offset);
extern void spi_gpio_rust_write(void *state, uint64_t offset, uint32_t value);

static uint64_t spi_gpio_read(void *opaque, hwaddr addr, unsigned size)
{
    SPIGpioState *s = opaque;

    if (size != 4) {
        return 0;
    }

    return spi_gpio_rust_read(s->rust_state, addr);
}

static void spi_gpio_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    SPIGpioState *s = opaque;

    if (size != 4) {
        return;
    }

    spi_gpio_rust_write(s->rust_state, addr, (uint32_t)val);
}

static const MemoryRegionOps spi_gpio_ops = {
    .read = spi_gpio_read,
    .write = spi_gpio_write,
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

static void spi_gpio_reset_hold(Object *obj, ResetType type)
{
    SPIGpioState *s = SPI_GPIO(obj);

    spi_gpio_rust_reset(s->rust_state);
}

static void spi_gpio_realize(DeviceState *dev, Error **errp)
{
    SPIGpioState *s = SPI_GPIO(dev);

    s->rust_state = spi_gpio_rust_new(dev);
    if (!s->rust_state) {
        error_setg(errp, "failed to initialize Rust spi-gpio state");
    }
}

static void spi_gpio_finalize(Object *obj)
{
    SPIGpioState *s = SPI_GPIO(obj);

    spi_gpio_rust_free(s->rust_state);
    s->rust_state = NULL;
}

static void spi_gpio_init(Object *obj)
{
    SPIGpioState *s = SPI_GPIO(obj);

    memory_region_init_io(&s->iomem, obj, &spi_gpio_ops, s, TYPE_SPI_GPIO, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void spi_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = spi_gpio_realize;
    rc->phases.hold = spi_gpio_reset_hold;
}

static const TypeInfo spi_gpio_info = {
    .name = TYPE_SPI_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SPIGpioState),
    .instance_init = spi_gpio_init,
    .instance_finalize = spi_gpio_finalize,
    .class_init = spi_gpio_class_init,
};

static void spi_gpio_register_types(void)
{
    type_register_static(&spi_gpio_info);
}

type_init(spi_gpio_register_types)
