#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/spicontroller/spicontroller.h"

static void g233_spicontroller_update_irq(G233SPIControllerState *s)
{
    bool level = false;

    if ((s->cr1 & G233_SPI_CR1_ERRIE) && (s->sr & G233_SPI_SR_OVERRUN)) {
        level = true;
    }
    if ((s->cr1 & G233_SPI_CR1_RXNEIE) && (s->sr & G233_SPI_SR_RXNE)) {
        level = true;
    }
    if ((s->cr1 & G233_SPI_CR1_TXEIE) && (s->sr & G233_SPI_SR_TXE)) {
        level = true;
    }
    qemu_set_irq(s->irq, level);
}

static void g233_spicontroller_drive_cs(G233SPIControllerState *s)
{
    uint32_t i;

    if (!s->cs_lines) {
        return;
    }

    /*
     * SSI peripherals usually default to active-low CS.
     * Selected device sees level 0, all others see level 1.
     */
    for (i = 0; i < s->num_cs; i++) {
        qemu_set_irq(s->cs_lines[i], i == s->active_cs ? 0 : 1);
    }
}

static void g233_spicontroller_transfer(G233SPIControllerState *s, uint8_t tx)
{
    uint8_t rx = 0xff;

    if (!(s->cr1 & G233_SPI_CR1_SPE)) {
        return;
    }

    s->last_tx = tx;

    /*
     * Minimal overrun rule for the current educational model:
     * if the guest has not drained the previous byte yet, a new completed
     * transfer sets OVERRUN.
     */
    if (s->sr & G233_SPI_SR_RXNE) {
        s->sr |= G233_SPI_SR_OVERRUN;
    }

    if (s->spi) {
        rx = ssi_transfer(s->spi, tx) & 0xff;
    }

    s->last_rx = rx;
    s->dr = rx;
    s->sr |= G233_SPI_SR_RXNE | G233_SPI_SR_TXE;
    g233_spicontroller_update_irq(s);
}

static uint64_t g233_spicontroller_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    G233SPIControllerState *s = opaque;
    uint32_t value = 0;

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unsupported read size %u at 0x%" HWADDR_PRIx "\n",
                      TYPE_G233_SPI_CONTROLLER, size, offset);
        return 0;
    }

    switch (offset) {
    case G233_SPI_REG_CR1:
        value = s->cr1;
        break;
    case G233_SPI_REG_CR2:
        value = s->cr2;
        break;
    case G233_SPI_REG_SR:
        value = s->sr;
        break;
    case G233_SPI_REG_DR:
        value = s->dr;
        s->sr &= ~G233_SPI_SR_RXNE;
        g233_spicontroller_update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                      TYPE_G233_SPI_CONTROLLER, offset);
        break;
    }

    return value;
}

static void g233_spicontroller_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    G233SPIControllerState *s = opaque;

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unsupported write size %u at 0x%" HWADDR_PRIx "\n",
                      TYPE_G233_SPI_CONTROLLER, size, offset);
        return;
    }

    switch (offset) {
        // 
    case G233_SPI_REG_CR1:
        s->cr1 = value;
        if (s->cr1 & G233_SPI_CR1_SPE) {
            s->sr |= G233_SPI_SR_TXE;
        } else {
            s->sr &= ~(G233_SPI_SR_TXE | G233_SPI_SR_RXNE);
        }
        g233_spicontroller_update_irq(s);
        break;
    case G233_SPI_REG_CR2:
        s->cr2 = value;
        s->active_cs = value & G233_SPI_CR2_CS_MASK;
        g233_spicontroller_drive_cs(s);
        break;
    case G233_SPI_REG_SR:
        /* The current tests only need OVERRUN as write-1-to-clear. */
        if (value & G233_SPI_SR_OVERRUN) {
            s->sr &= ~G233_SPI_SR_OVERRUN;
        }
        g233_spicontroller_update_irq(s);
        break;
    case G233_SPI_REG_DR:
        s->dr = value & 0xff;
        g233_spicontroller_transfer(s, s->dr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write offset 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n",
                      TYPE_G233_SPI_CONTROLLER, offset, value);
        break;
    }
}


// MemoryReg
static const MemoryRegionOps g233_spicontroller_ops = {
    .read = g233_spicontroller_read,
    .write = g233_spicontroller_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void g233_spicontroller_reset(DeviceState *dev)
{
    G233SPIControllerState *s = G233_SPI_CONTROLLER(dev);

    s->cr1 = 0;
    s->cr2 = 0;
    s->sr = 0;
    s->dr = 0;
    s->last_tx = 0;
    s->last_rx = 0;
    s->active_cs = 0;

    g233_spicontroller_drive_cs(s);
    g233_spicontroller_update_irq(s);
}

static void g233_spicontroller_realize(DeviceState *dev, Error **errp)
{
    G233SPIControllerState *s = G233_SPI_CONTROLLER(dev);

    if (s->num_cs == 0) {
        error_setg(errp, "num-cs must be greater than zero");
        return;
    }

    s->spi = ssi_create_bus(dev, "spi");
    s->cs_lines = g_new0(qemu_irq, s->num_cs);
    qdev_init_gpio_out_named(dev, s->cs_lines, "cs", s->num_cs);
}

static void g233_spicontroller_init(Object *obj)
{
    G233SPIControllerState *s = G233_SPI_CONTROLLER(obj);

    memory_region_init_io(&s->mmio, obj, &g233_spicontroller_ops, s,
                          TYPE_G233_SPI_CONTROLLER, G233_SPI_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void g233_spicontroller_finalize(Object *obj)
{
    G233SPIControllerState *s = G233_SPI_CONTROLLER(obj);

    if (s->transfer_timer) {
        timer_free(s->transfer_timer);
        s->transfer_timer = NULL;
    }

    g_free(s->cs_lines);
    s->cs_lines = NULL;
}

static const VMStateDescription vmstate_g233_spicontroller = {
    .name = TYPE_G233_SPI_CONTROLLER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cr1, G233SPIControllerState),
        VMSTATE_UINT32(cr2, G233SPIControllerState),
        VMSTATE_UINT32(sr, G233SPIControllerState),
        VMSTATE_UINT32(dr, G233SPIControllerState),
        VMSTATE_UINT8(active_cs, G233SPIControllerState),
        VMSTATE_UINT8(last_tx, G233SPIControllerState),
        VMSTATE_UINT8(last_rx, G233SPIControllerState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property g233_spicontroller_properties[] = {
    DEFINE_PROP_UINT32("num-cs", G233SPIControllerState, num_cs,
                       G233_SPI_DEFAULT_CS_COUNT),
};

static void g233_spicontroller_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = g233_spicontroller_realize;
    dc->vmsd = &vmstate_g233_spicontroller;
    device_class_set_legacy_reset(dc, g233_spicontroller_reset);
    device_class_set_props(dc, g233_spicontroller_properties);
}

static const TypeInfo g233_spicontroller_info = {
    .name = TYPE_G233_SPI_CONTROLLER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(G233SPIControllerState),
    .instance_init = g233_spicontroller_init,
    .instance_finalize = g233_spicontroller_finalize,
    .class_init = g233_spicontroller_class_init,
};

static void g233_spicontroller_register_types(void)
{
    type_register_static(&g233_spicontroller_info);
}

type_init(g233_spicontroller_register_types)
