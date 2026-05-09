/*
 * GPIO controller for G233
*/
#include "qemu/osdep.h"
#include "hw/gpio/g233_gpio.h"
#include "hw/core/sysbus.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"

/* Any place that causes the value of pin or IE&IS to change, will need to call this func to update the irq state */
static void g233_gpio_update_irq(G233GPIOState *g)
{
    uint32_t ie = g->regs[G233_GPIO_IE >> 2];
    uint32_t is = g->regs[G233_GPIO_IS >> 2];

    uint32_t pending = ie & is;
    if (pending) {
        qemu_set_irq(g->irq, 1);  // 升起中断线，PLIC会将对应pending位置1
    } else {
        qemu_set_irq(g->irq, 0);  // 降下中断线，PLIC会感知下降沿（如果配置为电平触发）
    }
}

static void g233_gpio_recalc(G233GPIOState *g, uint32_t prev)
{
    uint32_t trig = g->regs[G233_GPIO_TRIG >> 2];
    uint32_t pol = g->regs[G233_GPIO_POL >> 2];
    uint32_t dir = g->regs[G233_GPIO_DIR >> 2];
    uint32_t curr = g->regs[G233_GPIO_OUT >> 2];
    uint32_t ie = g->regs[G233_GPIO_IE >> 2];
    
    /* 安全地更新 IN 寄存器 (输出回环)，保留物理输入引脚状态 */
    uint32_t old_in = g->regs[G233_GPIO_IN >> 2];
    g->regs[G233_GPIO_IN >> 2] = (old_in & ~dir) | (curr & dir);

    /* 处理中断 */
    // 如果硬件手册真的是这种特殊设计，就把 ie 乘回来
    uint32_t cared = dir & ie;
    // uint32_t cared = dir;   
    
    uint32_t edge_rising = ~prev & curr & cared;
    uint32_t edge_falling = prev & (~curr) & cared;
    uint32_t level_low = ~curr & cared;
    uint32_t level_high = curr & cared;

    /* 更新状态 */
    g->regs[G233_GPIO_IS >> 2] |= edge_rising & pol & (~trig); 
    g->regs[G233_GPIO_IS >> 2] |= edge_falling & (~pol) & (~trig);
    g->regs[G233_GPIO_IS >> 2] |= level_low & ~(pol) & trig;
    g->regs[G233_GPIO_IS >> 2] |= level_high & pol & trig;
    
    /* 清除不再满足条件的电平中断状态 */
    g->regs[G233_GPIO_IS >> 2] &= ~(level_low & pol & trig);        /* 规则是高电平触发，但当前电平变低了，清除 */
    g->regs[G233_GPIO_IS >> 2] &= ~(level_high & (~pol) & trig);    /* 规则是低电平触发，但当前电平变高了，清除 */

    g233_gpio_update_irq(g);
}
/*
    寄存器映射：
    Offset	寄存器	        访问	    复位值	    描述
    0x00	GPIO_DIR	    R/W	    0x0000_0000	方向寄存器
    0x04	GPIO_OUT	    R/W	    0x0000_0000	输出数据寄存器
    0x08	GPIO_IN	        R	    0x0000_0000	输入数据寄存器
    0x0C	GPIO_IE	        R/W	    0x0000_0000	中断使能寄存器
    0x10	GPIO_IS	        R/W	    0x0000_0000	中断状态寄存器
    0x14	GPIO_TRIG	    R/W	    0x0000_0000	中断触发类型寄存器
    0x18	GPIO_POL	    R/W	    0x0000_0000	中断极性寄存器
*/
/* 寄存器位宽都是4个字节，因此一般不会用到size，而是通过.valid来限制 */
static uint64_t g233_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
    uint32_t value = 0;    /* register */
    G233GPIOState *g = G233_GPIO(opaque);
    switch (addr)
    {
    case G233_GPIO_DIR:
    case G233_GPIO_OUT:
    case G233_GPIO_IN:
    case G233_GPIO_IE:
    case G233_GPIO_IS:
    case G233_GPIO_TRIG:
    case G233_GPIO_POL:
        value = g->regs[addr >> 2];
        break;
    /* 非法地址 */
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad read offset 0x%" HWADDR_PRIx "\n",
              __func__, addr);
    }
    return value;
}

/* 写的时候就需要发生相应的动作 */
/* 引脚为输入模式时，其电平值由外部决定，输出模式时，则由DIR和OUT寄存器来决定 */
static void g233_gpio_write(void *opaque, hwaddr addr, uint64_t val64, unsigned int size)
{
    G233GPIOState *g = G233_GPIO(opaque);
    uint32_t prev;
    switch (addr)
    {
    /* GPIO_OUT可以和DIR配合，来控制引脚的输出。对于输入引脚的中断触发，这里不负责管理 */
    /* 中断状态变化无非两种情况：规则变化(电平固定)，对应前三种case，和电平变化(规则固定)，对应G233_GPIO_OUT */
    case G233_GPIO_DIR:
    case G233_GPIO_POL:
    case G233_GPIO_TRIG:
    case G233_GPIO_OUT:
        /* 记录任意规则/电平变化前的 OUT 寄存器状态 */
        prev = g->regs[G233_GPIO_OUT >> 2];
        /* 更新寄存器 */
        g->regs[addr >> 2] = (uint32_t)val64;
        g233_gpio_recalc(g, prev);
        return;
    case G233_GPIO_IE:
        g->regs[addr >> 2] = (uint32_t)val64;
        g233_gpio_update_irq(g);
        return;
    case G233_GPIO_IS:
        g->regs[addr >> 2] &= ~((uint32_t)val64);  /* 写1清零，否则什么都不做 */
        g233_gpio_update_irq(g);
        return;
    default:
        break;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: write: addr=0x%" HWADDR_PRIx
              " val=0x%016" PRIx64 "\n",
              __func__, addr, val64);
}

static const MemoryRegionOps g233_gpio_ops = {
    .read = g233_gpio_read,
    .write = g233_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,   /* 大小端 */
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static void g233_gpio_init(Object *obj)
{
    G233GPIOState *g = G233_GPIO(obj);
    /* 制造一片内存区域 (包含读写回调函数 g233_gpio_ops)，总线路由就可以访问到回调函数了 */
    memory_region_init_io(&g->mmio, obj, &g233_gpio_ops, g, TYPE_G233_GPIO, 0x100);
    /* 声明 MMIO 接口 (引出数据总线引脚) */
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &g->mmio);
    /* 声明中断输出线 (引出一根叫 irq 的物理线) */
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &g->irq);
}

static const TypeInfo g233_gpio_info = {
    .name = TYPE_G233_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(G233GPIOState),
    .instance_init = g233_gpio_init,
};

static void g233_gpio_register_types(void)
{
    type_register_static(&g233_gpio_info);
}

type_init(g233_gpio_register_types)


