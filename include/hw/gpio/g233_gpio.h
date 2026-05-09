#ifndef HW_G233_GPIO_H
#define HW_G233_GPIO_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_G233_GPIO "riscv.g233.gpio"

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
#define G233_GPIO_DIR   0x0000
#define G233_GPIO_OUT   0x0004
#define G233_GPIO_IN    0x0008
#define G233_GPIO_IE    0x000C 
#define G233_GPIO_IS    0x0010
#define G233_GPIO_TRIG  0x0014
#define G233_GPIO_POL   0x0018
#define G233_GPIO_NREGS 7

typedef struct G233GPIOState G233GPIOState;
/* Checker为G233_GPIO, 检查类型为G233GPIOState */
DECLARE_INSTANCE_CHECKER(G233GPIOState, G233_GPIO, TYPE_G233_GPIO);

struct G233GPIOState
{
    SysBusDevice parent_obj;            /* 系统总线设备，其实就是通过访问内存来互动的设备 */
    MemoryRegion mmio;                  /* MMIO region */
    uint32_t regs[G233_GPIO_NREGS];     /* register table */

    qemu_irq irq;                       /* interrupt line connected with PLIC */
};




#endif