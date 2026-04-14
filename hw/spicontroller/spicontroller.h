#ifndef HW_SPICONTROLLER_SPICONTROLLER_H
#define HW_SPICONTROLLER_SPICONTROLLER_H

/*
 * Skeleton for the custom G233 SPI host controller.
 *
 * This model is the SPI master/MMIO front-end seen by the guest CPU.
 * The actual flash/EEPROM protocol stays in SSI peripherals such as
 * m25p80 or at25. The board model is responsible for:
 *   1. mapping this device into the system address space
 *   2. wiring its IRQ output to PLIC
 *   3. attaching flash/eeprom devices to the SSI bus and CS lines
 */

#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/ssi/ssi.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_G233_SPI_CONTROLLER "g233-spi-controller"
OBJECT_DECLARE_SIMPLE_TYPE(G233SPIControllerState, G233_SPI_CONTROLLER)

/*
 * Guest-visible MMIO window.
 *
 * Current qtests only touch 0x00..0x0c, but it is convenient to reserve
 * a full 4 KiB page so the board can map it like a normal SoC peripheral.
 */


 /**
  * 9.2 寄存器映射¶
SPI 基地址：0x1001_8000。

Offset	寄存器	访问	复位值	描述
0x00	SPI_CR1	R/W	0x0000_0000	控制寄存器 1（使能/主从/中断使能）
0x04	SPI_CR2	R/W	0x0000_0000	控制寄存器 2（片选选择）
0x08	SPI_SR	R/W	0x0000_0002	状态寄存器
0x0C	SPI_DR	R/W	0x0000_0000	数据寄存器
Note

SPI 控制器不使用独立的片选控制寄存器，片选选择合并到 SPI_CR2[1:0]。

9.3 寄存器定义¶
9.3.1 SPI_CR1（Offset 0x00）— Control Register 1¶
Bit	名称	访问	复位	描述
31:8	Reserved	-	0	保留位，必须保持为 0
7	TXEIE	R/W	0	TXE 中断使能：0 禁用；1 使能
6	RXNEIE	R/W	0	RXNE 中断使能：0 禁用；1 使能
5	ERRIE	R/W	0	错误中断使能（OVERRUN）：0 禁用；1 使能
4:3	Reserved	-	0	保留位，必须保持为 0
2	MSTR	R/W	0	主从选择：0 从模式；1 主模式
1	Reserved	-	0	保留位，必须保持为 0
0	SPE	R/W	0	SPI 使能：0 禁用；1 使能
9.3.2 SPI_CR2（Offset 0x04）— Control Register 2¶
Bit	名称	访问	复位	描述
31:2	Reserved	-	0	保留位，必须保持为 0
1:0	CS_SEL	R/W	0	片选选择：0 CS0；1 CS1；2 CS2；3 CS3
Note

写 SPI_CR2[1:0] 选择当前激活的片选通道；切换 CS 时控制器应自动释放之前的 CS 并激活新的 CS。CS 的实际有效电平与时序为实现定义。

9.3.3 SPI_SR（Offset 0x08）— Status Register¶
Bit	名称	访问	复位	描述
31:5	Reserved	R	0	保留位
4	OVERRUN	R/W1C	0	溢出错误：RXNE=1 时又收到新字节则置 1；写 1 清除
3:2	Reserved	R	0	保留位
1	TXE	R	1	发送缓冲区空：1 空；0 满
0	RXNE	R	0	接收缓冲区非空：1 非空；0 为空（读 SPI_DR 清除）
9.3.4 SPI_DR（Offset 0x0C）— Data Register¶
Bit	名称	访问	复位	描述
31:8	Reserved	-	0	保留位，必须保持为 0
7:0	DATA	R/W	0	写入：发送数据；读取：接收数据

  */
#define G233_SPI_MMIO_SIZE          0x1000

/* Register offsets */
#define G233_SPI_REG_CR1            0x00
#define G233_SPI_REG_CR2            0x04
#define G233_SPI_REG_SR             0x08
#define G233_SPI_REG_DR             0x0c

/* CR1 bits expected by the educational qtests */
/*SPI enable */  
#define G233_SPI_CR1_SPE            (1u << 0)
/*Masster/slave mode select*/ 
#define G233_SPI_CR1_MSTR           (1u << 2)
/* Error enable */
#define G233_SPI_CR1_ERRIE          (1u << 5)
/* RXNE interrupt enable */
#define G233_SPI_CR1_RXNEIE         (1u << 6)
/* TXE interrupt enable */
#define G233_SPI_CR1_TXEIE          (1u << 7)

/* CR2 bits */
#define G233_SPI_CR2_CS_MASK        0x3u

/* SR bits */
#define G233_SPI_SR_RXNE            (1u << 0)
#define G233_SPI_SR_TXE             (1u << 1)
#define G233_SPI_SR_OVERRUN         (1u << 4)

#define G233_SPI_DEFAULT_CS_COUNT   2

typedef struct G233SPIControllerState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;

    /*
     * Single interrupt output towards the board IRQ chip/PLIC.
     * The board decides which PLIC source this line lands on.
     */
    qemu_irq irq;

    /*
     * Master-side SPI topology.
     * - spi: shared serial bus carrying MOSI/MISO/clock
     * - cs_lines: external chip-select outputs, one per peripheral
     */
    SSIBus *spi;
    qemu_irq *cs_lines;
    uint32_t num_cs;
    uint8_t active_cs;

    /*
     * Minimal register bank required by the current tests.
     * Keeping these explicit is easier to reason about than a raw regs[] array
     * while the model is still small.
     */
    uint32_t cr1;
    uint32_t cr2;
    uint32_t sr;
    uint32_t dr;

    /*
     * Staging fields for later refinement.
     * You can expand this into FIFOs/timers once you start modelling latency,
     * burst transfers, or more accurate overrun timing.
     */
    uint8_t last_tx;
    uint8_t last_rx;
    QEMUTimer *transfer_timer;
} G233SPIControllerState;

#endif /* HW_SPICONTROLLER_SPICONTROLLER_H */
