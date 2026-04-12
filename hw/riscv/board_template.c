/*
 * QEMU RISC-V board template
 *
 * This file is intentionally written as a "machine skeleton" you can study
 * and then rename.  It is not a pseudocode sample: the machine is wired into
 * the build, can be selected with `-machine board-template`, and contains the
 * minimum board-level pieces needed by a small RISC-V OS:
 *
 *   - a hart array
 *   - DRAM
 *   - mask ROM reset vector
 *   - ACLINT (IPI + timer)
 *   - PLIC
 *   - one NS16550A-compatible UART
 *   - SiFive test device for poweroff/reboot
 *   - a generated device tree
 *   - firmware/kernel/DTB handoff
 *
 * Read it as a "where does each responsibility belong?" example:
 *
 *   class_init    -> machine-wide defaults and board capabilities
 *   instance_init -> initialize this machine object's child objects/defaults
 *   machine_init  -> assemble CPU + memory + IRQ + devices into one board
 *
 * If you later build your own board, the usual workflow is:
 *   1. copy/rename this file
 *   2. rename TYPE/struct/machine strings
 *   3. replace the memmap and IRQ layout
 *   4. replace or extend the devices created in machine_init()
 *   5. adjust create_fdt() so the guest sees your real hardware contract
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/char/serial-mm.h"
#include "target/riscv/cpu.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/boot.h"
#include "hw/intc/riscv_aclint.h"
#include "hw/intc/sifive_plic.h"
#include "hw/misc/sifive_test.h"
#include "chardev/char.h"
#include "system/device_tree.h"
#include "system/system.h"
#include "system/kvm.h"

#define TYPE_RISCV_BOARD_TEMPLATE_MACHINE MACHINE_TYPE_NAME("inkbottle-board")
OBJECT_DECLARE_SIMPLE_TYPE(RISCVBoardTemplateState,
                           RISCV_BOARD_TEMPLATE_MACHINE)

/*
 * ---------------------------------------------------------------------------
 * 1. Board-wide constants
 * ---------------------------------------------------------------------------
 *
 * Keep the board's physical contract here:
 *   - MMIO layout
 *   - IRQ numbering seen by the interrupt controller
 *   - controller geometry
 *
 * When you design your own board, edit this section first.
 */

enum {
    BOARD_TEMPLATE_MROM,
    BOARD_TEMPLATE_TEST,
    BOARD_TEMPLATE_CLINT,
    BOARD_TEMPLATE_PLIC,
    BOARD_TEMPLATE_UART0,
    BOARD_TEMPLATE_UART1,
    BOARD_TEMPLATE_DRAM,
};

static const MemMapEntry board_template_memmap[] = {
    [BOARD_TEMPLATE_MROM]  = {      0x1000,      0xf000 },
    [BOARD_TEMPLATE_TEST]  = {    0x100000,      0x1000 },
    [BOARD_TEMPLATE_CLINT] = {   0x2000000,     0x10000 },
    [BOARD_TEMPLATE_PLIC]  = {   0xc000000,   0x4000000 },
    [BOARD_TEMPLATE_UART0] = {  0x10000000,       0x100 },
    [BOARD_TEMPLATE_UART1] = {  0x20000000,       0x100 },
    [BOARD_TEMPLATE_DRAM]  = {  0x80000000,           0 },
};

enum {
    BOARD_TEMPLATE_UART0_IRQ = 10,
    BOARD_TEMPLATE_UART1_IRQ = 11,
    BOARD_TEMPLATE_PLIC_NUM_SOURCES = 127,
};

#define BOARD_TEMPLATE_FDT_PLIC_ADDR_CELLS 0
#define BOARD_TEMPLATE_FDT_PLIC_INT_CELLS  1

/*
 * These offsets follow the commonly used SiFive PLIC register layout.
 * For an educational skeleton, this is a good default.  Replace them only if
 * your guest-visible PLIC-compatible block truly differs.
 */
#define BOARD_TEMPLATE_PLIC_NUM_PRIORITIES 7
#define BOARD_TEMPLATE_PLIC_PRIORITY_BASE  0x0
#define BOARD_TEMPLATE_PLIC_PENDING_BASE   0x1000
#define BOARD_TEMPLATE_PLIC_ENABLE_BASE    0x2000
#define BOARD_TEMPLATE_PLIC_ENABLE_STRIDE  0x80
#define BOARD_TEMPLATE_PLIC_CONTEXT_BASE   0x200000
#define BOARD_TEMPLATE_PLIC_CONTEXT_STRIDE 0x1000

/*
 * ---------------------------------------------------------------------------
 * 2. Machine state
 * ---------------------------------------------------------------------------
 *
 * This structure stores the board-level child objects and runtime state
 * needed during machine assembly.
 */

struct RISCVBoardTemplateState {
    /*< private >*/
    MachineState parent_obj;

    /*< public >*/
    RISCVHartArrayState harts;
    MemoryRegion mrom;
    DeviceState *plic;
    int fdt_size;
};

/*
 * ---------------------------------------------------------------------------
 * 3. Small board-local helpers
 * ---------------------------------------------------------------------------
 *
 * Keep helpers "board-level":
 *   - create one controller at a fixed board address
 *   - generate one FDT node that describes the board contract
 *
 * Do not put device-internal behaviour here.
 */

static DeviceState *board_template_create_plic(MachineState *machine)
{
    char *hart_config = riscv_plic_hart_config_string(machine->smp.cpus);

    return sifive_plic_create(board_template_memmap[BOARD_TEMPLATE_PLIC].base,
                              hart_config,
                              machine->smp.cpus,
                              0,
                              BOARD_TEMPLATE_PLIC_NUM_SOURCES,
                              BOARD_TEMPLATE_PLIC_NUM_PRIORITIES,
                              BOARD_TEMPLATE_PLIC_PRIORITY_BASE,
                              BOARD_TEMPLATE_PLIC_PENDING_BASE,
                              BOARD_TEMPLATE_PLIC_ENABLE_BASE,
                              BOARD_TEMPLATE_PLIC_ENABLE_STRIDE,
                              BOARD_TEMPLATE_PLIC_CONTEXT_BASE,
                              BOARD_TEMPLATE_PLIC_CONTEXT_STRIDE,
                              board_template_memmap[BOARD_TEMPLATE_PLIC].size);
}

static void board_template_create_clint(MachineState *machine)
{
    riscv_aclint_swi_create(board_template_memmap[BOARD_TEMPLATE_CLINT].base,
                            0, machine->smp.cpus, false);
    riscv_aclint_mtimer_create(
        board_template_memmap[BOARD_TEMPLATE_CLINT].base +
        RISCV_ACLINT_SWI_SIZE,
        RISCV_ACLINT_DEFAULT_MTIMER_SIZE,
        0, machine->smp.cpus,
        RISCV_ACLINT_DEFAULT_MTIMECMP,
        RISCV_ACLINT_DEFAULT_MTIME,
        RISCV_ACLINT_DEFAULT_TIMEBASE_FREQ,
        false);
}

static void board_template_create_uart(MachineState *machine,
                                       DeviceState *irqchip)
{
    serial_mm_init(get_system_memory(),
                   board_template_memmap[BOARD_TEMPLATE_UART0].base,
                   0,
                   qdev_get_gpio_in(irqchip, BOARD_TEMPLATE_UART0_IRQ),
                   3686400,
                   serial_hd(0),
                   DEVICE_LITTLE_ENDIAN);
    serial_mm_init(get_system_memory(),
                   board_template_memmap[BOARD_TEMPLATE_UART1].base,
                   0,
                   qdev_get_gpio_in(irqchip, BOARD_TEMPLATE_UART1_IRQ),
                   3686400,
                   serial_hd(1),
                   DEVICE_LITTLE_ENDIAN);
}

static void board_template_create_test(MachineState *machine)
{
    sifive_test_create(board_template_memmap[BOARD_TEMPLATE_TEST].base);
}

/*
 * ---------------------------------------------------------------------------
 * 4. FDT generation
 * ---------------------------------------------------------------------------
 *
 * The board object graph alone is not enough for most guests.  The guest also
 * needs a discoverable hardware description.  For RISC-V boards, that usually
 * means a DTB.  Keep the DT layout in sync with your MMIO/IRQ map above.
 */

static void board_template_fdt_add_cpus(RISCVBoardTemplateState *s,
                                        uint32_t *next_phandle,
                                        uint32_t *intc_phandles)
{
    MachineState *machine = MACHINE(s);
    bool is_32_bit = riscv_is_32bit(&s->harts);
    int cpu;

    qemu_fdt_add_subnode(machine->fdt, "/cpus");
    qemu_fdt_setprop_cell(machine->fdt, "/cpus", "#address-cells", 0x1);
    qemu_fdt_setprop_cell(machine->fdt, "/cpus", "#size-cells", 0x0);
    qemu_fdt_setprop_cell(machine->fdt, "/cpus", "timebase-frequency",
                          RISCV_ACLINT_DEFAULT_TIMEBASE_FREQ);

    qemu_fdt_add_subnode(machine->fdt, "/cpus/cpu-map");
    qemu_fdt_add_subnode(machine->fdt, "/cpus/cpu-map/cluster0");

    for (cpu = 0; cpu < s->harts.num_harts; cpu++) {
        g_autofree char *cpu_name =
            g_strdup_printf("/cpus/cpu@%u", s->harts.hartid_base + cpu);
        g_autofree char *intc_name =
            g_strdup_printf("%s/interrupt-controller", cpu_name);
        g_autofree char *core_name =
            g_strdup_printf("/cpus/cpu-map/cluster0/core%d", cpu);
        uint32_t cpu_phandle = (*next_phandle)++;

        qemu_fdt_add_subnode(machine->fdt, cpu_name);
        qemu_fdt_setprop_string(machine->fdt, cpu_name, "device_type", "cpu");
        qemu_fdt_setprop_string(machine->fdt, cpu_name, "compatible", "riscv");
        qemu_fdt_setprop_string(machine->fdt, cpu_name, "status", "okay");
        qemu_fdt_setprop_cell(machine->fdt, cpu_name, "reg",
                              s->harts.hartid_base + cpu);
        qemu_fdt_setprop_string(machine->fdt, cpu_name, "mmu-type",
                                is_32_bit ? "riscv,sv32" : "riscv,sv48");
        riscv_isa_write_fdt(&s->harts.harts[cpu], machine->fdt, cpu_name);
        qemu_fdt_setprop_cell(machine->fdt, cpu_name, "phandle", cpu_phandle);

        qemu_fdt_add_subnode(machine->fdt, intc_name);
        intc_phandles[cpu] = (*next_phandle)++;
        qemu_fdt_setprop_string(machine->fdt, intc_name, "compatible",
                                "riscv,cpu-intc");
        qemu_fdt_setprop(machine->fdt, intc_name, "interrupt-controller",
                         NULL, 0);
        qemu_fdt_setprop_cell(machine->fdt, intc_name, "#interrupt-cells", 1);
        qemu_fdt_setprop_cell(machine->fdt, intc_name, "phandle",
                              intc_phandles[cpu]);

        qemu_fdt_add_subnode(machine->fdt, core_name);
        qemu_fdt_setprop_cell(machine->fdt, core_name, "cpu", cpu_phandle);
    }
}

static void board_template_fdt_add_memory(RISCVBoardTemplateState *s)
{
    MachineState *machine = MACHINE(s);
    g_autofree char *mem_name =
        g_strdup_printf("/memory@%" HWADDR_PRIx,
                        board_template_memmap[BOARD_TEMPLATE_DRAM].base);

    qemu_fdt_add_subnode(machine->fdt, mem_name);
    qemu_fdt_setprop_string(machine->fdt, mem_name, "device_type", "memory");
    qemu_fdt_setprop_sized_cells(machine->fdt, mem_name, "reg",
                                 2, board_template_memmap[BOARD_TEMPLATE_DRAM].base,
                                 2, machine->ram_size);
}

static void board_template_fdt_add_clint(RISCVBoardTemplateState *s,
                                         uint32_t *intc_phandles)
{
    MachineState *machine = MACHINE(s);
    g_autofree char *clint_name =
        g_strdup_printf("/soc/clint@%" HWADDR_PRIx,
                        board_template_memmap[BOARD_TEMPLATE_CLINT].base);
    g_autofree uint32_t *clint_cells =
        g_new0(uint32_t, s->harts.num_harts * 4);
    int cpu;
    static const char * const clint_compat[2] = {
        "sifive,clint0",
        "riscv,clint0",
    };

    /*
     * This skeleton advertises one CLINT-style node because it is shorter and
     * works well as a teaching board.  If you later model ACLINT sub-blocks
     * separately, split this into mswi/mtimer/sswi child nodes.
     */
    for (cpu = 0; cpu < s->harts.num_harts; cpu++) {
        clint_cells[cpu * 4 + 0] = cpu_to_be32(intc_phandles[cpu]);
        clint_cells[cpu * 4 + 1] = cpu_to_be32(IRQ_M_SOFT);
        clint_cells[cpu * 4 + 2] = cpu_to_be32(intc_phandles[cpu]);
        clint_cells[cpu * 4 + 3] = cpu_to_be32(IRQ_M_TIMER);
    }

    qemu_fdt_add_subnode(machine->fdt, clint_name);
    qemu_fdt_setprop_string_array(machine->fdt, clint_name, "compatible",
                                  (char **)&clint_compat,
                                  ARRAY_SIZE(clint_compat));
    qemu_fdt_setprop_sized_cells(machine->fdt, clint_name, "reg",
                                 2, board_template_memmap[BOARD_TEMPLATE_CLINT].base,
                                 2, board_template_memmap[BOARD_TEMPLATE_CLINT].size);
    qemu_fdt_setprop(machine->fdt, clint_name, "interrupts-extended",
                     clint_cells,
                     s->harts.num_harts * sizeof(uint32_t) * 4);
}

static uint32_t board_template_fdt_add_plic(RISCVBoardTemplateState *s,
                                            uint32_t next_phandle,
                                            uint32_t *intc_phandles)
{
    MachineState *machine = MACHINE(s);
    g_autofree char *plic_name =
        g_strdup_printf("/soc/plic@%" HWADDR_PRIx,
                        board_template_memmap[BOARD_TEMPLATE_PLIC].base);
    uint32_t plic_phandle = next_phandle;
    uint32_t cells_per_hart = kvm_enabled() ? 2 : 4;
    g_autofree uint32_t *plic_cells =
        g_new0(uint32_t, s->harts.num_harts * cells_per_hart);
    int cpu;
    static const char * const plic_compat[2] = {
        "sifive,plic-1.0.0",
        "riscv,plic0",
    };

    for (cpu = 0; cpu < s->harts.num_harts; cpu++) {
        if (kvm_enabled()) {
            plic_cells[cpu * 2 + 0] = cpu_to_be32(intc_phandles[cpu]);
            plic_cells[cpu * 2 + 1] = cpu_to_be32(IRQ_S_EXT);
        } else {
            plic_cells[cpu * 4 + 0] = cpu_to_be32(intc_phandles[cpu]);
            plic_cells[cpu * 4 + 1] = cpu_to_be32(IRQ_M_EXT);
            plic_cells[cpu * 4 + 2] = cpu_to_be32(intc_phandles[cpu]);
            plic_cells[cpu * 4 + 3] = cpu_to_be32(IRQ_S_EXT);
        }
    }

    qemu_fdt_add_subnode(machine->fdt, plic_name);
    qemu_fdt_setprop_string_array(machine->fdt, plic_name, "compatible",
                                  (char **)&plic_compat,
                                  ARRAY_SIZE(plic_compat));
    qemu_fdt_setprop(machine->fdt, plic_name, "interrupt-controller",
                     NULL, 0);
    qemu_fdt_setprop_cell(machine->fdt, plic_name, "#interrupt-cells",
                          BOARD_TEMPLATE_FDT_PLIC_INT_CELLS);
    qemu_fdt_setprop_cell(machine->fdt, plic_name, "#address-cells",
                          BOARD_TEMPLATE_FDT_PLIC_ADDR_CELLS);
    qemu_fdt_setprop(machine->fdt, plic_name, "interrupts-extended",
                     plic_cells,
                     s->harts.num_harts * sizeof(uint32_t) * cells_per_hart);
    qemu_fdt_setprop_sized_cells(machine->fdt, plic_name, "reg",
                                 2, board_template_memmap[BOARD_TEMPLATE_PLIC].base,
                                 2, board_template_memmap[BOARD_TEMPLATE_PLIC].size);
    qemu_fdt_setprop_cell(machine->fdt, plic_name, "riscv,ndev",
                          BOARD_TEMPLATE_PLIC_NUM_SOURCES - 1);
    qemu_fdt_setprop_cell(machine->fdt, plic_name, "phandle", plic_phandle);

    return plic_phandle;
}

static void board_template_fdt_add_uart(RISCVBoardTemplateState *s,
                                        uint32_t plic_phandle)
{
    MachineState *machine = MACHINE(s);
    g_autofree char *uart_name =
        g_strdup_printf("/soc/serial@%" HWADDR_PRIx,
                        board_template_memmap[BOARD_TEMPLATE_UART0].base);

    g_autofree char *uart_name2 =
    g_strdup_printf("/soc/serial@%" HWADDR_PRIx,
                    board_template_memmap[BOARD_TEMPLATE_UART1].base);

    qemu_fdt_add_subnode(machine->fdt, uart_name);
    qemu_fdt_add_subnode(machine->fdt, uart_name2);
    qemu_fdt_setprop_string(machine->fdt, uart_name, "compatible", "ns16550a");
    qemu_fdt_setprop_string(machine->fdt, uart_name2, "compatible", "ns16550a");

    qemu_fdt_setprop_sized_cells(machine->fdt, uart_name, "reg",
                                 2, board_template_memmap[BOARD_TEMPLATE_UART0].base,
                                 2, board_template_memmap[BOARD_TEMPLATE_UART0].size);

    qemu_fdt_setprop_sized_cells(machine->fdt, uart_name2, "reg",
                                 2, board_template_memmap[BOARD_TEMPLATE_UART1].base,
                                 2, board_template_memmap[BOARD_TEMPLATE_UART1].size);

    qemu_fdt_setprop_cell(machine->fdt, uart_name, "clock-frequency", 3686400);
    qemu_fdt_setprop_cell(machine->fdt, uart_name2, "clock-frequency", 3686400);

    qemu_fdt_setprop_cell(machine->fdt, uart_name, "interrupt-parent",
                          plic_phandle);
    qemu_fdt_setprop_cell(machine->fdt, uart_name2, "interrupt-parent",
                          plic_phandle);
    qemu_fdt_setprop_cell(machine->fdt, uart_name, "interrupts",
                          BOARD_TEMPLATE_UART0_IRQ);
    qemu_fdt_setprop_cell(machine->fdt, uart_name2, "interrupts",
                          BOARD_TEMPLATE_UART1_IRQ);

    qemu_fdt_setprop_string(machine->fdt, "/chosen", "stdout-path", uart_name);
    qemu_fdt_setprop_string(machine->fdt, "/aliases", "serial0", uart_name);
    qemu_fdt_setprop_string(machine->fdt, "/aliases", "serial1", uart_name2);

}

static void board_template_fdt_add_test(RISCVBoardTemplateState *s,
                                        uint32_t *next_phandle)
{
    MachineState *machine = MACHINE(s);
    g_autofree char *test_name =
        g_strdup_printf("/soc/test@%" HWADDR_PRIx,
                        board_template_memmap[BOARD_TEMPLATE_TEST].base);
    uint32_t test_phandle = (*next_phandle)++;
    static const char * const compat[3] = {
        "sifive,test1",
        "sifive,test0",
        "syscon",
    };

    qemu_fdt_add_subnode(machine->fdt, test_name);
    qemu_fdt_setprop_string_array(machine->fdt, test_name, "compatible",
                                  (char **)&compat, ARRAY_SIZE(compat));
    qemu_fdt_setprop_sized_cells(machine->fdt, test_name, "reg",
                                 2, board_template_memmap[BOARD_TEMPLATE_TEST].base,
                                 2, board_template_memmap[BOARD_TEMPLATE_TEST].size);
    qemu_fdt_setprop_cell(machine->fdt, test_name, "phandle", test_phandle);

    qemu_fdt_add_subnode(machine->fdt, "/reboot");
    qemu_fdt_setprop_string(machine->fdt, "/reboot", "compatible",
                            "syscon-reboot");
    qemu_fdt_setprop_cell(machine->fdt, "/reboot", "regmap", test_phandle);
    qemu_fdt_setprop_cell(machine->fdt, "/reboot", "offset", 0x0);
    qemu_fdt_setprop_cell(machine->fdt, "/reboot", "value", FINISHER_RESET);

    qemu_fdt_add_subnode(machine->fdt, "/poweroff");
    qemu_fdt_setprop_string(machine->fdt, "/poweroff", "compatible",
                            "syscon-poweroff");
    qemu_fdt_setprop_cell(machine->fdt, "/poweroff", "regmap", test_phandle);
    qemu_fdt_setprop_cell(machine->fdt, "/poweroff", "offset", 0x0);
    qemu_fdt_setprop_cell(machine->fdt, "/poweroff", "value", FINISHER_PASS);
}

static void board_template_create_fdt(RISCVBoardTemplateState *s)
{
    MachineState *machine = MACHINE(s);
    uint32_t next_phandle = 1;
    g_autofree uint32_t *intc_phandles = g_new0(uint32_t, s->harts.num_harts);
    uint32_t plic_phandle;

    machine->fdt = create_device_tree(&s->fdt_size);
    if (!machine->fdt) {
        error_report("create_device_tree() failed");
        exit(1);
    }

    /*
     * TODO(board-name): replace model/compatible with your board's identity.
     * Guest software can and does key off these strings.
     */
    qemu_fdt_setprop_string(machine->fdt, "/", "model",
                            "qemu,riscv-board-template");
    qemu_fdt_setprop_string(machine->fdt, "/", "compatible",
                            "qemu,riscv-board-template");
    qemu_fdt_setprop_cell(machine->fdt, "/", "#size-cells", 0x2);
    qemu_fdt_setprop_cell(machine->fdt, "/", "#address-cells", 0x2);

    qemu_fdt_add_subnode(machine->fdt, "/soc");
    qemu_fdt_setprop(machine->fdt, "/soc", "ranges", NULL, 0);
    qemu_fdt_setprop_string(machine->fdt, "/soc", "compatible", "simple-bus");
    qemu_fdt_setprop_cell(machine->fdt, "/soc", "#size-cells", 0x2);
    qemu_fdt_setprop_cell(machine->fdt, "/soc", "#address-cells", 0x2);

    qemu_fdt_add_subnode(machine->fdt, "/chosen");
    qemu_fdt_add_subnode(machine->fdt, "/aliases");

    board_template_fdt_add_cpus(s, &next_phandle, intc_phandles);
    board_template_fdt_add_memory(s);
    board_template_fdt_add_clint(s, intc_phandles);
    plic_phandle = board_template_fdt_add_plic(s, next_phandle,
                                               intc_phandles);
    next_phandle = plic_phandle + 1;
    board_template_fdt_add_uart(s, plic_phandle);
    board_template_fdt_add_test(s, &next_phandle);
}

/*
 * ---------------------------------------------------------------------------
 * 5. Boot flow helper
 * ---------------------------------------------------------------------------
 *
 * This section wires firmware/kernel/DTB/reset-vector handoff.
 * Without it, the board may "exist" in QEMU but it is not yet a bootable
 * system model.
 */

static void board_template_setup_boot(RISCVBoardTemplateState *s)
{
    MachineState *machine = MACHINE(s);
    RISCVBootInfo boot_info;
    g_autofree char *firmware_name = NULL;
    hwaddr firmware_load_addr = board_template_memmap[BOARD_TEMPLATE_DRAM].base;
    hwaddr firmware_end_addr = firmware_load_addr;
    hwaddr kernel_start_addr;
    hwaddr fdt_load_addr = 0;
    uint64_t kernel_entry = 0;
    hwaddr start_addr;

    if (machine->dtb) {
        machine->fdt = load_device_tree(machine->dtb, &s->fdt_size);
        if (!machine->fdt) {
            error_report("load_device_tree() failed");
            exit(1);
        }
    } else {
        board_template_create_fdt(s);
    }

    if (machine->firmware && !strcmp(machine->firmware, "none")) {
        if (!machine->kernel_filename) {
            error_report("for -bios none, a kernel is required");
            exit(1);
        }
        firmware_load_addr = 0;
    } else {
        firmware_name = riscv_find_firmware(machine->firmware,
                                            riscv_default_firmware_name(&s->harts));
        if (firmware_name) {
            firmware_end_addr = riscv_load_firmware(firmware_name,
                                                    &firmware_load_addr,
                                                    NULL);
        }
    }

    riscv_boot_info_init(&boot_info, &s->harts);
    if (machine->kernel_filename) {
        kernel_start_addr = riscv_calc_kernel_start_addr(&boot_info,
                                                         firmware_end_addr);
        riscv_load_kernel(machine, &boot_info, kernel_start_addr,
                          true, NULL);
        kernel_entry = boot_info.image_low_addr;
    }

    if (machine->fdt) {
        fdt_load_addr = riscv_compute_fdt_addr(
            board_template_memmap[BOARD_TEMPLATE_DRAM].base,
            machine->ram_size, machine, &boot_info);
        riscv_load_fdt(fdt_load_addr, machine->fdt);
    }

    /*
     * The reset vector jumps either into firmware (preferred) or directly into
     * the kernel entry if `-bios none` was requested.
     */
    start_addr = firmware_name ? firmware_load_addr : kernel_entry;
    if (!start_addr) {
        start_addr = firmware_load_addr;
    }

    riscv_setup_rom_reset_vec(machine, &s->harts,
                              start_addr,
                              board_template_memmap[BOARD_TEMPLATE_MROM].base,
                              board_template_memmap[BOARD_TEMPLATE_MROM].size,
                              kernel_entry,
                              fdt_load_addr);
}

/*
 * ---------------------------------------------------------------------------
 * 6. Machine assembly
 * ---------------------------------------------------------------------------
 *
 * This is the board's "final assembly line":
 *   CPU topology
 *   memory map
 *   interrupt fabric
 *   basic peripherals
 *   boot handoff
 */

static void board_template_machine_init(MachineState *machine)
{
    RISCVBoardTemplateState *s = RISCV_BOARD_TEMPLATE_MACHINE(machine);
    MemoryRegion *system_memory = get_system_memory();

    /*
     * TODO(cpu-topology):
     *   - keep one socket if you want the simplest board
     *   - if you later add sockets/NUMA, this is where you fan out hart arrays
     */
    object_property_set_str(OBJECT(&s->harts), "cpu-type",
                            machine->cpu_type, &error_abort);
    object_property_set_int(OBJECT(&s->harts), "hartid-base", 0,
                            &error_abort);
    object_property_set_int(OBJECT(&s->harts), "num-harts", machine->smp.cpus,
                            &error_abort);
    object_property_set_int(OBJECT(&s->harts), "resetvec",
                            board_template_memmap[BOARD_TEMPLATE_MROM].base,
                            &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&s->harts), &error_fatal);

    /* Main guest RAM.  machine->ram is prepared by generic machine code. */
    memory_region_add_subregion(system_memory,
                                board_template_memmap[BOARD_TEMPLATE_DRAM].base,
                                machine->ram);

    /* Mask ROM that will host the reset vector blob generated below. */
    /*
     * The board object itself is a MachineState, not a DeviceState.
     * For machine-owned ROM blobs, upstream RISC-V boards pass NULL here.
     */
    memory_region_init_rom(&s->mrom, NULL,
                           "riscv.board.template.mrom",
                           board_template_memmap[BOARD_TEMPLATE_MROM].size,
                           &error_fatal);
    memory_region_add_subregion(system_memory,
                                board_template_memmap[BOARD_TEMPLATE_MROM].base,
                                &s->mrom);

    board_template_create_clint(machine);
    s->plic = board_template_create_plic(machine);
    board_template_create_test(machine);
    board_template_create_uart(machine, s->plic);
    board_template_setup_boot(s);
}

/*
 * ---------------------------------------------------------------------------
 * 7. QOM hooks
 * ---------------------------------------------------------------------------
 *
 * class_init:
 *   machine-wide defaults and board policy
 *
 * instance_init:
 *   initialize child objects and board-local defaults for this instance
 */

static void board_template_machine_instance_init(Object *obj)
{
    RISCVBoardTemplateState *s = RISCV_BOARD_TEMPLATE_MACHINE(obj);

    object_initialize_child(obj, "harts", &s->harts, TYPE_RISCV_HART_ARRAY);
}

static void board_template_machine_class_init(ObjectClass *oc,
                                              const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Inkbottle's RISCV board";
    mc->init = board_template_machine_init;
    mc->max_cpus = 8;
    mc->default_cpu_type = TYPE_RISCV_CPU_BASE;
    mc->default_ram_id = "riscv.board.template.ram";
    mc->default_ram_size = 256 * MiB;
}

static const TypeInfo board_template_machine_typeinfo = {
    .name = TYPE_RISCV_BOARD_TEMPLATE_MACHINE,
    .parent = TYPE_MACHINE,
    .class_init = board_template_machine_class_init,
    .instance_init = board_template_machine_instance_init,
    .instance_size = sizeof(RISCVBoardTemplateState),
};

static void board_template_machine_register_types(void)
{
    type_register_static(&board_template_machine_typeinfo);
}

type_init(board_template_machine_register_types)
