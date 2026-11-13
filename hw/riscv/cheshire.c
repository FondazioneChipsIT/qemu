/*
 * QEMU RISC-V Cheshire board (PULP platform / ETH Zurich)
 *
 * Only the devices described by the Cheshire device tree are modelled
 * (cpu-benchmarks-collection/target/cheshire/ports/rtems/cheshire.dts):
 *
 *   0x00001000  MROM        QEMU reset vector (not in the DTS, needed to boot)
 *   0x02040000  CLINT       riscv,clint0 (ACLINT SWI + MTIMER, SiFive layout),
 *                           timebase 32768 Hz
 *   0x03001000  AXI LLC     eth,axi-llc: configuration registers only, modelled
 *                           as plain read/write storage (no caching effect).
 *                           Its DTS size (0x5000) overlaps UART0: mapped at
 *                           lower priority so the UART wins.
 *   0x03002000  UART0       ns16550a, reg-shift 2, 200 MHz, PLIC irq 1
 *   0x04000000  PLIC        riscv,plic0, 51 sources, max priority 7
 *   0xA0000000  DRAM        machine->ram (-m, memory-backend=...)
 *                           NOTE: the Cheshire hardware has DRAM at
 *                           0x80000000; it is moved here to match the RTEMS
 *                           link address and the GVSoC magia_v2 UMA host base
 *                           (UMA_HOST_ADDR_START in pulp/chips/magia_v2/arch.py).
 *
 * Every other access in [0x10000, 0x80000000) - i.e. the Cheshire
 * peripherals that are not in the DTS - hits a catch-all region that reads
 * as zero, ignores writes and logs the access (enable with "-d unimp").
 * [0x80000000, 0xA0000000) is not mapped: accesses there raise an access
 * fault.
 *
 * Optional HTIF ("-M cheshire,htif=on"): tohost/fromhost are taken from the
 * ELF symbols of -bios/-kernel, as on "spike". It is not part of the DTS and
 * is meant for test termination (power-off / exit code). Its console, if
 * used, goes to the second serial port (-serial ... -serial ...).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/sysbus.h"
#include "hw/char/serial-mm.h"
#include "hw/char/riscv_htif.h"
#include "hw/intc/riscv_aclint.h"
#include "hw/intc/sifive_plic.h"
#include "hw/riscv/boot.h"
#include "hw/riscv/cheshire.h"
#include "hw/riscv/riscv_hart.h"
#include "target/riscv/cpu.h"
#include "chardev/char.h"
#include "system/device_tree.h"
#include "system/system.h"
#include <libfdt.h>

static const MemMapEntry cheshire_memmap[] = {
    [CHESHIRE_MROM]     = {     0x1000,      0xf000 },
    [CHESHIRE_UNMAPPED] = {    0x10000, 0x7fff0000 },
    [CHESHIRE_CLINT]    = {  0x2040000,    0x40000 },
    [CHESHIRE_AXI_LLC]  = {  0x3001000,     0x5000 },
    [CHESHIRE_UART0]    = {  0x3002000,     0x1000 },
    [CHESHIRE_PLIC]     = {  0x4000000,  0x4000000 },
    [CHESHIRE_DRAM]     = { 0xA0000000, 0x1FF00000 },
};

/*
 * Catch-all for the Cheshire address space that the DTS does not describe.
 */
static uint64_t cheshire_unmapped_read(void *opaque, hwaddr addr,
                                       unsigned size)
{
    qemu_log_mask(LOG_UNIMP,
                  "cheshire: unexpected read  at 0x%08" HWADDR_PRIx
                  " (size %u): device not in the Cheshire DTS\n",
                  addr + cheshire_memmap[CHESHIRE_UNMAPPED].base, size);
    return 0;
}

static void cheshire_unmapped_write(void *opaque, hwaddr addr, uint64_t val,
                                    unsigned size)
{
    qemu_log_mask(LOG_UNIMP,
                  "cheshire: unexpected write at 0x%08" HWADDR_PRIx
                  " (size %u, value 0x%" PRIx64 "): device not in the "
                  "Cheshire DTS\n",
                  addr + cheshire_memmap[CHESHIRE_UNMAPPED].base, size, val);
}

static const MemoryRegionOps cheshire_unmapped_ops = {
    .read = cheshire_unmapped_read,
    .write = cheshire_unmapped_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

/*
 * AXI LLC configuration registers: plain 32-bit storage. QEMU has no cache
 * model, so writing the configuration has no functional effect.
 */
static uint64_t cheshire_axi_llc_read(void *opaque, hwaddr addr, unsigned size)
{
    CheshireState *s = opaque;

    return s->axi_llc_regs[addr >> 2];
}

static void cheshire_axi_llc_write(void *opaque, hwaddr addr, uint64_t val,
                                   unsigned size)
{
    CheshireState *s = opaque;

    s->axi_llc_regs[addr >> 2] = val;
}

static const MemoryRegionOps cheshire_axi_llc_ops = {
    .read = cheshire_axi_llc_read,
    .write = cheshire_axi_llc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/*
 * Device tree: same nodes and values as cheshire.dts. Only the memory size
 * follows -m, so that the guest sees the RAM QEMU actually provides.
 */
static void create_fdt(CheshireState *s, const MemMapEntry *memmap)
{
    MachineState *ms = MACHINE(s);
    void *fdt;
    int fdt_size;
    uint64_t mem_addr = memmap[CHESHIRE_DRAM].base;
    uint64_t mem_size = ms->ram_size;
    uint32_t intc_phandle = 1, plic_phandle = 2;
    g_autofree char *mem_name = NULL;
    g_autofree char *clint_name = NULL;
    g_autofree char *llc_name = NULL;
    g_autofree char *plic_name = NULL;
    g_autofree char *uart_name = NULL;
    static const char * const cpu_compat[] = { "eth,ariane", "riscv" };
    static const char * const soc_compat[] = {
        "eth,cheshire-bare-soc", "simple-bus"
    };

    fdt = ms->fdt = create_device_tree(&fdt_size);
    if (!fdt) {
        error_report("create_device_tree() failed");
        exit(1);
    }

    qemu_fdt_setprop_cell(fdt, "/", "#address-cells", 0x2);
    qemu_fdt_setprop_cell(fdt, "/", "#size-cells", 0x2);
    qemu_fdt_setprop_string(fdt, "/", "compatible", "eth,cheshire-dev");
    qemu_fdt_setprop_string(fdt, "/", "model", "eth,cheshire");

    uart_name = g_strdup_printf("/soc/serial@%" HWADDR_PRIx,
                                memmap[CHESHIRE_UART0].base);

    qemu_fdt_add_subnode(fdt, "/aliases");
    qemu_fdt_setprop_string(fdt, "/aliases", "serial0", uart_name);

    qemu_fdt_add_subnode(fdt, "/chosen");
    qemu_fdt_setprop_string(fdt, "/chosen", "stdout-path", uart_name);

    mem_name = g_strdup_printf("/memory@%" PRIx64, mem_addr);
    qemu_fdt_add_subnode(fdt, mem_name);
    qemu_fdt_setprop_string(fdt, mem_name, "device_type", "memory");
    qemu_fdt_setprop_cells(fdt, mem_name, "reg",
                           mem_addr >> 32, mem_addr, mem_size >> 32, mem_size);

    qemu_fdt_add_subnode(fdt, "/cpus");
    qemu_fdt_setprop_cell(fdt, "/cpus", "#address-cells", 0x1);
    qemu_fdt_setprop_cell(fdt, "/cpus", "#size-cells", 0x0);
    qemu_fdt_setprop_cell(fdt, "/cpus", "timebase-frequency",
                          CHESHIRE_TIMEBASE_FREQ);

    qemu_fdt_add_subnode(fdt, "/cpus/cpu@0");
    qemu_fdt_setprop_string(fdt, "/cpus/cpu@0", "device_type", "cpu");
    qemu_fdt_setprop_string(fdt, "/cpus/cpu@0", "status", "okay");
    qemu_fdt_setprop_string_array(fdt, "/cpus/cpu@0", "compatible",
                                  (char **)&cpu_compat,
                                  ARRAY_SIZE(cpu_compat));
    qemu_fdt_setprop_cell(fdt, "/cpus/cpu@0", "clock-frequency",
                          CHESHIRE_CPU_CLOCK_FREQ);
    qemu_fdt_setprop_string(fdt, "/cpus/cpu@0", "riscv,isa", "rv64imafdc");
    qemu_fdt_setprop_string(fdt, "/cpus/cpu@0", "mmu-type", "riscv,sv39");
    qemu_fdt_setprop(fdt, "/cpus/cpu@0", "tlb-split", NULL, 0);
    qemu_fdt_setprop_cell(fdt, "/cpus/cpu@0", "reg", 0);

    qemu_fdt_add_subnode(fdt, "/cpus/cpu@0/interrupt-controller");
    qemu_fdt_setprop_cell(fdt, "/cpus/cpu@0/interrupt-controller",
                          "#address-cells", 0);
    qemu_fdt_setprop_cell(fdt, "/cpus/cpu@0/interrupt-controller",
                          "#interrupt-cells", 1);
    qemu_fdt_setprop(fdt, "/cpus/cpu@0/interrupt-controller",
                     "interrupt-controller", NULL, 0);
    qemu_fdt_setprop_string(fdt, "/cpus/cpu@0/interrupt-controller",
                            "compatible", "riscv,cpu-intc");
    qemu_fdt_setprop_cell(fdt, "/cpus/cpu@0/interrupt-controller",
                          "phandle", intc_phandle);

    qemu_fdt_add_subnode(fdt, "/soc");
    qemu_fdt_setprop_cell(fdt, "/soc", "#address-cells", 0x2);
    qemu_fdt_setprop_cell(fdt, "/soc", "#size-cells", 0x2);
    qemu_fdt_setprop_string_array(fdt, "/soc", "compatible",
                                  (char **)&soc_compat,
                                  ARRAY_SIZE(soc_compat));
    qemu_fdt_setprop(fdt, "/soc", "ranges", NULL, 0);

    clint_name = g_strdup_printf("/soc/clint@%" HWADDR_PRIx,
                                 memmap[CHESHIRE_CLINT].base);
    qemu_fdt_add_subnode(fdt, clint_name);
    qemu_fdt_setprop_string(fdt, clint_name, "compatible", "riscv,clint0");
    qemu_fdt_setprop_cells(fdt, clint_name, "interrupts-extended",
                           intc_phandle, IRQ_M_SOFT,
                           intc_phandle, IRQ_M_TIMER);
    qemu_fdt_setprop_string(fdt, clint_name, "reg-names", "control");
    qemu_fdt_setprop_cells(fdt, clint_name, "reg",
                           0x0, memmap[CHESHIRE_CLINT].base,
                           0x0, memmap[CHESHIRE_CLINT].size);

    llc_name = g_strdup_printf("/soc/axi_llc@%" HWADDR_PRIx,
                               memmap[CHESHIRE_AXI_LLC].base);
    qemu_fdt_add_subnode(fdt, llc_name);
    qemu_fdt_setprop_string(fdt, llc_name, "compatible", "eth,axi-llc");
    qemu_fdt_setprop_cells(fdt, llc_name, "reg",
                           0x0, memmap[CHESHIRE_AXI_LLC].base,
                           0x0, memmap[CHESHIRE_AXI_LLC].size);

    plic_name = g_strdup_printf("/soc/interrupt-controller@%" HWADDR_PRIx,
                                memmap[CHESHIRE_PLIC].base);
    qemu_fdt_add_subnode(fdt, plic_name);
    qemu_fdt_setprop_string(fdt, plic_name, "compatible", "riscv,plic0");
    qemu_fdt_setprop_cell(fdt, plic_name, "#address-cells", 0);
    qemu_fdt_setprop_cell(fdt, plic_name, "#interrupt-cells", 1);
    qemu_fdt_setprop(fdt, plic_name, "interrupt-controller", NULL, 0);
    qemu_fdt_setprop_cells(fdt, plic_name, "interrupts-extended",
                           intc_phandle, IRQ_M_EXT,
                           intc_phandle, IRQ_S_EXT);
    qemu_fdt_setprop_cell(fdt, plic_name, "riscv,max-priority",
                          CHESHIRE_PLIC_MAX_PRIORITY);
    qemu_fdt_setprop_cell(fdt, plic_name, "riscv,ndev", CHESHIRE_PLIC_NDEV);
    qemu_fdt_setprop_cells(fdt, plic_name, "reg",
                           0x0, memmap[CHESHIRE_PLIC].base,
                           0x0, memmap[CHESHIRE_PLIC].size);
    qemu_fdt_setprop_cell(fdt, plic_name, "phandle", plic_phandle);

    qemu_fdt_add_subnode(fdt, uart_name);
    qemu_fdt_setprop_string(fdt, uart_name, "compatible", "ns16550a");
    qemu_fdt_setprop_cell(fdt, uart_name, "clock-frequency",
                          CHESHIRE_UART0_CLOCK_FREQ);
    qemu_fdt_setprop_cell(fdt, uart_name, "current-speed",
                          CHESHIRE_UART0_SPEED);
    qemu_fdt_setprop_cell(fdt, uart_name, "interrupt-parent", plic_phandle);
    qemu_fdt_setprop_cell(fdt, uart_name, "interrupts", CHESHIRE_UART0_IRQ);
    qemu_fdt_setprop_cells(fdt, uart_name, "reg",
                           0x0, memmap[CHESHIRE_UART0].base,
                           0x0, memmap[CHESHIRE_UART0].size);
    qemu_fdt_setprop_cell(fdt, uart_name, "reg-shift",
                          CHESHIRE_UART0_REG_SHIFT);
    qemu_fdt_setprop_cell(fdt, uart_name, "reg-io-width", 4);
}

static void cheshire_board_init(MachineState *machine)
{
    const MemMapEntry *memmap = cheshire_memmap;
    CheshireState *s = CHESHIRE_MACHINE(machine);
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *mask_rom = g_new(MemoryRegion, 1);
    hwaddr firmware_end_addr = memmap[CHESHIRE_DRAM].base;
    hwaddr firmware_load_addr = memmap[CHESHIRE_DRAM].base;
    symbol_fn_t sym_cb = s->htif ? htif_symbol_callback : NULL;
    g_autofree char *plic_hart_config = NULL;
    vaddr kernel_start_addr;
    char *firmware_name;
    uint64_t fdt_load_addr;
    uint64_t kernel_entry;
    RISCVBootInfo boot_info;
    int fdt_size;

    if (machine->smp.cpus > CHESHIRE_CPUS_MAX) {
        error_report("Cheshire has a single hart (-smp %u requested)",
                     machine->smp.cpus);
        exit(1);
    }

    /* Hart */
    object_initialize_child(OBJECT(machine), "soc", &s->soc,
                            TYPE_RISCV_HART_ARRAY);
    object_property_set_str(OBJECT(&s->soc), "cpu-type", machine->cpu_type,
                            &error_abort);
    object_property_set_int(OBJECT(&s->soc), "hartid-base", 0, &error_abort);
    object_property_set_int(OBJECT(&s->soc), "num-harts", machine->smp.cpus,
                            &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&s->soc), &error_fatal);

    /* Catch-all for the address space not described by the DTS */
    memory_region_init_io(&s->unmapped, OBJECT(machine),
                          &cheshire_unmapped_ops, s, "cheshire.unmapped",
                          memmap[CHESHIRE_UNMAPPED].size);
    memory_region_add_subregion_overlap(system_memory,
                                        memmap[CHESHIRE_UNMAPPED].base,
                                        &s->unmapped, -1000);

    /* CLINT: SWI at +0, MTIMER at +0x4000 (mtimecmp +0x4000, mtime +0xbff8) */
    riscv_aclint_swi_create(system_memory, memmap[CHESHIRE_CLINT].base, 0,
                            machine->smp.cpus, false);
    riscv_aclint_mtimer_create(system_memory, memmap[CHESHIRE_CLINT].base +
                                   RISCV_ACLINT_SWI_SIZE,
                               RISCV_ACLINT_DEFAULT_MTIMER_SIZE, 0,
                               machine->smp.cpus,
                               RISCV_ACLINT_DEFAULT_MTIMECMP,
                               RISCV_ACLINT_DEFAULT_MTIME,
                               CHESHIRE_TIMEBASE_FREQ, true);

    /*
     * AXI LLC configuration registers. The DTS gives it 0x5000 bytes, which
     * overlaps UART0 at 0x3002000: map it below the other devices so that
     * the UART (and anything else in that window) takes precedence.
     */
    memory_region_init_io(&s->axi_llc, OBJECT(machine), &cheshire_axi_llc_ops,
                          s, "cheshire.axi_llc", memmap[CHESHIRE_AXI_LLC].size);
    memory_region_add_subregion_overlap(system_memory,
                                        memmap[CHESHIRE_AXI_LLC].base,
                                        &s->axi_llc, -1);

    /* PLIC */
    plic_hart_config = riscv_plic_hart_config_string(machine->smp.cpus);
    s->plic = sifive_plic_create(system_memory,
                                 memmap[CHESHIRE_PLIC].base, plic_hart_config,
                                 machine->smp.cpus, 0,
                                 CHESHIRE_PLIC_NDEV + 1,
                                 CHESHIRE_PLIC_MAX_PRIORITY,
                                 CHESHIRE_PLIC_PRIORITY_BASE,
                                 CHESHIRE_PLIC_PENDING_BASE,
                                 CHESHIRE_PLIC_ENABLE_BASE,
                                 CHESHIRE_PLIC_ENABLE_STRIDE,
                                 CHESHIRE_PLIC_CONTEXT_BASE,
                                 CHESHIRE_PLIC_CONTEXT_STRIDE,
                                 memmap[CHESHIRE_PLIC].size);

    /* UART0: ns16550a, 32-bit registers (reg-shift 2) */
    serial_mm_init(system_memory, memmap[CHESHIRE_UART0].base,
                   CHESHIRE_UART0_REG_SHIFT,
                   qdev_get_gpio_in(s->plic, CHESHIRE_UART0_IRQ),
                   CHESHIRE_UART0_CLOCK_FREQ / 16, serial_hd(0),
                   DEVICE_LITTLE_ENDIAN);

    /* DRAM */
    memory_region_add_subregion(system_memory, memmap[CHESHIRE_DRAM].base,
                                machine->ram);

    /* Boot ROM (reset vector) */
    memory_region_init_rom(mask_rom, NULL, "riscv.cheshire.mrom",
                           memmap[CHESHIRE_MROM].size, &error_fatal);
    memory_region_add_subregion(system_memory, memmap[CHESHIRE_MROM].base,
                                mask_rom);

    riscv_boot_info_init(&boot_info, &s->soc);

    /* Firmware (-bios) */
    firmware_name = riscv_find_firmware(machine->firmware,
                        riscv_default_firmware_name(&s->soc));
    if (firmware_name) {
        firmware_end_addr = riscv_load_firmware(machine, &boot_info,
                                                firmware_name,
                                                &firmware_load_addr, sym_cb);
        g_free(firmware_name);
    }

    /* Device tree: built-in (as cheshire.dts) or -dtb */
    if (machine->dtb) {
        machine->fdt = load_device_tree(machine->dtb, &fdt_size);
        if (!machine->fdt) {
            error_report("load_device_tree() failed");
            exit(1);
        }
    } else {
        create_fdt(s, memmap);
    }

    /* Kernel (-kernel) */
    if (machine->kernel_filename) {
        kernel_start_addr = riscv_calc_kernel_start_addr(&boot_info,
                                                         firmware_end_addr);
        riscv_load_kernel(machine, &boot_info, kernel_start_addr, true,
                          sym_cb);
        kernel_entry = boot_info.image_low_addr;
    } else {
        kernel_entry = 0;
    }

    fdt_load_addr = riscv_compute_fdt_addr(memmap[CHESHIRE_DRAM].base,
                                           machine->ram_size, machine,
                                           &boot_info);
    riscv_load_fdt(fdt_load_addr, machine->fdt);

    riscv_setup_rom_reset_vec(machine, &s->soc, firmware_load_addr,
                              memmap[CHESHIRE_MROM].base,
                              memmap[CHESHIRE_MROM].size, kernel_entry,
                              fdt_load_addr);

    /* Optional HTIF, located through the tohost/fromhost ELF symbols */
    if (s->htif) {
        htif_mm_init(system_memory, serial_hd(1), 0, false);
    }
}

static bool cheshire_get_htif(Object *obj, Error **errp)
{
    return CHESHIRE_MACHINE(obj)->htif;
}

static void cheshire_set_htif(Object *obj, bool value, Error **errp)
{
    CHESHIRE_MACHINE(obj)->htif = value;
}

static void cheshire_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "RISC-V Cheshire board (PULP/ETH, as described by cheshire.dts)";
    mc->init = cheshire_board_init;
    mc->max_cpus = CHESHIRE_CPUS_MAX;
    mc->default_cpu_type = TYPE_RISCV_CPU_BASE;
    mc->default_ram_size = CHESHIRE_DRAM_DEFAULT_SIZE;
    mc->default_ram_id = "riscv.cheshire.ram";

    object_class_property_add_bool(oc, "htif", cheshire_get_htif,
                                   cheshire_set_htif);
    object_class_property_set_description(oc, "htif",
        "Enable HTIF at the tohost/fromhost ELF symbols (test power-off / "
        "exit code; console on the second serial port). Not in the DTS.");
}

static const TypeInfo cheshire_machine_typeinfo = {
    .name          = TYPE_CHESHIRE_MACHINE,
    .parent        = TYPE_MACHINE,
    .class_init    = cheshire_machine_class_init,
    .instance_size = sizeof(CheshireState),
};

static void cheshire_machine_init_register_types(void)
{
    type_register_static(&cheshire_machine_typeinfo);
}

type_init(cheshire_machine_init_register_types)
