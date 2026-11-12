/*
 * QEMU RISC-V Cheshire board (PULP platform / ETH Zurich)
 *
 * Memory map and devices follow the Cheshire device tree
 * (cpu-benchmarks-collection/target/cheshire/ports/rtems/cheshire.dts).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_RISCV_CHESHIRE_H
#define HW_RISCV_CHESHIRE_H

#include "hw/core/boards.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/core/sysbus.h"

#define CHESHIRE_CPUS_MAX 1

#define TYPE_CHESHIRE_MACHINE MACHINE_TYPE_NAME("cheshire")
typedef struct CheshireState CheshireState;
DECLARE_INSTANCE_CHECKER(CheshireState, CHESHIRE_MACHINE,
                         TYPE_CHESHIRE_MACHINE)

struct CheshireState {
    /*< private >*/
    MachineState parent;

    /*< public >*/
    RISCVHartArrayState soc;
    DeviceState *plic;
    MemoryRegion unmapped;
    MemoryRegion axi_llc;
    uint32_t axi_llc_regs[0x5000 / 4];
    bool htif;
};

enum {
    CHESHIRE_MROM,
    CHESHIRE_UNMAPPED,
    CHESHIRE_CLINT,
    CHESHIRE_AXI_LLC,
    CHESHIRE_UART0,
    CHESHIRE_PLIC,
    CHESHIRE_DRAM,
};

/* From cheshire.dts */
#define CHESHIRE_TIMEBASE_FREQ      32768
#define CHESHIRE_CPU_CLOCK_FREQ     200000000
#define CHESHIRE_UART0_CLOCK_FREQ   200000000
#define CHESHIRE_UART0_SPEED        115200
#define CHESHIRE_UART0_IRQ          1
#define CHESHIRE_UART0_REG_SHIFT    2
#define CHESHIRE_PLIC_NDEV          51
#define CHESHIRE_PLIC_MAX_PRIORITY  7
#define CHESHIRE_DRAM_DEFAULT_SIZE  0x04000000

/* Standard SiFive PLIC register layout */
#define CHESHIRE_PLIC_PRIORITY_BASE  0x00
#define CHESHIRE_PLIC_PENDING_BASE   0x1000
#define CHESHIRE_PLIC_ENABLE_BASE    0x2000
#define CHESHIRE_PLIC_ENABLE_STRIDE  0x80
#define CHESHIRE_PLIC_CONTEXT_BASE   0x200000
#define CHESHIRE_PLIC_CONTEXT_STRIDE 0x1000

#endif
