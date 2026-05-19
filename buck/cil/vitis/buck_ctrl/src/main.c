// Minimal CIL bring-up: open-loop duty, settle, print V_OUT / I_OUT once.
//
// Target: standalone on psu_cortexr5_0 (R5F).  Build in Vitis against the
// platform exported from buck/cil/vivado_proj.

#include "xparameters.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "sleep.h"

// Base address of buck_axi_wrapper.  Vitis generates the macro from the
// Vivado address-editor entry; if the auto-generated name differs, replace
// with the literal (default 0xA0000000 on HPM0_FPD).
#ifdef XPAR_BUCK_AXI_WRAPPER_0_BASEADDR
  #define BUCK_BASE  XPAR_BUCK_AXI_WRAPPER_0_BASEADDR
#else
  #define BUCK_BASE  0xA0000000U
#endif

#define R_CTRL     0x00
#define R_STATUS   0x04
#define R_V_IN     0x08
#define R_G_LOAD   0x0C
#define R_DUTY     0x10
#define R_V_OUT    0x14
#define R_I_OUT    0x18
#define R_PERIOD   0x1C

// Q8.24 -> milli-units (xil_printf has no %f).
static int q24_to_milli(int32_t q) {
    return (int)(((int64_t)q * 1000) >> 24);
}

// Mailbox at a fixed DDR address — read these from XSDB with `mrd`.
// 0x10000000 is well inside DDR on ZynqMP and not used by the BSP/FSBL.
#define MAILBOX_ADDR  0x10000000U
#define MBX_MAGIC     0xC0FFEE00U   // sentinel: "results are valid"

int main(void) {
    // Hold plant in reset, then release.
    Xil_Out32(BUCK_BASE + R_CTRL, 0x2);
    Xil_Out32(BUCK_BASE + R_CTRL, 0x0);

    // Open-loop: D = 0.5 (50 of 100 cycles), enable plant, no IRQ.
    Xil_Out32(BUCK_BASE + R_DUTY, 50);
    Xil_Out32(BUCK_BASE + R_CTRL, 0x1);

    // ~1000 switching periods at fsw = 500 kHz.  Plenty for the LC to settle.
    usleep(2000);

    int32_t  v = (int32_t) Xil_In32(BUCK_BASE + R_V_OUT);
    int32_t  i = (int32_t) Xil_In32(BUCK_BASE + R_I_OUT);
    uint32_t n =           Xil_In32(BUCK_BASE + R_PERIOD);

    // Try UART/JTAG-UART path (in case it works).
    xil_printf("periods elapsed: %u\r\n", n);
    xil_printf("V_OUT = %d mV  (raw 0x%08x)\r\n", q24_to_milli(v), (unsigned)v);
    xil_printf("I_OUT = %d mA  (raw 0x%08x)\r\n", q24_to_milli(i), (unsigned)i);

    // Always write the mailbox so XSDB can read it regardless.
    Xil_Out32(MAILBOX_ADDR + 0x04, (uint32_t)v);
    Xil_Out32(MAILBOX_ADDR + 0x08, (uint32_t)i);
    Xil_Out32(MAILBOX_ADDR + 0x0C, n);
    Xil_Out32(MAILBOX_ADDR + 0x00, MBX_MAGIC);   // magic last — only set when v/i/n are written

    Xil_Out32(BUCK_BASE + R_CTRL, 0x0);
    while (1) { ; }   // park here so XSDB can peek before the CPU runs off
}
