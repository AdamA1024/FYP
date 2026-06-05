// Hypothesis test: is gamma reaching the datapath on hardware? Force gamma_in=0
// (what a broken AXI reg1 plumbing would give) and watch V2 over a long run.
#include "Vdab_top.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vdab_top* dut = new Vdab_top;
    dut->V1 = 0x64000000; dut->phase_shift = 13;
    dut->gamma_in = (argc > 1) ? (uint32_t)strtoul(argv[1],0,0) : 0x0;  // default 0
    dut->rst_n = 0;
    for (int c=0;c<4;c++){dut->clk=0;dut->eval();dut->clk=1;dut->eval();}
    dut->rst_n = 1;
    const double Q24=16777216.0;
    for (long s=0; s<10000000; s++){          // 200 ms at dt=20ns
        dut->clk=0;dut->eval();dut->clk=1;dut->eval();
        if (s%1000000==0) printf("  t=%5.1f ms  V2=%9.3f V  i_L=%9.3f A\n",
            s*20e-9*1e3,(int32_t)dut->V2_out/Q24,(int32_t)dut->i_L_out/Q24);
    }
    dut->final(); delete dut; return 0;
}
