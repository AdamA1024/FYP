// First 1 ms of the controller op-point (V1=100, gamma=0x476 -> R=10, phase=13),
// sampled every 50 us, to check the model for any drift/instability.
#include "Vdab_top.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vdab_top* dut = new Vdab_top;
    dut->V1 = 0x64000000; dut->phase_shift = 13; dut->gamma_in = 0x476;
    dut->rst_n = 0;
    for (int c=0;c<4;c++){dut->clk=0;dut->eval();dut->clk=1;dut->eval();}
    dut->rst_n = 1;
    const double Q24=16777216.0;
    const long N = 50000;          // 1 ms at dt=20ns
    for (long s=0; s<=N; s++){
        if (s%2500==0) printf("  t=%5.0f us  V2=%8.4f V  i_L=%8.4f A\n",
            s*20e-9*1e6,(int32_t)dut->V2_out/Q24,(int32_t)dut->i_L_out/Q24);
        dut->clk=0;dut->eval();dut->clk=1;dut->eval();
    }
    dut->final(); delete dut; return 0;
}
