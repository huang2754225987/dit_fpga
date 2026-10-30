#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vtop.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

#define CLK_PERIOD 10
#define MAX_CYCLES 200

static vluint64_t sim_time = 0;
static Vtop* dut;

void tick() {
    dut->clk = 0; dut->eval(); sim_time += CLK_PERIOD/2;
    dut->clk = 1; dut->eval(); sim_time += CLK_PERIOD/2;
}

void reset() {
    dut->rst_n = 0;
    for (int i = 0; i < 10; i++) tick();
    dut->rst_n = 1;
    for (int i = 0; i < 5; i++) tick();
}

void load_ucode(const char* filename) {
    FILE* f = fopen(filename, "rb");
    if (!f) { printf("ERROR: cannot open %s\n", filename); exit(1); }
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    unsigned char* buf = (unsigned char*)malloc(size);
    fread(buf, 1, size, f);
    fclose(f);
    int n_instr = size / 4;
    printf("[SIM] Loading %d instructions\n", n_instr);
    for (int i = 0; i < n_instr; i++) {
        dut->tb_ucode_wr_en = 1;
        dut->tb_ucode_wr_addr = i;
        dut->tb_ucode_wr_data =
            (buf[i*4+3] << 24) | (buf[i*4+2] << 16) |
            (buf[i*4+1] << 8)  | buf[i*4+0];
        tick();
    }
    dut->tb_ucode_wr_en = 0;
    free(buf);
    
    // Print instructions
    fseek(f, 0, SEEK_SET);
    f = fopen(filename, "rb");
    fseek(f, 0, SEEK_SET);
    buf = (unsigned char*)malloc(size);
    fread(buf, 1, size, f);
    fclose(f);
    printf("[SIM] Instructions:\n");
    for (int i = 0; i < n_instr; i++) {
        uint32_t instr = (buf[i*4+3] << 24) | (buf[i*4+2] << 16) | (buf[i*4+1] << 8) | buf[i*4+0];
        uint8_t opcode = (instr >> 28) & 0xF;
        uint8_t vd = (instr >> 24) & 0xF;
        uint8_t vs1 = (instr >> 20) & 0xF;
        uint8_t vs2 = (instr >> 16) & 0xF;
        uint16_t imm = instr & 0xFFFF;
        const char* op_names[] = {"NOP","VLD","VST","GLD.W","GLD.A","GEMM","SOFTMAX","RMSNORM","ROPE","SILU","ADD","MUL","DMA_RD","DMA_WR","BARRIER","END"};
        printf("  [%2d] 0x%08X  %s vd=%d vs1=%d vs2=%d imm=0x%04X\n", i, instr, op_names[opcode], vd, vs1, vs2, imm);
    }
    free(buf);
}

void axil_write(uint32_t addr, uint32_t data) {
    dut->s_axil_awaddr = addr;
    dut->s_axil_awvalid = 1;
    dut->s_axil_wdata = data;
    dut->s_axil_wstrb = 0xF;
    dut->s_axil_wvalid = 1;
    tick();
    int timeout = 100;
    while (!(dut->s_axil_awready && dut->s_axil_wready) && timeout > 0) { tick(); timeout--; }
    dut->s_axil_awvalid = 0;
    dut->s_axil_wvalid = 0;
    timeout = 100;
    while (!dut->s_axil_bvalid && timeout > 0) { tick(); timeout--; }
    dut->s_axil_bready = 1;
    tick();
    dut->s_axil_bready = 0;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtop;

    dut->clk = 0; dut->rst_n = 0;
    dut->s_axil_awaddr = 0; dut->s_axil_awvalid = 0;
    dut->s_axil_wdata = 0; dut->s_axil_wstrb = 0;
    dut->s_axil_wvalid = 0; dut->s_axil_bready = 0;
    dut->s_axil_araddr = 0; dut->s_axil_arvalid = 0;
    dut->s_axil_rready = 1;
    dut->tb_ucode_wr_en = 0; dut->tb_ucode_wr_addr = 0; dut->tb_ucode_wr_data = 0;
    dut->m_axi_arready = 1; dut->m_axi_rvalid = 0;
    dut->m_axi_rdata[0] = 0; dut->m_axi_rdata[1] = 0;
    dut->m_axi_rdata[2] = 0; dut->m_axi_rdata[3] = 0;
    dut->m_axi_rlast = 0; dut->m_axi_rresp = 0; dut->m_axi_rid = 0;
    dut->m_axi_awready = 1; dut->m_axi_wready = 1;
    dut->m_axi_bvalid = 0; dut->m_axi_bresp = 0; dut->m_axi_bid = 0;

    reset();
    load_ucode("ucode.bin");
    for (int i = 0; i < 5; i++) tick();

    axil_write(0x0C, 12);
    axil_write(0x10, 0x10000000);
    axil_write(0x14, 0x20000000);
    axil_write(0x1C, 0x30000000);
    axil_write(0x48, 0x40000000);
    axil_write(0x4C, 32);
    axil_write(0x50, 32);
    axil_write(0x54, 32);
    for (int i = 0; i < 5; i++) tick();

    printf("[SIM] Starting execution...\n");
    axil_write(0x00, 1);

    int ar_pending = 0, ar_len = 0, ar_cnt = 0;

    for (int cycle = 0; cycle < MAX_CYCLES; cycle++) {
        if (dut->m_axi_arvalid && dut->m_axi_arready) {
            ar_pending = 1; ar_len = dut->m_axi_arlen; ar_cnt = 0;
            dut->m_axi_arready = 0;
            printf("[SIM] cycle=%d AXI AR: addr=0x%08X len=%d\n", cycle, dut->m_axi_araddr, ar_len);
        }
        if (ar_pending) {
            dut->m_axi_rvalid = 1;
            dut->m_axi_rdata[0] = 0xCAFE0000 | ar_cnt;
            dut->m_axi_rdata[1] = 0xCAFE0000 | ar_cnt;
            dut->m_axi_rdata[2] = 0xCAFE0000 | ar_cnt;
            dut->m_axi_rdata[3] = 0xCAFE0000 | ar_cnt;
            dut->m_axi_rlast = (ar_cnt == ar_len) ? 1 : 0;
            if (dut->m_axi_rvalid && dut->m_axi_rready) {
                if (ar_cnt == ar_len) {
                    ar_pending = 0; dut->m_axi_rvalid = 0; dut->m_axi_rlast = 0; dut->m_axi_arready = 1;
                } else { ar_cnt++; }
            }
        }
        if (dut->m_axi_wvalid && dut->m_axi_wready && dut->m_axi_wlast) dut->m_axi_bvalid = 1;
        if (dut->m_axi_bvalid && dut->m_axi_bready) dut->m_axi_bvalid = 0;
        tick();
    }

    printf("[SIM] Done\n");
    delete dut;
    return 0;
}
