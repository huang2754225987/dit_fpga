#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vtop.h"
#include "Vtop_top.h"
#include "Vtop_ucode_fetch.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

#define CLK_PERIOD 10
#define MAX_CYCLES 50000

// DDR memory model: 4MB total, 1MB per region
#define DDR_SIZE (4 * 1024 * 1024)
#define REGION_SIZE (1 * 1024 * 1024)
static uint8_t ddr_mem[DDR_SIZE];

// Map AXI physical address to ddr_mem offset
// 0x10xxxxxx (ACT)   -> offset 0x000000
// 0x20xxxxxx (WGT)   -> offset 0x100000
// 0x30xxxxxx (OUT)   -> offset 0x200000
// 0x40xxxxxx (SCALE) -> offset 0x300000
static uint32_t axi_to_ddr(uint32_t addr) {
    uint32_t region = (addr >> 28) & 0xF;
    uint32_t offset = addr & 0x000FFFFF;
    switch (region) {
        case 1: return 0x000000 + offset;
        case 2: return 0x100000 + offset;
        case 3: return 0x200000 + offset;
        case 4: return 0x300000 + offset;
        default:
            printf("[DDR] WARNING: unmapped addr 0x%08X\n", addr);
            return 0;
    }
}

// Initialize DDR with test data
static void init_ddr() {
    memset(ddr_mem, 0, DDR_SIZE);

    // --- Weight region (0x100000-0x1FFFFF): identity matrix tiles ---
    // Each 32x32 INT8 tile = 1024 bytes. Identity: row[i][i]=1, rest=0
    // Fill 16 tiles (enough for all weight DMA reads)
    for (int tile = 0; tile < 16; tile++) {
        uint8_t *base = &ddr_mem[0x100000 + tile * 1024];
        for (int r = 0; r < 32; r++) {
            for (int c = 0; c < 32; c++) {
                base[r * 32 + c] = (r == c) ? 1 : 0;
            }
        }
    }

    // --- Activation region (0x000000-0x0FFFFF): incrementing pattern ---
    // Fill 8KB with values 1..255 repeating (avoid 0 for easy verification)
    for (int i = 0; i < 8192; i++) {
        ddr_mem[0x000000 + i] = (uint8_t)((i % 255) + 1);
    }

    // --- Scale/Gamma region (0x300000-0x3FFFFF): all 0x01 (represents ~1.0) ---
    for (int i = 0; i < 4096; i++) {
        ddr_mem[0x300000 + i] = 0x01;
    }

    // --- RoPE region: cos=0x40(~1.0 in Q7), sin=0x00(~0.0) ---
    // Interleaved cos/sin pairs
    for (int i = 0; i < 2048; i += 2) {
        ddr_mem[0x300000 + 0x2000 + i]     = 0x40; // cos
        ddr_mem[0x300000 + 0x2000 + i + 1] = 0x00; // sin
    }

    printf("[DDR] Initialized: Weight=identity, Act=incr, Gamma=1, RoPE=identity\n");
}

static vluint64_t sim_time = 0;
static Vtop* dut;
static VerilatedVcdC* tfp;

void tick() {
    dut->clk = 0;
    dut->eval();
    if (tfp) tfp->dump(sim_time);
    sim_time += CLK_PERIOD/2;

    dut->clk = 1;
    dut->eval();
    if (tfp) tfp->dump(sim_time);
    sim_time += CLK_PERIOD/2;
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
    printf("[SIM] Loading %d instructions from %s\n", n_instr, filename);

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
    printf("[SIM] UCODE loaded\n");
}

void axil_write(uint32_t addr, uint32_t data) {
    dut->s_axil_awaddr = addr;
    dut->s_axil_awvalid = 1;
    dut->s_axil_wdata = data;
    dut->s_axil_wstrb = 0xF;
    dut->s_axil_wvalid = 1;
    tick();

    int timeout = 100;
    while (!(dut->s_axil_awready && dut->s_axil_wready) && timeout > 0) {
        tick();
        timeout--;
    }
    dut->s_axil_awvalid = 0;
    dut->s_axil_wvalid = 0;

    timeout = 100;
    while (!dut->s_axil_bvalid && timeout > 0) {
        tick();
        timeout--;
    }
    dut->s_axil_bready = 1;
    tick();
    dut->s_axil_bready = 0;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);

    dut = new Vtop;
    tfp = new VerilatedVcdC;
    dut->trace(tfp, 99);
    tfp->open("npu_sim.vcd");

    // Initialize DDR memory with test data
    init_ddr();

    // Initialize inputs
    dut->clk = 0;
    dut->rst_n = 0;
    dut->s_axil_awaddr = 0;
    dut->s_axil_awvalid = 0;
    dut->s_axil_wdata = 0;
    dut->s_axil_wstrb = 0;
    dut->s_axil_wvalid = 0;
    dut->s_axil_bready = 0;
    dut->s_axil_araddr = 0;
    dut->s_axil_arvalid = 0;
    dut->s_axil_rready = 1;
    dut->tb_ucode_wr_en = 0;
    dut->tb_ucode_wr_addr = 0;
    dut->tb_ucode_wr_data = 0;

    // AXI slave defaults
    dut->m_axi_arready = 0;
    dut->m_axi_rvalid = 0;
    dut->m_axi_rdata[0] = 0;
    dut->m_axi_rdata[1] = 0;
    dut->m_axi_rdata[2] = 0;
    dut->m_axi_rdata[3] = 0;
    dut->m_axi_rlast = 0;
    dut->m_axi_rresp = 0;
    dut->m_axi_rid = 0;
    dut->m_axi_awready = 1;
    dut->m_axi_wready = 1;
    dut->m_axi_bvalid = 0;
    dut->m_axi_bresp = 0;
    dut->m_axi_bid = 0;

    // Reset
    printf("[SIM] Resetting...\n");
    reset();

    // Load ucode
    load_ucode("ucode.bin");
    for (int i = 0; i < 5; i++) tick();

    // Configure
    printf("[SIM] Configuring registers...\n");
    axil_write(0x0C, 153);        // UCODE_LEN
    axil_write(0x10, 0x10000000); // DDR_BASE_ACT
    axil_write(0x14, 0x20000000); // DDR_BASE_WGT
    axil_write(0x1C, 0x30000000); // DDR_BASE_OUT
    axil_write(0x48, 0x40000000); // DDR_BASE_SCALE
    axil_write(0x4C, 32);        // CFG_M
    axil_write(0x50, 32);        // CFG_N
    axil_write(0x54, 32);        // CFG_K
    for (int i = 0; i < 5; i++) tick();

    // Start
    printf("[SIM] Starting execution...\n");
    axil_write(0x00, 1); // CTRL.START

    // Main simulation loop
    printf("[SIM] Running (full DDR memory model)...\n");
    int done = 0;

    // AXI read channel state
    int ar_pending = 0;
    int ar_len = 0;
    int ar_cnt = 0;
    int total_ar = 0;
    uint32_t ar_addr = 0;

    // AXI write channel state
    int aw_pending = 0;
    uint32_t aw_addr = 0;
    int w_cnt = 0;
    int total_aw = 0;

    for (int cycle = 0; cycle < MAX_CYCLES && !done; cycle++) {
        // --- Drive AXI read slave signals BEFORE tick ---
        dut->m_axi_arready = !ar_pending;

        if (ar_pending) {
            dut->m_axi_rvalid = 1;
            // Read 128-bit (16 bytes) from DDR at current beat address
            uint32_t beat_addr = ar_addr + ar_cnt * 16;
            uint32_t ddr_off = axi_to_ddr(beat_addr);
            if (ddr_off + 16 <= DDR_SIZE) {
                memcpy(&dut->m_axi_rdata[0], &ddr_mem[ddr_off], 16);
            } else {
                dut->m_axi_rdata[0] = 0;
                dut->m_axi_rdata[1] = 0;
                dut->m_axi_rdata[2] = 0;
                dut->m_axi_rdata[3] = 0;
            }
            dut->m_axi_rlast = (ar_cnt == ar_len) ? 1 : 0;
        } else {
            dut->m_axi_rvalid = 0;
            dut->m_axi_rlast = 0;
        }

        // --- Sample DUT outputs BEFORE tick ---
        int pre_arvalid = dut->m_axi_arvalid;
        int pre_arlen   = dut->m_axi_arlen;
        uint32_t pre_araddr = dut->m_axi_araddr;
        int pre_rready  = dut->m_axi_rready;
        int ar_pending_before = ar_pending;

        // Sample AXI write signals before tick
        int pre_wvalid = dut->m_axi_wvalid;
        int pre_wlast  = dut->m_axi_wlast;
        int pre_awvalid = dut->m_axi_awvalid;
        uint32_t pre_awaddr = dut->m_axi_awaddr;
        int pre_bready = dut->m_axi_bready;

        tick();

        // --- Process handshakes ---

        // AXI B channel: clear bvalid after handshake
        if (dut->m_axi_bvalid && pre_bready) {
            dut->m_axi_bvalid = 0;
        }

        // AXI AW handshake: capture write address
        if (pre_awvalid && dut->m_axi_awready) {
            aw_addr = pre_awaddr;
            aw_pending = 1;
            w_cnt = 0;
            total_aw++;
            printf("[SIM] cycle=%d AW#%d: addr=0x%08X\n", cycle, total_aw, aw_addr);
        }

        // AXI W channel: store data to DDR
        if (pre_wvalid && dut->m_axi_wready) {
            uint32_t beat_addr = aw_addr + w_cnt * 16;
            uint32_t ddr_off = axi_to_ddr(beat_addr);
            if (ddr_off + 16 <= DDR_SIZE) {
                memcpy(&ddr_mem[ddr_off], &dut->m_axi_wdata[0], 16);
            }
            w_cnt++;

            if (pre_wlast) {
                dut->m_axi_bvalid = 1;
                aw_pending = 0;
            }
        }

        // AR handshake: accept read address
        if (!ar_pending_before && pre_arvalid) {
            ar_pending = 1;
            ar_addr = pre_araddr;
            ar_len = pre_arlen;
            ar_cnt = 0;
            total_ar++;
            if (total_ar <= 20 || total_ar % 10 == 0) {
                printf("[SIM] cycle=%d AR#%d: addr=0x%08X len=%d\n",
                       cycle, total_ar, ar_addr, ar_len);
            }
        }

        // R beat: advance or complete burst
        if (ar_pending_before && pre_rready) {
            if (ar_cnt >= ar_len) {
                ar_pending = 0;
            } else {
                ar_cnt++;
            }
        }

        // --- Debug output ---
        if (cycle < 20 || (cycle < 200 && cycle % 50 == 0) || cycle % 500 == 0) {
            auto *ftch = dut->top->u_fetch;
            printf("[%5d] fetch_st=%d pc=%d | arv=%d arr=%d rv=%d | awv=%d wv=%d wl=%d bv=%d\n",
                cycle,
                ftch->state_q, ftch->pc_q,
                (int)dut->m_axi_arvalid, (int)dut->m_axi_arready,
                (int)dut->m_axi_rvalid,
                (int)dut->m_axi_awvalid, (int)dut->m_axi_wvalid,
                (int)dut->m_axi_wlast, (int)dut->m_axi_bvalid);
        }

        // --- Check NPU DONE ---
        if (cycle > 50 && cycle % 50 == 0) {
            dut->s_axil_araddr = 0x04;
            dut->s_axil_arvalid = 1;
        }
        if (dut->s_axil_rvalid && dut->s_axil_rready) {
            uint32_t status = dut->s_axil_rdata;
            dut->s_axil_arvalid = 0;
            if (status & 0x1) {
                printf("[SIM] NPU DONE at cycle %d (status=0x%08X)\n", cycle, status);
                done = 1;
            }
        }
        if (cycle > 200 && cycle % 500 == 0) {
            printf("[SIM] cycle=%d still running... (AR=%d, AW=%d)\n",
                   cycle, total_ar, total_aw);
        }
    }

    // --- Post-sim verification: compare output vs expected ---
    // Weight = identity matrix, so GEMM output should preserve diagonal structure
    // Output region: AXI 0x30004000+ -> ddr_mem[0x204000+]
    printf("\n========== CORRECTNESS VERIFICATION ==========\n");
    printf("[VERIFY] Total AXI: %d reads, %d writes\n", total_ar, total_aw);

    // Build expected output tile (32x32): identity matrix pattern
    // Weight=identity => GEMM(act, identity) preserves diagonal structure
    uint8_t expected[1024];
    memset(expected, 0, 1024);
    for (int r = 0; r < 32; r++) {
        expected[r * 32 + r] = 1;
    }

    // Compare actual vs expected
    uint32_t out_base = 0x204000;
    int mismatch = 0;
    int nonzero_actual = 0;
    int nonzero_expected = 0;
    for (int i = 0; i < 1024; i++) {
        uint8_t actual = ddr_mem[out_base + i];
        if (actual != 0) nonzero_actual++;
        if (expected[i] != 0) nonzero_expected++;
        if (actual != expected[i]) {
            if (mismatch < 16) {
                int row = i / 32, col = i % 32;
                printf("[VERIFY] MISMATCH [%d][%d] (byte %d): expected=0x%02X actual=0x%02X\n",
                       row, col, i, expected[i], actual);
            }
            mismatch++;
        }
    }

    printf("[VERIFY] Output tile (32x32 = 1024 bytes):\n");
    printf("[VERIFY]   Non-zero expected: %d, Non-zero actual: %d\n",
           nonzero_expected, nonzero_actual);
    printf("[VERIFY]   Total mismatches: %d / 1024\n", mismatch);

    // Hex dump: first 4 rows (128 bytes)
    printf("[VERIFY] First 4 rows of output (expect diagonal 0x01):\n");
    for (int r = 0; r < 4; r++) {
        printf("  row[%d]: ", r);
        for (int c = 0; c < 32; c++) {
            printf("%02X ", ddr_mem[out_base + r * 32 + c]);
        }
        printf("\n");
    }

    // Final verdict
    if (mismatch == 0) {
        printf("\n[VERIFY] *** PASS *** Output matches expected (identity matrix pattern)\n");
    } else {
        printf("\n[VERIFY] *** FAIL *** %d mismatches found\n", mismatch);
    }

    printf("===============================================\n");
    printf("[SIM] Simulation finished after %lu ns\n", (unsigned long)sim_time);

    tfp->close();
    delete tfp;
    delete dut;
    return 0;
}
