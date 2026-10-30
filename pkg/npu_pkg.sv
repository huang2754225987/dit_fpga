// =============================================================================
// NPU Package - Global parameters, types, and utility functions
// =============================================================================
`ifndef NPU_PKG_SV
`define NPU_PKG_SV

package npu_pkg;

    // =========================================================================
    // Systolic Array
    // =========================================================================
    parameter int ARRAY_M       = 32;
    parameter int ARRAY_N       = 32;
    parameter int NUM_MACS      = ARRAY_M * ARRAY_N; // 256

    // =========================================================================
    // Data Widths
    // =========================================================================
    parameter int DATA_W        = 8;   // INT8 activations & weights
    parameter int ACC_W         = 32;  // INT32 accumulators
    parameter int INTER_W       = 16;  // INT16 intermediates
    parameter int AXI_DATA_W    = 256; // AXI data bus width
    parameter int AXI_ADDR_W    = 32;
    parameter int AXI_STRB_W    = AXI_DATA_W / 8;
    parameter int AXI_ID_W      = 4;
    parameter int AXIL_DATA_W   = 32;
    parameter int AXIL_ADDR_W   = 32;

    // =========================================================================
    // SRAM Configuration
    // =========================================================================
    parameter int SRAM_ADDR_W   = 16;
    parameter int ACT_BANKS     = 4;
    parameter int WGT_BANKS     = 4;
    parameter int ACC_BANKS     = 2;
    parameter int BANK_DEPTH    = 4096;  // entries per bank
    parameter int UCODE_DEPTH   = 1024;

    // L1 Cache: 32-bank structure. With 4096 256-bit words, capacity is 128 KiB
    // (equivalent to 32 banks × 32-bit × 1024 rows).
    parameter int L1_BANKS      = 32;
    parameter int L1_BANK_WIDTH = 32;    // 4 bytes per bank
    parameter int L1_BANK_DEPTH = 4096;
    parameter int L1_ADDR_W     = $clog2(L1_BANK_DEPTH);

    // =========================================================================
    // Model Config (Tiny validation defaults)
    // =========================================================================
    parameter int TINY_HIDDEN   = 64;
    parameter int TINY_HEADS    = 4;
    parameter int TINY_HEAD_DIM = 16;
    parameter int TINY_FFN      = 256;
    parameter int TINY_SEQ_LEN  = 8;
    parameter int TINY_LAYERS   = 1;

    // Tiny LLaMA config
    parameter int TINY_KV_HEADS    = 2;
    parameter int TINY_LLAMA_FFN   = 128;
    parameter int TINY_LLAMA_LAYERS = 4;

    // GPT-2 Small config (for documentation / parameterization):
    // HIDDEN=768, HEADS=12, HEAD_DIM=64, FFN=3072, LAYERS=12, VOCAB=50257

    // =========================================================================
    // Engine IDs
    // =========================================================================
    parameter int NUM_ENGINES = 6;

    typedef enum logic [2:0] {
        ENG_GEMM      = 3'd0,
        ENG_SOFTMAX   = 3'd1,
        ENG_LAYERNORM = 3'd2,
        ENG_GELU      = 3'd3,
        ENG_VEC       = 3'd4,
        ENG_DMA       = 3'd5,
        ENG_RMSNORM   = 3'd6,
        ENG_ROPE      = 3'd7
    } engine_id_t;

    // =========================================================================
    // AXI-Lite Register Map
    // =========================================================================
    parameter logic [AXIL_ADDR_W-1:0] REG_CTRL          = 32'h00;
    parameter logic [AXIL_ADDR_W-1:0] REG_STATUS        = 32'h04;
    parameter logic [AXIL_ADDR_W-1:0] REG_UCODE_BASE    = 32'h08;
    parameter logic [AXIL_ADDR_W-1:0] REG_UCODE_LEN     = 32'h0C;
    parameter logic [AXIL_ADDR_W-1:0] REG_DDR_BASE_ACT  = 32'h10;
    parameter logic [AXIL_ADDR_W-1:0] REG_DDR_BASE_WGT  = 32'h14;
    parameter logic [AXIL_ADDR_W-1:0] REG_DDR_BASE_KV   = 32'h18;
    parameter logic [AXIL_ADDR_W-1:0] REG_DDR_BASE_OUT  = 32'h1C;
    parameter logic [AXIL_ADDR_W-1:0] REG_MODEL_HIDDEN  = 32'h20;
    parameter logic [AXIL_ADDR_W-1:0] REG_MODEL_HEADS   = 32'h24;
    parameter logic [AXIL_ADDR_W-1:0] REG_MODEL_HEAD_DIM= 32'h28;
    parameter logic [AXIL_ADDR_W-1:0] REG_SEQ_LEN       = 32'h2C;
    parameter logic [AXIL_ADDR_W-1:0] REG_TOKEN_IDX     = 32'h30;
    parameter logic [AXIL_ADDR_W-1:0] REG_DEBUG_CTRL    = 32'h34;

    // Graph Mode registers
    parameter logic [AXIL_ADDR_W-1:0] REG_EXEC_MODE     = 32'h38;
    parameter logic [AXIL_ADDR_W-1:0] REG_GRAPH_STATUS  = 32'h3C;
    parameter logic [AXIL_ADDR_W-1:0] REG_GRAPH_PC      = 32'h40;
    parameter logic [AXIL_ADDR_W-1:0] REG_GRAPH_LAST_OP = 32'h44;
    parameter logic [AXIL_ADDR_W-1:0] REG_DDR_BASE_SCALE= 32'h48;
    parameter logic [AXIL_ADDR_W-1:0] REG_CFG_M         = 32'h4C;
    parameter logic [AXIL_ADDR_W-1:0] REG_CFG_N         = 32'h50;
    parameter logic [AXIL_ADDR_W-1:0] REG_CFG_K         = 32'h54;

    // ---- Host-in-the-loop MoE registers ----
    parameter logic [AXIL_ADDR_W-1:0] REG_HOST_CMD          = 32'h58;
    parameter logic [AXIL_ADDR_W-1:0] REG_TRAP_STATUS       = 32'h5C;
    parameter logic [AXIL_ADDR_W-1:0] REG_UCODE_LOAD_ADDR   = 32'h60;
    parameter logic [AXIL_ADDR_W-1:0] REG_UCODE_LOAD_LEN    = 32'h64;
    parameter logic [AXIL_ADDR_W-1:0] REG_RESUME_PC         = 32'h68;
    parameter logic [AXIL_ADDR_W-1:0] REG_ACC_FLUSH_ADDR    = 32'h6C;
    parameter logic [AXIL_ADDR_W-1:0] REG_ACC_FLUSH_LEN     = 32'h70;
    parameter logic [AXIL_ADDR_W-1:0] REG_ACC_ACCUM         = 32'h74;

    // ---- Dequant group-wise asymmetric config (qwen1.5quant) ----
    parameter logic [AXIL_ADDR_W-1:0] REG_DEQUANT_CFG       = 32'h78;  // [2:0]=nbits_code
    parameter logic [AXIL_ADDR_W-1:0] REG_DEQUANT_SCALE_LEN  = 32'h7C; // 128-bit beats to DMA
    // One-write expert precision table entry:
    //   [1:0] = prec, [3:2] = projection, [9:4] = expert index.
    parameter logic [AXIL_ADDR_W-1:0] REG_EXPERT_PREC_CTRL   = 32'h80;
    // Expert descriptor table load (180 entries x 3 x 32-bit words).
    //   REG_EDESC_IDX   = entry index 0..179 (exp_idx*3 + proj) — latches target
    //   REG_EDESC_W0    = [31:0]  weight_base
    //   REG_EDESC_W1    = [31:0]  scale_base
    //   REG_EDESC_W2    = [31] valid, [30:23] tile_bytes/16,
    //                     [22:16]/[5] K tiles, [15:9]/[6] N tiles,
    //                     [8:7] packing, [1:0] precision
    parameter logic [AXIL_ADDR_W-1:0] REG_EDESC_IDX = 32'h84;
    parameter logic [AXIL_ADDR_W-1:0] REG_EDESC_W0  = 32'h88;
    parameter logic [AXIL_ADDR_W-1:0] REG_EDESC_W1  = 32'h8C;
    parameter logic [AXIL_ADDR_W-1:0] REG_EDESC_W2  = 32'h90;

    // HOST_CMD bits
    parameter int HOST_CMD_RESUME     = 0;  // resume from REG_RESUME_PC
    parameter int HOST_CMD_LOAD_UCODE = 1;  // DMA DDR->ucode SRAM
    parameter int HOST_CMD_FLUSH_ACC  = 2;  // flush acc_buffer to DDR
    parameter int HOST_CMD_LOAD_SCALE = 3;  // DMA DDR->scale_zero_mem

    // Execution mode constants
    parameter int EXEC_MODE_LEGACY = 0;
    parameter int EXEC_MODE_GRAPH  = 1;

    // CTRL register bits
    parameter int CTRL_START      = 0;
    parameter int CTRL_SOFT_RESET = 1;

    // STATUS register bits
    parameter int STATUS_DONE  = 0;
    parameter int STATUS_BUSY  = 1;
    parameter int STATUS_ERROR = 2;

    // =========================================================================
    // Utility Functions
    // =========================================================================

    // Saturate to signed N-bit range
    function automatic logic signed [7:0] saturate_i8(input logic signed [31:0] val);
        if (val > 127)
            return 8'sd127;
        else if (val < -128)
            return -8'sd128;
        else
            return val[7:0];
    endfunction

    function automatic logic signed [15:0] saturate_i16(input logic signed [31:0] val);
        if (val > 32767)
            return 16'sd32767;
        else if (val < -32768)
            return -16'sd32768;
        else
            return val[15:0];
    endfunction

    // Requantize: (acc * scale) >> shift with round-to-nearest, clamp to int8
    function automatic logic signed [7:0] requantize(
        input logic signed [31:0] acc,
        input logic        [7:0]  scale,
        input logic        [7:0]  shift
    );
        logic signed [63:0] product;
        logic signed [63:0] rounded;
        logic signed [31:0] shifted;

        product = 64'(acc) * 64'(signed'({1'b0, scale}));

        // Round-to-nearest: add (1 << (shift-1)) before shifting
        if (shift > 0)
            rounded = product + (64'sd1 <<< (shift - 1));
        else
            rounded = product;

        shifted = 32'(rounded >>> shift);
        return saturate_i8(shifted);
    endfunction

    // Saturating 32-bit add
    function automatic logic signed [31:0] sat_add_i32(
        input logic signed [31:0] a,
        input logic signed [31:0] b
    );
        logic signed [32:0] sum;
        sum = 33'(a) + 33'(b);
        if (sum > 33'sd2147483647)
            return 32'sd2147483647;
        else if (sum < -33'sd2147483648)
            return -32'sd2147483648;
        else
            return sum[31:0];
    endfunction

endpackage

`endif
