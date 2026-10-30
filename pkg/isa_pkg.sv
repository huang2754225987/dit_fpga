// =============================================================================
// ISA Package - 32-bit Fine-Grained Instruction Format
// =============================================================================
`ifndef ISA_PKG_SV
`define ISA_PKG_SV

package isa_pkg;

    parameter int INSTR_W = 32;

    // =========================================================================
    // Opcodes (4-bit, 16 instructions max)
    // =========================================================================
    typedef enum logic [3:0] {
        OP_NOP      = 4'h0,  // NOP / extended ops (sub-opcode in imm)
        OP_VLD      = 4'h1,   // SRAM -> PostProc RF; vs1[3] enables token stride
        OP_VST      = 4'h2,   // PostProc RF -> SRAM; vd[3] enables token stride
        OP_GLD_W    = 4'h3,   // SRAM -> GEMM Weight RF
        OP_GLD_A    = 4'h4,   // SRAM -> GEMM Activation RF
        OP_GEMM     = 4'h5,   // Weight RF x Act RF -> Accumulator
        OP_SOFTMAX  = 4'h6,   // online state slots 0..63 legacy; vs1[3] marks extended 0..127
        OP_RMSNORM  = 4'h7,   // imm[15:14]: legacy/accumulate/finalize/apply
        OP_ROPE     = 4'h8,
        OP_SILU     = 4'h9,
        OP_ADD      = 4'hA,
        OP_MUL      = 4'hB,
        OP_DMA_RD   = 4'hC,   // DDR -> SRAM (vs2: expert or token length, selected by vs1[3])
        OP_DMA_WR   = 4'hD,   // SRAM -> DDR
        OP_BARRIER  = 4'hE,
        OP_END      = 4'hF
    } opcode_t;

    // =========================================================================
    // Extended NOP sub-opcodes (OP_NOP with imm[15:12] = sub-opcode)
    // =========================================================================
    typedef enum logic [3:0] {
        NOP_NOP   = 4'h0,  // genuine NOP
        NOP_TOPK  = 4'h1,  // TOPK: trigger TopK unit
        NOP_SCALE = 4'h2,  // SCALE: trigger per-expert scale/zero DMA load (expert mode); imm[5:0]=N-tile
        NOP_GROUP_WEIGHT = 4'h3, // broadcast route-group row's cached weight to PRF vd
        NOP_LOOP    = 4'h4, // hardware loop: imm[11:0]=iter_count, vs1=body_len, vd[3:2]=stride_code
        NOP_AC2V    = 4'h5, // ACC row -> PRF
        NOP_ROUTE   = 4'h6, // restore route; imm[6]=1 scans cached tokens for expert
        NOP_RWEIGHT = 4'h7, // broadcast selected route weight to PRF vd; vs2=slot 1..4
        NOP_TOKEN_BEGIN = 4'h8, // begin outer token loop; imm[5:0]=count
        NOP_TOKEN_END   = 4'h9, // jump to token-loop body for next token
        NOP_SET_WPAGE   = 4'hA, // set static-weight 64-KiB page; imm[11:0]
        NOP_STATIC_SCALE = 4'hB, // load one 128-byte scale/zero group from static page
        NOP_TGLD         = 4'hC, // transpose-load 32x32 FP16 L1 tile into WRF
        NOP_BCAST        = 4'hD, // broadcast PRF vs1 lane imm[4:0] into PRF vd
        NOP_SET_CFG_K    = 4'hE, // set internal K tile width; imm[11:0]
        NOP_SET_CFG_M    = 4'hF  // set active GEMM row count; imm[5:0]
    } nop_subop_t;

    // =========================================================================
    // Hardware Loop stride codes (vd[3:2] of LOOP instruction)
    // =========================================================================
    // Stride is in imm units (16 bytes per unit)
    typedef enum logic [1:0] {
        STRIDE_FP16 = 2'b00, // 256 = 4096B / 16B
        STRIDE_INT1 = 2'b01, // 16  = 256B  / 16B
        STRIDE_INT2 = 2'b10, // 32  = 512B  / 16B
        STRIDE_INT3 = 2'b11  // 48  = 768B  / 16B
    } stride_code_t;

    function automatic logic [15:0] stride_code_to_value(input logic [1:0] code);
        case (code)
            STRIDE_FP16: stride_code_to_value = 16'd256;
            STRIDE_INT1: stride_code_to_value = 16'd16;
            STRIDE_INT2: stride_code_to_value = 16'd32;
            STRIDE_INT3: stride_code_to_value = 16'd48;
            default:     stride_code_to_value = 16'd256;
        endcase
    endfunction

    // =========================================================================
    // Precision Mode (vd[1:0]) — shared by GLD.W and GEMM
    // =========================================================================
    // Matches Qwen1.5-MoE per-block mixed-precision quantization:
    //   1-bit (1bit_u8), 2-bit (2bit_u8), 3-bit (3bit_32)
    // FP16 bypasses dequant entirely.
    typedef enum logic [1:0] {
        PREC_FP16 = 2'b00,  // FP16, bypass dequant
        PREC_INT1 = 2'b01,  // 1-bit quantization (nbits_code=3'b001)
        PREC_INT2 = 2'b10,  // 2-bit quantization (nbits_code=3'b010)
        PREC_INT3 = 2'b11   // 3-bit quantization (nbits_code=3'b011)
    } precision_t;

    // Derive dequant_unit nbits_code from precision mode
    function automatic logic [2:0] prec_to_nbits_code(input precision_t p);
        case (p)
            PREC_INT1: prec_to_nbits_code = 3'b001;
            PREC_INT2: prec_to_nbits_code = 3'b010;
            PREC_INT3: prec_to_nbits_code = 3'b011;
            default:   prec_to_nbits_code = 3'b100; // unused for FP16
        endcase
    endfunction

    // =========================================================================
    // Instruction Encoding
    //   [31:28] opcode  (4-bit)
    //   [27:24] vd      (4-bit) destination reg or mode
    //   [23:20] vs1     (4-bit) source 1
    //   [19:16] vs2     (4-bit) source 2
    //   [15:0]  imm     (16-bit) immediate / SRAM address
    //
    // Token-strided vector memory forms use the current outer token-loop index:
    //   VLD: vs1[3]=1, stride_lines_minus_1={vs1[2:0],vs2}
    //   VST: vd[3]=1,  stride_lines_minus_1={vd[2:0],vs2}
    // Stride lines are 32-byte L1 words. Encoded strides span 32..4096 bytes;
    // compiler helpers require at least one complete 64-byte vector per row.
    // =========================================================================
    typedef struct packed {
        logic [3:0]  opcode;
        logic [3:0]  vd;
        logic [3:0]  vs1;
        logic [3:0]  vs2;
        logic [15:0] imm;
    } instr_t;

    // =========================================================================
    // Decoded Instruction (internal representation)
    // =========================================================================
    typedef struct packed {
        opcode_t     opcode;
        logic [3:0]  vd;
        logic [3:0]  vs1;
        logic [3:0]  vs2;
        logic [15:0] imm;
        precision_t  prec_mode;  // extracted from vd[1:0] for GLD.W
    } decoded_instr_t;

    // =========================================================================
    // Field Extraction Functions
    // =========================================================================
    function automatic opcode_t get_opcode(input logic [INSTR_W-1:0] instr);
        return opcode_t'(instr[31:28]);
    endfunction

    function automatic logic [3:0] get_vd(input logic [INSTR_W-1:0] instr);
        return instr[27:24];
    endfunction

    function automatic logic [3:0] get_vs1(input logic [INSTR_W-1:0] instr);
        return instr[23:20];
    endfunction

    function automatic logic [3:0] get_vs2(input logic [INSTR_W-1:0] instr);
        return instr[19:16];
    endfunction

    function automatic logic [15:0] get_imm(input logic [INSTR_W-1:0] instr);
        return instr[15:0];
    endfunction

    function automatic precision_t get_precision(input logic [INSTR_W-1:0] instr);
        return precision_t'(instr[25:24]);
    endfunction

    function automatic decoded_instr_t decode_instr(input logic [INSTR_W-1:0] raw);
        decoded_instr_t d;
        d.opcode    = opcode_t'(raw[31:28]);
        d.vd        = raw[27:24];
        d.vs1       = raw[23:20];
        d.vs2       = raw[19:16];
        d.imm       = raw[15:0];
        d.prec_mode = (d.opcode == OP_GLD_W) ? precision_t'(raw[25:24]) : PREC_FP16;
        return d;
    endfunction

    // =========================================================================
    // Scoreboard Resource IDs (8-bit)
    // =========================================================================
    // Split WRF/ARF into WR (GLD writes inactive bank) and RD (GEMM reads
    // active bank) so GLD for next tile can run during GEMM.
    typedef enum logic [2:0] {
        RES_DMA_RD   = 3'd0,
        RES_DMA_WR   = 3'd1,
        RES_GEMM     = 3'd2,
        RES_POSTPROC = 3'd3,
        RES_WRF_WR   = 3'd4,   // GLD_W writing to inactive WRF bank
        RES_ARF_WR   = 3'd5,   // GLD_A writing to inactive ARF bank
        RES_WRF_RD   = 3'd6,   // GEMM reading from active WRF bank
        RES_ARF_RD   = 3'd7    // GEMM reading from active ARF bank
    } resource_t;

    // Backward-compatible aliases (deprecated, use new names)
    localparam logic [2:0] RES_WEIGHT_RF = RES_WRF_WR;
    localparam logic [2:0] RES_ACT_RF    = RES_ARF_WR;

endpackage

`endif
