`default_nettype none

// =============================================================================
// supra2_layernorm_modulate_32lane.sv - 32-row tile LayerNorm/AdaLN
// =============================================================================
// Tile-parallel version of supra2_layernorm_modulate: processes a 32 x 576
// FP16 tile (one 32-row array batch) in about 600 cycles instead of the
// 295,168 cycles the row-sequential unit needs for 256 rows (2 cyc/elem).
// All 32 rows compute statistics in parallel over feature-major beats; the
// numeric contract is identical to the row-sequential unit (FP32 sum/sumsq,
// mean = sum/576, var = E[x^2]-mean^2 clamped, 3-iteration rsqrt NR with
// eps=1e-6, FP16 normalize -> (1+scale) multiply -> +shift), so per-row
// results match the verified unit within FP16 rounding order only.
//
// Pass 1 reuses one sum and one sum-of-squares accumulator per row across all
// features. The original 32-input combinational FP32 trees replicated nearly
// two thousand adders and made whole-block synthesis impractical.
// =============================================================================

module supra2_layernorm_modulate_32lane
  import fp16_utils_pkg::*;
#(
    parameter int ROWS     = 32,
    parameter int FEATURES = 576,
    parameter int KGROUP   = 32,
    parameter int BEATS    = FEATURES/KGROUP,   // 18
    parameter int ROW_W    = 5,
    parameter int BEAT_W   = $clog2(BEATS)      // 5
) (
    input  wire                     clk,
    input  wire                     rst_n,

    // x fill: one beat = KGROUP features of one row
    input  wire                     wr_en,
    input  wire [ROW_W-1:0]         wr_row,
    input  wire [BEAT_W-1:0]        wr_beat,
    input  wire [KGROUP*16-1:0]     wr_data,

    // scale/shift fill: shared per-feature vectors, one beat each
    input  wire                     wr_ss_en,
    input  wire                     wr_ss_sel,   // 0 = shift, 1 = scale
    input  wire [BEAT_W-1:0]        wr_ss_beat,
    input  wire [KGROUP*16-1:0]     wr_ss_data,

    input  wire                     start,
    output logic                    busy,
    output logic                    done,

    // output: one beat = KGROUP features of one row
    output logic                    out_valid,
    output logic [ROW_W-1:0]        out_row,
    output logic [BEAT_W-1:0]       out_beat,
    output logic [KGROUP*16-1:0]    out_data
);

    // ---------------- memories ----------------
    logic [KGROUP*16-1:0] x_mem_rd_data [0:ROWS-1];
    logic [BEAT_W-1:0] x_mem_rd_beat;
    logic x_mem_rd_en;
    logic [KGROUP*16-1:0] shift_mem [0:BEATS-1];
    logic [KGROUP*16-1:0] scale_mem [0:BEATS-1];

    // One independently addressed, synchronous-read BRAM bank per row.
    for (genvar r = 0; r < ROWS; r++) begin : g_x_mem
        (* ram_style = "block" *) logic [KGROUP*16-1:0] x_mem [0:BEATS-1];
        always_ff @(posedge clk) begin
            if (wr_en && wr_row == ROW_W'(r)) x_mem[wr_beat] <= wr_data;
            if (x_mem_rd_en) x_mem_rd_data[r] <= x_mem[x_mem_rd_beat];
        end
    end
    always_ff @(posedge clk) begin
        if (wr_ss_en) begin
            if (wr_ss_sel) scale_mem[wr_ss_beat] <= wr_ss_data;
            else           shift_mem[wr_ss_beat] <= wr_ss_data;
        end
    end

    // ---------------- constants (same as row-sequential unit) ----------------
    localparam logic [31:0] INV_FEATURES_FP32 = 32'h3ae38e39; // 1/576
    localparam logic [31:0] EPSILON_FP32      = 32'h358637bd; // 1e-6
    localparam logic [31:0] HALF_FP32         = 32'h3f000000;
    localparam logic [31:0] ONE_AND_HALF_FP32 = 32'h3fc00000;

    function automatic logic [31:0] fp32_rsqrt_nr(input logic [31:0] value);
        logic [31:0] estimate, half_value, estimate_sq, correction_term;
        logic [31:0] correction;
        begin
            estimate = 32'h5f3759df - (value >> 1);
            half_value = fp32_mul_rne(value, HALF_FP32);
            for (int iteration = 0; iteration < 3; iteration++) begin
                estimate_sq = fp32_mul_rne(estimate, estimate);
                correction_term = fp32_mul_rne(half_value, estimate_sq);
                correction = fp32_add_rne(ONE_AND_HALF_FP32,
                                           correction_term ^ 32'h80000000);
                estimate = fp32_mul_rne(estimate, correction);
            end
            return estimate;
        end
    endfunction

    // ---------------- FSM ----------------
    typedef enum logic [2:0] {
        S_IDLE, S_P1, S_STAT, S_P2, S_DONE
    } state_t;
    state_t state;

    logic [BEAT_W-1:0] p1_beat;
    logic [$clog2(KGROUP)-1:0] p1_group;
    logic [ROW_W-1:0]  p2_row;
    logic [BEAT_W-1:0] p2_beat;

    logic [31:0] sum_fp32   [0:ROWS-1];
    logic [31:0] sumsq_fp32 [0:ROWS-1];
    logic [31:0] mean_fp32  [0:ROWS-1];
    logic [31:0] invstd_fp32[0:ROWS-1];
    logic [KGROUP*16-1:0] mod_data;

    // pass-1 accumulation trees (combinational)
    logic [31:0] p1_word [0:ROWS-1];

    for (genvar r = 0; r < ROWS; r++) begin : g_p1
        assign p1_word[r] = fp16_to_fp32(
            x_mem_rd_data[r][int'(p1_group)*16 +: 16]);
    end

    integer i;

    always_comb begin
        x_mem_rd_en = 1'b0;
        x_mem_rd_beat = '0;
        if (state == S_IDLE && start) begin
            x_mem_rd_en = 1'b1;
            x_mem_rd_beat = '0;
        end else if (state == S_P1 && p1_group == KGROUP-1) begin
            x_mem_rd_en = 1'b1;
            x_mem_rd_beat = (p1_beat == BEATS-1) ? '0 : p1_beat + 1'b1;
        end else if (state == S_STAT) begin
            x_mem_rd_en = 1'b1;
            x_mem_rd_beat = '0;
        end else if (state == S_P2) begin
            x_mem_rd_en = 1'b1;
            x_mem_rd_beat = (p2_beat == BEATS-1) ? '0 : p2_beat + 1'b1;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            out_valid <= 1'b0;
            p1_beat <= '0;
            p1_group <= '0;
            p2_row <= '0;
            p2_beat <= '0;
            for (i = 0; i < ROWS; i++) begin
                sum_fp32[i] <= '0;
                sumsq_fp32[i] <= '0;
                mean_fp32[i] <= '0;
                invstd_fp32[i] <= '0;
            end
        end else begin
            done <= 1'b0;
            case (state)
                S_IDLE: begin
                    out_valid <= 1'b0;
                    if (start) begin
                        state <= S_P1;
                        busy <= 1'b1;
                        p1_beat <= '0;
                        p1_group <= '0;
                        for (i = 0; i < ROWS; i++) begin
                            sum_fp32[i] <= '0;
                            sumsq_fp32[i] <= '0;
                        end
                    end
                end

                // -------- pass 1: 18 feature beats, all rows in parallel --------
                S_P1: begin
                    for (i = 0; i < ROWS; i++) begin
                        sum_fp32[i] <= fp32_add_rne(sum_fp32[i], p1_word[i]);
                        sumsq_fp32[i] <= fp32_add_rne(sumsq_fp32[i],
                            fp32_mul_rne(p1_word[i], p1_word[i]));
                    end
                    if (p1_group == KGROUP-1) begin
                        p1_group <= '0;
                        if (p1_beat == BEATS-1) state <= S_STAT;
                        else p1_beat <= p1_beat + 1'b1;
                    end else p1_group <= p1_group + 1'b1;
                end

                // -------- stats: one cycle (comb rsqrt) --------
                S_STAT: begin
                    for (i = 0; i < ROWS; i++) begin : g_stat
                        logic [31:0] mean_v, avg_sq_v, mean_sq_v, var_v;
                        mean_v = fp32_mul_rne(sum_fp32[i], INV_FEATURES_FP32);
                        avg_sq_v = fp32_mul_rne(sumsq_fp32[i], INV_FEATURES_FP32);
                        mean_sq_v = fp32_mul_rne(mean_v, mean_v);
                        var_v = fp32_add_rne(avg_sq_v, mean_sq_v ^ 32'h80000000);
                        if (var_v[31]) var_v = 32'b0;
                        mean_fp32[i] <= mean_v;
                        invstd_fp32[i] <= fp32_rsqrt_nr(
                            fp32_add_rne(var_v, EPSILON_FP32));
                    end
                    p2_row <= '0;
                    p2_beat <= '0;
                    state <= S_P2;
                end

                // -------- pass 2: 32 rows x 18 beats output --------
                S_P2: begin
                    out_valid <= 1'b1;
                    out_row <= p2_row;
                    out_beat <= p2_beat;
                    out_data <= mod_data;
                    if (p2_beat == BEATS-1) begin
                        p2_beat <= '0;
                        if (p2_row == ROWS-1) state <= S_DONE;
                        else p2_row <= p2_row + 1'b1;
                    end else p2_beat <= p2_beat + 1'b1;
                end

                S_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    out_valid <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // -------- pass-1 operand selection + pass-2 output datapath --------
    always_comb begin
        // pass 2 modulation chain (same FP16 ops as the row-sequential unit);
        // registered into out_data/out_row/out_beat in the S_P2 cycle
        mod_data = '0;
        if (state == S_P2) begin
            for (int g = 0; g < KGROUP; g++) begin
                logic [31:0] centered_fp32, normalized_fp32;
                logic [15:0] normalized_fp16, scale_factor_fp16;
                centered_fp32 = fp32_add_rne(
                    fp16_to_fp32(x_mem_rd_data[p2_row][g*16 +: 16]),
                    mean_fp32[p2_row] ^ 32'h80000000);
                normalized_fp32 = fp32_mul_rne(centered_fp32, invstd_fp32[p2_row]);
                normalized_fp16 = fp32_to_fp16(normalized_fp32);
                scale_factor_fp16 = fp16_add(16'h3c00,
                    scale_mem[p2_beat][g*16 +: 16]);
                mod_data[g*16 +: 16] = fp16_add(
                    fp16_mul(normalized_fp16, scale_factor_fp16),
                    shift_mem[p2_beat][g*16 +: 16]);
            end
        end
    end

endmodule
