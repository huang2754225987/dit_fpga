`default_nettype none

// =============================================================================
// supra2_softmax_2pass.sv - Buffered two-pass softmax for attention tiles
// =============================================================================
// Computes P = softmax(scores) for one ROWS x KEYS FP16 score tile without
// the online-rescale serialization of the current wrapper. The score tile is
// buffered in per-row RAMs, then:
//   pass 1: per-row max            (one key column per cycle)
//   pass 2: exp(score - max) via the shared graph FP16 exp LUT with linear
//           interpolation (same numerics as softmax_engine), FP32 running
//           sums, probabilities written to a second per-row RAM
//   pass 3: P * recip(sum) via the recip LUT, streamed out one key column
//           (ROWS values) per beat for the dense PV GEMM pass B
//
// Cycle budget at ROWS=32, KEYS=256: measured 2,401 XSim cycles per tile
// (pass 1: 2/column, pass 2: 5/column incl. registered-LUT waits, recip:
// 3/row, pass 3: 2/column) versus ~115k cycles the online-rescale wrapper
// spends per 32-query tile today; the block schedule simulator shows SFT at
// 1.4% busy with this measured latency, so it stays hidden under the array.
// =============================================================================

module supra2_softmax_2pass
  import fp16_utils_pkg::*;
#(
    parameter int ROWS    = 32,
    parameter int KEYS    = 256,
    parameter int KEY_W   = 16,
    parameter int ROW_W   = 5,
    parameter int KGROUP  = 32,              // keys per fill beat
    parameter int KBEAT_W = $clog2(KEYS/KGROUP)
) (
    input  wire                     clk,
    input  wire                     rst_n,

    // score fill: one beat = KGROUP keys of one row
    input  wire                     wr_en,
    input  wire [ROW_W-1:0]         wr_row,
    input  wire [KBEAT_W-1:0]       wr_beat,
    input  wire [KGROUP*KEY_W-1:0]  wr_data,

    input  wire                     start,
    output logic                    busy,
    output logic                    done,

    // normalized probability stream: one key column (all rows) per beat
    output logic                    out_valid,
    output logic [$clog2(KEYS)-1:0] out_col,   // key index 0..KEYS-1
    output logic [ROWS*KEY_W-1:0]   out_data
);

    localparam int KCNT_W = $clog2(KEYS);

    // ---------------- per-row score / probability RAMs ----------------
    logic [KEY_W-1:0] score_ram [0:ROWS-1][0:KEYS-1];
    logic [KEY_W-1:0] prob_ram  [0:ROWS-1][0:KEYS-1];

    always_ff @(posedge clk) begin
        if (wr_en) begin
            for (int g = 0; g < KGROUP; g++)
                score_ram[wr_row][wr_beat*KGROUP + g] <= wr_data[g*KEY_W +: KEY_W];
        end
    end

    // ---------------- FSM ----------------
    typedef enum logic [3:0] {
        S_IDLE,
        S_P1_ADDR, S_P1_CALC,
        S_P2_ADDR, S_P2_DIFF, S_P2_LUT, S_P2_WAIT, S_P2_ACC,
        S_RECIP_ADDR, S_RECIP_WAIT, S_RECIP_LATCH,
        S_P3_ADDR, S_P3_OUT,
        S_DONE
    } state_t;
    state_t state;

    logic [KCNT_W-1:0] col;
    logic [ROW_W-1:0]  recip_row;
    logic              rows_init;

    logic [KEY_W-1:0] row_max   [0:ROWS-1];
    logic [31:0]      row_sum   [0:ROWS-1];
    logic [KEY_W-1:0] row_recip [0:ROWS-1];
    logic [KEY_W-1:0] diff_q    [0:ROWS-1];
    logic [3:0]       diff_frac [0:ROWS-1];
    logic [11:0]      exp_addr  [0:ROWS-1];
    logic [KEY_W-1:0] col_score [0:ROWS-1];
    logic [KEY_W-1:0] col_prob  [0:ROWS-1];

    // ---------------- exp lanes (LUT + 1/16-step interpolation) ----------
    logic [KEY_W-1:0] exp_base [0:ROWS-1];
    logic [KEY_W-1:0] exp_next [0:ROWS-1];

    for (genvar r = 0; r < ROWS; r++) begin : g_exp
        graph_exp_lut_fp164k u_exp_lut (
            .clk      (clk),
            .addr     (exp_addr[r]),
            .data_out (exp_base[r])
        );
        graph_exp_lut_fp164k u_exp_lut_next (
            .clk      (clk),
            .addr     (exp_addr[r] + 12'd1),
            .data_out (exp_next[r])
        );
    end

    // e = base + (frac/16) * (next - base); 2^-4 = 16'h2C00
    logic [KEY_W-1:0] exp_result [0:ROWS-1];
    for (genvar r = 0; r < ROWS; r++) begin : g_interp
        assign exp_result[r] = fp16_add(
            exp_base[r],
            fp16_mul(fp16_mul(int8_to_fp16({4'd0, diff_frac[r]}),
                              16'h2C00),
                     fp16_sub(exp_next[r], exp_base[r])));
    end

    // reciprocal LUT (one port: 2 cycles per row)
    logic [11:0]      recip_addr;
    logic [KEY_W-1:0] recip_data;
    logic [KEY_W-1:0] recip_sum_fp16;
    assign recip_sum_fp16 = fp32_to_fp16(row_sum[recip_row]);
    recip_lut4k u_recip_lut (
        .clk      (clk),
        .addr     (recip_addr),
        .data_out (recip_data)
    );

    integer i;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            out_valid <= 1'b0;
            out_col <= '0;
            out_data <= '0;
            col <= '0;
            recip_row <= '0;
            rows_init <= 1'b0;
            recip_addr <= '0;
            for (i = 0; i < ROWS; i++) begin
                row_max[i] <= '0;
                row_sum[i] <= '0;
                row_recip[i] <= '0;
                diff_q[i] <= '0;
                diff_frac[i] <= '0;
                exp_addr[i] <= '0;
                col_score[i] <= '0;
                col_prob[i] <= '0;
            end
        end else begin
            done <= 1'b0;
            case (state)
                S_IDLE: begin
                    out_valid <= 1'b0;
                    if (start) begin
                        state <= S_P1_ADDR;
                        busy <= 1'b1;
                        col <= '0;
                        rows_init <= 1'b0;
                    end
                end

                // -------- pass 1: per-row max (2 cycles/column) --------
                S_P1_ADDR: begin
                    for (i = 0; i < ROWS; i++)
                        col_score[i] <= score_ram[i][col];
                    state <= S_P1_CALC;
                end
                S_P1_CALC: begin
                    for (i = 0; i < ROWS; i++) begin
                        if (!rows_init || fp16_cmp_gt(col_score[i], row_max[i]))
                            row_max[i] <= col_score[i];
                    end
                    rows_init <= 1'b1;
                    if (col == KEYS-1) begin
                        state <= S_P2_ADDR;
                        col <= '0;
                    end else begin
                        col <= col + 1'b1;
                        state <= S_P1_ADDR;
                    end
                end

                // -------- pass 2: exp + FP32 sum (4 cycles/column) --------
                S_P2_ADDR: begin
                    for (i = 0; i < ROWS; i++)
                        col_score[i] <= score_ram[i][col];
                    state <= S_P2_DIFF;
                end
                S_P2_DIFF: begin
                    for (i = 0; i < ROWS; i++)
                        diff_q[i] <= fp16_sub(col_score[i], row_max[i]);
                    state <= S_P2_LUT;
                end
                S_P2_LUT: begin
                    for (i = 0; i < ROWS; i++) begin
                        exp_addr[i] <= diff_q[i][15:4];
                        diff_frac[i] <= diff_q[i][3:0];
                    end
                    state <= S_P2_WAIT;  // registered LUT: addr settles, data next cycle
                end
                S_P2_WAIT: begin
                    state <= S_P2_ACC;   // exp_base/exp_next valid now
                end
                S_P2_ACC: begin
                    for (i = 0; i < ROWS; i++) begin
                        prob_ram[i][col] <= exp_result[i];
                        row_sum[i] <= fp32_add_rne(row_sum[i],
                                                   fp16_to_fp32(exp_result[i]));
                    end
                    if (col == KEYS-1) begin
                        state <= S_RECIP_ADDR;
                        recip_row <= '0;
                    end else begin
                        col <= col + 1'b1;
                        state <= S_P2_ADDR;
                    end
                end

                // -------- reciprocals (3 cycles/row: addr, LUT, latch) --------
                S_RECIP_ADDR: begin
                    recip_addr <= recip_sum_fp16[15:4];
                    state <= S_RECIP_WAIT;
                end
                S_RECIP_WAIT: begin
                    state <= S_RECIP_LATCH;  // recip_data valid now
                end
                S_RECIP_LATCH: begin
                    row_recip[recip_row] <= recip_data;
                    if (recip_row == ROWS-1) begin
                        state <= S_P3_ADDR;
                        col <= '0;
                    end else begin
                        recip_row <= recip_row + 1'b1;
                        state <= S_RECIP_ADDR;
                    end
                end

                // -------- pass 3: normalize + stream (2 cycles/column) --------
                S_P3_ADDR: begin
                    out_valid <= 1'b0;
                    for (i = 0; i < ROWS; i++)
                        col_prob[i] <= prob_ram[i][col];
                    state <= S_P3_OUT;
                end
                S_P3_OUT: begin
                    out_valid <= 1'b1;
                    out_col <= col;
                    for (i = 0; i < ROWS; i++)
                        out_data[i*KEY_W +: KEY_W] <= fp16_mul(col_prob[i], row_recip[i]);
                    if (col == KEYS-1) begin
                        state <= S_DONE;
                    end else begin
                        col <= col + 1'b1;
                        state <= S_P3_ADDR;
                    end
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

endmodule
