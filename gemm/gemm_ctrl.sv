// =============================================================================
// gemm_ctrl.sv - GEMM Controller (32-Bank WRF, pipelined integer dequant)
// =============================================================================
// 32x32 output-stationary systolic array.
//
// 32-bank WRF, 16 read ports. K-aware banking: word (K,w) -> bank w, entry K.
//
// FP16 (16 words/K):
//   WRF/ARF are asynchronous-read RFs. Read and commit one K row in the same
//   cycle, issuing one row per cycle (vs 2 cycles/K before).
//
// INT1/2/3 (1/2/3 words/K):
//   Read packed words and ARF data through the two-stage dequant pipeline.
//   After the first ordered commit, overlap remaining K gathers/commits with
//   systolic feed; the committed K prefix is always available before use.
//
// K_tile max = 256 after widening the ping-pong WRF/ARF entry address.
//
// Acc write: 32-bank parallel, STORE in up to 32 cycles (one row per cycle);
// routed-expert GEMMs may trim padded M rows after latching cfg_M.
// =============================================================================
`default_nettype none

module gemm_ctrl
  import isa_pkg::*;
#(
    parameter int ARRAY_DIM  = 32,
    parameter int FP16_W     = 16,
    parameter int ACC_W      = 32,
    parameter int MAX_K      = 256,
    parameter bit USE_NBITS_OVERRIDE = 1'b0
) (
    input  wire                          clk,
    input  wire                          rst_n,

    input  wire                          cmd_valid,
    output logic                         cmd_ready,
    input  wire                          trim_store_rows,

    input  wire  [15:0]                  cfg_M,
    input  wire  [15:0]                  cfg_N,
    input  wire  [15:0]                  cfg_K,

    input  wire  precision_t             prec_mode,
    output logic [2:0]                   dq_nbits_code,
    output wire                          dequant_group_sel,
    input  wire                          acc_accumulate,
    input  wire  [15:0]                  cmd_K,
    input  wire                          store_skip,
    input  wire                          nbits_override_valid,
    input  wire  [2:0]                   nbits_override,

    // 32-bank WRF read interface (16 read ports)
    output logic                         wrf_rd_en,
    output logic                         wrf_rd_bank_sel,  // 0: banks 0..15, 1: banks 16..31
    output logic [7:0]  wrf_rd_addr,      // entry index (K)
    input  wire  [31:0]                  wrf_rd_data0, wrf_rd_data1, wrf_rd_data2, wrf_rd_data3,
    input  wire  [31:0]                  wrf_rd_data4, wrf_rd_data5, wrf_rd_data6, wrf_rd_data7,
    input  wire  [31:0]                  wrf_rd_data8, wrf_rd_data9, wrf_rd_data10, wrf_rd_data11,
    input  wire  [31:0]                  wrf_rd_data12, wrf_rd_data13, wrf_rd_data14, wrf_rd_data15,

    // ARF read interface (32-bank, bank 0 read)
    output logic                         arf_rd_en,
    output logic [7:0]  arf_rd_addr,
    input  wire  [ARRAY_DIM*FP16_W-1:0]  arf_rd_data,
    input  wire                          arf_rd_data_valid,
    input  wire                          arf_batch_mode,

    output logic                         dequant_in_valid,
    input  wire                          dequant_in_ready,
    input  wire                          dequant_params_ready,
    output logic [ARRAY_DIM*8-1:0]       dequant_in_data,
    input  wire                          dequant_out_valid,
    output logic                         dequant_out_ready,
    input  wire  [ARRAY_DIM*FP16_W-1:0] dequant_out_data,

    output logic                         sa_clear,
    output logic                         sa_en,
    // Stable PE row mask derived from the command-latched M/trim controls.
    output wire  [5:0]                   sa_active_rows,
    output logic signed [FP16_W-1:0]     sa_a_col [0:ARRAY_DIM-1],
    output logic signed [FP16_W-1:0]     sa_b_row [0:ARRAY_DIM-1],

    // Acc write port: 32-bank parallel (32 columns per cycle)
    output logic                         acc_wr_en,
    output logic [4:0]                   acc_wr_addr,      // row index (0..31)
    output logic [ACC_W-1:0]             acc_wr_data0,
    output logic [ACC_W-1:0]             acc_wr_data1,
    output logic [ACC_W-1:0]             acc_wr_data2,
    output logic [ACC_W-1:0]             acc_wr_data3,
    output logic [ACC_W-1:0]             acc_wr_data4,
    output logic [ACC_W-1:0]             acc_wr_data5,
    output logic [ACC_W-1:0]             acc_wr_data6,
    output logic [ACC_W-1:0]             acc_wr_data7,
    output logic [ACC_W-1:0]             acc_wr_data8,
    output logic [ACC_W-1:0]             acc_wr_data9,
    output logic [ACC_W-1:0]             acc_wr_data10,
    output logic [ACC_W-1:0]             acc_wr_data11,
    output logic [ACC_W-1:0]             acc_wr_data12,
    output logic [ACC_W-1:0]             acc_wr_data13,
    output logic [ACC_W-1:0]             acc_wr_data14,
    output logic [ACC_W-1:0]             acc_wr_data15,
    output logic [ACC_W-1:0]             acc_wr_data16,
    output logic [ACC_W-1:0]             acc_wr_data17,
    output logic [ACC_W-1:0]             acc_wr_data18,
    output logic [ACC_W-1:0]             acc_wr_data19,
    output logic [ACC_W-1:0]             acc_wr_data20,
    output logic [ACC_W-1:0]             acc_wr_data21,
    output logic [ACC_W-1:0]             acc_wr_data22,
    output logic [ACC_W-1:0]             acc_wr_data23,
    output logic [ACC_W-1:0]             acc_wr_data24,
    output logic [ACC_W-1:0]             acc_wr_data25,
    output logic [ACC_W-1:0]             acc_wr_data26,
    output logic [ACC_W-1:0]             acc_wr_data27,
    output logic [ACC_W-1:0]             acc_wr_data28,
    output logic [ACC_W-1:0]             acc_wr_data29,
    output logic [ACC_W-1:0]             acc_wr_data30,
    output logic [ACC_W-1:0]             acc_wr_data31,

    // Acc read port (unused by GEMM, but declared for top-level mux)
    output logic                         acc_rd_en,
    output logic [9:0]                   acc_rd_addr,
    input  wire  [ACC_W-1:0]             acc_rd_data,

    input  wire  signed [ACC_W-1:0]      sa_acc [0:ARRAY_DIM-1][0:ARRAY_DIM-1],

    output logic                         busy,
    output logic                         done
);

    typedef enum logic [3:0] {
        ST_IDLE, ST_SETUP, ST_CLEAR,
        ST_GATHER, ST_OVERLAP, ST_FEED, ST_DRAIN, ST_STORE, ST_DONE
    } state_t;

    state_t state, state_nxt;

    logic [15:0] r_K, r_M, r_N;
    precision_t  r_prec;
    logic [2:0]  r_nbits_code;
    logic        r_store_skip;
    logic        r_acc_accumulate;
    logic        r_act_batch;
    logic        r_trim_store_rows;

    // 9-bit counters represent the terminal count 256 as well as the longer
    // K+M+N systolic wavefront. RF addresses use the low 8 bits only.
    logic [8:0] r_gather_k;
    logic [8:0] r_commit_k;
    logic [8:0] r_feed_k, r_feed_max;
    logic [4:0] r_store_cnt;
    logic [4:0] r_store_last;

    // INT pipelined gather (Task B): accept/commit decoupling. Dequant has
    // fixed 2-cycle latency; commits wait for the matching activation skid.
    // Accepts are paced by input readiness and active scale/zero readiness;
    // r_gather_k counts ACCEPTS, r_commit_k counts output COMMITS.
    wire int_gather = ((state == ST_GATHER) || (state == ST_OVERLAP)) &&
                      (r_prec != PREC_FP16);
    wire fp16_read_commit = (state == ST_GATHER) &&
                            (r_prec == PREC_FP16) &&
                            (r_gather_k < r_K[8:0]);
    // Do not consume packed weight data until the scale/zero group selected
    // by the matching GLD.W has been assembled. This is required when weight
    // prefetch overlaps the preceding GEMM: command ordering alone must not
    // be relied on to hide scale-loader latency.
    wire int_params_ready = dequant_params_ready;
    wire int_accept = int_gather && (r_gather_k < r_K[8:0]) &&
                      int_params_ready && dequant_in_ready;
    wire dq_commit  = int_gather && dequant_out_valid && dequant_out_ready;
    // Only feed rows already committed by dequant. If scale/zero readiness or
    // input back-pressure pauses the dequant stream, the systolic feed must
    // pause at the committed-prefix boundary instead of reading stale wt_buf.
    wire int_feed_prefix_ready = (r_feed_k < r_commit_k);
    logic [ARRAY_DIM*FP16_W-1:0] act_d1, act_d2;
    logic act_d1_valid, act_d2_valid;

    function automatic logic [5:0] nwords_of(input logic [2:0] code);
        case (code)
            3'b000: nwords_of = 6'd8;
            3'b001: nwords_of = 6'd1;
            3'b010: nwords_of = 6'd2;
            3'b011: nwords_of = 6'd3;
            3'b100: nwords_of = 6'd4;
            default nwords_of = 6'd8;
        endcase
    endfunction

    logic [5:0] r_nwords;
    always_comb r_nwords = nwords_of(r_nbits_code);
    // Keep the unpack width aligned with the precision latched for this GEMM.
    assign dq_nbits_code = r_nbits_code;
    assign dequant_group_sel = r_gather_k[7];

    // Buffers
    logic [FP16_W-1:0] act_buf [0:MAX_K-1][0:ARRAY_DIM-1];
    logic [FP16_W-1:0] wt_buf [0:MAX_K-1][0:ARRAY_DIM-1];

    logic [15:0] arf_word;

    // =========================================================================
    // Config latch
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_K <= '0; r_M <= '0; r_N <= '0;
            r_prec <= PREC_FP16; r_nbits_code <= 3'd0;
            r_store_skip <= 1'b0;
            r_acc_accumulate <= 1'b0;
            r_trim_store_rows <= 1'b0;
            r_act_batch <= 1'b0;
        end else if (state == ST_IDLE && cmd_valid) begin
            r_K    <= cmd_K;
            r_M    <= cfg_M;
            r_N    <= cfg_N;
            r_prec <= prec_mode;
            r_nbits_code <= (USE_NBITS_OVERRIDE && nbits_override_valid)
                            ? nbits_override : prec_to_nbits_code(prec_mode);
            r_store_skip <= store_skip;
            r_acc_accumulate <= acc_accumulate;
            r_trim_store_rows <= trim_store_rows;
            r_act_batch <= arf_batch_mode;
        end
    end

    // =========================================================================
    // FSM
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) state <= ST_IDLE;
        else        state <= state_nxt;

    always_comb begin
        state_nxt = state;
        case (state)
            ST_IDLE:    if (cmd_valid) state_nxt = ST_SETUP;
            ST_SETUP:   state_nxt = ST_CLEAR;
            ST_CLEAR:   state_nxt = ST_GATHER;
            ST_GATHER: begin
                if (r_prec == PREC_FP16) begin
                    // Async WRF/ARF reads are committed on the same edge.
                    if (r_gather_k + 9'd1 >= r_K[8:0])
                        state_nxt = ST_FEED;
                end else begin
                    // Start the array as soon as the first ordered K row is
                    // committed. K=1 has no remaining gather work to overlap.
                    if (dq_commit)
                        state_nxt = (r_commit_k == r_K[8:0] - 9'd1)
                                  ? ST_FEED : ST_OVERLAP;
                end
            end
            ST_OVERLAP: if (dq_commit && r_commit_k == r_K[8:0] - 9'd1)
                            state_nxt = ST_FEED;
            ST_FEED:    if (r_feed_k >= r_feed_max) state_nxt = ST_DRAIN;
            // r_feed_max already includes the systolic wavefront drain. Keep
            // one ST_DRAIN cycle for the final product to retire through the
            // second MAC pipeline stage before store or the next K chunk.
            ST_DRAIN:   state_nxt = r_store_skip ? ST_DONE : ST_STORE;
            ST_STORE:   if (r_store_cnt == r_store_last) state_nxt = ST_DONE;
            ST_DONE:    state_nxt = ST_IDLE;
            default:    state_nxt = ST_IDLE;
        endcase
    end

    always_comb begin
        r_feed_max = r_K
                   + ((r_M < 16'd32) ? r_M : 16'd32)
                   + ((r_N < 16'd32) ? r_N : 16'd32)
                   - 16'd3;
    end

    always_comb begin
        if (r_trim_store_rows && r_M != 0 && r_M < 16'd32)
            r_store_last = r_M[4:0] - 5'd1;
        else
            r_store_last = 5'd31;
    end

    // =========================================================================
    // INT pipelined gather (Task B): accept/commit counters + act skid
    // (declarations at the gather counter block above)
    // =========================================================================
    // gather_k (accept) / commit_k counters
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_gather_k <= '0;
            r_commit_k <= '0;
        end else if (state == ST_SETUP) begin
            r_gather_k <= '0;
            r_commit_k <= '0;
        end else if (state == ST_GATHER || state == ST_OVERLAP) begin
            if (r_prec == PREC_FP16) begin
                if (fp16_read_commit)
                    r_gather_k <= r_gather_k + 9'd1;
            end else begin
                if (int_accept) r_gather_k <= r_gather_k + 9'd1;
                if (dq_commit)  r_commit_k <= r_commit_k + 9'd1;
            end
        end
    end

    // Activation skid: align arf_rd_data (sampled at accept) with the
    // dequant output 2 cycles later. Shift every gather cycle, inserting a
    // zero for bubbles so a stalled input cannot replay the previous accepted
    // activation when a later dequant result commits. The final accepted
    // activation still advances through both stages without requiring a
    // trailing accept.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            act_d1 <= '0;
            act_d2 <= '0;
            act_d1_valid <= 1'b0;
            act_d2_valid <= 1'b0;
        end else begin
            if (int_gather) begin
                // ARF batch reads return a full 32-lane word even when the
                // current GEMM has fewer active M rows. Unwritten padding
                // banks are intentionally don't-care; mask them here so X
                // values can never enter the dequant/activation pipeline.
                for (int row = 0; row < ARRAY_DIM; row++) begin
                    if (int_accept &&
                        (r_act_batch ? (row < r_M) : (row == 0)))
                        act_d1[row*FP16_W +: FP16_W] <=
                            arf_rd_data[row*FP16_W +: FP16_W];
                    else
                        act_d1[row*FP16_W +: FP16_W] <= '0;
                end
                act_d2 <= act_d1;
                act_d1_valid <= int_accept;
                act_d2_valid <= act_d1_valid;
            end else begin
                act_d1 <= '0;
                act_d2 <= '0;
                act_d1_valid <= 1'b0;
                act_d2_valid <= 1'b0;
            end
        end
    end

    // =========================================================================
    // WRF read address
    // =========================================================================
    // FP16 async read is consumed on the same edge; INT stays held while an
    // accept is pending (address stable under dequant back-pressure).
    assign wrf_rd_en      = fp16_read_commit || int_accept;
    assign wrf_rd_bank_sel = 1'b0;  // always read from banks 0..15
    assign wrf_rd_addr      = r_gather_k[7:0];  // 0..255

    // ARF async read (new 32-bank interface, reads bank 0)
    assign arf_rd_en   = fp16_read_commit || int_accept;
    assign arf_rd_addr = r_gather_k[7:0];

    // =========================================================================
    // FP16 gather: commit the asynchronous RF values on the read edge.
    // =========================================================================
    always_ff @(posedge clk) begin
        if (fp16_read_commit) begin
            // Commit: distribute 16 words to wt_buf[k][0..31]
            wt_buf[r_gather_k[7:0]][0]  <= wrf_rd_data0[15:0];
            wt_buf[r_gather_k[7:0]][1]  <= wrf_rd_data0[31:16];
            wt_buf[r_gather_k[7:0]][2]  <= wrf_rd_data1[15:0];
            wt_buf[r_gather_k[7:0]][3]  <= wrf_rd_data1[31:16];
            wt_buf[r_gather_k[7:0]][4]  <= wrf_rd_data2[15:0];
            wt_buf[r_gather_k[7:0]][5]  <= wrf_rd_data2[31:16];
            wt_buf[r_gather_k[7:0]][6]  <= wrf_rd_data3[15:0];
            wt_buf[r_gather_k[7:0]][7]  <= wrf_rd_data3[31:16];
            wt_buf[r_gather_k[7:0]][8]  <= wrf_rd_data4[15:0];
            wt_buf[r_gather_k[7:0]][9]  <= wrf_rd_data4[31:16];
            wt_buf[r_gather_k[7:0]][10] <= wrf_rd_data5[15:0];
            wt_buf[r_gather_k[7:0]][11] <= wrf_rd_data5[31:16];
            wt_buf[r_gather_k[7:0]][12] <= wrf_rd_data6[15:0];
            wt_buf[r_gather_k[7:0]][13] <= wrf_rd_data6[31:16];
            wt_buf[r_gather_k[7:0]][14] <= wrf_rd_data7[15:0];
            wt_buf[r_gather_k[7:0]][15] <= wrf_rd_data7[31:16];
            wt_buf[r_gather_k[7:0]][16] <= wrf_rd_data8[15:0];
            wt_buf[r_gather_k[7:0]][17] <= wrf_rd_data8[31:16];
            wt_buf[r_gather_k[7:0]][18] <= wrf_rd_data9[15:0];
            wt_buf[r_gather_k[7:0]][19] <= wrf_rd_data9[31:16];
            wt_buf[r_gather_k[7:0]][20] <= wrf_rd_data10[15:0];
            wt_buf[r_gather_k[7:0]][21] <= wrf_rd_data10[31:16];
            wt_buf[r_gather_k[7:0]][22] <= wrf_rd_data11[15:0];
            wt_buf[r_gather_k[7:0]][23] <= wrf_rd_data11[31:16];
            wt_buf[r_gather_k[7:0]][24] <= wrf_rd_data12[15:0];
            wt_buf[r_gather_k[7:0]][25] <= wrf_rd_data12[31:16];
            wt_buf[r_gather_k[7:0]][26] <= wrf_rd_data13[15:0];
            wt_buf[r_gather_k[7:0]][27] <= wrf_rd_data13[31:16];
            wt_buf[r_gather_k[7:0]][28] <= wrf_rd_data14[15:0];
            wt_buf[r_gather_k[7:0]][29] <= wrf_rd_data14[31:16];
            wt_buf[r_gather_k[7:0]][30] <= wrf_rd_data15[15:0];
            wt_buf[r_gather_k[7:0]][31] <= wrf_rd_data15[31:16];
            // ARF returns one activation per systolic row in batch mode;
            // legacy mode places its single activation in lane zero.
            for (int row = 0; row < ARRAY_DIM; row++)
                act_buf[r_gather_k[7:0]][row] <=
                    arf_rd_data[row*FP16_W +: FP16_W];
        end
    end

    // =========================================================================
    // INT gather: 1 read cycle, dequant, commit
    // =========================================================================
    logic [ARRAY_DIM*8-1:0] dq_field;
    always_comb begin
        dq_field = '0;
        dq_field[31:0] = wrf_rd_data0;
        if (r_nwords >= 6'd2) dq_field[63:32]  = wrf_rd_data1;
        if (r_nwords >= 6'd3) dq_field[95:64]  = wrf_rd_data2;
        if (r_nwords >= 6'd4) dq_field[127:96] = wrf_rd_data3;
        if (r_nwords >= 6'd5) dq_field[159:128] = wrf_rd_data4;
        if (r_nwords >= 6'd6) dq_field[191:160] = wrf_rd_data5;
        if (r_nwords >= 6'd7) dq_field[223:192] = wrf_rd_data6;
        if (r_nwords >= 6'd8) dq_field[255:224] = wrf_rd_data7;
    end

    // Use the same acceptance event for both RF reads and the dequant input.
    // This keeps weight, activation, and gather counters on one transaction
    // boundary if the dequant pipeline ever applies back-pressure.
    assign dequant_in_valid = int_accept;
    assign dequant_in_data  = dq_field;
    // Commit only when the activation accepted with this weight is available.
    assign dequant_out_ready = int_gather && act_d2_valid;

    always_ff @(posedge clk) begin
        if (dq_commit) begin
            // Commit dequant output to wt_buf (in-order, fixed 2-cycle
            // latency from accept; act via the accept-aligned skid)
            for (int j = 0; j < ARRAY_DIM; j++)
                wt_buf[r_commit_k[7:0]][j] <= dequant_out_data[j*FP16_W +: FP16_W];
            for (int row = 0; row < ARRAY_DIM; row++)
                act_buf[r_commit_k[7:0]][row] <=
                    act_d2[row*FP16_W +: FP16_W];
        end
    end

    // =========================================================================
    // Feed phase
    // =========================================================================
    always_comb begin
        for (int gi = 0; gi < ARRAY_DIM; gi++) begin
            automatic int m_a = int'(r_feed_k) - gi;
            if ((state == ST_FEED ||
                 (state == ST_OVERLAP && r_feed_k < r_feed_max &&
                  int_feed_prefix_ready)) &&
                m_a >= 0 && m_a < r_K[8:0] &&
                (!r_act_batch || gi < r_M))
                sa_a_col[gi] = r_act_batch
                             ? act_buf[m_a[7:0]][gi]
                             : act_buf[m_a[7:0]][0];
            else
                sa_a_col[gi] = '0;
        end
        for (int gj = 0; gj < ARRAY_DIM; gj++) begin
            automatic int m_b = int'(r_feed_k) - gj;
            if ((state == ST_FEED ||
                 (state == ST_OVERLAP && r_feed_k < r_feed_max &&
                  int_feed_prefix_ready)) &&
                m_b >= 0 && m_b < r_K[8:0])
                sa_b_row[gj] = wt_buf[m_b[7:0]][gj];
            else
                sa_b_row[gj] = '0;
        end
    end

    // Separate productive MAC feed cycles from zero-input drain/flush cycles
    // so utilization telemetry does not count pipeline retirement as compute.
    wire sa_compute_active = (state == ST_FEED) ||
                             (state == ST_OVERLAP &&
                              r_feed_k < r_feed_max &&
                              int_feed_prefix_ready);
    assign sa_clear = (state == ST_CLEAR) && !r_acc_accumulate;
    assign sa_en    = sa_compute_active || (state == ST_DRAIN);
    assign sa_active_rows = (r_trim_store_rows && r_M != 0 && r_M < 16'd32)
                          ? r_M[5:0] : 6'd32;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) r_feed_k <= '0;
        else if (state == ST_SETUP) r_feed_k <= '0;
        else if (((state == ST_OVERLAP && int_feed_prefix_ready) ||
                  state == ST_FEED) && r_feed_k < r_feed_max)
            r_feed_k <= r_feed_k + 9'd1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) r_store_cnt <= '0;
        else if (state == ST_DRAIN && state_nxt == ST_STORE) r_store_cnt <= '0;
        else if (state == ST_STORE) r_store_cnt <= r_store_cnt + 5'd1;
    end

    // =========================================================================
    // Store phase: write one 32-column row per cycle. Routed-expert commands
    // stop at the latched active M count; ordinary GEMMs retain all 32 rows.
    // =========================================================================
    assign acc_wr_en   = (state == ST_STORE) &&
                         (!r_trim_store_rows || r_M == 0 || r_M >= 16'd32 ||
                          r_store_cnt < r_M[4:0]);
    assign acc_wr_addr = r_store_cnt;
    assign acc_wr_data0  = sa_acc[r_store_cnt][0];
    assign acc_wr_data1  = sa_acc[r_store_cnt][1];
    assign acc_wr_data2  = sa_acc[r_store_cnt][2];
    assign acc_wr_data3  = sa_acc[r_store_cnt][3];
    assign acc_wr_data4  = sa_acc[r_store_cnt][4];
    assign acc_wr_data5  = sa_acc[r_store_cnt][5];
    assign acc_wr_data6  = sa_acc[r_store_cnt][6];
    assign acc_wr_data7  = sa_acc[r_store_cnt][7];
    assign acc_wr_data8  = sa_acc[r_store_cnt][8];
    assign acc_wr_data9  = sa_acc[r_store_cnt][9];
    assign acc_wr_data10 = sa_acc[r_store_cnt][10];
    assign acc_wr_data11 = sa_acc[r_store_cnt][11];
    assign acc_wr_data12 = sa_acc[r_store_cnt][12];
    assign acc_wr_data13 = sa_acc[r_store_cnt][13];
    assign acc_wr_data14 = sa_acc[r_store_cnt][14];
    assign acc_wr_data15 = sa_acc[r_store_cnt][15];
    assign acc_wr_data16 = sa_acc[r_store_cnt][16];
    assign acc_wr_data17 = sa_acc[r_store_cnt][17];
    assign acc_wr_data18 = sa_acc[r_store_cnt][18];
    assign acc_wr_data19 = sa_acc[r_store_cnt][19];
    assign acc_wr_data20 = sa_acc[r_store_cnt][20];
    assign acc_wr_data21 = sa_acc[r_store_cnt][21];
    assign acc_wr_data22 = sa_acc[r_store_cnt][22];
    assign acc_wr_data23 = sa_acc[r_store_cnt][23];
    assign acc_wr_data24 = sa_acc[r_store_cnt][24];
    assign acc_wr_data25 = sa_acc[r_store_cnt][25];
    assign acc_wr_data26 = sa_acc[r_store_cnt][26];
    assign acc_wr_data27 = sa_acc[r_store_cnt][27];
    assign acc_wr_data28 = sa_acc[r_store_cnt][28];
    assign acc_wr_data29 = sa_acc[r_store_cnt][29];
    assign acc_wr_data30 = sa_acc[r_store_cnt][30];
    assign acc_wr_data31 = sa_acc[r_store_cnt][31];

    // Acc read (unused by GEMM)
    assign acc_rd_en   = 1'b0;
    assign acc_rd_addr = '0;

    assign cmd_ready = (state == ST_IDLE);
    assign busy      = (state != ST_IDLE);
    assign done      = (state == ST_DONE);

endmodule

`default_nettype wire
