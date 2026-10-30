// =============================================================================
// supra2_gemm_engine.sv - Supra2 GEMM engine wrapper
//
// Wraps the verified token-level chain (Hadamard -> dynamic INT8 quantizer ->
// 32x32 systolic array -> FP32 scale/bias epilogue) into a single engine with
// a start/busy/done interface. Two modes:
//
//   INT8 QLinear (cfg_fp16=0): streams cfg_m raw FP16 rows through the
//     frontend (row read port, 32 lanes/beat), then runs K-major INT8 GEMM
//     from the engine-local weight buffer, draining FP16 tiles of 32 columns.
//   Attention QK (cfg_fp16=1): quantizes the engine-local FP16 Q/K tiles to
//     INT8 Q4 at the array boundary, accumulates in INT32, then dequantizes
//     int32->FP32 and applies cfg_scale before emitting FP16 scores.
//
// Output: one beat per (row, 32-col tile): out_row/out_ct/out_data.
// Behavioral buffer sizes target the Supra2 shapes (max M=256, N=2304,
// K=2304); this module is simulation-oriented bring-up RTL, not yet the
// synthesis-ready tiled-memory engine. The asynchronous raw read address is
// held stable while the block-buffered Hadamard frontend deasserts ready.
// =============================================================================
`default_nettype none

module supra2_gemm_engine
  import fp16_utils_pkg::*;
#(
    parameter int LANES  = 32,
    parameter int MAX_M  = 256,
    parameter int MAX_N  = 2304,
    parameter int MAX_K  = 2304,
    // The model's largest individual QLinear matrix is 2304x576 (or its
    // transpose shape 576x2304), not MAX_N*MAX_K. Keep a model-specific
    // capacity override so synthesis does not allocate the impossible cross
    // product of independent maximum dimensions.
    parameter int MAX_WEIGHT_ELEMS = MAX_N * MAX_K,
    parameter int MAX_F16_K = 64,
    parameter int MAX_F16_N = 64,
    parameter bit EXTERNAL_ARRAY = 1'b0,
    parameter bit DUAL_WEIGHT_BUFFERS = 1'b0
) (
    input  wire                     clk,
    input  wire                     rst_n,

    // configuration / control
    input  wire                     cfg_fp16,
    input  wire                     cfg_fp16_bias_en,
    input  wire [31:0]              cfg_fp16_act_scale,
    input  wire [31:0]              cfg_fp16_wt_scale,
    input  wire [15:0]              cfg_m,
    input  wire [15:0]              cfg_n,
    input  wire [15:0]              cfg_k,
    input  wire [8:0]               cfg_h,
    input  wire [15:0]              cfg_ni0,   // column-tile resume offset (0 for full)
    input  wire [15:0]              cfg_ni1,   // column-tile slice end (0 = cfg_n)
    input  wire [31:0]              cfg_scale,
    input  wire                     start,
    output logic                    busy,
    output logic                    done,

    // raw FP16 row read for the INT8 frontend (async read, 32 lanes/beat)
    output logic [15:0]             raw_row,
    output logic [15:0]             raw_beat,
    output logic                    raw_prefetch_en,
    output logic [15:0]             raw_prefetch_row,
    output logic [15:0]             raw_prefetch_beat,
    input  wire [LANES*16-1:0]      raw_rd_data,

    // FP16-mode local tile load (32 FP16 per beat)
    input  wire                     f16_wr_en,
    input  wire                     f16_wr_sel,     // 0 = act, 1 = wt
    input  wire [15:0]              f16_wr_beat,
    input  wire [LANES*16-1:0]      f16_wr_data,

    // INT8 weight load: 32 int8 per beat, flat index = beat*32+lane = n*K+k
    input  wire                     wt_wr_en,
    input  wire [15:0]              wt_wr_beat,
    input  wire [LANES*8-1:0]       wt_wr_data,
    // 双缓冲使能时，加载端参数独立于当前计算的 cfg_k。
    input  wire                     weight_write_slot,
    input  wire                     weight_read_slot,
    input  wire [15:0]              weight_load_k,

    // per-column fp32 scale/bias load
    input  wire                     sb_wr_en,
    input  wire                     sb_wr_sel,      // 0 = weight_scale, 1 = bias
    input  wire [15:0]              sb_wr_addr,
    input  wire [31:0]              sb_wr_data,

    // result drain: 32 FP16 = columns [out_ct*32, out_ct*32+32) of out_row
    output logic                    out_valid,
    output logic [15:0]             out_row,
    output logic [15:0]             out_ct,
    output logic [LANES*16-1:0]     out_data,

    // Optional shared-array interface. When EXTERNAL_ARRAY=1, the parent
    // supplies the accumulator grid and valid pulse and arbitrates this
    // request with other users of the physical array.
    output wire                     shared_array_clear,
    output wire                     shared_array_en,
    output wire signed [7:0]        shared_array_a [LANES],
    output wire signed [7:0]        shared_array_b [LANES],
    input  wire signed [31:0]       shared_array_acc [LANES][LANES],
    input  wire                     shared_array_acc_valid
);

    typedef enum logic [3:0] {
        S_IDLE, S_FE_START, S_FE_FEED, S_FE_DRAIN,
        S_TILE_CLEAR, S_FEED, S_WAIT_ACC, S_DRAIN, S_DONE
    } state_t;
    state_t state;
    typedef enum logic [2:0] {FE_IDLE, FE_START, FE_FEED, FE_DRAIN, FE_WAIT_SLOT} fe_state_t;
    fe_state_t fe_state;
    logic [((MAX_M+LANES-1)/LANES)-1:0] activation_ready;
    logic [15:0] row, beat, obeat;
    logic [15:0] mi, ni, t;
    logic [5:0] drain_i;
    logic drain_active;
    logic [15:0] drain_mi, drain_ni;
    logic fe_q_out_valid;
    logic [LANES*8-1:0] fe_q_out_data;

    // ---------------- engine-local buffers ----------------
    logic [31:0] token_scale [0:MAX_M-1];
    localparam int ACT_SLOTS = (MAX_M > LANES) ? 2 : 1;
    localparam int ACT_BANK_DEPTH = ACT_SLOTS*((MAX_K+LANES-1)/LANES);
    localparam int WEIGHT_BANK_DEPTH =
        (MAX_WEIGHT_ELEMS+LANES*LANES-1)/(LANES*LANES);
    localparam int SCALE_BANK_DEPTH = (MAX_N+LANES-1)/LANES;
    localparam int F16_ACT_BANK_DEPTH =
        ((MAX_M+LANES-1)/LANES)*((MAX_F16_K+LANES-1)/LANES);
    localparam int F16_WT_BANK_DEPTH =
        ((MAX_F16_N+LANES-1)/LANES)*((MAX_F16_K+LANES-1)/LANES);
    localparam int ACT_ADDR_W = (ACT_BANK_DEPTH>1)?$clog2(ACT_BANK_DEPTH):1;
    localparam int WEIGHT_ADDR_W = (WEIGHT_BANK_DEPTH>1)?$clog2(WEIGHT_BANK_DEPTH):1;
    localparam int SCALE_ADDR_W = (SCALE_BANK_DEPTH>1)?$clog2(SCALE_BANK_DEPTH):1;
    localparam int F16_ACT_ADDR_W = (F16_ACT_BANK_DEPTH>1)?$clog2(F16_ACT_BANK_DEPTH):1;
    localparam int F16_WT_ADDR_W = (F16_WT_BANK_DEPTH>1)?$clog2(F16_WT_BANK_DEPTH):1;
    logic [ACT_ADDR_W-1:0] act_rd_addr [0:LANES-1];
    logic [WEIGHT_ADDR_W-1:0] wt_rd_addr [0:LANES-1];
    logic [F16_ACT_ADDR_W-1:0] f16_act_rd_addr [0:LANES-1];
    logic [F16_WT_ADDR_W-1:0] f16_wt_rd_addr [0:LANES-1];
    logic [SCALE_ADDR_W-1:0] scale_rd_addr;
    logic [LANES*8-1:0] act_bank_rdata [0:LANES-1];
    logic [LANES*8-1:0] wt_bank_rdata [0:LANES-1];
    wire [LANES*8-1:0] wt_bank0_rdata [0:LANES-1], wt_bank1_rdata [0:LANES-1];
    logic [LANES*16-1:0] f16_act_bank_rdata [0:LANES-1];
    logic [LANES*16-1:0] f16_wt_bank_rdata [0:LANES-1];
    logic [31:0] scale_bank_rdata [0:LANES-1];
    logic [31:0] bias_bank_rdata [0:LANES-1];
    wire [31:0] scale_bank0_rdata [0:LANES-1], scale_bank1_rdata [0:LANES-1];
    wire [31:0] bias_bank0_rdata [0:LANES-1], bias_bank1_rdata [0:LANES-1];
    wire [15:0] load_k = DUAL_WEIGHT_BUFFERS ? weight_load_k : cfg_k;
    logic [4:0] feed_byte_index [0:LANES-1];
    logic [4:0] feed_half_index [0:LANES-1];
    logic [LANES-1:0] feed_read_valid;
    logic [LANES-1:0] f16_feed_read_valid;
    logic [15:0] read_target_t;
    logic [15:0] weight_wr_row, weight_wr_kbeat;
    logic [WEIGHT_ADDR_W-1:0] weight_wr_addr;
    logic [15:0] f16_wr_row, f16_wr_kbeat;
    logic [F16_ACT_ADDR_W-1:0] f16_act_wr_addr;
    logic [F16_WT_ADDR_W-1:0] f16_wt_wr_addr;
    logic [ACT_ADDR_W-1:0] act_wr_addr;
    logic [31:0] weight_wr_linear_byte;
    logic [31:0] address_temp;
    logic [LANES-1:0] weight_bank_wr_en;
    logic [WEIGHT_ADDR_W-1:0] weight_bank_wr_addr [0:LANES-1];
    logic [LANES*8-1:0] weight_bank_wr_data [0:LANES-1];
    // FP16-mode tiles. Defaults cover QK; the final head overrides the
    // geometry for its 256x576 activation and 32x576 padded weight matrices.

    // Store 32-byte/FP16 beats as wide words in 32 independent output-lane
    // banks. This preserves the current row-major load format while giving
    // each array lane one synchronous read port instead of 32 ports on one
    // flattened memory.
    always_comb begin : banked_memory_addresses
        integer source_k;
        integer target_t_value;
        integer row_value;
        integer kbeat_value;
        read_target_t = (state == S_FEED) ? t + 1'b1 : 16'd0;
        weight_wr_linear_byte = 32'(wt_wr_beat) * LANES;
        weight_wr_row = 0;
        weight_wr_kbeat = 0;
        weight_wr_addr = '0;
        f16_wr_row = 0;
        f16_wr_kbeat = 0;
        f16_act_wr_addr = '0;
        f16_wt_wr_addr = '0;
        act_wr_addr = '0;
        scale_rd_addr = SCALE_ADDR_W'((drain_active ? drain_ni : ni) / LANES);
        if (load_k >= LANES) begin
            weight_wr_row = 16'(weight_wr_linear_byte / load_k);
            weight_wr_kbeat = 16'((weight_wr_linear_byte % load_k) / LANES);
            weight_wr_addr = WEIGHT_ADDR_W'(
                (weight_wr_row / LANES)*((load_k+LANES-1)/LANES)+weight_wr_kbeat);
        end
        if (cfg_k != 0) begin
            f16_wr_row = f16_wr_beat / ((cfg_k+LANES-1)/LANES);
            f16_wr_kbeat = f16_wr_beat % ((cfg_k+LANES-1)/LANES);
            f16_act_wr_addr = F16_ACT_ADDR_W'(
                (f16_wr_row/LANES)*((cfg_k+LANES-1)/LANES)+f16_wr_kbeat);
            f16_wt_wr_addr = F16_WT_ADDR_W'(
                (f16_wr_row/LANES)*((cfg_k+LANES-1)/LANES)+f16_wr_kbeat);
        end
        if (cfg_k != 0) begin
            act_wr_addr = ACT_ADDR_W'(
                ((int'(row)/LANES)%ACT_SLOTS)*((cfg_k+LANES-1)/LANES)+int'(obeat));
        end
        target_t_value = int'(read_target_t);
        for (int lane=0; lane<LANES; lane++) begin
            source_k = target_t_value-lane;
            act_rd_addr[lane] = '0;
            wt_rd_addr[lane] = '0;
            f16_act_rd_addr[lane] = '0;
            f16_wt_rd_addr[lane] = '0;
            if (source_k >= 0 && source_k < int'(cfg_k)) begin
                row_value = int'(mi)+lane;
                kbeat_value = source_k/LANES;
                act_rd_addr[lane] = ACT_ADDR_W'(
                    ((row_value/LANES)%ACT_SLOTS)*((cfg_k+LANES-1)/LANES)+kbeat_value);
                f16_act_rd_addr[lane] = F16_ACT_ADDR_W'(
                    (row_value/LANES)*((cfg_k+LANES-1)/LANES)+kbeat_value);
                row_value = int'(ni)+lane;
                wt_rd_addr[lane] = WEIGHT_ADDR_W'(
                    (row_value/LANES)*((cfg_k+LANES-1)/LANES)+kbeat_value);
                f16_wt_rd_addr[lane] = F16_WT_ADDR_W'(
                    (row_value/LANES)*((cfg_k+LANES-1)/LANES)+kbeat_value);
            end
        end
    end

    always_comb begin : banked_weight_load
        integer linear_index;
        integer row_index;
        integer col_index;
        for (int bank=0; bank<LANES; bank++) begin
            weight_bank_wr_en[bank] = 1'b0;
            weight_bank_wr_addr[bank] = '0;
            weight_bank_wr_data[bank] = '0;
        end
        if (wt_wr_en && load_k != 0) begin
            for (int lane=0; lane<LANES; lane++) begin
                linear_index = int'(wt_wr_beat)*LANES+lane;
                row_index = linear_index/int'(load_k);
                col_index = linear_index%int'(load_k);
                if (row_index < MAX_N && row_index%LANES < LANES) begin
                    weight_bank_wr_en[row_index%LANES] = 1'b1;
                    weight_bank_wr_addr[row_index%LANES] = WEIGHT_ADDR_W'(
                        (row_index/LANES)*((load_k+LANES-1)/LANES)+col_index/LANES);
                    weight_bank_wr_data[row_index%LANES][(col_index%LANES)*8 +: 8] =
                        wt_wr_data[lane*8 +: 8];
                end
            end
            if (load_k >= LANES) begin
                for (int bank=0; bank<LANES; bank++) begin
                    if (int'(weight_wr_row)%LANES == bank) begin
                        weight_bank_wr_en[bank] = 1'b1;
                        weight_bank_wr_addr[bank] = weight_wr_addr;
                        weight_bank_wr_data[bank] = wt_wr_data;
                    end
                end
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin : feed_read_tags
        integer source_k;
        if (!rst_n) begin
            feed_read_valid <= '0;
            f16_feed_read_valid <= '0;
            for (int lane=0; lane<LANES; lane++) begin
                feed_byte_index[lane] <= '0;
                feed_half_index[lane] <= '0;
            end
        end else begin
            for (int lane=0; lane<LANES; lane++) begin
                source_k = int'(read_target_t)-lane;
                feed_read_valid[lane] <=
                    (state == S_TILE_CLEAR || state == S_FEED) &&
                    source_k >= 0 && source_k < int'(cfg_k);
                f16_feed_read_valid[lane] <=
                    (state == S_TILE_CLEAR || state == S_FEED) && cfg_fp16 &&
                    source_k >= 0 && source_k < int'(cfg_k);
                if (source_k >= 0) begin
                    feed_byte_index[lane] <= 5'(source_k % LANES);
                    feed_half_index[lane] <= 5'(source_k % LANES);
                end
            end
        end
    end

    generate
        for (genvar bank=0; bank<LANES; bank++) begin : gen_operand_ram_banks
            localparam int B=bank;
            supra2_sync_word_ram #(.DATA_W(LANES*8),.DEPTH(ACT_BANK_DEPTH),
                .ADDR_W(ACT_ADDR_W)) act_ram (
                .clk(clk), .wr_en(fe_q_out_valid && (int'(row)%LANES==B)),
                .wr_addr(act_wr_addr), .wr_data(fe_q_out_data),
                .rd_addr(act_rd_addr[B]), .rd_data(act_bank_rdata[B]));
            supra2_sync_word_ram #(.DATA_W(LANES*8),.DEPTH(WEIGHT_BANK_DEPTH),
                .ADDR_W(WEIGHT_ADDR_W)) weight_ram (
                .clk(clk), .wr_en(weight_bank_wr_en[B] && (!DUAL_WEIGHT_BUFFERS || !weight_write_slot)),
                .wr_addr(weight_bank_wr_addr[B]), .wr_data(weight_bank_wr_data[B]),
                .rd_addr(wt_rd_addr[B]), .rd_data(wt_bank0_rdata[B]));
            supra2_sync_word_ram #(.DATA_W(LANES*16),.DEPTH(F16_ACT_BANK_DEPTH),
                .ADDR_W(F16_ACT_ADDR_W)) f16_act_ram (
                .clk(clk), .wr_en(f16_wr_en && !f16_wr_sel && cfg_k!=0 &&
                                    (int'(f16_wr_row)%LANES==B)),
                .wr_addr(f16_act_wr_addr), .wr_data(f16_wr_data),
                .rd_addr(f16_act_rd_addr[B]), .rd_data(f16_act_bank_rdata[B]));
            supra2_sync_word_ram #(.DATA_W(LANES*16),.DEPTH(F16_WT_BANK_DEPTH),
                .ADDR_W(F16_WT_ADDR_W)) f16_wt_ram (
                .clk(clk), .wr_en(f16_wr_en && f16_wr_sel && cfg_k!=0 &&
                                    (int'(f16_wr_row)%LANES==B)),
                .wr_addr(f16_wt_wr_addr), .wr_data(f16_wr_data),
                .rd_addr(f16_wt_rd_addr[B]), .rd_data(f16_wt_bank_rdata[B]));
            supra2_sync_word_ram #(.DATA_W(32),.DEPTH(SCALE_BANK_DEPTH),
                .ADDR_W(SCALE_ADDR_W)) scale_ram (
                .clk(clk), .wr_en(sb_wr_en && !sb_wr_sel && (!DUAL_WEIGHT_BUFFERS || !weight_write_slot) &&
                                    (int'(sb_wr_addr)%LANES==B)),
                .wr_addr(SCALE_ADDR_W'(sb_wr_addr/LANES)), .wr_data(sb_wr_data),
                .rd_addr(scale_rd_addr), .rd_data(scale_bank0_rdata[B]));
            supra2_sync_word_ram #(.DATA_W(32),.DEPTH(SCALE_BANK_DEPTH),
                .ADDR_W(SCALE_ADDR_W)) bias_ram (
                .clk(clk), .wr_en(sb_wr_en && sb_wr_sel && (!DUAL_WEIGHT_BUFFERS || !weight_write_slot) &&
                                    (int'(sb_wr_addr)%LANES==B)),
                .wr_addr(SCALE_ADDR_W'(sb_wr_addr/LANES)), .wr_data(sb_wr_data),
                .rd_addr(scale_rd_addr), .rd_data(bias_bank0_rdata[B]));
            if (DUAL_WEIGHT_BUFFERS) begin : g_weight_second_slot
                supra2_sync_word_ram #(.DATA_W(LANES*8),.DEPTH(WEIGHT_BANK_DEPTH),
                    .ADDR_W(WEIGHT_ADDR_W)) weight_ram1 (
                    .clk(clk), .wr_en(weight_bank_wr_en[B] && weight_write_slot),
                    .wr_addr(weight_bank_wr_addr[B]), .wr_data(weight_bank_wr_data[B]),
                    .rd_addr(wt_rd_addr[B]), .rd_data(wt_bank1_rdata[B]));
                supra2_sync_word_ram #(.DATA_W(32),.DEPTH(SCALE_BANK_DEPTH),
                    .ADDR_W(SCALE_ADDR_W)) scale_ram1 (
                    .clk(clk), .wr_en(sb_wr_en && !sb_wr_sel && weight_write_slot &&
                                        (int'(sb_wr_addr)%LANES==B)),
                    .wr_addr(SCALE_ADDR_W'(sb_wr_addr/LANES)), .wr_data(sb_wr_data),
                    .rd_addr(scale_rd_addr), .rd_data(scale_bank1_rdata[B]));
                supra2_sync_word_ram #(.DATA_W(32),.DEPTH(SCALE_BANK_DEPTH),
                    .ADDR_W(SCALE_ADDR_W)) bias_ram1 (
                    .clk(clk), .wr_en(sb_wr_en && sb_wr_sel && weight_write_slot &&
                                        (int'(sb_wr_addr)%LANES==B)),
                    .wr_addr(SCALE_ADDR_W'(sb_wr_addr/LANES)), .wr_data(sb_wr_data),
                    .rd_addr(scale_rd_addr), .rd_data(bias_bank1_rdata[B]));
            end else begin : g_no_weight_second_slot
                assign wt_bank1_rdata[B]='0;
                assign scale_bank1_rdata[B]='0;
                assign bias_bank1_rdata[B]='0;
            end
            assign wt_bank_rdata[B] = DUAL_WEIGHT_BUFFERS && weight_read_slot ? wt_bank1_rdata[B] : wt_bank0_rdata[B];
            assign scale_bank_rdata[B] = DUAL_WEIGHT_BUFFERS && weight_read_slot ? scale_bank1_rdata[B] : scale_bank0_rdata[B];
            assign bias_bank_rdata[B] = DUAL_WEIGHT_BUFFERS && weight_read_slot ? bias_bank1_rdata[B] : bias_bank0_rdata[B];
        end
    endgenerate

`ifndef SYNTHESIS
    // Simulation-only compatibility mirrors for existing DMA/load assertions.
    // These aliases are excluded from Vivado synthesis; functional storage is
    // exclusively implemented by the banked synchronous RAM instances above.
    logic signed [7:0] wt_int8 [0:MAX_WEIGHT_ELEMS-1];
    logic [31:0] ws_fp32 [0:MAX_N-1];
    logic [31:0] bias_fp32 [0:MAX_N-1];
    logic [15:0] f16_act [0:MAX_M*MAX_K-1];
    logic [15:0] f16_wt [0:MAX_F16_N*MAX_F16_K-1];
    always_ff @(posedge clk) begin
        if (wt_wr_en)
            for (int lane=0; lane<LANES; lane++) begin
                if (int'(wt_wr_beat)*LANES+lane < MAX_WEIGHT_ELEMS)
                    wt_int8[int'(wt_wr_beat)*LANES+lane] <= wt_wr_data[lane*8 +: 8];
            end
        if (sb_wr_en && sb_wr_addr < MAX_N) begin
            if (sb_wr_sel) bias_fp32[sb_wr_addr] <= sb_wr_data;
            else ws_fp32[sb_wr_addr] <= sb_wr_data;
        end
        if (f16_wr_en)
            for (int lane=0; lane<LANES; lane++) begin
                if (f16_wr_sel) begin
                    if (int'(f16_wr_beat)*LANES+lane < MAX_F16_N*MAX_F16_K)
                        f16_wt[int'(f16_wr_beat)*LANES+lane] <= f16_wr_data[lane*16 +: 16];
                end else if (int'(f16_wr_beat)*LANES+lane < MAX_M*MAX_K) begin
                    f16_act[int'(f16_wr_beat)*LANES+lane] <= f16_wr_data[lane*16 +: 16];
                end
            end
    end
`endif

    // ---------------- frontend (INT8 mode) ----------------
    logic                     fe_start;
    logic                     fe_in_valid;
    logic                     fe_h_in_ready;
    logic                     fe_h_out_valid;
    logic [LANES*16-1:0]      fe_h_out_data;
    logic                     fe_q_in_ready;
    logic [31:0]              fe_q_scale;
    logic                     fe_h_busy, fe_h_done, fe_q_busy, fe_q_done;

    supra2_hadamard_transform #(
        .MAX_K(MAX_K), .MAX_H(256), .LANES(LANES), .K_W(12), .H_W(9)
    ) u_hadamard (
        .clk(clk), .rst_n(rst_n), .start(fe_start),
        .vector_k(12'(cfg_k)), .block_size(cfg_h),
        .in_valid(fe_in_valid), .in_ready(fe_h_in_ready),
        .in_fp16(raw_rd_data), .out_valid(fe_h_out_valid),
        .out_ready(fe_q_in_ready),
        .out_fp16(fe_h_out_data),
        .busy(fe_h_busy), .done(fe_h_done)
    );

    supra2_activation_quantizer #(
        .MAX_K(MAX_K), .LANES(LANES), .K_W(12)
    ) u_quantizer (
        .clk(clk), .rst_n(rst_n), .start(fe_start),
        .vector_k(12'(cfg_k)), .in_valid(fe_h_out_valid),
        .in_ready(fe_q_in_ready), .in_fp16(fe_h_out_data),
        .out_valid(fe_q_out_valid), .out_ready(1'b1),
        .out_int8(fe_q_out_data), .scale_fp32(fe_q_scale),
        .busy(fe_q_busy), .done(fe_q_done)
    );

    // ---------------- systolic array ----------------
    logic                   clear_acc;
    logic                   array_en;
    logic signed [7:0]      a_col [LANES];
    logic signed [7:0]      b_row [LANES];
    logic signed [31:0]     acc_out [LANES][LANES];
    logic                   acc_valid;

    assign shared_array_clear = clear_acc;
    assign shared_array_en = array_en;
    for (genvar array_lane = 0; array_lane < LANES; array_lane++) begin : g_shared_array_ports
        assign shared_array_a[array_lane] = a_col[array_lane];
        assign shared_array_b[array_lane] = b_row[array_lane];
    end

    generate
        if (EXTERNAL_ARRAY) begin : g_external_array
            for (genvar array_row = 0; array_row < LANES; array_row++) begin : g_acc_row
                for (genvar array_col = 0; array_col < LANES; array_col++) begin : g_acc_col
                    assign acc_out[array_row][array_col] =
                        shared_array_acc[array_row][array_col];
                end
            end
            assign acc_valid = shared_array_acc_valid;
        end else begin : g_internal_array
            systolic_array #(
                .M(LANES), .N(LANES), .DATA_W(8), .ACC_W(32), .PIPE_STAGE(32)
            ) u_array (
                .clk(clk), .rst_n(rst_n), .clear_acc(clear_acc), .en(array_en),
                .a_unsigned(1'b0), .active_rows(6'(LANES)),
                .a_col(a_col), .b_row(b_row),
                .acc_out(acc_out), .acc_valid(acc_valid)
            );
        end
    endgenerate

    // ---------------- epilogue (combinational) ----------------
    logic [LANES*32-1:0] epi_acc_bus, epi_ws_bus, epi_bias_bus;
    logic [31:0]         epi_token_scale;
    logic [LANES*16-1:0] epi_output_bus;

    supra2_qlinear_epilogue #(.LANES(LANES)) u_epilogue (
        .acc_int32(epi_acc_bus), .token_scale_fp32(epi_token_scale),
        .weight_scale_fp32(epi_ws_bus), .bias_fp32(epi_bias_bus),
        .output_fp16(epi_output_bus)
    );

    // ---------------- FSM ----------------
    // Snapshot the accumulator grid when acc_valid arrives: the comb epilogue
    // must NOT track acc_out during feed (it would re-evaluate 32 lanes of
    // FP32 RNE arithmetic on every accumulator update -- a ~4x XSim slowdown
    // observed during bring-up). Declared after state_t: XSim 2021.1 requires
    // declaration before use.
    logic signed [31:0] acc_snap [LANES][LANES];
    always_ff @(posedge clk) begin
        if (state == S_WAIT_ACC && acc_valid && !drain_active) acc_snap <= acc_out;
    end

    logic [15:0] beats_per_row;

    wire [15:0] m_tiles = (cfg_m + LANES - 1) / LANES;
    wire [15:0] n_tiles = (cfg_n + LANES - 1) / LANES;
    wire [31:0] feed_cycles = {16'd0, cfg_k} + LANES + LANES + 3;

    // frontend quantized output collection (streams during FEED and DRAIN)
    wire fe_collect = (fe_state == FE_FEED || fe_state == FE_DRAIN);
    always_ff @(posedge clk) begin
        if (fe_state == FE_START) obeat <= '0;
        if (fe_collect && fe_q_out_valid) begin
            if (obeat == 0) token_scale[row] <= fe_q_scale;
            obeat <= obeat + 1'b1;
        end
    end

    // array feed (combinational skew, matching the verified testbench timing)
    always_comb begin
        for (int i = 0; i < LANES; i++) begin
            a_col[i] = '0;
            b_row[i] = '0;
        end
        if (state == S_FEED) begin
            for (int i = 0; i < LANES; i++) begin
                if (feed_read_valid[i]) begin
                    if (cfg_fp16)
                        a_col[i] = fp16_to_int8_scaled(
                            f16_act_bank_rdata[i][feed_half_index[i]*16 +: 16],
                            cfg_fp16_act_scale);
                    else
                        a_col[i] = act_bank_rdata[i][feed_byte_index[i]*8 +: 8];
                end
            end
            for (int j = 0; j < LANES; j++) begin
                if (feed_read_valid[j]) begin
                    if (cfg_fp16)
                        b_row[j] = fp16_to_int8_scaled(
                            f16_wt_bank_rdata[j][feed_half_index[j]*16 +: 16],
                            cfg_fp16_wt_scale);
                    else
                        b_row[j] = wt_bank_rdata[j][feed_byte_index[j]*8 +: 8];
                end
            end
        end
    end

    // drain path (combinational epilogue / fp16 scale for current drain row)
    always_comb begin
        for (int j = 0; j < LANES; j++) begin
            epi_acc_bus[j*32 +: 32]  = acc_snap[drain_i][j];
            epi_ws_bus[j*32 +: 32]   = scale_bank_rdata[j];
            epi_bias_bus[j*32 +: 32] = bias_bank_rdata[j];
        end
        epi_token_scale = token_scale[int'(drain_mi)+int'(drain_i)];
    end

    assign busy = (state != S_IDLE) && (state != S_DONE);
    assign array_en = (state == S_FEED) || (state == S_TILE_CLEAR);
    assign clear_acc = (state == S_TILE_CLEAR);
    assign beats_per_row = (cfg_k + LANES - 1) / LANES;
    // The raw read boundary is asynchronous. Hold its address and valid
    // through every Hadamard block-processing/output stall.
    assign raw_row = row;
    assign raw_beat = beat;
    assign fe_in_valid = (fe_state == FE_FEED);
    // Request the first word before feeding, then prefetch each successor on
    // acceptance of the current word. This supports synchronous BRAM reads
    // without inserting bubbles in the frontend stream.
    always_comb begin
        raw_prefetch_en = 1'b0;
        raw_prefetch_row = row;
        raw_prefetch_beat = '0;
        if (fe_state == FE_START) begin
            raw_prefetch_en = 1'b1;
        end else if (fe_state == FE_FEED && fe_h_in_ready &&
                     beat < beats_per_row - 1'b1) begin
            raw_prefetch_en = 1'b1;
            raw_prefetch_beat = beat + 1'b1;
        end
    end

    // column-slice end: cfg_ni1 == 0 selects the full configured width
    wire [15:0] ni_lim = (cfg_ni1 == 16'd0) ? cfg_n : cfg_ni1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            fe_state <= FE_IDLE; activation_ready <= '0;
            row <= '0; beat <= '0;
            mi <= '0; ni <= '0; t <= '0; drain_i <= '0;
            drain_active <= 1'b0; drain_mi <= '0; drain_ni <= '0;
            fe_start <= 1'b0;
            out_valid <= 1'b0; out_row <= '0; out_ct <= '0; out_data <= '0;
            done <= 1'b0;
        end else begin
            fe_start <= 1'b0;
            out_valid <= 1'b0;
            done <= 1'b0;
            // 前端按行独立推进；完整 32 行就绪后阵列可使用该 M tile。
            // 两个 M tile 槽轮换；前两组的所有 N tile 用完后才能复用槽。
            case (fe_state)
                FE_IDLE: if (state == S_IDLE && start && !cfg_fp16) begin
                    activation_ready <= '0;
                    row <= '0;
                    fe_state <= FE_START;
                end
                FE_START: begin
                    fe_start <= 1'b1;
                    beat <= '0;
                    fe_state <= FE_FEED;
                end
                FE_FEED: if (fe_h_in_ready) begin
                    if (beat == beats_per_row - 1) begin
                        beat <= '0;
                        fe_state <= FE_DRAIN;
                    end else beat <= beat + 1'b1;
                end
                FE_DRAIN: if (obeat == beats_per_row) begin
                    if ((int'(row)%LANES == LANES-1) || row == cfg_m-1)
                        activation_ready[int'(row)/LANES] <= 1'b1;
                    if (row == cfg_m-1) fe_state <= FE_IDLE;
                    else begin
                        row <= row + 1'b1;
                        fe_state <= (int'(row)%LANES == LANES-1) ? FE_WAIT_SLOT : FE_START;
                    end
                end
                FE_WAIT_SLOT: if (int'(row)/LANES < ACT_SLOTS ||
                    !activation_ready[int'(row)/LANES-ACT_SLOTS]) fe_state <= FE_START;
                default: fe_state <= FE_IDLE;
            endcase
            // 快照是完整 K 累加值；读出使用独立索引，阵列可计算下一个 tile。
            // 捕获下一份快照前必须等本次读出完成，避免覆盖在途结果。
            if (drain_active) begin
                out_valid <= 1'b1;
                out_row <= drain_mi + {10'd0, drain_i};
                out_ct <= drain_ni / LANES;
                if (cfg_fp16) begin
                    for (int j = 0; j < LANES; j++) begin
                        logic [31:0] scaled_acc;
                        scaled_acc = fp32_mul_rne(
                            fp32_from_int32_rne(acc_snap[drain_i][j]), cfg_scale);
                        if (cfg_fp16_bias_en)
                            scaled_acc = fp32_add_rne(scaled_acc, bias_bank_rdata[j]);
                        out_data[j*16 +: 16] <= fp32_to_fp16(scaled_acc);
                    end
                end else out_data <= epi_output_bus;
                if (drain_i == LANES - 1) begin
                    drain_active <= 1'b0;
                    drain_i <= '0;
                end else drain_i <= drain_i + 1'b1;
            end
            case (state)
                S_IDLE: begin
                    if (start) begin
                        mi <= '0; ni <= cfg_ni0; row <= '0;
                        if (cfg_fp16) begin
                            state <= S_TILE_CLEAR;
                        end else begin
                            state <= S_FE_DRAIN;
                        end
                    end
                end
                S_FE_DRAIN: if (activation_ready[int'(mi)/LANES])
                    state <= S_TILE_CLEAR;
                // ---- GEMM ----
                S_TILE_CLEAR: begin
                    t <= '0;
                    state <= S_FEED;
                end
                S_FEED: begin
                    if (t == feed_cycles[15:0] - 1) state <= S_WAIT_ACC;
                    else t <= t + 1'b1;
                end
                S_WAIT_ACC: begin
                    if (acc_valid && !drain_active) begin
                        drain_i <= '0;
                        drain_active <= 1'b1;
                        drain_mi <= mi;
                        drain_ni <= ni;
                        if (ni == ni_lim - LANES) begin
                            ni <= cfg_ni0;
                            if (!cfg_fp16) activation_ready[int'(mi)/LANES] <= 1'b0;
                            if (mi == cfg_m - LANES) state <= S_DRAIN;
                            else begin
                                mi <= mi + LANES;
                                state <= cfg_fp16 ? S_TILE_CLEAR : S_FE_DRAIN;
                            end
                        end else begin ni <= ni + LANES; state <= S_TILE_CLEAR; end
                    end
                end
                S_DRAIN: begin
                    if (!drain_active) state <= S_DONE;
                end
                S_DONE: begin
                    done <= 1'b1;
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
