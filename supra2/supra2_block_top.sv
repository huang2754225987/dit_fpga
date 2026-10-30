// =============================================================================
// supra2_block_top.sv - Connected one-block Supra2 DiT datapath (M=256)
//
// First connected-datapath bring-up top: runs ONE transformer block
// (self-attn -> cross-attn -> gated MLP) at M=256 entirely on-chip, with no
// file bridges between operators. Stages are sequenced by a serial FSM;
// GEMMs share one supra2_gemm_engine (Hadamard + quantizer + 32x32 array +
// FP32 epilogue), attention PV reuses the verified online-softmax batch
// units, and LN/GELU/residual use the verified 32-lane streaming units.
//
// Weight/parameter/input loading happens through tb-driven ports that model
// DMA (wload_req/stage/ack handshake per GEMM stage). AdaLN scale/shift/gate
// parameters are host-precomputed fixtures, matching the serial chain
// contract. This is a serial-schedule datapath: the pipelined tile schedule
// (3.66M cycles at M=512) is a separate, later integration.
// =============================================================================
`default_nettype none

module supra2_block_top
  import fp16_utils_pkg::*;
#(
    parameter bit RESULT_VEC_PORT = 1'b0,
    parameter bit PROGRAMMED_STAGES = 1'b0,
    parameter bit PROGRAMMED_TILES = 1'b0,
    parameter bit WEIGHT_PREFETCH = 1'b0,
    parameter MLP_RAM_STYLE = "ultra"
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         start,
    // Segmented-simulation entry controls (tie to 0 for a full run):
    // start_stage jumps the FSM straight to a mid-block stage, start_head
    // selects the first attention head of an attention stage, and start_ni
    // offsets the GEMM engine's output-column tile origin. stop_ni bounds
    // the column-tile range from the right (0 = full configured width), so
    // a GEMM stage can be split into disjoint [start_ni, stop_ni) slices.
    input  wire [4:0]   start_stage,
    input  wire [3:0]   start_head,
    input  wire [15:0]  start_ni,
    input  wire [15:0]  stop_ni,
    input  wire         stage_cmd_valid,
    input  wire [4:0]   stage_cmd_id,
    output wire         stage_cmd_ready,
    input  wire         tile_cmd_valid,
    input  wire [127:0] tile_cmd_word,
    output wire         tile_cmd_ready,
    output logic        tile_cmd_done,
    output logic        busy,
    output logic        done,

    // tb element load port (models DMA): FP16 tensors
    input  wire         ld_en,
    input  wire [3:0]   ld_target,   // 0=x_buf, 1=ctx_buf, 2=param_buf
    input  wire [20:0]  ld_addr,
    input  wire [15:0]  ld_data,
    // 512-bit vector load: one beat writes 32 consecutive FP16 elements.
    input  wire         ld_vec_en,
    input  wire [3:0]   ld_vec_target,
    input  wire [15:0]  ld_vec_beat,
    input  wire [511:0] ld_vec_data,

    // Wide runtime AdaLN table load: one 512-bit beat writes 32 FP16 values.
    input  wire         param_vec_wr_en,
    input  wire [7:0]   param_vec_wr_beat,
    input  wire [511:0] param_vec_wr_data,

    // tb weight load port (models DMA), passed to the GEMM engine
    input  wire         wt_wr_en,
    input  wire [15:0]  wt_wr_beat,
    input  wire [255:0] wt_wr_data,
    input  wire         sb_wr_en,
    input  wire         sb_wr_sel,
    input  wire [15:0]  sb_wr_addr,
    input  wire [31:0]  sb_wr_data,

    // Result addresses are sampled on clk while idle/finished; data returns
    // one cycle later. RESULT_VEC_PORT selects the shared read-port address.
    input  wire [14:0]  result_rd_beat,
    output wire [127:0] result_rd_data,
    input  wire [12:0]  result_vec_rd_beat,
    output wire [511:0] result_vec_rd_data,

    // weight-load handshake
    output logic        wload_req,
    output logic [3:0]  wload_stage,
    input  wire         wload_ack,

    output logic [4:0]  stage_id
);

    // ---------------- geometry ----------------
    localparam int ROWS = 256, FEAT = 576, HEADS = 9, HD = 64;
    localparam int XN = ROWS*FEAT;             // 147,456
    localparam int QKV_N = 1728, MLP_N = 2304, CKV_N = 1152;
    localparam int MAX_QWEIGHT_ELEMENTS = MLP_N * FEAT;
    localparam int SCORE_N = HEADS*ROWS*ROWS;  // 589,824
    localparam int CKEYS = 32;
    localparam int CTX_BEATS = (CKEYS*FEAT)/32;

    // ---------------- stage ids ----------------
    localparam logic [4:0]
        ST_LN0 = 0,  ST_QKV = 1,  ST_ATTN_SELF = 2,  ST_SPROJ = 3,
        ST_RES0 = 4, ST_LN1 = 5,  ST_CQ = 6,         ST_CKV = 7,
        ST_ATTN_CROSS = 8, ST_CPROJ = 9, ST_RES1 = 10, ST_LN2 = 11,
        ST_MLP0 = 12, ST_GELU = 13, ST_MLP2 = 14,    ST_RES2 = 15,
        ST_FIN = 16, ST_IDLE = 17, ST_WAIT = 18, ST_WLOAD = 19,
        ST_NOOP = 20;

    // engine src/dest encodings
    localparam logic [1:0] SRC_LN = 0, SRC_CTX = 1, SRC_ATTN = 2, SRC_MLP = 3;
    localparam logic [2:0]
        DST_QKV = 0, DST_PROJ = 1, DST_CQ = 2, DST_CKV = 3,
        DST_MLP = 4, DST_SCORE = 5, DST_CSCORE = 6;

    // param_buf layout: ln0_scale, ln0_shift, ln1_scale, ln1_shift,
    //                   ln2_scale, ln2_shift, gate0, gate1 (8 x 576)
    localparam int P_GATE0 = 3456, P_GATE1 = 4032;

    // One enabled synchronous read and one arbitrated write per physical bank.
    localparam int FEATURE_BEATS = XN/32;
    localparam int MLP_BEATS = ROWS*MLP_N/32;
    logic x_we, ln_we, v_we, proj_we, mlp_we, q_we, k_we, ctx_we, param_we;
    logic x_re, ln_re, v_re, proj_re, mlp_re, q_re, k_re, ctx_re, param_re;
    logic [12:0] x_wa, ln_wa, v_wa, proj_wa, q_wa, k_wa;
    logic [12:0] x_ra, ln_ra, v_ra, proj_ra, q_ra, k_ra;
    logic [14:0] mlp_wa, mlp_ra;
    logic [9:0] ctx_wa, ctx_ra;
    logic [7:0] param_wa, param_ra;
    logic [511:0] x_wd, ln_wd, v_wd, proj_wd, mlp_wd, q_wd, k_wd;
    logic [511:0] ctx_wd, param_wd;
    wire [511:0] x_rd, ln_rd, v_rd, proj_rd, mlp_rd, q_rd, k_rd;
    wire [511:0] ctx_rd, param_read_beat;
    logic [31:0] x_wmask, ln_wmask, ctx_wmask, param_wmask;
    logic [1:0] result_subword_q;
    logic q_load_q, k_load_q, v_load_q;
    logic [15:0] qk_load_beat_q;
    logic [4:0] v_load_lane_q;
    logic ln_input_q;
    logic [4:0] ln_input_row_q, ln_input_beat_q;

    // Small score tiles intentionally retain their asynchronous-read-capable
    // distributed implementation; the registered consumer remains unchanged.
    (* ram_style = "distributed" *) logic [511:0] score_buf [0:31];
    (* ram_style = "distributed" *) logic [511:0] cscore_buf [0:31];
`ifndef SYNTHESIS
    // Observation/checkpoint mirrors only. All datapath reads use the RAM
    // instances below in both behavioral simulation and synthesis.
    logic [15:0] x_buf [0:XN-1], ln_buf [0:XN-1], v_buf [0:XN-1];
    logic [15:0] cv_buf [0:HEADS*CKEYS*HD-1], attn_buf [0:XN-1];
    logic [15:0] proj_buf [0:XN-1], mlp_buf [0:ROWS*MLP_N-1];
    logic [511:0] q_buf [0:FEATURE_BEATS-1], k_buf [0:FEATURE_BEATS-1];
    logic [511:0] cq_buf [0:FEATURE_BEATS-1];
    logic [511:0] ck_buf [0:HEADS*CKEYS*(HD/32)-1];
    logic [15:0] q_probe_buf [0:XN-1], k_probe_buf [0:XN-1];
    logic [15:0] cq_probe_buf [0:XN-1], ck_probe_buf [0:HEADS*CKEYS*HD-1];
    logic [15:0] ctx_buf [0:CKEYS*FEAT-1], param_buf [0:4607];
    logic [15:0] score_probe_buf [0:SCORE_N-1];
    logic [15:0] cscore_probe_buf [0:HEADS*ROWS*CKEYS-1];
`endif

    // ---------------- FSM state ----------------
    logic [4:0]  stage;
    logic [3:0]  ss;
    logic [15:0] c0, c1, c2, c3;
    logic [2:0]  cur_dest;
    logic [3:0]  cur_head;
    logic [3:0]  attn_key_tile;
    logic        attn_v_loaded;
    logic [2:0]  command_row_tile;
    logic [3:0]  command_head;
    logic [6:0]  weights_loaded;

    wire is_self = (stage == ST_ATTN_SELF);
    wire attn_stage = is_self || (stage == ST_ATTN_CROSS);
    wire res_stage = (stage == ST_RES0) || (stage == ST_RES1) ||
                     (stage == ST_RES2);

    // per-stage engine configuration signals (driven combinationally below)
    logic       gemm_stage;
    logic [3:0] wload_of_stage;
    logic [1:0] cur_src;

    // Simulation-speed clock gating: each big array only toggles while its
    // stage owns it. State retention is safe because each unit is idle
    // outside its stage. (Bring-up optimization, not a synthesis construct.)
    // Tile-program DMA_WEIGHT runs while the controller is in ST_WLOAD.
    // Keep the engine clock alive so its weight, scale, and bias RAM ports
    // capture the streamed beats before the following GEMM_TILE.
    wire eng_cg = gemm_stage || attn_stage ||
                  (WEIGHT_PREFETCH && (wload_req || wt_wr_en || sb_wr_en)) ||
                  (PROGRAMMED_TILES && stage == ST_WLOAD);
    wire eng_clk = clk & eng_cg;
    wire pv_clk  = clk & is_self;
    wire cpv_clk = clk & (stage == ST_ATTN_CROSS);

    assign stage_id = stage;
    assign busy = (stage != ST_IDLE) && (stage != ST_FIN);
    assign done = (stage == ST_FIN);
    assign stage_cmd_ready = PROGRAMMED_STAGES && (stage == ST_WAIT);
    assign tile_cmd_ready = PROGRAMMED_TILES && (stage == ST_WAIT);

    // ---------------- GEMM engine ----------------
    logic         eng_start, eng_busy, eng_done;
    logic         eng_cfg_fp16;
    logic [15:0]  eng_cfg_m, eng_cfg_n, eng_cfg_k;
    logic [15:0]  eng_raw_row, eng_raw_beat;
    logic         eng_raw_prefetch_en;
    logic [15:0]  eng_raw_prefetch_row, eng_raw_prefetch_beat;
    logic [511:0] eng_raw_data;
    logic         eng_f16_wr_en, eng_f16_wr_sel;
    logic [15:0]  eng_f16_wr_beat;
    logic [511:0] eng_f16_wr_data;
    logic         eng_out_valid;
    logic [15:0]  eng_out_row, eng_out_ct;
    logic [511:0] eng_out_data;
    logic         eng_array_clear, eng_array_en;
    logic signed [7:0] eng_array_a [32], eng_array_b [32];
    logic signed [31:0] shared_pv_acc [32][32];
    logic shared_pv_acc_valid;

    supra2_gemm_engine #(
        .MAX_WEIGHT_ELEMS(MAX_QWEIGHT_ELEMENTS),
        // Self-attention loads all 256 keys into the FP16 weight bank.
        // The engine default only covers 64 keys, which silently truncates
        // the K operand for this model's 256x256 attention matrix.
        .MAX_F16_N(ROWS), .EXTERNAL_ARRAY(1'b1),
        .DUAL_WEIGHT_BUFFERS(WEIGHT_PREFETCH)
    ) u_eng (
        .clk(eng_clk), .rst_n(rst_n),
        .cfg_fp16(eng_cfg_fp16), .cfg_fp16_bias_en(1'b0),
        .cfg_fp16_act_scale(32'h4180_0000),
        .cfg_fp16_wt_scale(32'h4180_0000),
        .cfg_m(eng_cfg_m), .cfg_n(eng_cfg_n),
        .cfg_k(eng_cfg_k),
        // Hadamard block size must match the software golden fixtures:
        // K=576 GEMMs use h=64 (576=64*9), K=2304 (MLP2) uses h=256.
        // A non-dividing block size makes the Hadamard frontend reject start
        // and hangs the INT8 frontend (observed during block-top bring-up).
        .cfg_h((eng_cfg_k == 16'd576) ? 9'd64 : 9'd256),
        .cfg_ni0(start_ni), .cfg_ni1(stop_ni),
        // QK operands are quantized to Q4 at the INT8 array boundary;
        // 1/sqrt(head_dim) therefore becomes 1/(16*16*8) after the dot.
        .cfg_scale(32'h3a00_0000),
        .start(eng_start), .busy(eng_busy), .done(eng_done),
        .raw_row(eng_raw_row), .raw_beat(eng_raw_beat),
        .raw_prefetch_en(eng_raw_prefetch_en),
        .raw_prefetch_row(eng_raw_prefetch_row),
        .raw_prefetch_beat(eng_raw_prefetch_beat),
        .raw_rd_data(eng_raw_data),
        .f16_wr_en(eng_f16_wr_en), .f16_wr_sel(eng_f16_wr_sel),
        .f16_wr_beat(eng_f16_wr_beat), .f16_wr_data(eng_f16_wr_data),
        .wt_wr_en(wt_wr_en), .wt_wr_beat(wt_wr_beat), .wt_wr_data(wt_wr_data),
        .weight_write_slot(wload_stage[0]), .weight_read_slot(wload_of_stage[0]),
        .weight_load_k((wload_stage == 4'd6) ? 16'd2304 : 16'd576),
        .sb_wr_en(sb_wr_en), .sb_wr_sel(sb_wr_sel),
        .sb_wr_addr(sb_wr_addr), .sb_wr_data(sb_wr_data),
        .out_valid(eng_out_valid), .out_row(eng_out_row),
        .out_ct(eng_out_ct), .out_data(eng_out_data),
        .shared_array_clear(eng_array_clear), .shared_array_en(eng_array_en),
        .shared_array_a(eng_array_a), .shared_array_b(eng_array_b),
        .shared_array_acc(shared_pv_acc),
        .shared_array_acc_valid(shared_pv_acc_valid)
    );

    // per-stage engine configuration (fully combinational)
    always_comb begin
        gemm_stage = 1'b1;
        eng_cfg_fp16 = 1'b0;
        eng_cfg_m = PROGRAMMED_TILES ? 16'd32 : ROWS;
        eng_cfg_n = QKV_N; eng_cfg_k = FEAT;
        cur_src = SRC_LN;
        wload_of_stage = 4'd0;
        if (attn_stage) begin
            gemm_stage = 1'b0;
            eng_cfg_fp16 = 1'b1;                 // QK runs FP16
            eng_cfg_m = 32;
            eng_cfg_n = 32;
            eng_cfg_k = HD;
        end else begin
            case (stage)
                ST_QKV: begin
                    eng_cfg_n = QKV_N; cur_src = SRC_LN; wload_of_stage = 4'd0;
                end
                ST_SPROJ: begin
                    eng_cfg_n = FEAT; cur_src = SRC_ATTN; wload_of_stage = 4'd1;
                end
                ST_CQ: begin
                    eng_cfg_n = FEAT; cur_src = SRC_LN; wload_of_stage = 4'd2;
                end
                ST_CKV: begin
                    eng_cfg_m = CKEYS; eng_cfg_n = CKV_N; cur_src = SRC_CTX;
                    wload_of_stage = 4'd3;
                end
                ST_CPROJ: begin
                    eng_cfg_n = FEAT; cur_src = SRC_ATTN; wload_of_stage = 4'd4;
                end
                ST_MLP0: begin
                    eng_cfg_n = MLP_N; cur_src = SRC_LN; wload_of_stage = 4'd5;
                end
                ST_MLP2: begin
                    eng_cfg_n = FEAT; eng_cfg_k = MLP_N; cur_src = SRC_MLP;
                    wload_of_stage = 4'd6;
                end
                ST_WLOAD: begin
                    gemm_stage = 1'b0;
                    // Only MLP2 has a K dimension different from FEAT; the
                    // engine's banked DMA address generator uses cfg_k while
                    // loading the weight matrix.
                    eng_cfg_k = (wload_stage == 4'd6) ? MLP_N : FEAT;
                end
                default: gemm_stage = 1'b0;
            endcase
        end
    end

    // raw_prefetch_* requests each operand one cycle before Hadamard accepts
    // it. RAM outputs hold during frontend backpressure; no extra data register.
    always_comb begin
        case (cur_src)
            SRC_CTX: eng_raw_data = ctx_rd;
            SRC_MLP: eng_raw_data = mlp_rd;
            default: eng_raw_data = ln_rd;
        endcase
    end

    // ---------------- 32-lane LN/AdaLN ----------------
    logic        ln_wr_en, ln_wr_ss_en, ln_wr_ss_sel, ln_start;
    logic [4:0]  ln_wr_row, ln_wr_beat, ln_wr_ss_beat;
    logic [511:0] ln_wr_data, ln_wr_ss_data;
    logic        ln_busy, ln_done, ln_out_valid;
    logic [4:0]  ln_out_row, ln_out_beat;
    logic [511:0] ln_out_data;

    supra2_layernorm_modulate_32lane u_ln (
        .clk(clk), .rst_n(rst_n),
        .wr_en(ln_wr_en), .wr_row(ln_wr_row), .wr_beat(ln_wr_beat),
        .wr_data(ln_wr_data),
        .wr_ss_en(ln_wr_ss_en), .wr_ss_sel(ln_wr_ss_sel),
        .wr_ss_beat(ln_wr_ss_beat), .wr_ss_data(ln_wr_ss_data),
        .start(ln_start), .busy(ln_busy), .done(ln_done),
        .out_valid(ln_out_valid), .out_row(ln_out_row),
        .out_beat(ln_out_beat), .out_data(ln_out_data)
    );

    // ---------------- 32-lane GELU / residual ----------------
    wire [511:0] x_beat = x_rd;
    wire [511:0] proj_beat = proj_rd;
    wire [511:0] mlp_beat = mlp_rd;
    logic [511:0] gate_beat;
    integer gbase, param_read_beat_addr;
    logic [7:0] param_lane_rd_addr;
    logic param_ln_rd_valid_q, param_ln_rd_sel_q;
    logic [4:0] param_ln_rd_beat_q;
    always_comb begin
        gbase = (stage == ST_RES2 ? P_GATE1 : P_GATE0) + (int'(c1)%18)*32;
        if ((stage == ST_LN0 || stage == ST_LN1 || stage == ST_LN2) && ss == 0)
            param_read_beat_addr = ln_site_base(stage)/32 +
                (c1 >= 18 ? 18 : 0) + (int'(c1)%18);
        else param_read_beat_addr = gbase/32;
        param_lane_rd_addr = 8'(param_read_beat_addr);
        gate_beat = stage == ST_RES1 ? {32{16'h3c00}} : param_read_beat;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            param_ln_rd_valid_q <= 1'b0;
            param_ln_rd_sel_q <= 1'b0;
            param_ln_rd_beat_q <= '0;
        end else begin
            param_ln_rd_valid_q <=
                (stage == ST_LN0 || stage == ST_LN1 || stage == ST_LN2) &&
                ss == 0 && c1 < 36;
            if ((stage == ST_LN0 || stage == ST_LN1 || stage == ST_LN2) &&
                ss == 0 && c1 < 36) begin
                param_ln_rd_sel_q <= (c1 < 18);
                param_ln_rd_beat_q <= 5'(c1 % 18);
            end
        end
    end

    assign ln_wr_ss_en = param_ln_rd_valid_q;
    assign ln_wr_ss_sel = param_ln_rd_sel_q;
    assign ln_wr_ss_beat = param_ln_rd_beat_q;
    assign ln_wr_ss_data = param_read_beat;

    wire [15:0] res_tile_start = PROGRAMMED_TILES ?
        16'(int'(command_row_tile)*576) : 16'd0;
    wire [15:0] res_tile_end = PROGRAMMED_TILES ?
        16'((int'(command_row_tile)+1)*576) : 16'd4608;
    wire [15:0] gelu_tile_start = PROGRAMMED_TILES ?
        16'(int'(command_row_tile)*32*72) : 16'd0;
    wire [15:0] gelu_tile_end = PROGRAMMED_TILES ?
        16'((int'(command_row_tile)+1)*32*72) : 16'd18432;
    wire res_in_valid = res_stage && c1 >= res_tile_start && c1 < res_tile_end;
    wire res_out_valid;
    wire [511:0] res_out_data;
    wire [511:0] residual_x_q = x_beat;
    wire [511:0] residual_update_q = proj_beat;
    logic residual_valid_q;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            residual_valid_q <= 1'b0;
        end else begin
            // Data is registered by the RAMs on this same request edge.
            residual_valid_q <= res_in_valid;
        end
    end
    supra2_residual_gate_32lane u_res (
        .clk(clk), .rst_n(rst_n),
        .in_valid(residual_valid_q), .residual_fp16(residual_x_q),
        .update_fp16(residual_update_q), .gate_fp16(gate_beat),
        .out_valid(res_out_valid), .out_fp16(res_out_data)
    );

    wire gelu_in_valid = (stage == ST_GELU) &&
        c1 >= gelu_tile_start && c1 < gelu_tile_end;
    wire gelu_out_valid;
    wire [511:0] gelu_out_data;
    logic gelu_read_valid_q;
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) gelu_read_valid_q <= 1'b0;
        else gelu_read_valid_q <= gelu_in_valid;
    supra2_gelu_tanh_32lane u_gelu (
        .clk(clk), .rst_n(rst_n),
        .in_valid(gelu_read_valid_q), .in_fp16(mlp_beat),
        .out_valid(gelu_out_valid), .out_fp16(gelu_out_data)
    );

    // ---------------- online-softmax PV units ----------------
    logic         pv_v_load_start, pv_v_load_ready, pv_v_load_valid;
    logic         pv_v_load_accept;
    logic [15:0]  pv_v_load_fp16;
    logic         pv_batch_start, pv_batch_ready;
    wire          pv_score_ready;
    wire          pv_score_valid_self;
    logic [511:0] pv_scores;
    wire          pv_out_valid;
    logic [15:0]  pv_out_fp16;
    logic         pv_busy, pv_done;

    logic         cpv_v_load_start, cpv_v_load_ready, cpv_v_load_valid;
    logic         cpv_v_load_accept;
    logic [15:0]  cpv_v_load_fp16;
    logic         cpv_batch_start, cpv_batch_ready;
    wire          cpv_score_ready;
    wire          cpv_score_valid_x;
    logic [511:0] cpv_scores;
    wire          cpv_out_valid;
    logic [15:0]  cpv_out_fp16;
    logic         cpv_busy, cpv_done;

    // score valid: comb, aligned with the comb score data
    assign pv_score_valid_self =
        is_self && (ss == 4'd7) && pv_score_ready;
    assign cpv_score_valid_x =
        (stage == ST_ATTN_CROSS) && (ss == 4'd7) && cpv_score_ready;

    logic pv_array_clear, pv_array_en, cpv_array_clear, cpv_array_en;
    logic signed [7:0] pv_array_a [32], pv_array_b [32];
    logic signed [7:0] cpv_array_a [32], cpv_array_b [32];
    logic signed [7:0] shared_pv_a [32], shared_pv_b [32];
    logic signed [7:0] shared_array_a [32], shared_array_b [32];
    logic shared_array_en, shared_array_clear;
    logic shared_array_a_unsigned;

    supra2_attention_online_pv_batch #(
        .TOKENS(ROWS), .HEAD_DIM(HD), .EXTERNAL_ARRAY(1'b1)) u_pv_self (
        .clk(pv_clk), .rst_n(rst_n),
        .v_load_start(pv_v_load_start), .v_load_ready(pv_v_load_ready),
        .v_load_valid(pv_v_load_valid), .v_load_accept(pv_v_load_accept),
        .v_load_fp16(pv_v_load_fp16),
        .batch_start(pv_batch_start), .batch_ready(pv_batch_ready),
        .valid_keys(9'd256),
        .score_valid(pv_score_valid_self), .score_ready(pv_score_ready),
        .scores_fp16(pv_scores), .out_valid(pv_out_valid), .out_ready(1'b1),
        .out_fp16(pv_out_fp16), .busy(pv_busy), .done(pv_done),
        .array_clear(pv_array_clear), .array_en(pv_array_en),
        .array_a_col(pv_array_a), .array_b_row(pv_array_b),
        .external_array_acc(shared_pv_acc),
        .external_array_acc_valid(shared_pv_acc_valid)
    );

    supra2_attention_online_pv_batch #(
        .TOKENS(CKEYS), .HEAD_DIM(HD), .EXTERNAL_ARRAY(1'b1)) u_pv_cross (
        .clk(cpv_clk), .rst_n(rst_n),
        .v_load_start(cpv_v_load_start), .v_load_ready(cpv_v_load_ready),
        .v_load_valid(cpv_v_load_valid), .v_load_accept(cpv_v_load_accept),
        .v_load_fp16(cpv_v_load_fp16),
        .batch_start(cpv_batch_start), .batch_ready(cpv_batch_ready),
        .valid_keys(9'd15),
        .score_valid(cpv_score_valid_x), .score_ready(cpv_score_ready),
        .scores_fp16(cpv_scores), .out_valid(cpv_out_valid), .out_ready(1'b1),
        .out_fp16(cpv_out_fp16), .busy(cpv_busy), .done(cpv_done),
        .array_clear(cpv_array_clear), .array_en(cpv_array_en),
        .array_a_col(cpv_array_a), .array_b_row(cpv_array_b),
        .external_array_acc(shared_pv_acc),
        .external_array_acc_valid(shared_pv_acc_valid)
    );

    always_comb begin
        shared_pv_a = (stage == ST_ATTN_CROSS) ? cpv_array_a : pv_array_a;
        shared_pv_b = (stage == ST_ATTN_CROSS) ? cpv_array_b : pv_array_b;
        shared_array_a = eng_array_en ? eng_array_a : shared_pv_a;
        shared_array_b = eng_array_en ? eng_array_b : shared_pv_b;
        shared_array_en = eng_array_en ||
            (is_self && pv_array_en) ||
            ((stage == ST_ATTN_CROSS) && cpv_array_en);
        shared_array_clear = eng_array_clear ||
            (is_self && pv_array_clear) ||
            ((stage == ST_ATTN_CROSS) && cpv_array_clear);
        // GEMM and QK both use signed A; PV uses unsigned probabilities.
        // Select by the active client because QK runs during attention stages.
        shared_array_a_unsigned = !eng_array_en;
    end
    systolic_array #(.M(32), .N(32), .DATA_W(8), .ACC_W(32),
                     .UNSIGNED_A(1'b0), .RUNTIME_UNSIGNED_A(1'b1),
                     .PIPE_STAGE(32)) u_shared_array (
        .clk(clk), .rst_n(rst_n),
        .clear_acc(shared_array_clear), .en(shared_array_en),
        .a_unsigned(shared_array_a_unsigned), .active_rows(6'd32),
        .a_col(shared_array_a), .b_row(shared_array_b),
        .acc_out(shared_pv_acc), .acc_valid(shared_pv_acc_valid)
    );

    // Packed score memories are synchronously read; ss=9 provides the BRAM
    // read latency between accepted score beats.
    integer score_rd_addr, cscore_rd_addr;
    always_comb begin
        score_rd_addr = int'(c3);
        cscore_rd_addr = int'(c3);
    end

    // LN site parameter base: stage -> param_buf site offset
    function automatic int ln_site_base(input logic [4:0] st);
        case (st)
            ST_LN0: ln_site_base = 0;
            ST_LN1: ln_site_base = 1152;
            default: ln_site_base = 2304;
        endcase
    endfunction

    function automatic logic [3:0] weight_index_for_stage(input logic [7:0] st);
        case (st)
            ST_QKV: weight_index_for_stage=4'd0;
            ST_SPROJ: weight_index_for_stage=4'd1;
            ST_CQ: weight_index_for_stage=4'd2;
            ST_CKV: weight_index_for_stage=4'd3;
            ST_CPROJ: weight_index_for_stage=4'd4;
            ST_MLP0: weight_index_for_stage=4'd5;
            ST_MLP2: weight_index_for_stage=4'd6;
            default: weight_index_for_stage=4'hf;
        endcase
    endfunction

    // ---------------- physical memory ports ----------------
    supra2_masked_sync_ram #(.DEPTH(FEATURE_BEATS), .ADDR_W(13)) u_x_ram (
        .clk(clk), .wr_en(x_we), .wr_addr(x_wa), .wr_data(x_wd), .wr_mask(x_wmask),
        .rd_en(x_re), .rd_addr(x_ra), .rd_data(x_rd));
    supra2_masked_sync_ram #(.DEPTH(FEATURE_BEATS), .ADDR_W(13)) u_ln_ram (
        .clk(clk), .wr_en(ln_we), .wr_addr(ln_wa), .wr_data(ln_wd), .wr_mask(ln_wmask),
        .rd_en(ln_re), .rd_addr(ln_ra), .rd_data(ln_rd));
    supra2_masked_sync_ram #(.DEPTH(FEATURE_BEATS), .ADDR_W(13),
        .MASKED_WRITES(1'b0)) u_v_ram (
        .clk(clk), .wr_en(v_we), .wr_addr(v_wa), .wr_data(v_wd), .wr_mask('1),
        .rd_en(v_re), .rd_addr(v_ra), .rd_data(v_rd));
    supra2_masked_sync_ram #(.DEPTH(FEATURE_BEATS), .ADDR_W(13),
        .MASKED_WRITES(1'b0)) u_proj_ram (
        .clk(clk), .wr_en(proj_we), .wr_addr(proj_wa), .wr_data(proj_wd), .wr_mask('1),
        .rd_en(proj_re), .rd_addr(proj_ra), .rd_data(proj_rd));
    supra2_masked_sync_ram #(.DEPTH(MLP_BEATS), .ADDR_W(15),
        .MASKED_WRITES(1'b0), .RAM_STYLE(MLP_RAM_STYLE)) u_mlp_ram (
        .clk(clk), .wr_en(mlp_we), .wr_addr(mlp_wa), .wr_data(mlp_wd), .wr_mask('1),
        .rd_en(mlp_re), .rd_addr(mlp_ra), .rd_data(mlp_rd));
    supra2_masked_sync_ram #(.DEPTH(FEATURE_BEATS), .ADDR_W(13),
        .MASKED_WRITES(1'b0)) u_q_ram (
        .clk(clk), .wr_en(q_we), .wr_addr(q_wa), .wr_data(q_wd), .wr_mask('1),
        .rd_en(q_re), .rd_addr(q_ra), .rd_data(q_rd));
    supra2_masked_sync_ram #(.DEPTH(FEATURE_BEATS), .ADDR_W(13),
        .MASKED_WRITES(1'b0)) u_k_ram (
        .clk(clk), .wr_en(k_we), .wr_addr(k_wa), .wr_data(k_wd), .wr_mask('1),
        .rd_en(k_re), .rd_addr(k_ra), .rd_data(k_rd));
    supra2_masked_sync_ram #(.DEPTH(CTX_BEATS), .ADDR_W(10)) u_ctx_ram (
        .clk(clk), .wr_en(ctx_we), .wr_addr(ctx_wa), .wr_data(ctx_wd), .wr_mask(ctx_wmask),
        .rd_en(ctx_re), .rd_addr(ctx_ra), .rd_data(ctx_rd));
    supra2_masked_sync_ram #(.DEPTH(144), .ADDR_W(8)) u_param_ram (
        .clk(clk), .wr_en(param_we), .wr_addr(param_wa), .wr_data(param_wd), .wr_mask(param_wmask),
        .rd_en(param_re), .rd_addr(param_ra), .rd_data(param_read_beat));

    integer qkv_part, qkv_col, self_write_beat, cross_write_beat, out_global_row;
    integer raw_read_beat, v_read_word;
    wire ln_site = stage == ST_LN0 || stage == ST_LN1 || stage == ST_LN2;
    wire ln_input_request = ln_site && ss == 1 && c1 < 576;
    wire query_load_request = attn_stage && ss == 0 && attn_v_loaded &&
        !(is_self && attn_key_tile != 0 && !pv_score_ready);
    always_comb begin
        // A single priority mux feeds each write port. Whole-beat writes and
        // scalar lane writes use one address with explicit 16-bit enables.
        x_we=1'b0; x_wa='0; x_wd='0; x_wmask='1;
        ln_we=1'b0; ln_wa='0; ln_wd='0; ln_wmask='1;
        v_we=1'b0; v_wa='0; v_wd=eng_out_data;
        proj_we=1'b0; proj_wa='0; proj_wd=eng_out_data;
        mlp_we=1'b0; mlp_wa='0; mlp_wd=eng_out_data;
        q_we=1'b0; q_wa='0; q_wd=eng_out_data;
        k_we=1'b0; k_wa='0; k_wd=eng_out_data;
        ctx_we=1'b0; ctx_wa='0; ctx_wd='0; ctx_wmask='1;
        param_we=1'b0; param_wa='0; param_wd='0; param_wmask='1;
        qkv_part=int'(eng_out_ct)/18;
        qkv_col=int'(eng_out_ct)%18;
        out_global_row=int'(eng_out_row)+
            (PROGRAMMED_TILES ? int'(command_row_tile)*32 : 0);
        self_write_beat=(qkv_col>>1)*512 + out_global_row*2 + (qkv_col&1);
        cross_write_beat=(qkv_col>>1)*64 + out_global_row*2 + (qkv_col&1);
        if (res_out_valid && res_stage) begin
            x_we=1'b1; x_wa=13'(c3); x_wd=res_out_data;
        end else if (ld_vec_en && ld_vec_target==0 && ld_vec_beat<FEATURE_BEATS) begin
            x_we=1'b1; x_wa=13'(ld_vec_beat); x_wd=ld_vec_data;
        end else if (ld_en && ld_target==0 && ld_addr<XN) begin
            x_we=1'b1; x_wa=13'(ld_addr>>5); x_wd={32{ld_data}};
            x_wmask=32'b1<<ld_addr[4:0];
        end
        if ((cpv_out_valid && stage==ST_ATTN_CROSS) || (pv_out_valid && is_self)) begin
            ln_we=1'b1; ln_wa=13'(int'(cur_head)*512 + int'(c2)*64 + (int'(c3)>>5));
            ln_wd={32{is_self ? pv_out_fp16 : cpv_out_fp16}};
            ln_wmask=32'b1<<c3[4:0];
        end else if (ln_out_valid) begin
            ln_we=1'b1;
            ln_wa=13'((int'(c2)*32+int'(ln_out_row))*18+int'(ln_out_beat));
            ln_wd=ln_out_data;
        end
        if (eng_out_valid) begin
            case (cur_dest)
                DST_QKV: begin
                    if (qkv_part==0) begin q_we=1'b1; q_wa=13'(self_write_beat); end
                    else if (qkv_part==1) begin k_we=1'b1; k_wa=13'(self_write_beat); end
                    else begin v_we=1'b1; v_wa=13'(self_write_beat); end
                end
                DST_CQ: begin
                    q_we=1'b1;
                    q_wa=13'((int'(eng_out_ct)>>1)*512+out_global_row*2+(int'(eng_out_ct)&1));
                end
                DST_CKV: begin
                    if (qkv_part==0) begin k_we=1'b1; k_wa=13'(cross_write_beat); end
                    else begin v_we=1'b1; v_wa=13'(cross_write_beat); end
                end
                DST_PROJ: begin
                    proj_we=1'b1; proj_wa=13'(out_global_row*18+int'(eng_out_ct));
                end
                DST_MLP: begin
                    mlp_we=1'b1; mlp_wa=15'(out_global_row*72+int'(eng_out_ct));
                end
                default: ;
            endcase
        end
        if (gelu_out_valid && stage==ST_GELU) begin
            mlp_we=1'b1; mlp_wa=15'(c3); mlp_wd=gelu_out_data;
        end
        if (ld_vec_en && ld_vec_target==1 && ld_vec_beat<CTX_BEATS) begin
            ctx_we=1'b1; ctx_wa=10'(ld_vec_beat); ctx_wd=ld_vec_data;
        end else if (ld_en && ld_target==1 && ld_addr<CKEYS*FEAT) begin
            ctx_we=1'b1; ctx_wa=10'(ld_addr>>5); ctx_wd={32{ld_data}};
            ctx_wmask=32'b1<<ld_addr[4:0];
        end
        if (param_vec_wr_en && param_vec_wr_beat<144) begin
            param_we=1'b1; param_wa=param_vec_wr_beat; param_wd=param_vec_wr_data;
        end else if (ld_vec_en && ld_vec_target==2 && ld_vec_beat<144) begin
            param_we=1'b1; param_wa=8'(ld_vec_beat); param_wd=ld_vec_data;
        end else if (ld_en && ld_target==2 && ld_addr<4608) begin
            param_we=1'b1; param_wa=8'(ld_addr>>5); param_wd={32{ld_data}};
            param_wmask=32'b1<<ld_addr[4:0];
        end

        // Stage ownership guarantees these read clients are mutually exclusive.
        // Engine requests have no ready input and must never be postponed.
        x_re=1'b0; x_ra='0; ln_re=1'b0; ln_ra='0;
        v_re=1'b0; v_ra='0; proj_re=1'b0; proj_ra='0;
        mlp_re=1'b0; mlp_ra='0; q_re=1'b0; q_ra='0; k_re=1'b0; k_ra='0;
        ctx_re=1'b0; ctx_ra='0; param_re=1'b0; param_ra=param_lane_rd_addr;
        raw_read_beat=0; v_read_word=0;
        if (ln_input_request) begin
            x_re=1'b1; x_ra=13'(int'(c2)*576+int'(c1));
        end else if (res_in_valid) begin
            x_re=1'b1; x_ra=13'(c1); proj_re=1'b1; proj_ra=13'(c1);
        end else if ((stage==ST_IDLE || stage==ST_FIN) && !x_we && !start) begin
            // External readers own the port only after compute has drained.
            // RESULT_VEC_PORT selects its word address; the bridge maps scalar
            // debug addresses to that word when its vector client is inactive.
            x_ra=RESULT_VEC_PORT ? result_vec_rd_beat : 13'(result_rd_beat>>2);
            // The DMA advances its read index to FEATURE_BEATS after the last
            // output beat is accepted. No RAM access is needed at that index.
            x_re=(int'(x_ra)<FEATURE_BEATS);
        end
        if (gelu_in_valid) begin mlp_re=1'b1; mlp_ra=15'(c1); end
        if (eng_raw_prefetch_en && gemm_stage) begin
            case (cur_src)
                SRC_CTX: begin
                    ctx_re=1'b1;
                    ctx_ra=10'((int'(eng_raw_prefetch_row)+
                        (PROGRAMMED_TILES ? int'(command_row_tile)*32 : 0))*18+
                        int'(eng_raw_prefetch_beat));
                end
                SRC_MLP: begin
                    mlp_re=1'b1;
                    mlp_ra=15'((int'(eng_raw_prefetch_row)+
                        (PROGRAMMED_TILES ? int'(command_row_tile)*32 : 0))*72+
                        int'(eng_raw_prefetch_beat));
                end
                default: begin
                    ln_re=1'b1;
                    if (cur_src==SRC_ATTN)
                        raw_read_beat=(int'(eng_raw_prefetch_beat)>>1)*512+
                            (int'(eng_raw_prefetch_row)+
                             (PROGRAMMED_TILES ? int'(command_row_tile)*32 : 0))*2+
                            (int'(eng_raw_prefetch_beat)&1);
                    else raw_read_beat=(int'(eng_raw_prefetch_row)+
                        (PROGRAMMED_TILES ? int'(command_row_tile)*32 : 0))*18+
                        int'(eng_raw_prefetch_beat);
                    ln_ra=13'(raw_read_beat);
                end
            endcase
        end
        if (query_load_request) begin
            q_re=1'b1; q_ra=13'(int'(cur_head)*512+int'(c2)*64+int'(c1));
        end
        if (attn_stage && ss==1) begin
            k_re=1'b1;
            k_ra=is_self ? 13'(int'(cur_head)*512+int'(attn_key_tile)*64+int'(c1)) :
                13'(int'(cur_head)*64+int'(c1));
        end
        if (attn_stage && ss==5) begin
            v_read_word=is_self ? int'(cur_head)*16384+(int'(c1)%ROWS)*HD+int'(c1)/ROWS :
                int'(cur_head)*2048+(int'(c1)%CKEYS)*HD+int'(c1)/CKEYS;
            v_re=1'b1; v_ra=13'(v_read_word>>5);
        end
        param_re=(ln_site && ss==0 && c1<36) || res_in_valid;
    end

    // RAM data and this metadata become valid together after the request edge.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_load_q<=1'b0; k_load_q<=1'b0; v_load_q<=1'b0;
            qk_load_beat_q<='0; v_load_lane_q<='0; result_subword_q<='0;
            ln_input_q<=1'b0; ln_input_row_q<='0; ln_input_beat_q<='0;
        end else begin
            q_load_q<=q_re; k_load_q<=k_re; v_load_q<=v_re;
            ln_input_q<=ln_input_request;
            if (q_re || k_re) qk_load_beat_q<=c1;
            if (v_re) v_load_lane_q<=5'(v_read_word);
            if (x_re && (stage==ST_IDLE || stage==ST_FIN))
                result_subword_q<=result_rd_beat[1:0];
            if (ln_input_request) begin
                ln_input_row_q<=5'(c1/18); ln_input_beat_q<=5'(c1%18);
            end
        end
    end
    assign result_rd_data=x_rd[result_subword_q*128 +: 128];
    assign result_vec_rd_data=x_rd;
    assign ln_wr_en=ln_input_q;
    assign ln_wr_row=ln_input_row_q;
    assign ln_wr_beat=ln_input_beat_q;
    assign ln_wr_data=x_rd;
    assign eng_f16_wr_en=q_load_q || k_load_q;
    assign eng_f16_wr_sel=k_load_q;
    assign eng_f16_wr_beat=qk_load_beat_q;
    assign eng_f16_wr_data=k_load_q ? k_rd : q_rd;
    assign pv_v_load_valid=v_load_q && is_self;
    assign cpv_v_load_valid=v_load_q && stage==ST_ATTN_CROSS;
    assign pv_v_load_fp16=v_rd[v_load_lane_q*16 +: 16];
    assign cpv_v_load_fp16=v_rd[v_load_lane_q*16 +: 16];

`ifndef SYNTHESIS
    // Segment checkpoints restore observation mirrors first, then explicitly
    // seed the physical banks. This task is not a second datapath implementation.
    task automatic restore_probe_memories(input logic [4:0] resume_stage);
        for (int i=0; i<XN; i++) begin
            u_x_ram.mem[i/32][(i%32)*16 +: 16]=x_buf[i];
            u_proj_ram.mem[i/32][(i%32)*16 +: 16]=proj_buf[i];
            if ((resume_stage>=ST_ATTN_SELF && resume_stage<=ST_RES0) ||
                (resume_stage>=ST_ATTN_CROSS && resume_stage<=ST_RES1))
                u_ln_ram.mem[i/32][(i%32)*16 +: 16]=attn_buf[i];
            else u_ln_ram.mem[i/32][(i%32)*16 +: 16]=ln_buf[i];
            u_q_ram.mem[i/32][(i%32)*16 +: 16]=
                resume_stage>=ST_CQ ? cq_probe_buf[i] : q_probe_buf[i];
            if (resume_stage>=ST_CKV) begin
                u_k_ram.mem[i/32][(i%32)*16 +: 16]=
                    i<HEADS*CKEYS*HD ? ck_probe_buf[i] : 16'b0;
                u_v_ram.mem[i/32][(i%32)*16 +: 16]=
                    i<HEADS*CKEYS*HD ? cv_buf[i] : 16'b0;
            end else begin
                u_k_ram.mem[i/32][(i%32)*16 +: 16]=k_probe_buf[i];
                u_v_ram.mem[i/32][(i%32)*16 +: 16]=v_buf[i];
            end
        end
        for (int i=0; i<ROWS*MLP_N; i++)
            u_mlp_ram.mem[i/32][(i%32)*16 +: 16]=mlp_buf[i];
        for (int i=0; i<CKEYS*FEAT; i++)
            u_ctx_ram.mem[i/32][(i%32)*16 +: 16]=ctx_buf[i];
        for (int i=0; i<4608; i++)
            u_param_ram.mem[i/32][(i%32)*16 +: 16]=param_buf[i];
    endtask
`endif

    // ---------------- buffer writes ----------------
    integer waddr;
    integer which_qkv, c_in_which;
    always_ff @(posedge clk) begin
        pv_scores <= score_buf[score_rd_addr];
        cpv_scores <= cscore_buf[cscore_rd_addr];
        // tb loads (FSM idle)
        if (ld_en) begin
            case (ld_target)
`ifndef SYNTHESIS
                4'd0: x_buf[ld_addr] <= ld_data;
`endif
                4'd1: begin
`ifndef SYNTHESIS
                    ctx_buf[ld_addr] <= ld_data;
`endif
                end
`ifndef SYNTHESIS
                4'd2: param_buf[ld_addr] <= ld_data;
`endif
                default: ;
            endcase
        end
        if (ld_vec_en) begin
            for (int lane = 0; lane < 32; lane++) begin
                case (ld_vec_target)
`ifndef SYNTHESIS
                    4'd0: x_buf[int'(ld_vec_beat)*32+lane] <=
                        ld_vec_data[lane*16+:16];
`endif
                    4'd1: begin
`ifndef SYNTHESIS
                        ctx_buf[int'(ld_vec_beat)*32+lane] <=
                            ld_vec_data[lane*16+:16];
`endif
                    end
`ifndef SYNTHESIS
                    4'd2: param_buf[int'(ld_vec_beat)*32+lane] <=
                        ld_vec_data[lane*16+:16];
`endif
                    default: ;
                endcase
            end
        end
`ifndef SYNTHESIS
        if (param_vec_wr_en)
            for (int lane = 0; lane < 32; lane++)
                param_buf[int'(param_vec_wr_beat)*32 + lane] <=
                    param_vec_wr_data[lane*16 +: 16];
`endif
        // GEMM engine drain
        if (eng_out_valid) begin
            case (cur_dest)
                DST_QKV: begin
                    which_qkv = int'(eng_out_ct) / 18;
                    c_in_which = int'(eng_out_ct) % 18;
                    // 2026-10-21 watchdog fix: waddr must be buffer-relative;
                    // the which_qkv*XN base pushed k/v writes out of bounds
                    // (k_buf/v_buf stayed all-X, self-attention ran on X).
                    waddr = (c_in_which>>1)*16384 +
                            (int'(eng_out_row)+(PROGRAMMED_TILES ? int'(command_row_tile)*32 : 0))*HD +
                            (c_in_which&1)*32;
                    if (which_qkv == 0) begin
`ifndef SYNTHESIS
                        q_buf[waddr>>5] <= eng_out_data;
                        for (int l = 0; l < 32; l++)
                            q_probe_buf[waddr+l] <= eng_out_data[l*16+:16];
`endif
                    end else if (which_qkv == 1) begin
`ifndef SYNTHESIS
                        k_buf[waddr>>5] <= eng_out_data;
                        for (int l = 0; l < 32; l++)
                            k_probe_buf[waddr+l] <= eng_out_data[l*16+:16];
`endif
                    end else begin
`ifndef SYNTHESIS
                        for (int l = 0; l < 32; l++)
                            v_buf[waddr+l] <= eng_out_data[l*16+:16];
`endif
                    end
                end
                DST_PROJ: begin
                    waddr = (int'(eng_out_row)+(PROGRAMMED_TILES ? int'(command_row_tile)*32 : 0))*FEAT +
                            int'(eng_out_ct)*32;
`ifndef SYNTHESIS
                    for (int l = 0; l < 32; l++)
                        proj_buf[waddr + l] <= eng_out_data[l*16 +: 16];
`endif
                end
                DST_CQ: begin
                    waddr = (int'(eng_out_ct)>>1)*16384 +
                            (int'(eng_out_row)+(PROGRAMMED_TILES ? int'(command_row_tile)*32 : 0))*HD +
                            (int'(eng_out_ct)&1)*32;
`ifndef SYNTHESIS
                    cq_buf[waddr>>5] <= eng_out_data;
                    for (int l = 0; l < 32; l++)
                        cq_probe_buf[waddr+l] <= eng_out_data[l*16+:16];
`endif
                end
                DST_CKV: begin
                    which_qkv = int'(eng_out_ct) / 18;
                    c_in_which = int'(eng_out_ct) % 18;
                    waddr = (c_in_which>>1)*2048 +
                            (int'(eng_out_row)+(PROGRAMMED_TILES ? int'(command_row_tile)*32 : 0))*HD +
                            (c_in_which&1)*32;
                    if (which_qkv == 0) begin
`ifndef SYNTHESIS
                        ck_buf[waddr>>5] <= eng_out_data;
                        for (int l = 0; l < 32; l++)
                            ck_probe_buf[waddr+l] <= eng_out_data[l*16+:16];
`endif
                    end else begin
`ifndef SYNTHESIS
                        for (int l = 0; l < 32; l++)
                            cv_buf[waddr+l] <= eng_out_data[l*16+:16];
`endif
                    end
                end
                DST_MLP: begin
                    waddr = (int'(eng_out_row)+(PROGRAMMED_TILES ? int'(command_row_tile)*32 : 0))*MLP_N +
                            int'(eng_out_ct)*32;
`ifndef SYNTHESIS
                    for (int l = 0; l < 32; l++)
                        mlp_buf[waddr + l] <= eng_out_data[l*16 +: 16];
`endif
                end
                DST_SCORE: begin
                    waddr = int'(eng_out_row);
                    score_buf[waddr] <= eng_out_data;
                    for (int l = 0; l < 32; l++) begin
`ifndef SYNTHESIS
                        score_probe_buf[(int'(cur_head)*ROWS + int'(c2)*32 +
                            int'(eng_out_row))*ROWS + int'(attn_key_tile)*32 +
                            int'(eng_out_ct)*32 + l] <= eng_out_data[l*16 +: 16];
`endif
                    end
                end
                DST_CSCORE: begin
                    waddr = int'(eng_out_row);
                    cscore_buf[waddr] <= eng_out_data;
                    for (int l = 0; l < 32; l++) begin
`ifndef SYNTHESIS
                        cscore_probe_buf[(int'(cur_head)*ROWS + int'(c2)*32 +
                                         int'(eng_out_row))*CKEYS +
                                         int'(eng_out_ct)*32 + l] <= eng_out_data[l*16 +: 16];
`endif
                    end
                end
                default: ;
            endcase
        end
        // LN drain
        if (ln_out_valid) begin
            waddr = (int'(c2)*32 + int'(ln_out_row))*FEAT + int'(ln_out_beat)*32;
`ifndef SYNTHESIS
            for (int l = 0; l < 32; l++)
                ln_buf[waddr + l] <= ln_out_data[l*16 +: 16];
`endif
        end
        // attention PV output streams
        if (pv_out_valid && is_self) begin
            waddr = (int'(cur_head)*ROWS + int'(c2)*32 + int'(c3)/HD)*HD +
                    int'(c3)%HD;
`ifndef SYNTHESIS
            attn_buf[waddr] <= pv_out_fp16;
`endif
        end
        if (cpv_out_valid && (stage == ST_ATTN_CROSS)) begin
            waddr = (int'(cur_head)*ROWS + int'(c2)*32 + int'(c3)/HD)*HD +
                    int'(c3)%HD;
`ifndef SYNTHESIS
            attn_buf[waddr] <= cpv_out_fp16;
`endif
        end
        // residual write-back (c3 trails the input beat counter c1 by 1)
        if (res_out_valid && res_stage) begin
`ifndef SYNTHESIS
            for (int l = 0; l < 32; l++)
                x_buf[int'(c3)*32 + l] <= res_out_data[l*16 +: 16];
`endif
        end
        // GELU write-back
        if (gelu_out_valid && (stage == ST_GELU)) begin
`ifndef SYNTHESIS
            for (int l = 0; l < 32; l++)
                mlp_buf[int'(c3)*32 + l] <= gelu_out_data[l*16 +: 16];
`endif
        end
    end

    // ---------------- main FSM ----------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stage <= ST_IDLE; ss <= '0;
            c0 <= '0; c1 <= '0; c2 <= '0; c3 <= '0;
            attn_key_tile <= '0; attn_v_loaded <= 1'b0;
            cur_head <= '0; cur_dest <= DST_QKV;
            eng_start <= 1'b0;
            wload_req <= 1'b0; wload_stage <= '0;
            ln_start <= 1'b0;
            pv_v_load_start <= 1'b0;
            pv_batch_start <= 1'b0;
            cpv_v_load_start <= 1'b0;
            cpv_batch_start <= 1'b0;
            tile_cmd_done <= 1'b0;
            command_row_tile <= '0; command_head <= '0;
            weights_loaded <= '0;
        end else begin
            // default one-cycle pulses
            eng_start <= 1'b0;
            ln_start <= 1'b0;
            pv_v_load_start <= 1'b0; pv_batch_start <= 1'b0;
            cpv_v_load_start <= 1'b0; cpv_batch_start <= 1'b0;
            tile_cmd_done <= 1'b0;

            // 后台权重 DMA 与当前 stage 独立完成；请求保持到 ACK。
            if (WEIGHT_PREFETCH && wload_req && wload_ack) begin
                wload_req <= 1'b0;
                if (wload_stage < 7) weights_loaded[wload_stage] <= 1'b1;
            end

            if (gemm_stage) begin
                // ---- shared GEMM stage flow: WLOAD -> START -> WAIT ----
                case (ss)
                    4'd0: begin
                        if (WEIGHT_PREFETCH) begin
                            if (weights_loaded[wload_of_stage]) ss <= 4'd1;
                            else if (!wload_req && !wload_ack) begin
                                wload_req <= 1'b1;
                                wload_stage <= wload_of_stage;
                                for (int wi=0; wi<7; wi++)
                                    if ((wi%2)==int'(wload_of_stage[0])) weights_loaded[wi] <= 1'b0;
                            end
                        end else begin
                            wload_req <= 1'b1;
                            wload_stage <= wload_of_stage;
                            if (PROGRAMMED_TILES && weights_loaded[wload_of_stage]) begin
                                wload_req <= 1'b0;
                                ss <= 4'd1;
                            end else if (wload_ack) begin
                                wload_req <= 1'b0;
                                if (PROGRAMMED_TILES) weights_loaded[wload_of_stage] <= 1'b1;
                                ss <= 4'd1;
                            end
                        end
                    end
                    4'd1: begin
                        eng_start <= 1'b1;
                        case (stage)
                            ST_QKV:   cur_dest <= DST_QKV;
                            ST_CQ:    cur_dest <= DST_CQ;
                            ST_CKV:   cur_dest <= DST_CKV;
                            ST_MLP0:  cur_dest <= DST_MLP;
                            ST_MLP2:  cur_dest <= DST_PROJ;
                            default:  cur_dest <= DST_PROJ;
                        endcase
                        ss <= 4'd2;
                    end
                    4'd2: begin
                        // 顺序 block 调度：当前 stage 使用一个槽时预取下一 stage 到另一个槽。
                        if (WEIGHT_PREFETCH && !wload_req && !wload_ack &&
                            wload_of_stage < 6 && !weights_loaded[wload_of_stage+1'b1]) begin
                            wload_req <= 1'b1;
                            wload_stage <= wload_of_stage+1'b1;
                            for (int wi=0; wi<7; wi++)
                                if ((wi%2)!=int'(wload_of_stage[0])) weights_loaded[wi] <= 1'b0;
                        end
                        if (eng_done) begin
                            ss <= 4'd0;
                            c1 <= '0; c2 <= '0; c3 <= '0;
                            stage <= (PROGRAMMED_STAGES || PROGRAMMED_TILES) ? ST_WAIT : stage + 1'b1;
                            if (PROGRAMMED_TILES) tile_cmd_done <= 1'b1;
                        end
                    end
                    default: ss <= 4'd0;
                endcase
            end else begin
                case (stage)
                    // ---------------- LayerNorm/AdaLN sites ----------------
                    ST_LN0, ST_LN1, ST_LN2: begin
                        case (ss)
                            // fill scale/shift (36 beats) once per site
                            4'd0: begin
                                // unit port contract: wr_ss_sel 0=shift,
                                // 1=scale. c1<18 streams the scale region,
                                // c1>=18 the shift region (this was swapped
                                // during bring-up: LN outputs came out as
                                // norm*(1+shift)+scale until 2026-10-21).
                                if (c1 == 35) begin
                                    c1 <= '0;
                                    if (!PROGRAMMED_TILES) c2 <= '0;
                                    ss <= 4'd1;
                                end else c1 <= c1 + 1'b1;
                            end
                            // fill one 32-row x tile (576 beats)
                            4'd1: begin
                                // Address/metadata issue here; the RAM response
                                // drives the LN write port one cycle later.
                                if (c1 == 575) begin
                                    c1 <= '0; ss <= 4'd2;
                                end else c1 <= c1 + 1'b1;
                            end
                            4'd2: begin
                                ln_start <= 1'b1;
                                ss <= 4'd3;
                            end
                            // drain 576 beats (writes handled above)
                            4'd3: begin
                                if (ln_done) begin
                                    c1 <= '0;
                            if (PROGRAMMED_TILES || c2 == 7) begin
                                        ss <= 4'd0;
                                        if (!PROGRAMMED_TILES) c2 <= '0;
                                        stage <= (PROGRAMMED_STAGES || PROGRAMMED_TILES) ? ST_WAIT : stage + 1'b1;
                                        if (PROGRAMMED_TILES) tile_cmd_done <= 1'b1;
                                    end else begin
                                        c2 <= c2 + 1'b1;
                                        ss <= 4'd1;
                                    end
                                end
                            end
                            default: ss <= 4'd0;
                        endcase
                    end

                    // ---------------- self / cross attention ----------------
                    ST_ATTN_SELF, ST_ATTN_CROSS: begin
                        case (ss)
                            // Load one 32-row query tile (64 FP16 beats).
                            4'd0: begin
                                if (!attn_v_loaded ||
                                    (is_self && (attn_key_tile != 0) &&
                                     !pv_score_ready)) begin
                                    c1 <= '0;
                                    if (!attn_v_loaded) ss <= 4'd4;
                                end else begin
                                if (c1 == 63) begin
                                    c1 <= '0; ss <= 4'd1;
                                end else c1 <= c1 + 1'b1;
                                end
                            end
                            // Load one 32-key tile (64 FP16 beats).
                            4'd1: begin
                                if (c1 == 63) begin
                                    c1 <= '0; ss <= 4'd2;
                                end else c1 <= c1 + 1'b1;
                            end
                            // QK engine run (cfg is combinational)
                            4'd2: begin
                                eng_start <= 1'b1;
                                cur_dest <= is_self ? DST_SCORE : DST_CSCORE;
                                ss <= 4'd3;
                            end
                            4'd3: begin
                                if (eng_done) begin
                                    c1 <= '0; c3 <= '0; ss <= 4'd9;
                                end
                            end
                            // PV: V load start
                            4'd4: begin
                                if (is_self) pv_v_load_start <= 1'b1;
                                else cpv_v_load_start <= 1'b1;
                                c1 <= '0;
                                ss <= 4'd5;
                            end
                            // PV: V load stream (dim-major order)
                            4'd5: begin
                                if (is_self) begin
                                    if (c1 == 16383) begin
                                        c1 <= '0; attn_v_loaded <= 1'b1; ss <= 4'd6;
                                    end else c1 <= c1 + 1'b1;
                                end else begin
                                    if (c1 == 2047) begin
                                        c1 <= '0; attn_v_loaded <= 1'b1; ss <= 4'd6;
                                    end else c1 <= c1 + 1'b1;
                                end
                            end
                            // PV: batch start
                            4'd6: begin
                                if (is_self && pv_batch_ready) begin
                                    pv_batch_start <= 1'b1;
                                    attn_key_tile <= '0; c1 <= '0; c3 <= '0;
                                    ss <= 4'd0;
                                end else if (!is_self && cpv_batch_ready) begin
                                    cpv_batch_start <= 1'b1;
                                    attn_key_tile <= '0; c1 <= '0; c3 <= '0;
                                    ss <= 4'd0;
                                end
                            end
                            // PV: score feed from the synchronous score RAM
                            4'd7: begin
                                if (is_self ? (pv_score_ready) :
                                              (cpv_score_ready)) begin
                                    if (c3 == 31) begin
                                        c3 <= '0;
                                        if (!is_self || attn_key_tile == 7) begin
                                            c1 <= '0; c3 <= '0; ss <= 4'd8;
                                        end else begin
                                            attn_key_tile <= attn_key_tile + 1'b1;
                                            c1 <= '0; ss <= 4'd0;
                                        end
                                    end else begin
                                        c3 <= c3 + 1'b1; ss <= 4'd9;
                                    end
                                end
                            end
                            // One-cycle BRAM read latency before the next score beat.
                            4'd9: ss <= 4'd7;
                            // PV: output capture (2048 elems per batch)
                            4'd8: begin
                                if (is_self ? pv_out_valid : cpv_out_valid) begin
                                    if (c3 == 2047) begin
                                        c3 <= '0;
                                        if (PROGRAMMED_TILES || c2 == 7) begin
                                            if (cur_head == HEADS - 1) begin
                                                cur_head <= '0;
                                                ss <= 4'd0;
                                                c1 <= '0; c2 <= '0;
                                                attn_v_loaded <= 1'b0;
                                                stage <= (PROGRAMMED_STAGES || PROGRAMMED_TILES) ? ST_WAIT : stage + 1'b1;
                                                if (PROGRAMMED_TILES) tile_cmd_done <= 1'b1;
                                            end else if (PROGRAMMED_TILES) begin
                                                ss <= 4'd0;
                                                c1 <= '0;
                                                attn_v_loaded <= 1'b0;
                                                stage <= ST_WAIT;
                                                tile_cmd_done <= 1'b1;
                                            end else begin
                                                cur_head <= cur_head + 1'b1;
                                                c1 <= '0; c2 <= '0;
                                                attn_v_loaded <= 1'b0;
                                                ss <= 4'd0;
                                            end
                                        end else begin
                                            c2 <= c2 + 1'b1;
                                            attn_key_tile <= '0;
                                            ss <= 4'd6;
                                        end
                                    end else c3 <= c3 + 1'b1;
                                end
                            end
                            default: ss <= 4'd0;
                        endcase
                    end

                    // ---------------- gated/ungated residual ----------------
                    ST_RES0, ST_RES1, ST_RES2: begin
                        if (c1 < res_tile_end) c1 <= c1 + 1'b1;
                        if (res_out_valid) begin
                            if (c3 == res_tile_end-1'b1) begin
                                c1 <= '0; c3 <= '0;
                                stage <= (PROGRAMMED_STAGES || PROGRAMMED_TILES) ? ST_WAIT : stage + 1'b1;
                                if (PROGRAMMED_TILES) tile_cmd_done <= 1'b1;
                            end else c3 <= c3 + 1'b1;
                        end
                    end

                    // ---------------- GELU ----------------
                    ST_GELU: begin
                        if (c1 < gelu_tile_end) c1 <= c1 + 1'b1;
                        if (gelu_out_valid) begin
                            if (c3 == gelu_tile_end-1'b1) begin
                                c1 <= '0; c3 <= '0;
                                stage <= (PROGRAMMED_STAGES || PROGRAMMED_TILES) ? ST_WAIT : stage + 1'b1;
                                if (PROGRAMMED_TILES) tile_cmd_done <= 1'b1;
                            end else c3 <= c3 + 1'b1;
                        end
                    end

                    ST_IDLE: begin
                        if (start) begin
                            ss <= '0;
                            c1 <= '0; c2 <= '0; c3 <= '0;
                            attn_key_tile <= '0; attn_v_loaded <= 1'b0;
                            cur_head <= start_head;
                            stage <= (PROGRAMMED_STAGES || PROGRAMMED_TILES) ? ST_WAIT :
                                ((start_stage <= ST_RES2) ? start_stage : ST_LN0);
                            weights_loaded <= '0;
                            command_row_tile <= '0; command_head <= '0;
                        end
                    end
                    ST_WLOAD: begin
                        wload_req <= 1'b1;
                        if (wload_ack) begin
                            wload_req <= 1'b0;
                            if (wload_stage < 7) weights_loaded[wload_stage] <= 1'b1;
                            stage <= ST_WAIT;
                            tile_cmd_done <= 1'b1;
                        end
                    end
                    ST_NOOP: begin
                        stage <= ST_WAIT;
                        tile_cmd_done <= 1'b1;
                    end
                    ST_WAIT: begin
                        if (PROGRAMMED_TILES && tile_cmd_valid && tile_cmd_ready) begin
                            ss <= '0;
                            c1 <= '0; c2 <= 16'(tile_cmd_word[111:104]); c3 <= '0;
                            command_row_tile <= tile_cmd_word[106:104];
                            command_head <= tile_cmd_word[99:96];
                            cur_head <= tile_cmd_word[99:96];
                            attn_key_tile <= '0; attn_v_loaded <= 1'b0;
                            case (tile_cmd_word[127:120])
                                8'd1: begin
                                    wload_stage <= weight_index_for_stage(tile_cmd_word[119:112]);
                                    stage <= (weight_index_for_stage(tile_cmd_word[119:112]) < 7) ? ST_WLOAD : ST_FIN;
                                end
                                8'd7: stage <= ST_NOOP;
                                8'd8: stage <= ST_FIN;
                                8'd2, 8'd3, 8'd4, 8'd5, 8'd6: begin
                                    stage <= tile_cmd_word[119:112] <= ST_RES2 ?
                                        5'(tile_cmd_word[119:112]) : ST_FIN;
                                    if (tile_cmd_word[127:120] == 8'd5) begin
                                        c1 <= 16'(int'(tile_cmd_word[111:104])*576);
                                        c3 <= 16'(int'(tile_cmd_word[111:104])*576);
                                    end else if (tile_cmd_word[127:120] == 8'd6) begin
                                        c1 <= 16'(int'(tile_cmd_word[111:104])*32*72);
                                        c3 <= 16'(int'(tile_cmd_word[111:104])*32*72);
                                    end
                                end
                                default: stage <= ST_FIN;
                            endcase
                        end else if (stage_cmd_valid) begin
                            ss <= '0;
                            c1 <= '0; c2 <= '0; c3 <= '0;
                            cur_head <= '0;
                            stage <= (stage_cmd_id <= ST_FIN) ? stage_cmd_id : ST_FIN;
                        end
                    end
                    // Completion is a one-cycle pulse; return to idle so the
                    // outer 14-block/50-step controller can reuse this engine.
                    ST_FIN: begin
                        stage <= ST_IDLE;
                        if (PROGRAMMED_TILES) tile_cmd_done <= 1'b1;
                    end
                    default: stage <= ST_FIN;
                endcase
            end
        end
    end

endmodule

`default_nettype wire
