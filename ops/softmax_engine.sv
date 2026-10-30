// =============================================================================
// softmax_engine.sv - Vector Softmax (PostProc RF interface)
// =============================================================================
// 3-pass softmax over 32 FP16 elements from a single PostProc RF register.
// Pass 1: find max  (tree reduce)
// Pass 2: exp(x-max), accumulate sum  (four-lane replicated exp LUT)
// Pass 3: normalize by 1/sum  (four lanes per cycle)
// =============================================================================
`default_nettype none

module softmax_engine
  import isa_pkg::*;
#(
    parameter int LANES  = 32,
    parameter int ELEM_W = 16
) (
    input  wire                          clk,
    input  wire                          rst_n,

    // Command from ucode_decode
    input  wire                          cmd_valid,
    output logic                         cmd_ready,
    input  wire  [3:0]                   cmd_vd,
    input  wire  [3:0]                   cmd_vs1,
    input  wire  [3:0]                   cmd_alpha_vd,
    input  wire  [6:0]                   cmd_state_slot,
    input  wire  [5:0]                   cmd_valid_lanes,
    input  wire                          cmd_online,
    input  wire                          cmd_online_reset,
    input  wire                          cmd_raw_output,
    input  wire                          cmd_finalize,

    // PostProc RF read port
    output logic [3:0]                   rf_rd_addr,
    input  wire  [LANES*ELEM_W-1:0]     rf_rd_data,

    // PostProc RF write port
    output logic                         rf_wr_en,
    output logic [3:0]                   rf_wr_addr,
    output logic [LANES*ELEM_W-1:0]     rf_wr_data,

    output logic                         busy,
    output logic                         done
);

    localparam int VEC_W = LANES * ELEM_W;
    localparam int EXP_LANES = 4;

    typedef enum logic [4:0] {
        S_IDLE,
        S_LOAD,         // latch RF read data
        S_P1_MAX,       // tree-reduce max (combinational, 1 cycle)
        S_P2_INIT,      // reset sum, index
        S_ONLINE_SCALE_ADDR,
        S_ONLINE_SCALE_LATCH,
        S_P2_EXP,       // exp LUT addresses for two consecutive lanes
        S_P2_EXP2,      // exp LUT addr+1 driven (read LUT[base+1] for interp)
        S_P2_ACC,       // latch two exp results, accumulate in lane order
        S_ONLINE_DENOM,
        S_P3_RECIP,     // drive recip LUT
        S_P3_WAIT,      // latch recip
        S_P3_NR_SUB,
        S_P3_NR_FINAL,
        S_P3_NORM,      // normalize two lanes per cycle
        S_P3_NEXT,      // advance normalization lane index
        S_WRITE,        // write result to RF
        S_WRITE_ALPHA,
        S_DONE
    } state_t;

    state_t state, state_nxt;

    // Latched command
    logic [3:0] r_vd, r_vs1, r_alpha_vd;
    logic [5:0] r_valid_lanes;
    logic r_online, r_online_reset, r_raw_output, r_finalize;
    logic [6:0] r_state_slot;
    logic r_online_valid [0:127];
    logic [15:0] r_online_max [0:127];
    logic [15:0] r_online_sum [0:127];
    logic [15:0] r_old_scale, r_old_weight, r_online_denom;
    logic [15:0] r_online_alpha;
    logic [15:0] online_old_diff;
    logic [15:0] online_old_weight;
    logic [15:0] online_new_max;

    // Internal vector storage
    logic [ELEM_W-1:0] vec_in  [0:LANES-1];
    logic [ELEM_W-1:0] vec_out [0:LANES-1];

    // Pass 1: max
    logic [ELEM_W-1:0] r_max;

    // Pass 2: exp + sum (two lanes per cycle)
    logic [4:0]        r_idx;
    logic [ELEM_W-1:0] r_sum;
    logic [ELEM_W-1:0] exp_result [0:LANES-1];

    // Pass 3: recip
    logic [ELEM_W-1:0] r_recip;
    logic [ELEM_W-1:0] r_recip_product, r_recip_correction;
    logic [ELEM_W-1:0] recip_refined;
    wire [ELEM_W-1:0] norm_denom = r_finalize
                                    ? r_online_sum[r_state_slot]
                                    : r_online ? r_online_denom : r_sum;

    // Exp LUT (FP16) - 4096-entry for precision (addr=fp16[15:4])
    logic [11:0] exp_lut_addr;
    logic [15:0] exp_lut_data;

    logic [11:0] exp_lut_lane_addr [0:EXP_LANES-1];
    logic [15:0] exp_lut_lane_data [0:EXP_LANES-1];
    logic [ELEM_W-1:0] sum_after_exp;
    logic [ELEM_W-1:0] lane_diff;

    for (genvar exp_lane = 0; exp_lane < EXP_LANES; exp_lane++) begin : g_exp_lut
        graph_exp_lut_fp164k u_exp_lut (
            .clk      (clk),
            .addr     (exp_lut_lane_addr[exp_lane]),
            .data_out (exp_lut_lane_data[exp_lane])
        );
    end
    assign exp_lut_data = exp_lut_lane_data[0];

    // Recip LUT (FP16) - 4096-entry, 1/x for fp16 sum (addr=fp16[15:4])
    logic [11:0] recip_lut_addr;
    logic [15:0] recip_lut_data;

    recip_lut4k u_recip_lut (
        .clk      (clk),
        .addr     (recip_lut_addr),
        .data_out (recip_lut_data)
    );

    // ----------------------------------------------------------------
    // FP16 helpers (imported from fp16_utils_pkg)
    // ----------------------------------------------------------------
    import fp16_utils_pkg::*;

    // Tree-reduce max over 32 elements (combinational)
    logic [ELEM_W-1:0] max_tree [0:LANES-1];
    logic [ELEM_W-1:0] max_l1 [0:15];
    logic [ELEM_W-1:0] max_l2 [0:7];
    logic [ELEM_W-1:0] max_l3 [0:3];
    logic [ELEM_W-1:0] max_l4 [0:1];
    logic [ELEM_W-1:0] max_final;

    always_comb begin
        for (int i = 0; i < LANES; i++)
            max_tree[i] = (i < r_valid_lanes) ? vec_in[i] : 16'hFC00;
        for (int i = 0; i < 16; i++)
            max_l1[i] = fp16_cmp_gt(max_tree[2*i], max_tree[2*i+1])
                        ? max_tree[2*i] : max_tree[2*i+1];
        for (int i = 0; i < 8; i++)
            max_l2[i] = fp16_cmp_gt(max_l1[2*i], max_l1[2*i+1]) ? max_l1[2*i] : max_l1[2*i+1];
        for (int i = 0; i < 4; i++)
            max_l3[i] = fp16_cmp_gt(max_l2[2*i], max_l2[2*i+1]) ? max_l2[2*i] : max_l2[2*i+1];
        for (int i = 0; i < 2; i++)
            max_l4[i] = fp16_cmp_gt(max_l3[2*i], max_l3[2*i+1]) ? max_l3[2*i] : max_l3[2*i+1];
        max_final = fp16_cmp_gt(max_l4[0], max_l4[1]) ? max_l4[0] : max_l4[1];
    end

    // FP16 subtract for current element
    logic [ELEM_W-1:0] cur_diff;
    always_comb begin
        online_new_max = (r_online_reset || !r_online_valid[r_state_slot] ||
                          fp16_cmp_gt(max_final, r_online_max[r_state_slot]))
                         ? max_final : r_online_max[r_state_slot];
        online_old_diff = fp16_sub(r_online_max[r_state_slot], r_max);
        online_old_weight = (r_online_reset || !r_online_valid[r_state_slot])
                            ? 16'h0000
                            : fp16_mul(r_online_sum[r_state_slot], r_old_scale);
        cur_diff = fp16_sub(vec_in[r_idx], r_max);
    end

    // Q0.16 to FP16 conversion for recip
    function automatic logic [15:0] q016_to_fp16(input logic [15:0] val);
        logic [4:0] msb_pos;
        logic [4:0] fp16_exp;
        logic [9:0] fp16_mant;
        integer i;
        if (val == 16'h0000) return 16'h0000;
        msb_pos = 5'd0;
        for (i = 15; i >= 0; i = i - 1)
            if (val[i]) begin msb_pos = i[4:0]; break; end
        if (msb_pos == 5'd0) return 16'h0000;
        fp16_exp = msb_pos - 5'd1;
        case (msb_pos)
            5'd1:  fp16_mant = {val[0],    9'h000};
            5'd2:  fp16_mant = {val[1:0],  8'h00};
            5'd3:  fp16_mant = {val[2:0],  7'h00};
            5'd4:  fp16_mant = {val[3:0],  6'h00};
            5'd5:  fp16_mant = {val[4:0],  5'h00};
            5'd6:  fp16_mant = {val[5:0],  4'h0};
            5'd7:  fp16_mant = {val[6:0],  3'h0};
            5'd8:  fp16_mant = {val[7:0],  2'h0};
            5'd9:  fp16_mant = {val[8:0],  1'h0};
            5'd10: fp16_mant = val[9:0];
            5'd11: fp16_mant = val[10:1];
            5'd12: fp16_mant = val[11:2];
            5'd13: fp16_mant = val[12:3];
            5'd14: fp16_mant = val[13:4];
            5'd15: fp16_mant = val[14:5];
            default: fp16_mant = 10'h000;
        endcase
        return {1'b0, fp16_exp, fp16_mant};
    endfunction

    // ----------------------------------------------------------------
    // FSM
    // ----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) state <= S_IDLE;
        else        state <= state_nxt;

    always_comb begin
        state_nxt = state;
        case (state)
            S_IDLE:    if (cmd_valid)
                           state_nxt = (cmd_finalize === 1'b1)
                                       ? S_P3_RECIP : S_LOAD;
            S_LOAD:    state_nxt = S_P1_MAX;
            S_P1_MAX:  state_nxt = S_P2_INIT;
            S_P2_INIT: begin
                if (r_online && r_online_valid[r_state_slot] &&
                    !r_online_reset)
                    state_nxt = S_ONLINE_SCALE_ADDR;
                else
                    state_nxt = S_P2_EXP;
            end
            S_ONLINE_SCALE_ADDR: state_nxt = S_ONLINE_SCALE_LATCH;
            S_ONLINE_SCALE_LATCH: state_nxt = S_P2_EXP;
            S_P2_EXP:  state_nxt = S_P2_ACC;
            S_P2_ACC: begin
                if (r_idx == LANES[4:0] - EXP_LANES[4:0])
                    state_nxt = r_online ? S_ONLINE_DENOM : S_P3_RECIP;
                else
                    state_nxt = S_P2_EXP;
            end
            S_ONLINE_DENOM: state_nxt = r_raw_output ? S_WRITE : S_P3_RECIP;
            S_P3_RECIP: state_nxt = S_P3_WAIT;
            S_P3_WAIT:  state_nxt = S_P3_NR_SUB;
            S_P3_NR_SUB: state_nxt = S_P3_NR_FINAL;
            S_P3_NR_FINAL: state_nxt = r_finalize ? S_WRITE_ALPHA : S_P3_NORM;
            S_P3_NORM: begin
                if (r_idx == LANES[4:0] - EXP_LANES[4:0])
                    state_nxt = S_WRITE;
                else
                    state_nxt = S_P3_NEXT;
            end
            S_P3_NEXT: state_nxt = S_P3_NORM;
            S_WRITE:   state_nxt = r_online ? S_WRITE_ALPHA : S_DONE;
            S_WRITE_ALPHA: state_nxt = S_DONE;
            S_DONE:    state_nxt = S_IDLE;
            default:   state_nxt = S_IDLE;
        endcase
    end

    // ----------------------------------------------------------------
    // Command latch
    // ----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_vd  <= '0;
            r_vs1 <= '0;
            r_alpha_vd <= '0;
            r_state_slot <= '0;
            r_valid_lanes <= 6'd32;
            r_online <= 1'b0;
            r_online_reset <= 1'b0;
            r_raw_output <= 1'b0;
            r_finalize <= 1'b0;
        end else if (state == S_IDLE && cmd_valid) begin
            r_vd  <= cmd_vd;
            r_vs1 <= cmd_vs1;
            r_alpha_vd <= cmd_alpha_vd;
            r_state_slot <= cmd_state_slot;
            r_valid_lanes <= cmd_valid_lanes;
            // Omitted named ports in legacy standalone testbenches are Z;
            // treat anything other than an explicit 1 as the legacy mode.
            r_online <= (cmd_online === 1'b1);
            r_online_reset <= (cmd_online_reset === 1'b1);
            r_raw_output <= (cmd_raw_output === 1'b1);
            r_finalize <= (cmd_finalize === 1'b1);
        end
    end

    // RF read: drive vs1 address during S_IDLE (for S_LOAD latch)
    assign rf_rd_addr = (state == S_IDLE) ? cmd_vs1 : r_vs1;

    // ----------------------------------------------------------------
    // Latch input vector from RF
    // ----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (state == S_LOAD) begin
            for (int i = 0; i < LANES; i++)
                vec_in[i] <= rf_rd_data[i*ELEM_W +: ELEM_W];
        end
    end

    // ----------------------------------------------------------------
    // Pass 1: latch max
    // ----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (state == S_P1_MAX)
            r_max <= r_online ? online_new_max : max_final;
    end

    // ----------------------------------------------------------------
    // Index counter
    // ----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            r_idx <= '0;
        else if (state == S_P2_INIT)
            r_idx <= '0;
        else if (state == S_P2_ACC && state_nxt == S_P2_EXP)
            r_idx <= r_idx + EXP_LANES[4:0];
        else if (state == S_P3_WAIT)
            r_idx <= '0;
        else if (state == S_P3_NORM && state_nxt == S_P3_NEXT)
            r_idx <= r_idx + EXP_LANES[4:0];
    end

    // ----------------------------------------------------------------
    // Pass 2: replicated 4096-entry LUTs evaluate four adjacent lanes in
    // parallel. The additions retain the original FP16 lane order.
    // ----------------------------------------------------------------
    assign exp_lut_addr = (state == S_P2_EXP) ? cur_diff[15:4] :
                          (state == S_ONLINE_SCALE_ADDR)
                          ? online_old_diff[15:4] : 12'd0;

    always_comb begin
        sum_after_exp = r_sum;
        lane_diff = '0;
        for (int lane = 0; lane < EXP_LANES; lane++) begin
            exp_lut_lane_addr[lane] = 12'd0;
            if (lane == 0)
                exp_lut_lane_addr[lane] = exp_lut_addr;
            else if (state == S_P2_EXP && r_idx + lane < r_valid_lanes)
                begin
                    lane_diff = fp16_sub(vec_in[r_idx + lane], r_max);
                    exp_lut_lane_addr[lane] = lane_diff[15:4];
                end

            if (r_idx + lane < r_valid_lanes)
                sum_after_exp = fp16_add(
                    sum_after_exp, exp_lut_lane_data[lane]);
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            r_sum <= 16'h0000;
        else if (state == S_P2_INIT)
            r_sum <= 16'h0000;
        else if (state == S_P2_ACC) begin
            for (int lane = 0; lane < EXP_LANES; lane++)
                exp_result[r_idx + lane] <=
                    (r_idx + lane < r_valid_lanes)
                    ? exp_lut_lane_data[lane] : 16'h0000;
            r_sum <= sum_after_exp;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_old_scale <= 16'h0000;
            r_old_weight <= 16'h0000;
            r_online_denom <= 16'h0000;
            for (int slot = 0; slot < 128; slot++) begin
                r_online_max[slot] <= 16'hFC00;
                r_online_sum[slot] <= 16'h0000;
                r_online_valid[slot] <= 1'b0;
            end
        end else begin
            if (state == S_P2_INIT) begin
                if (r_online &&
                    (r_online_reset || !r_online_valid[r_state_slot]))
                    r_old_scale <= 16'h0000;
            end else if (state == S_ONLINE_SCALE_LATCH) begin
                r_old_scale <= exp_lut_data;
            end
            if (state == S_ONLINE_DENOM) begin
                r_old_weight <= online_old_weight;
                r_online_denom <= fp16_add(r_sum, online_old_weight);
                r_online_sum[r_state_slot] <= fp16_add(r_sum, online_old_weight);
                r_online_max[r_state_slot] <= r_max;
                r_online_valid[r_state_slot] <= 1'b1;
            end
        end
    end

    // Preserve the original dedicated alpha register process.  Some Questa
    // releases diagnose the intentionally phase-exclusive writers as 7061;
    // the project Questa runners suppress that diagnostic for this unit.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            r_online_alpha <= 16'h0000;
        else begin
            if (state == S_ONLINE_DENOM && r_raw_output)
                r_online_alpha <= r_old_scale;
            if (state == S_P3_NR_FINAL && r_online)
                r_online_alpha <= r_finalize ? recip_refined
                                              : fp16_mul(r_old_weight,
                                                         recip_refined);
        end
    end

    // ----------------------------------------------------------------
    // Pass 3: recip LUT + two-lane normalize (4096-entry fp16 LUT)
    // ----------------------------------------------------------------
    assign recip_lut_addr = (state == S_P3_RECIP)
                            ? (r_finalize
                               ? r_online_sum[r_state_slot][15:4]
                               : r_online ? r_online_denom[15:4] : r_sum[15:4])
                            : 12'd0;

    always_ff @(posedge clk) begin
        if (state == S_P3_WAIT) begin
            r_recip <= recip_lut_data;
            r_recip_product <= fp16_mul(recip_lut_data, norm_denom);
        end else if (state == S_P3_NR_SUB) begin
            r_recip_correction <= fp16_sub(16'h4000, r_recip_product);
        end else if (state == S_P3_NR_FINAL) begin
            r_recip <= recip_refined;
        end
    end

    // The 4K table is indexed by only the upper 12 FP16 denominator bits.
    // One Newton-Raphson step recovers most of the discarded mantissa
    // precision: r <- r * (2 - x*r).
    always_comb begin
        recip_refined = fp16_mul(r_recip, r_recip_correction);
    end

    always_ff @(posedge clk) begin
        if (state == S_P3_NORM) begin
            for (int lane = 0; lane < EXP_LANES; lane++)
                vec_out[r_idx + lane] <=
                    (r_idx + lane < r_valid_lanes)
                    ? fp16_mul(exp_result[r_idx + lane], r_recip)
                    : 16'h0000;
        end
    end

    // ----------------------------------------------------------------
    // Write result to RF
    // ----------------------------------------------------------------
    always_comb begin
        rf_wr_en   = (state == S_WRITE || state == S_WRITE_ALPHA);
        rf_wr_addr = (state == S_WRITE_ALPHA) ? r_alpha_vd : r_vd;
        for (int i = 0; i < LANES; i++)
            rf_wr_data[i*ELEM_W +: ELEM_W] =
                (state == S_WRITE_ALPHA) ? r_online_alpha :
                r_raw_output ? exp_result[i] : vec_out[i];
    end

    // ----------------------------------------------------------------
    // Status
    // ----------------------------------------------------------------
    assign cmd_ready = (state == S_IDLE);
    assign busy      = (state != S_IDLE);
    assign done      = (state == S_DONE);

endmodule

`default_nettype wire
