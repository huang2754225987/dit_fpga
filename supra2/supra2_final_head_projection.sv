`default_nettype none

// Final 576->16 projection and token-to-latent unpatchify stage.
// Input activations are already LayerNorm/AdaLN modulated FP16 values.
module supra2_final_head_projection
  import fp16_utils_pkg::*;
#(
    parameter int TOKENS = 256,
    parameter int WIDTH = 576,
    parameter int PATCH_VALUES = 16,
    parameter int LANES = 32
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  act_wr_en,
    input  wire [15:0]           act_wr_beat,
    input  wire [LANES*16-1:0]   act_wr_data,
    input  wire                  weight_wr_en,
    input  wire [15:0]           weight_wr_beat,
    input  wire [LANES*16-1:0]   weight_wr_data,
    input  wire                  bias_wr_en,
    input  wire [4:0]            bias_wr_addr,
    input  wire [15:0]           bias_wr_data,
    input  wire                  start,
    output logic                 busy,
    output logic                 done,
    output logic                 out_valid,
    input  wire                  out_ready,
    output logic                 out_last,
    output logic [LANES*16-1:0]  out_data
);
    localparam int ACT_BEATS = TOKENS*WIDTH/LANES;

    logic eng_busy, eng_done, eng_out_valid;
    logic [15:0] eng_out_row, eng_out_ct;
    logic [LANES*16-1:0] eng_out_data;
    logic [LANES*16-1:0] eng_f16_wr_data;
    logic eng_f16_wr_en, eng_f16_wr_sel;
    logic [15:0] eng_f16_wr_beat;
`ifdef SYNTHESIS
    // Unpatchify consumes one token-row per 32-value output beat. Bank by the
    // token's x coordinate so each bank has one asynchronous read and one
    // full-patch write; the old scalar array inferred 32 read ports and up to
    // 16 writes in one clock process.
    (* ram_style = "distributed" *) logic [PATCH_VALUES*16-1:0]
        token_bank [0:15][0:15];
    logic [PATCH_VALUES*16-1:0] token_row [0:15];
`else
    logic [15:0] token_mem [0:TOKENS*LANES-1];
`endif
    logic [31:0] bias_fp32;
    logic sb_wr_en;
    logic [15:0] sb_wr_addr;
    logic [6:0] beat_index;
    typedef enum logic [1:0] {S_IDLE, S_GEMM, S_UNPATCH, S_DONE} state_t;
    state_t state;

    assign sb_wr_en = bias_wr_en;
    assign sb_wr_addr = {11'd0, bias_wr_addr};
    assign busy = (state != S_IDLE) && (state != S_DONE);
    assign out_valid = (state == S_UNPATCH);
    assign out_last = out_valid && (beat_index == TOKENS*PATCH_VALUES/LANES-1);
    always_comb begin
        out_data = '0;
`ifdef SYNTHESIS
        for (int bank = 0; bank < 16; bank++)
            token_row[bank] = token_bank[bank][(int'(beat_index)%32)/2];
`endif
        for (int l=0; l<LANES; l++) begin
            int idx, x, y, c, tok, patch;
            idx = int'(beat_index)*LANES+l;
            x = idx % 32;
            y = (idx / 32) % 32;
            c = idx / 1024;
            tok = (y/2)*16+x/2;
            patch = c*4+(y%2)*2+x%2;
`ifdef SYNTHESIS
            out_data[l*16 +: 16] = token_row[tok%16][patch*16 +: 16];
`else
            out_data[l*16 +: 16] = token_mem[tok*LANES+patch];
`endif
        end
    end

`ifdef SYNTHESIS
    for (genvar token_x = 0; token_x < 16; token_x++) begin : g_token_bank
        always_ff @(posedge clk) begin
            if (rst_n && state == S_GEMM && eng_out_valid && eng_out_ct == 0 &&
                (int'(eng_out_row)%16) == token_x)
                token_bank[token_x][int'(eng_out_row)/16] <=
                    eng_out_data[PATCH_VALUES*16-1:0];
        end
    end
`endif

    supra2_gemm_engine #(
        .LANES(LANES), .MAX_M(TOKENS), .MAX_N(LANES), .MAX_K(WIDTH),
        .MAX_WEIGHT_ELEMS(1),
        .MAX_F16_K(WIDTH), .MAX_F16_N(LANES)
    ) u_engine (
        .clk(clk), .rst_n(rst_n), .cfg_fp16(1'b1), .cfg_fp16_bias_en(1'b1),
        .cfg_fp16_act_scale(32'h4203_3333), .cfg_fp16_wt_scale(32'h436a_0000),
        .cfg_m(16'(TOKENS)), .cfg_n(16'(LANES)), .cfg_k(16'(WIDTH)),
        .cfg_h(9'd0), .cfg_ni0(16'd0), .cfg_ni1(16'd0),
        .cfg_scale(32'h3908_9e64), .start(start && state == S_IDLE),
        .busy(eng_busy), .done(eng_done), .raw_row(), .raw_beat(),
        .raw_rd_data('0),
        .f16_wr_en(eng_f16_wr_en), .f16_wr_sel(eng_f16_wr_sel),
        .f16_wr_beat(eng_f16_wr_beat), .f16_wr_data(eng_f16_wr_data),
        .wt_wr_en(1'b0), .wt_wr_beat('0), .wt_wr_data('0),
        .weight_write_slot(1'b0), .weight_read_slot(1'b0), .weight_load_k(16'd0),
        .sb_wr_en(sb_wr_en), .sb_wr_sel(1'b1), .sb_wr_addr(sb_wr_addr),
        .sb_wr_data(bias_fp32), .out_valid(eng_out_valid),
        .out_row(eng_out_row), .out_ct(eng_out_ct), .out_data(eng_out_data),
        .shared_array_clear(), .shared_array_en(), .shared_array_a(),
        .shared_array_b(), .shared_array_acc(), .shared_array_acc_valid(1'b0)
    );

    always_comb begin
        eng_f16_wr_en = act_wr_en || weight_wr_en;
        eng_f16_wr_sel = weight_wr_en;
        eng_f16_wr_beat = weight_wr_en ? weight_wr_beat : act_wr_beat;
        eng_f16_wr_data = weight_wr_en ? weight_wr_data : act_wr_data;
        bias_fp32 = fp16_to_fp32(bias_wr_data);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            beat_index <= '0;
            done <= 1'b0;
        end else begin
            done <= 1'b0;
            case (state)
                S_IDLE: if (start) state <= S_GEMM;
                S_GEMM: begin
`ifndef SYNTHESIS
                    if (eng_out_valid && eng_out_ct == 0)
                        for (int j=0; j<PATCH_VALUES; j++)
                            token_mem[int'(eng_out_row)*LANES+j] <= eng_out_data[j*16 +: 16];
`endif
                    if (eng_done) begin
                        beat_index <= '0;
                        state <= S_UNPATCH;
                    end
                end
                S_UNPATCH: begin
                    if (out_ready) begin
                        if (beat_index == TOKENS*PATCH_VALUES/LANES-1)
                            state <= S_DONE;
                        else beat_index <= beat_index + 1'b1;
                    end
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
