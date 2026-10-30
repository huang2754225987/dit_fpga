`default_nettype none

// 32-query microbatch online attention. Scores are normalized row-by-row,
// while one 32x32 raw-exp tile is buffered and consumed by all PV rows.
module supra2_attention_online_pv_batch #(
    parameter int TOKENS=256,
    parameter int HEAD_DIM=64,
    parameter int TILE=32,
    parameter int COUNT_W=15,
    parameter bit EXTERNAL_ARRAY=1'b0
) (
    input  wire logic          clk,
    input  wire logic          rst_n,
    input  wire logic          v_load_start,
    output logic               v_load_ready,
    input  wire logic          v_load_valid,
    output logic               v_load_accept,
    input  wire logic [15:0]   v_load_fp16,
    input  wire logic          batch_start,
    input  wire logic [8:0]    valid_keys,
    output logic               batch_ready,
    input  wire logic          score_valid,
    output logic               score_ready,
    input  wire logic [TILE*16-1:0] scores_fp16,
    output logic               out_valid,
    input  wire logic          out_ready,
    output logic [15:0]        out_fp16,
    output logic               busy,
    output logic               done,
    output logic               array_clear,
    output logic               array_en,
    output logic signed [7:0]  array_a_col [32],
    output logic signed [7:0]  array_b_row [32],
    input  wire logic signed [31:0] external_array_acc [32][32],
    input  wire logic               external_array_acc_valid
);
    import fp16_utils_pkg::*;

    localparam int PV_CYCLES=3*TILE+3;
    typedef enum logic [3:0] {
        S_WAIT, S_LOAD_V, S_WAIT_BATCH, S_WAIT_SCORE, S_CMD_SOFTMAX,
        S_WAIT_SOFTMAX, S_PV_CLEAR, S_PV_FEED, S_PV_DRAIN, S_PV_MERGE,
        S_CMD_FINAL, S_WAIT_FINAL, S_OUTPUT
    } state_t;
    state_t state;

    // Feature-bank V store: 32 banks x (2 x 256 keys) x FP16.
    localparam int V_ADDR_W=$clog2(2*TOKENS);
`ifndef SYNTHESIS
    // Simulation-visible mirror for focused bank-map assertions only.
    logic [7:0] v_mem [0:TILE-1][0:(2*TOKENS)-1];
`endif
    logic [7:0] v_read_data [0:TILE-1];
    logic [V_ADDR_W-1:0] v_read_addr [0:TILE-1];
    logic v_read_en [0:TILE-1];
    logic v_read_valid [0:TILE-1];
    logic [7:0] v_read_raw [0:TILE-1];
    logic [7:0] v_prefetch_count;

    logic [15:0] raw_exp [0:TILE-1][0:TILE-1];
    logic [15:0] row_alpha [0:TILE-1];
    logic [15:0] row_reciprocal [0:TILE-1];
    logic [15:0] output_acc [0:TILE-1][0:HEAD_DIM-1];
    logic [TILE*16-1:0] score_reg;
    logic alpha_seen, final_scale_seen, v_loaded;
    logic [COUNT_W-1:0] v_count;
    logic [5:0] v_write_bank;
    logic [V_ADDR_W-1:0] v_write_addr;
    logic v_write_fire;
    localparam int V_COUNT=TOKENS*HEAD_DIM;
    localparam int KEY_TILES=TOKENS/TILE;
    logic [7:0] key_tile;
    logic feature_tile;
    logic [5:0] row_index;
    logic [4:0] merge_row;
    logic [6:0] feed_count, drain_count;
    logic [10:0] output_index;
    logic [15:0] merged_tile_values [0:TILE-1];

    logic sm_cmd_valid, sm_cmd_ready, sm_done, sm_busy;
    logic [3:0] sm_rf_rd_addr, sm_rf_wr_addr;
    logic [TILE*16-1:0] sm_rf_rd_data, sm_rf_wr_data;
    logic sm_rf_wr_en;

    logic array_acc_valid;
    logic signed [31:0] array_acc [32][32];

    logic [5:0] sm_valid_lanes;
    integer remaining_valid_keys;
    integer v_write_feature, v_write_key;
    always_comb begin
        v_write_feature = int'(v_count) / TOKENS;
        v_write_key = int'(v_count) % TOKENS;
        v_write_bank = 6'(v_write_feature % TILE);
        v_write_addr = V_ADDR_W'((v_write_feature / TILE)*TOKENS + v_write_key);
        v_write_fire = (state==S_LOAD_V) && v_load_valid && v_load_accept;
        remaining_valid_keys = int'(valid_keys) - int'(key_tile)*TILE;
        if (remaining_valid_keys >= TILE) sm_valid_lanes = 6'(TILE);
        else if (remaining_valid_keys > 0) sm_valid_lanes = 6'(remaining_valid_keys);
        else sm_valid_lanes = 6'd0;
    end

    softmax_engine #(.LANES(TILE), .ELEM_W(16)) u_softmax (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(sm_cmd_valid), .cmd_ready(sm_cmd_ready),
        .cmd_vd(4'd1), .cmd_vs1(4'd0), .cmd_alpha_vd(4'd2),
        .cmd_state_slot(7'(row_index)), .cmd_valid_lanes(sm_valid_lanes),
        .cmd_online(1'b1), .cmd_online_reset(key_tile==0),
        .cmd_raw_output(state==S_CMD_SOFTMAX),
        .cmd_finalize(state==S_CMD_FINAL),
        .rf_rd_addr(sm_rf_rd_addr), .rf_rd_data(sm_rf_rd_data),
        .rf_wr_en(sm_rf_wr_en), .rf_wr_addr(sm_rf_wr_addr),
        .rf_wr_data(sm_rf_wr_data), .busy(sm_busy), .done(sm_done)
    );

    generate
        for (genvar bank=0; bank<TILE; bank++) begin : g_v_bank
            supra2_sync_word_ram #(
                .DATA_W(8), .DEPTH(2*TOKENS), .ADDR_W(V_ADDR_W)
            ) u_v_mem (
                .clk(clk),
                .wr_en(v_write_fire && (v_write_bank==bank)),
                .wr_addr(v_write_addr),
                .wr_data(fp16_to_int8_q(v_load_fp16,4)),
                .rd_addr(v_read_addr[bank]),
                .rd_data(v_read_raw[bank])
            );
        end
        if (EXTERNAL_ARRAY) begin : g_external_array
            always_comb begin
                array_acc = external_array_acc;
                array_acc_valid = external_array_acc_valid;
            end
        end else begin : g_local_array
            systolic_array #(.M(32), .N(32), .DATA_W(8), .ACC_W(32),
                             .UNSIGNED_A(1'b1), .PIPE_STAGE(32)) u_pv_array (
                .clk(clk), .rst_n(rst_n), .clear_acc(array_clear), .en(array_en),
                .a_unsigned(1'b1),
                .active_rows(6'(TILE)),
                .a_col(array_a_col), .b_row(array_b_row),
                .acc_out(array_acc), .acc_valid(array_acc_valid)
            );
        end
    endgenerate

    always_comb begin
        v_load_ready=(state==S_WAIT || state==S_WAIT_BATCH || state==S_LOAD_V);
        v_load_accept=v_load_ready;
        batch_ready=(state==S_WAIT_BATCH) && v_loaded;
        score_ready=(state==S_WAIT_SCORE);
        sm_cmd_valid=(state==S_CMD_SOFTMAX || state==S_CMD_FINAL);
        sm_rf_rd_data=score_reg;
        array_clear=(state==S_PV_CLEAR);
        array_en=(state==S_PV_CLEAR || state==S_PV_FEED || state==S_PV_DRAIN);
        for(int m=0;m<32;m++) array_a_col[m]='0;
        for(int n=0;n<TILE;n++) begin
            merged_tile_values[n]=fp16_add(
                fp16_mul(output_acc[merge_row][feature_tile*TILE+n],
                         row_alpha[merge_row]),
                fp32_to_fp16(fp32_mul_rne(
                    fp32_from_int32_rne(array_acc[merge_row][n]),
                    32'h3980_0000)));
        end
        for(int n=0;n<32;n++) begin
            array_b_row[n]='0;
            array_b_row[n]=v_read_valid[n] ? v_read_raw[n] : 8'h00;
            v_read_addr[n]='0;
            v_read_en[n]=1'b0;
        end
        v_prefetch_count=(state==S_PV_CLEAR)?8'd0:({1'b0,feed_count}+8'd1);
        if(state==S_PV_CLEAR || state==S_PV_FEED)
            for(int n=0;n<TILE;n++)
                if(v_prefetch_count>=n && (v_prefetch_count-n)<TILE) begin
                    v_read_en[n]=1'b1;
                    v_read_addr[n]=V_ADDR_W'(feature_tile*TOKENS+
                        key_tile*TILE+(v_prefetch_count-n));
                end
        if(state==S_PV_FEED) begin
            for(int m=0;m<TILE;m++)
                if(feed_count>=m && (feed_count-m)<TILE)
                    array_a_col[m]=fp16_to_uint8_q(raw_exp[m][feed_count-m],8);
        end
        out_valid=(state==S_OUTPUT);
        out_fp16=16'h0000;
        if(state==S_OUTPUT)
            out_fp16=fp16_mul(
                output_acc[output_index/HEAD_DIM][output_index%HEAD_DIM],
                row_reciprocal[output_index/HEAD_DIM]);
        busy=(state!=S_WAIT && state!=S_WAIT_BATCH);
        done=out_valid && out_ready && (output_index==TILE*HEAD_DIM-1);
    end

    always_ff @(posedge clk or negedge rst_n) begin : controller
        integer feature_index, key_index;
        if(!rst_n) begin
            state<=S_WAIT;
            v_count<='0;
            key_tile<='0;
            feature_tile<=1'b0;
            row_index<='0;
            merge_row<='0;
            feed_count<='0;
            drain_count<='0;
            output_index<='0;
            alpha_seen<=1'b0;
            final_scale_seen<=1'b0;
            v_loaded<=1'b0;
            score_reg<='0;
            for(int r=0;r<TILE;r++) begin
                row_alpha[r]<='0;
                row_reciprocal[r]<='0;
                for(int k=0;k<TILE;k++) raw_exp[r][k]<='0;
                for(int d=0;d<HEAD_DIM;d++) output_acc[r][d]<='0;
            end
        end else begin
            if(sm_rf_wr_en && sm_rf_wr_addr==4'd1)
                for(int k=0;k<TILE;k++)
                    raw_exp[row_index][k]<=sm_rf_wr_data[k*16 +:16];
            if(sm_rf_wr_en && sm_rf_wr_addr==4'd2) begin
                if(state==S_WAIT_FINAL) begin
                    row_reciprocal[row_index]<=sm_rf_wr_data[15:0];
                    final_scale_seen<=1'b1;
                end else begin
                    row_alpha[row_index]<=sm_rf_wr_data[15:0];
                    alpha_seen<=1'b1;
                end
            end

            case(state)
                S_WAIT: if(v_load_start && !v_loaded) begin
                    v_count<='0;
                    state<=S_LOAD_V;
                end
                S_LOAD_V: if(v_load_valid && v_load_accept) begin
`ifndef SYNTHESIS
                    feature_index=v_count/TOKENS;
                    key_index=v_count%TOKENS;
                    v_mem[feature_index%TILE]
                         [(feature_index/TILE)*TOKENS+key_index]<=
                             fp16_to_int8_q(v_load_fp16,4);
`endif
                    if(v_count==V_COUNT-1) begin
                        v_loaded<=1'b1;
                        state<=S_WAIT_BATCH;
                    end else v_count<=v_count+1'b1;
                end
                S_WAIT_BATCH: begin
                    if(v_load_start) begin
                        v_loaded<=1'b0;
                        v_count<='0;
                        state<=S_LOAD_V;
                    end else if(batch_start && batch_ready) begin
                        key_tile<='0;
                        feature_tile<=1'b0;
                        row_index<='0;
                        alpha_seen<=1'b0;
                        final_scale_seen<=1'b0;
                        for(int r=0;r<TILE;r++)
                            for(int d=0;d<HEAD_DIM;d++) output_acc[r][d]<='0;
                        state<=S_WAIT_SCORE;
                    end
                end
                S_WAIT_SCORE: if(score_valid && score_ready) begin
                    score_reg<=scores_fp16;
                    state<=S_CMD_SOFTMAX;
                end
                S_CMD_SOFTMAX: if(sm_cmd_valid && sm_cmd_ready)
                    state<=S_WAIT_SOFTMAX;
                S_WAIT_SOFTMAX: if(sm_done && alpha_seen) begin
                    alpha_seen<=1'b0;
                    if(row_index==TILE-1) begin
                        feature_tile<=1'b0;
                        state<=S_PV_CLEAR;
                    end else begin
                        row_index<=row_index+1'b1;
                        state<=S_WAIT_SCORE;
                    end
                end
                S_PV_CLEAR: begin
                    feed_count<='0;
                    state<=S_PV_FEED;
                end
                S_PV_FEED: begin
                    if(feed_count==PV_CYCLES-1) begin
                        drain_count<='0;
                        state<=S_PV_DRAIN;
                    end else feed_count<=feed_count+1'b1;
                end
                S_PV_DRAIN: begin
                    if(drain_count==2) begin
                        merge_row<='0;
                        state<=S_PV_MERGE;
                    end else drain_count<=drain_count+1'b1;
                end
                S_PV_MERGE: begin
                    for(int n=0;n<TILE;n++)
                        output_acc[merge_row][feature_tile*TILE+n]
                            <=merged_tile_values[n];
                    if(merge_row==TILE-1) begin
                        if(!feature_tile) begin
                            feature_tile<=1'b1;
                            state<=S_PV_CLEAR;
                        end else if(key_tile<KEY_TILES-1) begin
                            key_tile<=key_tile+1'b1;
                            row_index<='0;
                            feature_tile<=1'b0;
                            state<=S_WAIT_SCORE;
                        end else begin
                            row_index<='0;
                            state<=S_CMD_FINAL;
                        end
                    end else merge_row<=merge_row+1'b1;
                end
                S_CMD_FINAL: if(sm_cmd_valid && sm_cmd_ready)
                    state<=S_WAIT_FINAL;
                S_WAIT_FINAL: if(sm_done && final_scale_seen) begin
                    final_scale_seen<=1'b0;
                    if(row_index==TILE-1) begin
                        output_index<='0;
                        state<=S_OUTPUT;
                    end else begin
                        row_index<=row_index+1'b1;
                        state<=S_CMD_FINAL;
                    end
                end
                S_OUTPUT: if(out_valid && out_ready) begin
                    if(output_index==TILE*HEAD_DIM-1) begin
                        output_index<='0;
                        state<=S_WAIT_BATCH;
                    end else output_index<=output_index+1'b1;
                end
                default: state<=S_WAIT;
            endcase
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            for(int n=0;n<TILE;n++) v_read_valid[n]<=1'b0;
        end else if(state==S_PV_CLEAR || state==S_PV_FEED) begin
            for(int n=0;n<TILE;n++)
                v_read_valid[n]<=v_read_en[n];
        end
    end

    always_comb begin
        for(int n=0;n<TILE;n++)
            v_read_data[n]=v_read_valid[n] ? v_read_raw[n] : 8'h00;
    end

    initial begin
        if(TOKENS%TILE!=0 || HEAD_DIM%TILE!=0 || TILE!=32)
            $error("Batched online PV requires 32-aligned shapes and TILE=32");
    end
endmodule

`default_nettype wire
