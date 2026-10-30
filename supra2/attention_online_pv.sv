`default_nettype none

// Single-query/head online attention core. K/V are loaded once per head;
// each score tile is normalized online and immediately consumed by PV GEMM.
module supra2_attention_online_pv #(
    parameter int TOKENS=256,
    parameter int HEAD_DIM=64,
    parameter int TILE=32,
    parameter int COUNT_W=15
) (
    input  wire logic          clk,
    input  wire logic          rst_n,
    input  wire logic          v_load_start,
    output logic               v_load_ready,
    input  wire logic          v_load_valid,
    output logic               v_load_accept,
    input  wire logic [15:0]   v_load_fp16,
    input  wire logic          query_start,
    output logic               query_ready,
    input  wire logic          score_valid,
    output logic               score_ready,
    input  wire logic [TILE*16-1:0] scores_fp16,
    output logic               out_valid,
    input  wire logic          out_ready,
    output logic [15:0]        out_fp16,
    output logic               busy,
    output logic               done
);
    import fp16_utils_pkg::*;

    localparam int V_COUNT=TOKENS*HEAD_DIM;
    localparam int KEY_TILES=TOKENS/TILE;
    localparam int TILE_CYCLES=TILE+32+32+3;
    typedef enum logic [3:0] {
        S_WAIT, S_LOAD_V, S_WAIT_QUERY, S_WAIT_SCORE, S_CMD_SOFTMAX,
        S_WAIT_SOFTMAX, S_PV_CLEAR, S_PV_FEED, S_PV_DRAIN,
        S_CMD_FINAL, S_WAIT_FINAL, S_OUTPUT
    } state_t;
    state_t state;

    // Bank by feature modulo TILE. Each bank stores the two 256-key feature
    // rows and serves exactly one systolic B lane per cycle (512 x 16 bits).
    logic [7:0] v_mem [0:TILE-1][0:(2*TOKENS)-1];
    logic [7:0] v_read_data [0:TILE-1];
    logic [9:0] v_read_addr [0:TILE-1];
    logic v_read_en [0:TILE-1];
    logic [7:0] v_prefetch_count;
    logic [15:0] raw_exp [0:TILE-1];
    logic [15:0] output_acc [0:HEAD_DIM-1];
    logic [511:0] score_reg;
    logic [15:0] alpha_fp16, reciprocal_fp16;
    logic alpha_seen, final_scale_seen, v_loaded;
    logic [COUNT_W-1:0] v_count;
    logic [7:0] key_tile;
    logic feature_tile;
    logic [6:0] feed_count, drain_count, output_index;

    logic sm_cmd_valid, sm_cmd_ready, sm_done, sm_busy;
    logic [3:0] sm_rf_rd_addr, sm_rf_wr_addr;
    logic [511:0] sm_rf_rd_data, sm_rf_wr_data;
    logic sm_rf_wr_en;

    logic array_clear, array_en, array_acc_valid;
    logic signed [7:0] a_col [32], b_row [32];
    logic signed [31:0] array_acc [32][32];

    softmax_engine #(.LANES(TILE), .ELEM_W(16)) u_softmax (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(sm_cmd_valid), .cmd_ready(sm_cmd_ready),
        .cmd_vd(4'd1), .cmd_vs1(4'd0), .cmd_alpha_vd(4'd2),
        .cmd_state_slot(7'd0), .cmd_valid_lanes(6'(TILE)),
        .cmd_online(1'b1), .cmd_online_reset(key_tile==0),
        .cmd_raw_output(state==S_CMD_SOFTMAX),
        .cmd_finalize(state==S_CMD_FINAL),
        .rf_rd_addr(sm_rf_rd_addr), .rf_rd_data(sm_rf_rd_data),
        .rf_wr_en(sm_rf_wr_en), .rf_wr_addr(sm_rf_wr_addr),
        .rf_wr_data(sm_rf_wr_data), .busy(sm_busy), .done(sm_done)
    );

    systolic_array #(.M(32), .N(32), .DATA_W(8), .ACC_W(32),
                     .UNSIGNED_A(1'b1), .PIPE_STAGE(32))
    u_pv_array (
        .clk(clk), .rst_n(rst_n), .clear_acc(array_clear), .en(array_en),
        .a_unsigned(1'b1),
        .active_rows(6'd1),
        .a_col(a_col), .b_row(b_row),
        .acc_out(array_acc), .acc_valid(array_acc_valid)
    );

    always_comb begin
        v_load_ready = (state==S_WAIT || state==S_WAIT_QUERY || state==S_LOAD_V);
        v_load_accept = v_load_ready;
        query_ready = (state==S_WAIT_QUERY) && v_loaded;
        score_ready = (state==S_WAIT_SCORE);
        sm_cmd_valid = (state==S_CMD_SOFTMAX || state==S_CMD_FINAL);
        sm_rf_rd_data = score_reg;
        array_clear = (state==S_PV_CLEAR);
        array_en = (state==S_PV_CLEAR || state==S_PV_FEED || state==S_PV_DRAIN);
        for (int m=0;m<32;m++) begin
            a_col[m]='0;
        end
        for (int n=0;n<32;n++) begin
            b_row[n]='0;
            b_row[n]=v_read_data[n];
            v_read_addr[n]='0;
            v_read_en[n]=1'b0;
        end
        v_prefetch_count=(state==S_PV_CLEAR)?8'd0:({1'b0,feed_count}+8'd1);
        if(state==S_PV_CLEAR || state==S_PV_FEED)
            for(int n=0;n<TILE;n++)
                if(v_prefetch_count>=n && (v_prefetch_count-n)<TILE) begin
                    v_read_en[n]=1'b1;
                    v_read_addr[n]=10'(feature_tile*TOKENS +
                        key_tile*TILE + (v_prefetch_count-n));
                end
        if (state==S_PV_FEED) begin
            if (feed_count<TILE)
                a_col[0]=fp16_to_uint8_q(raw_exp[feed_count],8);
        end
        out_valid=(state==S_OUTPUT);
        out_fp16=16'b0;
        if (state==S_OUTPUT)
            out_fp16=fp16_mul(output_acc[output_index],reciprocal_fp16);
        busy=(state!=S_WAIT && state!=S_WAIT_QUERY);
        done=out_valid && out_ready && (output_index==HEAD_DIM-1);
    end

    always_ff @(posedge clk or negedge rst_n) begin : controller
        logic [15:0] tile_value;
        integer feature_index, key_index;
        if (!rst_n) begin
            state<=S_WAIT;
            v_count<='0;
            key_tile<='0;
            feature_tile<=1'b0;
            feed_count<='0;
            drain_count<='0;
            output_index<='0;
            alpha_fp16<='0;
            reciprocal_fp16<='0;
            alpha_seen<=1'b0;
            final_scale_seen<=1'b0;
            v_loaded<=1'b0;
            score_reg<='0;
            for (int k=0;k<TILE;k++) raw_exp[k]<='0;
            for (int d=0;d<HEAD_DIM;d++) output_acc[d]<='0;
        end else begin
            if (sm_rf_wr_en && sm_rf_wr_addr==4'd1)
                for (int k=0;k<TILE;k++) raw_exp[k]<=sm_rf_wr_data[k*16 +: 16];
            if (sm_rf_wr_en && sm_rf_wr_addr==4'd2) begin
                if (state==S_WAIT_FINAL) begin
                    reciprocal_fp16<=sm_rf_wr_data[15:0];
                    final_scale_seen<=1'b1;
                end else begin
                    alpha_fp16<=sm_rf_wr_data[15:0];
                    alpha_seen<=1'b1;
                end
            end

            case (state)
                S_WAIT: begin
                    if (v_load_start && !v_loaded) begin
                        v_count<='0;
                        state<=S_LOAD_V;
                    end
                end
                S_LOAD_V: if (v_load_valid && v_load_accept) begin
                    feature_index=v_count/TOKENS;
                    key_index=v_count%TOKENS;
                    v_mem[feature_index%TILE]
                         [(feature_index/TILE)*TOKENS+key_index]<=
                             fp16_to_int8_q(v_load_fp16,4);
                    if (v_count==V_COUNT-1) begin
                        v_loaded<=1'b1;
                        state<=S_WAIT_QUERY;
                    end else v_count<=v_count+1'b1;
                end
                S_WAIT_QUERY: begin
                    if (v_load_start) begin
                        v_loaded<=1'b0;
                        v_count<='0;
                        state<=S_LOAD_V;
                    end else if (query_start && query_ready) begin
                        key_tile<='0;
                        feature_tile<=1'b0;
                        alpha_seen<=1'b0;
                        final_scale_seen<=1'b0;
                        for (int d=0;d<HEAD_DIM;d++) output_acc[d]<='0;
                        state<=S_WAIT_SCORE;
                    end
                end
                S_WAIT_SCORE: if (score_valid && score_ready) begin
                    score_reg<=scores_fp16;
                    state<=S_CMD_SOFTMAX;
                end
                S_CMD_SOFTMAX: if (sm_cmd_valid && sm_cmd_ready)
                    state<=S_WAIT_SOFTMAX;
                S_WAIT_SOFTMAX: begin
                    if (sm_done && alpha_seen) begin
                        alpha_seen<=1'b0;
                        feature_tile<=1'b0;
                        state<=S_PV_CLEAR;
                    end
                end
                S_PV_CLEAR: begin
                    feed_count<='0;
                    state<=S_PV_FEED;
                end
                S_PV_FEED: begin
                    if (feed_count==TILE_CYCLES-1) begin
                        drain_count<='0;
                        state<=S_PV_DRAIN;
                    end else feed_count<=feed_count+1'b1;
                end
                S_PV_DRAIN: begin
                    if (drain_count==2) begin
                        for (int n=0;n<TILE;n++) begin
                            tile_value=fp32_to_fp16(fp32_mul_rne(
                                fp32_from_int32_rne(array_acc[0][n]),32'h3980_0000));
                            output_acc[feature_tile*TILE+n]<=fp16_add(
                                fp16_mul(output_acc[feature_tile*TILE+n],alpha_fp16),
                                tile_value);
                        end
                        if (!feature_tile) begin
                            feature_tile<=1'b1;
                            state<=S_PV_CLEAR;
                        end else if (key_tile<KEY_TILES-1) begin
                            key_tile<=key_tile+1'b1;
                            feature_tile<=1'b0;
                            state<=S_WAIT_SCORE;
                        end else begin
                            state<=S_CMD_FINAL;
                        end
                    end else drain_count<=drain_count+1'b1;
                end
                S_CMD_FINAL: if (sm_cmd_valid && sm_cmd_ready)
                    state<=S_WAIT_FINAL;
                S_WAIT_FINAL: begin
                    if (sm_done && final_scale_seen) begin
                        final_scale_seen<=1'b0;
                        output_index<='0;
                        state<=S_OUTPUT;
                    end
                end
                S_OUTPUT: if (out_valid && out_ready) begin
                    if (output_index==HEAD_DIM-1) begin
                        output_index<='0;
                        state<=S_WAIT_QUERY;
                    end else output_index<=output_index+1'b1;
                end
                default: state<=S_WAIT;
            endcase
        end
    end

    // Synchronous bank reads are scheduled one cycle ahead of the systolic
    // feed. The clear cycle prefetches feed_count=0; every feed cycle fetches
    // the next diagonal B-lane value.
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            for(int n=0;n<TILE;n++) v_read_data[n]<=8'h00;
        end else if(state==S_PV_CLEAR || state==S_PV_FEED) begin
            for(int n=0;n<TILE;n++)
                v_read_data[n]<=v_read_en[n]
                    ? v_mem[n][v_read_addr[n]] : 8'h00;
        end
    end

    initial begin
        if (TOKENS%TILE!=0 || HEAD_DIM%TILE!=0 || TILE!=32)
            $error("Online PV wrapper currently requires 32-aligned shapes");
    end
endmodule

`default_nettype wire
