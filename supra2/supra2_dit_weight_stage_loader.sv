`default_nettype none

// Converts descriptor-driven 512-bit weight/scalar tiles into the existing
// 32-lane GEMM engine load-port protocol.
module supra2_dit_weight_stage_loader
    import supra2_dit_ddr_layout::*;
(
    input  wire logic         clk,
    input  wire logic         rst_n,
    input  wire logic         wload_req,
    input  wire logic [3:0]   wload_stage,
    input  wire logic [3:0]   block_index,
    output logic              wload_ack,

    output logic              stage_request,
    input  wire logic         stage_request_ready,
    output logic [2:0]        stage_request_index,
    output logic [1:0]        stage_component, // 0=weight, 1=scale, 2=bias
    output logic              tile_select,
    output logic [15:0]       n_tile_index,
    output logic [15:0]       k_tile_index,
    input  wire logic [511:0] stage_data,
    input  wire logic         stage_valid,
    output logic              stage_ready,
    input  wire logic         stage_last,
    input  wire logic         stage_dma_done,
    input  wire logic         dma_error,

    output logic              wt_wr_en,
    output logic [15:0]       wt_wr_beat,
    output logic [255:0]      wt_wr_data,
    output logic              sb_wr_en,
    output logic              sb_wr_sel,
    output logic [15:0]       sb_wr_addr,
    output logic [31:0]       sb_wr_data,
    output logic              loader_error
);
    typedef enum logic [3:0] {
        S_IDLE, S_WEIGHT_REQ, S_WEIGHT_STREAM, S_WEIGHT_EMIT,
        S_SCALE_REQ, S_SCALE_STREAM, S_SCALE_EMIT,
        S_BIAS_REQ, S_BIAS_STREAM, S_BIAS_EMIT, S_ACK, S_ERROR
    } state_t;
    state_t state;

    logic [3:0] block_index_r;
    logic [2:0] stage_index_r;
    logic [15:0] n_tile_r, k_tile_r;
    logic [5:0] stream_beat;
    logic [5:0] emit_index;
    logic [7:0] weight_tile [0:1023];
    logic [31:0] scalar_tile [0:31];
    logic [1:0] expected_component;
    logic [6:0] expected_beats;
    logic [6:0] n_tiles;
    logic [6:0] k_tiles;

    always_comb begin
        n_tiles = 7'(BLOCK_STAGE_N_TILES[block_index_r][stage_index_r]);
        k_tiles = 7'(BLOCK_STAGE_K_TILES[block_index_r][stage_index_r]);
        expected_component = 2'd0;
        expected_beats = 7'd16;
        case (state)
            S_SCALE_REQ, S_SCALE_STREAM: begin
                expected_component = 2'd1;
                expected_beats = 7'd2;
            end
            S_BIAS_REQ, S_BIAS_STREAM: begin
                expected_component = 2'd2;
                expected_beats = 7'd2;
            end
            default: begin
                expected_component = 2'd0;
                expected_beats = 7'd16;
            end
        endcase

        stage_request = (state == S_WEIGHT_REQ) ||
                        (state == S_SCALE_REQ) || (state == S_BIAS_REQ);
        stage_request_index = {1'b0, stage_index_r};
        stage_component = expected_component;
        tile_select = 1'b1;
        n_tile_index = n_tile_r;
        k_tile_index = (state == S_WEIGHT_REQ || state == S_WEIGHT_STREAM) ?
                       k_tile_r : 16'd0;
        stage_ready = (state == S_WEIGHT_STREAM) ||
                      (state == S_SCALE_STREAM) || (state == S_BIAS_STREAM);

        wload_ack = (state == S_ACK);
        wt_wr_en = (state == S_WEIGHT_EMIT);
        wt_wr_beat = 16'(((n_tile_r * 32 + emit_index) * k_tiles) + k_tile_r);
        wt_wr_data = '0;
        for (int lane = 0; lane < 32; lane++)
            wt_wr_data[lane*8 +: 8] = weight_tile[lane*32 + emit_index];

        sb_wr_en = (state == S_SCALE_EMIT) || (state == S_BIAS_EMIT);
        sb_wr_sel = (state == S_BIAS_EMIT);
        sb_wr_addr = 16'(n_tile_r * 32 + emit_index);
        sb_wr_data = scalar_tile[emit_index[4:0]];
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            block_index_r <= '0;
            stage_index_r <= '0;
            n_tile_r <= '0;
            k_tile_r <= '0;
            stream_beat <= '0;
            emit_index <= '0;
            loader_error <= 1'b0;
        end else begin
            if (dma_error) begin
                loader_error <= 1'b1;
                state <= S_ERROR;
            end else begin
                case (state)
                    S_IDLE: if (wload_req) begin
                        if ((block_index >= BLOCK_COUNT) ||
                            (wload_stage >= BLOCK_STAGE_COUNT)) begin
                            loader_error <= 1'b1;
                            state <= S_ERROR;
                        end else begin
                            block_index_r <= block_index;
                            stage_index_r <= wload_stage[2:0];
                            n_tile_r <= '0;
                            k_tile_r <= '0;
                            state <= S_WEIGHT_REQ;
                        end
                    end
                    S_WEIGHT_REQ: if (stage_request_ready) begin
                        stream_beat <= '0;
                        state <= S_WEIGHT_STREAM;
                    end
                    S_WEIGHT_STREAM: begin
                        if (stage_valid && stage_ready) begin
                            for (int byte_lane = 0; byte_lane < 64; byte_lane++)
                                weight_tile[int'(stream_beat)*64 + byte_lane] <=
                                    stage_data[byte_lane*8 +: 8];
                            if (stage_last != (stream_beat == expected_beats-1)) begin
                                loader_error <= 1'b1;
                                state <= S_ERROR;
                            end else stream_beat <= stream_beat + 1'b1;
                        end
                        if (stage_dma_done) begin
                            if (stream_beat != expected_beats) begin
                                loader_error <= 1'b1;
                                state <= S_ERROR;
                            end else begin
                                emit_index <= '0;
                                state <= S_WEIGHT_EMIT;
                            end
                        end
                    end
                    S_WEIGHT_EMIT: begin
                        if (emit_index == 31) begin
                            if (k_tile_r + 1 < k_tiles) begin
                                k_tile_r <= k_tile_r + 1'b1;
                                state <= S_WEIGHT_REQ;
                            end else if (n_tile_r + 1 < n_tiles) begin
                                n_tile_r <= n_tile_r + 1'b1;
                                k_tile_r <= '0;
                                state <= S_WEIGHT_REQ;
                            end else begin
                                n_tile_r <= '0;
                                state <= S_SCALE_REQ;
                            end
                        end else emit_index <= emit_index + 1'b1;
                    end
                    S_SCALE_REQ, S_BIAS_REQ: if (stage_request_ready) begin
                        stream_beat <= '0;
                        state <= (state == S_SCALE_REQ) ? S_SCALE_STREAM : S_BIAS_STREAM;
                    end
                    S_SCALE_STREAM, S_BIAS_STREAM: begin
                        if (stage_valid && stage_ready) begin
                            for (int word_lane = 0; word_lane < 16; word_lane++)
                                scalar_tile[int'(stream_beat)*16 + word_lane] <=
                                    stage_data[word_lane*32 +: 32];
                            if (stage_last != (stream_beat == expected_beats-1)) begin
                                loader_error <= 1'b1;
                                state <= S_ERROR;
                            end else stream_beat <= stream_beat + 1'b1;
                        end
                        if (stage_dma_done) begin
                            if (stream_beat != expected_beats) begin
                                loader_error <= 1'b1;
                                state <= S_ERROR;
                            end else begin
                                emit_index <= '0;
                                state <= (state == S_SCALE_STREAM) ?
                                         S_SCALE_EMIT : S_BIAS_EMIT;
                            end
                        end
                    end
                    S_SCALE_EMIT, S_BIAS_EMIT: begin
                        if (emit_index == 31) begin
                            if (n_tile_r + 1 < n_tiles) begin
                                n_tile_r <= n_tile_r + 1'b1;
                                state <= (state == S_SCALE_EMIT) ?
                                         S_SCALE_REQ : S_BIAS_REQ;
                            end else if (state == S_SCALE_EMIT) begin
                                n_tile_r <= '0;
                                state <= S_BIAS_REQ;
                            end else state <= S_ACK;
                        end else emit_index <= emit_index + 1'b1;
                    end
                    S_ACK: if (!wload_req) state <= S_IDLE;
                    S_ERROR: state <= S_ERROR;
                    default: begin
                        loader_error <= 1'b1;
                        state <= S_ERROR;
                    end
                endcase
            end
        end
    end
endmodule

`default_nettype wire
