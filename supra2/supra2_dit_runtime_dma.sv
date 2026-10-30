`default_nettype none

// Connects the 50-step runtime controller's generated descriptors to the
// generic 512-bit AXI read engine. A block engine requests weight, scale, or
// bias data either as a full component or as one tile/vector slice.
module supra2_dit_runtime_dma
    import supra2_dit_ddr_layout::*;
(
    input  wire logic         clk,
    input  wire logic         rst_n,
    input  wire logic         start,
    output wire logic         busy,
    output wire logic         done,

    output wire logic         context_start,
    input  wire logic         context_done,
    output wire logic         step_prep_start,
    input  wire logic         step_prep_done,
    output wire logic         block_prepare_start,
    input  wire logic         block_prepare_done,
    output wire logic         block_start,
    output wire logic [5:0]   step_index,
    output wire logic         branch_index,
    output wire logic [3:0]   block_index,
    input  wire logic         block_done,
    output wire logic         final_start,
    input  wire logic         final_done,
    input  wire logic         final_param_dma_enable,
    output wire logic         final_param_valid,
    output wire logic [1:0]   final_param_kind,
    output wire logic [15:0]  final_param_beat,
    output wire logic [511:0] final_param_data,
    output wire logic         final_param_done,
    input  wire logic         vector_load_req,
    output wire logic         vector_load_ready,
    input  wire logic [31:0]  vector_load_address,
    input  wire logic [31:0]  vector_load_beats,
    input  wire logic [3:0]   vector_load_target,
    input  wire logic [15:0]  vector_load_base_beat,
    output wire logic         vector_load_valid,
    output wire logic [3:0]   vector_load_target_out,
    output wire logic [15:0]  vector_load_beat,
    output wire logic [511:0] vector_load_data,
    output wire logic         vector_load_last,
    output wire logic         vector_load_done,
    input  wire logic         qlinear_load_req,
    output wire logic         qlinear_load_ready,
    input  wire logic [31:0]  qlinear_load_address,
    input  wire logic [31:0]  qlinear_load_beats,
    output wire logic         qlinear_load_valid,
    input  wire logic         qlinear_load_data_ready,
    output wire logic [511:0] qlinear_load_data,
    output wire logic         qlinear_load_last,
    output wire logic         qlinear_load_done,
    output wire logic         cfg_combine_start,
    input  wire logic         cfg_combine_done,
    output wire logic         euler_start,
    input  wire logic         euler_done,
    output wire logic [31:0]  step_timestep_fp32,
    output wire logic         step_timestep_valid,
    input  wire logic         step_timestep_ready,
    input  wire logic         adaln_table_enable,
    output wire logic [511:0] adaln_data,
    output wire logic         adaln_valid,
    input  wire logic         adaln_ready,
    output wire logic         adaln_last,
    output wire logic         adaln_done,

    input  wire logic         dicache_enable,
    input  wire logic [31:0]  dicache_threshold_q16,
    input  wire logic [31:0]  dicache_probe_error_q16,
    input  wire logic         dicache_cache_refresh_done,
    input  wire logic         dicache_cache_refresh_branch_index,
    input  wire logic         dicache_reuse_done,
    output wire logic         dicache_probe_start,
    output wire logic         dicache_reuse_residual,
    output wire logic         dicache_refresh_residual,

    input  wire logic         stage_request,
    output wire logic         stage_request_ready,
    input  wire logic [2:0]   stage_request_index,
    input  wire logic [1:0]   stage_component, // 0=weight, 1=scale, 2=bias
    input  wire logic         tile_select,
    input  wire logic [15:0]  n_tile_index,
    input  wire logic [15:0]  k_tile_index,
    output wire logic [511:0] stage_data,
    output wire logic         stage_valid,
    input  wire logic         stage_ready,
    output wire logic         stage_last,
    output wire logic         stage_dma_done,
    output wire logic         dma_error,

    output wire logic [3:0]   m_axi_arid,
    output wire logic [31:0]  m_axi_araddr,
    output wire logic [7:0]   m_axi_arlen,
    output wire logic [2:0]   m_axi_arsize,
    output wire logic [1:0]   m_axi_arburst,
    output wire logic         m_axi_arvalid,
    input  wire logic         m_axi_arready,
    input  wire logic [3:0]   m_axi_rid,
    input  wire logic [511:0] m_axi_rdata,
    input  wire logic [1:0]   m_axi_rresp,
    input  wire logic         m_axi_rlast,
    input  wire logic         m_axi_rvalid,
    output wire logic         m_axi_rready
);
    logic [6:0] stage_layer_id;
    logic [31:0] stage_weight_address, stage_scale_address, stage_bias_address;
    logic [31:0] stage_weight_bytes, stage_scale_bytes, stage_bias_bytes;
    logic [31:0] stage_weight_beats, stage_scale_beats, stage_bias_beats;
    logic [15:0] stage_output_features, stage_input_features;
    logic [15:0] stage_n_tiles, stage_k_tiles;
    logic [31:0] stage_n_tile_stride_bytes, stage_k_tile_stride_bytes;
    logic [31:0] block_adaln_weight_address, block_adaln_bias_address;
    logic block_is_skipped;

    logic block_active;
    logic reader_start, reader_launch_pending;
    logic [31:0] reader_descriptor_address, reader_descriptor_beats;
    logic reader_busy, reader_done, reader_error;
    logic reader_timestep_owner;
    logic reader_adaln_owner;
    logic reader_final_param_owner;
    logic reader_vector_owner;
    logic reader_qlinear_owner;
    logic [3:0] vector_target_r;
    logic [15:0] vector_base_beat_r, vector_beat_count;
    typedef enum logic [2:0] {
        FP_IDLE, FP_WEIGHT_REQ, FP_WEIGHT_WAIT, FP_BIAS_REQ, FP_BIAS_WAIT,
        FP_ADALN_REQ, FP_ADALN_WAIT
    } final_param_state_t;
    final_param_state_t final_param_state;
    logic [1:0] final_param_kind_r;
    logic [15:0] final_param_beat_r;
    logic final_param_done_r;
    logic final_static_params_loaded;
    logic [31:0] timestep_address;
    logic [9:0] step_block_base_q;
    logic [10:0] block_runtime_linear_index;
    logic [31:0] final_adaln_offset_q, block_adaln_offset_q;
    logic [3:0] timestep_lane;
    logic timestep_cache_valid, timestep_replay_valid;
    logic [5:0] timestep_cached_step;
    logic [31:0] timestep_cached_value;
    logic [31:0] request_address, request_beats;
    logic request_descriptor_valid;
    logic request_error;
    logic dicache_decision_valid;
    logic [511:0] reader_stream_data;
    logic reader_stream_valid, reader_stream_last, reader_stream_ready;

    assign stage_request_ready = block_active && !reader_busy &&
                                 !reader_launch_pending && !step_prep_start &&
                                 !vector_load_req && !qlinear_load_req;
    assign vector_load_ready = !reader_busy && !reader_launch_pending &&
        !step_prep_start &&
        !(block_prepare_start && adaln_table_enable) &&
        final_param_state == FP_IDLE && !stage_request && !qlinear_load_req;
    assign qlinear_load_ready = !reader_busy && !reader_launch_pending &&
        !step_prep_start &&
        !(block_prepare_start && adaln_table_enable) &&
        final_param_state == FP_IDLE && !stage_request && !vector_load_req;
    assign reader_start = !reader_busy && !reader_launch_pending &&
                          ((step_prep_start &&
                            !(timestep_cache_valid &&
                              timestep_cached_step == step_index)) ||
                           (block_prepare_start && adaln_table_enable) ||
                           (final_param_state == FP_WEIGHT_REQ) ||
                           (final_param_state == FP_BIAS_REQ) ||
                           (final_param_state == FP_ADALN_REQ) ||
                           (qlinear_load_req && qlinear_load_ready &&
                            request_descriptor_valid) ||
                           (vector_load_req && vector_load_ready &&
                            request_descriptor_valid) ||
                           (stage_request && stage_request_ready &&
                            request_descriptor_valid));
    assign step_timestep_valid = timestep_replay_valid ||
                                 (reader_stream_valid && reader_timestep_owner);
    assign step_timestep_fp32 = timestep_replay_valid ? timestep_cached_value :
        reader_stream_data[timestep_lane*32 +: 32];
    assign adaln_data = reader_stream_data;
    assign adaln_valid = reader_stream_valid && reader_adaln_owner;
    assign adaln_last = reader_stream_last && reader_adaln_owner;
    assign adaln_done = reader_done && reader_adaln_owner;
    assign final_param_valid = reader_stream_valid && reader_final_param_owner;
    assign final_param_kind = final_param_kind_r;
    assign final_param_beat = final_param_beat_r;
    assign final_param_data = reader_stream_data;
    assign final_param_done = final_param_done_r;
    assign vector_load_valid = reader_stream_valid && reader_vector_owner;
    assign vector_load_target_out = vector_target_r;
    assign vector_load_beat = vector_base_beat_r + vector_beat_count;
    assign vector_load_data = reader_stream_data;
    assign vector_load_last = reader_stream_last && reader_vector_owner;
    assign vector_load_done = reader_done && reader_vector_owner;
    assign qlinear_load_valid = reader_stream_valid && reader_qlinear_owner;
    assign qlinear_load_data = reader_stream_data;
    assign qlinear_load_last = reader_stream_last && reader_qlinear_owner;
    assign qlinear_load_done = reader_done && reader_qlinear_owner;
    assign reader_stream_ready = reader_timestep_owner ? step_timestep_ready :
        (reader_adaln_owner ? adaln_ready :
         (reader_qlinear_owner ? qlinear_load_data_ready :
          ((reader_final_param_owner || reader_vector_owner) ? 1'b1 : stage_ready)));
    assign stage_valid = reader_stream_valid && !reader_timestep_owner &&
                         !reader_adaln_owner && !reader_final_param_owner &&
                         !reader_vector_owner && !reader_qlinear_owner;
    assign stage_data = reader_stream_data;
    assign stage_last = reader_stream_last && !reader_timestep_owner &&
                        !reader_adaln_owner && !reader_final_param_owner &&
                        !reader_vector_owner && !reader_qlinear_owner;
    assign stage_dma_done = reader_done && !reader_timestep_owner &&
                            !reader_adaln_owner && !reader_final_param_owner &&
                            !reader_vector_owner && !reader_qlinear_owner;
    assign dma_error = reader_error || request_error;
    assign block_runtime_linear_index = {1'b0, step_block_base_q} +
                                        {7'd0, block_index};

    always_comb begin
        request_address = '0;
        request_beats = '0;
        request_descriptor_valid = 1'b1;
        if (final_param_state == FP_WEIGHT_REQ) begin
            request_address = DDR_BASE + PASSTHROUGH_OFFSET[33];
            request_beats = PASSTHROUGH_BYTES[33] / 64;
        end else if (final_param_state == FP_BIAS_REQ) begin
            request_address = DDR_BASE + PASSTHROUGH_OFFSET[34];
            // The 32-byte bias tensor occupies one aligned 64-byte AXI beat.
            request_beats = 32'd1;
        end else if (final_param_state == FP_ADALN_REQ) begin
            request_address = DDR_BASE + FINAL_ADALN_RUNTIME_OFFSET +
                              final_adaln_offset_q;
            request_beats = 32'd36;
        end else if (step_prep_start) begin
            request_address = {timestep_address[31:6], 6'b0};
            request_beats = 32'd1;
        end else if (block_prepare_start && adaln_table_enable) begin
            request_address = DDR_BASE + BLOCK_ADALN_RUNTIME_OFFSET +
                              block_adaln_offset_q;
            request_beats = 32'd144;
        end else if (qlinear_load_req && qlinear_load_ready) begin
            request_address = qlinear_load_address;
            request_beats = qlinear_load_beats;
            request_descriptor_valid = (qlinear_load_address[5:0] == 6'd0) &&
                                       (qlinear_load_beats != 32'd0);
        end else if (vector_load_req && vector_load_ready) begin
            request_address = vector_load_address;
            request_beats = vector_load_beats;
            request_descriptor_valid = (vector_load_address[5:0] == 6'd0) &&
                                       (vector_load_beats != 32'd0);
        end else case (stage_component)
            2'd0: begin
                if (tile_select) begin
                    if ((n_tile_index >= stage_n_tiles) ||
                        (k_tile_index >= stage_k_tiles)) begin
                        request_descriptor_valid = 1'b0;
                    end else begin
                        request_address = stage_weight_address +
                            n_tile_index * stage_n_tile_stride_bytes +
                            k_tile_index * stage_k_tile_stride_bytes;
                        request_beats = WEIGHT_TILE_BYTES / 64;
                    end
                end else begin
                    request_address = stage_weight_address;
                    request_beats = stage_weight_beats;
                end
            end
            2'd1, 2'd2: begin
                if (tile_select) begin
                    if (n_tile_index >= stage_n_tiles) begin
                        request_descriptor_valid = 1'b0;
                    end else if (stage_component == 2'd1) begin
                        request_address = stage_scale_address +
                                          n_tile_index * 32'd128;
                        request_beats = 32'd2;
                    end else begin
                        request_address = stage_bias_address +
                                          n_tile_index * 32'd128;
                        request_beats = 32'd2;
                    end
                end else if (stage_component == 2'd1) begin
                    request_address = stage_scale_address;
                    request_beats = stage_scale_beats;
                end else begin
                    request_address = stage_bias_address;
                    request_beats = stage_bias_beats;
                end
            end
            default: request_descriptor_valid = 1'b0;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            block_active <= 1'b0;
            request_error <= 1'b0;
            reader_timestep_owner <= 1'b0;
            reader_adaln_owner <= 1'b0;
            reader_final_param_owner <= 1'b0;
            reader_vector_owner <= 1'b0;
            reader_qlinear_owner <= 1'b0;
            vector_target_r <= '0;
            vector_base_beat_r <= '0;
            vector_beat_count <= '0;
            final_param_state <= FP_IDLE;
            final_param_kind_r <= 2'd0;
            final_param_beat_r <= '0;
            final_param_done_r <= 1'b0;
            final_static_params_loaded <= 1'b0;
            reader_launch_pending <= 1'b0;
            reader_descriptor_address <= '0;
            reader_descriptor_beats <= '0;
            step_block_base_q <= '0;
            final_adaln_offset_q <= '0;
            block_adaln_offset_q <= '0;
            timestep_lane <= '0;
            timestep_cache_valid <= 1'b0;
            timestep_replay_valid <= 1'b0;
            timestep_cached_step <= '0;
            timestep_cached_value <= '0;
        end else begin
            final_param_done_r <= 1'b0;
            reader_launch_pending <= reader_start;
            step_block_base_q <= step_index * 10'(BLOCK_COUNT);
            final_adaln_offset_q <= step_index * 32'd2304;
            block_adaln_offset_q <= block_runtime_linear_index * 32'd9216;
            if (start) request_error <= 1'b0;
            if (start) begin
                timestep_cache_valid <= 1'b0;
                timestep_replay_valid <= 1'b0;
                final_static_params_loaded <= 1'b0;
            end
            if (reader_start) begin
                reader_descriptor_address <= request_address;
                reader_descriptor_beats <= request_beats;
                reader_timestep_owner <= step_prep_start;
                reader_adaln_owner <= block_prepare_start && adaln_table_enable;
                reader_final_param_owner <= (final_param_state == FP_WEIGHT_REQ) ||
                    (final_param_state == FP_BIAS_REQ) ||
                    (final_param_state == FP_ADALN_REQ);
                reader_vector_owner <= vector_load_req && vector_load_ready &&
                    !(step_prep_start || (block_prepare_start && adaln_table_enable) ||
                      final_param_state != FP_IDLE);
                reader_qlinear_owner <= qlinear_load_req && qlinear_load_ready &&
                    !(step_prep_start || (block_prepare_start && adaln_table_enable) ||
                      final_param_state != FP_IDLE);
                if (vector_load_req && vector_load_ready &&
                    !(step_prep_start || (block_prepare_start && adaln_table_enable) ||
                      final_param_state != FP_IDLE)) begin
                    vector_target_r <= vector_load_target;
                    vector_base_beat_r <= vector_load_base_beat;
                    vector_beat_count <= '0;
                end
                if ((final_param_state == FP_WEIGHT_REQ) ||
                    (final_param_state == FP_BIAS_REQ) ||
                    (final_param_state == FP_ADALN_REQ))
                    final_param_beat_r <= '0;
                if (step_prep_start) timestep_lane <= timestep_address[5:2];
            end
            if (reader_done) reader_timestep_owner <= 1'b0;
            if (reader_done) reader_adaln_owner <= 1'b0;
            if (reader_done) reader_final_param_owner <= 1'b0;
            if (reader_done) reader_vector_owner <= 1'b0;
            if (reader_done) reader_qlinear_owner <= 1'b0;
            if (final_start && final_param_dma_enable && final_param_state == FP_IDLE) begin
                if (final_static_params_loaded) begin
                    final_param_kind_r <= 2'd2;
                    final_param_state <= FP_ADALN_REQ;
                end else begin
                    final_param_kind_r <= 2'd0;
                    final_param_state <= FP_WEIGHT_REQ;
                end
            end
            case (final_param_state)
                FP_WEIGHT_REQ: if (reader_start) final_param_state <= FP_WEIGHT_WAIT;
                FP_WEIGHT_WAIT: if (reader_done) begin
                    final_param_kind_r <= 2'd1;
                    final_param_state <= FP_BIAS_REQ;
                end
                FP_BIAS_REQ: if (reader_start) final_param_state <= FP_BIAS_WAIT;
                FP_BIAS_WAIT: if (reader_done) begin
                    final_static_params_loaded <= 1'b1;
                    final_param_kind_r <= 2'd2;
                    final_param_state <= FP_ADALN_REQ;
                end
                FP_ADALN_REQ: if (reader_start) final_param_state <= FP_ADALN_WAIT;
                FP_ADALN_WAIT: if (reader_done) begin
                    final_param_done_r <= 1'b1;
                    final_param_state <= FP_IDLE;
                end
                default: begin end
            endcase
            if (final_param_valid && reader_stream_ready)
                final_param_beat_r <= final_param_beat_r + 1'b1;
            if (vector_load_valid && reader_stream_ready)
                vector_beat_count <= vector_beat_count + 1'b1;
            if (step_prep_start && !reader_busy && timestep_cache_valid &&
                timestep_cached_step == step_index)
                timestep_replay_valid <= 1'b1;
            if (step_timestep_valid && step_timestep_ready) begin
                timestep_replay_valid <= 1'b0;
                if (reader_timestep_owner) begin
                    timestep_cached_value <= reader_stream_data[timestep_lane*32 +: 32];
                    timestep_cached_step <= step_index;
                    timestep_cache_valid <= 1'b1;
                end
            end
            if (block_prepare_start) block_active <= 1'b1;
            if (block_done) block_active <= 1'b0;
            if (stage_request && stage_request_ready && !request_descriptor_valid)
                request_error <= 1'b1;
            if (vector_load_req && vector_load_ready && !request_descriptor_valid)
                request_error <= 1'b1;
            if (qlinear_load_req && qlinear_load_ready && !request_descriptor_valid)
                request_error <= 1'b1;
        end
    end

    supra2_dit_runtime_controller u_controller (
        .clk(clk), .rst_n(rst_n), .start(start), .busy(busy), .done(done),
        .context_start(context_start), .context_done(context_done),
        .step_prep_start(step_prep_start), .step_prep_done(step_prep_done),
        .block_prepare_start(block_prepare_start),
        .block_prepare_done(block_prepare_done),
        .block_start(block_start), .block_done(block_done),
        .final_start(final_start), .final_done(final_done),
        .cfg_combine_start(cfg_combine_start), .cfg_combine_done(cfg_combine_done),
        .euler_start(euler_start), .euler_done(euler_done),
        .dicache_enable(dicache_enable),
        .dicache_decision_valid(dicache_decision_valid),
        .dicache_reuse(dicache_reuse_residual),
        .dicache_reuse_done(dicache_reuse_done),
        .dicache_probe_start(dicache_probe_start),
        .step_index(step_index), .branch_index(branch_index),
        .block_index(block_index),
        .dma_stage_index(stage_request_index), .block_is_skipped(block_is_skipped),
        .timestep_address(timestep_address), .stage_layer_id(stage_layer_id),
        .stage_weight_address(stage_weight_address),
        .stage_scale_address(stage_scale_address), .stage_bias_address(stage_bias_address),
        .stage_weight_bytes(stage_weight_bytes), .stage_scale_bytes(stage_scale_bytes),
        .stage_bias_bytes(stage_bias_bytes), .stage_weight_beats(stage_weight_beats),
        .stage_scale_beats(stage_scale_beats), .stage_bias_beats(stage_bias_beats),
        .stage_output_features(stage_output_features),
        .stage_input_features(stage_input_features), .stage_n_tiles(stage_n_tiles),
        .stage_k_tiles(stage_k_tiles),
        .stage_n_tile_stride_bytes(stage_n_tile_stride_bytes),
        .stage_k_tile_stride_bytes(stage_k_tile_stride_bytes),
        .block_adaln_weight_address(block_adaln_weight_address),
        .block_adaln_bias_address(block_adaln_bias_address)
    );

    supra2_dicache_online_policy u_dicache_policy (
        .clk(clk), .rst_n(rst_n), .run_start(start), .enable(dicache_enable),
        .branch_index(dicache_cache_refresh_done ?
                      dicache_cache_refresh_branch_index : branch_index),
        .probe_valid(dicache_probe_start),
        .probe_error_q16(dicache_probe_error_q16),
        .threshold_q16(dicache_threshold_q16),
        .cache_refresh_done(dicache_cache_refresh_done),
        .decision_valid(dicache_decision_valid),
        .reuse_residual(dicache_reuse_residual),
        .refresh_residual(dicache_refresh_residual),
        .accumulated_error_q16(), .cache_valid()
    );

    supra2_axi_read_dma #(.AXI_ID(4'h0)) u_read_dma (
        .clk(clk), .rst_n(rst_n), .start(reader_launch_pending),
        .descriptor_address(reader_descriptor_address),
        .descriptor_beats(reader_descriptor_beats),
        .busy(reader_busy), .done(reader_done), .dma_error(reader_error),
        .stream_data(reader_stream_data), .stream_valid(reader_stream_valid),
        .stream_ready(reader_stream_ready), .stream_last(reader_stream_last),
        .m_axi_arid(m_axi_arid), .m_axi_araddr(m_axi_araddr),
        .m_axi_arlen(m_axi_arlen), .m_axi_arsize(m_axi_arsize),
        .m_axi_arburst(m_axi_arburst), .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(m_axi_arready), .m_axi_rid(m_axi_rid),
        .m_axi_rdata(m_axi_rdata), .m_axi_rresp(m_axi_rresp),
        .m_axi_rlast(m_axi_rlast), .m_axi_rvalid(m_axi_rvalid),
        .m_axi_rready(m_axi_rready)
    );
endmodule

`default_nettype wire
