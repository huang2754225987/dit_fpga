`default_nettype none

// Connects the package-driven block/step controller, descriptor DMA, tile
// unpacker, existing Supra2 block compute engine, and CFG-to-Euler stream path.
// Context, activation, and final-head producers attach through explicit
// handshakes; the runtime AdaLN table is fetched internally by the DMA path.
module supra2_dit_compute_bridge (
    input  wire logic         clk,
    input  wire logic         rst_n,
    input  wire logic         start,
    output wire logic         busy,
    output wire logic         done,
    output wire logic         runtime_error,

    output wire logic         context_start,
    input  wire logic         context_done,
    output wire logic         step_prep_start,
    input  wire logic         step_prep_done,
    output wire logic [31:0]   step_timestep_fp32,
    output wire logic         step_timestep_valid,
    input  wire logic         step_timestep_ready,
    input  wire logic         adaln_table_enable,
    output wire logic         block_prepare_start,
    input  wire logic         block_prepare_done,
    output wire logic         block_start,
    output wire logic [5:0]   step_index,
    output wire logic         branch_index,
    output wire logic [3:0]   block_index,
    output wire logic [4:0]   compute_stage,
    output wire logic         final_start,
    input  wire logic         final_done,
    input  wire logic         final_head_enable,
    input  wire logic         final_param_dma_enable,
    input  wire logic         final_shift_wr_en,
    input  wire logic [4:0]   final_shift_wr_beat,
    input  wire logic [511:0] final_shift_wr_data,
    input  wire logic         final_scale_wr_en,
    input  wire logic [4:0]   final_scale_wr_beat,
    input  wire logic [511:0] final_scale_wr_data,
    input  wire logic         final_weight_wr_en,
    input  wire logic [15:0]  final_weight_wr_beat,
    input  wire logic [511:0] final_weight_wr_data,
    input  wire logic         final_bias_wr_en,
    input  wire logic [4:0]   final_bias_wr_addr,
    input  wire logic [15:0]  final_bias_wr_data,
    output wire logic         cfg_combine_start,

    // Aligned conditional/unconditional final-head velocity beats. The bridge
    // applies CFG=3, buffers all 4096 outputs, then feeds Euler from that buffer.
    input  wire logic         cfg_in_valid,
    output wire logic         cfg_in_ready,
    input  wire logic [511:0] cfg_velocity_uncond_fp16,
    input  wire logic [511:0] cfg_velocity_cond_fp16,
    input  wire logic [15:0]  cfg_guidance_fp16,
    output wire logic         euler_start,

    // The runtime controller now waits for the real 4096-element Euler
    // datapath. The final-head/CFG producer supplies aligned latent/velocity
    // beats; updated latent beats feed the next-step activation path.
    input  wire logic [15:0]  euler_step_size_fp16,
    input  wire logic         euler_latent_valid,
    output wire logic         euler_latent_ready,
    input  wire logic [511:0] euler_latent_fp16,
    output wire logic         euler_out_valid,
    input  wire logic         euler_out_ready,
    output wire logic [511:0] euler_updated_latent_fp16,

    // DiCache is runtime selectable. A shallow-probe reducer supplies the
    // relative-L1 estimate and trajectory coefficient; the full residual store
    // and DCTA stream are integrated below.
    input  wire logic         dicache_enable,
    input  wire logic [31:0]  dicache_threshold_q16,
    input  wire logic [31:0]  dicache_probe_error_q16,
    input  wire logic [15:0]  dicache_gamma_fp16,
    input  wire logic         dicache_residual_in_valid,
    output wire logic         dicache_residual_in_ready,
    input  wire logic [511:0] dicache_residual_in_data,
    input  wire logic         dicache_residual_in_last,
    output wire logic         dicache_residual_out_valid,
    input  wire logic         dicache_residual_out_ready,
    output wire logic [511:0] dicache_residual_out_data,
    output wire logic         dicache_residual_out_last,
    output wire logic         dicache_residual_cache_valid,
    output wire logic         dicache_residual_dcta_valid,
    output wire logic         dicache_cache_error,
    output wire logic         dicache_probe_start,
    output wire logic         dicache_reuse_residual,
    output wire logic         dicache_refresh_residual,

    // External context, timestep/AdaLN, and x-activation preparation port.
    input  wire logic         load_en,
    input  wire logic [3:0]   load_target,
    input  wire logic [20:0]  load_address,
    input  wire logic [15:0]  load_data,
    input  wire logic         load_vec_en,
    input  wire logic [3:0]   load_vec_target,
    input  wire logic [15:0]  load_vec_beat,
    input  wire logic [511:0] load_vec_data,
    input  wire logic         vector_load_req,
    output wire logic         vector_load_ready,
    input  wire logic [31:0]  vector_load_address,
    input  wire logic [31:0]  vector_load_beats,
    input  wire logic [3:0]   vector_load_target,
    input  wire logic [15:0]  vector_load_base_beat,
    output wire logic         vector_load_done,
    input  wire logic         qlinear_tile_req,
    output wire logic         qlinear_tile_ready,
    input  wire logic [6:0]   qlinear_layer_id,
    input  wire logic [1:0]   qlinear_component,
    input  wire logic [15:0]  qlinear_n_tile_index,
    input  wire logic [15:0]  qlinear_k_tile_index,
    output wire logic         qlinear_descriptor_valid,
    output wire logic [15:0]  qlinear_output_features,
    output wire logic [15:0]  qlinear_input_features,
    output wire logic [15:0]  qlinear_n_tiles,
    output wire logic [15:0]  qlinear_k_tiles,
    output wire logic         qlinear_data_valid,
    input  wire logic         qlinear_data_ready,
    output wire logic [511:0] qlinear_data,
    output wire logic         qlinear_data_last,
    output wire logic         qlinear_data_done,
    // Scalar debug read has one-cycle latency and is available while the
    // compute block is idle and the final-head vector reader is inactive.
    input  wire logic [14:0]  result_read_beat,
    output wire logic [127:0] result_read_data,

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
    logic [2:0] stage_request_index;
    logic [1:0] stage_component;
    logic tile_select;
    logic [15:0] n_tile_index, k_tile_index;
    logic [511:0] stage_data;
    logic stage_valid, stage_ready, stage_last, stage_dma_done;
    logic [511:0] adaln_dma_data;
    logic adaln_dma_valid, adaln_dma_ready, adaln_dma_last;
    logic [7:0] adaln_beat_index;
    logic block_prepare_done_seen, adaln_params_loaded;
    logic controller_block_prepare_done;
    logic stage_request, stage_request_ready;
    logic dma_error, loader_error;
    logic [15:0] wt_wr_beat, sb_wr_addr;
    logic [255:0] wt_wr_data;
    logic [31:0] sb_wr_data;
    logic wt_wr_en, sb_wr_en, sb_wr_sel;
    logic block_compute_busy, block_compute_done;
    logic stage_loader_ack;
    logic wload_req;
    logic [3:0] wload_stage;
    logic dicache_cache_refresh_done;
    logic dicache_cache_refresh_branch_index;
    logic dicache_cache_refresh_start;
    logic dicache_cache_reuse_done;
    logic dicache_cache_refresh_valid, dicache_cache_refresh_ready;
    logic [511:0] dicache_cache_refresh_data;
    logic [511:0] dicache_refresh_velocity_data;
    logic dicache_cache_refresh_last, dicache_cache_reuse_ready;
    logic [6:0] dicache_velocity_index;
    logic dicache_branch_reused, dicache_velocity_loaded;
    logic [1:0] dicache_refresh_pending;
    logic [511:0] latent_mem [0:1][0:127];
    logic current_latent_bank;
    logic [6:0] latent_in_index, latent_out_index;
    logic [6:0] dicache_refresh_index;
    logic dicache_refresh_branch;
    logic controller_euler_done;
    typedef enum logic [2:0] {
        DCR_IDLE, DCR_START, DCR_STREAM, DCR_WAIT_DONE, DCR_POLICY,
        DCR_FINISH
    } dicache_refresh_state_t;
    dicache_refresh_state_t dicache_refresh_state;
    logic cfg_combine_done;
    logic euler_update_done;
    logic final_head_done, final_head_busy, final_head_start;
    logic final_read_active, final_act_wr_en;
    logic final_read_valid_q;
    logic [12:0] final_read_beat_q;
    logic final_activation_loaded, final_params_loaded;
    logic runtime_final_param_valid, runtime_final_param_done;
    logic [1:0] runtime_final_param_kind;
    logic [15:0] runtime_final_param_beat;
    logic [511:0] runtime_final_param_data;
    logic runtime_vector_load_valid, runtime_vector_load_last;
    logic [3:0] runtime_vector_load_target;
    logic [15:0] runtime_vector_load_beat;
    logic [511:0] runtime_vector_load_data;
    logic [31:0] qlinear_descriptor_address, qlinear_descriptor_beats;
    logic qlinear_dma_req, qlinear_dma_ready;
    logic qlinear_descriptor_error;
    logic block_vec_load_en;
    logic [3:0] block_vec_load_target;
    logic [15:0] block_vec_load_beat;
    logic [511:0] block_vec_load_data;
    logic [511:0] final_bias_param_pack;
    logic final_bias_expand_active;
    logic [3:0] final_bias_expand_lane;
    logic final_shift_wr_en_i, final_scale_wr_en_i, final_weight_wr_en_i;
    logic [4:0] final_shift_wr_beat_i, final_scale_wr_beat_i;
    logic [15:0] final_weight_wr_beat_i;
    logic [511:0] final_shift_wr_data_i, final_scale_wr_data_i;
    logic [511:0] final_weight_wr_data_i;
    logic final_bias_wr_en_i;
    logic [4:0] final_bias_wr_addr_i;
    logic [15:0] final_bias_wr_data_i;
    logic [14:0] final_read_beat;
    logic [15:0] final_act_wr_beat;
    logic [511:0] final_act_wr_data;
    logic [14:0] block_result_read_beat;
    logic [127:0] block_result_read_data;
    logic [12:0] block_result_vec_read_beat;
    logic [511:0] block_result_vec_read_data;
    logic final_out_valid, final_out_ready, final_out_last;
    logic [511:0] final_out_data;
    logic [6:0] final_velocity_index;
    logic [511:0] uncond_velocity_mem [0:127];
    logic [511:0] cond_velocity_mem [0:127];
    logic [6:0] cfg_pair_index;
    logic cfg_pair_active;
    logic cfg_path_valid, cfg_path_ready;
    logic [511:0] cfg_path_uncond, cfg_path_cond;
    logic controller_final_done;

    function automatic logic [511:0] add_latent_to_residual(
        input logic [511:0] residual,
        input logic [511:0] latent
    );
        logic [511:0] result;
        begin
            for (int lane = 0; lane < 32; lane++)
                result[lane*16 +: 16] = fp16_utils_pkg::fp16_add(
                    residual[lane*16 +: 16], latent[lane*16 +: 16]);
            return result;
        end
    endfunction

    assign runtime_error = dma_error || loader_error || dicache_cache_error ||
                           qlinear_descriptor_error;
    assign qlinear_dma_req = qlinear_tile_req && qlinear_descriptor_valid;
    assign qlinear_tile_ready = qlinear_descriptor_valid && qlinear_dma_ready;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) qlinear_descriptor_error <= 1'b0;
        else if (start) qlinear_descriptor_error <= 1'b0;
        else if (qlinear_tile_req && qlinear_dma_ready &&
                 !qlinear_descriptor_valid)
            qlinear_descriptor_error <= 1'b1;
    end

    supra2_dit_qlinear_descriptor u_qlinear_descriptor (
        .layer_id(qlinear_layer_id), .component(qlinear_component),
        .n_tile_index(qlinear_n_tile_index), .k_tile_index(qlinear_k_tile_index),
        .descriptor_valid(qlinear_descriptor_valid),
        .descriptor_address(qlinear_descriptor_address),
        .descriptor_beats(qlinear_descriptor_beats),
        .output_features(qlinear_output_features),
        .input_features(qlinear_input_features), .n_tiles(qlinear_n_tiles),
        .k_tiles(qlinear_k_tiles)
    );
    assign controller_final_done = final_head_enable
        ? (dicache_branch_reused ? dicache_velocity_loaded : final_head_done)
        : final_done;
    assign adaln_dma_ready = adaln_table_enable;
    assign controller_block_prepare_done = adaln_table_enable
        ? ((block_prepare_done_seen || block_prepare_done) && adaln_params_loaded)
        : block_prepare_done;
    assign block_result_read_beat = (final_head_enable && final_read_active)
        ? final_read_beat : result_read_beat;
    // One physical X read port, with a one-cycle response. The final-head
    // reader owns it while active; otherwise serve the scalar debug reader.
    assign block_result_vec_read_beat = final_read_active
        ? final_read_beat[12:0] : result_read_beat[14:2];
    assign result_read_data = block_result_read_data;
    assign block_vec_load_en = runtime_vector_load_valid || load_vec_en;
    assign block_vec_load_target = runtime_vector_load_valid
        ? runtime_vector_load_target : load_vec_target;
    assign block_vec_load_beat = runtime_vector_load_valid
        ? runtime_vector_load_beat : load_vec_beat;
    assign block_vec_load_data = runtime_vector_load_valid
        ? runtime_vector_load_data : load_vec_data;
    assign cfg_in_ready = final_head_enable ? 1'b0 : cfg_path_ready;
    assign cfg_path_valid = final_head_enable ? cfg_pair_active : cfg_in_valid;
    assign cfg_path_uncond = final_head_enable
        ? uncond_velocity_mem[cfg_pair_index] : cfg_velocity_uncond_fp16;
    assign cfg_path_cond = final_head_enable
        ? cond_velocity_mem[cfg_pair_index] : cfg_velocity_cond_fp16;
    assign final_out_ready = final_head_enable;
    assign dicache_cache_refresh_start = final_head_enable
        ? (dicache_refresh_state == DCR_START)
        : dicache_refresh_residual;
    assign dicache_cache_refresh_valid = final_head_enable
        ? ((dicache_refresh_state == DCR_STREAM) &&
           (dicache_cache_refresh_ready))
        : dicache_residual_in_valid;
    assign dicache_refresh_velocity_data = dicache_refresh_branch
        ? cond_velocity_mem[dicache_refresh_index]
        : uncond_velocity_mem[dicache_refresh_index];
    always_comb begin
        dicache_cache_refresh_data = final_head_enable
            ? '0 : dicache_residual_in_data;
        dicache_cache_refresh_last = final_head_enable
            ? (dicache_refresh_index == 7'd127) : dicache_residual_in_last;
        if (final_head_enable) begin
            for (int lane = 0; lane < 32; lane++) begin
                dicache_cache_refresh_data[lane*16 +: 16] =
                    fp16_utils_pkg::fp16_sub(
                        dicache_refresh_velocity_data[lane*16 +: 16],
                        latent_mem[~current_latent_bank][dicache_refresh_index][lane*16 +: 16]);
            end
        end
    end
    assign dicache_cache_reuse_ready = final_head_enable
        ? 1'b1 : dicache_residual_out_ready;
    assign dicache_residual_in_ready = final_head_enable
        ? 1'b0
        : dicache_cache_refresh_ready;
    assign dicache_cache_refresh_branch_index = final_head_enable
        ? dicache_refresh_branch : branch_index;
    assign final_shift_wr_en_i = final_param_dma_enable
        ? (runtime_final_param_valid && runtime_final_param_kind == 2'd2 &&
           runtime_final_param_beat < 16'd18) : final_shift_wr_en;
    assign final_shift_wr_beat_i = final_param_dma_enable
        ? runtime_final_param_beat[4:0] : final_shift_wr_beat;
    assign final_shift_wr_data_i = final_param_dma_enable
        ? runtime_final_param_data : final_shift_wr_data;
    assign final_scale_wr_en_i = final_param_dma_enable
        ? (runtime_final_param_valid && runtime_final_param_kind == 2'd2 &&
           runtime_final_param_beat >= 16'd18) : final_scale_wr_en;
    assign final_scale_wr_beat_i = final_param_dma_enable
        ? runtime_final_param_beat[4:0] - 5'd18 : final_scale_wr_beat;
    assign final_scale_wr_data_i = final_param_dma_enable
        ? runtime_final_param_data : final_scale_wr_data;
    assign final_weight_wr_en_i = final_param_dma_enable
        ? (runtime_final_param_valid && runtime_final_param_kind == 2'd0)
        : final_weight_wr_en;
    assign final_weight_wr_beat_i = final_param_dma_enable
        ? runtime_final_param_beat : final_weight_wr_beat;
    assign final_weight_wr_data_i = final_param_dma_enable
        ? runtime_final_param_data : final_weight_wr_data;
    assign final_bias_wr_en_i = final_param_dma_enable
        ? final_bias_expand_active : final_bias_wr_en;
    assign final_bias_wr_addr_i = final_param_dma_enable
        ? {1'b0, final_bias_expand_lane} : final_bias_wr_addr;
    assign final_bias_wr_data_i = final_param_dma_enable
        ? final_bias_param_pack[final_bias_expand_lane*16 +: 16]
        : final_bias_wr_data;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            final_read_active <= 1'b0;
            final_read_valid_q <= 1'b0;
            final_read_beat_q <= '0;
            final_read_beat <= '0;
            final_act_wr_en <= 1'b0;
            final_act_wr_data <= '0;
            final_act_wr_beat <= '0;
            final_head_start <= 1'b0;
            final_activation_loaded <= 1'b0;
            final_params_loaded <= 1'b0;
            final_bias_param_pack <= '0;
            final_bias_expand_active <= 1'b0;
            final_bias_expand_lane <= '0;
            final_velocity_index <= '0;
            cfg_pair_index <= '0;
            cfg_pair_active <= 1'b0;
            dicache_velocity_index <= '0;
            dicache_branch_reused <= 1'b0;
            dicache_velocity_loaded <= 1'b0;
            dicache_refresh_pending <= '0;
            current_latent_bank <= 1'b0;
            latent_in_index <= '0;
            latent_out_index <= '0;
            dicache_refresh_index <= '0;
            dicache_refresh_branch <= 1'b0;
            dicache_refresh_state <= DCR_IDLE;
            controller_euler_done <= 1'b0;
            adaln_beat_index <= '0;
            block_prepare_done_seen <= 1'b0;
            adaln_params_loaded <= 1'b0;
        end else begin
            final_act_wr_en <= 1'b0;
            final_read_valid_q <= final_read_active;
            if (final_read_active) final_read_beat_q <= final_read_beat[12:0];
            final_head_start <= 1'b0;
            controller_euler_done <= 1'b0;
            if (step_prep_start) begin
                dicache_branch_reused <= 1'b0;
                dicache_velocity_loaded <= 1'b0;
            end
            if (final_head_enable && dicache_enable && dicache_refresh_residual)
                dicache_refresh_pending[branch_index] <= 1'b1;
            if (final_param_dma_enable && runtime_final_param_valid &&
                runtime_final_param_kind == 2'd1) begin
                final_bias_param_pack <= runtime_final_param_data;
                final_bias_expand_lane <= '0;
                final_bias_expand_active <= 1'b1;
            end else if (final_param_dma_enable && final_bias_expand_active) begin
                if (final_bias_expand_lane == 4'd15)
                    final_bias_expand_active <= 1'b0;
                else
                    final_bias_expand_lane <= final_bias_expand_lane + 1'b1;
            end
            if (final_param_dma_enable && runtime_final_param_done) begin
                final_params_loaded <= 1'b1;
                if (final_activation_loaded && !final_read_active)
                    final_head_start <= 1'b1;
            end
            if (dicache_reuse_residual) begin
                dicache_branch_reused <= 1'b1;
                dicache_velocity_loaded <= 1'b0;
                dicache_velocity_index <= '0;
            end
            if (final_head_enable && dicache_residual_out_valid &&
                dicache_cache_reuse_ready) begin
                if (!branch_index)
                    uncond_velocity_mem[dicache_velocity_index] <=
                        add_latent_to_residual(dicache_residual_out_data,
                            latent_mem[current_latent_bank][dicache_velocity_index]);
                else
                    cond_velocity_mem[dicache_velocity_index] <=
                        add_latent_to_residual(dicache_residual_out_data,
                            latent_mem[current_latent_bank][dicache_velocity_index]);
                if (dicache_residual_out_last)
                    dicache_velocity_loaded <= 1'b1;
                else
                    dicache_velocity_index <= dicache_velocity_index + 1'b1;
            end
            if (block_prepare_start) begin
                adaln_beat_index <= '0;
                block_prepare_done_seen <= 1'b0;
                adaln_params_loaded <= 1'b0;
            end
            if (adaln_table_enable && block_prepare_done)
                block_prepare_done_seen <= 1'b1;
            if (adaln_table_enable && adaln_dma_valid && adaln_dma_ready) begin
                if (adaln_dma_last && adaln_beat_index == 8'd143)
                    adaln_params_loaded <= 1'b1;
                else adaln_beat_index <= adaln_beat_index + 1'b1;
            end
            if (final_start && final_head_enable && !dicache_branch_reused) begin
                final_read_active <= 1'b1;
                final_activation_loaded <= 1'b0;
                final_params_loaded <= !final_param_dma_enable;
                final_read_beat <= '0;
                final_velocity_index <= '0;
            end else if (final_read_active) begin
                if (final_read_beat == 15'd4607)
                    final_read_active <= 1'b0;
                else final_read_beat <= final_read_beat + 1'b1;
            end
            if (final_read_valid_q) begin
                final_act_wr_en <= 1'b1;
                final_act_wr_beat <= 16'(final_read_beat_q);
                final_act_wr_data <= block_result_vec_read_data;
                if (final_read_beat_q == 13'd4607) begin
                    final_activation_loaded <= 1'b1;
                    if (final_params_loaded || !final_param_dma_enable)
                        final_head_start <= 1'b1;
                end
            end
            if (final_head_enable && final_out_valid && final_out_ready) begin
                if (!branch_index) uncond_velocity_mem[final_velocity_index] <= final_out_data;
                else cond_velocity_mem[final_velocity_index] <= final_out_data;
                final_velocity_index <= final_velocity_index + 1'b1;
            end
            if (final_head_enable && cfg_combine_start) begin
                cfg_pair_index <= '0;
                cfg_pair_active <= 1'b1;
            end else if (final_head_enable && cfg_pair_active &&
                         cfg_path_valid && cfg_path_ready) begin
                if (cfg_pair_index == 7'd127) cfg_pair_active <= 1'b0;
                else cfg_pair_index <= cfg_pair_index + 1'b1;
            end

            if (euler_start) begin
                latent_in_index <= '0;
                latent_out_index <= '0;
            end
            if (euler_latent_valid && euler_latent_ready) begin
                latent_mem[current_latent_bank][latent_in_index] <= euler_latent_fp16;
                if (latent_in_index != 7'd127)
                    latent_in_index <= latent_in_index + 1'b1;
            end
            if (euler_out_valid && euler_out_ready) begin
                latent_mem[~current_latent_bank][latent_out_index] <=
                    euler_updated_latent_fp16;
                if (latent_out_index != 7'd127)
                    latent_out_index <= latent_out_index + 1'b1;
            end
            if (euler_update_done) current_latent_bank <= ~current_latent_bank;

            case (dicache_refresh_state)
                DCR_IDLE: begin
                    if (euler_update_done) begin
                        if (final_head_enable && dicache_enable &&
                            (|dicache_refresh_pending)) begin
                            dicache_refresh_branch <= dicache_refresh_pending[0]
                                ? 1'b0 : 1'b1;
                            dicache_refresh_index <= '0;
                            dicache_refresh_state <= DCR_START;
                        end else begin
                            controller_euler_done <= 1'b1;
                        end
                    end
                end
                DCR_START: dicache_refresh_state <= DCR_STREAM;
                DCR_STREAM: begin
                    if (dicache_cache_refresh_valid && dicache_cache_refresh_ready) begin
                        if (dicache_refresh_index == 7'd127)
                            dicache_refresh_state <= DCR_WAIT_DONE;
                        else
                            dicache_refresh_index <= dicache_refresh_index + 1'b1;
                    end
                end
                DCR_WAIT_DONE: begin
                    if (dicache_cache_refresh_done) begin
                        dicache_refresh_pending[dicache_refresh_branch] <= 1'b0;
                        dicache_refresh_state <= DCR_POLICY;
                    end
                end
                DCR_POLICY: begin
                    if (|dicache_refresh_pending) begin
                        dicache_refresh_branch <= dicache_refresh_pending[0]
                            ? 1'b0 : 1'b1;
                        dicache_refresh_index <= '0;
                        dicache_refresh_state <= DCR_START;
                    end else begin
                        dicache_refresh_state <= DCR_FINISH;
                    end
                end
                DCR_FINISH: begin
                    controller_euler_done <= 1'b1;
                    dicache_refresh_state <= DCR_IDLE;
                end
                default: dicache_refresh_state <= DCR_IDLE;
            endcase
        end
    end

    supra2_cfg_euler_path #(.LANES(32), .ELEMENTS(4096), .COUNT_W(13))
    u_cfg_euler_path (
        .clk(clk), .rst_n(rst_n), .run_start(start),
        .cfg_start(cfg_combine_start), .cfg_guidance_fp16(cfg_guidance_fp16),
        .cfg_in_valid(cfg_path_valid), .cfg_in_ready(cfg_path_ready),
        .velocity_uncond_fp16(cfg_path_uncond),
        .velocity_cond_fp16(cfg_path_cond), .cfg_done(cfg_combine_done),
        .euler_start(euler_start), .euler_step_size_fp16(euler_step_size_fp16),
        .latent_valid(euler_latent_valid), .latent_ready(euler_latent_ready),
        .latent_fp16(euler_latent_fp16), .updated_valid(euler_out_valid),
        .updated_ready(euler_out_ready),
        .updated_latent_fp16(euler_updated_latent_fp16),
        .euler_done(euler_update_done)
    );

    supra2_dicache_residual_cache #(.ELEMENTS(4096), .LANES(32)) u_dicache_cache (
        .clk(clk), .rst_n(rst_n), .run_start(start),
        .branch_index((final_head_enable &&
                       (dicache_refresh_state != DCR_IDLE))
                          ? dicache_refresh_branch : branch_index),
        .refresh_start(dicache_cache_refresh_start),
        .refresh_valid(dicache_cache_refresh_valid),
        .refresh_ready(dicache_cache_refresh_ready),
        .refresh_residual_fp16(dicache_cache_refresh_data),
        .refresh_last(dicache_cache_refresh_last),
        .refresh_done(dicache_cache_refresh_done),
        .reuse_start(dicache_reuse_residual), .gamma_fp16(dicache_gamma_fp16),
        .reuse_valid(dicache_residual_out_valid),
        .reuse_ready(dicache_cache_reuse_ready),
        .reuse_residual_fp16(dicache_residual_out_data),
        .reuse_last(dicache_residual_out_last), .reuse_done(dicache_cache_reuse_done),
        .cache_valid(dicache_residual_cache_valid),
        .dcta_valid(dicache_residual_dcta_valid),
        .cache_error(dicache_cache_error)
    );

    supra2_dit_runtime_dma u_runtime_dma (
        .clk(clk), .rst_n(rst_n), .start(start), .busy(busy), .done(done),
        .context_start(context_start), .context_done(context_done),
        .step_prep_start(step_prep_start), .step_prep_done(step_prep_done),
        .step_timestep_fp32(step_timestep_fp32),
        .step_timestep_valid(step_timestep_valid),
        .step_timestep_ready(step_timestep_ready),
        .adaln_table_enable(adaln_table_enable),
        .adaln_data(adaln_dma_data), .adaln_valid(adaln_dma_valid),
        .adaln_ready(adaln_dma_ready), .adaln_last(adaln_dma_last),
        .adaln_done(),
        .block_prepare_start(block_prepare_start),
        .block_prepare_done(controller_block_prepare_done),
        .block_start(block_start), .step_index(step_index),
        .branch_index(branch_index), .block_index(block_index),
        .block_done(block_compute_done),
        .final_start(final_start), .final_done(controller_final_done),
        .final_param_dma_enable(final_param_dma_enable &&
                                !dicache_branch_reused),
        .final_param_valid(runtime_final_param_valid),
        .final_param_kind(runtime_final_param_kind),
        .final_param_beat(runtime_final_param_beat),
        .final_param_data(runtime_final_param_data),
        .final_param_done(runtime_final_param_done),
        .vector_load_req(vector_load_req), .vector_load_ready(vector_load_ready),
        .vector_load_address(vector_load_address), .vector_load_beats(vector_load_beats),
        .vector_load_target(vector_load_target),
        .vector_load_base_beat(vector_load_base_beat),
        .vector_load_valid(runtime_vector_load_valid),
        .vector_load_target_out(runtime_vector_load_target),
        .vector_load_beat(runtime_vector_load_beat),
        .vector_load_data(runtime_vector_load_data),
        .vector_load_last(runtime_vector_load_last),
        .vector_load_done(vector_load_done),
        .qlinear_load_req(qlinear_dma_req),
        .qlinear_load_ready(qlinear_dma_ready),
        .qlinear_load_address(qlinear_descriptor_address),
        .qlinear_load_beats(qlinear_descriptor_beats),
        .qlinear_load_valid(qlinear_data_valid),
        .qlinear_load_data_ready(qlinear_data_ready),
        .qlinear_load_data(qlinear_data), .qlinear_load_last(qlinear_data_last),
        .qlinear_load_done(qlinear_data_done),
        .cfg_combine_start(cfg_combine_start), .cfg_combine_done(cfg_combine_done),
        .euler_start(euler_start), .euler_done(controller_euler_done),
        .dicache_enable(dicache_enable),
        .dicache_threshold_q16(dicache_threshold_q16),
        .dicache_probe_error_q16(dicache_probe_error_q16),
        .dicache_cache_refresh_done(dicache_cache_refresh_done),
        .dicache_cache_refresh_branch_index(dicache_cache_refresh_branch_index),
        .dicache_reuse_done(dicache_cache_reuse_done),
        .dicache_probe_start(dicache_probe_start),
        .dicache_reuse_residual(dicache_reuse_residual),
        .dicache_refresh_residual(dicache_refresh_residual),
        .stage_request(stage_request), .stage_request_ready(stage_request_ready),
        .stage_request_index(stage_request_index), .stage_component(stage_component),
        .tile_select(tile_select), .n_tile_index(n_tile_index),
        .k_tile_index(k_tile_index), .stage_data(stage_data),
        .stage_valid(stage_valid), .stage_ready(stage_ready),
        .stage_last(stage_last), .stage_dma_done(stage_dma_done),
        .dma_error(dma_error), .m_axi_arid(m_axi_arid),
        .m_axi_araddr(m_axi_araddr), .m_axi_arlen(m_axi_arlen),
        .m_axi_arsize(m_axi_arsize), .m_axi_arburst(m_axi_arburst),
        .m_axi_arvalid(m_axi_arvalid), .m_axi_arready(m_axi_arready),
        .m_axi_rid(m_axi_rid), .m_axi_rdata(m_axi_rdata),
        .m_axi_rresp(m_axi_rresp), .m_axi_rlast(m_axi_rlast),
        .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(m_axi_rready)
    );

    supra2_dit_weight_stage_loader u_stage_loader (
        .clk(clk), .rst_n(rst_n), .wload_req(wload_req),
        .wload_stage(wload_stage), .block_index(block_index),
        .wload_ack(stage_loader_ack),
        .stage_request(stage_request), .stage_request_ready(stage_request_ready),
        .stage_request_index(stage_request_index), .stage_component(stage_component),
        .tile_select(tile_select), .n_tile_index(n_tile_index),
        .k_tile_index(k_tile_index), .stage_data(stage_data),
        .stage_valid(stage_valid), .stage_ready(stage_ready),
        .stage_last(stage_last), .stage_dma_done(stage_dma_done),
        .dma_error(dma_error), .wt_wr_en(wt_wr_en),
        .wt_wr_beat(wt_wr_beat), .wt_wr_data(wt_wr_data),
        .sb_wr_en(sb_wr_en), .sb_wr_sel(sb_wr_sel),
        .sb_wr_addr(sb_wr_addr), .sb_wr_data(sb_wr_data),
        .loader_error(loader_error)
    );

    supra2_block_top #(.RESULT_VEC_PORT(1'b1)) u_block_compute (
        .clk(clk), .rst_n(rst_n), .start(block_start),
        .start_stage(5'd0), .start_head(4'd0), .start_ni(16'd0),
        .stop_ni(16'd0), .busy(block_compute_busy), .done(block_compute_done),
        .ld_en(load_en), .ld_target(load_target), .ld_addr(load_address),
        .ld_data(load_data), .ld_vec_en(block_vec_load_en),
        .ld_vec_target(block_vec_load_target), .ld_vec_beat(block_vec_load_beat),
        .ld_vec_data(block_vec_load_data),
        .wt_wr_en(wt_wr_en), .wt_wr_beat(wt_wr_beat),
        .wt_wr_data(wt_wr_data), .sb_wr_en(sb_wr_en), .sb_wr_sel(sb_wr_sel),
        .sb_wr_addr(sb_wr_addr), .sb_wr_data(sb_wr_data),
        .param_vec_wr_en(adaln_dma_valid && adaln_dma_ready),
        .param_vec_wr_beat(adaln_beat_index), .param_vec_wr_data(adaln_dma_data),
        .result_rd_beat(block_result_read_beat), .result_rd_data(block_result_read_data),
        .result_vec_rd_beat(block_result_vec_read_beat),
        .result_vec_rd_data(block_result_vec_read_data),
        .wload_req(wload_req), .wload_stage(wload_stage),
        .wload_ack(stage_loader_ack), .stage_id(compute_stage)
    );

    supra2_final_head_datapath u_final_head (
        .clk(clk), .rst_n(rst_n),
        .act_wr_en(final_act_wr_en), .act_wr_beat(final_act_wr_beat),
        .act_wr_data(final_act_wr_data),
        .shift_wr_en(final_shift_wr_en_i), .shift_wr_beat(final_shift_wr_beat_i),
        .shift_wr_data(final_shift_wr_data_i),
        .scale_wr_en(final_scale_wr_en_i), .scale_wr_beat(final_scale_wr_beat_i),
        .scale_wr_data(final_scale_wr_data_i),
        .weight_wr_en(final_weight_wr_en_i), .weight_wr_beat(final_weight_wr_beat_i),
        .weight_wr_data(final_weight_wr_data_i),
        .bias_wr_en(final_bias_wr_en_i), .bias_wr_addr(final_bias_wr_addr_i),
        .bias_wr_data(final_bias_wr_data_i), .start(final_head_start),
        .busy(final_head_busy), .done(final_head_done),
        .out_valid(final_out_valid), .out_ready(final_out_ready),
        .out_last(final_out_last), .out_data(final_out_data)
    );
endmodule

`default_nettype wire
