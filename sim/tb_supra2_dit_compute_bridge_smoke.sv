`timescale 1ns/1ps
`default_nettype none

module tb_supra2_dit_compute_bridge_smoke;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    always #5 clk = ~clk;

    supra2_dit_compute_bridge dut (
        .clk(clk), .rst_n(rst_n), .start(1'b0), .busy(), .done(),
        .runtime_error(), .context_start(), .context_done(1'b0),
        .step_prep_start(), .step_prep_done(1'b0),
        .step_timestep_fp32(), .step_timestep_valid(), .step_timestep_ready(1'b1),
        .adaln_table_enable(1'b0),
        .block_prepare_start(), .block_prepare_done(1'b0), .block_start(),
        .step_index(), .branch_index(), .block_index(), .compute_stage(),
        .final_start(), .final_done(1'b0), .final_head_enable(1'b0),
        .final_param_dma_enable(1'b0),
        .final_shift_wr_en(1'b0), .final_shift_wr_beat(5'd0), .final_shift_wr_data(512'd0),
        .final_scale_wr_en(1'b0), .final_scale_wr_beat(5'd0), .final_scale_wr_data(512'd0),
        .final_weight_wr_en(1'b0), .final_weight_wr_beat(16'd0), .final_weight_wr_data(512'd0),
        .final_bias_wr_en(1'b0), .final_bias_wr_addr(5'd0), .final_bias_wr_data(16'd0),
        .cfg_combine_start(),
        .cfg_in_valid(1'b0), .cfg_in_ready(),
        .cfg_velocity_uncond_fp16(512'd0), .cfg_velocity_cond_fp16(512'd0),
        .cfg_guidance_fp16(16'h4200), .euler_start(),
        .euler_step_size_fp16(16'h0000),
        .euler_latent_valid(1'b0), .euler_latent_ready(),
        .euler_latent_fp16(512'd0), .euler_out_valid(),
        .euler_out_ready(1'b0), .euler_updated_latent_fp16(),
        .dicache_enable(1'b0), .dicache_threshold_q16(32'd6554),
        .dicache_probe_error_q16(32'd0), .dicache_gamma_fp16(16'h0000),
        .dicache_residual_in_valid(1'b0), .dicache_residual_in_ready(),
        .dicache_residual_in_data(512'd0), .dicache_residual_in_last(1'b0),
        .dicache_residual_out_valid(), .dicache_residual_out_ready(1'b1),
        .dicache_residual_out_data(), .dicache_residual_out_last(),
        .dicache_residual_cache_valid(), .dicache_residual_dcta_valid(),
        .dicache_cache_error(),
        .dicache_probe_start(), .dicache_reuse_residual(),
        .dicache_refresh_residual(),
        .load_en(1'b0), .load_target(4'd0), .load_address(21'd0),
        .load_data(16'd0), .load_vec_en(1'b0), .load_vec_target(4'd0),
        .load_vec_beat(16'd0), .load_vec_data(512'd0),
        .vector_load_req(1'b0), .vector_load_ready(),
        .vector_load_address(32'd0), .vector_load_beats(32'd0),
        .vector_load_target(4'd0), .vector_load_base_beat(16'd0),
        .vector_load_done(), .qlinear_tile_req(1'b0), .qlinear_tile_ready(),
        .qlinear_layer_id(7'd0), .qlinear_component(2'd0),
        .qlinear_n_tile_index(16'd0), .qlinear_k_tile_index(16'd0),
        .qlinear_descriptor_valid(), .qlinear_output_features(),
        .qlinear_input_features(), .qlinear_n_tiles(), .qlinear_k_tiles(),
        .qlinear_data_valid(), .qlinear_data_ready(1'b1), .qlinear_data(),
        .qlinear_data_last(), .qlinear_data_done(),
        .result_read_beat(15'd0), .result_read_data(),
        .m_axi_arid(), .m_axi_araddr(), .m_axi_arlen(), .m_axi_arsize(),
        .m_axi_arburst(), .m_axi_arvalid(), .m_axi_arready(1'b0),
        .m_axi_rid(4'd0), .m_axi_rdata(512'd0), .m_axi_rresp(2'd0),
        .m_axi_rlast(1'b0), .m_axi_rvalid(1'b0), .m_axi_rready()
    );

    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk) rst_n = 1'b1;
        repeat (3) @(posedge clk);
        $display("SUPRA2_DIT_COMPUTE_BRIDGE_ELAB_PASS");
        $finish;
    end
endmodule

`default_nettype wire
