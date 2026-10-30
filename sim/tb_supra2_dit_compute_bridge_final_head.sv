`timescale 1ns/1ps
`default_nettype none
module tb_supra2_dit_compute_bridge_final_head;
    import fp16_utils_pkg::*;
    logic clk=0, rst_n=0;
    logic final_head_enable=1;
    logic dicache_enable=1;
    logic final_shift_wr_en=0, final_scale_wr_en=0;
    logic [4:0] final_shift_wr_beat=0, final_scale_wr_beat=0;
    logic [511:0] final_shift_wr_data='0, final_scale_wr_data='0;
    logic final_weight_wr_en=0;
    logic [15:0] final_weight_wr_beat=0;
    logic [511:0] final_weight_wr_data='0;
    logic final_bias_wr_en=0;
    logic [4:0] final_bias_wr_addr=0;
    logic [15:0] final_bias_wr_data=0;
    logic load_en=0;
    logic [3:0] load_target=0;
    logic [20:0] load_address=0;
    logic [15:0] load_data=0;
    logic [15:0] act_mem[0:256*576-1];
    logic [15:0] shift_mem[0:575], scale_mem[0:575];
    logic [15:0] weight_mem[0:16*576-1], bias_mem[0:15];
    logic [15:0] expected[0:4095];
    logic [511:0] saved_uncond[0:127];
    logic [14:0] unused_result_beat=0;
    integer output_count=0;
    integer final_activation_write_count=0;
    integer reuse_beat_count=0;
    integer controller_euler_done_count=0;
    real peak_value=0.0, max_error=0.0, reuse_max_error=0.0;
    real got_value, exp_value, error_value;
    string root="build/supra2_final_head/";
    always #5 clk=~clk;
    always @(posedge clk)
        if (rst_n && dut.final_act_wr_en) final_activation_write_count++;

    function automatic real half_real(input logic [15:0] h);
        integer e,m; real v;
        begin
            e=h[14:10]; m=h[9:0];
            if(e==0) v=m/1024.0*(2.0**-14);
            else if(e==31) v=0.0;
            else v=(1.0+m/1024.0)*(2.0**(e-15));
            return h[15] ? -v : v;
        end
    endfunction

    function automatic logic [15:0] cfg_reference_lane(
        input logic [15:0] uncond_value,
        input logic [15:0] cond_value
    );
        logic [31:0] delta, scaled_delta;
        begin
            delta=fp32_add_rne(fp16_to_fp32(cond_value),
                fp16_to_fp32(uncond_value)^32'h8000_0000);
            scaled_delta=fp32_mul_rne(delta,fp16_to_fp32(16'h4200));
            return fp32_to_fp16(fp32_add_rne(
                fp16_to_fp32(uncond_value),scaled_delta));
        end
    endfunction

    supra2_dit_compute_bridge dut (
        .clk(clk), .rst_n(rst_n), .start(1'b0), .busy(), .done(),
        .runtime_error(), .context_start(), .context_done(1'b0),
        .step_prep_start(), .step_prep_done(1'b0), .block_prepare_start(),
        .step_timestep_fp32(), .step_timestep_valid(), .step_timestep_ready(1'b1),
        .adaln_table_enable(1'b0),
        .block_prepare_done(1'b0), .block_start(), .step_index(), .branch_index(),
        .block_index(), .compute_stage(), .final_start(), .final_done(1'b0),
        .final_head_enable(final_head_enable), .final_param_dma_enable(1'b0),
        .final_shift_wr_en(final_shift_wr_en), .final_shift_wr_beat(final_shift_wr_beat),
        .final_shift_wr_data(final_shift_wr_data), .final_scale_wr_en(final_scale_wr_en),
        .final_scale_wr_beat(final_scale_wr_beat), .final_scale_wr_data(final_scale_wr_data),
        .final_weight_wr_en(final_weight_wr_en), .final_weight_wr_beat(final_weight_wr_beat),
        .final_weight_wr_data(final_weight_wr_data), .final_bias_wr_en(final_bias_wr_en),
        .final_bias_wr_addr(final_bias_wr_addr), .final_bias_wr_data(final_bias_wr_data),
        .cfg_combine_start(), .cfg_in_valid(1'b0), .cfg_in_ready(),
        .cfg_velocity_uncond_fp16(512'd0), .cfg_velocity_cond_fp16(512'd0),
        .cfg_guidance_fp16(16'h4200), .euler_start(), .euler_step_size_fp16(16'd0),
        .euler_latent_valid(1'b0), .euler_latent_ready(), .euler_latent_fp16(512'd0),
        .euler_out_valid(), .euler_out_ready(1'b0), .euler_updated_latent_fp16(),
        .dicache_enable(dicache_enable), .dicache_threshold_q16(32'd6554),
        .dicache_probe_error_q16(32'd0), .dicache_gamma_fp16(16'd0),
        .dicache_residual_in_valid(1'b0), .dicache_residual_in_ready(),
        .dicache_residual_in_data(512'd0), .dicache_residual_in_last(1'b0),
        .dicache_residual_out_valid(), .dicache_residual_out_ready(1'b1),
        .dicache_residual_out_data(), .dicache_residual_out_last(),
        .dicache_residual_cache_valid(), .dicache_residual_dcta_valid(),
        .dicache_cache_error(), .dicache_probe_start(), .dicache_reuse_residual(),
        .dicache_refresh_residual(), .load_en(load_en), .load_target(load_target),
        .load_address(load_address), .load_data(load_data), .load_vec_en(1'b0),
        .load_vec_target(4'd0), .load_vec_beat(16'd0), .load_vec_data(512'd0),
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
        .result_read_beat(unused_result_beat),
        .result_read_data(), .m_axi_arid(), .m_axi_araddr(), .m_axi_arlen(),
        .m_axi_arsize(), .m_axi_arburst(), .m_axi_arvalid(), .m_axi_arready(1'b0),
        .m_axi_rid(4'd0), .m_axi_rdata(512'd0), .m_axi_rresp(2'd0),
        .m_axi_rlast(1'b0), .m_axi_rvalid(1'b0), .m_axi_rready()
    );

    task automatic start_final(input logic branch);
        wait(!dut.final_head_done);
        @(negedge clk);
        if(branch) force dut.branch_index=1'b1;
        else force dut.branch_index=1'b0;
        // Exercise the integrated velocity-cache refresh path alongside the
        // final-head output stream for this CFG branch.
        force dut.dicache_refresh_residual=1'b1;
        @(negedge clk);
        release dut.dicache_refresh_residual;
        $display("FINAL_START branch=%0d",branch);
        force dut.final_start=1'b1;
        @(negedge clk);
        release dut.final_start;
        fork
            begin wait(dut.final_head_done); end
            begin repeat(40000) @(negedge clk); $display("TIMEOUT status read_active=%b beat=%0d act_wr=%b head_start=%b head_busy=%b head_done=%b head_state=%0d proj_state=%0d",dut.final_read_active,dut.final_read_beat,dut.final_act_wr_en,dut.final_head_start,dut.final_head_busy,dut.final_head_done,dut.u_final_head.state,dut.u_final_head.u_projection.state); $fatal(1,"final-head timeout branch=%0d",branch); end
        join_any
        disable fork;
        if (final_activation_write_count != (branch+1)*4608 ||
            dut.final_read_beat != 15'd4607)
            $fatal(1,"final activation vector reads=%0d last_beat=%0d",
                   final_activation_write_count,dut.final_read_beat);
        if (!dut.dicache_refresh_pending[branch])
            $fatal(1,"DiCache residual refresh was not queued for branch=%0d",branch);
        release dut.branch_index;
    endtask

    task automatic load_final_parameters;
        for(int b=0;b<18;b++) begin
            for(int l=0;l<32;l++) begin
                final_shift_wr_data[l*16+:16]=shift_mem[b*32+l];
                final_scale_wr_data[l*16+:16]=scale_mem[b*32+l];
            end
            final_shift_wr_beat=b; final_scale_wr_beat=b;
            final_shift_wr_en=1; final_scale_wr_en=1; @(negedge clk);
        end
        final_shift_wr_en=0; final_scale_wr_en=0;
        for(int b=0;b<576;b++) begin
            for(int l=0;l<32;l++) begin
                int q;
                q=b*32+l;
                final_weight_wr_data[l*16+:16]=(q<16*576)?weight_mem[q]:16'd0;
            end
            final_weight_wr_beat=b; final_weight_wr_en=1; @(negedge clk);
        end
        final_weight_wr_en=0;
        for(int j=0;j<16;j++) begin
            final_bias_wr_addr=j; final_bias_wr_data=bias_mem[j];
            final_bias_wr_en=1; @(negedge clk);
        end
        final_bias_wr_en=0;
    endtask

    initial begin
        $readmemh({root,"final_norm_x_fp16.hex"},act_mem);
        $readmemh({root,"final_shift_fp16.hex"},shift_mem);
        $readmemh({root,"final_scale_fp16.hex"},scale_mem);
        $readmemh({root,"linear_weight_fp16.hex"},weight_mem);
        $readmemh({root,"linear_bias_fp16.hex"},bias_mem);
        $readmemh({root,"reference_latent_fp16.hex"},expected);
        repeat(5) @(negedge clk); rst_n=1;
        load_target=4'd0;
        for(int q=0;q<256*576;q++) begin
            load_en=1; load_address=q; load_data=act_mem[q]; @(negedge clk);
        end
        load_en=0;
        load_final_parameters();

        start_final(1'b0);
        if(output_count!=128) $fatal(1,"branch-0 output beat count=%0d",output_count);
        if(dut.final_velocity_index!=0) $fatal(1,"branch-0 velocity count did not wrap");
        for(int b=0;b<128;b++) begin
            saved_uncond[b]=dut.uncond_velocity_mem[b];
            for(int l=0;l<32;l++) begin
                int idx; idx=b*32+l;
                if($isunknown(saved_uncond[b][l*16+:16])) $fatal(1,"X in branch-0 output idx=%0d",idx);
                got_value=half_real(saved_uncond[b][l*16+:16]);
                exp_value=half_real(expected[idx]);
                error_value=(got_value>=exp_value)?got_value-exp_value:exp_value-got_value;
                if(error_value>max_error) max_error=error_value;
                if((exp_value>=0.0?exp_value:-exp_value)>peak_value)
                    peak_value=(exp_value>=0.0?exp_value:-exp_value);
            end
        end

        start_final(1'b1);
        if(output_count!=256) $fatal(1,"branch-1 total output beat count=%0d",output_count);
        for(int b=0;b<128;b++)
            if(dut.cond_velocity_mem[b] !== saved_uncond[b])
                $fatal(1,"conditional/unconditional beat mismatch at %0d",b);

        // Seed both latent banks with x_t=1, then model completion of the
        // Euler update. The bridge must cache velocity-x_t for both branches.
        for(int b=0;b<128;b++) begin
            for(int l=0;l<32;l++) begin
                dut.latent_mem[0][b][l*16+:16]=16'h3c00;
                dut.latent_mem[1][b][l*16+:16]=16'h3c00;
            end
        end
        @(negedge clk);
        force dut.euler_update_done=1'b1;
        @(negedge clk);
        release dut.euler_update_done;
        fork
            begin
                wait(dut.dicache_refresh_pending==0 &&
                     dut.u_runtime_dma.u_dicache_policy.residual_valid==2'b11 &&
                     controller_euler_done_count==1);
            end
            begin repeat(2000) @(negedge clk);
                $fatal(1,"DiCache residual refresh timeout state=%0d pending=%b anchors=%0d/%0d index=%0d cache_active=%b cache_error=%b done=%b branch=%b",
                       dut.dicache_refresh_state,dut.dicache_refresh_pending,
                       dut.u_dicache_cache.anchor_count[0],dut.u_dicache_cache.anchor_count[1],
                       dut.dicache_refresh_index,dut.u_dicache_cache.refresh_active,
                       dut.dicache_cache_error,dut.dicache_cache_refresh_done,
                       dut.dicache_refresh_branch);
            end
        join_any
        disable fork;
        if(dut.dicache_refresh_pending!=0 || controller_euler_done_count!=1 ||
           dut.u_runtime_dma.u_dicache_policy.residual_valid!=2'b11)
            $fatal(1,"Euler/cache completion mismatch pending=%b euler_done_count=%0d policy_valid=%b state=%0d",
                   dut.dicache_refresh_pending,controller_euler_done_count,
                   dut.u_runtime_dma.u_dicache_policy.residual_valid,
                   dut.dicache_refresh_state);

        // Reuse branch 0's seeded residual. DCTA computes a residual velocity,
        // which must have the current latent added back before CFG consumes it.
        @(negedge clk);
        force dut.branch_index=1'b0;
        force dut.dicache_reuse_residual=1'b1;
        @(negedge clk);
        release dut.dicache_reuse_residual;
        fork
            begin wait(dut.dicache_velocity_loaded); end
            begin repeat(1000) @(negedge clk); $fatal(1,"DiCache velocity reuse timeout"); end
        join_any
        disable fork;
        if(reuse_beat_count!=128) $fatal(1,"DiCache reuse beats=%0d",reuse_beat_count);
        reuse_max_error=0.0;
        for(int b=0;b<128;b++) begin
            for(int l=0;l<32;l++) begin
                got_value=half_real(dut.uncond_velocity_mem[b][l*16+:16]);
                exp_value=half_real(saved_uncond[b][l*16+:16]);
                error_value=(got_value>=exp_value)?got_value-exp_value:exp_value-got_value;
                if(error_value>reuse_max_error) reuse_max_error=error_value;
            end
            saved_uncond[b]=dut.uncond_velocity_mem[b];
        end
        if(reuse_max_error/peak_value>0.01)
            $fatal(1,"DiCache reconstructed velocity rel_peak=%e",reuse_max_error/peak_value);
        if(!dut.dicache_branch_reused || !dut.controller_final_done ||
           dut.final_read_active)
            $fatal(1,"DiCache reuse did not arm final-head bypass");
        force dut.final_start=1'b1;
        @(negedge clk);
        release dut.final_start;
        if(dut.final_read_active || dut.final_head_start)
            $fatal(1,"final-head projection restarted after DiCache reuse");
        release dut.branch_index;

        @(negedge clk);
        force dut.cfg_combine_start=1'b1;
        @(negedge clk);
        release dut.cfg_combine_start;
        fork
            begin wait(dut.u_cfg_euler_path.write_ptr == 8'd128); end
            begin repeat(2000) @(negedge clk);
                $display("CFG_DIAG pair_active=%0d pair_index=%0d in_valid=%0d in_ready=%0d cfg_state=%0d cfg_count=%0d cfg_done=%0d",
                         dut.cfg_pair_active,dut.cfg_pair_index,dut.cfg_path_valid,dut.cfg_path_ready,
                         dut.u_cfg_euler_path.u_cfg.state,dut.u_cfg_euler_path.u_cfg.element_count,
                         dut.cfg_combine_done);
                $fatal(1,"CFG pair replay timeout");
            end
        join_any
        disable fork;
        @(negedge clk);
        $display("CFG_FINAL write_ptr=%0d element_count=%0d state=%0d pair_active=%0d pair_index=%0d",
                 dut.u_cfg_euler_path.write_ptr,
                 dut.u_cfg_euler_path.u_cfg.element_count,
                 dut.u_cfg_euler_path.u_cfg.state,
                 dut.cfg_pair_active,dut.cfg_pair_index);
        for(int b=0;b<128;b++) begin
            for(int l=0;l<32;l++) begin
                logic [15:0] expected_cfg;
                expected_cfg=cfg_reference_lane(saved_uncond[b][l*16+:16],
                    dut.cond_velocity_mem[b][l*16+:16]);
                if(dut.u_cfg_euler_path.velocity_mem[b][l*16+:16] !== expected_cfg)
                    $fatal(1,"CFG replay mismatch beat=%0d lane=%0d got=%h expected=%h",
                        b,l,dut.u_cfg_euler_path.velocity_mem[b][l*16+:16],expected_cfg);
            end
        end
        if(peak_value==0.0 || max_error/peak_value>0.01)
            $fatal(1,"integrated final-head golden error=%e",max_error/peak_value);
        $display("SUPRA2_BRIDGE_FINAL_HEAD_CFG_PASS branches=2 beats=128 fp16_rel_peak=%e",
                 max_error/peak_value);
        $finish;
    end

    always @(posedge clk) begin
        if(dut.controller_euler_done) controller_euler_done_count++;
        if(dut.final_out_valid && dut.final_out_ready) output_count++;
        if(dut.dicache_residual_out_valid && dut.dicache_cache_reuse_ready)
            reuse_beat_count++;
        if(dut.final_head_start) $display("INTERNAL_HEAD_START t=%0t",$time);
        if(dut.final_head_done) $display("INTERNAL_HEAD_DONE t=%0t",$time);
    end
endmodule
`default_nettype wire
