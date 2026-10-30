`timescale 1ns/1ps
`default_nettype none

module tb_supra2_dit_bridge_final_param_dma;
    import supra2_dit_ddr_layout::*;
    logic clk=1'b0, rst_n=1'b0;
    logic axi_model_active=1'b0;
    logic [6:0] axi_beats_left='0;
    logic [31:0] axi_model_address='0;
    integer weight_writes=0, bias_writes=0, shift_writes=0, scale_writes=0;
    integer output_beats=0;
    wire [3:0] m_axi_arid;
    wire [31:0] m_axi_araddr;
    wire [7:0] m_axi_arlen;
    wire [2:0] m_axi_arsize;
    wire [1:0] m_axi_arburst;
    wire m_axi_arvalid, m_axi_rready;
    wire m_axi_rlast=axi_model_active && axi_beats_left==1;
    wire m_axi_rvalid=axi_model_active;
    wire final_start;
    logic vector_load_req=1'b0;
    wire vector_load_ready, vector_load_done;
    logic [31:0] vector_load_address=32'h0000_1000;
    logic [31:0] vector_load_beats=32'd4;
    logic [3:0] vector_load_target=4'd0;
    logic [15:0] vector_load_base_beat=16'd10;
    logic vector_fixture_active=1'b0;
    logic [511:0] m_axi_rdata;
    logic runtime_error;
    logic qlinear_tile_req=1'b0;
    wire qlinear_tile_ready, qlinear_descriptor_valid;
    logic [6:0] qlinear_layer_id=7'(X_EMBED_LAYER_ID);
    logic [1:0] qlinear_component=2'd0;
    logic [15:0] qlinear_n_tile_index=16'd0, qlinear_k_tile_index=16'd0;
    wire [15:0] qlinear_output_features, qlinear_input_features;
    wire [15:0] qlinear_n_tiles, qlinear_k_tiles;
    wire qlinear_data_valid, qlinear_data_last, qlinear_data_done;
    wire [511:0] qlinear_data;
    integer qlinear_beats=0, qlinear_axi_bursts=0;

    always #5 clk=~clk;

    task automatic run_vector_load(input logic [3:0] target,
                                   input logic [15:0] base_beat,
                                   input logic [31:0] beats);
        begin
            vector_load_target=target;
            vector_load_base_beat=base_beat;
            vector_load_beats=beats;
            wait(vector_load_ready);
            vector_fixture_active=1'b1;
            @(negedge clk) vector_load_req=1'b1;
            @(negedge clk) vector_load_req=1'b0;
            wait(vector_load_done);
            @(negedge clk);
            vector_fixture_active=1'b0;
        end
    endtask

    task automatic run_qlinear_tile;
        begin
            if (!qlinear_descriptor_valid || qlinear_output_features != 576 ||
                qlinear_input_features != 16 || qlinear_n_tiles != 18 ||
                qlinear_k_tiles != 1)
                $fatal(1,"x_embed tile descriptor metadata mismatch");
            wait(qlinear_tile_ready);
            vector_fixture_active=1'b1;
            @(negedge clk) qlinear_tile_req=1'b1;
            @(negedge clk) qlinear_tile_req=1'b0;
            wait(qlinear_data_done);
            @(negedge clk);
            vector_fixture_active=1'b0;
            if(qlinear_beats!=16 || qlinear_axi_bursts!=1)
                $fatal(1,"QLinear tile transfer beats=%0d bursts=%0d",
                       qlinear_beats,qlinear_axi_bursts);
        end
    endtask

    supra2_dit_compute_bridge dut (
        .clk(clk), .rst_n(rst_n), .start(1'b0), .busy(), .done(),
        .runtime_error(runtime_error), .context_start(), .context_done(1'b0),
        .step_prep_start(), .step_prep_done(1'b0), .step_timestep_fp32(),
        .step_timestep_valid(), .step_timestep_ready(1'b1),
        .adaln_table_enable(1'b0), .block_prepare_start(),
        .block_prepare_done(1'b0), .block_start(), .step_index(),
        .branch_index(), .block_index(), .compute_stage(), .final_start(final_start),
        .final_done(1'b0), .final_head_enable(1'b1),
        .final_param_dma_enable(1'b1),
        .final_shift_wr_en(1'b0), .final_shift_wr_beat(5'd0),
        .final_shift_wr_data(512'd0), .final_scale_wr_en(1'b0),
        .final_scale_wr_beat(5'd0), .final_scale_wr_data(512'd0),
        .final_weight_wr_en(1'b0), .final_weight_wr_beat(16'd0),
        .final_weight_wr_data(512'd0), .final_bias_wr_en(1'b0),
        .final_bias_wr_addr(5'd0), .final_bias_wr_data(16'd0),
        .cfg_combine_start(), .cfg_in_valid(1'b0), .cfg_in_ready(),
        .cfg_velocity_uncond_fp16(512'd0), .cfg_velocity_cond_fp16(512'd0),
        .cfg_guidance_fp16(16'h4200), .euler_start(),
        .euler_step_size_fp16(16'h0000), .euler_latent_valid(1'b0),
        .euler_latent_ready(), .euler_latent_fp16(512'd0),
        .euler_out_valid(), .euler_out_ready(1'b0),
        .euler_updated_latent_fp16(), .dicache_enable(1'b0),
        .dicache_threshold_q16(32'd6554), .dicache_probe_error_q16(32'd0),
        .dicache_gamma_fp16(16'd0), .dicache_residual_in_valid(1'b0),
        .dicache_residual_in_ready(), .dicache_residual_in_data(512'd0),
        .dicache_residual_in_last(1'b0), .dicache_residual_out_valid(),
        .dicache_residual_out_ready(1'b1), .dicache_residual_out_data(),
        .dicache_residual_out_last(), .dicache_residual_cache_valid(),
        .dicache_residual_dcta_valid(), .dicache_cache_error(),
        .dicache_probe_start(), .dicache_reuse_residual(),
        .dicache_refresh_residual(), .load_en(1'b0), .load_target(4'd0),
        .load_address(21'd0), .load_data(16'd0), .load_vec_en(1'b0),
        .load_vec_target(4'd0), .load_vec_beat(16'd0), .load_vec_data(512'd0),
        .vector_load_req(vector_load_req), .vector_load_ready(vector_load_ready),
        .vector_load_address(vector_load_address), .vector_load_beats(vector_load_beats),
        .vector_load_target(vector_load_target), .vector_load_base_beat(vector_load_base_beat),
        .vector_load_done(vector_load_done), .qlinear_tile_req(qlinear_tile_req),
        .qlinear_tile_ready(qlinear_tile_ready), .qlinear_layer_id(qlinear_layer_id),
        .qlinear_component(qlinear_component), .qlinear_n_tile_index(qlinear_n_tile_index),
        .qlinear_k_tile_index(qlinear_k_tile_index),
        .qlinear_descriptor_valid(qlinear_descriptor_valid),
        .qlinear_output_features(qlinear_output_features),
        .qlinear_input_features(qlinear_input_features), .qlinear_n_tiles(qlinear_n_tiles),
        .qlinear_k_tiles(qlinear_k_tiles), .qlinear_data_valid(qlinear_data_valid),
        .qlinear_data_ready(1'b1), .qlinear_data(qlinear_data),
        .qlinear_data_last(qlinear_data_last), .qlinear_data_done(qlinear_data_done),
        .result_read_beat(15'd0),
        .result_read_data(), .m_axi_arid(m_axi_arid), .m_axi_araddr(m_axi_araddr),
        .m_axi_arlen(m_axi_arlen), .m_axi_arsize(m_axi_arsize),
        .m_axi_arburst(m_axi_arburst), .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(1'b1), .m_axi_rid(4'd0), .m_axi_rdata(m_axi_rdata),
        .m_axi_rresp(2'b00), .m_axi_rlast(m_axi_rlast),
        .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(m_axi_rready)
    );

    always_comb begin
        m_axi_rdata='0;
        if(vector_fixture_active)
            for(int lane=0;lane<32;lane++)
                m_axi_rdata[lane*16+:16]=16'(axi_model_address[15:1]+lane);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_model_active<=1'b0;
            axi_beats_left<='0;
            axi_model_address<='0;
        end else begin
            if(!axi_model_active && m_axi_arvalid) begin
                axi_model_active<=1'b1;
                axi_beats_left<=m_axi_arlen[6:0]+1'b1;
                axi_model_address<=m_axi_araddr;
            end else if(axi_model_active && m_axi_rready) begin
                if(axi_beats_left==1) begin
                    axi_model_active<=1'b0;
                    axi_beats_left<='0;
                end else begin
                    axi_beats_left<=axi_beats_left-1'b1;
                    axi_model_address<=axi_model_address+32'd64;
                end
            end
        end
    end

    always @(posedge clk) begin
        if(rst_n && m_axi_arvalid && dut.u_runtime_dma.reader_qlinear_owner) begin
            if(m_axi_araddr !== DDR_BASE + QLAYER_WEIGHT_OFFSET[X_EMBED_LAYER_ID] ||
               m_axi_arlen !== 8'd15 || m_axi_arsize !== 3'd6 ||
               m_axi_arburst !== 2'b01)
                $fatal(1,"bad resolved x_embed AXI descriptor addr=%h len=%0d",
                       m_axi_araddr,m_axi_arlen);
            qlinear_axi_bursts++;
        end
        if(rst_n && qlinear_data_valid) begin
            if(qlinear_beats>=16 ||
               qlinear_data_last !== (qlinear_beats==15))
                $fatal(1,"bad QLinear stream framing beat=%0d last=%b",
                       qlinear_beats,qlinear_data_last);
            for(int lane=0;lane<32;lane++)
                if(qlinear_data[lane*16+:16] !==
                   16'(axi_model_address[15:1]+lane))
                    $fatal(1,"QLinear stream data mismatch beat=%0d lane=%0d got=%h expected=%h model_addr=%h",
                           qlinear_beats,lane,qlinear_data[lane*16+:16],
                           16'(axi_model_address[15:1]+lane),
                           axi_model_address);
            qlinear_beats++;
        end
        if(dut.final_weight_wr_en_i) weight_writes++;
        if(dut.final_bias_wr_en_i) bias_writes++;
        if(dut.final_shift_wr_en_i) shift_writes++;
        if(dut.final_scale_wr_en_i) scale_writes++;
        if(dut.final_head_start &&
           (weight_writes!=288 || bias_writes!=16 ||
            shift_writes!=18 || scale_writes!=18))
            $fatal(1,"final-head started before all DDR parameters loaded");
        if(dut.final_out_valid) begin
            if(dut.final_out_data!==512'd0)
                $fatal(1,"zero fixture produced nonzero final output");
            output_beats++;
        end
        if(dut.final_out_last && output_beats!=128)
            $fatal(1,"final-head output beat count at last=%0d",output_beats);
    end

    initial begin
        repeat(5) @(negedge clk);
        rst_n=1'b1;
        run_qlinear_tile();
        run_vector_load(4'd0,16'd10,32'd4);
        for(int i=0;i<128;i++)
            if(dut.u_block_compute.x_buf[320+i] !== 16'(16'h0800+i))
                $fatal(1,"DDR vector DMA x element %0d got %h expected %h",i,
                       dut.u_block_compute.x_buf[320+i],16'(16'h0800+i));
        vector_load_address=32'h0000_2000;
        run_vector_load(4'd1,16'd3,32'd2);
        for(int i=0;i<64;i++)
            if(dut.u_block_compute.ctx_buf[96+i] !== 16'(16'h1000+i))
                $fatal(1,"DDR vector DMA context element %0d got %h expected %h",i,
                       dut.u_block_compute.ctx_buf[96+i],16'(16'h1000+i));
            else if(dut.u_block_compute.ctx_bank[(96+i)%32][(96+i)/32] !==
                    16'(16'h1000+i))
                $fatal(1,"context RAM bank element %0d got %h expected %h",i,
                       dut.u_block_compute.ctx_bank[(96+i)%32][(96+i)/32],
                       16'(16'h1000+i));
        vector_load_address=32'h0000_3000;
        run_vector_load(4'd2,16'd4,32'd1);
        for(int i=0;i<32;i++)
            if(dut.u_block_compute.param_buf[128+i] !== 16'(16'h1800+i))
                $fatal(1,"DDR vector DMA parameter element %0d got %h expected %h",i,
                       dut.u_block_compute.param_buf[128+i],16'(16'h1800+i));
        force dut.block_result_vec_read_data=512'd0;
        @(negedge clk); force dut.final_start=1'b1;
        @(negedge clk); release dut.final_start;
        fork
            begin wait(dut.final_head_done); end
            begin repeat(50000) @(negedge clk);
                $fatal(1,"bridge final-param DMA/head timeout param_state=%0d read=%b act_loaded=%b param_loaded=%b head=%b",
                    dut.u_runtime_dma.final_param_state,dut.final_read_active,
                    dut.final_activation_loaded,dut.final_params_loaded,dut.final_head_busy);
            end
        join_any
        disable fork;
        if(runtime_error || weight_writes!=288 || bias_writes!=16 ||
           shift_writes!=18 || scale_writes!=18 || output_beats!=128)
            $fatal(1,"integrated final-param totals weight=%0d bias=%0d shift=%0d scale=%0d output=%0d",
                   weight_writes,bias_writes,shift_writes,scale_writes,output_beats);
        if(dut.final_head_start!==1'b0 && dut.final_head_busy!==1'b0)
            $fatal(1,"final-head did not return idle");
        $display("SUPRA2_BRIDGE_FINAL_PARAM_DMA_PASS qlinear_beats=%0d vector_beats=7 weight=%0d bias=%0d shift=%0d scale=%0d output=%0d",
                 qlinear_beats,weight_writes,bias_writes,shift_writes,scale_writes,output_beats);
        $finish;
    end
endmodule

`default_nettype wire
