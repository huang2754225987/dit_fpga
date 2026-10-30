`timescale 1ns/1ps
`default_nettype none

module tb_supra2_dit_final_param_dma;
    import supra2_dit_ddr_layout::*;

    logic clk=1'b0, rst_n=1'b0;
    logic final_param_dma_enable=1'b1;
    logic [5:0] forced_step=6'd7;
    logic model_active=1'b0;
    logic [6:0] model_beats_left='0;
    logic [31:0] model_address='0;
    logic [511:0] model_data;
    wire [3:0] m_axi_arid;
    wire [31:0] m_axi_araddr;
    wire [7:0] m_axi_arlen;
    wire [2:0] m_axi_arsize;
    wire [1:0] m_axi_arburst;
    wire m_axi_arvalid, m_axi_rready;
    wire [3:0] m_axi_rid=4'd0;
    wire [1:0] m_axi_rresp=2'b00;
    wire m_axi_rlast=model_active && model_beats_left==1;
    wire m_axi_rvalid=model_active;
    wire final_param_valid, final_param_done;
    wire [1:0] final_param_kind;
    wire [15:0] final_param_beat;
    wire [511:0] final_param_data;
    integer kind_beats[0:2];
    integer transfer_beats[0:2];
    integer ar_count=0;
    logic [31:0] expected_address='0;
    logic [1:0] ar_kind=2'd0;

    always #5 clk=~clk;

    always_comb begin
        for(int lane=0;lane<32;lane++)
            model_data[lane*16+:16]=16'(model_address[15:0]+16'(lane));
    end

    supra2_dit_runtime_dma dut (
        .clk(clk), .rst_n(rst_n), .start(1'b0), .busy(), .done(),
        .context_start(), .context_done(1'b0), .step_prep_start(),
        .step_prep_done(1'b0), .block_prepare_start(),
        .block_prepare_done(1'b0), .block_start(), .step_index(),
        .branch_index(), .block_index(), .block_done(1'b0),
        .final_start(), .final_done(1'b0),
        .final_param_dma_enable(final_param_dma_enable),
        .final_param_valid(final_param_valid), .final_param_kind(final_param_kind),
        .final_param_beat(final_param_beat), .final_param_data(final_param_data),
        .final_param_done(final_param_done),
        .vector_load_req(1'b0), .vector_load_ready(),
        .vector_load_address(32'd0), .vector_load_beats(32'd0),
        .vector_load_target(4'd0), .vector_load_base_beat(16'd0),
        .vector_load_valid(), .vector_load_target_out(), .vector_load_beat(),
        .vector_load_data(), .vector_load_last(), .vector_load_done(),
        .qlinear_load_req(1'b0), .qlinear_load_ready(),
        .qlinear_load_address(32'd0), .qlinear_load_beats(32'd0),
        .qlinear_load_valid(), .qlinear_load_data_ready(1'b1),
        .qlinear_load_data(), .qlinear_load_last(), .qlinear_load_done(),
        .cfg_combine_start(), .cfg_combine_done(1'b0), .euler_start(),
        .euler_done(1'b0), .step_timestep_fp32(), .step_timestep_valid(),
        .step_timestep_ready(1'b1), .adaln_table_enable(1'b0), .adaln_data(),
        .adaln_valid(), .adaln_ready(1'b1), .adaln_last(), .adaln_done(),
        .dicache_enable(1'b0), .dicache_threshold_q16(32'd6554),
        .dicache_probe_error_q16(32'd0), .dicache_cache_refresh_done(1'b0),
        .dicache_cache_refresh_branch_index(1'b0), .dicache_reuse_done(1'b0),
        .dicache_probe_start(), .dicache_reuse_residual(),
        .dicache_refresh_residual(), .stage_request(1'b0),
        .stage_request_ready(), .stage_request_index(3'd0),
        .stage_component(2'd0), .tile_select(1'b0), .n_tile_index(16'd0),
        .k_tile_index(16'd0), .stage_data(), .stage_valid(),
        .stage_ready(1'b1), .stage_last(), .stage_dma_done(), .dma_error(),
        .m_axi_arid(m_axi_arid), .m_axi_araddr(m_axi_araddr),
        .m_axi_arlen(m_axi_arlen), .m_axi_arsize(m_axi_arsize),
        .m_axi_arburst(m_axi_arburst), .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(1'b1), .m_axi_rid(m_axi_rid), .m_axi_rdata(model_data),
        .m_axi_rresp(m_axi_rresp), .m_axi_rlast(m_axi_rlast),
        .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(m_axi_rready)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            model_active<=1'b0;
            model_beats_left<='0;
            model_address<='0;
        end else begin
            if(!model_active && m_axi_arvalid) begin
                model_active<=1'b1;
                model_beats_left<=m_axi_arlen[6:0]+1'b1;
                model_address<=m_axi_araddr;
            end else if(model_active && m_axi_rready) begin
                if(model_beats_left==1) begin
                    model_active<=1'b0;
                    model_beats_left<='0;
                end else begin
                    model_beats_left<=model_beats_left-1'b1;
                    model_address<=model_address+32'd64;
                end
            end
        end
    end

    always @(posedge clk) begin
        if(rst_n && m_axi_arvalid) begin
            if(m_axi_arsize!=3'd6 || m_axi_arburst!=2'b01 || m_axi_arlen>=64)
                $fatal(1,"invalid final-param burst size=%0d len=%0d",m_axi_arsize,m_axi_arlen);
            if(final_param_kind==2'd2 &&
               m_axi_araddr==DDR_BASE+FINAL_ADALN_RUNTIME_OFFSET+32'd7*32'd2304) begin
                expected_address=DDR_BASE+FINAL_ADALN_RUNTIME_OFFSET+32'd7*32'd2304;
                ar_kind=2'd2;
            end
            if(m_axi_araddr!==expected_address)
                $fatal(1,"final-param address mismatch kind=%0d got=%h expected=%h",
                       final_param_kind,m_axi_araddr,expected_address);
            expected_address=expected_address+(32'(m_axi_arlen)+1)*32'd64;
            ar_count++;
        end
        if(rst_n && final_param_valid) begin
            if(final_param_beat==15'd0) transfer_beats[final_param_kind]=0;
            if(final_param_kind>2 || final_param_beat!=transfer_beats[final_param_kind])
                $fatal(1,"final-param ordering kind=%0d beat=%0d expected=%0d",
                       final_param_kind,final_param_beat,transfer_beats[final_param_kind]);
            if(final_param_data[15:0]!==model_address[15:0])
                $fatal(1,"final-param data mismatch kind=%0d beat=%0d",
                       final_param_kind,final_param_beat);
            kind_beats[final_param_kind]++;
            transfer_beats[final_param_kind]++;
        end
    end

    initial begin
        kind_beats[0]=0; kind_beats[1]=0; kind_beats[2]=0;
        transfer_beats[0]=0; transfer_beats[1]=0; transfer_beats[2]=0;
        expected_address=DDR_BASE+PASSTHROUGH_OFFSET[33];
        repeat(5) @(negedge clk);
        rst_n=1'b1;
        force dut.step_index=6'd7;
        @(negedge clk); force dut.final_start=1'b1;
        @(negedge clk); release dut.final_start;
        fork
            begin wait(final_param_done); end
            begin repeat(10000) @(negedge clk); $fatal(1,"final-param DMA timeout"); end
        join_any
        disable fork;
        @(negedge clk);
        @(negedge clk); force dut.final_start=1'b1;
        @(negedge clk); release dut.final_start;
        fork
            begin wait(final_param_done); end
            begin repeat(10000) @(negedge clk); $fatal(1,"cached final-param DMA timeout"); end
        join_any
        disable fork;
        @(negedge clk);
        if(kind_beats[0]!=288 || kind_beats[1]!=1 || kind_beats[2]!=72)
            $fatal(1,"final-param beat totals weight=%0d bias=%0d adaln=%0d",
                   kind_beats[0],kind_beats[1],kind_beats[2]);
        if(ar_count<4 || ar_count>8) $fatal(1,"final-param DMA burst count=%0d",ar_count);
        $display("SUPRA2_FINAL_PARAM_DMA_PASS weight=%0d bias=%0d adaln=%0d bursts=%0d cached_static=1",
                 kind_beats[0],kind_beats[1],kind_beats[2],ar_count);
        $finish;
    end
endmodule

`default_nettype wire
