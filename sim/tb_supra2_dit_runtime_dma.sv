`timescale 1ns/1ps
`default_nettype none

module tb_supra2_dit_runtime_dma;
    import supra2_dit_ddr_layout::*;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic start = 1'b0;
    logic busy, done;
    logic context_start, context_done = 1'b0;
    logic step_prep_start, step_prep_done = 1'b0;
    logic [31:0] step_timestep_fp32;
    logic step_timestep_valid, step_timestep_ready;
    logic adaln_table_enable = 1'b1;
    logic [511:0] adaln_data;
    logic adaln_valid, adaln_ready = 1'b1, adaln_last, adaln_done;
    logic timestep_stalled = 1'b0;
    logic [31:0] stalled_timestep = '0;
    logic block_prepare_start, block_prepare_done = 1'b0;
    logic block_start;
    logic [5:0] step_index;
    logic branch_index;
    logic [3:0] block_index;
    logic block_done = 1'b0;
    logic final_start, final_done = 1'b0;
    logic final_param_dma_enable = 1'b0;
    logic vector_load_req = 1'b0;
    logic [31:0] vector_load_address = '0, vector_load_beats = '0;
    logic [3:0] vector_load_target = '0;
    logic [15:0] vector_load_base_beat = '0;
    wire vector_load_ready, vector_load_valid, vector_load_last, vector_load_done;
    wire [3:0] vector_load_target_out;
    wire [15:0] vector_load_beat;
    wire [511:0] vector_load_data;
    logic qlinear_load_req = 1'b0;
    logic [31:0] qlinear_load_address =
        DDR_BASE + QLAYER_WEIGHT_OFFSET[X_EMBED_LAYER_ID];
    logic [31:0] qlinear_load_beats = 32'd16;
    wire qlinear_load_ready, qlinear_load_valid, qlinear_load_last, qlinear_load_done;
    wire [511:0] qlinear_load_data;
    logic qlinear_load_data_ready;
    logic qlinear_stalled = 1'b0;
    logic [511:0] stalled_qlinear_data = '0;
    logic final_param_valid, final_param_done;
    logic [1:0] final_param_kind;
    logic [15:0] final_param_beat;
    logic [511:0] final_param_data;
    logic cfg_combine_start, cfg_combine_done = 1'b0;
    logic euler_start, euler_done = 1'b0;
    logic dicache_enable = 1'b0;
    logic [31:0] dicache_threshold_q16 = 32'd6554;
    logic [31:0] dicache_probe_error_q16 = '0;
    logic dicache_cache_refresh_done = 1'b0;
    logic dicache_cache_refresh_branch_index = 1'b0;
    logic dicache_reuse_done = 1'b0;
    logic dicache_probe_start, dicache_reuse_residual, dicache_refresh_residual;
    logic stage_request = 1'b0;
    logic stage_request_ready;
    logic [2:0] stage_request_index = 3'd0;
    logic [1:0] stage_component = 2'd0;
    logic tile_select = 1'b1;
    logic [15:0] n_tile_index = 16'd0;
    logic [15:0] k_tile_index = 16'd0;
    logic [511:0] stage_data;
    logic stage_valid, stage_ready = 1'b1, stage_last, stage_dma_done, dma_error;
    logic [3:0] m_axi_arid;
    logic [31:0] m_axi_araddr;
    logic [7:0] m_axi_arlen;
    logic [2:0] m_axi_arsize;
    logic [1:0] m_axi_arburst;
    logic m_axi_arvalid, m_axi_arready = 1'b1;
    logic [3:0] m_axi_rid = 4'd0;
    logic [511:0] m_axi_rdata;
    logic [1:0] m_axi_rresp = 2'b00;
    logic m_axi_rlast, m_axi_rvalid, m_axi_rready;

    logic model_active = 1'b0;
    logic [31:0] model_address = '0;
    logic [6:0] model_beats_left = '0;
    logic fetch_launched = 1'b0;
    integer block_events = 0;
    integer axi_bursts = 0;
    integer stream_beats = 0;
    integer dma_done_events = 0;
    integer timestep_reads = 0;
    integer timestep_values = 0;
    integer stage_axi_bursts = 0;
    integer qlinear_axi_bursts = 0;
    integer qlinear_stream_beats = 0;
    integer adaln_axi_bursts = 0;
    integer adaln_request_count = 0;
    integer adaln_done_count = 0;
    integer adaln_beat_count = 0;
    integer adaln_beats_this_block = 0;
    logic adaln_first_burst_pending = 1'b0;
    logic [31:0] expected_adaln_address = '0;
    integer cycles = 0;

    always #5 clk = ~clk;
    always_comb begin
        step_timestep_ready = ((cycles % 4) != 0);
        qlinear_load_data_ready = ((cycles % 5) != 0);
    end
    assign m_axi_rvalid = model_active;
    always_comb begin
        if (model_address >= DDR_BASE + T_SCHEDULE_OFFSET &&
            model_address < DDR_BASE + T_SCHEDULE_OFFSET + 32'd256) begin
            for (int lane=0; lane<16; lane++)
                m_axi_rdata[lane*32+:32] = model_address + lane*4;
        end else m_axi_rdata = {16{model_address}};
    end
    assign m_axi_rlast = model_active && (model_beats_left == 1);

    supra2_dit_runtime_dma dut (.*);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            model_active <= 1'b0;
            model_address <= '0;
            model_beats_left <= '0;
            context_done <= 1'b0;
            step_prep_done <= 1'b0;
            block_prepare_done <= 1'b0;
            block_done <= 1'b0;
            final_done <= 1'b0;
            cfg_combine_done <= 1'b0;
            euler_done <= 1'b0;
            stage_request <= 1'b0;
            fetch_launched <= 1'b0;
        end else begin
            context_done <= context_start;
            step_prep_done <= 1'b0;
            if (step_timestep_valid && step_timestep_ready)
                step_prep_done <= 1'b1;
            block_prepare_done <= adaln_done;
            final_done <= final_start;
            cfg_combine_done <= cfg_combine_start;
            euler_done <= euler_start;
            block_done <= 1'b0;

            if (block_start) begin
                if ((step_index == 0) && (block_index == 0) && !fetch_launched) begin
                    stage_request <= 1'b1;
                    fetch_launched <= 1'b1;
                end else begin
                    block_done <= 1'b1;
                end
            end
            if (stage_request && stage_request_ready) stage_request <= 1'b0;
            if (stage_dma_done) block_done <= 1'b1;

            if (m_axi_arvalid && m_axi_arready) begin
                if (model_active) $fatal(1, "overlapping AXI read bursts");
                model_active <= 1'b1;
                model_address <= m_axi_araddr;
                model_beats_left <= {1'b0, m_axi_arlen} + 1'b1;
            end
            if (m_axi_rvalid && m_axi_rready) begin
                if (model_beats_left == 1) begin
                    model_active <= 1'b0;
                    model_beats_left <= '0;
                end else begin
                    model_address <= model_address + 32'd64;
                    model_beats_left <= model_beats_left - 1'b1;
                end
            end
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            block_events = 0;
            axi_bursts = 0;
            stream_beats = 0;
            dma_done_events = 0;
            cycles = 0;
            timestep_reads = 0;
            timestep_values = 0;
            stage_axi_bursts = 0;
            qlinear_axi_bursts = 0;
            qlinear_stream_beats = 0;
            adaln_axi_bursts = 0;
            adaln_request_count = 0;
            adaln_done_count = 0;
            adaln_beat_count = 0;
            adaln_beats_this_block = 0;
            adaln_first_burst_pending = 1'b0;
            expected_adaln_address = '0;
            timestep_stalled = 1'b0;
            stalled_timestep = '0;
            qlinear_stalled = 1'b0;
            stalled_qlinear_data = '0;
        end else begin
            cycles = cycles + 1;
            if (cycles > 200000) $fatal(1, "runtime DMA shell watchdog");
            if (timestep_stalled &&
                (!step_timestep_valid || step_timestep_fp32 !== stalled_timestep))
                $fatal(1,"timestep stream changed while backpressured");
            timestep_stalled = step_timestep_valid && !step_timestep_ready;
            if (timestep_stalled) stalled_timestep = step_timestep_fp32;
            if (qlinear_stalled &&
                (!qlinear_load_valid || qlinear_load_data !== stalled_qlinear_data))
                $fatal(1, "QLinear stream changed while backpressured");
            qlinear_stalled = qlinear_load_valid && !qlinear_load_data_ready;
            if (qlinear_stalled) stalled_qlinear_data = qlinear_load_data;
            if (block_start) block_events = block_events + 1;
            if (block_prepare_start) begin
                adaln_request_count = adaln_request_count + 1;
                adaln_beats_this_block = 0;
                adaln_first_burst_pending = 1'b1;
                expected_adaln_address = DDR_BASE + BLOCK_ADALN_RUNTIME_OFFSET +
                    ((step_index * 14) + block_index) * 32'd9216;
            end
            if (m_axi_arvalid && m_axi_arready) begin
                axi_bursts = axi_bursts + 1;
                if (m_axi_araddr >= DDR_BASE + T_SCHEDULE_OFFSET &&
                    m_axi_araddr < DDR_BASE + T_SCHEDULE_OFFSET + 32'd256) begin
                    if (m_axi_araddr[5:0] != 0 || m_axi_arlen != 0 ||
                        m_axi_arsize != 3'd6 || m_axi_arburst != 2'b01)
                        $fatal(1,"bad aligned timestep descriptor addr=%h len=%0d",
                               m_axi_araddr,m_axi_arlen);
                    timestep_reads = timestep_reads + 1;
                end else if (dut.reader_qlinear_owner) begin
                    if (m_axi_araddr !== qlinear_load_address || m_axi_arlen !== 8'd15 ||
                        m_axi_arsize !== 3'd6 || m_axi_arburst !== 2'b01)
                        $fatal(1, "unexpected qlinear AXI descriptor addr=%h len=%0d",
                               m_axi_araddr,m_axi_arlen);
                    qlinear_axi_bursts = qlinear_axi_bursts + 1;
                end else if (m_axi_araddr >= DDR_BASE + BLOCK_ADALN_RUNTIME_OFFSET &&
                             m_axi_araddr < DDR_BASE + BLOCK_ADALN_RUNTIME_OFFSET +
                                            32'd6451200) begin
                    adaln_axi_bursts = adaln_axi_bursts + 1;
                    if (adaln_first_burst_pending) begin
                        if (m_axi_araddr !== expected_adaln_address)
                            $fatal(1,"bad AdaLN descriptor got=%h expected=%h step=%0d block=%0d",
                                   m_axi_araddr,expected_adaln_address,step_index,block_index);
                        adaln_first_burst_pending = 1'b0;
                    end
                    if (m_axi_arlen > 8'd63 || m_axi_arsize != 3'd6 ||
                        m_axi_arburst != 2'b01 ||
                        (m_axi_araddr[11:0] + ({4'b0,m_axi_arlen}+1)*64 > 4096))
                        $fatal(1,"AdaLN DMA emitted unsafe burst addr=%h len=%0d",
                               m_axi_araddr,m_axi_arlen);
                end else begin
                    if (m_axi_araddr !== DDR_BASE + BLOCK_STAGE_WEIGHT_OFFSET[0][0] ||
                        m_axi_arlen !== 8'd15 || m_axi_arsize !== 3'd6 ||
                        m_axi_arburst !== 2'b01)
                        $fatal(1, "unexpected first tile AXI descriptor");
                    stage_axi_bursts = stage_axi_bursts + 1;
                end
            end
            if (step_timestep_valid && step_timestep_ready) begin
                if (step_timestep_fp32 !== DDR_BASE + T_SCHEDULE_OFFSET +
                                          (step_index * 4))
                    $fatal(1,"wrong timestep lane step=%0d got=%h expected=%h",
                           step_index,step_timestep_fp32,
                           DDR_BASE+T_SCHEDULE_OFFSET+(step_index*4));
                timestep_values = timestep_values + 1;
            end
            if (stage_valid && stage_ready) begin
                if (stream_beats >= 16 ||
                    stage_data !== {16{DDR_BASE + BLOCK_STAGE_WEIGHT_OFFSET[0][0] +
                                       stream_beats * 64}} ||
                    stage_last !== (stream_beats == 15))
                    $fatal(1, "bad DMA stream beat %0d", stream_beats);
                stream_beats = stream_beats + 1;
            end
            if (qlinear_load_valid && qlinear_load_data_ready) begin
                if (qlinear_stream_beats >= 16 ||
                    qlinear_load_data !==
                        {16{qlinear_load_address + qlinear_stream_beats * 64}} ||
                    qlinear_load_last !== (qlinear_stream_beats == 15))
                    $fatal(1, "bad qlinear DMA stream beat %0d", qlinear_stream_beats);
                qlinear_stream_beats = qlinear_stream_beats + 1;
            end
            if (adaln_valid && adaln_ready) begin
                if (adaln_data !== {16{model_address}} ||
                    adaln_last !== (adaln_beats_this_block == 143))
                    $fatal(1,"bad AdaLN stream beat=%0d last=%b",
                           adaln_beats_this_block,adaln_last);
                adaln_beats_this_block = adaln_beats_this_block + 1;
                adaln_beat_count = adaln_beat_count + 1;
            end
            if (adaln_done) begin
                if (adaln_beats_this_block != 144 || adaln_first_burst_pending)
                    $fatal(1,"AdaLN request ended at beats=%0d",
                           adaln_beats_this_block);
                adaln_done_count = adaln_done_count + 1;
                adaln_beats_this_block = 0;
            end
            if (stage_dma_done) dma_done_events = dma_done_events + 1;
            if (done) begin
                if (busy || dma_error || !fetch_launched || block_events != 828 ||
                    axi_bursts != 52 + adaln_axi_bursts || stage_axi_bursts != 1 ||
                    qlinear_axi_bursts != 1 || qlinear_stream_beats != 16 ||
                    timestep_reads != 50 || timestep_values != 100 ||
                    stream_beats != 16 || dma_done_events != 1 ||
                    adaln_request_count != 828 || adaln_done_count != 828 ||
                    adaln_beat_count != 828*144)
                    $fatal(1, "bad totals blocks=%0d bursts=%0d t_reads=%0d t_values=%0d adaln_req=%0d adaln_done=%0d adaln_beats=%0d tile_beats=%0d dma_done=%0d err=%b",
                           block_events,axi_bursts,timestep_reads,timestep_values,
                           adaln_request_count,adaln_done_count,adaln_beat_count,
                           stream_beats,dma_done_events,dma_error);
                $display("SUPRA2_DIT_RUNTIME_DMA_PASS steps=50 blocks=828 timestep_reads=%0d adaln_reads=%0d adaln_beats=%0d tile_beats=%0d qlinear_beats=%0d",
                         timestep_reads,adaln_done_count,adaln_beat_count,stream_beats,
                         qlinear_stream_beats);
                $finish;
            end
        end
    end

    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk) rst_n = 1'b1;
        wait(qlinear_load_ready);
        @(negedge clk) qlinear_load_req = 1'b1;
        @(negedge clk) qlinear_load_req = 1'b0;
        wait(qlinear_load_done);
        @(negedge clk);
        @(negedge clk) start = 1'b1;
        @(negedge clk) start = 1'b0;
    end
endmodule

`default_nettype wire
