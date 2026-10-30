`timescale 1ns/1ps
`default_nettype none

module tb_supra2_block_top_rearm;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic start = 1'b0;
    wire busy, done;
    wire [4:0] stage_id;
    wire [127:0] result_rd_data;
    logic [14:0] result_rd_beat = '0;
    logic ld_en = 1'b0;
    logic [3:0] ld_target = '0;
    logic [20:0] ld_addr = '0;
    logic [15:0] ld_data = '0;
    logic param_vec_wr_en = 1'b0;
    logic [7:0] param_vec_wr_beat = '0;
    logic [511:0] param_vec_wr_data = '0;
    logic ld_vec_en = 1'b0;
    logic [3:0] ld_vec_target = '0;
    logic [15:0] ld_vec_beat = '0;
    logic [511:0] ld_vec_data = '0;
    wire wload_req;
    wire [3:0] wload_stage;

    always #5 clk = ~clk;

    supra2_block_top dut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .start_stage(5'd15), .start_head(4'd0), .start_ni(16'd0),
        .stop_ni(16'd0), .busy(busy), .done(done),
        .ld_en(ld_en), .ld_target(ld_target), .ld_addr(ld_addr), .ld_data(ld_data),
        .ld_vec_en(ld_vec_en), .ld_vec_target(ld_vec_target),
        .ld_vec_beat(ld_vec_beat), .ld_vec_data(ld_vec_data),
        .wt_wr_en(1'b0), .wt_wr_beat(16'd0), .wt_wr_data(256'd0),
        .sb_wr_en(1'b0), .sb_wr_sel(1'b0), .sb_wr_addr(16'd0),
        .sb_wr_data(32'd0), .result_rd_beat(result_rd_beat),
        .result_vec_rd_beat(13'd0), .result_vec_rd_data(),
        .param_vec_wr_en(param_vec_wr_en), .param_vec_wr_beat(param_vec_wr_beat),
        .param_vec_wr_data(param_vec_wr_data),
        .result_rd_data(result_rd_data), .wload_req(wload_req),
        .wload_stage(wload_stage), .wload_ack(1'b0), .stage_id(stage_id)
    );

    task automatic pulse_start;
        @(negedge clk) start = 1'b1;
        @(negedge clk) start = 1'b0;
    endtask

    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk) rst_n = 1'b1;

        // Model one complete 8 x 576 FP16 AdaLN table over 144 AXI beats.
        for (int beat = 0; beat < 144; beat++) begin
            @(negedge clk);
            param_vec_wr_en = 1'b1;
            param_vec_wr_beat = 8'(beat);
            for (int lane = 0; lane < 32; lane++)
                param_vec_wr_data[lane*16 +: 16] = 16'(beat*32 + lane);
        end
        @(negedge clk) param_vec_wr_en = 1'b0;
        if (dut.param_buf[0] !== 16'd0 || dut.param_buf[31] !== 16'd31 ||
            dut.param_buf[32] !== 16'd32 || dut.param_buf[4607] !== 16'd4607)
            $fatal(1, "512-bit AdaLN vector write lane/index mapping failed");
        if (dut.u_param_ram.mem[0][15:0] !== 16'd0 ||
            dut.u_param_ram.mem[0][511:496] !== 16'd31 ||
            dut.u_param_ram.mem[1][15:0] !== 16'd32 ||
            dut.u_param_ram.mem[143][511:496] !== 16'd4607)
            $fatal(1, "Packed parameter RAM differs from observation mirror");

        @(negedge clk);
        ld_vec_en = 1'b1;
        ld_vec_target = 4'd0;
        ld_vec_beat = 16'd5;
        for (int lane = 0; lane < 32; lane++)
            ld_vec_data[lane*16 +: 16] = 16'h5100 + 16'(lane);
        @(negedge clk);
        ld_vec_target = 4'd1;
        ld_vec_beat = 16'd2;
        for (int lane = 0; lane < 32; lane++)
            ld_vec_data[lane*16 +: 16] = 16'h6200 + 16'(lane);
        @(negedge clk) ld_vec_en = 1'b0;
        if (dut.x_buf[160] !== 16'h5100 || dut.x_buf[191] !== 16'h511f ||
            dut.ctx_buf[64] !== 16'h6200 || dut.ctx_buf[95] !== 16'h621f)
            $fatal(1, "512-bit activation/context vector write mapping failed");

        // Scalar writes must preserve every other lane in an existing beat.
        @(negedge clk);
        ld_en=1'b1; ld_target=0; ld_addr=177; ld_data=16'h7bcd;
        @(negedge clk);
        ld_target=1; ld_addr=73; ld_data=16'h4242;
        @(negedge clk);
        ld_target=2; ld_addr=36; ld_data=16'h5151;
        @(negedge clk) ld_en=1'b0;
        for (int lane=0; lane<32; lane++) begin
            if (dut.u_x_ram.mem[5][lane*16 +: 16] !==
                (lane==17 ? 16'h7bcd : 16'h5100+16'(lane)))
                $fatal(1, "Masked X write corrupts lane=%0d", lane);
            if (dut.u_ctx_ram.mem[2][lane*16 +: 16] !==
                (lane==9 ? 16'h4242 : 16'h6200+16'(lane)))
                $fatal(1, "Masked context write corrupts lane=%0d", lane);
            if (dut.u_param_ram.mem[1][lane*16 +: 16] !==
                (lane==4 ? 16'h5151 : 16'(32+lane)))
                $fatal(1, "Masked parameter write corrupts lane=%0d", lane);
        end
        // Exercise consecutive result subwords, including the modified lane.
        for (int part=0; part<4; part++) begin
            @(negedge clk) result_rd_beat=15'(20+part);
            @(posedge clk); #1;
            for (int lane=0; lane<8; lane++)
                if (result_rd_data[lane*16 +: 16] !==
                    (part*8+lane==17 ? 16'h7bcd : 16'h5100+16'(part*8+lane)))
                    $fatal(1, "Synchronous result read misaligned part=%0d lane=%0d", part, lane);
        end
        $display("SUPRA2_BLOCK_MEMORY_MAPPING_PASS masked_lanes=96 result_subwords=4");

        pulse_start();
        fork : first_watchdog
            begin wait (done); end
            begin repeat (20000) @(posedge clk); $fatal(1, "first block timeout stage=%0d", stage_id); end
        join_any
        disable first_watchdog;
        if (busy || stage_id != 5'd16) $fatal(1, "first completion did not pulse from FIN");

        wait (!done && !busy);
        if (stage_id != 5'd17) $fatal(1, "block engine did not return to IDLE");
        pulse_start();
        fork : second_watchdog
            begin wait (done); end
            begin repeat (20000) @(posedge clk); $fatal(1, "second block timeout stage=%0d", stage_id); end
        join_any
        disable second_watchdog;
        if (busy || stage_id != 5'd16) $fatal(1, "second completion failed");
        $display("SUPRA2_BLOCK_TOP_REARM_PASS completed_runs=2 completion_pulse=1");
        $finish;
    end
endmodule

`default_nettype wire
