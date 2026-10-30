`default_nettype none

module tb_supra2_cfg_combine_32lane;
    localparam int LANES = 32;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic start = 1'b0;
    logic [6:0] vector_elements = 7'd64;
    logic [15:0] guidance_fp16 = 16'h4200; // 3.0
    logic in_valid = 1'b0;
    wire in_ready;
    logic [LANES*16-1:0] uncond_data = '0;
    logic [LANES*16-1:0] cond_data = '0;
    wire out_valid;
    logic out_ready = 1'b0;
    wire [LANES*16-1:0] cfg_data;
    wire busy, done;
    integer accepted = 0;
    integer cycles = 0;

    always #5 clk = ~clk;

    supra2_cfg_combine_32lane #(.LANES(LANES), .COUNT_W(7)) dut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .vector_elements(vector_elements), .guidance_fp16(guidance_fp16),
        .in_valid(in_valid), .in_ready(in_ready),
        .velocity_uncond_fp16(uncond_data), .velocity_cond_fp16(cond_data),
        .out_valid(out_valid), .out_ready(out_ready),
        .velocity_cfg_fp16(cfg_data), .busy(busy), .done(done)
    );

    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk); start = 1'b1;
        @(negedge clk); start = 1'b0;

        // Stall the first beat, then check v_u=1, v_c=2 => v_cfg=4.
        for (int lane = 0; lane < LANES; lane++) begin
            uncond_data[lane*16 +: 16] = 16'h3c00;
            cond_data[lane*16 +: 16] = 16'h4000;
        end
        in_valid = 1'b1;
        repeat (3) begin
            @(negedge clk);
            if (!out_valid || in_ready || done)
                $fatal(1, "CFG backpressure contract failed");
        end
        out_ready = 1'b1;
        #1;
        if (!out_valid || !in_ready || done)
            $fatal(1, "CFG first-beat handshake invalid valid=%b ready=%b done=%b",
                out_valid, in_ready, done);
        for (int lane = 0; lane < LANES; lane++)
            if (cfg_data[lane*16 +: 16] !== 16'h4400)
                $fatal(1, "CFG first beat lane %0d got %h", lane,
                    cfg_data[lane*16 +: 16]);

        // Second beat: v_u=2, v_c=1 => v_cfg=-1.
        for (int lane = 0; lane < LANES; lane++) begin
            uncond_data[lane*16 +: 16] = 16'h4000;
            cond_data[lane*16 +: 16] = 16'h3c00;
        end
        @(negedge clk);
        if (!out_valid || !in_ready || !done)
            $fatal(1, "CFG final-beat/done contract invalid");
        for (int lane = 0; lane < LANES; lane++)
            if (cfg_data[lane*16 +: 16] !== 16'hbc00)
                $fatal(1, "CFG second beat lane %0d got %h", lane,
                    cfg_data[lane*16 +: 16]);
        @(negedge clk);
        in_valid = 1'b0;
        if (busy || done || accepted != 64)
            $fatal(1, "CFG unit did not return idle");
        $display("SUPRA2_CFG_COMBINE_32LANE_PASS lanes=%0d elements=64 backpressure=1", LANES);
        $finish;
    end

    always @(posedge clk) begin
        if (rst_n) begin
            cycles <= cycles + 1;
            if (out_valid && out_ready) accepted <= accepted + LANES;
            if (cycles > 100) $fatal(1, "CFG test timed out");
        end
    end
endmodule

`default_nettype wire
