`timescale 1ns/1ps
`default_nettype none

module tb_supra2_dicache_online_policy;
    logic clk = 1'b0, rst_n = 1'b0;
    logic run_start = 1'b0, enable = 1'b1, branch_index = 1'b0;
    logic probe_valid = 1'b0;
    logic [31:0] probe_error_q16 = '0, threshold_q16 = 32'd6554;
    logic cache_refresh_done = 1'b0;
    wire decision_valid, reuse_residual, refresh_residual;
    wire [31:0] accumulated_error_q16;
    wire cache_valid;

    always #5 clk = ~clk;

    supra2_dicache_online_policy dut (.*);

    task automatic probe(input logic branch, input logic [31:0] error_q16,
                         input logic expect_reuse, input logic expect_refresh,
                         input logic [31:0] expect_accum);
        @(negedge clk);
        branch_index = branch;
        probe_error_q16 = error_q16;
        probe_valid = 1'b1;
        @(posedge clk); #1;
        if (!decision_valid || reuse_residual !== expect_reuse ||
            refresh_residual !== expect_refresh ||
            accumulated_error_q16 !== expect_accum)
            $fatal(1, "DiCache decision mismatch branch=%0d err=%h reuse=%b refresh=%b accum=%h",
                   branch, error_q16, reuse_residual, refresh_residual,
                   accumulated_error_q16);
        @(negedge clk); probe_valid = 1'b0;
    endtask

    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk); rst_n = 1'b1; run_start = 1'b1;
        @(negedge clk); run_start = 1'b0;

        // First probe has no cache and must request a full residual refresh.
        probe(1'b0, 32'd1000, 1'b0, 1'b1, 32'd0);
        @(negedge clk); cache_refresh_done = 1'b1;
        @(negedge clk); cache_refresh_done = 1'b0;
        if (!cache_valid) $fatal(1, "conditional residual cache was not seeded");

        // Accumulated probe error below delta permits cached residual reuse.
        probe(1'b0, 32'd2000, 1'b1, 1'b0, 32'd2000);
        probe(1'b0, 32'd3000, 1'b1, 1'b0, 32'd5000);
        // Crossing delta requests a full recompute and resets the accumulator.
        probe(1'b0, 32'd2000, 1'b0, 1'b1, 32'd0);

        // Branch caches are independent; unconditional starts unseeded.
        probe(1'b1, 32'd500, 1'b0, 1'b1, 32'd0);
        @(negedge clk); branch_index = 1'b1; cache_refresh_done = 1'b1;
        @(negedge clk); cache_refresh_done = 1'b0;
        probe(1'b0, 32'd1000, 1'b1, 1'b0, 32'd1000);

        // Runtime disable forces exact recomputation without using the cache.
        @(negedge clk); enable = 1'b0;
        probe(1'b0, 32'd1, 1'b0, 1'b1, 32'd0);

        $display("SUPRA2_DICACHE_ONLINE_POLICY_PASS threshold_q16=%0d branches=2 enable_bypass=1",
                 threshold_q16);
        $finish;
    end

    initial begin
        #100_000;
        $fatal(1, "DiCache policy test timeout");
    end
endmodule

`default_nettype wire
