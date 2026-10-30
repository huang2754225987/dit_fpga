`timescale 1ns/1ps
`default_nettype none

module tb_supra2_dicache_residual_cache;
    localparam int LANES = 4;
    localparam int ELEMENTS = 64;
    localparam int BEATS = ELEMENTS / LANES;
    logic clk = 1'b0, rst_n = 1'b0;
    logic run_start = 1'b0;
    logic branch_index = 1'b0;
    logic refresh_start = 1'b0, refresh_valid = 1'b0;
    wire refresh_ready;
    logic [LANES*16-1:0] refresh_residual_fp16 = '0;
    logic refresh_last = 1'b0;
    wire refresh_done;
    logic reuse_start = 1'b0;
    logic [15:0] gamma_fp16 = 16'h3800;
    wire reuse_valid;
    logic reuse_ready = 1'b1;
    wire [LANES*16-1:0] reuse_residual_fp16;
    wire reuse_last, reuse_done, cache_valid, dcta_valid, cache_error;

    always #5 clk = ~clk;

    supra2_dicache_residual_cache #(
        .ELEMENTS(ELEMENTS), .LANES(LANES)
    ) dut (.*);

    task automatic write_anchor(input logic [15:0] value);
        @(negedge clk); refresh_start = 1'b1;
        @(negedge clk); refresh_start = 1'b0;
        for (int beat = 0; beat < BEATS; beat++) begin
            @(negedge clk);
            refresh_valid = 1'b1;
            refresh_residual_fp16 = {LANES{value}};
            refresh_last = (beat == BEATS-1);
            do @(posedge clk); while (!refresh_ready);
        end
        @(negedge clk); refresh_valid = 1'b0; refresh_last = 1'b0;
        if (!refresh_done || cache_error)
            $fatal(1, "residual anchor refresh failed value=%h", value);
    endtask

    task automatic check_reuse(input logic [15:0] expected);
        @(negedge clk); reuse_start = 1'b1;
        @(negedge clk); reuse_start = 1'b0;
        for (int beat = 0; beat < BEATS; beat++) begin
            do @(negedge clk); while (!reuse_valid);
            if (reuse_residual_fp16 !== {LANES{expected}} ||
                reuse_last !== (beat == BEATS-1))
                $fatal(1, "cached DCTA beat %0d mismatch got=%h last=%b",
                       beat, reuse_residual_fp16, reuse_last);
            if (beat == BEATS-1 && !reuse_done)
                $fatal(1, "reuse_done did not coincide with final beat");
            @(posedge clk);
        end
        @(negedge clk);
    endtask

    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk); rst_n = 1'b1;

        write_anchor(16'h3c00); // r_beta = 1.0
        if (!cache_valid || dcta_valid)
            $fatal(1, "single-anchor cache status is wrong");
        check_reuse(16'h3c00); // one anchor returns directly for any gamma

        write_anchor(16'h4200); // r_alpha = 3.0
        if (!cache_valid || !dcta_valid)
            $fatal(1, "two-anchor cache status is wrong");
        check_reuse(16'h4000); // 1.0 + 0.5 * (3.0 - 1.0) = 2.0
        if (cache_error) $fatal(1, "unexpected residual-cache error");

        @(negedge clk); branch_index = 1'b1; reuse_start = 1'b1;
        @(negedge clk); reuse_start = 1'b0;
        @(posedge clk); #1;
        if (!cache_error || cache_valid)
            $fatal(1, "empty branch cache did not fail closed");
        @(negedge clk); run_start = 1'b1;
        @(negedge clk); run_start = 1'b0; branch_index = 1'b0;
        if (cache_valid || cache_error)
            $fatal(1, "new run did not invalidate stale residual anchors");
        $display("SUPRA2_DICACHE_RESIDUAL_CACHE_PASS elements=%0d beats=%0d anchors=2 branches=2",
                 ELEMENTS, BEATS);
        $finish;
    end

    initial begin
        #200_000;
        $fatal(1, "residual cache test timeout");
    end
endmodule

`default_nettype wire
