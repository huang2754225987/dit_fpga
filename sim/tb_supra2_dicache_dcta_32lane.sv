`timescale 1ns/1ps
`default_nettype none

module tb_supra2_dicache_dcta_32lane;
    localparam int LANES = 32;
    logic clk = 1'b0, rst_n = 1'b0;
    logic in_valid = 1'b0;
    wire in_ready;
    logic in_last = 1'b1;
    logic [LANES*16-1:0] residual_alpha_fp16 = '0;
    logic [LANES*16-1:0] residual_beta_fp16 = '0;
    logic [15:0] gamma_fp16 = '0;
    wire out_valid;
    wire out_last;
    logic out_ready = 1'b0;
    logic [LANES*16-1:0] residual_out_fp16;

    always #5 clk = ~clk;

    supra2_dicache_dcta_32lane #(.LANES(LANES)) dut (.*);

    task automatic run_case(input logic [15:0] gamma,
                            input logic [15:0] expected);
        @(negedge clk);
        residual_alpha_fp16 = {LANES{16'h4200}}; // 3.0
        residual_beta_fp16 = {LANES{16'h3c00}};  // 1.0
        gamma_fp16 = gamma;
        in_valid = 1'b1;
        do @(posedge clk); while (!in_ready);
        @(negedge clk); in_valid = 1'b0;

        do @(negedge clk); while (!out_valid);
        if (residual_out_fp16 !== {LANES{expected}} || !out_last)
            $fatal(1, "DCTA mismatch gamma=%h got_lane0=%h expected=%h",
                   gamma, residual_out_fp16[15:0], expected);
        repeat (2) begin
            @(negedge clk);
            if (!out_valid || !out_last || residual_out_fp16 !== {LANES{expected}})
                $fatal(1, "DCTA output was not held under backpressure");
        end
        out_ready = 1'b1;
        @(posedge clk);
        @(negedge clk); out_ready = 1'b0;
    endtask

    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk); rst_n = 1'b1;
        run_case(16'h0000, 16'h3c00); // gamma=0 -> beta
        run_case(16'h3c00, 16'h4200); // gamma=1 -> alpha
        run_case(16'h3800, 16'h4000); // gamma=0.5 -> midpoint 2.0
        $display("SUPRA2_DICACHE_DCTA_32LANE_PASS lanes=%0d fp32_intermediate=1 backpressure=1",
                 LANES);
        $finish;
    end

    initial begin
        #100_000;
        $fatal(1, "DCTA test timeout");
    end
endmodule

`default_nettype wire
