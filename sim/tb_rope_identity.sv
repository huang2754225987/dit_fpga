`timescale 1ns/1ps
`default_nettype none

module tb_rope_identity;
    import isa_pkg::*;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic cmd_valid = 1'b0;
    logic cmd_ready;
    logic [3:0] cmd_vd = 4'd0;
    logic [3:0] cmd_vs1 = 4'd0;
    logic [3:0] cmd_vs2 = 4'd3;
    logic [3:0] rf_rd_addr_a, rf_rd_addr_b;
    logic [511:0] rf_rd_data_a, rf_rd_data_b;
    logic rf_wr_en;
    logic [3:0] rf_wr_addr;
    logic [511:0] rf_wr_data;
    logic busy, done;

    always #5 clk = ~clk;

    always_comb begin
        rf_rd_data_a = '0;
        rf_rd_data_b = '0;
        for (int lane = 0; lane < 32; lane++) begin
            rf_rd_data_a[lane * 16 +: 16] = rf_rd_addr_a == 4'd0
                                            ? 16'h3400 : 16'h0000;
            rf_rd_data_b[lane * 16 +: 16] = rf_rd_addr_b == 4'd3
                                            ? (lane < 16 ? 16'h3c00
                                                         : 16'h0000)
                                            : 16'h4000;
        end
    end

    rope_engine dut (
        .clk(clk),
        .rst_n(rst_n),
        .cmd_valid(cmd_valid),
        .cmd_ready(cmd_ready),
        .cmd_vd(cmd_vd),
        .cmd_vs1(cmd_vs1),
        .cmd_vs2(cmd_vs2),
        .rf_rd_addr_a(rf_rd_addr_a),
        .rf_rd_data_a(rf_rd_data_a),
        .rf_rd_addr_b(rf_rd_addr_b),
        .rf_rd_data_b(rf_rd_data_b),
        .rf_wr_en(rf_wr_en),
        .rf_wr_addr(rf_wr_addr),
        .rf_wr_data(rf_wr_data),
        .busy(busy),
        .done(done)
    );

    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk);
        cmd_valid = 1'b1;
        @(negedge clk);
        cmd_valid = 1'b0;
        cmd_vs1 = 4'd7;
        cmd_vs2 = 4'd8;
        wait(done);
        @(negedge clk);
        for (int lane = 0; lane < 32; lane++) begin
            if (rf_wr_data[lane * 16 +: 16] !== 16'h3400) begin
                $error("identity RoPE changed lane %0d: got %h",
                       lane, rf_wr_data[lane * 16 +: 16]);
                $fatal(1);
            end
        end
        $display("ROPE_IDENTITY_PASS lanes=32 value=0x3400");
        $finish;
    end
endmodule

`default_nettype wire
