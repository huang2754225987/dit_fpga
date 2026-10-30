`timescale 1ns/1ps
`default_nettype none

module tb_supra2_shared_array_modes;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic clear_acc = 1'b0;
    logic en = 1'b0;
    logic a_unsigned = 1'b0;
    logic signed [7:0] a_col [1];
    logic signed [7:0] b_row [1];
    wire signed [31:0] acc_out [1][1];
    wire acc_valid;

    always #5 clk = ~clk;

    systolic_array #(
        .M(1), .N(1), .DATA_W(8), .ACC_W(32),
        .RUNTIME_UNSIGNED_A(1'b1), .PIPE_STAGE(32)
    ) dut (
        .clk(clk), .rst_n(rst_n), .clear_acc(clear_acc), .en(en),
        .a_unsigned(a_unsigned), .active_rows(6'd1),
        .a_col(a_col), .b_row(b_row), .acc_out(acc_out), .acc_valid(acc_valid)
    );

    task automatic run_case(
        input logic unsigned_mode,
        input logic signed [7:0] a_value,
        input logic signed [7:0] b_value,
        input logic signed [31:0] expected
    );
        begin
            @(negedge clk);
            en = 1'b0;
            clear_acc = 1'b1;
            @(negedge clk);
            clear_acc = 1'b0;
            a_unsigned = unsigned_mode;
            a_col[0] = a_value;
            b_row[0] = b_value;
            en = 1'b1;
            repeat (2) @(negedge clk);
            en = 1'b0;
            repeat (2) @(negedge clk);
            if (acc_out[0][0] !== expected)
                $fatal(1, "signed-mode mismatch unsigned=%0d got=%0d expected=%0d",
                    unsigned_mode, acc_out[0][0], expected);
        end
    endtask

    initial begin
        a_col[0] = '0;
        b_row[0] = '0;
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        run_case(1'b0, -8'sd1, 8'sd3, -32'sd6);
        run_case(1'b1, -8'sd1, 8'sd3, 32'sd1530);
        $display("SUPRA2_SHARED_ARRAY_MODES_PASS signed=-6 unsigned=1530");
        $finish;
    end

    initial begin
        #1000;
        $fatal(1, "Shared-array mode test timeout");
    end
endmodule

`default_nettype wire
