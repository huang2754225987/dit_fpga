`timescale 1ns/1ps
`default_nettype none

module tb_systolic_array_16x32;
    localparam int ROWS = 16;
    localparam int COLS = 32;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic clear_acc = 1'b0;
    logic en = 1'b0;
    logic [5:0] active_rows = ROWS;
    logic signed [7:0] a_col [ROWS];
    logic signed [7:0] b_row [COLS];
    logic signed [31:0] acc_out [ROWS][COLS];
    logic acc_valid;

    always #5 clk = ~clk;

    systolic_array #(
        .M(ROWS), .N(COLS), .DATA_W(8), .ACC_W(32),
        .UNSIGNED_A(1'b1), .PIPE_STAGE(32)
    ) dut (
        .clk(clk), .rst_n(rst_n), .clear_acc(clear_acc), .en(en),
        .a_unsigned(1'b0), .active_rows(active_rows), .a_col(a_col), .b_row(b_row),
        .acc_out(acc_out), .acc_valid(acc_valid)
    );

    task automatic run_gemm(input integer k_len);
        integer source_k;
        integer cycles;
        integer expected;
        begin
            @(negedge clk); clear_acc = 1'b1;
            @(negedge clk); clear_acc = 1'b0; en = 1'b1;
            cycles = k_len + ROWS + COLS + 2;
            for (int t = 0; t < cycles; t++) begin
                @(negedge clk);
                for (int r = 0; r < ROWS; r++) begin
                    source_k = t - r;
                    a_col[r] = (source_k >= 0 && source_k < k_len) ? 8'(r + 1) : 8'sd0;
                end
                for (int c = 0; c < COLS; c++) begin
                    source_k = t - c;
                    b_row[c] = (source_k >= 0 && source_k < k_len) ? 8'(c - 10) : 8'sd0;
                end
                @(posedge clk);
            end
            @(negedge clk); en = 1'b0;
            repeat (2) @(posedge clk);
            if (!acc_valid) $fatal(1, "acc_valid missing for K=%0d", k_len);
            for (int r = 0; r < ROWS; r++) begin
                for (int c = 0; c < COLS; c++) begin
                    expected = (r + 1) * (c - 10) * k_len;
                    if (acc_out[r][c] !== expected)
                        $fatal(1, "INT8 array mismatch K=%0d row=%0d col=%0d got=%0d expected=%0d",
                               k_len, r, c, acc_out[r][c], expected);
                end
            end
            $display("SUPRA2_ARRAY_16X32_PASS K=%0d feed_cycles=%0d MACs=%0d",
                     k_len, cycles, ROWS*COLS*k_len);
        end
    endtask

    initial begin
        for (int r = 0; r < ROWS; r++) a_col[r] = '0;
        for (int c = 0; c < COLS; c++) b_row[c] = '0;
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        run_gemm(576);
        run_gemm(2304);
        $display("SUPRA2_ARRAY_16X32_ALL_PASS");
        $finish;
    end

    initial begin
        #100000;
        $fatal(1, "16x32 systolic array test timeout");
    end
endmodule

`default_nettype wire
