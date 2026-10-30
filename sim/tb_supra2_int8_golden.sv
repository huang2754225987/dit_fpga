`timescale 1ns/1ps
`default_nettype none

module tb_supra2_int8_golden;
    localparam int M = 32;
    localparam int N = 32;
    localparam int MAX_K = 2304;
    localparam int MAX_ELEMS = M * MAX_K;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic clear_acc = 1'b0;
    logic en = 1'b0;
    logic [5:0] active_rows = 6'd32;
    logic dtype_fp16 = 1'b0;
    logic signed [7:0] a_col [M];
    logic signed [7:0] b_row [N];
    logic signed [15:0] a_col_fp16 [M];
    logic signed [15:0] b_row_fp16 [N];
    logic signed [31:0] acc_out [M][N];
    logic acc_valid;

    logic signed [7:0] activation_mem [0:MAX_ELEMS-1];
    logic signed [7:0] weight_mem [0:MAX_ELEMS-1];
    logic signed [31:0] expected_mem [0:M*N-1];
    logic [31:0] token_scale_mem [0:M-1];
    logic [31:0] weight_scale_mem [0:N-1];
    logic [31:0] bias_mem [0:N-1];
    logic [31:0] output_absmax_mem [0:0];
    logic [15:0] expected_output_mem [0:M*N-1];
    logic [M*32-1:0] epilogue_acc_bus;
    logic [N*32-1:0] epilogue_weight_scale_bus, epilogue_bias_bus;
    logic [31:0] epilogue_token_scale;
    logic [N*16-1:0] epilogue_output_bus;

    string activation_file;
    string weight_file;
    string expected_file;
    string token_scale_file, weight_scale_file, bias_file;
    string expected_output_file, output_absmax_file;
    integer k_size;
    integer t, i, j, source_k;
    integer mismatches;
    integer plusarg_ok;
    integer array_feed_cycles;
    logic count_array_cycles;
    real output_max_error, output_reference_absmax, got_fp16, expected_fp16;

    always #5 clk = ~clk;
    always @(posedge clk) begin
        if (count_array_cycles)
            array_feed_cycles <= array_feed_cycles + 1;
    end

    systolic_array #(
        .M(M), .N(N), .DATA_W(8), .ACC_W(32), .PIPE_STAGE(32)
    ) dut (
        .clk(clk), .rst_n(rst_n), .clear_acc(clear_acc), .en(en),
        .a_unsigned(1'b0), .active_rows(active_rows),
        .a_col(a_col), .b_row(b_row),
        .acc_out(acc_out), .acc_valid(acc_valid)
    );

    supra2_qlinear_epilogue #(.LANES(N)) u_epilogue (
        .acc_int32(epilogue_acc_bus),
        .token_scale_fp32(epilogue_token_scale),
        .weight_scale_fp32(epilogue_weight_scale_bus),
        .bias_fp32(epilogue_bias_bus),
        .output_fp16(epilogue_output_bus)
    );

    function automatic real half_to_real(input logic [15:0] value);
        integer exponent;
        real significand, result;
        begin
            exponent = value[14:10];
            if (exponent == 0)
                significand = value[9:0] / 1024.0;
            else
                significand = 1.0 + value[9:0] / 1024.0;
            if (exponent == 0)
                result = significand * (2.0 ** -14);
            else if (exponent == 31)
                result = 0.0;
            else
                result = significand * (2.0 ** (exponent - 15));
            half_to_real = value[15] ? -result : result;
        end
    endfunction

    function automatic real single_to_real(input logic [31:0] value);
        integer exponent;
        real significand, result;
        begin
            exponent = value[30:23];
            if (exponent == 0)
                significand = value[22:0] / 8388608.0;
            else
                significand = 1.0 + value[22:0] / 8388608.0;
            if (exponent == 0)
                result = significand * (2.0 ** -126);
            else
                result = significand * (2.0 ** (exponent - 127));
            single_to_real = value[31] ? -result : result;
        end
    endfunction

    initial begin
        plusarg_ok = $value$plusargs("K=%d", k_size);
        if (!plusarg_ok || k_size < 1 || k_size > MAX_K) begin
            $fatal(1, "Provide K in [1,%0d]", MAX_K);
        end
        if (!$value$plusargs("X_FILE=%s", activation_file) ||
            !$value$plusargs("W_FILE=%s", weight_file) ||
            !$value$plusargs("GOLD_FILE=%s", expected_file) ||
            !$value$plusargs("TOKEN_SCALE_FILE=%s", token_scale_file) ||
            !$value$plusargs("WEIGHT_SCALE_FILE=%s", weight_scale_file) ||
            !$value$plusargs("BIAS_FILE=%s", bias_file) ||
            !$value$plusargs("OUTPUT_FILE=%s", expected_output_file) ||
            !$value$plusargs("OUTPUT_ABSMAX_FILE=%s", output_absmax_file)) begin
            $fatal(1, "Provide all activation, weight, accumulator and output files");
        end

        for (i = 0; i < MAX_ELEMS; i++) begin
            activation_mem[i] = '0;
            weight_mem[i] = '0;
        end
        $readmemh(activation_file, activation_mem, 0, M*k_size-1);
        $readmemh(weight_file, weight_mem, 0, N*k_size-1);
        $readmemh(expected_file, expected_mem);
        $readmemh(token_scale_file, token_scale_mem);
        $readmemh(weight_scale_file, weight_scale_mem);
        $readmemh(bias_file, bias_mem);
        $readmemh(expected_output_file, expected_output_mem);
        $readmemh(output_absmax_file, output_absmax_mem);
        output_reference_absmax = single_to_real(output_absmax_mem[0]);
        for (j = 0; j < N; j++) begin
            epilogue_weight_scale_bus[j*32 +: 32] = weight_scale_mem[j];
            epilogue_bias_bus[j*32 +: 32] = bias_mem[j];
        end
        epilogue_token_scale = 0;
        epilogue_acc_bus = 0;
        for (i = 0; i < M; i++) begin
            a_col[i] = '0;
            a_col_fp16[i] = '0;
        end
        for (j = 0; j < N; j++) begin
            b_row[j] = '0;
            b_row_fp16[j] = '0;
        end
        array_feed_cycles = 0;
        count_array_cycles = 1'b0;

        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        en = 1'b1;
        clear_acc = 1'b1;
        @(negedge clk);
        clear_acc = 1'b0;
        array_feed_cycles = 0;
        count_array_cycles = 1'b1;

        // Skew the input rows/columns to align each A[i,k] and B[k,j]
        // product at PE(i,j) on the same cycle.
        for (t = 0; t < k_size + M + N + 3; t++) begin
            for (i = 0; i < M; i++) begin
                source_k = t - i;
                a_col[i] = (source_k >= 0 && source_k < k_size)
                         ? activation_mem[i*k_size + source_k] : 8'sd0;
            end
            for (j = 0; j < N; j++) begin
                source_k = t - j;
                b_row[j] = (source_k >= 0 && source_k < k_size)
                         ? weight_mem[j*k_size + source_k] : 8'sd0;
            end
            @(negedge clk);
        end
        count_array_cycles = 1'b0;

        repeat (3) @(negedge clk);
        if (!acc_valid) $fatal(1, "systolic array did not assert acc_valid");
        mismatches = 0;
        for (i = 0; i < M; i++) begin
            for (j = 0; j < N; j++) begin
                if (acc_out[i][j] !== expected_mem[i*N+j]) begin
                    if (mismatches < 8)
                        $display("MISMATCH row=%0d col=%0d got=%0d expected=%0d",
                                 i, j, acc_out[i][j], expected_mem[i*N+j]);
                    mismatches++;
                end
            end
        end
        if (mismatches != 0)
            $fatal(1, "INT8 golden GEMM failed: %0d mismatches", mismatches);
        $display("SUPRA2_INT8_GOLDEN_PASS M=%0d N=%0d K=%0d", M, N, k_size);
        $display("SUPRA2_ARRAY_FEED_CYCLES %0d", array_feed_cycles);

        output_max_error = 0.0;
        for (i = 0; i < M; i++) begin
            epilogue_token_scale = token_scale_mem[i];
            for (j = 0; j < N; j++)
                epilogue_acc_bus[j*32 +: 32] = acc_out[i][j];
            #1;
            for (j = 0; j < N; j++) begin
                got_fp16 = half_to_real(epilogue_output_bus[j*16 +: 16]);
                expected_fp16 = half_to_real(expected_output_mem[i*N+j]);
                if ((got_fp16 >= expected_fp16
                     ? got_fp16 - expected_fp16
                     : expected_fp16 - got_fp16) > output_max_error)
                    output_max_error = (got_fp16 >= expected_fp16)
                        ? got_fp16 - expected_fp16
                        : expected_fp16 - got_fp16;
            end
        end
        if (output_max_error / output_reference_absmax >= 0.0045)
            $fatal(1, "QLinear output error too large: max_abs=%e rel=%e",
                   output_max_error,
                   output_max_error / output_reference_absmax);
        $display("SUPRA2_QLINEAR_GOLDEN_PASS max_abs=%e rel=%e",
                 output_max_error, output_max_error / output_reference_absmax);
        $finish;
    end
endmodule

`default_nettype wire
