`default_nettype none

module tb_supra2_final_head;
    import fp16_utils_pkg::*;

    localparam int M = 32;
    localparam int N = 32;
    localparam int TOKENS = 256;
    localparam int K = 576;
    localparam int OUT = 16;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic clear_acc = 1'b0;
    logic en = 1'b1;
    logic [5:0] active_rows = M;
    logic dtype_fp16 = 1'b1;
    logic signed [7:0] a_col [M];
    logic signed [7:0] b_row [N];
    logic signed [15:0] a_col_fp16 [M];
    logic signed [15:0] b_row_fp16 [N];
    logic signed [31:0] acc_out [M][N];
    logic acc_valid;

    logic [15:0] activation_mem [0:TOKENS*K-1];
    logic [15:0] weight_mem [0:OUT*K-1];
    logic [15:0] bias_mem [0:OUT-1];
    logic [15:0] expected_mem [0:TOKENS*OUT-1];
    logic [15:0] actual_mem [0:TOKENS*OUT-1];
    string activation_file, weight_file, bias_file, expected_file, actual_file;
    integer fd, status;
    integer token_base, t, i, j, source_k, token, k_index, out_index;
    integer mismatches;
    real got_value, expected_value, error_value, peak_value, max_abs_error;

    always #5 clk = ~clk;

    systolic_array #(.M(M), .N(N), .DATA_W(8), .ACC_W(32), .PIPE_STAGE(32))
    u_array (
        .clk(clk), .rst_n(rst_n), .clear_acc(clear_acc), .en(en),
        .a_unsigned(1'b0), .active_rows(active_rows),
        .a_col(a_col), .b_row(b_row),
        .acc_out(acc_out), .acc_valid(acc_valid)
    );

    function automatic real half_to_real(input logic [15:0] h);
        logic sign;
        integer exponent;
        integer mantissa;
        real value;
        begin
            sign = h[15];
            exponent = h[14:10];
            mantissa = h[9:0];
            if (exponent == 0)
                value = mantissa / 1024.0 * (2.0 ** -14);
            else if (exponent == 31)
                value = (mantissa == 0) ? 1.0e30 : 0.0;
            else
                value = (1.0 + mantissa / 1024.0) * (2.0 ** (exponent - 15));
            return sign ? -value : value;
        end
    endfunction

    initial begin
        if (!$value$plusargs("ACTIVATION_FILE=%s", activation_file) ||
            !$value$plusargs("WEIGHT_FILE=%s", weight_file) ||
            !$value$plusargs("BIAS_FILE=%s", bias_file) ||
            !$value$plusargs("EXPECTED_FILE=%s", expected_file) ||
            !$value$plusargs("ACTUAL_FILE=%s", actual_file))
            $fatal(1, "Missing final-head fixture plusargs");

        $readmemh(activation_file, activation_mem);
        $readmemh(weight_file, weight_mem);
        $readmemh(bias_file, bias_mem);
        $readmemh(expected_file, expected_mem);
        for (i = 0; i < M; i++) a_col_fp16[i] = '0;
        for (j = 0; j < N; j++) b_row_fp16[j] = '0;
        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        mismatches = 0;
        max_abs_error = 0.0;
        peak_value = 0.0;

        for (token_base = 0; token_base < TOKENS; token_base = token_base + M) begin
            clear_acc = 1'b1;
            @(negedge clk);
            clear_acc = 1'b0;
            for (t = 0; t < K + M + N + 3; t++) begin
                for (i = 0; i < M; i++) begin
                    source_k = t - i;
                    if (source_k >= 0 && source_k < K)
                        a_col[i] = fp16_to_int8_scaled(
                            activation_mem[(token_base+i)*K+source_k],32'h4203_3333);
                    else a_col[i] = '0;
                end
                for (j = 0; j < N; j++) begin
                    source_k = t - j;
                    if (j < OUT && source_k >= 0 && source_k < K)
                        b_row[j] = fp16_to_int8_scaled(weight_mem[j*K+source_k],32'h436a_0000);
                    else b_row[j] = '0;
                end
                @(negedge clk);
            end
            repeat (3) @(negedge clk);
            if (!acc_valid) $fatal(1, "FP16 array acc_valid missing at token %0d", token_base);

            for (i = 0; i < M; i++) begin
                token = token_base + i;
                for (j = 0; j < OUT; j++) begin
                    out_index = token*OUT+j;
                    k_index = fp32_add_rne(fp32_mul_rne(fp32_from_int32_rne(acc_out[i][j]),
                                           32'h39089e64),
                                           fp16_to_fp32(bias_mem[j]));
                    actual_mem[out_index] = fp32_to_fp16(k_index);
                    got_value = half_to_real(actual_mem[out_index]);
                    expected_value = half_to_real(expected_mem[out_index]);
                    error_value = (got_value >= expected_value)
                        ? got_value - expected_value : expected_value - got_value;
                    if (error_value > max_abs_error) max_abs_error = error_value;
                    if ((expected_value >= 0.0 ? expected_value : -expected_value) > peak_value)
                        peak_value = expected_value >= 0.0 ? expected_value : -expected_value;
                end
            end
        end

        fd = $fopen(actual_file, "w");
        if (fd == 0) $fatal(1, "Cannot open final-head output file");
        for (i = 0; i < TOKENS*OUT; i++) begin
            $fdisplay(fd, "%04h", actual_mem[i]);
            if (actual_mem[i] !== expected_mem[i]) mismatches++;
        end
        $fclose(fd);
        if (peak_value == 0.0 || max_abs_error/peak_value > 0.01)
            $fatal(1, "Final-head relative error %.6f exceeds 1%%", max_abs_error/peak_value);
        $display("SUPRA2_FINAL_HEAD_PASS tokens=%0d in=%0d out=%0d", TOKENS, K, OUT);
        $display("SUPRA2_FINAL_HEAD_MAX_ABS %.9e", max_abs_error);
        $display("SUPRA2_FINAL_HEAD_REL_TO_PEAK %.9e", max_abs_error/peak_value);
        $display("SUPRA2_FINAL_HEAD_FP16_BIT_MISMATCHES %0d", mismatches);
        $finish;
    end
endmodule

`default_nettype wire
