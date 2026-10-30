`default_nettype none

module tb_supra2_attention_qk #(
    parameter int TOKENS=256,
    parameter int KEYS=256,
    parameter real REL_PEAK_GATE=0.001,
    parameter bit RUNTIME_SIGNED_A=1'b0
);
    localparam int M = 32, N = 32, K = 64, HEADS = 9;
    localparam int Q_COUNT = TOKENS*HEADS*K;
    localparam int K_COUNT = KEYS*HEADS*K;
    localparam int SCORE_COUNT = TOKENS*HEADS*KEYS;
    logic clk = 1'b0, rst_n = 1'b0;
    logic clear_acc = 1'b0, array_en = 1'b1, dtype_fp16 = 1'b1;
    logic [5:0] active_rows = M;
    logic signed [7:0] a_col [M], b_row [N];
    logic signed [15:0] a_col_fp16 [M], b_row_fp16 [N];
    logic signed [31:0] acc_out [M][N];
    logic acc_valid;
    logic [15:0] q_mem [0:Q_COUNT-1], k_mem [0:K_COUNT-1];
    logic [15:0] expected_mem [0:SCORE_COUNT-1];
    logic [15:0] actual_mem [0:SCORE_COUNT-1];
    string q_file, k_file, expected_file, actual_file;
    integer head, query_tile, key_tile, t, i, j, source_k, row, key, flat;
    integer cycles, mismatches, output_fd;
    real got, expected, abs_error, max_error, max_ref;

    always #5 clk = ~clk;
    systolic_array #(.M(M), .N(N), .DATA_W(8), .ACC_W(32),
                     .RUNTIME_UNSIGNED_A(RUNTIME_SIGNED_A), .PIPE_STAGE(32)) dut (
        .clk, .rst_n, .clear_acc, .en(array_en), .a_unsigned(1'b0), .active_rows,
        .a_col, .b_row,
        .acc_out, .acc_valid
    );

    function automatic real half_to_real(input logic [15:0] value);
        integer exponent;
        real significand;
        begin
            exponent = value[14:10];
            if (exponent == 0) significand = value[9:0] / 1024.0;
            else significand = 1.0 + value[9:0] / 1024.0;
            half_to_real = (value[15] ? -1.0 : 1.0) * significand *
                (exponent == 0 ? (2.0 ** -14) : (2.0 ** (exponent - 15)));
        end
    endfunction

    always @(posedge clk) if (array_en) cycles <= cycles + 1;

    initial begin
        if (!$value$plusargs("Q_FILE=%s", q_file) ||
            !$value$plusargs("K_FILE=%s", k_file) ||
            !$value$plusargs("EXPECTED_FILE=%s", expected_file) ||
            !$value$plusargs("ACTUAL_FILE=%s", actual_file))
            $fatal(1, "Missing QK fixture plusargs");
        $readmemh(q_file, q_mem);
        $readmemh(k_file, k_mem);
        $readmemh(expected_file, expected_mem);
        output_fd = $fopen(actual_file, "w");
        if (!output_fd) $fatal(1, "Cannot open RTL QK score output");
        cycles = 0;
        mismatches = 0;
        max_error = 0.0;
        max_ref = 0.0;
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        for (head = 0; head < HEADS; head++) begin
            for (query_tile = 0; query_tile < TOKENS/M; query_tile++) begin
                for (key_tile = 0; key_tile < KEYS/N; key_tile++) begin
                    clear_acc = 1'b1;
                    @(negedge clk);
                    clear_acc = 1'b0;
                    for (t = 0; t < K+M+N+3; t++) begin
                        for (i = 0; i < M; i++) begin
                            source_k = t-i;
                            row = query_tile*M+i;
                            a_col[i] = (source_k >= 0 && source_k < K)
                                ? fp16_utils_pkg::fp16_to_int8_q(
                                    q_mem[(head*TOKENS+row)*K+source_k],4) : 8'sd0;
                        end
                        for (j = 0; j < N; j++) begin
                            source_k = t-j;
                            key = key_tile*N+j;
                            b_row[j] = (source_k >= 0 && source_k < K)
                                ? fp16_utils_pkg::fp16_to_int8_q(
                                    k_mem[(head*KEYS+key)*K+source_k],4) : 8'sd0;
                        end
                        @(negedge clk);
                    end
                    repeat (3) @(negedge clk);
                    if (!acc_valid) $fatal(1, "QK tile did not assert acc_valid");
                    for (i = 0; i < M; i++) begin
                        row = query_tile*M+i;
                        for (j = 0; j < N; j++) begin
                            key = key_tile*N+j;
                            flat = (row*HEADS+head)*KEYS+key;
                            got = half_to_real(fp16_utils_pkg::fp32_to_fp16(
                                fp16_utils_pkg::fp32_mul_rne(
                                    fp16_utils_pkg::fp32_from_int32_rne(acc_out[i][j]),
                                    32'h3a000000)));
                            expected = half_to_real(expected_mem[flat]);
                            abs_error = (got >= expected) ? got-expected : expected-got;
                            if (abs_error > max_error) max_error = abs_error;
                            if ((expected >= 0 ? expected : -expected) > max_ref)
                                max_ref = (expected >= 0 ? expected : -expected);
                            if (abs_error > 0.005) mismatches++;
                            actual_mem[flat] = fp16_utils_pkg::fp32_to_fp16(
                                fp16_utils_pkg::fp32_mul_rne(
                                    fp16_utils_pkg::fp32_from_int32_rne(acc_out[i][j]),
                                    32'h3a000000));
                        end
                    end
                end
            end
        end
        for (flat = 0; flat < SCORE_COUNT; flat++)
            $fdisplay(output_fd, "%04h", actual_mem[flat]);
        $fclose(output_fd);
        $display("SUPRA2_ATTN_QK_PASS queries=%0d heads=%0d keys=%0d cycles=%0d",
                 TOKENS, HEADS, KEYS, cycles);
        $display("SUPRA2_ATTN_QK_MAX_ABS %.9g", max_error);
        $display("SUPRA2_ATTN_QK_REL_TO_MAX %.9g", max_error/max_ref);
        $display("SUPRA2_ATTN_QK_TOL_MISMATCHES %0d", mismatches);
        if (max_error/max_ref > REL_PEAK_GATE)
            $fatal(1, "QK scores exceeded relative-to-peak error gate %.9g", REL_PEAK_GATE);
        $finish;
    end
endmodule

`default_nettype wire
