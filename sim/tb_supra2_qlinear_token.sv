`timescale 1ns/1ps
`default_nettype none

module tb_supra2_qlinear_token;
    localparam int LANES = 32;
    localparam int MAX_K = 2304;
    localparam int ARRAY_M = 32;
    localparam int ARRAY_N = 32;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic hadamard_start = 1'b0;
    logic quant_start = 1'b0;
    logic [11:0] vector_k = '0;
    logic [8:0] hadamard_h = '0;
    logic hadamard_in_valid = 1'b0;
    logic hadamard_in_ready;
    logic [LANES*16-1:0] hadamard_in_fp16 = '0;
    logic hadamard_out_valid;
    logic [LANES*16-1:0] hadamard_out_fp16;
    logic hadamard_busy, hadamard_done;
    logic quant_in_ready;
    logic quant_out_valid;
    logic quant_out_ready = 1'b1;
    logic [LANES*8-1:0] quant_out_int8;
    logic [31:0] quant_scale_fp32;
    logic quant_busy, quant_done;
    logic hadamard_done_seen = 1'b0;

    logic clear_acc = 1'b0;
    logic array_en = 1'b1;
    logic [5:0] active_rows = 6'd1;
    logic dtype_fp16 = 1'b0;
    logic signed [7:0] a_col [ARRAY_M];
    logic signed [7:0] b_row [ARRAY_N];
    logic signed [15:0] a_col_fp16 [ARRAY_M];
    logic signed [15:0] b_row_fp16 [ARRAY_N];
    logic signed [31:0] acc_out [ARRAY_M][ARRAY_N];
    logic acc_valid;

    logic [ARRAY_N*32-1:0] epi_acc_bus, epi_ws_bus, epi_bias_bus;
    logic [31:0] epi_token_scale;
    logic [ARRAY_N*16-1:0] epi_output_bus;

    logic [15:0] input_mem [0:MAX_K-1];
    logic signed [7:0] quant_expected_mem [0:MAX_K-1];
    logic signed [7:0] quant_result_mem [0:MAX_K-1];
    logic signed [7:0] weight_mem [0:ARRAY_N*MAX_K-1];
    logic signed [31:0] acc_expected_mem [0:ARRAY_N-1];
    logic [31:0] ws_mem [0:ARRAY_N-1];
    logic [31:0] bias_mem [0:ARRAY_N-1];
    logic [15:0] compute_expected_mem [0:ARRAY_N-1];
    logic [15:0] package_output_mem [0:ARRAY_N-1];
    logic [31:0] compute_absmax_mem [0:0];
    logic [31:0] package_absmax_mem [0:0];

    string input_file, q_expected_file, weight_file, acc_expected_file;
    string ws_file, bias_file, compute_output_file, package_output_file;
    string compute_absmax_file, package_absmax_file;
    integer k_size, h_size, row_index, beat, lane, t, i, j, source_k;
    integer q_mismatches, acc_mismatches;
    integer count_cycles, total_cycles;
    logic count_cycles_enable;
    real compute_max_error, package_max_error;
    real compute_reference_absmax, package_reference_absmax;
    real got_value, expected_value, package_value, output_rel_error;

    always #5 clk = ~clk;
    always @(posedge clk) begin
        if (count_cycles_enable)
            total_cycles <= total_cycles + 1;
        if (hadamard_done)
            hadamard_done_seen <= 1'b1;
    end

    supra2_hadamard_transform #(
        .MAX_K(MAX_K), .MAX_H(256), .LANES(LANES), .K_W(12), .H_W(9)
    ) u_hadamard (
        .clk(clk), .rst_n(rst_n), .start(hadamard_start),
        .vector_k(vector_k), .block_size(hadamard_h),
        .in_valid(hadamard_in_valid), .in_ready(hadamard_in_ready),
        .in_fp16(hadamard_in_fp16), .out_valid(hadamard_out_valid),
        .out_ready(quant_in_ready), .out_fp16(hadamard_out_fp16),
        .busy(hadamard_busy), .done(hadamard_done)
    );

    supra2_activation_quantizer #(
        .MAX_K(MAX_K), .LANES(LANES), .K_W(12)
    ) u_quantizer (
        .clk(clk), .rst_n(rst_n), .start(quant_start), .vector_k(vector_k),
        .in_valid(hadamard_out_valid), .in_ready(quant_in_ready),
        .in_fp16(hadamard_out_fp16), .out_valid(quant_out_valid),
        .out_ready(quant_out_ready), .out_int8(quant_out_int8),
        .scale_fp32(quant_scale_fp32), .busy(quant_busy), .done(quant_done)
    );

    systolic_array #(
        .M(ARRAY_M), .N(ARRAY_N), .DATA_W(8), .ACC_W(32), .PIPE_STAGE(32)
    ) u_array (
        .clk(clk), .rst_n(rst_n), .clear_acc(clear_acc), .en(array_en),
        .a_unsigned(1'b0), .active_rows(active_rows),
        .a_col(a_col), .b_row(b_row),
        .acc_out(acc_out), .acc_valid(acc_valid)
    );

    supra2_qlinear_epilogue #(.LANES(ARRAY_N)) u_epilogue (
        .acc_int32(epi_acc_bus), .token_scale_fp32(epi_token_scale),
        .weight_scale_fp32(epi_ws_bus), .bias_fp32(epi_bias_bus),
        .output_fp16(epi_output_bus)
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
        if (!$value$plusargs("K=%d", k_size) ||
            !$value$plusargs("H=%d", h_size) ||
            !$value$plusargs("ROW=%d", row_index) ||
            k_size <= 0 || k_size > MAX_K || (k_size % LANES) != 0 ||
            !$value$plusargs("INPUT_FILE=%s", input_file) ||
            !$value$plusargs("Q_EXPECTED_FILE=%s", q_expected_file) ||
            !$value$plusargs("WEIGHT_FILE=%s", weight_file) ||
            !$value$plusargs("ACC_EXPECTED_FILE=%s", acc_expected_file) ||
            !$value$plusargs("WS_FILE=%s", ws_file) ||
            !$value$plusargs("BIAS_FILE=%s", bias_file) ||
            !$value$plusargs("COMPUTE_OUTPUT_FILE=%s", compute_output_file) ||
            !$value$plusargs("PACKAGE_OUTPUT_FILE=%s", package_output_file) ||
            !$value$plusargs("COMPUTE_ABSMAX_FILE=%s", compute_absmax_file) ||
            !$value$plusargs("PACKAGE_ABSMAX_FILE=%s", package_absmax_file))
            $fatal(1, "Missing QLinear token fixture arguments");

        $readmemh(input_file, input_mem, 0, k_size-1);
        $readmemh(q_expected_file, quant_expected_mem, 0, k_size-1);
        $readmemh(weight_file, weight_mem, 0, ARRAY_N*k_size-1);
        $readmemh(acc_expected_file, acc_expected_mem);
        $readmemh(ws_file, ws_mem);
        $readmemh(bias_file, bias_mem);
        $readmemh(compute_output_file, compute_expected_mem);
        $readmemh(package_output_file, package_output_mem);
        $readmemh(compute_absmax_file, compute_absmax_mem);
        $readmemh(package_absmax_file, package_absmax_mem);
        compute_reference_absmax = single_to_real(compute_absmax_mem[0]);
        package_reference_absmax = single_to_real(package_absmax_mem[0]);

        for (i = 0; i < ARRAY_M; i++) begin
            a_col[i] = '0;
            a_col_fp16[i] = '0;
        end
        for (j = 0; j < ARRAY_N; j++) begin
            b_row[j] = '0;
            b_row_fp16[j] = '0;
            epi_ws_bus[j*32 +: 32] = ws_mem[j];
            epi_bias_bus[j*32 +: 32] = bias_mem[j];
        end
        quant_result_mem = '{default: '0};
        count_cycles_enable = 1'b0;
        total_cycles = 0;
        q_mismatches = 0;
        acc_mismatches = 0;

        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk);
        vector_k = k_size;
        count_cycles_enable = 1'b1;
        quant_start = 1'b1;
        hadamard_start = 1'b1;
        hadamard_h = h_size;
        @(negedge clk);
        quant_start = 1'b0;
        hadamard_start = 1'b0;
        hadamard_in_valid = 1'b1;
        for (beat = 0; beat < k_size/LANES; beat++) begin
            for (lane = 0; lane < LANES; lane++)
                hadamard_in_fp16[lane*16 +: 16] = input_mem[beat*LANES+lane];
            @(posedge clk);
            while (!hadamard_in_ready) @(posedge clk);
            @(negedge clk);
        end
        hadamard_in_valid = 1'b0;
        hadamard_in_fp16 = '0;

        beat = 0;
        while (beat < k_size/LANES) begin
            if (quant_out_valid) begin
                if (beat == 0)
                    epi_token_scale = quant_scale_fp32;
                for (lane = 0; lane < LANES; lane++) begin
                    quant_result_mem[beat*LANES+lane] =
                        quant_out_int8[lane*8 +: 8];
                    if (quant_out_int8[lane*8 +: 8] !==
                        quant_expected_mem[beat*LANES+lane]) begin
                        if (q_mismatches < 8)
                            $display("Q_MISMATCH index=%0d got=%h expected=%h",
                                     beat*LANES+lane,
                                     quant_out_int8[lane*8 +: 8],
                                     quant_expected_mem[beat*LANES+lane]);
                        q_mismatches++;
                    end
                end
                beat++;
            end
            @(negedge clk);
        end
        if (q_mismatches != 0)
            $fatal(1, "RTL activation quantization had %0d mismatches", q_mismatches);
        if (hadamard_busy || !hadamard_done_seen)
            $fatal(1, "Hadamard completion handshake missing");

        clear_acc = 1'b1;
        @(negedge clk);
        clear_acc = 1'b0;
        for (t = 0; t < k_size + ARRAY_M + ARRAY_N + 3; t++) begin
            for (i = 0; i < ARRAY_M; i++) begin
                source_k = t - i;
                if (i == 0 && source_k >= 0 && source_k < k_size)
                    a_col[i] = quant_result_mem[source_k];
                else
                    a_col[i] = 0;
            end
            for (j = 0; j < ARRAY_N; j++) begin
                source_k = t - j;
                b_row[j] = (source_k >= 0 && source_k < k_size)
                         ? weight_mem[j*k_size+source_k] : 8'sd0;
            end
            @(negedge clk);
        end
        repeat (3) @(negedge clk);
        if (!acc_valid)
            $fatal(1, "systolic array did not assert acc_valid");
        for (j = 0; j < ARRAY_N; j++) begin
            if (acc_out[0][j] !== acc_expected_mem[j]) begin
                if (acc_mismatches < 8)
                    $display("ACC_MISMATCH col=%0d got=%0d expected=%0d",
                             j, acc_out[0][j], acc_expected_mem[j]);
                acc_mismatches++;
            end
            epi_acc_bus[j*32 +: 32] = acc_out[0][j];
        end
        if (acc_mismatches != 0)
            $fatal(1, "QLinear INT32 array result had %0d mismatches", acc_mismatches);

        #1;
        compute_max_error = 0.0;
        package_max_error = 0.0;
        for (j = 0; j < ARRAY_N; j++) begin
            got_value = half_to_real(epi_output_bus[j*16 +: 16]);
            expected_value = half_to_real(compute_expected_mem[j]);
            package_value = half_to_real(package_output_mem[j]);
            if ((got_value >= expected_value ? got_value - expected_value
                                              : expected_value - got_value) > compute_max_error)
                compute_max_error = (got_value >= expected_value)
                    ? got_value - expected_value : expected_value - got_value;
            if ((got_value >= package_value ? got_value - package_value
                                             : package_value - got_value) > package_max_error)
                package_max_error = (got_value >= package_value)
                    ? got_value - package_value : package_value - got_value;
        end
        output_rel_error = compute_max_error /
            ((compute_reference_absmax > 1.0e-20) ? compute_reference_absmax : 1.0);
        if (output_rel_error >= 0.0045)
            $fatal(1, "Integrated QLinear differs from RTL-chain reference: %e",
                   output_rel_error);
        count_cycles_enable = 1'b0;
        $display("SUPRA2_QTOKEN_PASS layer_row=%0d K=%0d H=%0d N=32",
                 row_index, k_size, h_size);
        $display("SUPRA2_QTOKEN_CHAIN_CYCLES %0d", total_cycles);
        $display("SUPRA2_QTOKEN_REFERENCE_REL %e", output_rel_error);
        $display("SUPRA2_QTOKEN_PACKAGE_REL %e",
                 package_max_error / package_reference_absmax);
        $finish;
    end
endmodule

`default_nettype wire
