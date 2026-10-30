`default_nettype none

module tb_supra2_qlinear_projection;
    localparam int LANES = 32;
    localparam int ARRAY_M = 32;
    localparam int ARRAY_N = 32;
`ifdef SUPRA2_XSIM_QKV_BATCH
    localparam int MAX_M = 32;
    localparam int MAX_N = 1728;
    localparam int MAX_K = 576;
`elsif SUPRA2_XSIM_XEMBED
    localparam int MAX_M = 256;
    localparam int MAX_N = 576;
    // The frontend consumes a full 32-lane beat even when x_embed has K=16.
    localparam int MAX_K = 32;
`else
    localparam int MAX_M = 256;
    localparam int MAX_N = 2304;
    localparam int MAX_K = 2304;
`endif
    localparam int MAX_H = (MAX_K < 256) ? MAX_K : 256;
    localparam int MAX_ACTIVATIONS = MAX_M * MAX_K;
    localparam int MAX_WEIGHTS = MAX_N * MAX_K;
    localparam int MAX_OUTPUTS = MAX_M * MAX_N;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic clear_acc = 1'b0;
    logic array_en = 1'b1;
    logic [5:0] active_rows = ARRAY_M;
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

    logic frontend_hadamard_start = 1'b0;
    logic frontend_quant_start = 1'b0;
    logic [11:0] frontend_k = '0;
    logic [8:0] frontend_h = '0;
    logic frontend_input_valid = 1'b0;
    logic frontend_input_ready;
    logic [LANES*16-1:0] frontend_input_fp16 = '0;
    logic frontend_rotated_valid;
    logic [LANES*16-1:0] frontend_rotated_fp16;
    logic frontend_quant_input_ready;
    logic frontend_quant_output_valid;
    logic [LANES*8-1:0] frontend_quantized;
    logic [31:0] frontend_scale_fp32;
    logic frontend_hadamard_busy, frontend_hadamard_done;
    logic frontend_quant_busy, frontend_quant_done;

    logic [7:0] activation_mem [0:MAX_ACTIVATIONS-1];
    logic [7:0] expected_quant_mem [0:MAX_ACTIVATIONS-1];
    logic [15:0] raw_input_mem [0:MAX_ACTIVATIONS-1];
    logic [15:0] expected_rotated_mem [0:MAX_ACTIVATIONS-1];
    logic [7:0] weight_mem [0:MAX_WEIGHTS-1];
    logic [31:0] expected_acc_mem [0:MAX_OUTPUTS-1];
    logic [15:0] expected_output_mem [0:MAX_OUTPUTS-1];
    logic [15:0] reference_output_mem [0:MAX_OUTPUTS-1];
    logic [15:0] actual_output_mem [0:MAX_OUTPUTS-1];
    logic [31:0] token_scale_mem [0:MAX_M-1];
    logic [31:0] expected_token_scale_mem [0:MAX_M-1];
    logic [31:0] weight_scale_mem [0:MAX_N-1];
    logic [31:0] bias_mem [0:MAX_N-1];

    string activation_file, raw_input_file, expected_quant_file;
    string expected_rotated_file;
    string weight_file, acc_file, output_file;
    string reference_output_file;
    string actual_output_file;
    integer actual_output_fd, dump_actual_output;
    string token_scale_file, weight_scale_file, bias_file;
    integer m_size, n_size, k_size, h_size;
    integer mi, ni, t, i, j, beat, lane, source_k, row_idx, col_idx, flat_idx;
    integer total_cycles, acc_mismatches, output_mismatches, chain_mismatches;
    integer quant_mismatches, frontend_mode, has_activation_file;
    integer hadamard_mismatches, hadamard_vector_index, current_frontend_row;
    logic capture_hadamard_output;
    integer count_cycles;
    real got_value, expected_value, abs_error, max_abs_error, chain_abs_error;
    real max_chain_error, chain_relative_error;
    real output_absmax, relative_error;

    always #5 clk = ~clk;
    initial begin
        repeat (2000000) @(posedge clk);
        $fatal(1, "Full QLinear projection regression timeout");
    end
    always @(posedge clk) begin
        if (count_cycles) total_cycles <= total_cycles + 1;
        if (capture_hadamard_output && frontend_rotated_valid &&
            frontend_quant_input_ready) begin
            for (int h_lane = 0; h_lane < LANES; h_lane++) begin
                if (hadamard_vector_index+h_lane < k_size &&
                    frontend_rotated_fp16[h_lane*16 +: 16] !==
                    expected_rotated_mem[current_frontend_row*k_size+
                                         hadamard_vector_index+h_lane]) begin
                    if (hadamard_mismatches < 8)
                        $display("FRONTEND_H_MISMATCH row=%0d k=%0d got=%h expected=%h",
                            current_frontend_row, hadamard_vector_index+h_lane,
                            frontend_rotated_fp16[h_lane*16 +: 16],
                            expected_rotated_mem[current_frontend_row*k_size+
                                                 hadamard_vector_index+h_lane]);
                    hadamard_mismatches++;
                end
                if (hadamard_vector_index+h_lane >= k_size &&
                    frontend_rotated_fp16[h_lane*16 +: 16] !== 16'b0)
                    hadamard_mismatches++;
            end
            hadamard_vector_index = hadamard_vector_index + LANES;
        end
    end

    supra2_hadamard_transform #(
        .MAX_K(MAX_K), .MAX_H(MAX_H), .LANES(LANES), .K_W(12), .H_W(9)
    ) u_hadamard (
        .clk(clk), .rst_n(rst_n), .start(frontend_hadamard_start),
        .vector_k(frontend_k), .block_size(frontend_h),
        .in_valid(frontend_input_valid), .in_ready(frontend_input_ready),
        .in_fp16(frontend_input_fp16), .out_valid(frontend_rotated_valid),
        .out_ready(frontend_quant_input_ready),
        .out_fp16(frontend_rotated_fp16),
        .busy(frontend_hadamard_busy), .done(frontend_hadamard_done)
    );

    supra2_activation_quantizer #(
        .MAX_K(MAX_K), .LANES(LANES), .K_W(12)
    ) u_activation_quantizer (
        .clk(clk), .rst_n(rst_n), .start(frontend_quant_start),
        .vector_k(frontend_k), .in_valid(frontend_rotated_valid),
        .in_ready(frontend_quant_input_ready), .in_fp16(frontend_rotated_fp16),
        .out_valid(frontend_quant_output_valid), .out_ready(1'b1),
        .out_int8(frontend_quantized), .scale_fp32(frontend_scale_fp32),
        .busy(frontend_quant_busy), .done(frontend_quant_done)
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
            if (exponent == 0) significand = value[9:0] / 1024.0;
            else significand = 1.0 + value[9:0] / 1024.0;
            if (exponent == 0) result = significand * (2.0 ** -14);
            else if (exponent == 31) result = 0.0;
            else result = significand * (2.0 ** (exponent - 15));
            half_to_real = value[15] ? -result : result;
        end
    endfunction

    initial begin
        if (!$value$plusargs("M=%d", m_size) ||
            !$value$plusargs("N=%d", n_size) ||
            !$value$plusargs("K=%d", k_size) ||
            !$value$plusargs("WEIGHT_FILE=%s", weight_file) ||
            !$value$plusargs("ACC_FILE=%s", acc_file) ||
            !$value$plusargs("OUTPUT_FILE=%s", output_file) ||
            !$value$plusargs("REFERENCE_OUTPUT_FILE=%s", reference_output_file) ||
            !$value$plusargs("TOKEN_SCALE_FILE=%s", token_scale_file) ||
            !$value$plusargs("WEIGHT_SCALE_FILE=%s", weight_scale_file) ||
            !$value$plusargs("BIAS_FILE=%s", bias_file))
            $fatal(1, "Missing full-projection fixture arguments");
        has_activation_file = $value$plusargs("ACTIVATION_FILE=%s", activation_file);
        frontend_mode = $value$plusargs("RAW_INPUT_FILE=%s", raw_input_file);
        if (frontend_mode) begin
            if (!$value$plusargs("EXPECTED_Q_FILE=%s", expected_quant_file) ||
                !$value$plusargs("EXPECTED_ROTATED_FILE=%s", expected_rotated_file) ||
                !$value$plusargs("H=%d", h_size))
                $fatal(1, "Frontend mode requires expected Q/Hadamard files and H");
        end else if (!has_activation_file) begin
            $fatal(1, "Provide ACTIVATION_FILE or RAW_INPUT_FILE");
        end
        if (m_size <= 0 || m_size > MAX_M || n_size <= 0 || n_size > MAX_N ||
            k_size <= 0 || k_size > MAX_K ||
            (m_size % ARRAY_M) != 0 || (n_size % ARRAY_N) != 0)
            $fatal(1, "Projection dimensions must fit the configured tile limits");

        if (frontend_mode) begin
            $readmemh(raw_input_file, raw_input_mem, 0, m_size*k_size-1);
            $readmemh(expected_quant_file, expected_quant_mem, 0, m_size*k_size-1);
            $readmemh(expected_rotated_file, expected_rotated_mem, 0, m_size*k_size-1);
        end else begin
            $readmemh(activation_file, activation_mem, 0, m_size*k_size-1);
        end
        $readmemh(weight_file, weight_mem, 0, n_size*k_size-1);
        $readmemh(acc_file, expected_acc_mem, 0, m_size*n_size-1);
        $readmemh(output_file, expected_output_mem, 0, m_size*n_size-1);
        $readmemh(reference_output_file, reference_output_mem, 0, m_size*n_size-1);
        dump_actual_output = 0;
        if ($value$plusargs("ACTUAL_OUTPUT_FILE=%s", actual_output_file)) begin
            actual_output_fd = $fopen(actual_output_file, "w");
            if (!actual_output_fd) $fatal(1, "Cannot open actual QLinear output file");
            dump_actual_output = 1;
        end
        $readmemh(token_scale_file, expected_token_scale_mem, 0, m_size-1);
        if (!frontend_mode)
            for (i = 0; i < m_size; i++)
                token_scale_mem[i] = expected_token_scale_mem[i];
        $readmemh(weight_scale_file, weight_scale_mem, 0, n_size-1);
        $readmemh(bias_file, bias_mem, 0, n_size-1);

        for (i = 0; i < ARRAY_M; i++) begin
            a_col[i] = '0;
            a_col_fp16[i] = '0;
        end
        for (j = 0; j < ARRAY_N; j++) begin
            b_row[j] = '0;
            b_row_fp16[j] = '0;
        end
        count_cycles = 0;
        total_cycles = 0;
        acc_mismatches = 0;
        output_mismatches = 0;
        chain_mismatches = 0;
        quant_mismatches = 0;
        hadamard_mismatches = 0;
        hadamard_vector_index = 0;
        current_frontend_row = 0;
        capture_hadamard_output = 1'b0;
        max_abs_error = 0.0;
        max_chain_error = 0.0;
        output_absmax = 0.0;

        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        count_cycles = 1;

        if (frontend_mode) begin
            frontend_k = k_size;
            frontend_h = h_size;
            for (row_idx = 0; row_idx < m_size; row_idx++) begin
                current_frontend_row = row_idx;
                hadamard_vector_index = 0;
                capture_hadamard_output = 1'b1;
                frontend_hadamard_start = 1'b1;
                frontend_quant_start = 1'b1;
                @(negedge clk);
                frontend_hadamard_start = 1'b0;
                frontend_quant_start = 1'b0;
                frontend_input_valid = 1'b1;
                for (beat = 0; beat < (k_size+LANES-1)/LANES; beat++) begin
                    for (lane = 0; lane < LANES; lane++) begin
                        if (beat*LANES+lane < k_size)
                            frontend_input_fp16[lane*16 +: 16] =
                                raw_input_mem[row_idx*k_size + beat*LANES + lane];
                        else frontend_input_fp16[lane*16 +: 16] = 16'b0;
                    end
                    @(posedge clk);
                    while (!frontend_input_ready) @(posedge clk);
                    @(negedge clk);
                end
                frontend_input_valid = 1'b0;
                frontend_input_fp16 = '0;
                beat = 0;
                while (beat < (k_size+LANES-1)/LANES) begin
                    if (frontend_quant_output_valid) begin
                        if (beat == 0) begin
                            token_scale_mem[row_idx] = frontend_scale_fp32;
                            if (frontend_scale_fp32 !==
                                expected_token_scale_mem[row_idx])
                                $fatal(1, "Token scale mismatch at row %0d",
                                       row_idx);
                        end
                        for (lane = 0; lane < LANES; lane++) begin
                            if (beat*LANES+lane < k_size) begin
                                activation_mem[row_idx*k_size + beat*LANES + lane] =
                                    frontend_quantized[lane*8 +: 8];
                            end
                            if (beat*LANES+lane < k_size &&
                                frontend_quantized[lane*8 +: 8] !==
                                expected_quant_mem[row_idx*k_size + beat*LANES + lane]) begin
                                if (quant_mismatches < 8)
                                    $display("FRONTEND_Q_MISMATCH row=%0d k=%0d got=%h expected=%h",
                                        row_idx, beat*LANES+lane,
                                        frontend_quantized[lane*8 +: 8],
                                        expected_quant_mem[row_idx*k_size + beat*LANES + lane]);
                                quant_mismatches++;
                            end
                            if (beat*LANES+lane >= k_size &&
                                frontend_quantized[lane*8 +: 8] !== 8'b0)
                                quant_mismatches++;
                        end
                        beat++;
                    end
                    @(negedge clk);
                end
                if (frontend_hadamard_busy || frontend_quant_busy)
                    $fatal(1, "Frontend did not return idle after row %0d", row_idx);
                capture_hadamard_output = 1'b0;
                if (hadamard_vector_index !=
                    ((k_size+LANES-1)/LANES)*LANES)
                    $fatal(1, "Hadamard emitted %0d padded elements for K=%0d row %0d",
                           hadamard_vector_index, k_size, row_idx);
            end
            if (hadamard_mismatches != 0)
                $fatal(1, "Hadamard produced %0d FP16 mismatches", hadamard_mismatches);
            if (quant_mismatches != 0)
                $fatal(1, "Frontend produced %0d INT8 mismatches", quant_mismatches);
        end

        for (mi = 0; mi < m_size; mi = mi + ARRAY_M) begin
            for (ni = 0; ni < n_size; ni = ni + ARRAY_N) begin
                clear_acc = 1'b1;
                @(negedge clk);
                clear_acc = 1'b0;
                for (t = 0; t < k_size + ARRAY_M + ARRAY_N + 3; t++) begin
                    for (i = 0; i < ARRAY_M; i++) begin
                        source_k = t - i;
                        if (source_k >= 0 && source_k < k_size)
                            a_col[i] = $signed(activation_mem[
                                (mi+i)*k_size + source_k]);
                        else a_col[i] = 0;
                    end
                    for (j = 0; j < ARRAY_N; j++) begin
                        source_k = t - j;
                        if (source_k >= 0 && source_k < k_size)
                            b_row[j] = $signed(weight_mem[
                                (ni+j)*k_size + source_k]);
                        else b_row[j] = 0;
                    end
                    @(negedge clk);
                end
                repeat (3) @(negedge clk);
                if (!acc_valid)
                    $fatal(1, "acc_valid missing for tile M=%0d N=%0d", mi, ni);

                for (i = 0; i < ARRAY_M; i++) begin
                    row_idx = mi + i;
                    epi_token_scale = token_scale_mem[row_idx];
                    for (j = 0; j < ARRAY_N; j++) begin
                        col_idx = ni + j;
                        flat_idx = row_idx*n_size + col_idx;
                        if (acc_out[i][j] !== $signed(expected_acc_mem[flat_idx])) begin
                            if (acc_mismatches < 8)
                                $display("ACC_MISMATCH m=%0d n=%0d got=%0d expected=%0d",
                                         row_idx, col_idx, acc_out[i][j],
                                         $signed(expected_acc_mem[flat_idx]));
                            acc_mismatches++;
                        end
                        epi_acc_bus[j*32 +: 32] = acc_out[i][j];
                        epi_ws_bus[j*32 +: 32] = weight_scale_mem[col_idx];
                        epi_bias_bus[j*32 +: 32] = bias_mem[col_idx];
                    end
                    #1;
                    for (j = 0; j < ARRAY_N; j++) begin
                        col_idx = ni + j;
                        flat_idx = row_idx*n_size + col_idx;
                        got_value = half_to_real(epi_output_bus[j*16 +: 16]);
                        if (dump_actual_output)
                            actual_output_mem[flat_idx] = epi_output_bus[j*16 +: 16];
                        expected_value = half_to_real(expected_output_mem[flat_idx]);
                        chain_abs_error = (got_value >=
                            half_to_real(reference_output_mem[flat_idx]))
                            ? got_value - half_to_real(reference_output_mem[flat_idx])
                            : half_to_real(reference_output_mem[flat_idx]) - got_value;
                        if (chain_abs_error > max_chain_error)
                            max_chain_error = chain_abs_error;
                        abs_error = (got_value >= expected_value)
                                  ? got_value - expected_value
                                  : expected_value - got_value;
                        if (abs_error > max_abs_error) max_abs_error = abs_error;
                        if ((expected_value >= 0 ? expected_value : -expected_value)
                                > output_absmax)
                            output_absmax = expected_value >= 0
                                          ? expected_value : -expected_value;
                        if (epi_output_bus[j*16 +: 16] !== expected_output_mem[flat_idx])
                            output_mismatches++;
                        if (epi_output_bus[j*16 +: 16] !== reference_output_mem[flat_idx])
                            chain_mismatches++;
                    end
                end
            end
        end

        count_cycles = 0;
        if (acc_mismatches != 0)
            $fatal(1, "Full projection had %0d INT32 mismatches", acc_mismatches);
        relative_error = max_abs_error / ((output_absmax > 1.0e-20) ? output_absmax : 1.0);
        chain_relative_error = max_chain_error /
            ((output_absmax > 1.0e-20) ? output_absmax : 1.0);
        if (relative_error >= 0.0045) begin
            $display("QPROJ_DIAGNOSTIC output_rel=%e chain_rel=%e max_abs=%e expected_peak=%e got0=%h expected0=%h ref0=%h acc0=%h token_scale0=%h ws0=%h bias0=%h",
                relative_error, chain_relative_error, max_abs_error, output_absmax,
                actual_output_mem[0], expected_output_mem[0], reference_output_mem[0],
                expected_acc_mem[0], token_scale_mem[0], weight_scale_mem[0], bias_mem[0]);
            $display("QPROJ_FP32_STAGES acc=%h mul_token=%h mul_weight=%h add_bias=%h",
                u_epilogue.int32_to_fp32($signed(expected_acc_mem[0])),
                u_epilogue.fp32_mul_rne(
                    u_epilogue.int32_to_fp32($signed(expected_acc_mem[0])), token_scale_mem[0]),
                u_epilogue.fp32_mul_rne(
                    u_epilogue.fp32_mul_rne(
                        u_epilogue.int32_to_fp32($signed(expected_acc_mem[0])), token_scale_mem[0]),
                    weight_scale_mem[0]),
                u_epilogue.fp32_add_rne(
                    u_epilogue.fp32_mul_rne(
                        u_epilogue.fp32_mul_rne(
                            u_epilogue.int32_to_fp32($signed(expected_acc_mem[0])), token_scale_mem[0]),
                        weight_scale_mem[0]), bias_mem[0]));
            $fatal(1, "Full projection output error exceeds gate: %e", relative_error);
        end
        if (chain_relative_error >= 0.0045)
            $fatal(1, "FP32 epilogue numerical error exceeds gate: %e",
                   chain_relative_error);
        $display("SUPRA2_QPROJ_PASS M=%0d N=%0d K=%0d", m_size, n_size, k_size);
        $display("SUPRA2_QPROJ_CYCLES %0d", total_cycles);
        $display("SUPRA2_QPROJ_OUTPUT_BIT_MISMATCHES %0d", output_mismatches);
        $display("SUPRA2_QPROJ_SOFTWARE_CHAIN_MISMATCHES %0d", chain_mismatches);
        $display("SUPRA2_QPROJ_SOFTWARE_CHAIN_REL_ERROR %e", chain_relative_error);
        $display("SUPRA2_QPROJ_FRONTEND_Q_MISMATCHES %0d", quant_mismatches);
        $display("SUPRA2_QPROJ_FRONTEND_H_MISMATCHES %0d", hadamard_mismatches);
        $display("SUPRA2_QPROJ_OUTPUT_REL_ERROR %e", relative_error);
        if (dump_actual_output) begin
            for (flat_idx = 0; flat_idx < m_size*n_size; flat_idx++)
                $fdisplay(actual_output_fd, "%04h", actual_output_mem[flat_idx]);
            $fclose(actual_output_fd);
        end
        $finish;
    end
endmodule

`default_nettype wire
