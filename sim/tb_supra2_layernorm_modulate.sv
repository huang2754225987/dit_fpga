`default_nettype none

module tb_supra2_layernorm_modulate #(parameter int ROWS=256);
    localparam int FEATURES = 576;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic start = 1'b0;
    logic in_valid = 1'b0;
    logic in_ready;
    logic [15:0] x_fp16, shift_fp16, scale_fp16;
    logic out_valid;
    logic out_ready = 1'b1;
    logic [15:0] out_fp16;
    logic busy, done;
    logic [15:0] x_mem [0:ROWS*FEATURES-1];
    logic [15:0] shift_mem [0:FEATURES-1];
    logic [15:0] scale_mem [0:FEATURES-1];
    logic [15:0] expected_mem [0:ROWS*FEATURES-1];
    logic [15:0] qkv_mem [0:ROWS*FEATURES-1];
    string x_file, shift_file, scale_file, expected_file, qkv_file, output_file;
    integer row, feature, index, output_count, cycles, output_fd;
    real got, expected, qkv, abs_err, max_abs, max_ref, max_qkv_abs, max_qkv_ref;

    always #5 clk = ~clk;
    supra2_layernorm_modulate #(.FEATURES(FEATURES), .COUNT_W(10)) dut (
        .clk, .rst_n, .start, .in_valid, .in_ready, .x_fp16, .shift_fp16,
        .scale_fp16, .out_valid, .out_ready, .out_fp16, .busy, .done
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

    always @(posedge clk) begin
        if (busy) cycles <= cycles + 1;
        if (out_valid && out_ready) begin
            got = half_to_real(out_fp16);
            expected = half_to_real(expected_mem[output_count]);
            qkv = half_to_real(qkv_mem[output_count]);
            $fdisplay(output_fd, "%04h", out_fp16);
            abs_err = (got >= expected) ? got-expected : expected-got;
            if (abs_err > max_abs) max_abs = abs_err;
            if ((expected >= 0 ? expected : -expected) > max_ref)
                max_ref = (expected >= 0 ? expected : -expected);
            abs_err = (got >= qkv) ? got-qkv : qkv-got;
            if (abs_err > max_qkv_abs) max_qkv_abs = abs_err;
            if ((qkv >= 0 ? qkv : -qkv) > max_qkv_ref)
                max_qkv_ref = (qkv >= 0 ? qkv : -qkv);
            output_count <= output_count + 1;
        end
    end

    initial begin
        if (!$value$plusargs("X_FILE=%s", x_file) ||
            !$value$plusargs("SHIFT_FILE=%s", shift_file) ||
            !$value$plusargs("SCALE_FILE=%s", scale_file) ||
            !$value$plusargs("EXPECTED_FILE=%s", expected_file) ||
            !$value$plusargs("QKV_FILE=%s", qkv_file) ||
            !$value$plusargs("OUTPUT_FILE=%s", output_file))
            $fatal(1, "Missing LayerNorm fixture plusargs");
        $readmemh(x_file, x_mem);
        $readmemh(shift_file, shift_mem);
        $readmemh(scale_file, scale_mem);
        $readmemh(expected_file, expected_mem);
        $readmemh(qkv_file, qkv_mem);
        output_count = 0;
        cycles = 0;
        max_abs = 0.0;
        max_ref = 0.0;
        max_qkv_abs = 0.0;
        max_qkv_ref = 0.0;
        output_fd = $fopen(output_file, "w");
        if (!output_fd) $fatal(1, "Cannot open LayerNorm output fixture");
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        for (row = 0; row < ROWS; row = row + 1) begin
            @(negedge clk);
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            for (feature = 0; feature < FEATURES; feature = feature + 1) begin
                while (!in_ready) @(negedge clk);
                index = row*FEATURES + feature;
                in_valid = 1'b1;
                x_fp16 = x_mem[index];
                shift_fp16 = shift_mem[feature];
                scale_fp16 = scale_mem[feature];
                @(negedge clk);
                in_valid = 1'b0;
            end
            wait (done);
            @(negedge clk);
        end
        @(posedge clk);
        #1;
        if (output_count != ROWS*FEATURES)
            $fatal(1, "Output count mismatch: %0d", output_count);
        $display("SUPRA2_LAYERNORM_MODULATE rows=%0d features=%0d cycles=%0d",
                 ROWS, FEATURES, cycles);
        $display("SUPRA2_LAYERNORM_MODULATE_MAX_ABS %.9g", max_abs);
        $display("SUPRA2_LAYERNORM_MODULATE_REL_MAX %.9g", max_abs/max_ref);
        $display("SUPRA2_LAYERNORM_TO_QKV_RAW_MAX_ABS %.9g", max_qkv_abs);
        $display("SUPRA2_LAYERNORM_TO_QKV_RAW_REL_MAX %.9g", max_qkv_abs/max_qkv_ref);
        if (max_abs/max_ref > 0.005)
            $fatal(1, "LayerNorm/AdaLN relative error exceeds 0.5%% gate");
        $fclose(output_fd);
        $display("SUPRA2_LAYERNORM_MODULATE_PASS");
        $finish;
    end
endmodule

`default_nettype wire
