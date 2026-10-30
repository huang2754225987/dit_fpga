`timescale 1ns/1ps
`default_nettype none

module tb_supra2_qlinear_epilogue;
    localparam int LANES = 32;
    logic [LANES*32-1:0] acc_bus, weight_scale_bus, bias_bus;
    logic [31:0] token_scale;
    logic [LANES*16-1:0] output_bus;
    logic [31:0] acc_mem [0:LANES*LANES-1];
    logic [31:0] token_scale_mem [0:LANES-1];
    logic [31:0] weight_scale_mem [0:LANES-1];
    logic [31:0] bias_mem [0:LANES-1];
    logic [31:0] reference_absmax_mem [0:0];
    logic [15:0] expected_mem [0:LANES*LANES-1];
    string acc_file, token_scale_file, weight_scale_file;
    string bias_file, expected_file, reference_absmax_file;
    integer i, j;
    real got_value, expected_value, abs_error, max_error, reference_absmax;
    real relative_error;

    supra2_qlinear_epilogue #(.LANES(LANES)) dut (
        .acc_int32(acc_bus), .token_scale_fp32(token_scale),
        .weight_scale_fp32(weight_scale_bus), .bias_fp32(bias_bus),
        .output_fp16(output_bus)
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
        if (!$value$plusargs("ACC_FILE=%s", acc_file) ||
            !$value$plusargs("TOKEN_SCALE_FILE=%s", token_scale_file) ||
            !$value$plusargs("WEIGHT_SCALE_FILE=%s", weight_scale_file) ||
            !$value$plusargs("BIAS_FILE=%s", bias_file) ||
            !$value$plusargs("EXPECTED_FILE=%s", expected_file) ||
            !$value$plusargs("REFERENCE_ABSMAX_FILE=%s", reference_absmax_file))
            $fatal(1, "Provide all epilogue fixture file plusargs");

        $readmemh(acc_file, acc_mem);
        $readmemh(token_scale_file, token_scale_mem);
        $readmemh(weight_scale_file, weight_scale_mem);
        $readmemh(bias_file, bias_mem);
        $readmemh(expected_file, expected_mem);
        $readmemh(reference_absmax_file, reference_absmax_mem);
        reference_absmax = single_to_real(reference_absmax_mem[0]);

        for (j = 0; j < LANES; j++) begin
            weight_scale_bus[j*32 +: 32] = weight_scale_mem[j];
            bias_bus[j*32 +: 32] = bias_mem[j];
        end
        max_error = 0.0;
        for (i = 0; i < LANES; i++) begin
            token_scale = token_scale_mem[i];
            for (j = 0; j < LANES; j++)
                acc_bus[j*32 +: 32] = acc_mem[i*LANES+j];
            #1;
            for (j = 0; j < LANES; j++) begin
                got_value = half_to_real(output_bus[j*16 +: 16]);
                expected_value = half_to_real(expected_mem[i*LANES+j]);
                abs_error = (got_value >= expected_value)
                          ? got_value - expected_value : expected_value - got_value;
                if (abs_error > max_error) begin
                    max_error = abs_error;
                end
            end
        end
        relative_error = max_error / ((reference_absmax > 1.0e-20)
                                     ? reference_absmax : 1.0);
        if (relative_error >= 0.0045)
            $fatal(1, "QLinear epilogue error too large: max_abs=%e rel=%e",
                   max_error, relative_error);
        $display("SUPRA2_QLINEAR_EPILOGUE_PASS max_abs=%e rel=%e",
                 max_error, relative_error);
        $finish;
    end
endmodule

`default_nettype wire
