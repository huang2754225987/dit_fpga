`timescale 1ns/1ps
`default_nettype none

module tb_supra2_activation_quantizer;
    localparam int LANES = 32;
    localparam int MAX_K = 2304;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic start = 1'b0;
    logic [11:0] vector_k = '0;
    logic in_valid = 1'b0;
    logic in_ready;
    logic [LANES*16-1:0] in_fp16 = '0;
    logic out_valid;
    logic out_ready = 1'b1;
    logic [LANES*8-1:0] out_int8;
    logic [31:0] scale_fp32;
    logic busy, done;
    logic done_seen = 1'b0;
    logic [15:0] input_mem [0:MAX_K-1];
    logic [7:0] expected_mem [0:MAX_K-1];
    logic [31:0] scale_mem [0:0];
    string input_file, expected_file, scale_file;
    integer k_size, i, beat, lane, mismatches;
    integer plusarg_ok;
    real expected_scale, actual_scale, scale_relative_error;

    always #5 clk = ~clk;
    always @(posedge clk)
        if (done) done_seen <= 1'b1;

    supra2_activation_quantizer #(
        .MAX_K(MAX_K), .LANES(LANES), .K_W(12)
    ) dut (
        .clk(clk), .rst_n(rst_n), .start(start), .vector_k(vector_k),
        .in_valid(in_valid), .in_ready(in_ready), .in_fp16(in_fp16),
        .out_valid(out_valid), .out_ready(out_ready), .out_int8(out_int8),
        .scale_fp32(scale_fp32), .busy(busy), .done(done)
    );

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
        if (!$value$plusargs("K=%d", k_size) || k_size <= 0 ||
            k_size > MAX_K ||
            !$value$plusargs("INPUT_FILE=%s", input_file) ||
            !$value$plusargs("EXPECTED_FILE=%s", expected_file) ||
            !$value$plusargs("SCALE_FILE=%s", scale_file))
            $fatal(1, "Provide K, INPUT_FILE, EXPECTED_FILE, and SCALE_FILE");

        $readmemh(input_file, input_mem, 0, k_size-1);
        $readmemh(expected_file, expected_mem, 0, k_size-1);
        $readmemh(scale_file, scale_mem);
        expected_scale = single_to_real(scale_mem[0]);

        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk);
        vector_k = k_size;
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;
        in_valid = 1'b1;
        for (beat = 0; beat < (k_size+LANES-1)/LANES; beat++) begin
            for (lane = 0; lane < LANES; lane++) begin
                if (beat*LANES+lane < k_size)
                    in_fp16[lane*16 +: 16] = input_mem[beat*LANES+lane];
                else in_fp16[lane*16 +: 16] = 16'b0;
            end
            @(negedge clk);
        end
        in_valid = 1'b0;
        in_fp16 = '0;

        mismatches = 0;
        beat = 0;
        while (beat < (k_size+LANES-1)/LANES) begin
            if (out_valid) begin
                for (lane = 0; lane < LANES; lane++) begin
                    if (beat*LANES+lane < k_size &&
                        out_int8[lane*8 +: 8] !== expected_mem[beat*LANES+lane]) begin
                        if (mismatches < 8)
                            $display("QUANT_MISMATCH index=%0d got=%h expected=%h",
                                     beat*LANES+lane,
                                     out_int8[lane*8 +: 8],
                                     expected_mem[beat*LANES+lane]);
                        mismatches++;
                    end
                    if (beat*LANES+lane >= k_size &&
                        out_int8[lane*8 +: 8] !== 8'b0)
                        mismatches++;
                end
                if (beat == 0) begin
                    actual_scale = single_to_real(scale_fp32);
                    scale_relative_error = (actual_scale >= expected_scale)
                        ? (actual_scale - expected_scale) / expected_scale
                        : (expected_scale - actual_scale) / expected_scale;
                end
                beat++;
            end
            @(negedge clk);
        end
        if (mismatches != 0)
            $fatal(1, "Activation quantizer produced %0d mismatches", mismatches);
        if (scale_relative_error > 0.001)
            $fatal(1, "Activation scale error too large: rel=%e", scale_relative_error);
        if (busy || !done_seen)
            $fatal(1, "Quantizer completion handshake missing");
        $display("SUPRA2_ACTIVATION_QUANTIZER_PASS K=%0d scale_rel=%e",
                 k_size, scale_relative_error);
        $finish;
    end
endmodule

`default_nettype wire
