`default_nettype none

module tb_fp16_to_fp32_subnormal;
    import fp16_utils_pkg::*;

    logic [15:0] half_bits;
    logic [31:0] got;
    logic [31:0] expected;
    integer fraction;
    integer sign;
    integer highest;
    integer cases;

    initial begin
        cases = 0;
        for (fraction = 1; fraction < 1024; fraction = fraction + 1) begin
            highest = 0;
            for (integer bit_index = 0; bit_index < 10; bit_index = bit_index + 1)
                if ((fraction & (1 << bit_index)) != 0) highest = bit_index;
            for (sign = 0; sign < 2; sign = sign + 1) begin
                half_bits = {sign[0], 5'b0, fraction[9:0]};
                got = fp16_to_fp32(half_bits);
                expected = {sign[0], 8'(highest + 103),
                            23'(fraction << (23-highest))};
                if (got !== expected)
                    $fatal(1, "subnormal conversion mismatch half=%04h got=%08h expected=%08h",
                           half_bits, got, expected);
                cases = cases + 1;
            end
        end
        if (fp16_to_fp32(16'h0000) !== 32'h00000000 ||
            fp16_to_fp32(16'h8000) !== 32'h80000000)
            $fatal(1, "signed zero conversion mismatch");
        $display("FP16_TO_FP32_SUBNORMAL_PASS cases=%0d", cases);
        $finish;
    end
endmodule

`default_nettype wire
