`default_nettype none

// Supra2 INT32 accumulator -> FP16 output epilogue.
// Computes round_fp16(float(acc) * s_t * weight_scale + bias).
module supra2_qlinear_epilogue #(
    parameter int LANES = 32
) (
    input  wire logic [LANES*32-1:0] acc_int32,
    input  wire logic [31:0]         token_scale_fp32,
    input  wire logic [LANES*32-1:0] weight_scale_fp32,
    input  wire logic [LANES*32-1:0] bias_fp32,
    output logic [LANES*16-1:0] output_fp16
);
    import fp16_utils_pkg::*;

    function automatic logic [31:0] int32_to_fp32(input logic signed [31:0] value);
        logic sign;
        logic [31:0] magnitude;
        logic [63:0] normalized;
        logic [24:0] significand;
        logic [22:0] fraction;
        logic [7:0] exponent;
        logic guard_bit, round_bit, sticky_bit;
        begin
            sign = value[31];
            magnitude = sign ? $unsigned(-value) : $unsigned(value);
            if (magnitude == 0) begin
                int32_to_fp32 = 32'h00000000;
            end else begin
                normalized = {32'b0, magnitude};
                exponent = 0;
                // Scan low-to-high so the final set bit is the true MSB.
                // Using exponent==0 as a sentinel loses the MSB for values
                // whose only set bit is bit 0 and is fragile in simulators.
                for (int bitno = 0; bitno < 32; bitno++) begin
                    if (magnitude[bitno]) exponent = bitno;
                end
                normalized = normalized << (31 - exponent);
                significand = {1'b0, normalized[31:8]};
                guard_bit = normalized[7];
                round_bit = normalized[6];
                sticky_bit = |normalized[5:0];
                if (guard_bit && (round_bit || sticky_bit || significand[0]))
                    significand = significand + 1'b1;
                if (significand[24]) begin
                    // Rounding overflow: 1.111.. rounds to 10.000..
                    exponent = exponent + 1'b1;
                    fraction = 23'b0;
                end else begin
                    fraction = significand[22:0];
                end
                int32_to_fp32 = {sign, exponent + 8'd127, fraction};
            end
        end
    endfunction

    function automatic logic [31:0] fp32_mul_rne(
        input logic [31:0] a, input logic [31:0] b
    );
        logic sign;
        logic [7:0] ea, eb;
        logic [23:0] ma, mb;
        logic [47:0] product;
        logic [24:0] rounded;
        logic [7:0] exponent;
        logic signed [9:0] exp_work;
        logic guard_bit, round_bit, sticky_bit;
        logic [22:0] fraction;
        begin
            sign = a[31] ^ b[31];
            ea = a[30:23]; eb = b[30:23];
            if ((ea == 0 && a[22:0] == 0) || (eb == 0 && b[22:0] == 0)) begin
                fp32_mul_rne = {sign, 31'b0};
            end else if (ea == 8'hff || eb == 8'hff) begin
                fp32_mul_rne = {sign, 8'hff, 23'b0};
            end else begin
                ma = {1'b1, a[22:0]}; mb = {1'b1, b[22:0]};
                product = ma * mb;
                exp_work = $signed({2'b00, ea}) + $signed({2'b00, eb})
                         - 10'sd127;
                if (product[47]) begin
                    rounded = {1'b0, product[47:24]};
                    guard_bit = product[23];
                    round_bit = product[22];
                    sticky_bit = |product[21:0];
                    exp_work = exp_work + 10'sd1;
                end else begin
                    rounded = {1'b0, product[46:23]};
                    guard_bit = product[22];
                    round_bit = product[21];
                    sticky_bit = |product[20:0];
                end
                if (guard_bit && (round_bit || sticky_bit || rounded[0]))
                    rounded = rounded + 1'b1;
                if (rounded[24]) begin
                    rounded = rounded >> 1;
                    exp_work = exp_work + 10'sd1;
                end
                if (exp_work <= 0)
                    fp32_mul_rne = {sign, 31'b0};
                else if (exp_work >= 255)
                    fp32_mul_rne = {sign, 8'hff, 23'b0};
                else begin
                    exponent = exp_work[7:0];
                    fraction = rounded[22:0];
                    fp32_mul_rne = {sign, exponent, fraction};
                end
            end
        end
    endfunction

    function automatic logic [26:0] shift_right_jam(
        input logic [26:0] value, input logic [7:0] amount
    );
        logic sticky;
        logic [26:0] shifted;
        begin
            sticky = 1'b0;
            if (amount == 0) begin
                shifted = value;
            end else if (amount >= 27) begin
                shifted = 0;
                sticky = |value;
            end else begin
                shifted = value >> amount;
                sticky = |(value << (27 - amount));
            end
            shifted[0] = shifted[0] | sticky;
            return shifted;
        end
    endfunction

    function automatic logic [31:0] fp32_add_rne(
        input logic [31:0] a, input logic [31:0] b
    );
        logic sign_a, sign_b, sign_result;
        logic [7:0] exp_a, exp_b, exp_large, exp_small;
        logic [23:0] mant_a, mant_b, mant_large, mant_small;
        logic [26:0] ext_large, ext_small;
        logic [27:0] sum;
        logic [26:0] normalized;
        logic [24:0] rounded;
        logic [7:0] exponent;
        logic guard_bit, round_bit, sticky_bit;
        logic [22:0] fraction;
        logic [4:0] normalize_shift;
        begin
            sign_a = a[31]; sign_b = b[31];
            exp_a = a[30:23]; exp_b = b[30:23];
            mant_a = (exp_a == 0) ? {1'b0, a[22:0]} : {1'b1, a[22:0]};
            mant_b = (exp_b == 0) ? {1'b0, b[22:0]} : {1'b1, b[22:0]};
            if (exp_a > exp_b || (exp_a == exp_b && mant_a >= mant_b)) begin
                exp_large = exp_a; exp_small = exp_b;
                mant_large = mant_a; mant_small = mant_b;
                sign_result = sign_a;
            end else begin
                exp_large = exp_b; exp_small = exp_a;
                mant_large = mant_b; mant_small = mant_a;
                sign_result = sign_b;
            end
            if (exp_large == 0 && mant_large == 0) begin
                fp32_add_rne = (exp_small == 0 && mant_small == 0)
                             ? 32'b0 : (sign_b ? {1'b1,b[30:0]} : b);
            end else begin
                ext_large = {mant_large, 3'b000};
                ext_small = shift_right_jam({mant_small, 3'b000},
                                             exp_large - exp_small);
                exponent = exp_large;
                if (sign_a == sign_b) begin
                    sum = {1'b0, ext_large} + {1'b0, ext_small};
                    if (sum[27]) begin
                        normalized = sum[27:1];
                        normalized[0] = normalized[0] | sum[0];
                        exponent = exponent + 1'b1;
                    end else normalized = sum[26:0];
                end else begin
                    sum = {1'b0, ext_large} - {1'b0, ext_small};
                    normalized = sum[26:0];
                    if (normalized == 0) begin
                        fp32_add_rne = 32'b0;
                        return fp32_add_rne;
                    end
                    normalize_shift = fp32_lzc27(normalized);
                    if (exponent < {3'b000, normalize_shift})
                        normalize_shift = exponent[4:0];
                    normalized = normalized << normalize_shift;
                    exponent = exponent - {3'b000, normalize_shift};
                end
                guard_bit = normalized[2];
                round_bit = normalized[1];
                sticky_bit = normalized[0];
                rounded = {1'b0, normalized[26:3]};
                if (guard_bit && (round_bit || sticky_bit || rounded[0]))
                    rounded = rounded + 1'b1;
                if (rounded[24]) begin
                    rounded = rounded >> 1;
                    exponent = exponent + 1'b1;
                end
                if (exponent == 0)
                    fp32_add_rne = {sign_result,31'b0};
                else if (exponent >= 8'hff)
                    fp32_add_rne = {sign_result,8'hff,23'b0};
                else begin
                    fraction = rounded[22:0];
                    fp32_add_rne = {sign_result, exponent, fraction};
                end
            end
        end
    endfunction

    function automatic logic [15:0] qlinear_lane(
        input logic signed [31:0] acc,
        input logic [31:0] token_scale,
        input logic [31:0] weight_scale,
        input logic [31:0] bias
    );
        logic [31:0] acc_fp32, token_scaled, channel_scaled, result_fp32;
        begin
            acc_fp32 = int32_to_fp32(acc);
            token_scaled = fp32_mul_rne(acc_fp32, token_scale);
            channel_scaled = fp32_mul_rne(token_scaled, weight_scale);
            result_fp32 = fp32_add_rne(channel_scaled, bias);
            qlinear_lane = fp32_to_fp16(result_fp32);
        end
    endfunction

    always_comb begin
        for (int lane = 0; lane < LANES; lane++) begin
            output_fp16[lane*16 +: 16] = qlinear_lane(
                $signed(acc_int32[lane*32 +: 32]), token_scale_fp32,
                weight_scale_fp32[lane*32 +: 32], bias_fp32[lane*32 +: 32]);
        end
    end
endmodule

`default_nettype wire
