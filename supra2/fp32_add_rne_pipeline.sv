`default_nettype none

// Eight-stage, initiation-interval-one pipeline matching fp32_add_rne.
// Special-value and underflow behavior intentionally follows the package
// helper used by the current Supra2 datapath.
module supra2_fp32_add_rne_pipeline (
    input  wire logic        clk,
    input  wire logic        rst_n,
    input  wire logic        in_valid,
    input  wire logic [31:0] a,
    input  wire logic [31:0] b,
    output logic             out_valid,
    output logic [31:0]      result
);
    import fp16_utils_pkg::*;

    logic [7:0] valid_pipe;
    logic s0_sign;
    logic s0_same_sign;
    logic s0_zero;
    logic [7:0] s0_exp, s0_delta;
    logic [23:0] s0_large, s0_small;
    logic s1_sign, s1_same_sign, s1_zero;
    logic [7:0] s1_exp;
    logic [26:0] s1_large, s1_small;
    logic s2_sign, s2_same_sign, s2_zero;
    logic [7:0] s2_exp;
    logic [27:0] s2_sum;
    logic s3_sign, s3_zero, s3_subtract;
    logic [7:0] s3_exp;
    logic [26:0] s3_normalized;
    logic [4:0] s3_shift;
    logic s4_sign, s4_zero;
    logic [7:0] s4_exp;
    logic [26:0] s4_normalized;
    logic [7:0] s5_exp;
    logic s5_sign, s5_zero;
    logic [24:0] s5_rounded;
    logic s6_sign, s6_zero;
    logic [7:0] s6_exp;
    logic [24:0] s6_rounded;

    assign out_valid = valid_pipe[7];

    always_ff @(posedge clk or negedge rst_n) begin : pipeline
        logic sign_a, sign_b, same_sign, sign_result;
        logic [7:0] exp_a, exp_b, exponent, delta;
        logic [23:0] mant_a, mant_b, mant_large, mant_small;
        logic [26:0] ext_large, ext_small, normalized;
        logic [27:0] sum;
        logic [4:0] normalize_shift;
        logic [24:0] rounded;
        logic guard_bit, round_bit, sticky_bit;
        if (!rst_n) begin
            valid_pipe <= '0;
        end else begin
            valid_pipe <= {valid_pipe[6:0], in_valid};

            if (in_valid) begin
                sign_a = a[31];
                sign_b = b[31];
                exp_a = a[30:23];
                exp_b = b[30:23];
                mant_a = (exp_a == 0) ? {1'b0,a[22:0]} : {1'b1,a[22:0]};
                mant_b = (exp_b == 0) ? {1'b0,b[22:0]} : {1'b1,b[22:0]};
                if (exp_a > exp_b ||
                    (exp_a == exp_b && mant_a >= mant_b)) begin
                    exponent = exp_a;
                    mant_large = mant_a;
                    mant_small = mant_b;
                    sign_result = sign_a;
                end else begin
                    exponent = exp_b;
                    mant_large = mant_b;
                    mant_small = mant_a;
                    sign_result = sign_b;
                end
                s0_sign <= sign_result;
                s0_same_sign <= (sign_a == sign_b);
                s0_zero <= (mant_large == 0);
                s0_exp <= exponent;
                s0_delta <= (exp_a > exp_b) ? exp_a-exp_b : exp_b-exp_a;
                s0_large <= mant_large;
                s0_small <= mant_small;
            end

            if (valid_pipe[0]) begin
                s1_sign <= s0_sign;
                s1_same_sign <= s0_same_sign;
                s1_zero <= s0_zero;
                s1_exp <= s0_exp;
                s1_large <= {s0_large,3'b0};
                s1_small <= fp32_shift_right_jam({s0_small,3'b0},s0_delta);
            end

            if (valid_pipe[1]) begin
                s2_sign <= s1_sign;
                s2_same_sign <= s1_same_sign;
                s2_zero <= s1_zero;
                s2_exp <= s1_exp;
                if (s1_same_sign)
                    s2_sum <= {1'b0,s1_large} + {1'b0,s1_small};
                else
                    s2_sum <= {1'b0,s1_large} - {1'b0,s1_small};
            end

            if (valid_pipe[2]) begin
                s3_sign <= s2_sign;
                s3_zero <= s2_zero;
                s3_subtract <= !s2_same_sign;
                exponent = s2_exp;
                if (s2_same_sign) begin
                    if (s2_sum[27]) begin
                        normalized = s2_sum[27:1];
                        normalized[0] = normalized[0] | s2_sum[0];
                        exponent = exponent + 1'b1;
                    end else begin
                        normalized = s2_sum[26:0];
                    end
                end else begin
                    normalized = s2_sum[26:0];
                    if (normalized == 0) begin
                        s3_zero <= 1'b1;
                        normalize_shift = 0;
                    end else begin
                        normalize_shift = fp32_lzc27(normalized);
                        if (exponent < {3'b000,normalize_shift})
                            normalize_shift = exponent[4:0];
                    end
                end
                s3_exp <= exponent;
                s3_normalized <= normalized;
                s3_shift <= s2_same_sign ? 5'd0 : normalize_shift;
            end

            if (valid_pipe[3]) begin
                s4_sign <= s3_sign;
                s4_zero <= s3_zero;
                if (s3_subtract)
                    s4_exp <= s3_exp - {3'b000,s3_shift};
                else
                    s4_exp <= s3_exp;
                s4_normalized <= s3_normalized << s3_shift;
            end

            if (valid_pipe[4]) begin
                s5_sign <= s4_sign;
                s5_zero <= s4_zero;
                s5_exp <= s4_exp;
                rounded = {1'b0,s4_normalized[26:3]};
                guard_bit = s4_normalized[2];
                round_bit = s4_normalized[1];
                sticky_bit = s4_normalized[0];
                if (guard_bit && (round_bit || sticky_bit || rounded[0]))
                    rounded = rounded + 1'b1;
                s5_rounded <= rounded;
            end

            if (valid_pipe[5]) begin
                s6_sign <= s5_sign;
                s6_zero <= s5_zero;
                s6_exp <= s5_exp;
                s6_rounded <= s5_rounded;
            end

            if (valid_pipe[6]) begin
                sign_result = s6_sign;
                exponent = s6_exp;
                rounded = s6_rounded;
                if (rounded[24]) begin
                    rounded = rounded >> 1;
                    exponent = exponent + 1'b1;
                end
                if (s6_zero)
                    result <= 32'b0;
                else if (exponent == 0)
                    result <= {sign_result,31'b0};
                else if (exponent >= 8'hff)
                    result <= {sign_result,8'hff,23'b0};
                else
                    result <= {sign_result,exponent,rounded[22:0]};
            end
        end
    end

    initial begin
        if ($bits(result) != 32)
            $error("FP32 adder output width must remain 32 bits");
    end
endmodule

`default_nettype wire
