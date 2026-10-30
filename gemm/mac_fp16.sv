// =============================================================================
// FP16 Multiply-Accumulate Unit (3-stage pipelined MAC, 2-way interleaved
// accumulators, 2-stage output merge)
//
//   Stage 1: FP16 x FP16 -> FP32 product               (registered)
//   Stage 2: FP32 align + add/subtract + carry fold    (registered partial)
//   Stage 3: normalize + round-to-nearest-even         (slot writeback)
//   Merge 1: align(slot[0], slot[1])                   (registered partial)
//   Merge 2: finish -> acc_out                         (registered)
//
// The previous 2-stage design placed the entire FP32 RNE addition on the
// accumulator feedback path (~70 logic levels; post-synthesis WNS -12.1 ns
// at 200 MHz on xczu7ev-ffvc1156-2-e). The accumulation loop is now two
// cycles (stage-2 slot read -> stage-3 slot writeback), so two interleaved
// slot accumulators (slot = beat parity) sustain one product per cycle
// without a read-after-write hazard: a slot written at the end of cycle C+2
// is next read during cycle C+3.
//
// acc_out comes from a continuously running 2-stage merge of the two slots.
// Once the slot registers stop changing value, acc_out reflects the merged
// total two cycles later. Consumers never sample earlier than two cycles
// after the last useful beat (verified against supra2_gemm_engine
// S_WAIT_ACC, gemm_ctrl ST_STORE, attention_online_pv(_batch) S_PV_DRAIN and
// the direct-array testbenches).
//
// Numerics: each pipeline half replicates fp32_add_rne (fp16_utils_pkg)
// operation-for-operation, so one beat update is bit-identical to the old
// single-cycle adder. The K-dim summation order is reassociated to
// (sum of even-k products) + (sum of odd-k products), each slot strictly in
// order; FP addition is not associative, so results may differ from the
// 2-stage design by reassociation rounding only.
//
// clear_acc flushes the pipeline: a beat sampled on a clear cycle is dropped
// (en is masked with !clear_acc down the pipe) and the slots plus merge
// registers are cleared one cycle later (clear_d1), matching the original
// clear timing observed by the systolic controllers.
// =============================================================================
`default_nettype none

module mac_fp16 #(
    parameter int DATA_W = 16,  // FP16 = 16 bits
    parameter int ACC_W  = 32   // FP32 accumulator
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    clear_acc,
    input  wire                    en,
    input  wire  [DATA_W-1:0]     a_in,    // FP16 input
    input  wire  [DATA_W-1:0]     b_in,    // FP16 input
    output logic [ACC_W-1:0]      acc_out  // FP32 accumulator (merged slots)
);

    import fp16_utils_pkg::*;

    // =========================================================================
    // FP16 field extraction helpers
    // =========================================================================
    // FP16: sign[15], exponent[14:10] (5-bit, bias=15), mantissa[9:0]

    logic        a_sign, b_sign;
    logic [4:0]  a_exp,  b_exp;
    logic [9:0]  a_mant, b_mant;
    logic        a_is_zero, b_is_zero;
    logic        a_is_inf,  b_is_inf;
    logic        a_is_nan,  b_is_nan;

    always_comb begin
        a_sign    = a_in[15];
        a_exp     = a_in[14:10];
        a_mant    = a_in[9:0];
        b_sign    = b_in[15];
        b_exp     = b_in[14:10];
        b_mant    = b_in[9:0];

        a_is_zero = (a_exp == 5'h00) && (a_mant == 10'h000);
        b_is_zero = (b_exp == 5'h00) && (b_mant == 10'h000);
        a_is_inf  = (a_exp == 5'h1F) && (a_mant == 10'h000);
        b_is_inf  = (b_exp == 5'h1F) && (b_mant == 10'h000);
        a_is_nan  = (a_exp == 5'h1F) && (a_mant != 10'h000);
        b_is_nan  = (b_exp == 5'h1F) && (b_mant != 10'h000);
    end

    // =========================================================================
    // Stage 1: FP16 multiply -> FP32 product (registered)
    // =========================================================================
    logic [ACC_W-1:0] product_fp32;
    logic             en_d1;
    logic             clear_d1;
    logic             slot_cnt;   // beat parity: 0 = even beat, 1 = odd beat
    logic             slot_d1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            product_fp32 <= 32'h0000_0000;
            en_d1        <= 1'b0;
            clear_d1     <= 1'b0;
            slot_cnt     <= 1'b0;
            slot_d1      <= 1'b0;
        end else begin
            en_d1    <= en && !clear_acc;
            clear_d1 <= clear_acc;
            if (en && !clear_acc) begin
                product_fp32 <= fp16_mul_fields(a_sign, a_exp, a_mant,
                                         b_sign, b_exp, b_mant,
                                         a_is_zero, b_is_zero,
                                         a_is_inf,  b_is_inf,
                                         a_is_nan,  b_is_nan);
                slot_d1      <= slot_cnt;
                slot_cnt     <= ~slot_cnt;
            end else begin
                product_fp32 <= 32'h0000_0000;
                if (clear_acc)
                    slot_cnt <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Slot accumulators (declared before stage 2: stage 2 reads them)
    // =========================================================================
    logic [ACC_W-1:0] acc_slot [2];

    // =========================================================================
    // Stage 2: align + add/subtract partial (combinational from product and
    // the addressed slot), registered for stage 3.
    // =========================================================================
    logic [37:0] part2;      // {sign, exponent[7:0], normalized[26:0], zero, diff}
    logic        en_d2;
    logic        slot_d2;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            part2   <= 38'd0;
            en_d2   <= 1'b0;
            slot_d2 <= 1'b0;
        end else begin
            en_d2   <= en_d1 && !clear_acc;
            slot_d2 <= slot_d1;
            if (en_d1 && !clear_acc)
                part2 <= fp32_add_align(product_fp32, acc_slot[slot_d1]);
        end
    end

    // =========================================================================
    // Stage 3: normalize + round, written into the addressed slot.
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_slot[0] <= 32'h0000_0000;
            acc_slot[1] <= 32'h0000_0000;
        end else if (clear_d1) begin
            acc_slot[0] <= 32'h0000_0000;
            acc_slot[1] <= 32'h0000_0000;
        end else if (en_d2) begin
            acc_slot[slot_d2] <= fp32_add_finish(part2);
        end
    end

    // =========================================================================
    // Output merge: continuous 2-stage pipeline, converges two cycles after
    // the last slot update. Uses the same split adder so no single-cycle
    // FP32 RNE path exists anywhere in this module.
    // =========================================================================
    logic [37:0] merge_part;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            merge_part <= 38'd0;
            acc_out    <= 32'h0000_0000;
        end else if (clear_d1) begin
            merge_part <= 38'd0;
            acc_out    <= 32'h0000_0000;
        end else begin
            merge_part <= fp32_add_align(acc_slot[0], acc_slot[1]);
            acc_out    <= fp32_add_finish(merge_part);
        end
    end

    // =========================================================================
    // Function: FP16 multiply -> FP32 result
    // =========================================================================
    function automatic logic [31:0] fp16_mul_fields(
        input logic        sa,
        input logic [4:0]  ea,
        input logic [9:0]  ma,
        input logic        sb,
        input logic [4:0]  eb,
        input logic [9:0]  mb,
        input logic        a_zero,
        input logic        b_zero,
        input logic        a_inf,
        input logic        b_inf,
        input logic        a_nan,
        input logic        b_nan
    );
        logic        res_sign;
        // The 11x11 significand multiply fits one DSP48E2. Prefer hard DSPs
        // over LUT fabric; the ZU7EV has ample DSP headroom in this design.
        (* use_dsp = "yes" *) logic [21:0] mant_prod;
        logic [10:0] full_ma, full_mb;
        logic signed [9:0]  raw_exp;  // intermediate exponent (signed, needs >8 bits)
        logic [7:0]  fp32_exp;
        logic [22:0] fp32_mant;
        logic        prod_overflow;

        // Result sign
        res_sign = sa ^ sb;

        // ------------------------------------------------------------------
        // Special cases
        // ------------------------------------------------------------------
        if (a_nan || b_nan) begin
            // NaN propagation -> quiet NaN
            fp16_mul_fields = {1'b0, 8'hFF, 23'h40_0000};  // canonical qNaN
            return fp16_mul_fields;
        end
        if ((a_inf && b_zero) || (b_inf && a_zero)) begin
            // inf * 0 = NaN
            fp16_mul_fields = {1'b0, 8'hFF, 23'h40_0000};
            return fp16_mul_fields;
        end
        if (a_inf || b_inf) begin
            // inf * finite = inf
            fp16_mul_fields = {res_sign, 8'hFF, 23'h000000};
            return fp16_mul_fields;
        end
        if (a_zero || b_zero) begin
            // +/- zero
            fp16_mul_fields = {res_sign, 31'h0000_0000};
            return fp16_mul_fields;
        end

        // ------------------------------------------------------------------
        // Normal / subnormal multiply
        // ------------------------------------------------------------------
        // Implicit leading 1 for normals, 0 for subnormals
        full_ma = (ea != 5'h00) ? {1'b1, ma} : {1'b0, ma};
        full_mb = (eb != 5'h00) ? {1'b1, mb} : {1'b0, mb};

        mant_prod = full_ma * full_mb;  // 22-bit result

        // Raw exponent in FP16 domain:
        //   normal:    ea + eb - 15 (FP16 bias)
        //   subnormal: treat effective exponent as 1 (not 0)
        raw_exp = signed'({5'b0, (ea != 5'h00) ? ea : 5'h01})
                + signed'({5'b0, (eb != 5'h00) ? eb : 5'h01})
                - signed'(10'd15);

        // Normalise the 22-bit product.
        // If mant_prod[21] is set, the product is in [2, 4) -> shift right,
        // increment exponent.  Otherwise product is in [1, 2) -> no shift.
        if (mant_prod[21]) begin
            prod_overflow = 1'b1;
            raw_exp       = raw_exp + signed'(10'd1);
        end else begin
            prod_overflow = 1'b0;
        end

        // Convert to FP32 exponent: add bias difference (127 - 15 = 112)
        raw_exp = raw_exp + signed'(10'd112);

        // Clamp / detect underflow / overflow
        if (raw_exp <= 0) begin
            // Flush to zero (simplified: no FP32 subnormal generation)
            fp16_mul_fields = {res_sign, 31'h0000_0000};
            return fp16_mul_fields;
        end else if (raw_exp >= signed'(10'd255)) begin
            // Overflow -> infinity
            fp16_mul_fields = {res_sign, 8'hFF, 23'h000000};
            return fp16_mul_fields;
        end

        fp32_exp = raw_exp[7:0];

        // Map mantissa bits into FP32 23-bit mantissa (drop implicit 1).
        // mant_prod is 22 bits.  After normalisation:
        //   overflow case: significant bits are mant_prod[20:0], need 23 bits
        //                  -> {mant_prod[20:0], 2'b00}
        //   normal  case:  significant bits are mant_prod[19:0], need 23 bits
        //                  -> {mant_prod[19:0], 3'b000}
        if (prod_overflow)
            fp32_mant = {mant_prod[20:0], 2'b00};
        else
            fp32_mant = {mant_prod[19:0], 3'b000};

        fp16_mul_fields = {res_sign, fp32_exp, fp32_mant};
    endfunction

    // =========================================================================
    // Function: FP32 add, first half (align stage).
    //   Replicates fp32_add_rne (fp16_utils_pkg) through the carry fold:
    //   unpack, magnitude sort, jam shift, add/subtract, same-sign carry fold.
    //   Packed result: {sign[37], exponent[36:29], normalized[28:2],
    //                   is_zero[1], diff_sign[0]}.
    // =========================================================================
    function automatic logic [37:0] fp32_add_align(
        input logic [31:0] a,
        input logic [31:0] b
    );
        logic        sign_a, sign_b, sign_result;
        logic [7:0]  exp_a, exp_b, exponent;
        logic [23:0] mant_a, mant_b, mant_large, mant_small;
        logic [26:0] ext_large, ext_small, normalized;
        logic [27:0] sum;
        logic        is_zero, diff_sign;
        begin
            sign_a = a[31]; sign_b = b[31];
            exp_a  = a[30:23]; exp_b = b[30:23];
            mant_a = (exp_a == 8'd0) ? {1'b0, a[22:0]} : {1'b1, a[22:0]};
            mant_b = (exp_b == 8'd0) ? {1'b0, b[22:0]} : {1'b1, b[22:0]};
            if (exp_a > exp_b || (exp_a == exp_b && mant_a >= mant_b)) begin
                exponent    = exp_a;
                mant_large  = mant_a;
                mant_small  = mant_b;
                sign_result = sign_a;
            end else begin
                exponent    = exp_b;
                mant_large  = mant_b;
                mant_small  = mant_a;
                sign_result = sign_b;
            end
            diff_sign = (sign_a != sign_b);
            is_zero   = 1'b0;
            if (mant_large == 24'd0) begin
                // Both operands are zero; fp32_add_rne returns +0.
                is_zero    = 1'b1;
                normalized = 27'd0;
            end else begin
                ext_large = {mant_large, 3'b000};
                ext_small = fp32_shift_right_jam({mant_small, 3'b000},
                                    (exp_a > exp_b) ? (exp_a - exp_b)
                                                    : (exp_b - exp_a));
                if (!diff_sign) begin
                    sum = {1'b0, ext_large} + {1'b0, ext_small};
                    if (sum[27]) begin
                        normalized    = sum[27:1];
                        normalized[0] = normalized[0] | sum[0];
                        exponent      = exponent + 8'd1;
                    end else begin
                        normalized = sum[26:0];
                    end
                end else begin
                    sum        = {1'b0, ext_large} - {1'b0, ext_small};
                    normalized = sum[26:0];
                    // Exact cancellation -> +0 (matches fp32_add_rne).
                    if (normalized == 27'd0)
                        is_zero = 1'b1;
                end
            end
            fp32_add_align = {sign_result, exponent, normalized,
                              is_zero, diff_sign};
        end
    endfunction

    // =========================================================================
    // Function: FP32 add, second half (normalize + round stage).
    //   Replicates the tail of fp32_add_rne: bounded left-shift
    //   normalization (subtractive path only), GRS round-to-nearest-even,
    //   and the final exponent bounds checks.
    // =========================================================================
    function automatic logic [31:0] fp32_add_finish(
        input logic [37:0] p
    );
        logic        sign_result, is_zero, diff_sign;
        logic [7:0]  exponent;
        logic [26:0] normalized;
        logic [24:0] rounded;
        logic        guard_bit, round_bit, sticky_bit;
        logic [4:0]  normalize_shift;
        begin
            sign_result = p[37];
            exponent    = p[36:29];
            normalized  = p[28:2];
            is_zero     = p[1];
            diff_sign   = p[0];
            if (is_zero) begin
                fp32_add_finish = 32'h0000_0000;
            end else begin
                if (diff_sign) begin
                    // Logarithmic LZC plus one barrel shift replaces the
                    // replicated 27-step conditional normalization chain.
                    normalize_shift = fp32_lzc27(normalized);
                    if (exponent < {3'b000, normalize_shift})
                        normalize_shift = exponent[4:0];
                    normalized = normalized << normalize_shift;
                    exponent = exponent - {3'b000, normalize_shift};
                end
                guard_bit  = normalized[2];
                round_bit  = normalized[1];
                sticky_bit = normalized[0];
                rounded    = {1'b0, normalized[26:3]};
                if (guard_bit && (round_bit || sticky_bit || rounded[0]))
                    rounded = rounded + 25'd1;
                if (rounded[24]) begin
                    rounded  = rounded >> 1;
                    exponent = exponent + 8'd1;
                end
                if (exponent == 8'd0)
                    fp32_add_finish = {sign_result, 31'b0};
                else if (exponent >= 8'hFF)
                    fp32_add_finish = {sign_result, 8'hFF, 23'b0};
                else
                    fp32_add_finish = {sign_result, exponent, rounded[22:0]};
            end
        end
    endfunction

endmodule

`default_nettype wire
