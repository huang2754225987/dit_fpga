`default_nettype none

// Streaming per-token INT8 quantizer for FP16 rotated activations.
// The complete token vector is buffered for the absmax pass, then emitted in
// LANES-wide beats. A single reciprocal divider is shared by all output lanes.
module supra2_activation_quantizer #(
    parameter int MAX_K = 2304,
    parameter int LANES = 32,
    parameter int K_W = $clog2(MAX_K + 1)
) (
    input  wire logic                 clk,
    input  wire logic                 rst_n,
    input  wire logic                 start,
    input  wire logic [K_W-1:0]       vector_k,
    input  wire logic                 in_valid,
    output logic                      in_ready,
    input  wire logic [LANES*16-1:0]  in_fp16,
    output logic                      out_valid,
    input  wire logic                 out_ready,
    output logic [LANES*8-1:0]        out_int8,
    output logic [31:0]               scale_fp32,
    output logic                      busy,
    output logic                      done
);
    import fp16_utils_pkg::*;

    localparam int COUNT_W = $clog2(MAX_K + LANES + 1);
    typedef enum logic [1:0] {S_IDLE, S_LOAD, S_RECIP, S_OUTPUT} state_t;
    state_t state;

    localparam int INPUT_BEATS = (MAX_K + LANES - 1) / LANES;
    logic [LANES*16-1:0] activation_mem [0:INPUT_BEATS-1];
    logic [LANES*16-1:0] activation_word_reg;
    logic [K_W-1:0] k_reg;
    logic [COUNT_W-1:0] input_count, output_count;
    logic [14:0] max_abs_fp16, max_abs_next;
    logic [40:0] reciprocal_q40;
    logic [10:0] reciprocal_remainder;
    logic [40:0] reciprocal_candidate;
    logic [10:0] reciprocal_remainder_candidate;
    logic [10:0] max_significand;
    logic [15:0] max_abs_word;
    logic [15:0] scale_fp16;

    function automatic logic [7:0] quantize_lane(
        input logic [15:0] value,
        input logic [14:0] max_abs,
        input logic [40:0] reciprocal,
        input logic [10:0] reciprocal_rem
    );
        logic [14:0] abs_value;
        logic [10:0] value_significand, max_sig;
        logic [4:0] value_exp, max_exp;
        logic [7:0] exponent_delta;
        logic [6:0] shift_amount;
        logic [63:0] numerator, quotient, remainder, half_unit;
        logic [63:0] correction, distance_scaled;
        logic round_up;
        logic [7:0] magnitude;
        begin
            abs_value = {1'b0, value[14:0]};
            value_exp = abs_value[14:10];
            max_exp = max_abs[14:10];
            if (max_abs == 0 || abs_value == 0 || value_exp == 0) begin
                quantize_lane = 8'h00;
            end else begin
                value_significand = {1'b1, abs_value[9:0]};
                max_sig = {1'b1, max_abs[9:0]};
                if (value_exp > max_exp) begin
                    // An input larger than the detected maximum indicates a
                    // corrupted vector; saturate safely instead of wrapping.
                    magnitude = 8'd127;
                end else begin
                    exponent_delta = max_exp - value_exp;
                    numerator = value_significand * 64'd127 * reciprocal;
                    shift_amount = 7'd40 + exponent_delta;
                    if (shift_amount >= 64) begin
                        magnitude = 0;
                    end else begin
                        quotient = numerator >> shift_amount;
                        remainder = numerator & ((64'd1 << shift_amount) - 1'b1);
                        half_unit = 64'd1 << (shift_amount - 1'b1);
                        correction = value_significand * 64'd127 * reciprocal_rem;
                        round_up = 1'b0;
                        if (remainder > half_unit) begin
                            round_up = 1'b1;
                        end else if (remainder == half_unit) begin
                            round_up = (correction != 0) || quotient[0];
                        end else begin
                            distance_scaled = (half_unit - remainder) * max_sig;
                            if (correction > distance_scaled)
                                round_up = 1'b1;
                            else if (correction == distance_scaled)
                                round_up = quotient[0];
                        end
                        if (round_up)
                            quotient = quotient + 1'b1;
                        magnitude = (quotient > 127) ? 8'd127 : quotient[7:0];
                    end
                end
                quantize_lane = value[15] ? -magnitude : magnitude;
            end
        end
    endfunction

    always_comb begin
        max_abs_next = max_abs_fp16;
        for (int lane = 0; lane < LANES; lane++) begin
            logic [14:0] candidate;
            candidate = {1'b0, in_fp16[lane*16 +: 15]};
            if (candidate > max_abs_next)
                max_abs_next = candidate;
        end
    end

    always_comb begin
        max_significand = {1'b1, max_abs_fp16[9:0]};
        reciprocal_candidate = '0;
        reciprocal_remainder_candidate = '0;
        if (max_abs_fp16[14:10] != 0) begin
            reciprocal_candidate = (41'd1 << 40) / max_significand;
            reciprocal_remainder_candidate = (41'd1 << 40) -
                reciprocal_candidate * max_significand;
        end
    end

    always_comb begin
        max_abs_word = {1'b0, max_abs_fp16};
        scale_fp16 = fp16_mul(max_abs_word, 16'h2008);
        in_ready = (state == S_LOAD);
        out_valid = (state == S_OUTPUT);
        busy = (state != S_IDLE);
        done = (state == S_OUTPUT) && out_ready &&
               (output_count + LANES >= k_reg);
        scale_fp32 = 32'b0;
        if (max_abs_fp16[14:10] != 0 && scale_fp16[14:10] != 0) begin
            // 1/127 rounded to FP16, then exactly widened to FP32.
            scale_fp32 = {scale_fp16[15],
                          (scale_fp16[14:10] + 8'd112),
                          scale_fp16[9:0], 13'b0};
        end
        for (int lane = 0; lane < LANES; lane++) begin
            out_int8[lane*8 +: 8] = 8'b0;
            if (state == S_OUTPUT && output_count + lane < k_reg)
                out_int8[lane*8 +: 8] = quantize_lane(
                    activation_word_reg[lane*16 +: 16], max_abs_fp16,
                    reciprocal_q40, reciprocal_remainder);
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            k_reg <= '0;
            input_count <= '0;
            output_count <= '0;
            max_abs_fp16 <= '0;
            reciprocal_q40 <= '0;
            reciprocal_remainder <= '0;
            activation_word_reg <= '0;
        end else begin
            case (state)
                S_IDLE: if (start) begin
                    if (vector_k == 0 || vector_k > MAX_K) begin
                        state <= S_IDLE;
                    end else begin
                        k_reg <= vector_k;
                        input_count <= '0;
                        output_count <= '0;
                        max_abs_fp16 <= '0;
                        reciprocal_q40 <= '0;
                        reciprocal_remainder <= '0;
                    activation_word_reg <= '0;
                        state <= S_LOAD;
                    end
                end
                S_LOAD: if (in_valid && in_ready) begin
                    activation_mem[input_count / LANES] <= in_fp16;
                    max_abs_fp16 <= max_abs_next;
                    input_count <= input_count + LANES;
                    if (input_count + LANES >= k_reg)
                        state <= S_RECIP;
                end
                S_RECIP: begin
                    activation_word_reg <= activation_mem[0];
                    if (max_abs_fp16[14:10] == 0) begin
                        reciprocal_q40 <= '0;
                        reciprocal_remainder <= '0;
                    end else begin
                        reciprocal_q40 <= reciprocal_candidate;
                        reciprocal_remainder <= reciprocal_remainder_candidate;
                    end
                    state <= S_OUTPUT;
                end
                S_OUTPUT: if (out_valid && out_ready) begin
                    if (output_count + LANES >= k_reg) begin
                        state <= S_IDLE;
                        output_count <= '0;
                    end else begin
                        activation_word_reg <=
                            activation_mem[(output_count / LANES) + 1'b1];
                        output_count <= output_count + LANES;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    initial begin
        if (LANES <= 0 || MAX_K < LANES)
            $error("MAX_K must be at least one LANES beat");
    end
endmodule

`default_nettype wire
