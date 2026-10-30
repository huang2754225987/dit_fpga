`default_nettype none

// Token-wise FP16 LayerNorm and AdaLN modulation for Supra2 block inputs.
// AdaLN shift/scale are supplied by the host-precompute partition; normalization
// statistics use a shared iterative FP32 datapath.
module supra2_layernorm_modulate #(
    parameter int FEATURES = 576,
    parameter int COUNT_W = 10
) (
    input  wire logic             clk,
    input  wire logic             rst_n,
    input  wire logic             start,
    input  wire logic             in_valid,
    output logic                  in_ready,
    input  wire logic [15:0]      x_fp16,
    input  wire logic [15:0]      shift_fp16,
    input  wire logic [15:0]      scale_fp16,
    output logic                  out_valid,
    input  wire logic             out_ready,
    output logic [15:0]           out_fp16,
    output logic                  busy,
    output logic                  done
);
    import fp16_utils_pkg::*;

    typedef enum logic [2:0] {S_IDLE, S_LOAD, S_CALC, S_OUTPUT} state_t;
    state_t state;
    logic [15:0] x_mem [0:FEATURES-1];
    logic [15:0] shift_mem [0:FEATURES-1];
    logic [15:0] scale_mem [0:FEATURES-1];
    logic [COUNT_W-1:0] load_index, output_index;
    logic [31:0] sum_fp32, sumsq_fp32;
    logic [31:0] mean_fp32, invstd_fp32;

    localparam logic [31:0] INV_FEATURES_FP32 = 32'h3ae38e39; // 1/576
    localparam logic [31:0] EPSILON_FP32 = 32'h358637bd;      // 1e-6
    localparam logic [31:0] HALF_FP32 = 32'h3f000000;
    localparam logic [31:0] ONE_AND_HALF_FP32 = 32'h3fc00000;

    function automatic logic [31:0] fp32_rsqrt_nr(input logic [31:0] value);
        logic [31:0] estimate, half_value, estimate_sq, correction_term;
        logic [31:0] correction;
        begin
            estimate = 32'h5f3759df - (value >> 1);
            half_value = fp32_mul_rne(value, HALF_FP32);
            for (int iteration = 0; iteration < 3; iteration++) begin
                estimate_sq = fp32_mul_rne(estimate, estimate);
                correction_term = fp32_mul_rne(half_value, estimate_sq);
                correction = fp32_add_rne(ONE_AND_HALF_FP32,
                                           correction_term ^ 32'h80000000);
                estimate = fp32_mul_rne(estimate, correction);
            end
            return estimate;
        end
    endfunction

    always_comb begin : output_datapath
        logic [31:0] centered_fp32, normalized_fp32;
        logic [15:0] normalized_fp16, scale_factor_fp16;
        in_ready = (state == S_LOAD);
        out_valid = (state == S_OUTPUT);
        busy = (state != S_IDLE);
        done = out_valid && out_ready && (output_index == FEATURES-1);
        out_fp16 = 16'b0;
        if (state == S_OUTPUT) begin
            centered_fp32 = fp32_add_rne(
                fp16_to_fp32(x_mem[output_index]), mean_fp32 ^ 32'h80000000);
            normalized_fp32 = fp32_mul_rne(centered_fp32, invstd_fp32);
            normalized_fp16 = fp32_to_fp16(normalized_fp32);
            scale_factor_fp16 = fp16_add(16'h3c00, scale_mem[output_index]);
            out_fp16 = fp16_add(
                fp16_mul(normalized_fp16, scale_factor_fp16),
                shift_mem[output_index]);
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        logic [31:0] x_value_fp32, x_square_fp32;
        logic [31:0] mean_square_fp32, average_square_fp32, variance_fp32;
        if (!rst_n) begin
            state <= S_IDLE;
            load_index <= '0;
            output_index <= '0;
            sum_fp32 <= '0;
            sumsq_fp32 <= '0;
            mean_fp32 <= '0;
            invstd_fp32 <= '0;
        end else begin
            case (state)
                S_IDLE: if (start) begin
                    load_index <= '0;
                    output_index <= '0;
                    sum_fp32 <= '0;
                    sumsq_fp32 <= '0;
                    state <= S_LOAD;
                end
                S_LOAD: if (in_valid && in_ready) begin
                    x_mem[load_index] <= x_fp16;
                    shift_mem[load_index] <= shift_fp16;
                    scale_mem[load_index] <= scale_fp16;
                    x_value_fp32 = fp16_to_fp32(x_fp16);
                    x_square_fp32 = fp32_mul_rne(x_value_fp32, x_value_fp32);
                    sum_fp32 <= fp32_add_rne(sum_fp32, x_value_fp32);
                    sumsq_fp32 <= fp32_add_rne(sumsq_fp32, x_square_fp32);
                    if (load_index == FEATURES-1) begin
                        state <= S_CALC;
                    end else load_index <= load_index + 1'b1;
                end
                S_CALC: begin
                    mean_fp32 <= fp32_mul_rne(sum_fp32, INV_FEATURES_FP32);
                    average_square_fp32 = fp32_mul_rne(sumsq_fp32,
                                                        INV_FEATURES_FP32);
                    mean_square_fp32 = fp32_mul_rne(
                        fp32_mul_rne(sum_fp32, INV_FEATURES_FP32),
                        fp32_mul_rne(sum_fp32, INV_FEATURES_FP32));
                    variance_fp32 = fp32_add_rne(
                        average_square_fp32, mean_square_fp32 ^ 32'h80000000);
                    if (variance_fp32[31]) variance_fp32 = 32'b0;
                    invstd_fp32 <= fp32_rsqrt_nr(
                        fp32_add_rne(variance_fp32, EPSILON_FP32));
                    output_index <= '0;
                    state <= S_OUTPUT;
                end
                S_OUTPUT: if (out_valid && out_ready) begin
                    if (output_index == FEATURES-1) begin
                        output_index <= '0;
                        state <= S_IDLE;
                    end else output_index <= output_index + 1'b1;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    initial begin
        if (FEATURES <= 0 || (1 << COUNT_W) <= FEATURES)
            $error("Invalid Supra2 LayerNorm feature count");
    end
endmodule

`default_nettype wire
