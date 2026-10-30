`default_nettype none

// Streaming classifier-free guidance combine:
// v_cfg = v_uncond + guidance * (v_cond - v_uncond).
// Inputs and output are FP16; intermediate arithmetic is FP32.
module supra2_cfg_combine_32lane #(
    parameter int LANES = 32,
    parameter int COUNT_W = 13
) (
    input  wire logic                   clk,
    input  wire logic                   rst_n,
    input  wire logic                   start,
    input  wire logic [COUNT_W-1:0]     vector_elements,
    input  wire logic [15:0]            guidance_fp16,
    input  wire logic                   in_valid,
    output logic                        in_ready,
    input  wire logic [LANES*16-1:0]    velocity_uncond_fp16,
    input  wire logic [LANES*16-1:0]    velocity_cond_fp16,
    output logic                        out_valid,
    input  wire logic                   out_ready,
    output logic [LANES*16-1:0]         velocity_cfg_fp16,
    output logic                        busy,
    output logic                        done
);
    import fp16_utils_pkg::*;

    typedef enum logic {S_IDLE, S_RUN} state_t;
    state_t state;
    logic [COUNT_W-1:0] length_reg;
    logic [COUNT_W-1:0] element_count;
    logic [15:0] guidance_reg;

    function automatic logic [15:0] combine_lane(
        input logic [15:0] uncond_value,
        input logic [15:0] cond_value,
        input logic [15:0] guidance_value
    );
        logic [31:0] delta;
        logic [31:0] scaled_delta;
        begin
            delta = fp32_add_rne(fp16_to_fp32(cond_value),
                fp16_to_fp32(uncond_value) ^ 32'h8000_0000);
            scaled_delta = fp32_mul_rne(delta, fp16_to_fp32(guidance_value));
            return fp32_to_fp16(fp32_add_rne(
                fp16_to_fp32(uncond_value), scaled_delta));
        end
    endfunction

    always_comb begin
        in_ready = (state == S_RUN) && out_ready;
        out_valid = (state == S_RUN) && in_valid;
        busy = (state == S_RUN);
        done = out_valid && out_ready &&
               (element_count + LANES >= length_reg);
        for (int lane = 0; lane < LANES; lane++) begin
            velocity_cfg_fp16[lane*16 +: 16] = combine_lane(
                velocity_uncond_fp16[lane*16 +: 16],
                velocity_cond_fp16[lane*16 +: 16], guidance_reg);
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            length_reg <= '0;
            element_count <= '0;
            guidance_reg <= '0;
        end else begin
            case (state)
                S_IDLE: if (start && vector_elements != 0 &&
                            vector_elements % LANES == 0) begin
                    length_reg <= vector_elements;
                    element_count <= '0;
                    guidance_reg <= guidance_fp16;
                    state <= S_RUN;
                end
                S_RUN: if (out_valid && out_ready) begin
                    if (element_count + LANES >= length_reg) begin
                        element_count <= '0;
                        state <= S_IDLE;
                    end else begin
                        element_count <= element_count + LANES;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    initial begin
        if (LANES <= 0 || (LANES % 2) != 0)
            $error("CFG combine LANES must be a positive even number");
    end
endmodule

`default_nettype wire
