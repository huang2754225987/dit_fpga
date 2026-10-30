`default_nettype none

// Streaming single-precision-step Euler-flow update using FP16 arithmetic.
// The denoiser supplies v; this unit performs z_next = z + dt * v.
module supra2_euler_update #(
    parameter int LANES = 32,
    parameter int COUNT_W = 13
) (
    input  wire logic                    clk,
    input  wire logic                    rst_n,
    input  wire logic                    start,
    input  wire logic [COUNT_W-1:0]      vector_elements,
    input  wire logic [15:0]             step_size_fp16,
    input  wire logic                    in_valid,
    output logic                         in_ready,
    input  wire logic [LANES*16-1:0]     latent_fp16,
    input  wire logic [LANES*16-1:0]     velocity_fp16,
    output logic                         out_valid,
    input  wire logic                    out_ready,
    output logic [LANES*16-1:0]          updated_latent_fp16,
    output logic                         busy,
    output logic                         done
);
    import fp16_utils_pkg::*;

    typedef enum logic {S_IDLE, S_RUN} state_t;
    state_t state;
    logic [COUNT_W-1:0] length_reg;
    logic [COUNT_W-1:0] element_count;
    logic [15:0] dt_reg;

    always_comb begin
        in_ready = (state == S_RUN) && out_ready;
        out_valid = (state == S_RUN) && in_valid;
        busy = (state == S_RUN);
        done = out_valid && out_ready &&
               (element_count + LANES >= length_reg);
        for (int lane = 0; lane < LANES; lane++) begin
            updated_latent_fp16[lane*16 +: 16] = fp32_to_fp16(
                fp32_add_rne(
                    fp16_to_fp32(latent_fp16[lane*16 +: 16]),
                    fp16_to_fp32(fp32_to_fp16(fp32_mul_rne(
                        fp16_to_fp32(velocity_fp16[lane*16 +: 16]),
                        fp16_to_fp32(dt_reg))))));
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            length_reg <= '0;
            element_count <= '0;
            dt_reg <= '0;
        end else begin
            case (state)
                S_IDLE: if (start && vector_elements != 0 &&
                            vector_elements % LANES == 0) begin
                    length_reg <= vector_elements;
                    element_count <= '0;
                    dt_reg <= step_size_fp16;
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
            $error("Euler update LANES must be a positive even number");
    end
endmodule

`default_nettype wire
