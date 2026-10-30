`default_nettype none

// Streaming DiCache Dynamic Cache Trajectory Alignment (DCTA).
// For each feature lane, estimate r_t = r_beta + gamma * (r_alpha-r_beta).
// FP16 inputs/outputs match the model buffers; intermediate arithmetic is
// FP32. Three registered arithmetic stages sustain one 32-feature beat/cycle.
module supra2_dicache_dcta_32lane #(
    parameter int LANES = 32
) (
    input  wire logic                 clk,
    input  wire logic                 rst_n,
    input  wire logic                 in_valid,
    output wire logic                 in_ready,
    input  wire logic                 in_last,
    input  wire logic [LANES*16-1:0]  residual_alpha_fp16,
    input  wire logic [LANES*16-1:0]  residual_beta_fp16,
    input  wire logic [15:0]          gamma_fp16,
    output wire logic                 out_valid,
    output logic                      out_last,
    input  wire logic                 out_ready,
    output logic [LANES*16-1:0]       residual_out_fp16
);
    import fp16_utils_pkg::*;

    logic valid0, valid1, valid2, valid3;
    logic last0, last1, last2, last3;
    logic [LANES*16-1:0] alpha0, beta0;
    logic [15:0] gamma0;
    wire [31:0] alpha0_fp32 [0:LANES-1];
    wire [31:0] beta0_fp32 [0:LANES-1];
    logic [31:0] diff1 [0:LANES-1];
    logic [31:0] beta1 [0:LANES-1];
    logic [31:0] gamma1;
    logic [31:0] product2 [0:LANES-1];
    logic [31:0] beta2 [0:LANES-1];
    wire ready3 = !valid3 || out_ready;
    wire ready2 = !valid2 || ready3;
    wire ready1 = !valid1 || ready2;

    assign in_ready = !valid0 || ready1;
    assign out_valid = valid3;

    for (genvar lane = 0; lane < LANES; lane++) begin : g_fp16_convert
        assign alpha0_fp32[lane] = fp16_to_fp32(alpha0[lane*16 +: 16]);
        assign beta0_fp32[lane] = fp16_to_fp32(beta0[lane*16 +: 16]);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid0 <= 1'b0;
            valid1 <= 1'b0;
            valid2 <= 1'b0;
            valid3 <= 1'b0;
            last0 <= 1'b0;
            last1 <= 1'b0;
            last2 <= 1'b0;
            last3 <= 1'b0;
            alpha0 <= '0;
            beta0 <= '0;
            gamma0 <= '0;
            gamma1 <= '0;
            residual_out_fp16 <= '0;
            out_last <= 1'b0;
            for (int lane = 0; lane < LANES; lane++) begin
                diff1[lane] <= '0;
                beta1[lane] <= '0;
                product2[lane] <= '0;
                beta2[lane] <= '0;
            end
        end else begin
            if (ready3) begin
                valid3 <= valid2;
                last3 <= last2;
                if (valid2) begin
                    for (int lane = 0; lane < LANES; lane++)
                        residual_out_fp16[lane*16 +: 16] <= fp32_to_fp16(
                            fp32_add_rne(beta2[lane], product2[lane]));
                    out_last <= last2;
                end
            end
            if (ready2) begin
                valid2 <= valid1;
                last2 <= last1;
                if (valid1) begin
                    for (int lane = 0; lane < LANES; lane++)
                        product2[lane] <= fp32_mul_rne(
                            gamma1, diff1[lane]);
                    for (int lane = 0; lane < LANES; lane++)
                        beta2[lane] <= beta1[lane];
                end
            end
            if (ready1) begin
                valid1 <= valid0;
                last1 <= last0;
                if (valid0) begin
                    gamma1 <= fp16_to_fp32(gamma0);
                    for (int lane = 0; lane < LANES; lane++) begin
                        diff1[lane] <= fp32_add_rne(
                            alpha0_fp32[lane],
                            {~beta0_fp32[lane][31], beta0_fp32[lane][30:0]});
                        beta1[lane] <= beta0_fp32[lane];
                    end
                end
            end
            if (in_ready) begin
                valid0 <= in_valid;
                last0 <= in_valid && in_last;
                if (in_valid) begin
                    alpha0 <= residual_alpha_fp16;
                    beta0 <= residual_beta_fp16;
                    gamma0 <= gamma_fp16;
                end
            end
        end
    end

    initial if (LANES <= 0) $error("DCTA LANES must be positive");
endmodule

`default_nettype wire
