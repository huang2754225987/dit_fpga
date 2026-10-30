`default_nettype none

// 32-lane AdaLN-gated residual for the post-attention / post-MLP epilogues.
// Bit-identical per element to the verified 1-element supra2_residual_gate
// (fp16_add(residual, fp16_mul(update, gate))). One 32-wide beat per cycle:
// in_valid -> out_valid one cycle later. The gate beat for the current
// feature group is supplied alongside residual/update; the producer holds
// the 576-entry per-feature gate vector in a small register file and cycles
// it every FEATURES/LANES beats.
module supra2_residual_gate_32lane #(
    parameter int LANES = 32
) (
    input  wire logic                clk,
    input  wire logic                rst_n,
    input  wire logic                in_valid,
    input  wire logic [LANES*16-1:0] residual_fp16,
    input  wire logic [LANES*16-1:0] update_fp16,
    input  wire logic [LANES*16-1:0] gate_fp16,
    output logic                     out_valid,
    output logic [LANES*16-1:0]      out_fp16
);
    import fp16_utils_pkg::*;

    logic [15:0] lane_result [0:LANES-1];
    for (genvar l = 0; l < LANES; l++) begin : g_lane
        always_comb
            lane_result[l] = fp16_add(residual_fp16[l*16 +: 16],
                                      fp16_mul(update_fp16[l*16 +: 16],
                                               gate_fp16[l*16 +: 16]));
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            out_fp16  <= '0;
        end else begin
            out_valid <= in_valid;
            if (in_valid) begin
                for (int l = 0; l < LANES; l++)
                    out_fp16[l*16 +: 16] <= lane_result[l];
            end
        end
    end

endmodule

`default_nettype wire
