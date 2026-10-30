`default_nettype none

// One-element-per-cycle FP16 AdaLN-gated residual datapath.
module supra2_residual_gate (
    input  wire logic   clk,
    input  wire logic   rst_n,
    input  wire logic   in_valid,
    output logic        in_ready,
    input  wire logic [15:0] residual_fp16,
    input  wire logic [15:0] update_fp16,
    input  wire logic [15:0] gate_fp16,
    output logic        out_valid,
    input  wire logic   out_ready,
    output logic [15:0] out_fp16
);
    import fp16_utils_pkg::*;
    logic [15:0] result_reg;
    assign in_ready = !out_valid || out_ready;
    assign out_fp16 = result_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            result_reg <= 16'b0;
        end else if (in_ready) begin
            out_valid <= in_valid;
            if (in_valid)
                result_reg <= fp16_add(residual_fp16,
                                      fp16_mul(update_fp16, gate_fp16));
        end
    end
endmodule

`default_nettype wire
