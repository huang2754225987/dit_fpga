`default_nettype none
// Synthesis diagnostic for the shared binary32 RNE addition helper.
module fp32_add_synth_probe (
    input wire clk,
    input wire [31:0] a, b,
    output logic [31:0] result
);
    logic [31:0] a_reg, b_reg;
    always_ff @(posedge clk) begin
        a_reg <= a;
        b_reg <= b;
        result <= fp16_utils_pkg::fp32_add_rne(a_reg,b_reg);
    end
endmodule

module fp32_add_pipeline_synth_probe (
    input wire clk,
    input wire rst_n,
    input wire in_valid,
    input wire [31:0] a, b,
    output wire out_valid,
    output wire [31:0] result
);
    supra2_fp32_add_rne_pipeline u_pipeline (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid), .a(a), .b(b),
        .out_valid(out_valid), .result(result)
    );
endmodule
`default_nettype wire
