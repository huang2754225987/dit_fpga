`default_nettype none

module supra2_position_add (
    input  wire logic        clk,
    input  wire logic        rst_n,
    input  wire logic        in_valid,
    output logic             in_ready,
    input  wire logic [15:0] token_fp16,
    input  wire logic [15:0] position_fp16,
    output logic             out_valid,
    input  wire logic        out_ready,
    output logic [15:0]      out_fp16
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
                result_reg <= fp16_add_ieee(token_fp16, position_fp16);
        end
    end
endmodule

`default_nettype wire
