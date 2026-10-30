`default_nettype none

// Model GELU(tanh) approximation using a 4K-entry FP16 ROM over [-8, 8).
// Inputs outside this interval saturate to 0 (negative) or pass through
// (positive), where the approximation is already at its asymptote.
module supra2_gelu_tanh #(
    parameter int LUT_ADDR_W=12,
    parameter LUT_FILE="rtl/supra2/gelu_tanh_lut_fp16.hex"
) (
    input  wire logic        clk,
    input  wire logic        rst_n,
    input  wire logic        in_valid,
    output logic             in_ready,
    input  wire logic [15:0] in_fp16,
    output logic             out_valid,
    input  wire logic        out_ready,
    output logic [15:0]      out_fp16
);
    logic [15:0] lut [0:(1<<LUT_ADDR_W)-1];
    logic [15:0] result, result_reg;
    logic [11:0] address;
    integer exponent, shift;
    logic [10:0] significand;
    logic [31:0] magnitude;

    initial $readmemh(LUT_FILE,lut);

    always_comb begin
        exponent = in_fp16[14:10];
        significand = {1'b1,in_fp16[9:0]};
        shift = exponent - 17;
        magnitude = 0;
        address = 12'd2048;
        result = 16'h0000;
        if (exponent == 5'h1f) begin
            result = in_fp16[15] ? 16'h0000 : in_fp16;
        end else begin
            if (exponent != 0) begin
                if (shift >= 12) magnitude = 32'hffff_ffff;
                else if (shift >= 0) magnitude = {21'b0,significand} << shift;
                else magnitude = {21'b0,significand} >> (-shift);
            end
            if (magnitude >= 32'd2048) begin
                result = in_fp16[15] ? 16'h0000 : in_fp16;
            end else begin
                address = in_fp16[15] ? 12'(2048-magnitude) : 12'(2048+magnitude);
                result = lut[address];
            end
        end
    end

    assign in_ready = !out_valid || out_ready;
    assign out_fp16 = result_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            result_reg <= 16'h0000;
        end else if (in_ready) begin
            out_valid <= in_valid;
            if (in_valid) result_reg <= result;
        end
    end
endmodule

`default_nettype wire
