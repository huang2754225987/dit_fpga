`default_nettype none

// 32-lane GELU(tanh) for the MLP.0 epilogue drain.
// Every lane replicates the exact address computation and 4K-entry FP16 ROM
// of the verified 1-element supra2_gelu_tanh unit, so outputs are bit-identical
// to running the scalar unit per element. One 32-wide beat per cycle:
// in_valid -> out_valid one cycle later (single pipeline stage, same timing
// as the scalar unit). Intended to sit on the accumulator drain of the
// MLP.0 GEMM, eliminating the standalone 1-cyc/elem GELU pass.
module supra2_gelu_tanh_32lane #(
    parameter int LANES    = 32,
    parameter int LUT_ADDR_W = 12,
    parameter     LUT_FILE = "rtl/supra2/gelu_tanh_lut_fp16.hex"
) (
    input  wire logic                 clk,
    input  wire logic                 rst_n,
    input  wire logic                 in_valid,
    input  wire logic [LANES*16-1:0]  in_fp16,
    output logic                      out_valid,
    output logic [LANES*16-1:0]       out_fp16
);

    logic [15:0] lut [0:(1<<LUT_ADDR_W)-1];
    initial $readmemh(LUT_FILE, lut);

    // per-lane combinational lookup, identical to supra2_gelu_tanh
    logic [15:0] lane_result [0:LANES-1];
    for (genvar l = 0; l < LANES; l++) begin : g_lane
        logic [15:0] x;
        integer exponent, shift;
        logic [10:0] significand;
        logic [31:0] magnitude;
        logic [11:0] address;
        always_comb begin
            x = in_fp16[l*16 +: 16];
            exponent = x[14:10];
            significand = {1'b1, x[9:0]};
            shift = exponent - 17;
            magnitude = 0;
            address = 12'd2048;
            lane_result[l] = 16'h0000;
            if (exponent == 5'h1f) begin
                lane_result[l] = x[15] ? 16'h0000 : x;
            end else begin
                if (exponent != 0) begin
                    if (shift >= 12) magnitude = 32'hffff_ffff;
                    else if (shift >= 0) magnitude = {21'b0,significand} << shift;
                    else magnitude = {21'b0,significand} >> (-shift);
                end
                if (magnitude >= 32'd2048) begin
                    lane_result[l] = x[15] ? 16'h0000 : x;
                end else begin
                    address = x[15] ? 12'(2048-magnitude) : 12'(2048+magnitude);
                    lane_result[l] = lut[address];
                end
            end
        end
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
