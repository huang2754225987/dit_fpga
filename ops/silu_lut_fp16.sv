// =============================================================================
// silu_lut_fp16.sv - Sigmoid LUT for SiLU (FP16 indexed, FP16 output)
// =============================================================================
// 256-entry LUT: input = upper 8 bits of FP16 value (sign+exp+top2mant)
// output = FP16 sigmoid(x) approximation
// SiLU engine multiplies x * sigmoid(x) externally.
// =============================================================================
`default_nettype none

module silu_lut_fp16 (
    input  wire        clk,
    input  wire  [7:0] addr,
    output logic [15:0] data_out
);

    (* rom_style = "block" *)
    logic [15:0] rom [0:255];

    initial begin
        // sigmoid(x) for x encoded as FP16 upper 8 bits
        // Index [7] = sign, [6:2] = exponent, [1:0] = top 2 mantissa bits
        // For negative large x: sigmoid ~ 0, for positive large: sigmoid ~ 1
        // FP16 1.0 = 16'h3C00, 0.5 = 16'h3800, 0.0 = 16'h0000

        // Initialize all to 0.5 (midpoint)
        for (int i = 0; i < 256; i++)
            rom[i] = 16'h3800;  // 0.5

        // Positive values: sigmoid approaches 1.0
        // exp=15 (bias), mant=0 => x=1.0, sigmoid(1.0)~0.731
        rom[8'h3C] = 16'h3DD8;  // ~0.731
        rom[8'h3D] = 16'h3E48;  // ~0.785
        rom[8'h3E] = 16'h3EA0;  // ~0.831
        rom[8'h3F] = 16'h3EE0;  // ~0.869
        // exp=16: x=2..3.5, sigmoid -> 0.88..0.97
        rom[8'h40] = 16'h3F10;  // ~0.881
        rom[8'h41] = 16'h3F40;  // ~0.924
        rom[8'h42] = 16'h3F60;  // ~0.953
        rom[8'h43] = 16'h3F78;  // ~0.971
        // exp>=17: x>=4, sigmoid ~= 1.0
        for (int i = 8'h44; i < 8'h80; i++)
            rom[i] = 16'h3C00;  // 1.0

        // Negative values: sigmoid approaches 0.0
        // sign=1, exp=15, mant=0 => x=-1.0, sigmoid(-1.0)~0.269
        rom[8'hBC] = 16'h344E;  // ~0.269
        rom[8'hBD] = 16'h31B0;  // ~0.215
        rom[8'hBE] = 16'h2D80;  // ~0.169
        rom[8'hBF] = 16'h2880;  // ~0.131
        // exp=16: x=-2..-3.5, sigmoid -> 0.12..0.03
        rom[8'hC0] = 16'h2600;  // ~0.119
        rom[8'hC1] = 16'h2100;  // ~0.076
        rom[8'hC2] = 16'h1C00;  // ~0.047
        rom[8'hC3] = 16'h1800;  // ~0.029
        // exp>=17: x<=-4, sigmoid ~= 0.0
        for (int i = 8'hC4; i <= 8'hFF; i++)
            rom[i] = 16'h0000;  // 0.0

        // Small positive (exp<15): x in (0,1), sigmoid in (0.5, 0.731)
        rom[8'h38] = 16'h3900;  // x~0.5, sigmoid~0.622
        rom[8'h39] = 16'h3960;  // x~0.625, sigmoid~0.650
        rom[8'h3A] = 16'h39C0;  // x~0.75, sigmoid~0.679
        rom[8'h3B] = 16'h3A20;  // x~0.875, sigmoid~0.706

        // Small negative (sign=1, exp<15): x in (-1,0), sigmoid in (0.269, 0.5)
        rom[8'hB8] = 16'h3700;  // x~-0.5, sigmoid~0.378
        rom[8'hB9] = 16'h36A0;  // x~-0.625, sigmoid~0.350
        rom[8'hBA] = 16'h3640;  // x~-0.75, sigmoid~0.321
        rom[8'hBB] = 16'h35E0;  // x~-0.875, sigmoid~0.294

        // Zero region
        rom[8'h00] = 16'h3800;  // +0 -> 0.5
        rom[8'h80] = 16'h3800;  // -0 -> 0.5
    end

    always_ff @(posedge clk)
        data_out <= rom[addr];

endmodule

`default_nettype wire
