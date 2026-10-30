// =============================================================================
// INT8 Multiply-Accumulate Unit (2-stage pipeline)
// =============================================================================
`default_nettype none

module mac_int8 #(
    parameter int DATA_W = 8,
    parameter int ACC_W  = 32,
    parameter bit UNSIGNED_A = 1'b0,
    parameter bit RUNTIME_UNSIGNED_A = 1'b0
)(
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        clear_acc,
    input  wire                        en,
    input  wire                        a_unsigned,
    input  wire  signed [DATA_W-1:0]   a_in,
    input  wire  signed [DATA_W-1:0]   b_in,
    output logic signed [ACC_W-1:0]    acc_out
);

    // Stage 1: multiply.  The product register is deliberately not gated by
    // en: en_d1 in the accumulator stage masks bubbles, while an output mux
    // here would sit after the DSP and prevent clean DSP PREG inference.
    localparam int PROD_W = 2*DATA_W + ((UNSIGNED_A || RUNTIME_UNSIGNED_A) ? 1 : 0);
    (* use_dsp = "yes" *) logic signed [PROD_W-1:0] product;
    logic                       en_d1;
    logic                       clear_d1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            en_d1    <= 1'b0;
            clear_d1 <= 1'b0;
        end else begin
            en_d1    <= en;
            clear_d1 <= clear_acc;
            if (RUNTIME_UNSIGNED_A)
                product <= $signed({(a_unsigned ? 1'b0 : a_in[DATA_W-1]), a_in}) * b_in;
            else if (UNSIGNED_A)
                product <= $signed({{(PROD_W-DATA_W){1'b0}}, a_in}) *
                           $signed({{(PROD_W-DATA_W){b_in[DATA_W-1]}}, b_in});
            else
                product <= a_in * b_in;
        end
    end

    // Stage 2: accumulate
    logic signed [ACC_W-1:0] acc_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_reg <= '0;
        end else if (clear_d1) begin
            acc_reg <= '0;
        end else if (en_d1) begin
            // Model K is at most 2304, so the worst-case signed 8-bit dot
            // product is < 2^26 and cannot overflow this INT32 accumulator.
            acc_reg <= acc_reg +
                {{(ACC_W-PROD_W){product[PROD_W-1]}}, product};
        end
    end

    assign acc_out = acc_reg;

endmodule

`default_nettype wire
