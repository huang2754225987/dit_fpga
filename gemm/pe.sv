// =============================================================================
// Processing Element for Systolic Array
// Passes data east (a) and south (b) with 1-cycle delay
// =============================================================================
`default_nettype none

module pe #(
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
    // INT8 data from west / north
    input  wire  signed [DATA_W-1:0]   a_in,
    input  wire  signed [DATA_W-1:0]   b_in,
    // INT8 data to east / south (delayed 1 cycle)
    output logic signed [DATA_W-1:0]   a_out,
    output logic signed [DATA_W-1:0]   b_out,
    // INT32 accumulator result
    output logic signed [ACC_W-1:0]    acc_out
);

    // Clear wavefront state independently of en: rows masked by active-M can
    // otherwise retain stale values and feed them when a later GEMM grows M.
    // INT8 pass-through registers
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_out <= '0;
            b_out <= '0;
        end else if (clear_acc) begin
            a_out <= '0;
            b_out <= '0;
        end else if (en) begin
            a_out <= a_in;
            b_out <= b_in;
        end
    end

    // INT8 MAC unit
    logic signed [ACC_W-1:0] int8_acc;
    mac_int8 #(
        .DATA_W (DATA_W),
        .ACC_W  (ACC_W),
        .UNSIGNED_A (UNSIGNED_A),
        .RUNTIME_UNSIGNED_A (RUNTIME_UNSIGNED_A)
    ) u_mac_int8 (
        .clk       (clk),
        .rst_n     (rst_n),
        .clear_acc (clear_acc),
        .en        (en),
        .a_unsigned(a_unsigned),
        .a_in      (a_in),
        .b_in      (b_in),
        .acc_out   (int8_acc)
    );

    assign acc_out = int8_acc;

endmodule

`default_nettype wire
