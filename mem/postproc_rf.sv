// =============================================================================
// postproc_rf.sv - Post-Processing Register File (1KB, 2R1W)
// =============================================================================
// 16 vector registers (v0-v15), each 32 elements x 16-bit FP16 = 64 bytes.
// Implemented as distributed RAM for multi-port support.
// 2 read ports (for dual-operand instructions) + 1 write port.
// =============================================================================
`default_nettype none

module postproc_rf #(
    parameter int NUM_REGS   = 16,
    parameter int VEC_LANES  = 32,
    parameter int ELEM_W     = 16,
    parameter int REG_ADDR_W = 4     // log2(16)
) (
    input  wire                          clk,
    input  wire                          rst_n,

    // Write port
    input  wire                          wr_en,
    input  wire  [REG_ADDR_W-1:0]       wr_addr,
    input  wire  [VEC_LANES*ELEM_W-1:0] wr_data,

    // Read port A
    input  wire  [REG_ADDR_W-1:0]       rd_addr_a,
    output logic [VEC_LANES*ELEM_W-1:0] rd_data_a,

    // Read port B
    input  wire  [REG_ADDR_W-1:0]       rd_addr_b,
    output logic [VEC_LANES*ELEM_W-1:0] rd_data_b
);

    localparam int VEC_W = VEC_LANES * ELEM_W;  // 512 bits = 64 bytes

    // Register storage (synthesizes to distributed RAM / LUT-RAM)
    (* ram_style = "distributed" *)
    logic [VEC_W-1:0] regs [0:NUM_REGS-1];

    // Synchronous write
    always_ff @(posedge clk) begin
        if (wr_en) begin
            regs[wr_addr] <= wr_data;
        end
    end

    // Asynchronous reads (combinational, distributed RAM natural behavior)
    assign rd_data_a = regs[rd_addr_a];
    assign rd_data_b = regs[rd_addr_b];

endmodule

`default_nettype wire
