`default_nettype none

// Single-write/synchronous-read RAM primitive used for banked GEMM operands.
// Storage is intentionally not reset; loaders write every word before use.
module supra2_sync_word_ram #(
    parameter int DATA_W=256,
    parameter int DEPTH=1024,
    parameter int ADDR_W=$clog2(DEPTH)
) (
    input  wire logic                  clk,
    input  wire logic                  wr_en,
    input  wire logic [ADDR_W-1:0]     wr_addr,
    input  wire logic [DATA_W-1:0]     wr_data,
    input  wire logic [ADDR_W-1:0]     rd_addr,
    output logic [DATA_W-1:0]          rd_data
);
    (* ram_style = "block" *) logic [DATA_W-1:0] mem [0:DEPTH-1];
    always_ff @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
        rd_data <= mem[rd_addr];
    end
    initial begin
        if (DATA_W <= 0 || DEPTH <= 0 || ADDR_W <= 0)
            $error("Invalid synchronous word RAM dimensions");
    end
endmodule

// One write port and one enabled, one-cycle read port. All callers use the
// same storage/latency in simulation and synthesis. MASKED_WRITES is used only
// for BRAM banks that accept scalar FP16 updates; full-word banks can select
// UltraRAM without describing unsupported fine-grained write enables.
module supra2_masked_sync_ram #(
    parameter int DATA_W = 512,
    parameter int DEPTH = 4608,
    parameter int ADDR_W = $clog2(DEPTH),
    parameter int LANE_W = 16,
    parameter bit MASKED_WRITES = 1'b1,
    parameter RAM_STYLE = "block"
) (
    input wire logic clk,
    input wire logic wr_en,
    input wire logic [ADDR_W-1:0] wr_addr,
    input wire logic [DATA_W-1:0] wr_data,
    input wire logic [DATA_W/LANE_W-1:0] wr_mask,
    input wire logic rd_en,
    input wire logic [ADDR_W-1:0] rd_addr,
    output logic [DATA_W-1:0] rd_data
);
    (* ram_style = RAM_STYLE *) logic [DATA_W-1:0] mem [0:DEPTH-1];
    generate
        if (MASKED_WRITES) begin : g_masked
            always_ff @(posedge clk) begin
                if (wr_en)
                    for (int lane = 0; lane < DATA_W/LANE_W; lane++)
                        if (wr_mask[lane])
                            mem[wr_addr][lane*LANE_W +: LANE_W] <=
                                wr_data[lane*LANE_W +: LANE_W];
                if (rd_en) rd_data <= mem[rd_addr];
            end
        end else begin : g_full_word
            always_ff @(posedge clk) begin
                if (wr_en) mem[wr_addr] <= wr_data;
                if (rd_en) rd_data <= mem[rd_addr];
            end
        end
    endgenerate
`ifndef SYNTHESIS
    initial begin
        if (DATA_W <= 0 || DEPTH <= 0 || LANE_W <= 0 || DATA_W%LANE_W != 0)
            $fatal(1, "Invalid masked RAM dimensions");
        if (MASKED_WRITES && RAM_STYLE == "ultra")
            $fatal(1, "Use full-word writes for this UltraRAM wrapper");
    end
    always @(posedge clk) begin
        if (wr_en && int'(wr_addr) >= DEPTH)
            $fatal(1, "RAM write address out of bounds");
        if (rd_en && int'(rd_addr) >= DEPTH)
            $fatal(1, "RAM read address out of bounds addr=%0d depth=%0d", rd_addr, DEPTH);
        if (wr_en && rd_en && wr_addr == rd_addr)
            $fatal(1, "RAM same-address read/write is excluded by the scheduler");
    end
`endif
endmodule

`default_nettype wire
