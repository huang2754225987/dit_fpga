// =============================================================================
// L1 Cache - 32-Bank Interleaved Scratchpad
// =============================================================================
// Capacity is DEPTH × WIDTH bits. The NPU instance uses 4608 × 256-bit lines
// (144 KiB), with 512 extra lines (16 KiB) beyond the two legacy pages.
// Data interleaved by word address: bank_id = word_addr % 32.
//
// External interface: 256-bit (32 bytes = 8 banks per access).
// Port A: write (DMA receive, VST)
// Port B: read  (GLD.W, GLD.A, VLD, DMA send, scale read)
//
// Address mapping (ADDR_W-bit addr indexes 256-bit words; currently 12 bits):
//   addr[1:0]  → selects which group of 8 banks (0..3 → bank 0-7, 8-15, ... 24-31)
//   addr[ADDR_W-1:2] → row within each bank
// =============================================================================
`default_nettype none

module l1_cache
  import npu_pkg::*;
#(
    parameter int DEPTH  = 4608,
    parameter int WIDTH  = 256,
    parameter int ADDR_W = $clog2(DEPTH)
)( 
    input  wire                clk,
    input  wire                rst_n,
    // Write port
    input  wire                wr_en,
    input  wire  [ADDR_W-1:0] wr_addr,
    input  wire  [WIDTH-1:0]  wr_data,
    input  wire                clear_start,
    input  wire  [ADDR_W-1:0] clear_base,
    input  wire  [11:0]        clear_words,
    output logic               clear_busy,
    // Read port
    input  wire                rd_en,
    input  wire  [ADDR_W-1:0] rd_addr,
    output logic [WIDTH-1:0]  rd_data,
    output logic               rd_valid
);

    // 32 banks, each 32-bit wide
    localparam int BANK_W    = 32;
    localparam int ROW_BITS  = ADDR_W - 2;  // upper bits select row
    localparam int N_BANKS   = 32;
    localparam int BANK_DEPTH = (DEPTH + 3) / 4;

    logic [BANK_W-1:0] bank_mem [0:N_BANKS-1][0:BANK_DEPTH-1];

    logic [ADDR_W-1:0] clear_addr;
    logic [11:0] clear_remaining;
    wire mem_wr_en = clear_busy || wr_en;
    wire [ADDR_W-1:0] mem_wr_addr = clear_busy ? clear_addr : wr_addr;
    wire [WIDTH-1:0] mem_wr_data = clear_busy ? {WIDTH{1'b0}} : wr_data;

    // Bank group: 256-bit = 8 banks, addr[1:0] selects which group of 8
    wire [4:0] mem_wr_bank_base = {mem_wr_addr[1:0], 3'b000};
    wire [ROW_BITS-1:0] mem_wr_row = mem_wr_addr[ADDR_W-1:2];

    wire [4:0] rd_bank_base = {rd_addr[1:0], 3'b000};
    wire [ROW_BITS-1:0] rd_row = rd_addr[ADDR_W-1:2];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clear_busy <= 1'b0;
            clear_addr <= '0;
            clear_remaining <= '0;
        end else if (clear_start && !clear_busy && clear_words != 0) begin
            clear_busy <= 1'b1;
            clear_addr <= clear_base;
            clear_remaining <= clear_words;
        end else if (clear_busy) begin
            if (clear_remaining == 12'd1) begin
                clear_busy <= 1'b0;
                clear_remaining <= '0;
            end else begin
                clear_addr <= clear_addr + 1'b1;
                clear_remaining <= clear_remaining - 1'b1;
            end
        end
    end

    // Write: 256-bit → 8 consecutive banks
    always_ff @(posedge clk) begin
        if (mem_wr_en) begin
            bank_mem[mem_wr_bank_base + 5'd0][mem_wr_row] <= mem_wr_data[31:0];
            bank_mem[mem_wr_bank_base + 5'd1][mem_wr_row] <= mem_wr_data[63:32];
            bank_mem[mem_wr_bank_base + 5'd2][mem_wr_row] <= mem_wr_data[95:64];
            bank_mem[mem_wr_bank_base + 5'd3][mem_wr_row] <= mem_wr_data[127:96];
            bank_mem[mem_wr_bank_base + 5'd4][mem_wr_row] <= mem_wr_data[159:128];
            bank_mem[mem_wr_bank_base + 5'd5][mem_wr_row] <= mem_wr_data[191:160];
            bank_mem[mem_wr_bank_base + 5'd6][mem_wr_row] <= mem_wr_data[223:192];
            bank_mem[mem_wr_bank_base + 5'd7][mem_wr_row] <= mem_wr_data[255:224];
        end
    end

    // Read: 8 consecutive banks → 256-bit
    always_ff @(posedge clk) begin
        rd_valid <= rd_en;
        if (rd_en) begin
            rd_data[31:0]   <= bank_mem[rd_bank_base + 5'd0][rd_row];
            rd_data[63:32]  <= bank_mem[rd_bank_base + 5'd1][rd_row];
            rd_data[95:64]  <= bank_mem[rd_bank_base + 5'd2][rd_row];
            rd_data[127:96] <= bank_mem[rd_bank_base + 5'd3][rd_row];
            rd_data[159:128] <= bank_mem[rd_bank_base + 5'd4][rd_row];
            rd_data[191:160] <= bank_mem[rd_bank_base + 5'd5][rd_row];
            rd_data[223:192] <= bank_mem[rd_bank_base + 5'd6][rd_row];
            rd_data[255:224] <= bank_mem[rd_bank_base + 5'd7][rd_row];
        end
    end

endmodule

`default_nettype wire
