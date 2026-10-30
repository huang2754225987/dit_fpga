// =============================================================================
// gemm_weight_rf.sv - 32-Bank Parallel Weight RF (Double-Buffered, 32KB)
// =============================================================================
// 32 independent banks, each 256 x 32-bit = 1KB.
// Each set: 32 x 256 x 4B = 32KB; both ping-pong sets total 64KB.
//
// K-aware banking: word (K, w) -> bank w, entry K
//   FP16 (16 words/K): banks 0..15, entry K (K_tile max = BANK_DEPTH = 256)
//   INT3 (3 words/K):   banks 0..2,  entry K
//   INT2 (2 words/K):   banks 0..1,  entry K
//   INT1 (1 word/K):    bank 0,      entry K
//
// Read: 16 banks in parallel (bank_sel selects group 0..15 or 16..31).
//   FP16: bank_sel=K[0], entry=K[7:1] -> 16 words in 1 cycle
//   INT:  bank_sel=0, entry=K -> nwords from banks 0..nwords-1
//
// Write: 8 banks per cycle from one 256-bit L1 word.
//   bank_base cycles 0,1 for FP16 (2 L1 words per K).
//   bank_base=0 for INT (1 L1 word per K, up to 8 banks used).
// =============================================================================
`default_nettype none

module gemm_weight_rf
  import npu_pkg::*;
#(
    parameter int NUM_BANKS    = 32,
    parameter int BANK_DEPTH   = 256,
    parameter int DATA_W       = 32,
    parameter int BANK_ADDR_W  = 8,    // log2(256)
    parameter int RD_PORTS      = 16    // 16 read ports (cover banks 0..15 or 16..31)
) (
    input  wire                          clk,
    input  wire                          rst_n,

    // Ping-pong bank select (0 or 1)
    input  wire                          bank_sel,

    // Write port (8 banks in parallel from 256-bit L1 word)
    input  wire                          wr_en,
    input  wire [BANK_ADDR_W-1:0]        wr_addr,        // entry index (K)
    input  wire [2:0]                    wr_bank_base,   // bank group 0..3 (banks 0..7, 8..15, ...)
    input  wire [DATA_W-1:0]             wr_data0,
    input  wire [DATA_W-1:0]             wr_data1,
    input  wire [DATA_W-1:0]             wr_data2,
    input  wire [DATA_W-1:0]             wr_data3,
    input  wire [DATA_W-1:0]             wr_data4,
    input  wire [DATA_W-1:0]             wr_data5,
    input  wire [DATA_W-1:0]             wr_data6,
    input  wire [DATA_W-1:0]             wr_data7,

    // Read port (16 banks in parallel, same entry, bank group select)
    input  wire                          rd_en,
    input  wire                          rd_bank_sel,    // 0: banks 0..15, 1: banks 16..31
    input  wire [BANK_ADDR_W-1:0]        rd_addr,        // entry index (K)
    output logic [DATA_W-1:0]            rd_data0,
    output logic [DATA_W-1:0]            rd_data1,
    output logic [DATA_W-1:0]            rd_data2,
    output logic [DATA_W-1:0]            rd_data3,
    output logic [DATA_W-1:0]            rd_data4,
    output logic [DATA_W-1:0]            rd_data5,
    output logic [DATA_W-1:0]            rd_data6,
    output logic [DATA_W-1:0]            rd_data7,
    output logic [DATA_W-1:0]            rd_data8,
    output logic [DATA_W-1:0]            rd_data9,
    output logic [DATA_W-1:0]            rd_data10,
    output logic [DATA_W-1:0]            rd_data11,
    output logic [DATA_W-1:0]            rd_data12,
    output logic [DATA_W-1:0]            rd_data13,
    output logic [DATA_W-1:0]            rd_data14,
    output logic [DATA_W-1:0]            rd_data15,
    output logic                         rd_valid
);

    // 32 banks, 2 sets (ping-pong)
    logic [DATA_W-1:0] banks     [0:NUM_BANKS-1][0:BANK_DEPTH-1]; // active
    logic [DATA_W-1:0] banks_p   [0:NUM_BANKS-1][0:BANK_DEPTH-1]; // inactive (write target)

    // Write to inactive set
    always_ff @(posedge clk) begin
        if (wr_en) begin
            if (!bank_sel) begin
                // Write to banks_p (inactive when bank_sel=0)
                banks_p[wr_bank_base*8 + 0][wr_addr] <= wr_data0;
                banks_p[wr_bank_base*8 + 1][wr_addr] <= wr_data1;
                banks_p[wr_bank_base*8 + 2][wr_addr] <= wr_data2;
                banks_p[wr_bank_base*8 + 3][wr_addr] <= wr_data3;
                banks_p[wr_bank_base*8 + 4][wr_addr] <= wr_data4;
                banks_p[wr_bank_base*8 + 5][wr_addr] <= wr_data5;
                banks_p[wr_bank_base*8 + 6][wr_addr] <= wr_data6;
                banks_p[wr_bank_base*8 + 7][wr_addr] <= wr_data7;
            end else begin
                // Write to banks (inactive when bank_sel=1)
                banks[wr_bank_base*8 + 0][wr_addr] <= wr_data0;
                banks[wr_bank_base*8 + 1][wr_addr] <= wr_data1;
                banks[wr_bank_base*8 + 2][wr_addr] <= wr_data2;
                banks[wr_bank_base*8 + 3][wr_addr] <= wr_data3;
                banks[wr_bank_base*8 + 4][wr_addr] <= wr_data4;
                banks[wr_bank_base*8 + 5][wr_addr] <= wr_data5;
                banks[wr_bank_base*8 + 6][wr_addr] <= wr_data6;
                banks[wr_bank_base*8 + 7][wr_addr] <= wr_data7;
            end
        end
    end

    // Combinational read from active set (0-cycle latency)
    always_comb begin
        if (!bank_sel) begin
            // Read from banks (active when bank_sel=0)
            if (!rd_bank_sel) begin
                rd_data0  = banks[0][rd_addr];
                rd_data1  = banks[1][rd_addr];
                rd_data2  = banks[2][rd_addr];
                rd_data3  = banks[3][rd_addr];
                rd_data4  = banks[4][rd_addr];
                rd_data5  = banks[5][rd_addr];
                rd_data6  = banks[6][rd_addr];
                rd_data7  = banks[7][rd_addr];
                rd_data8  = banks[8][rd_addr];
                rd_data9  = banks[9][rd_addr];
                rd_data10 = banks[10][rd_addr];
                rd_data11 = banks[11][rd_addr];
                rd_data12 = banks[12][rd_addr];
                rd_data13 = banks[13][rd_addr];
                rd_data14 = banks[14][rd_addr];
                rd_data15 = banks[15][rd_addr];
            end else begin
                rd_data0  = banks[16][rd_addr];
                rd_data1  = banks[17][rd_addr];
                rd_data2  = banks[18][rd_addr];
                rd_data3  = banks[19][rd_addr];
                rd_data4  = banks[20][rd_addr];
                rd_data5  = banks[21][rd_addr];
                rd_data6  = banks[22][rd_addr];
                rd_data7  = banks[23][rd_addr];
                rd_data8  = banks[24][rd_addr];
                rd_data9  = banks[25][rd_addr];
                rd_data10 = banks[26][rd_addr];
                rd_data11 = banks[27][rd_addr];
                rd_data12 = banks[28][rd_addr];
                rd_data13 = banks[29][rd_addr];
                rd_data14 = banks[30][rd_addr];
                rd_data15 = banks[31][rd_addr];
            end
        end else begin
            // Read from banks_p (active when bank_sel=1)
            if (!rd_bank_sel) begin
                rd_data0  = banks_p[0][rd_addr];
                rd_data1  = banks_p[1][rd_addr];
                rd_data2  = banks_p[2][rd_addr];
                rd_data3  = banks_p[3][rd_addr];
                rd_data4  = banks_p[4][rd_addr];
                rd_data5  = banks_p[5][rd_addr];
                rd_data6  = banks_p[6][rd_addr];
                rd_data7  = banks_p[7][rd_addr];
                rd_data8  = banks_p[8][rd_addr];
                rd_data9  = banks_p[9][rd_addr];
                rd_data10 = banks_p[10][rd_addr];
                rd_data11 = banks_p[11][rd_addr];
                rd_data12 = banks_p[12][rd_addr];
                rd_data13 = banks_p[13][rd_addr];
                rd_data14 = banks_p[14][rd_addr];
                rd_data15 = banks_p[15][rd_addr];
            end else begin
                rd_data0  = banks_p[16][rd_addr];
                rd_data1  = banks_p[17][rd_addr];
                rd_data2  = banks_p[18][rd_addr];
                rd_data3  = banks_p[19][rd_addr];
                rd_data4  = banks_p[20][rd_addr];
                rd_data5  = banks_p[21][rd_addr];
                rd_data6  = banks_p[22][rd_addr];
                rd_data7  = banks_p[23][rd_addr];
                rd_data8  = banks_p[24][rd_addr];
                rd_data9  = banks_p[25][rd_addr];
                rd_data10 = banks_p[26][rd_addr];
                rd_data11 = banks_p[27][rd_addr];
                rd_data12 = banks_p[28][rd_addr];
                rd_data13 = banks_p[29][rd_addr];
                rd_data14 = banks_p[30][rd_addr];
                rd_data15 = banks_p[31][rd_addr];
            end
        end
    end

    assign rd_valid = rd_en;

endmodule

`default_nettype wire
