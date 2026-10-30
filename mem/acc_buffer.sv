// =============================================================================
// acc_buffer.sv - 32-Bank Accumulator Buffer (Double-Buffered)
// =============================================================================
// 32 banks × 32 entries × 32-bit = 4KB per set. Total 8KB (ping-pong).
// Write from systolic array: 32 banks in parallel (32 values/cycle).
// Read for store: 32 banks in parallel (32 values/cycle → STORE 32 cycles).
//
// Bank i stores accumulator [i] for all K entries:
//   bank 0: acc[0][0..31], bank 1: acc[1][0..31], ..., bank 31: acc[31][0..31]
// Wait, for 32x32 output: acc[i][j] where i=row, j=col.
// Bank j stores column j's accumulators across 32 rows.
// =============================================================================

`default_nettype none

module acc_buffer #(
    parameter int NUM_BANKS   = 32,
    parameter int BANK_DEPTH  = 32,    // 32 rows
    parameter int DATA_W      = 32,
    parameter int ADDR_W       = 5      // log2(32)
)(
    input  wire                          clk,
    input  wire                          rst_n,

    // Bank select (active bank for accumulation write)
    input  wire                          bank_sel,

    // Read set select (absolute): 0 -> bank0_mem, 1 -> bank1_mem.
    // Ping-pong readers drive ~bank_sel (read the inactive set); topk_unit
    // drives an explicit per-segment set so it can span both N-tiles.
    input  wire                          rd_set,

    // Write port (from systolic array, to active bank)
    // 32 columns written simultaneously
    input  wire                          wr_en,
    input  wire [ADDR_W-1:0]            wr_addr,     // row index (0..31)
    input  wire [DATA_W-1:0]            wr_data0,    // bank 0 (column 0)
    input  wire [DATA_W-1:0]            wr_data1,
    input  wire [DATA_W-1:0]            wr_data2,
    input  wire [DATA_W-1:0]            wr_data3,
    input  wire [DATA_W-1:0]            wr_data4,
    input  wire [DATA_W-1:0]            wr_data5,
    input  wire [DATA_W-1:0]            wr_data6,
    input  wire [DATA_W-1:0]            wr_data7,
    input  wire [DATA_W-1:0]            wr_data8,
    input  wire [DATA_W-1:0]            wr_data9,
    input  wire [DATA_W-1:0]            wr_data10,
    input  wire [DATA_W-1:0]            wr_data11,
    input  wire [DATA_W-1:0]            wr_data12,
    input  wire [DATA_W-1:0]            wr_data13,
    input  wire [DATA_W-1:0]            wr_data14,
    input  wire [DATA_W-1:0]            wr_data15,
    input  wire [DATA_W-1:0]            wr_data16,
    input  wire [DATA_W-1:0]            wr_data17,
    input  wire [DATA_W-1:0]            wr_data18,
    input  wire [DATA_W-1:0]            wr_data19,
    input  wire [DATA_W-1:0]            wr_data20,
    input  wire [DATA_W-1:0]            wr_data21,
    input  wire [DATA_W-1:0]            wr_data22,
    input  wire [DATA_W-1:0]            wr_data23,
    input  wire [DATA_W-1:0]            wr_data24,
    input  wire [DATA_W-1:0]            wr_data25,
    input  wire [DATA_W-1:0]            wr_data26,
    input  wire [DATA_W-1:0]            wr_data27,
    input  wire [DATA_W-1:0]            wr_data28,
    input  wire [DATA_W-1:0]            wr_data29,
    input  wire [DATA_W-1:0]            wr_data30,
    input  wire [DATA_W-1:0]            wr_data31,

    // Read port (from inactive bank, for store/VST)
    input  wire                          rd_en,
    input  wire [ADDR_W-1:0]            rd_addr,     // row index (0..31)
    output logic [DATA_W-1:0]           rd_data0,    // bank 0 (column 0)
    output logic [DATA_W-1:0]           rd_data1,
    output logic [DATA_W-1:0]           rd_data2,
    output logic [DATA_W-1:0]           rd_data3,
    output logic [DATA_W-1:0]           rd_data4,
    output logic [DATA_W-1:0]           rd_data5,
    output logic [DATA_W-1:0]           rd_data6,
    output logic [DATA_W-1:0]           rd_data7,
    output logic [DATA_W-1:0]           rd_data8,
    output logic [DATA_W-1:0]           rd_data9,
    output logic [DATA_W-1:0]           rd_data10,
    output logic [DATA_W-1:0]           rd_data11,
    output logic [DATA_W-1:0]           rd_data12,
    output logic [DATA_W-1:0]           rd_data13,
    output logic [DATA_W-1:0]           rd_data14,
    output logic [DATA_W-1:0]           rd_data15,
    output logic [DATA_W-1:0]           rd_data16,
    output logic [DATA_W-1:0]           rd_data17,
    output logic [DATA_W-1:0]           rd_data18,
    output logic [DATA_W-1:0]           rd_data19,
    output logic [DATA_W-1:0]           rd_data20,
    output logic [DATA_W-1:0]           rd_data21,
    output logic [DATA_W-1:0]           rd_data22,
    output logic [DATA_W-1:0]           rd_data23,
    output logic [DATA_W-1:0]           rd_data24,
    output logic [DATA_W-1:0]           rd_data25,
    output logic [DATA_W-1:0]           rd_data26,
    output logic [DATA_W-1:0]           rd_data27,
    output logic [DATA_W-1:0]           rd_data28,
    output logic [DATA_W-1:0]           rd_data29,
    output logic [DATA_W-1:0]           rd_data30,
    output logic [DATA_W-1:0]           rd_data31
);

    // 32 banks, 2 sets (ping-pong)
    logic [DATA_W-1:0] bank0_mem [0:NUM_BANKS-1][0:BANK_DEPTH-1]; // set 0
    logic [DATA_W-1:0] bank1_mem [0:NUM_BANKS-1][0:BANK_DEPTH-1]; // set 1

    // Write to active bank set (accumulation)
    always_ff @(posedge clk) begin
        if (wr_en) begin
            if (bank_sel == 1'b0) begin
                bank0_mem[0][wr_addr]  <= wr_data0;
                bank0_mem[1][wr_addr]  <= wr_data1;
                bank0_mem[2][wr_addr]  <= wr_data2;
                bank0_mem[3][wr_addr]  <= wr_data3;
                bank0_mem[4][wr_addr]  <= wr_data4;
                bank0_mem[5][wr_addr]  <= wr_data5;
                bank0_mem[6][wr_addr]  <= wr_data6;
                bank0_mem[7][wr_addr]  <= wr_data7;
                bank0_mem[8][wr_addr]  <= wr_data8;
                bank0_mem[9][wr_addr]  <= wr_data9;
                bank0_mem[10][wr_addr] <= wr_data10;
                bank0_mem[11][wr_addr] <= wr_data11;
                bank0_mem[12][wr_addr] <= wr_data12;
                bank0_mem[13][wr_addr] <= wr_data13;
                bank0_mem[14][wr_addr] <= wr_data14;
                bank0_mem[15][wr_addr] <= wr_data15;
                bank0_mem[16][wr_addr] <= wr_data16;
                bank0_mem[17][wr_addr] <= wr_data17;
                bank0_mem[18][wr_addr] <= wr_data18;
                bank0_mem[19][wr_addr] <= wr_data19;
                bank0_mem[20][wr_addr] <= wr_data20;
                bank0_mem[21][wr_addr] <= wr_data21;
                bank0_mem[22][wr_addr] <= wr_data22;
                bank0_mem[23][wr_addr] <= wr_data23;
                bank0_mem[24][wr_addr] <= wr_data24;
                bank0_mem[25][wr_addr] <= wr_data25;
                bank0_mem[26][wr_addr] <= wr_data26;
                bank0_mem[27][wr_addr] <= wr_data27;
                bank0_mem[28][wr_addr] <= wr_data28;
                bank0_mem[29][wr_addr] <= wr_data29;
                bank0_mem[30][wr_addr] <= wr_data30;
                bank0_mem[31][wr_addr] <= wr_data31;
            end else begin
                bank1_mem[0][wr_addr]  <= wr_data0;
                bank1_mem[1][wr_addr]  <= wr_data1;
                bank1_mem[2][wr_addr]  <= wr_data2;
                bank1_mem[3][wr_addr]  <= wr_data3;
                bank1_mem[4][wr_addr]  <= wr_data4;
                bank1_mem[5][wr_addr]  <= wr_data5;
                bank1_mem[6][wr_addr]  <= wr_data6;
                bank1_mem[7][wr_addr]  <= wr_data7;
                bank1_mem[8][wr_addr]  <= wr_data8;
                bank1_mem[9][wr_addr]  <= wr_data9;
                bank1_mem[10][wr_addr] <= wr_data10;
                bank1_mem[11][wr_addr] <= wr_data11;
                bank1_mem[12][wr_addr] <= wr_data12;
                bank1_mem[13][wr_addr] <= wr_data13;
                bank1_mem[14][wr_addr] <= wr_data14;
                bank1_mem[15][wr_addr] <= wr_data15;
                bank1_mem[16][wr_addr] <= wr_data16;
                bank1_mem[17][wr_addr] <= wr_data17;
                bank1_mem[18][wr_addr] <= wr_data18;
                bank1_mem[19][wr_addr] <= wr_data19;
                bank1_mem[20][wr_addr] <= wr_data20;
                bank1_mem[21][wr_addr] <= wr_data21;
                bank1_mem[22][wr_addr] <= wr_data22;
                bank1_mem[23][wr_addr] <= wr_data23;
                bank1_mem[24][wr_addr] <= wr_data24;
                bank1_mem[25][wr_addr] <= wr_data25;
                bank1_mem[26][wr_addr] <= wr_data26;
                bank1_mem[27][wr_addr] <= wr_data27;
                bank1_mem[28][wr_addr] <= wr_data28;
                bank1_mem[29][wr_addr] <= wr_data29;
                bank1_mem[30][wr_addr] <= wr_data30;
                bank1_mem[31][wr_addr] <= wr_data31;
            end
        end
    end

    // Read from the set selected by rd_set (ping-pong readers use ~bank_sel,
    // i.e. the inactive set; topk may address either set absolutely).
    // 32 banks in parallel → 32 values per cycle → STORE in 32 cycles
    always_comb begin
        if (rd_set == 1'b1) begin
            // Read bank1_mem
            rd_data0  = bank1_mem[0][rd_addr];
            rd_data1  = bank1_mem[1][rd_addr];
            rd_data2  = bank1_mem[2][rd_addr];
            rd_data3  = bank1_mem[3][rd_addr];
            rd_data4  = bank1_mem[4][rd_addr];
            rd_data5  = bank1_mem[5][rd_addr];
            rd_data6  = bank1_mem[6][rd_addr];
            rd_data7  = bank1_mem[7][rd_addr];
            rd_data8  = bank1_mem[8][rd_addr];
            rd_data9  = bank1_mem[9][rd_addr];
            rd_data10 = bank1_mem[10][rd_addr];
            rd_data11 = bank1_mem[11][rd_addr];
            rd_data12 = bank1_mem[12][rd_addr];
            rd_data13 = bank1_mem[13][rd_addr];
            rd_data14 = bank1_mem[14][rd_addr];
            rd_data15 = bank1_mem[15][rd_addr];
            rd_data16 = bank1_mem[16][rd_addr];
            rd_data17 = bank1_mem[17][rd_addr];
            rd_data18 = bank1_mem[18][rd_addr];
            rd_data19 = bank1_mem[19][rd_addr];
            rd_data20 = bank1_mem[20][rd_addr];
            rd_data21 = bank1_mem[21][rd_addr];
            rd_data22 = bank1_mem[22][rd_addr];
            rd_data23 = bank1_mem[23][rd_addr];
            rd_data24 = bank1_mem[24][rd_addr];
            rd_data25 = bank1_mem[25][rd_addr];
            rd_data26 = bank1_mem[26][rd_addr];
            rd_data27 = bank1_mem[27][rd_addr];
            rd_data28 = bank1_mem[28][rd_addr];
            rd_data29 = bank1_mem[29][rd_addr];
            rd_data30 = bank1_mem[30][rd_addr];
            rd_data31 = bank1_mem[31][rd_addr];
        end else begin
            // Read bank0_mem
            rd_data0  = bank0_mem[0][rd_addr];
            rd_data1  = bank0_mem[1][rd_addr];
            rd_data2  = bank0_mem[2][rd_addr];
            rd_data3  = bank0_mem[3][rd_addr];
            rd_data4  = bank0_mem[4][rd_addr];
            rd_data5  = bank0_mem[5][rd_addr];
            rd_data6  = bank0_mem[6][rd_addr];
            rd_data7  = bank0_mem[7][rd_addr];
            rd_data8  = bank0_mem[8][rd_addr];
            rd_data9  = bank0_mem[9][rd_addr];
            rd_data10 = bank0_mem[10][rd_addr];
            rd_data11 = bank0_mem[11][rd_addr];
            rd_data12 = bank0_mem[12][rd_addr];
            rd_data13 = bank0_mem[13][rd_addr];
            rd_data14 = bank0_mem[14][rd_addr];
            rd_data15 = bank0_mem[15][rd_addr];
            rd_data16 = bank0_mem[16][rd_addr];
            rd_data17 = bank0_mem[17][rd_addr];
            rd_data18 = bank0_mem[18][rd_addr];
            rd_data19 = bank0_mem[19][rd_addr];
            rd_data20 = bank0_mem[20][rd_addr];
            rd_data21 = bank0_mem[21][rd_addr];
            rd_data22 = bank0_mem[22][rd_addr];
            rd_data23 = bank0_mem[23][rd_addr];
            rd_data24 = bank0_mem[24][rd_addr];
            rd_data25 = bank0_mem[25][rd_addr];
            rd_data26 = bank0_mem[26][rd_addr];
            rd_data27 = bank0_mem[27][rd_addr];
            rd_data28 = bank0_mem[28][rd_addr];
            rd_data29 = bank0_mem[29][rd_addr];
            rd_data30 = bank0_mem[30][rd_addr];
            rd_data31 = bank0_mem[31][rd_addr];
        end
    end

endmodule

`default_nettype wire
