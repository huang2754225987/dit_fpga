// =============================================================================
// gemm_act_rf.sv - 32-Bank Parallel Activation RF (Double-Buffered)
// =============================================================================
// 32 independent banks, each 256 x 16-bit = 512B. Total 16KB per set.
// Legacy K-aware banking: word (K, w) -> bank w, entry K.
// Batched layout: token row -> bank, K -> entry (one value per row per read).
//   Same structure as WRF but 16-bit data (FP16 activations).
//
// Write: 16 banks per cycle from one 256-bit L1 word.
//   bank_base cycles 0,1 for FP16 (2 L1 words per K, 16 banks per word = 32 banks).
// Read: legacy mode returns one value; batch mode returns 32 values/cycle.
//
// Ping-pong: active bank set for read, inactive for write (same as WRF).
// =============================================================================

`default_nettype none

module gemm_act_rf #(
    parameter int NUM_BANKS    = 32,
    parameter int BANK_DEPTH   = 256,
    parameter int DATA_W       = 16,
    parameter int BANK_ADDR_W  = 8,    // log2(256)
    parameter int WR_BANKS     = 16     // 16 banks per write (from 256-bit L1 word)
)(
    input  wire                          clk,
    input  wire                          rst_n,

    // Bank select: 0 or 1 (active bank set for read)
    input  wire                          bank_sel,

    // Write port (to inactive bank set, from GLD_A)
    input  wire                          wr_en,
    input  wire                          wr_valid,    // data valid strobe
    input  wire [BANK_ADDR_W-1:0]        wr_addr,     // entry index (K)
    input  wire [1:0]                    wr_bank_base, // bank group 0..1 (banks 0..15, 16..31)
    input  wire                          wr_transpose, // write all 32 banks for a transposed K row
    input  wire [DATA_W-1:0]             wr_data0,
    input  wire [DATA_W-1:0]             wr_data1,
    input  wire [DATA_W-1:0]             wr_data2,
    input  wire [DATA_W-1:0]             wr_data3,
    input  wire [DATA_W-1:0]             wr_data4,
    input  wire [DATA_W-1:0]             wr_data5,
    input  wire [DATA_W-1:0]             wr_data6,
    input  wire [DATA_W-1:0]             wr_data7,
    input  wire [DATA_W-1:0]             wr_data8,
    input  wire [DATA_W-1:0]             wr_data9,
    input  wire [DATA_W-1:0]             wr_data10,
    input  wire [DATA_W-1:0]             wr_data11,
    input  wire [DATA_W-1:0]             wr_data12,
    input  wire [DATA_W-1:0]             wr_data13,
    input  wire [DATA_W-1:0]             wr_data14,
    input  wire [DATA_W-1:0]             wr_data15,
    input  wire [DATA_W-1:0]             wr_data16,
    input  wire [DATA_W-1:0]             wr_data17,
    input  wire [DATA_W-1:0]             wr_data18,
    input  wire [DATA_W-1:0]             wr_data19,
    input  wire [DATA_W-1:0]             wr_data20,
    input  wire [DATA_W-1:0]             wr_data21,
    input  wire [DATA_W-1:0]             wr_data22,
    input  wire [DATA_W-1:0]             wr_data23,
    input  wire [DATA_W-1:0]             wr_data24,
    input  wire [DATA_W-1:0]             wr_data25,
    input  wire [DATA_W-1:0]             wr_data26,
    input  wire [DATA_W-1:0]             wr_data27,
    input  wire [DATA_W-1:0]             wr_data28,
    input  wire [DATA_W-1:0]             wr_data29,
    input  wire [DATA_W-1:0]             wr_data30,
    input  wire [DATA_W-1:0]             wr_data31,

    // Read port (from active bank set, to GEMM)
    input  wire                          rd_en,
    input  wire [BANK_ADDR_W-1:0]        rd_addr,     // entry index (K)
    input  wire                          rd_batch,    // return one value per token row
    output logic [NUM_BANKS*DATA_W-1:0] rd_data,
    output logic                         rd_data_valid
);

    // 32 banks, 2 sets (ping-pong)
    logic [DATA_W-1:0] banks   [0:NUM_BANKS-1][0:BANK_DEPTH-1]; // active
    logic [DATA_W-1:0] banks_p [0:NUM_BANKS-1][0:BANK_DEPTH-1]; // inactive (write target)

    // Write to inactive bank set
    always_ff @(posedge clk) begin
        if (wr_en && wr_valid) begin
            if (!bank_sel) begin
                // Write to banks_p (inactive when bank_sel=0)
                banks_p[wr_bank_base*16 + 0][wr_addr] <= wr_data0;
                banks_p[wr_bank_base*16 + 1][wr_addr] <= wr_data1;
                banks_p[wr_bank_base*16 + 2][wr_addr] <= wr_data2;
                banks_p[wr_bank_base*16 + 3][wr_addr] <= wr_data3;
                banks_p[wr_bank_base*16 + 4][wr_addr] <= wr_data4;
                banks_p[wr_bank_base*16 + 5][wr_addr] <= wr_data5;
                banks_p[wr_bank_base*16 + 6][wr_addr] <= wr_data6;
                banks_p[wr_bank_base*16 + 7][wr_addr] <= wr_data7;
                banks_p[wr_bank_base*16 + 8][wr_addr] <= wr_data8;
                banks_p[wr_bank_base*16 + 9][wr_addr] <= wr_data9;
                banks_p[wr_bank_base*16 + 10][wr_addr] <= wr_data10;
                banks_p[wr_bank_base*16 + 11][wr_addr] <= wr_data11;
                banks_p[wr_bank_base*16 + 12][wr_addr] <= wr_data12;
                banks_p[wr_bank_base*16 + 13][wr_addr] <= wr_data13;
                banks_p[wr_bank_base*16 + 14][wr_addr] <= wr_data14;
                banks_p[wr_bank_base*16 + 15][wr_addr] <= wr_data15;
            end else begin
                // Write to banks (inactive when bank_sel=1)
                banks[wr_bank_base*16 + 0][wr_addr] <= wr_data0;
                banks[wr_bank_base*16 + 1][wr_addr] <= wr_data1;
                banks[wr_bank_base*16 + 2][wr_addr] <= wr_data2;
                banks[wr_bank_base*16 + 3][wr_addr] <= wr_data3;
                banks[wr_bank_base*16 + 4][wr_addr] <= wr_data4;
                banks[wr_bank_base*16 + 5][wr_addr] <= wr_data5;
                banks[wr_bank_base*16 + 6][wr_addr] <= wr_data6;
                banks[wr_bank_base*16 + 7][wr_addr] <= wr_data7;
                banks[wr_bank_base*16 + 8][wr_addr] <= wr_data8;
                banks[wr_bank_base*16 + 9][wr_addr] <= wr_data9;
                banks[wr_bank_base*16 + 10][wr_addr] <= wr_data10;
                banks[wr_bank_base*16 + 11][wr_addr] <= wr_data11;
                banks[wr_bank_base*16 + 12][wr_addr] <= wr_data12;
                banks[wr_bank_base*16 + 13][wr_addr] <= wr_data13;
                banks[wr_bank_base*16 + 14][wr_addr] <= wr_data14;
                banks[wr_bank_base*16 + 15][wr_addr] <= wr_data15;
            end
            // Transpose mode has the whole token row available at once, so
            // commit the second 16-bank group in the same cycle.
            if (wr_transpose) begin
                if (!bank_sel) begin
                    banks_p[16][wr_addr] <= wr_data16;
                    banks_p[17][wr_addr] <= wr_data17;
                    banks_p[18][wr_addr] <= wr_data18;
                    banks_p[19][wr_addr] <= wr_data19;
                    banks_p[20][wr_addr] <= wr_data20;
                    banks_p[21][wr_addr] <= wr_data21;
                    banks_p[22][wr_addr] <= wr_data22;
                    banks_p[23][wr_addr] <= wr_data23;
                    banks_p[24][wr_addr] <= wr_data24;
                    banks_p[25][wr_addr] <= wr_data25;
                    banks_p[26][wr_addr] <= wr_data26;
                    banks_p[27][wr_addr] <= wr_data27;
                    banks_p[28][wr_addr] <= wr_data28;
                    banks_p[29][wr_addr] <= wr_data29;
                    banks_p[30][wr_addr] <= wr_data30;
                    banks_p[31][wr_addr] <= wr_data31;
                end else begin
                    banks[16][wr_addr] <= wr_data16;
                    banks[17][wr_addr] <= wr_data17;
                    banks[18][wr_addr] <= wr_data18;
                    banks[19][wr_addr] <= wr_data19;
                    banks[20][wr_addr] <= wr_data20;
                    banks[21][wr_addr] <= wr_data21;
                    banks[22][wr_addr] <= wr_data22;
                    banks[23][wr_addr] <= wr_data23;
                    banks[24][wr_addr] <= wr_data24;
                    banks[25][wr_addr] <= wr_data25;
                    banks[26][wr_addr] <= wr_data26;
                    banks[27][wr_addr] <= wr_data27;
                    banks[28][wr_addr] <= wr_data28;
                    banks[29][wr_addr] <= wr_data29;
                    banks[30][wr_addr] <= wr_data30;
                    banks[31][wr_addr] <= wr_data31;
                end
            end
        end
    end

    // Legacy mode stores 16 sequential K values per entry in banks 0..15.
    // Batched mode stores one token row per bank and K at rd_addr, so all
    // systolic rows can read their own activation in the same cycle.
    always_comb begin
        rd_data = '0;
        if (rd_batch) begin
            for (int bank = 0; bank < NUM_BANKS; bank++) begin
                if (!bank_sel)
                    rd_data[bank*DATA_W +: DATA_W] = banks[bank][rd_addr];
                else
                    rd_data[bank*DATA_W +: DATA_W] = banks_p[bank][rd_addr];
            end
        end else begin
            if (!bank_sel)
                rd_data[0 +: DATA_W] = banks[rd_addr[3:0]][rd_addr[BANK_ADDR_W-1:4]];
            else
                rd_data[0 +: DATA_W] = banks_p[rd_addr[3:0]][rd_addr[BANK_ADDR_W-1:4]];
        end
    end

    // Registered valid
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rd_data_valid <= 1'b0;
        else
            rd_data_valid <= rd_en;
    end

endmodule

`default_nettype wire
