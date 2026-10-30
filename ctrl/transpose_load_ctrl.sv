// =============================================================================
// transpose_load_ctrl.sv - L1 FP16 tile -> direct or transposed WRF tile
// =============================================================================
// Reads 32 rows of 32 FP16 values from L1. Rows may be strided (for a
// token-major [token, head_dim] buffer). The tile can be transposed for QK
// (input=feature, output=key) or kept direct for PV
// (input=key, output=feature).
// =============================================================================
`default_nettype none

module transpose_load_ctrl #(
    parameter int L1_ADDR_W = 11
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         cmd_valid,
    input  wire [10:0]  cmd_base_line,
    input  wire [5:0]   cmd_row_stride,
    input  wire         cmd_transpose,

    output logic        l1_rd_en,
    output logic [L1_ADDR_W-1:0] l1_rd_addr,
    input  wire [255:0] l1_rd_data,
    input  wire         l1_rd_valid,

    output logic        wrf_wr_en,
    output logic [6:0]  wrf_wr_addr,
    output logic [2:0]  wrf_wr_bank_base,
    output logic [31:0] wrf_wr_data0,
    output logic [31:0] wrf_wr_data1,
    output logic [31:0] wrf_wr_data2,
    output logic [31:0] wrf_wr_data3,
    output logic [31:0] wrf_wr_data4,
    output logic [31:0] wrf_wr_data5,
    output logic [31:0] wrf_wr_data6,
    output logic [31:0] wrf_wr_data7,
    output logic        busy,
    output logic        done
);
    typedef enum logic [2:0] {S_IDLE, S_READ, S_WAIT, S_WRITE, S_DONE} state_t;
    state_t state;
    logic [10:0] r_base;
    logic [5:0] r_stride;
    logic r_transpose;
    logic [4:0] r_row, r_col;
    logic r_half, r_beat;
    logic [15:0] tile [0:31][0:31];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            r_base <= '0;
            r_stride <= '0;
            r_transpose <= 1'b1;
            r_row <= '0;
            r_col <= '0;
            r_half <= 1'b0;
            r_beat <= 1'b0;
        end else begin
            case (state)
                S_IDLE: if (cmd_valid) begin
                    r_base <= cmd_base_line;
                    r_stride <= cmd_row_stride;
                    r_transpose <= cmd_transpose;
                    r_row <= '0;
                    r_beat <= 1'b0;
                    state <= S_READ;
                end
                S_READ: state <= S_WAIT;
                S_WAIT: if (l1_rd_valid) begin
                    for (int lane = 0; lane < 16; lane++)
                        tile[r_row][{r_beat, 4'b0} + lane] <= l1_rd_data[lane*16 +: 16];
                    if (r_beat) begin
                        r_beat <= 1'b0;
                        if (r_row == 5'd31) begin
                            r_col <= '0;
                            r_half <= 1'b0;
                            state <= S_WRITE;
                        end else begin
                            r_row <= r_row + 5'd1;
                            state <= S_READ;
                        end
                    end else begin
                        r_beat <= 1'b1;
                        state <= S_READ;
                    end
                end
                S_WRITE: begin
                    if (r_half) begin
                        r_half <= 1'b0;
                        if (r_col == 5'd31)
                            state <= S_DONE;
                        else
                            r_col <= r_col + 5'd1;
                    end else begin
                        r_half <= 1'b1;
                    end
                end
                S_DONE: state <= S_IDLE;
                default: state <= S_IDLE;
            endcase
        end
    end

    assign l1_rd_en = state == S_READ;
    assign l1_rd_addr = r_base + r_row * r_stride + r_beat;
    assign wrf_wr_en = state == S_WRITE;
    assign wrf_wr_addr = {2'b0, r_col};
    assign wrf_wr_bank_base = {2'b0, r_half};

    always_comb begin
        if (r_transpose) begin
            wrf_wr_data0 = {tile[{r_half,4'd1}][r_col],  tile[{r_half,4'd0}][r_col]};
            wrf_wr_data1 = {tile[{r_half,4'd3}][r_col],  tile[{r_half,4'd2}][r_col]};
            wrf_wr_data2 = {tile[{r_half,4'd5}][r_col],  tile[{r_half,4'd4}][r_col]};
            wrf_wr_data3 = {tile[{r_half,4'd7}][r_col],  tile[{r_half,4'd6}][r_col]};
            wrf_wr_data4 = {tile[{r_half,4'd9}][r_col],  tile[{r_half,4'd8}][r_col]};
            wrf_wr_data5 = {tile[{r_half,4'd11}][r_col], tile[{r_half,4'd10}][r_col]};
            wrf_wr_data6 = {tile[{r_half,4'd13}][r_col], tile[{r_half,4'd12}][r_col]};
            wrf_wr_data7 = {tile[{r_half,4'd15}][r_col], tile[{r_half,4'd14}][r_col]};
        end else begin
            wrf_wr_data0 = {tile[r_col][{r_half,4'd1}],  tile[r_col][{r_half,4'd0}]};
            wrf_wr_data1 = {tile[r_col][{r_half,4'd3}],  tile[r_col][{r_half,4'd2}]};
            wrf_wr_data2 = {tile[r_col][{r_half,4'd5}],  tile[r_col][{r_half,4'd4}]};
            wrf_wr_data3 = {tile[r_col][{r_half,4'd7}],  tile[r_col][{r_half,4'd6}]};
            wrf_wr_data4 = {tile[r_col][{r_half,4'd9}],  tile[r_col][{r_half,4'd8}]};
            wrf_wr_data5 = {tile[r_col][{r_half,4'd11}], tile[r_col][{r_half,4'd10}]};
            wrf_wr_data6 = {tile[r_col][{r_half,4'd13}], tile[r_col][{r_half,4'd12}]};
            wrf_wr_data7 = {tile[r_col][{r_half,4'd15}], tile[r_col][{r_half,4'd14}]};
        end
    end

    assign busy = state != S_IDLE;
    assign done = state == S_DONE;
endmodule

`default_nettype wire
