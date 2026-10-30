// =============================================================================
// axi_to_mig_bridge.sv - AXI4 to MIG DDR4 App Bridge
// =============================================================================
// Single-clock bridge running on MIG ui_clk.  The AXI master must use this
// same clock; AXI valid/ready is not a clock-domain crossing mechanism.
//
// MIG app interface (non-AXI, native):
//   Write: app_cmd=3'b000, app_addr, app_en, app_wdf_data, app_wdf_end, app_wdf_mask, app_wdf_wren
//   Read:  app_cmd=3'b001, app_addr, app_en → app_rd_data, app_rd_data_valid
// =============================================================================

`timescale 1ns/1ps
`default_nettype none

module axi_to_mig_bridge #(
    parameter int AXI_DATA_W = 128,
    parameter int AXI_ADDR_W = 32,
    parameter int AXI_ID_W   = 4,
    parameter int MIG_DATA_W = 64,
    parameter int MIG_ADDR_W = 29,
    parameter int MIG_MASK_W = 8,
    parameter int RATIO      = AXI_DATA_W / MIG_DATA_W,
    parameter int READ_FIFO_BEATS = 8
)(
    input  wire                          clk,
    input  wire                          rst_n,

    // ---- AXI4 Slave (from NPU Master) ----
    input  wire  [AXI_ID_W-1:0]          s_axi_awid,
    input  wire  [AXI_ADDR_W-1:0]        s_axi_awaddr,
    input  wire  [7:0]                   s_axi_awlen,
    input  wire  [2:0]                   s_axi_awsize,
    input  wire  [1:0]                   s_axi_awburst,
    input  wire                          s_axi_awvalid,
    output wire                          s_axi_awready,

    input  wire  [AXI_DATA_W-1:0]        s_axi_wdata,
    input  wire  [AXI_DATA_W/8-1:0]     s_axi_wstrb,
    input  wire                          s_axi_wlast,
    input  wire                          s_axi_wvalid,
    output wire                          s_axi_wready,

    output wire  [AXI_ID_W-1:0]          s_axi_bid,
    output wire  [1:0]                   s_axi_bresp,
    output wire                          s_axi_bvalid,
    input  wire                          s_axi_bready,

    input  wire  [AXI_ID_W-1:0]          s_axi_arid,
    input  wire  [AXI_ADDR_W-1:0]        s_axi_araddr,
    input  wire  [7:0]                   s_axi_arlen,
    input  wire  [2:0]                   s_axi_arsize,
    input  wire  [1:0]                   s_axi_arburst,
    input  wire                          s_axi_arvalid,
    output wire                          s_axi_arready,

    output wire  [AXI_ID_W-1:0]          s_axi_rid,
    output wire  [AXI_DATA_W-1:0]       s_axi_rdata,
    output wire  [1:0]                   s_axi_rresp,
    output wire                          s_axi_rlast,
    output wire                          s_axi_rvalid,
    input  wire                          s_axi_rready,

    // ---- MIG App Interface (to DDR4) ----
    output wire  [MIG_ADDR_W-1:0]        app_addr,
    output wire  [2:0]                   app_cmd,
    output wire                          app_en,
    output wire                          app_hi_pri,
    output wire  [MIG_DATA_W-1:0]        app_wdf_data,
    output wire                          app_wdf_end,
    output wire  [MIG_MASK_W-1:0]        app_wdf_mask,
    output wire                          app_wdf_wren,
    input  wire  [MIG_DATA_W-1:0]        app_rd_data,
    input  wire                          app_rd_data_end,
    input  wire                          app_rd_data_valid,
    input  wire                          app_rdy,
    input  wire                          app_wdf_rdy
);

    // ========================================================================
    // Write Path: AXI AW+W → MIG app write
    // ========================================================================
    typedef enum logic [2:0] { W_IDLE, W_WAIT_WDATA, W_WRITE, W_RESP } w_state_t;

    logic [2:0]                    w_state;
    logic [AXI_ID_W-1:0]          w_id_r;
    logic [AXI_ADDR_W-1:0]        w_addr_r;
    localparam int RATIO_IDX_W = (RATIO <= 1) ? 1 : $clog2(RATIO);

    logic [7:0]                   w_beats_left_r;
    logic                         w_last_beat_r;
    logic [RATIO_IDX_W-1:0]       w_chunk_r;
    logic                         w_cmd_done_r;
    logic                         w_data_done_r;
    logic [AXI_DATA_W-1:0]        w_data_r;
    logic [AXI_DATA_W/8-1:0]      w_strb_r;

    assign s_axi_awready = (w_state == W_IDLE);
    assign s_axi_bid      = w_id_r;
    assign s_axi_bresp    = 2'b00;
    assign s_axi_bvalid   = (w_state == W_RESP);

    wire r_cmd_active;
    wire w_cmd_active = (w_state == W_WRITE);
    // Read commands have priority on the shared MIG command channel.
    wire w_cmd_accept = w_cmd_active && !r_cmd_active &&
                        !w_cmd_done_r && app_rdy;
    wire w_data_accept = w_cmd_active && !w_data_done_r && app_wdf_rdy;
    wire w_half_done = (w_cmd_done_r || w_cmd_accept) &&
                       (w_data_done_r || w_data_accept);

    // ========================================================================
    // Read Path: AXI AR → MIG app read → AXI R
    // ========================================================================
    logic [AXI_ID_W-1:0]          r_id_r;
    logic [AXI_ADDR_W-1:0]        r_issue_addr_r;
    logic                         r_active;
    logic [RATIO_IDX_W-1:0]       r_chunk_r;
    localparam int READ_WORD_COUNT_W = 9 + RATIO_IDX_W;
    localparam int READ_FIFO_PTR_W = $clog2(READ_FIFO_BEATS);
    localparam int READ_FIFO_COUNT_W = $clog2(READ_FIFO_BEATS + 1);
    localparam int READ_FIFO_WORD_CAPACITY = READ_FIFO_BEATS * RATIO;
    logic [READ_WORD_COUNT_W-1:0] r_issue_words_left;
    logic [READ_WORD_COUNT_W-1:0] r_rsp_words_left;
    logic [READ_WORD_COUNT_W-1:0] r_word_occupancy;
    logic [READ_FIFO_PTR_W-1:0]   r_fifo_wr_ptr, r_fifo_rd_ptr;
    logic [READ_FIFO_COUNT_W-1:0] r_fifo_count;
    logic [AXI_DATA_W-1:0]        r_assembly_r, r_assembly_next;
    logic [AXI_DATA_W-1:0]        r_fifo_data [0:READ_FIFO_BEATS-1];
    logic                         r_fifo_last [0:READ_FIFO_BEATS-1];
    wire                          r_app_issue;
    wire                          r_issue_fire;
    wire                          r_return_fire;
    wire                          r_fifo_push;
    wire                          r_fifo_pop;

    assign s_axi_arready = !r_active && (r_fifo_count == '0) &&
                           (r_word_occupancy == '0);
    assign s_axi_rid     = r_id_r;
    assign s_axi_rresp   = 2'b00;
    assign s_axi_rdata   = r_fifo_data[r_fifo_rd_ptr];
    assign s_axi_rlast   = s_axi_rvalid && r_fifo_last[r_fifo_rd_ptr];
    assign s_axi_rvalid  = (r_fifo_count != '0);

    // Reserve FIFO space for accepted MIG reads still in flight as well as
    // data already returned but not yet consumed on AXI R.
    assign r_app_issue = r_active && (r_issue_words_left != '0) &&
                         (r_word_occupancy <
                          READ_WORD_COUNT_W'(READ_FIFO_WORD_CAPACITY));
    assign r_cmd_active = r_app_issue;
    assign r_issue_fire = r_app_issue && app_rdy;
    assign r_return_fire = app_rd_data_valid && r_active &&
                           (r_rsp_words_left != '0);
    assign r_fifo_push = r_return_fire &&
                         (r_chunk_r == RATIO_IDX_W'(RATIO - 1));
    assign r_fifo_pop = s_axi_rvalid && s_axi_rready;

    always_comb begin
        r_assembly_next = r_assembly_r;
        if (r_return_fire)
            r_assembly_next[r_chunk_r * MIG_DATA_W +: MIG_DATA_W] = app_rd_data;
    end

`ifndef SYNTHESIS
    initial begin
        if (READ_FIFO_BEATS < 2 ||
            (READ_FIFO_BEATS & (READ_FIFO_BEATS - 1)) != 0)
            $fatal(1, "READ_FIFO_BEATS must be a power of two >= 2");
        if (RATIO < 1 || AXI_DATA_W != RATIO * MIG_DATA_W)
            $fatal(1, "AXI/MIG read widths must have an integral RATIO");
    end
`endif

    // ========================================================================
    // Read/Write mux for MIG app command interface
    // ========================================================================
    assign app_hi_pri    = 1'b0;
    assign app_addr      = r_cmd_active ? r_issue_addr_r[MIG_ADDR_W-1:0] :
                           w_cmd_active ? w_addr_r[MIG_ADDR_W-1:0] : '0;
    assign app_cmd       = r_cmd_active ? 3'b001 : 3'b000;
    assign app_en        = r_cmd_active ? 1'b1 :
                           w_cmd_active ? !w_cmd_done_r : 1'b0;

    assign app_wdf_data  = w_data_r[w_chunk_r * MIG_DATA_W +: MIG_DATA_W];
    // MIG mask uses 1=masked while AXI WSTRB uses 1=write this byte.
    assign app_wdf_mask  = ~w_strb_r[w_chunk_r * MIG_MASK_W +: MIG_MASK_W];
    assign app_wdf_wren  = w_cmd_active && !w_data_done_r;
    // APP_DATA_WIDTH is 64 bits, so every command carries one complete beat.
    assign app_wdf_end   = app_wdf_wren;

    // Capture a complete AXI beat before expanding it into RATIO MIG beats.
    // This keeps the AXI W handshake independent of MIG command/data stalls.
    assign s_axi_wready  = (w_state == W_WAIT_WDATA);

    // ========================================================================
    // Write state machine
    // ========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_state        <= W_IDLE;
            w_id_r         <= '0;
            w_addr_r       <= '0;
            w_beats_left_r <= '0;
            w_last_beat_r  <= 1'b0;
            w_chunk_r      <= '0;
            w_cmd_done_r   <= 1'b0;
            w_data_done_r  <= 1'b0;
            w_data_r       <= '0;
            w_strb_r       <= '0;
        end else begin
            case (w_state)
                W_IDLE: begin
                    if (s_axi_awvalid) begin
                        w_id_r         <= s_axi_awid;
                        w_addr_r       <= s_axi_awaddr;
                        w_beats_left_r <= s_axi_awlen;
                        w_last_beat_r  <= (s_axi_awlen == 8'd0);
                        w_chunk_r      <= '0;
                        w_cmd_done_r   <= 1'b0;
                        w_data_done_r  <= 1'b0;
                        w_state        <= W_WAIT_WDATA;
                    end
                end

                W_WAIT_WDATA: begin
                    if (s_axi_wvalid && s_axi_wready) begin
                        w_data_r      <= s_axi_wdata;
                        w_strb_r      <= s_axi_wstrb;
                        w_chunk_r     <= '0;
                        w_cmd_done_r  <= 1'b0;
                        w_data_done_r <= 1'b0;
                        w_state <= W_WRITE;
                    end
                end

                W_WRITE: begin
                    if (w_cmd_accept)
                        w_cmd_done_r <= 1'b1;
                    if (w_data_accept)
                        w_data_done_r <= 1'b1;

                    if (w_half_done) begin
                        w_cmd_done_r  <= 1'b0;
                        w_data_done_r <= 1'b0;
                        if (w_chunk_r == RATIO_IDX_W'(RATIO - 1)) begin
                            w_chunk_r <= '0;
                            if (w_last_beat_r) begin
                                w_state <= W_RESP;
                            end else begin
                                w_beats_left_r <= w_beats_left_r - 8'd1;
                                w_last_beat_r  <= (w_beats_left_r == 8'd1);
                                w_state        <= W_WAIT_WDATA;
                            end
                        end else begin
                            w_chunk_r <= w_chunk_r + 1'b1;
                        end
                        w_addr_r <= w_addr_r + MIG_DATA_W / 8;
                    end
                end

                W_RESP: begin
                    if (s_axi_bready) begin
                        w_state <= W_IDLE;
                    end
                end

                default: w_state <= W_IDLE;
            endcase
        end
    end

    // ========================================================================
    // Read state machine
    // ========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_id_r             <= '0;
            r_issue_addr_r     <= '0;
            r_active           <= 1'b0;
            r_chunk_r          <= '0;
            r_issue_words_left <= '0;
            r_rsp_words_left   <= '0;
            r_word_occupancy   <= '0;
            r_fifo_wr_ptr      <= '0;
            r_fifo_rd_ptr      <= '0;
            r_fifo_count       <= '0;
            r_assembly_r       <= '0;
        end else begin
            if (s_axi_arvalid && s_axi_arready) begin
                r_id_r         <= s_axi_arid;
                r_issue_addr_r <= s_axi_araddr;
                r_active       <= 1'b1;
                r_chunk_r      <= '0;
                r_assembly_r   <= '0;
                r_issue_words_left <=
                    (READ_WORD_COUNT_W'({1'b0, s_axi_arlen} + 9'd1) *
                     READ_WORD_COUNT_W'(RATIO));
                r_rsp_words_left <=
                    (READ_WORD_COUNT_W'({1'b0, s_axi_arlen} + 9'd1) *
                     READ_WORD_COUNT_W'(RATIO));
            end

            if (r_issue_fire) begin
                r_issue_addr_r <= r_issue_addr_r +
                                  AXI_ADDR_W'(MIG_DATA_W / 8);
                r_issue_words_left <= r_issue_words_left - 1'b1;
            end

            if (r_return_fire) begin
                r_assembly_r     <= r_assembly_next;
                r_rsp_words_left <= r_rsp_words_left - 1'b1;
                if (r_chunk_r == RATIO_IDX_W'(RATIO - 1))
                    r_chunk_r <= '0;
                else
                    r_chunk_r <= r_chunk_r + 1'b1;
                if (r_rsp_words_left == 1)
                    r_active <= 1'b0;
            end

            if (r_fifo_push) begin
                r_fifo_data[r_fifo_wr_ptr] <= r_assembly_next;
                r_fifo_last[r_fifo_wr_ptr] <= (r_rsp_words_left == 1);
                if (r_fifo_wr_ptr == READ_FIFO_PTR_W'(READ_FIFO_BEATS - 1))
                    r_fifo_wr_ptr <= '0;
                else
                    r_fifo_wr_ptr <= r_fifo_wr_ptr + 1'b1;
            end
            if (r_fifo_pop) begin
                if (r_fifo_rd_ptr == READ_FIFO_PTR_W'(READ_FIFO_BEATS - 1))
                    r_fifo_rd_ptr <= '0;
                else
                    r_fifo_rd_ptr <= r_fifo_rd_ptr + 1'b1;
            end

            case ({r_fifo_push, r_fifo_pop})
                2'b10: r_fifo_count <= r_fifo_count + 1'b1;
                2'b01: r_fifo_count <= r_fifo_count - 1'b1;
                default: ;
            endcase

            case ({r_issue_fire, r_fifo_pop})
                2'b10: r_word_occupancy <= r_word_occupancy + 1'b1;
                2'b01: r_word_occupancy <= r_word_occupancy -
                                           READ_WORD_COUNT_W'(RATIO);
                2'b11: r_word_occupancy <= r_word_occupancy + 1'b1 -
                                           READ_WORD_COUNT_W'(RATIO);
                default: ;
            endcase
        end
    end

endmodule

`default_nettype wire
