// ===========================================================================
// AXI4 DMA Read Master
//
// Accepts one command at a time, issues legal INCR bursts, and forwards
// in-order R data with backpressure. Multiple bursts from the active command
// may be outstanding under the same AXI ID; AXI therefore returns them in
// issue order. The response path remains single-beat-streamed (no data FIFO).
// ===========================================================================
module axi_dma_rd
    import axi_types_pkg::*;
#(
    parameter int AXI_DATA_W     = 128,
    parameter int AXI_ADDR_W     = 32,
    parameter int MAX_BURST_LEN  = 16,
    parameter int MAX_OUTSTANDING_BURSTS = 4
) (
    input  logic                      clk,
    input  logic                      rst_n,
    input  logic                      cmd_valid,
    output logic                      cmd_ready,
    input  logic [AXI_ADDR_W-1:0]    cmd_addr,
    input  logic [23:0]              cmd_len,
    input  logic [3:0]               cmd_tag,
    output logic [AXI_ID_W-1:0]      m_axi_arid,
    output logic [AXI_ADDR_W-1:0]    m_axi_araddr,
    output logic [7:0]               m_axi_arlen,
    output logic [2:0]               m_axi_arsize,
    output logic [1:0]               m_axi_arburst,
    output logic                     m_axi_arvalid,
    input  logic                     m_axi_arready,
    input  logic [AXI_ID_W-1:0]      m_axi_rid,
    input  logic [AXI_DATA_W-1:0]   m_axi_rdata,
    input  logic [1:0]               m_axi_rresp,
    input  logic                     m_axi_rlast,
    input  logic                     m_axi_rvalid,
    output logic                     m_axi_rready,
    output logic                     data_valid,
    input  logic                     data_ready,
    output logic [AXI_DATA_W-1:0]   data_out,
    output logic                     data_last,
    output logic [3:0]              data_tag,
    output logic                     busy,
    output logic                     done,
    output logic                     error
);

    localparam int BYTES_PER_BEAT = AXI_DATA_W / 8;
    localparam int BEAT_SHIFT = $clog2(BYTES_PER_BEAT);
    localparam int OUTSTANDING_W = (MAX_OUTSTANDING_BURSTS < 2)
                                 ? 1 : $clog2(MAX_OUTSTANDING_BURSTS + 1);
    localparam logic [OUTSTANDING_W-1:0] OUTSTANDING_LIMIT =
        OUTSTANDING_W'(MAX_OUTSTANDING_BURSTS);

    typedef enum logic [1:0] {S_IDLE, S_ACTIVE, S_DONE} state_t;
    state_t state;

    logic [AXI_ADDR_W-1:0] issue_addr;
    logic [23:0] issue_bytes_remaining;
    logic [23:0] total_beats;
    logic [23:0] beats_done;
    logic [7:0] burst_len;
    logic [OUTSTANDING_W-1:0] outstanding_count;
    logic [3:0] tag_r;
    logic error_r;
    logic ar_fire, r_fire, rlast_fire;

    logic [23:0] remaining_beats;
    logic [11:0] addr_in_4k;
    logic [12:0] bytes_to_4k;
    logic [8:0] beats_to_4k;
    logic [8:0] max_burst_beats;
    logic [8:0] capped_beats;
    logic [8:0] issue_burst_beats;
    logic [23:0] issue_burst_bytes;

    assign remaining_beats = issue_bytes_remaining[23:BEAT_SHIFT] +
        (|issue_bytes_remaining[BEAT_SHIFT-1:0] ? 24'd1 : 24'd0);
    assign addr_in_4k = issue_addr[11:0];
    assign bytes_to_4k = 13'd4096 - {1'b0, addr_in_4k};
    assign beats_to_4k = bytes_to_4k[12:BEAT_SHIFT] +
        (|bytes_to_4k[BEAT_SHIFT-1:0] ? 9'd1 : 9'd0);
    assign max_burst_beats = (remaining_beats < MAX_BURST_LEN[8:0])
                           ? remaining_beats[8:0] : MAX_BURST_LEN[8:0];
    assign capped_beats = (max_burst_beats < beats_to_4k)
                        ? max_burst_beats : beats_to_4k;
    assign issue_burst_beats = {1'b0, m_axi_arlen} + 9'd1;
    assign issue_burst_bytes = {15'd0, issue_burst_beats} << BEAT_SHIFT;

    assign cmd_ready = (state == S_IDLE);
    assign busy = (state == S_ACTIVE);
    assign done = (state == S_DONE);
    assign error = error_r;

    assign m_axi_arid = tag_r;
    assign m_axi_araddr = issue_addr;
    assign m_axi_arlen = capped_beats[7:0] - 8'd1;
    assign m_axi_arsize = axi_size_from_bytes(BYTES_PER_BEAT);
    assign m_axi_arburst = AXI_BURST_INCR;
    assign m_axi_arvalid = (state == S_ACTIVE) &&
                           (issue_bytes_remaining != 0) &&
                           (outstanding_count < OUTSTANDING_LIMIT);
    assign ar_fire = m_axi_arvalid && m_axi_arready;

    // A same-ID AXI stream is ordered. Do not accept response data before at
    // least one address has been accepted, and propagate consumer backpressure.
    assign m_axi_rready = (state == S_ACTIVE) &&
                          (outstanding_count != 0) && data_ready;
    assign r_fire = m_axi_rvalid && m_axi_rready;
    assign rlast_fire = r_fire && m_axi_rlast;
    assign data_valid = (state == S_ACTIVE) && m_axi_rvalid &&
                        (outstanding_count != 0);
    assign data_out = m_axi_rdata;
    assign data_last = data_valid &&
                       (beats_done == total_beats - 24'd1);
    assign data_tag = tag_r;

    initial begin
        if (AXI_DATA_W < 8 || (AXI_DATA_W & (AXI_DATA_W-1)) != 0 ||
            MAX_BURST_LEN < 1 || MAX_BURST_LEN > 256 ||
            MAX_OUTSTANDING_BURSTS < 1)
            $fatal(1, "axi_dma_rd invalid parameter configuration");
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            issue_addr <= '0;
            issue_bytes_remaining <= '0;
            total_beats <= '0;
            beats_done <= '0;
            outstanding_count <= '0;
            tag_r <= '0;
            error_r <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (cmd_valid) begin
                        issue_addr <= cmd_addr;
                        issue_bytes_remaining <= cmd_len;
                        total_beats <= cmd_len[23:BEAT_SHIFT] +
                            (|cmd_len[BEAT_SHIFT-1:0] ? 24'd1 : 24'd0);
                        beats_done <= '0;
                        outstanding_count <= '0;
                        tag_r <= cmd_tag;
                        error_r <= 1'b0;
                        state <= S_ACTIVE;
                    end
                end
                S_ACTIVE: begin
                    if (ar_fire) begin
                        issue_addr <= issue_addr + AXI_ADDR_W'(issue_burst_bytes);
                        if (issue_bytes_remaining <= issue_burst_bytes)
                            issue_bytes_remaining <= '0;
                        else
                            issue_bytes_remaining <= issue_bytes_remaining - issue_burst_bytes;
                    end
                    if (r_fire) begin
                        beats_done <= beats_done + 24'd1;
                        if (m_axi_rresp != AXI_RESP_OKAY || m_axi_rid != tag_r)
                            error_r <= 1'b1;
                    end
                    case ({ar_fire, rlast_fire})
                        2'b10: outstanding_count <= outstanding_count + 1'b1;
                        2'b01: outstanding_count <= outstanding_count - 1'b1;
                        default: ;
                    endcase
                    if (total_beats == 0 && outstanding_count == 0 && !ar_fire)
                        state <= S_DONE;
                    else if (r_fire && m_axi_rlast &&
                             (beats_done == total_beats - 24'd1))
                        state <= S_DONE;
                end
                S_DONE: state <= S_IDLE;
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule : axi_dma_rd
