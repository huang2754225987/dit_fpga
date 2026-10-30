`default_nettype none

// Single-outstanding 512-bit AXI4 read DMA for descriptor-driven payload reads.
// It streams each accepted R beat to a backpressure-capable consumer and
// splits bursts at both the 64-beat AXI limit and every 4-KiB boundary.
module supra2_axi_read_dma #(
    parameter logic [3:0] AXI_ID = 4'h0
) (
    input  wire logic         clk,
    input  wire logic         rst_n,
    input  wire logic         start,
    input  wire logic [31:0]  descriptor_address,
    input  wire logic [31:0]  descriptor_beats,
    output logic              busy,
    output logic              done,
    output logic              dma_error,

    output wire logic [511:0] stream_data,
    output wire logic         stream_valid,
    input  wire logic         stream_ready,
    output wire logic         stream_last,

    output wire logic [3:0]   m_axi_arid,
    output logic [31:0]       m_axi_araddr,
    output logic [7:0]        m_axi_arlen,
    output wire logic [2:0]   m_axi_arsize,
    output wire logic [1:0]   m_axi_arburst,
    output logic              m_axi_arvalid,
    input  wire logic         m_axi_arready,
    input  wire logic [3:0]   m_axi_rid,
    input  wire logic [511:0] m_axi_rdata,
    input  wire logic [1:0]   m_axi_rresp,
    input  wire logic         m_axi_rlast,
    input  wire logic         m_axi_rvalid,
    output logic              m_axi_rready
);
    typedef enum logic [1:0] {S_IDLE, S_AR, S_R} state_t;
    state_t state;

    logic [31:0] current_address;
    // A 32-bit byte address can cover at most 2^26 64-byte beats. Keep one
    // extra bit so a transfer spanning the entire 4-GiB space is representable.
    logic [26:0] beats_remaining;
    logic [6:0] burst_beats_remaining;
    logic [6:0] planned_burst_beats;
    logic [63:0] descriptor_end;
    logic [6:0] page_beats;
    logic [6:0] selected_burst_beats;

    assign m_axi_arid = AXI_ID;
    assign m_axi_arsize = 3'd6; // 64 bytes per 512-bit beat
    assign m_axi_arburst = 2'b01; // INCR
    assign m_axi_arvalid = (state == S_AR);
    assign m_axi_rready = (state == S_R) && stream_ready;
    assign stream_data = m_axi_rdata;
    assign stream_valid = (state == S_R) && m_axi_rvalid;
    assign stream_last = stream_valid && (beats_remaining == 1);

    always_comb begin
        descriptor_end = {32'b0, descriptor_address} +
                         ({32'b0, descriptor_beats} << 6);
        // current_address is 64-byte aligned. The low six page bits therefore
        // directly encode how many beats remain before the next 4-KiB edge.
        page_beats = 7'd64 - {1'b0, current_address[11:6]};
        selected_burst_beats = 7'd64;
        if (beats_remaining < 27'd64)
            selected_burst_beats = beats_remaining[6:0];
        if (page_beats < selected_burst_beats)
            selected_burst_beats = page_beats;
        planned_burst_beats = '0;
        m_axi_arlen = '0;
        m_axi_araddr = current_address;
        if ((state == S_AR) && (beats_remaining != 0)) begin
            planned_burst_beats = selected_burst_beats;
            m_axi_arlen = {1'b0, selected_burst_beats} - 8'd1;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            current_address <= '0;
            beats_remaining <= '0;
            burst_beats_remaining <= '0;
            busy <= 1'b0;
            done <= 1'b0;
            dma_error <= 1'b0;
        end else begin
            done <= 1'b0;
            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        dma_error <= 1'b0;
                        if ((descriptor_address[5:0] != 0) ||
                            (descriptor_beats == 0) ||
                            (descriptor_end > 64'h1_0000_0000)) begin
                            dma_error <= 1'b1;
                            done <= 1'b1;
                        end else begin
                            busy <= 1'b1;
                            current_address <= descriptor_address;
                            beats_remaining <= 27'(descriptor_beats);
                            state <= S_AR;
                        end
                    end
                end
                S_AR: if (m_axi_arvalid && m_axi_arready) begin
                    burst_beats_remaining <= planned_burst_beats;
                    state <= S_R;
                end
                S_R: if (m_axi_rvalid && m_axi_rready) begin
                    if ((m_axi_rid != AXI_ID) || (m_axi_rresp != 2'b00) ||
                        (m_axi_rlast != (burst_beats_remaining == 1))) begin
                        dma_error <= 1'b1;
                        busy <= 1'b0;
                        done <= 1'b1;
                        state <= S_IDLE;
                    end else begin
                        current_address <= current_address + 32'd64;
                        beats_remaining <= beats_remaining - 1'b1;
                        burst_beats_remaining <= burst_beats_remaining - 1'b1;
                        if (burst_beats_remaining == 1) begin
                            if (beats_remaining == 1) begin
                                busy <= 1'b0;
                                done <= 1'b1;
                                state <= S_IDLE;
                            end else begin
                                state <= S_AR;
                            end
                        end
                    end
                end
                default: begin
                    state <= S_IDLE;
                    busy <= 1'b0;
                    dma_error <= 1'b1;
                    done <= 1'b1;
                end
            endcase
        end
    end
endmodule

`default_nettype wire
