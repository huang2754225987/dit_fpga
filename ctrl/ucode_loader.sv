// =============================================================================
// ucode_loader.sv - DMA ucode from DDR into UCODE SRAM (host-in-the-loop)
// =============================================================================
// Triggered by host writing REG_HOST_CMD[HOST_CMD_LOAD_UCODE] with
// REG_UCODE_LOAD_ADDR (DDR source) and REG_UCODE_LOAD_LEN (32-bit words).
//
// Issues AXI4 reads from DDR (one 32-bit ucode word per beat), writes each
// into UCODE SRAM port A at sequential addresses from 0. Shares the
// top-level AXI master read port with axi_dma_rd via a mux in top.sv.
// =============================================================================
`default_nettype none

module ucode_loader #(
    parameter int AXI_DATA_W = 128,
    parameter int AXI_ADDR_W = 32,
    parameter int AXI_ID_W   = 4,
    parameter int UCODE_W    = 32,
    parameter int UCODE_DEPTH= 16384,
    parameter int UCODE_AW  = $clog2(UCODE_DEPTH)
)(
    input  wire                    clk,
    input  wire                    rst_n,

    input  wire                    trigger,        // 1-cycle pulse
    input  wire [AXI_ADDR_W-1:0]  ddr_addr,
    input  wire [31:0]             word_len,       // 32-bit words to load

    // AXI4 master read (shared via top-level mux)
    output logic                   m_axi_arvalid,
    input  wire                    m_axi_arready,
    output logic [AXI_ID_W-1:0]   m_axi_arid,
    output logic [AXI_ADDR_W-1:0] m_axi_araddr,
    output logic [7:0]            m_axi_arlen,
    output logic [2:0]            m_axi_arsize,
    output logic [1:0]            m_axi_arburst,
    input  wire                   m_axi_rvalid,
    output logic                   m_axi_rready,
    input  wire [AXI_ID_W-1:0]    m_axi_rid,
    input  wire [AXI_DATA_W-1:0]  m_axi_rdata,
    input  wire [1:0]             m_axi_rresp,
    input  wire                   m_axi_rlast,

    // UCODE SRAM port A write
    output logic                   uc_sram_we,
    output logic                   uc_sram_en,
    output logic [UCODE_AW-1:0]   uc_sram_addr,
    output logic [UCODE_W-1:0]    uc_sram_din,

    output logic                   load_active,
    output logic                   done
);
    typedef enum logic [1:0] { S_IDLE, S_AR, S_R, S_DONE } state_t;
    state_t state;

    logic [AXI_ADDR_W-1:0] cur_addr;
    logic [31:0]           words_left;
    logic [UCODE_AW-1:0]   uc_addr;

    assign load_active = (state != S_IDLE);
    assign m_axi_arid    = '0;
    assign m_axi_arsize  = 3'd2;   // 4 bytes (32-bit) per transfer
    assign m_axi_arburst = 2'b01;  // INCR
    assign m_axi_arlen   = 8'd0;   // 1 beat per AR

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            m_axi_arvalid <= 1'b0;
            m_axi_araddr  <= '0;
            m_axi_rready  <= 1'b0;
            cur_addr      <= '0;
            words_left    <= '0;
            uc_addr       <= '0;
            uc_sram_we    <= 1'b0;
            uc_sram_en    <= 1'b0;
            uc_sram_addr  <= '0;
            uc_sram_din   <= '0;
            done          <= 1'b0;
        end else begin
            uc_sram_we <= 1'b0;   // default deassert write
            done       <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (trigger) begin
                        cur_addr   <= ddr_addr;
                        words_left <= word_len;
                        uc_addr    <= '0;
                        state      <= S_AR;
                    end
                end
                S_AR: begin
                    if (words_left == 0) begin
                        state <= S_DONE;
                    end else begin
                        m_axi_arvalid <= 1'b1;
                        m_axi_araddr  <= cur_addr;
                        if (m_axi_arvalid && m_axi_arready) begin
                            m_axi_arvalid <= 1'b0;
                            m_axi_rready  <= 1'b1;
                            state         <= S_R;
                        end
                    end
                end
                S_R: begin
                    if (m_axi_rvalid && m_axi_rready) begin
                        // write the 32-bit word (low lane of 128-bit bus)
                        uc_sram_en   <= 1'b1;
                        uc_sram_we   <= 1'b1;
                        uc_sram_addr <= uc_addr;
                        uc_sram_din  <= m_axi_rdata[31:0];
                        uc_addr      <= uc_addr + 1'b1;
                        cur_addr     <= cur_addr + 32'd4;   // next 32-bit word
                        words_left   <= words_left - 1'b1;
                        m_axi_rready <= 1'b0;
                        state        <= S_AR;
                    end
                end
                S_DONE: begin
                    state <= S_IDLE;
                    done  <= 1'b1;
                end
            endcase
        end
    end
endmodule

`default_nettype wire
