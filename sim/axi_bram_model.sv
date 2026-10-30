// =============================================================================
// axi_bram_model.sv - Simple AXI4 slave + dual-port BRAM (no IP needed)
// =============================================================================
`timescale 1ns/1ps
module axi_bram_model #(
    parameter int ADDR_W = 13,
    parameter int DATA_W = 256,
    parameter int DEPTH  = 8192
)(
    input  wire                    clk,
    input  wire                    rst_n,
    // AXI4 Read channel
    input  wire [31:0]            s_axi_araddr,
    input  wire [7:0]             s_axi_arlen,
    input  wire [2:0]             s_axi_arsize,
    input  wire [1:0]             s_axi_arburst,
    input  wire                   s_axi_arvalid,
    output reg                    s_axi_arready,
    output reg  [DATA_W-1:0]      s_axi_rdata,
    output wire [1:0]             s_axi_rresp,
    output wire                   s_axi_rlast,    // combinational
    output reg                    s_axi_rvalid,
    input  wire                   s_axi_rready,
    // AXI4 Write channel
    input  wire [31:0]            s_axi_awaddr,
    input  wire [7:0]             s_axi_awlen,
    input  wire [2:0]             s_axi_awsize,
    input  wire [1:0]             s_axi_awburst,
    input  wire                   s_axi_awvalid,
    output reg                    s_axi_awready,
    input  wire [DATA_W-1:0]      s_axi_wdata,
    input  wire [DATA_W/8-1:0]    s_axi_wstrb,
    input  wire                   s_axi_wlast,
    input  wire                   s_axi_wvalid,
    output reg                    s_axi_wready,
    output reg  [1:0]             s_axi_bresp,
    output reg                    s_axi_bvalid,
    input  wire                   s_axi_bready,
    // BRAM port B backdoor
    input  wire                   b_clk,
    input  wire                   b_en,
    input  wire [31:0]            b_addr,
    input  wire [DATA_W-1:0]      b_din,
    input  wire [DATA_W/8-1:0]    b_we,
    input  wire                   b_rst
);
    reg [DATA_W-1:0] mem [0:DEPTH-1];

    // =========================================================================
    // Read channel
    // =========================================================================
    typedef enum {R_IDLE, R_RESP} r_state_t;
    r_state_t r_state;
    reg [ADDR_W-1:0] r_addr;
    reg [7:0] r_len, r_cnt;

    // rlast is combinational - based on current r_cnt
    assign s_axi_rlast = (r_state == R_RESP) && (r_cnt == r_len);
    assign s_axi_rresp = 2'b00;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_state <= R_IDLE;
            s_axi_arready <= 1'b1;
            s_axi_rvalid <= 1'b0;
            s_axi_rdata <= '0;
            r_addr <= '0; r_len <= '0; r_cnt <= '0;
        end else begin
            case (r_state)
                R_IDLE: begin
                    s_axi_arready <= 1'b1;
                    s_axi_rvalid <= 1'b0;
                    if (s_axi_arvalid && s_axi_arready) begin
                        // BRAM RD log suppressed
                        r_addr <= s_axi_araddr[ADDR_W+4:5];
                        r_len <= s_axi_arlen;
                        r_cnt <= 8'd0;
                        s_axi_arready <= 1'b0;
                        r_state <= R_RESP;
                    end
                end
                R_RESP: begin
                    // Drive data
                    s_axi_rvalid <= 1'b1;
                    s_axi_rdata <= mem[r_addr];
                    // rlast is combinational (see assign above)
                    if (s_axi_rvalid && s_axi_rready) begin
                        // Current beat accepted
                        if (s_axi_rlast) begin
                            // Last beat - go idle
                            r_state <= R_IDLE;
                            s_axi_rvalid <= 1'b0;
                            s_axi_arready <= 1'b1;
                        end else begin
                            r_addr <= r_addr + 1'b1;
                            r_cnt <= r_cnt + 8'd1;
                        end
                    end
                end
            endcase
        end
    end

    // =========================================================================
    // Write channel
    // =========================================================================
    typedef enum {W_ADDR, W_DATA, W_RESP} w_state_t;
    w_state_t w_state;
    reg [ADDR_W-1:0] w_addr;
    reg [7:0] w_len, w_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_state <= W_ADDR;
            s_axi_awready <= 1'b1;
            s_axi_wready <= 1'b0;
            s_axi_bvalid <= 1'b0;
            s_axi_bresp <= 2'b00;
            w_addr <= '0; w_len <= '0; w_cnt <= '0;
        end else begin
            case (w_state)
                W_ADDR: begin
                    s_axi_awready <= 1'b1;
                    if (s_axi_awvalid && s_axi_awready) begin
                        // BRAM WR log suppressed
                        w_addr <= s_axi_awaddr[ADDR_W+4:5];
                        w_len <= s_axi_awlen;
                        w_cnt <= 8'd0;
                        s_axi_awready <= 1'b0;
                        s_axi_wready <= 1'b1;
                        w_state <= W_DATA;
                    end
                end
                W_DATA: begin
                    if (s_axi_wvalid && s_axi_wready) begin
                        for (int i = 0; i < DATA_W/8; i++) begin
                            if (s_axi_wstrb[i])
                                mem[w_addr][i*8 +: 8] <= s_axi_wdata[i*8 +: 8];
                        end
                        if (w_cnt == w_len) begin
                            s_axi_wready <= 1'b0;
                            s_axi_bvalid <= 1'b1;
                            w_state <= W_RESP;
                        end else begin
                            w_addr <= w_addr + 1'b1;
                            w_cnt <= w_cnt + 8'd1;
                        end
                    end
                end
                W_RESP: begin
                    if (s_axi_bready) begin
                        s_axi_bvalid <= 1'b0;
                        s_axi_awready <= 1'b1;
                        w_state <= W_ADDR;
                    end
                end
            endcase
        end
    end

    // BRAM Port B backdoor
    always_ff @(posedge b_clk) begin
        if (b_en) begin
            for (int i = 0; i < DATA_W/8; i++) begin
                if (b_we[i])
                    mem[b_addr[ADDR_W+4:5]][i*8 +: 8] <= b_din[i*8 +: 8];
            end
        end
    end

endmodule
