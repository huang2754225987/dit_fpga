// =============================================================================
// tb_top.sv - NPU Top-Level Testbench
// =============================================================================
// Loads ucode.bin into UCODE SRAM, starts execution, monitors progress.
// AXI slave responses are auto-generated (always ready, return zeros for reads).
// =============================================================================
`timescale 1ns/1ps

module tb_top;

    parameter int CLK_PERIOD = 10;  // 100 MHz
    parameter int UCODE_DEPTH = 16384;
    parameter int AXI_DATA_W = 128;

    // Clock and reset
    logic clk, rst_n;
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // AXI-Lite signals
    logic [31:0] s_axil_awaddr;
    logic        s_axil_awvalid;
    wire         s_axil_awready;
    logic [31:0] s_axil_wdata;
    logic [3:0]  s_axil_wstrb;
    logic        s_axil_wvalid;
    wire         s_axil_wready;
    wire  [1:0]  s_axil_bresp;
    wire         s_axil_bvalid;
    logic        s_axil_bready;
    logic [31:0] s_axil_araddr;
    logic        s_axil_arvalid;
    wire         s_axil_arready;
    wire  [31:0] s_axil_rdata;
    wire  [1:0]  s_axil_rresp;
    wire         s_axil_rvalid;
    logic        s_axil_rready;

    // AXI4 Master Read Channel
    wire  [3:0]           m_axi_arid;
    wire  [31:0]          m_axi_araddr;
    wire  [7:0]           m_axi_arlen;
    wire  [2:0]           m_axi_arsize;
    wire  [1:0]           m_axi_arburst;
    wire                  m_axi_arvalid;
    logic                 m_axi_arready;
    logic [3:0]           m_axi_rid;
    logic [AXI_DATA_W-1:0] m_axi_rdata;
    logic [1:0]           m_axi_rresp;
    logic                 m_axi_rlast;
    logic                 m_axi_rvalid;
    wire                  m_axi_rready;

    // AXI4 Master Write Channel
    wire  [3:0]           m_axi_awid;
    wire  [31:0]          m_axi_awaddr;
    wire  [7:0]           m_axi_awlen;
    wire  [2:0]           m_axi_awsize;
    wire  [1:0]           m_axi_awburst;
    wire                  m_axi_awvalid;
    logic                 m_axi_awready;
    wire  [AXI_DATA_W-1:0] m_axi_wdata;
    wire  [AXI_DATA_W/8-1:0] m_axi_wstrb;
    wire                  m_axi_wlast;
    wire                  m_axi_wvalid;
    logic                 m_axi_wready;
    logic [3:0]           m_axi_bid;
    logic [1:0]           m_axi_bresp;
    logic                 m_axi_bvalid;
    wire                  m_axi_bready;

    // UCODE preload port
    logic                               tb_ucode_wr_en;
    logic [$clog2(UCODE_DEPTH)-1:0]     tb_ucode_wr_addr;
    logic [31:0]                        tb_ucode_wr_data;

    // DUT
    top #(
        .ARRAY_DIM     (32),
        .P_AXI_DATA_W  (AXI_DATA_W),
        .P_AXI_ADDR_W  (32),
        .P_AXI_ID_W    (4),
        .P_UCODE_DEPTH (UCODE_DEPTH)
    ) u_dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .s_axil_awaddr    (s_axil_awaddr),
        .s_axil_awvalid   (s_axil_awvalid),
        .s_axil_awready   (s_axil_awready),
        .s_axil_wdata     (s_axil_wdata),
        .s_axil_wstrb     (s_axil_wstrb),
        .s_axil_wvalid    (s_axil_wvalid),
        .s_axil_wready    (s_axil_wready),
        .s_axil_bresp     (s_axil_bresp),
        .s_axil_bvalid    (s_axil_bvalid),
        .s_axil_bready    (s_axil_bready),
        .s_axil_araddr    (s_axil_araddr),
        .s_axil_arvalid   (s_axil_arvalid),
        .s_axil_arready   (s_axil_arready),
        .s_axil_rdata     (s_axil_rdata),
        .s_axil_rresp     (s_axil_rresp),
        .s_axil_rvalid    (s_axil_rvalid),
        .s_axil_rready    (s_axil_rready),
        .m_axi_arid       (m_axi_arid),
        .m_axi_araddr     (m_axi_araddr),
        .m_axi_arlen      (m_axi_arlen),
        .m_axi_arsize     (m_axi_arsize),
        .m_axi_arburst    (m_axi_arburst),
        .m_axi_arvalid    (m_axi_arvalid),
        .m_axi_arready    (m_axi_arready),
        .m_axi_rid        (m_axi_rid),
        .m_axi_rdata      (m_axi_rdata),
        .m_axi_rresp      (m_axi_rresp),
        .m_axi_rlast      (m_axi_rlast),
        .m_axi_rvalid     (m_axi_rvalid),
        .m_axi_rready     (m_axi_rready),
        .m_axi_awid       (m_axi_awid),
        .m_axi_awaddr     (m_axi_awaddr),
        .m_axi_awlen      (m_axi_awlen),
        .m_axi_awsize     (m_axi_awsize),
        .m_axi_awburst    (m_axi_awburst),
        .m_axi_awvalid    (m_axi_awvalid),
        .m_axi_awready    (m_axi_awready),
        .m_axi_wdata      (m_axi_wdata),
        .m_axi_wstrb      (m_axi_wstrb),
        .m_axi_wlast      (m_axi_wlast),
        .m_axi_wvalid     (m_axi_wvalid),
        .m_axi_wready     (m_axi_wready),
        .m_axi_bid        (m_axi_bid),
        .m_axi_bresp      (m_axi_bresp),
        .m_axi_bvalid     (m_axi_bvalid),
        .m_axi_bready     (m_axi_bready),
        .tb_ucode_wr_en   (tb_ucode_wr_en),
        .tb_ucode_wr_addr (tb_ucode_wr_addr),
        .tb_ucode_wr_data (tb_ucode_wr_data)
    );

    // =========================================================================
    // AXI Read Slave Model (returns incrementing data, 4-beat bursts)
    // =========================================================================
    logic [7:0]  ar_len_r;
    logic [7:0]  ar_cnt;
    logic        ar_active;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axi_arready <= 1'b1;
            m_axi_rvalid  <= 1'b0;
            m_axi_rdata   <= '0;
            m_axi_rlast   <= 1'b0;
            m_axi_rresp   <= 2'b00;
            m_axi_rid     <= '0;
            ar_active     <= 1'b0;
            ar_len_r      <= '0;
            ar_cnt        <= '0;
        end else begin
            if (m_axi_arvalid && m_axi_arready) begin
                ar_active     <= 1'b1;
                ar_len_r      <= m_axi_arlen;
                ar_cnt        <= '0;
                m_axi_arready <= 1'b0;
                m_axi_rid     <= m_axi_arid;
            end

            if (ar_active) begin
                m_axi_rvalid <= 1'b1;
                m_axi_rdata  <= {4{32'hCAFE_0000 | {24'b0, ar_cnt}}};
                m_axi_rlast  <= (ar_cnt == ar_len_r);

                if (m_axi_rvalid && m_axi_rready) begin
                    if (ar_cnt == ar_len_r) begin
                        ar_active     <= 1'b0;
                        m_axi_rvalid  <= 1'b0;
                        m_axi_rlast   <= 1'b0;
                        m_axi_arready <= 1'b1;
                    end else begin
                        ar_cnt <= ar_cnt + 8'd1;
                    end
                end
            end
        end
    end

    // =========================================================================
    // AXI Write Slave Model (accept all writes)
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axi_awready <= 1'b1;
            m_axi_wready  <= 1'b1;
            m_axi_bvalid  <= 1'b0;
            m_axi_bresp   <= 2'b00;
            m_axi_bid     <= '0;
        end else begin
            if (m_axi_wvalid && m_axi_wready && m_axi_wlast) begin
                m_axi_bvalid <= 1'b1;
            end
            if (m_axi_bvalid && m_axi_bready) begin
                m_axi_bvalid <= 1'b0;
            end
        end
    end

    // =========================================================================
    // AXI-Lite write task
    // =========================================================================
    task axil_write(input logic [31:0] addr, input logic [31:0] data);
        @(posedge clk);
        s_axil_awaddr  <= addr;
        s_axil_awvalid <= 1'b1;
        s_axil_wdata   <= data;
        s_axil_wstrb   <= 4'hF;
        s_axil_wvalid  <= 1'b1;
        @(posedge clk);
        while (!(s_axil_awready && s_axil_wready)) @(posedge clk);
        s_axil_awvalid <= 1'b0;
        s_axil_wvalid  <= 1'b0;
        // Wait for bresp
        while (!s_axil_bvalid) @(posedge clk);
        s_axil_bready <= 1'b1;
        @(posedge clk);
        s_axil_bready <= 1'b0;
    endtask

    // =========================================================================
    // Load UCODE from binary file
    // =========================================================================
    logic [7:0] ucode_mem [0:1023];
    integer ucode_size;

    task load_ucode();
        integer fd, r;
        fd = $fopen("ucode.bin", "rb");
        if (fd == 0) begin
            $display("ERROR: Cannot open ucode.bin");
            $finish;
        end
        ucode_size = 0;
        while (!$feof(fd)) begin
            r = $fread(ucode_mem[ucode_size], fd);
            if (r > 0) ucode_size = ucode_size + r;
        end
        $fclose(fd);
        $display("[TB] Loaded %0d bytes from ucode.bin (%0d instructions)", ucode_size, ucode_size/4);

        // Write to UCODE SRAM (32-bit little-endian words)
        for (int i = 0; i < ucode_size/4; i++) begin
            @(posedge clk);
            tb_ucode_wr_en   <= 1'b1;
            tb_ucode_wr_addr <= i[$clog2(UCODE_DEPTH)-1:0];
            tb_ucode_wr_data <= {ucode_mem[i*4+3], ucode_mem[i*4+2],
                                 ucode_mem[i*4+1], ucode_mem[i*4+0]};
        end
        @(posedge clk);
        tb_ucode_wr_en <= 1'b0;
    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================
    initial begin
        $dumpfile("npu_sim.vcd");
        $dumpvars(0, tb_top);

        // Initialize
        rst_n = 0;
        s_axil_awaddr  = 0;
        s_axil_awvalid = 0;
        s_axil_wdata   = 0;
        s_axil_wstrb   = 0;
        s_axil_wvalid  = 0;
        s_axil_bready  = 0;
        s_axil_araddr  = 0;
        s_axil_arvalid = 0;
        s_axil_rready  = 1;
        tb_ucode_wr_en   = 0;
        tb_ucode_wr_addr = 0;
        tb_ucode_wr_data = 0;

        // Reset
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(5) @(posedge clk);

        // Load UCODE
        $display("[TB] Loading ucode...");
        load_ucode();
        repeat(5) @(posedge clk);

        // Configure registers via AXI-Lite
        $display("[TB] Configuring registers...");
        axil_write(32'h0C, 32'd12);       // UCODE_LEN = 12 instructions
        axil_write(32'h10, 32'h1000_0000); // DDR_BASE_ACT
        axil_write(32'h14, 32'h2000_0000); // DDR_BASE_WGT
        axil_write(32'h1C, 32'h3000_0000); // DDR_BASE_OUT
        axil_write(32'h48, 32'h4000_0000); // DDR_BASE_SCALE
        axil_write(32'h4C, 32'd32);       // CFG_M = 32
        axil_write(32'h50, 32'd32);       // CFG_N = 32
        axil_write(32'h54, 32'd32);       // CFG_K = 32
        repeat(5) @(posedge clk);

        // Start execution (write START bit to CTRL register)
        $display("[TB] Starting execution...");
        axil_write(32'h00, 32'h0000_0001); // CTRL.START = 1

        // Wait for completion or timeout
        $display("[TB] Waiting for completion...");
        fork
            begin
                integer cycle_cnt;
                cycle_cnt = 0;
                while (cycle_cnt < 5000) begin
                    @(posedge clk);
                    cycle_cnt = cycle_cnt + 1;

                    // Monitor key signals
                    if (u_dut.program_end) begin
                        $display("[TB] Program END reached at cycle %0d", cycle_cnt);
                    end
                    if (u_dut.dma_rd_done) begin
                        $display("[TB] DMA_RD done at cycle %0d", cycle_cnt);
                    end
                    if (u_dut.vv_vld_done) begin
                        $display("[TB] VLD done at cycle %0d", cycle_cnt);
                    end
                end
                $display("[TB] Timeout after %0d cycles", cycle_cnt);
            end
            begin
                @(posedge u_dut.npu_done_latch);
                $display("[TB] NPU DONE asserted!");
            end
        join_any
        disable fork;

        repeat(20) @(posedge clk);
        $display("[TB] Simulation complete.");
        $finish;
    end

endmodule
