// ZCU106 pure-PL board top: 512-bit DDR4 MIG + block DMA/compute
// + UART host loader. PS is not instantiated or used.
`default_nettype none
module supra2_zcu106_pl_top(
    input wire sys_rst,
    input wire c0_sys_clk_p,
    input wire c0_sys_clk_n,
    input wire uart_rx,
    output wire uart_tx,
    output wire [3:0] status_led,
    output wire c0_ddr4_act_n,
    output wire [16:0] c0_ddr4_adr,
    output wire [1:0] c0_ddr4_ba,
    output wire c0_ddr4_bg,
    output wire c0_ddr4_cke,
    output wire c0_ddr4_odt,
    output wire c0_ddr4_cs_n,
    output wire c0_ddr4_ck_t,
    output wire c0_ddr4_ck_c,
    output wire c0_ddr4_reset_n,
    inout wire [7:0] c0_ddr4_dm_dbi_n,
    inout wire [63:0] c0_ddr4_dq,
    inout wire [7:0] c0_ddr4_dqs_c,
    inout wire [7:0] c0_ddr4_dqs_t
);
    wire ui_clk, ui_sync_rst, init_calib_complete;
    wire mig_aresetn, user_rst_n;
    wire start_pulse, load_error;
    wire busy, done, dma_error;
    logic done_latched;

    wire [3:0] h_awid, h_bid, h_arid, h_rid;
    wire [31:0] h_awaddr, h_araddr;
    wire [7:0] h_awlen, h_arlen;
    wire [2:0] h_awsize, h_arsize;
    wire [1:0] h_awburst, h_arburst, h_bresp, h_rresp;
    wire h_awvalid, h_awready, h_wlast, h_wvalid, h_wready;
    wire h_bvalid, h_bready, h_arvalid, h_arready, h_rlast;
    wire h_rvalid, h_rready;
    wire [511:0] h_wdata, h_rdata;
    wire [63:0] h_wstrb;

    wire [3:0] m_awid, m_bid, m_arid, m_rid;
    wire [31:0] m_awaddr, m_araddr;
    wire [7:0] m_awlen, m_arlen;
    wire [2:0] m_awsize, m_arsize;
    wire [1:0] m_awburst, m_arburst, m_bresp, m_rresp;
    wire m_awvalid, m_awready, m_wlast, m_wvalid, m_wready;
    wire m_bvalid, m_bready, m_arvalid, m_arready, m_rlast;
    wire m_rvalid, m_rready;
    wire [511:0] m_wdata, m_rdata;
    wire [63:0] m_wstrb;

    wire [30:0] d_awaddr, d_araddr;
    wire [7:0] d_awlen, d_arlen;
    wire [2:0] d_awsize, d_arsize;
    wire [1:0] d_awburst, d_arburst, d_bresp, d_rresp;
    wire d_awlock, d_arlock, d_awvalid, d_awready;
    wire d_wlast, d_wvalid, d_wready, d_bvalid, d_bready;
    wire d_arvalid, d_arready, d_rlast, d_rvalid, d_rready;
    wire [3:0] d_awcache, d_awqos, d_awregion;
    wire [2:0] d_awprot;
    wire [3:0] d_arcache, d_arqos, d_arregion;
    wire [2:0] d_arprot;
    wire [511:0] d_wdata, d_rdata;
    wire [63:0] d_wstrb;

    assign mig_aresetn = ~ui_sync_rst & ~sys_rst;
    assign user_rst_n = mig_aresetn & init_calib_complete;
    assign status_led = {dma_error | load_error, done_latched, busy,
                         init_calib_complete};

    always_ff @(posedge ui_clk or negedge user_rst_n) begin
        if (!user_rst_n) done_latched<=1'b0;
        else if (done) done_latched<=1'b1;
    end

    uart_ddr_loader #(.AXI_DATA_W(512)) u_loader(
        .clk(ui_clk), .rst_n(user_rst_n), .uart_rx(uart_rx), .uart_tx(uart_tx),
        .block_done(done),
        .start_pulse(start_pulse), .load_error(load_error),
        .host_awid(h_awid), .host_awaddr(h_awaddr), .host_awlen(h_awlen),
        .host_awsize(h_awsize), .host_awburst(h_awburst), .host_awvalid(h_awvalid),
        .host_awready(h_awready), .host_wdata(h_wdata), .host_wstrb(h_wstrb),
        .host_wlast(h_wlast), .host_wvalid(h_wvalid), .host_wready(h_wready),
        .host_bid(h_bid), .host_bresp(h_bresp), .host_bvalid(h_bvalid),
        .host_bready(h_bready), .host_arid(h_arid), .host_araddr(h_araddr),
        .host_arlen(h_arlen), .host_arsize(h_arsize), .host_arburst(h_arburst),
        .host_arvalid(h_arvalid), .host_arready(h_arready), .host_rid(h_rid),
        .host_rdata(h_rdata), .host_rresp(h_rresp), .host_rlast(h_rlast),
        .host_rvalid(h_rvalid), .host_rready(h_rready));

    supra2_block_ddr_system_top #(.MIG_ADDR_W(29), .AXI_DATA_W(512), .EXTERNAL_AXI(1'b1)) u_system(
        .clk(ui_clk), .rst_n(user_rst_n), .start(start_pulse),
        .busy(busy), .done(done), .dma_error(dma_error),
        .host_awid(h_awid), .host_awaddr(h_awaddr), .host_awlen(h_awlen),
        .host_awsize(h_awsize), .host_awburst(h_awburst), .host_awvalid(h_awvalid),
        .host_awready(h_awready), .host_wdata(h_wdata), .host_wstrb(h_wstrb),
        .host_wlast(h_wlast), .host_wvalid(h_wvalid), .host_wready(h_wready),
        .host_bid(h_bid), .host_bresp(h_bresp), .host_bvalid(h_bvalid),
        .host_bready(h_bready), .host_arid(h_arid), .host_araddr(h_araddr),
        .host_arlen(h_arlen), .host_arsize(h_arsize), .host_arburst(h_arburst),
        .host_arvalid(h_arvalid), .host_arready(h_arready), .host_rid(h_rid),
        .host_rdata(h_rdata), .host_rresp(h_rresp), .host_rlast(h_rlast),
        .host_rvalid(h_rvalid), .host_rready(h_rready),
        .app_addr(), .app_cmd(), .app_en(), .app_hi_pri(), .app_wdf_data(),
        .app_wdf_end(), .app_wdf_mask(), .app_wdf_wren(), .app_rd_data(64'b0),
        .app_rd_data_end(1'b0), .app_rd_data_valid(1'b0), .app_rdy(1'b0),
        .app_wdf_rdy(1'b0),
        .mem_awid(m_awid), .mem_awaddr(m_awaddr), .mem_awlen(m_awlen),
        .mem_awsize(m_awsize), .mem_awburst(m_awburst), .mem_awvalid(m_awvalid),
        .mem_awready(m_awready), .mem_wdata(m_wdata), .mem_wstrb(m_wstrb),
        .mem_wlast(m_wlast), .mem_wvalid(m_wvalid), .mem_wready(m_wready),
        .mem_bid(m_bid), .mem_bresp(m_bresp), .mem_bvalid(m_bvalid),
        .mem_bready(m_bready), .mem_arid(m_arid), .mem_araddr(m_araddr),
        .mem_arlen(m_arlen), .mem_arsize(m_arsize), .mem_arburst(m_arburst),
        .mem_arvalid(m_arvalid), .mem_arready(m_arready), .mem_rid(m_rid),
        .mem_rdata(m_rdata), .mem_rresp(m_rresp), .mem_rlast(m_rlast),
        .mem_rvalid(m_rvalid), .mem_rready(m_rready));

    assign d_awaddr=m_awaddr[30:0];
    assign d_awlen=m_awlen; assign d_awsize=m_awsize; assign d_awburst=m_awburst;
    assign d_awlock=1'b0; assign d_awcache=4'b0011; assign d_awprot=3'b000;
    assign d_awregion=4'b0000; assign d_awqos=4'b0000;
    assign d_awvalid=m_awvalid; assign m_awready=d_awready;
    assign d_wdata=m_wdata; assign d_wstrb=m_wstrb; assign d_wlast=m_wlast;
    assign d_wvalid=m_wvalid; assign m_wready=d_wready;
    assign m_bresp=d_bresp; assign m_bvalid=d_bvalid;
    assign d_bready=m_bready;
    assign d_araddr=m_araddr[30:0];
    assign d_arlen=m_arlen; assign d_arsize=m_arsize; assign d_arburst=m_arburst;
    assign d_arlock=1'b0; assign d_arcache=4'b0011; assign d_arprot=3'b000;
    assign d_arregion=4'b0000; assign d_arqos=4'b0000;
    assign d_arvalid=m_arvalid; assign m_arready=d_arready;
    assign m_rdata=d_rdata; assign m_rresp=d_rresp;
    assign m_rlast=d_rlast; assign m_rvalid=d_rvalid; assign d_rready=m_rready;

    ddr4_pl_axi u_mig(
        .sys_rst(sys_rst), .c0_sys_clk_p(c0_sys_clk_p), .c0_sys_clk_n(c0_sys_clk_n),
        .c0_ddr4_act_n(c0_ddr4_act_n), .c0_ddr4_adr(c0_ddr4_adr),
        .c0_ddr4_ba(c0_ddr4_ba), .c0_ddr4_bg(c0_ddr4_bg),
        .c0_ddr4_cke(c0_ddr4_cke), .c0_ddr4_odt(c0_ddr4_odt),
        .c0_ddr4_cs_n(c0_ddr4_cs_n), .c0_ddr4_ck_t(c0_ddr4_ck_t),
        .c0_ddr4_ck_c(c0_ddr4_ck_c), .c0_ddr4_reset_n(c0_ddr4_reset_n),
        .c0_ddr4_dm_dbi_n(c0_ddr4_dm_dbi_n), .c0_ddr4_dq(c0_ddr4_dq),
        .c0_ddr4_dqs_c(c0_ddr4_dqs_c), .c0_ddr4_dqs_t(c0_ddr4_dqs_t),
        .c0_init_calib_complete(init_calib_complete), .c0_ddr4_ui_clk(ui_clk),
        .c0_ddr4_ui_clk_sync_rst(ui_sync_rst), .addn_ui_clkout1(), .dbg_clk(),
        .c0_ddr4_aresetn(mig_aresetn),
        .c0_ddr4_s_axi_awid(m_awid), .c0_ddr4_s_axi_awaddr(d_awaddr),
        .c0_ddr4_s_axi_awlen(d_awlen), .c0_ddr4_s_axi_awsize(d_awsize),
        .c0_ddr4_s_axi_awburst(d_awburst), .c0_ddr4_s_axi_awlock(d_awlock),
        .c0_ddr4_s_axi_awcache(d_awcache), .c0_ddr4_s_axi_awprot(d_awprot),
        .c0_ddr4_s_axi_awqos(d_awqos), .c0_ddr4_s_axi_awvalid(d_awvalid),
        .c0_ddr4_s_axi_awready(d_awready), .c0_ddr4_s_axi_wdata(d_wdata),
        .c0_ddr4_s_axi_wstrb(d_wstrb), .c0_ddr4_s_axi_wlast(d_wlast),
        .c0_ddr4_s_axi_wvalid(d_wvalid), .c0_ddr4_s_axi_wready(d_wready),
        .c0_ddr4_s_axi_bready(d_bready), .c0_ddr4_s_axi_bid(m_bid),
        .c0_ddr4_s_axi_bresp(d_bresp), .c0_ddr4_s_axi_bvalid(d_bvalid),
        .c0_ddr4_s_axi_arid(m_arid), .c0_ddr4_s_axi_araddr(d_araddr),
        .c0_ddr4_s_axi_arlen(d_arlen), .c0_ddr4_s_axi_arsize(d_arsize),
        .c0_ddr4_s_axi_arburst(d_arburst), .c0_ddr4_s_axi_arlock(d_arlock),
        .c0_ddr4_s_axi_arcache(d_arcache), .c0_ddr4_s_axi_arprot(d_arprot),
        .c0_ddr4_s_axi_arqos(d_arqos), .c0_ddr4_s_axi_arvalid(d_arvalid),
        .c0_ddr4_s_axi_arready(d_arready), .c0_ddr4_s_axi_rready(d_rready),
        .c0_ddr4_s_axi_rid(m_rid), .c0_ddr4_s_axi_rdata(d_rdata),
        .c0_ddr4_s_axi_rresp(d_rresp), .c0_ddr4_s_axi_rlast(d_rlast),
        .c0_ddr4_s_axi_rvalid(d_rvalid), .dbg_bus());
endmodule
`default_nettype wire
