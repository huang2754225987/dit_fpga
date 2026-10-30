// Supra2 block + model-specific DMA + AXI-to-MIG bridge.
// The clock input is MIG ui_clk in this first, single-clock integration.
`default_nettype none
module supra2_block_ddr_system_top #(
    parameter integer MIG_ADDR_W=29,
    parameter integer AXI_DATA_W=128,
    parameter bit EXTERNAL_AXI=1'b0,
    parameter bit USE_STAGE_PROGRAM=1'b0,
    parameter bit USE_TILE_PROGRAM=1'b0,
    parameter bit WEIGHT_PREFETCH=1'b1,
    parameter integer TILE_PROGRAM_WORDS=259,
    parameter integer TILE_DEPENDENCY_WORDS=1052
)(
    input wire clk, input wire rst_n, input wire start,
    output wire busy, output wire done, output wire dma_error,

    // Host AXI master is used for boot-time DDR initialization and debug.
    input wire [3:0] host_awid, input wire [31:0] host_awaddr,
    input wire [7:0] host_awlen, input wire [2:0] host_awsize,
    input wire [1:0] host_awburst, input wire host_awvalid, output wire host_awready,
    input wire [AXI_DATA_W-1:0] host_wdata, input wire [AXI_DATA_W/8-1:0] host_wstrb,
    input wire host_wlast, input wire host_wvalid, output wire host_wready,
    output wire [3:0] host_bid, output wire [1:0] host_bresp,
    output wire host_bvalid, input wire host_bready,
    input wire [3:0] host_arid, input wire [31:0] host_araddr,
    input wire [7:0] host_arlen, input wire [2:0] host_arsize,
    input wire [1:0] host_arburst, input wire host_arvalid, output wire host_arready,
    output wire [3:0] host_rid, output wire [AXI_DATA_W-1:0] host_rdata,
    output wire [1:0] host_rresp, output wire host_rlast,
    output wire host_rvalid, input wire host_rready,

    // MIG native application interface
    output wire [MIG_ADDR_W-1:0] app_addr, output wire [2:0] app_cmd,
    output wire app_en, output wire app_hi_pri,
    output wire [63:0] app_wdf_data, output wire app_wdf_end,
    output wire [7:0] app_wdf_mask, output wire app_wdf_wren,
    input wire [63:0] app_rd_data, input wire app_rd_data_end,
    input wire app_rd_data_valid, input wire app_rdy, input wire app_wdf_rdy,

    // AXI memory port. Set AXI_DATA_W=512 for the ZCU106 board MIG.
    output wire [3:0] mem_awid, output wire [31:0] mem_awaddr,
    output wire [7:0] mem_awlen, output wire [2:0] mem_awsize,
    output wire [1:0] mem_awburst, output wire mem_awvalid,
    input wire mem_awready,
    output wire [AXI_DATA_W-1:0] mem_wdata, output wire [AXI_DATA_W/8-1:0] mem_wstrb,
    output wire mem_wlast, output wire mem_wvalid, input wire mem_wready,
    input wire [3:0] mem_bid, input wire [1:0] mem_bresp,
    input wire mem_bvalid, output wire mem_bready,
    output wire [3:0] mem_arid, output wire [31:0] mem_araddr,
    output wire [7:0] mem_arlen, output wire [2:0] mem_arsize,
    output wire [1:0] mem_arburst, output wire mem_arvalid,
    input wire mem_arready,
    input wire [3:0] mem_rid, input wire [AXI_DATA_W-1:0] mem_rdata,
    input wire [1:0] mem_rresp, input wire mem_rlast,
    input wire mem_rvalid, output wire mem_rready
);
    wire dma_busy, block_busy, block_done, block_start, dma_start;
    wire ld_en, wt_wr_en, sb_wr_en, sb_wr_sel, wload_req, wload_ack;
    wire [3:0] ld_target, wload_stage;
    wire [20:0] ld_addr;
    wire [15:0] ld_data, wt_wr_beat, sb_wr_addr;
    wire [255:0] wt_wr_data;
    wire [31:0] sb_wr_data;
    wire [14:0] result_rd_beat;
    wire [127:0] result_rd_data;
    wire stage_cmd_valid, stage_cmd_ready, program_error;
    wire [4:0] stage_cmd_id;
    wire tile_cmd_valid, tile_cmd_ready, tile_cmd_done, tile_program_done, tile_program_error;
    wire [127:0] tile_cmd_word;
    wire [15:0] tile_program_counter;
    wire tile_loader_busy,tile_loader_done,tile_loader_error,tile_loader_start;
    wire [15:0] tile_load_addr,tile_dep_load_addr;
    wire [127:0] tile_load_data;
    wire [15:0] tile_dep_load_data;
    wire tile_load_we,tile_dep_load_we;
    wire [3:0] tile_arid;
    wire [31:0] tile_araddr;
    wire [7:0] tile_arlen;
    wire [2:0] tile_arsize;
    wire [1:0] tile_arburst;
    wire tile_arvalid,tile_arready,tile_rlast,tile_rvalid,tile_rready;
    wire [3:0] tile_rid;
    wire [AXI_DATA_W-1:0] tile_rdata;
    wire [1:0] tile_rresp;
    wire dma_error_core;

    assign dma_error = dma_error_core | tile_program_error | tile_loader_error;

    generate if (USE_TILE_PROGRAM) begin : g_tile_loader
        logic start_pending;
        logic dma_start_reg;
        assign tile_loader_start=start;
        assign dma_start=dma_start_reg;
        always_ff @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin start_pending<=1'b0; dma_start_reg<=1'b0; end
            else begin
                dma_start_reg<=1'b0;
                if(start) start_pending<=1'b1;
                if(start_pending && tile_loader_done && !dma_busy) begin
                    dma_start_reg<=1'b1;
                    start_pending<=1'b0;
                end
            end
        end
        supra2_tile_program_ddr_loader #(.AXI_DATA_W(AXI_DATA_W)) u_tile_loader (
            .clk(clk),.rst_n(rst_n),.start(tile_loader_start),
            .busy(tile_loader_busy),.done(tile_loader_done),.error(tile_loader_error),
            .program_words(16'(TILE_PROGRAM_WORDS)),
            .dependency_words(16'(TILE_DEPENDENCY_WORDS)),
            .load_we(tile_load_we),.load_addr(tile_load_addr),.load_data(tile_load_data),
            .dependency_we(tile_dep_load_we),.dependency_addr(tile_dep_load_addr),
            .dependency_data(tile_dep_load_data),
            .m_axi_arid(tile_arid),.m_axi_araddr(tile_araddr),.m_axi_arlen(tile_arlen),
            .m_axi_arsize(tile_arsize),.m_axi_arburst(tile_arburst),
            .m_axi_arvalid(tile_arvalid),.m_axi_arready(tile_arready),
            .m_axi_rid(tile_rid),.m_axi_rdata(tile_rdata),.m_axi_rresp(tile_rresp),
            .m_axi_rlast(tile_rlast),.m_axi_rvalid(tile_rvalid),.m_axi_rready(tile_rready));
    end else begin : g_no_tile_loader
        assign tile_loader_start=1'b0; assign tile_loader_busy=1'b0;
        assign tile_loader_done=1'b1; assign tile_loader_error=1'b0;
        assign tile_load_we=1'b0; assign tile_load_addr='0; assign tile_load_data='0;
        assign tile_dep_load_we=1'b0; assign tile_dep_load_addr='0; assign tile_dep_load_data='0;
        assign tile_arid='0; assign tile_araddr='0; assign tile_arlen='0;
        assign tile_arsize='0; assign tile_arburst='0; assign tile_arvalid=1'b0;
        assign tile_rready=1'b0;
        assign dma_start=start;
    end endgenerate

    generate if (USE_STAGE_PROGRAM) begin : g_stage_program
        supra2_block_stage_sequencer u_program (
            .clk(clk),.rst_n(rst_n),.start(block_start),
            .command_ready(stage_cmd_ready),.command_valid(stage_cmd_valid),
            .command_stage(stage_cmd_id),.program_error(program_error));
    end else begin : g_no_stage_program
        assign stage_cmd_valid=1'b0;
        assign stage_cmd_id=5'd0;
        assign program_error=1'b0;
    end endgenerate

    generate if (USE_TILE_PROGRAM) begin : g_tile_program
        supra2_tile_program_sequencer #(.DDR_LOAD(1'b1),
            .PROGRAM_WORDS(TILE_PROGRAM_WORDS),
            .DEPENDENCY_WORDS(TILE_DEPENDENCY_WORDS),
            .MAX_COMMANDS(512),.MAX_DEPENDENCIES(4096)) u_tile_program (
            .clk(clk),.rst_n(rst_n),.start(block_start),
            .command_valid(tile_cmd_valid),.command_ready(tile_cmd_ready),
            .command_word(tile_cmd_word),.command_done(tile_cmd_done),
            .program_done(tile_program_done),.program_error(tile_program_error),
            .program_counter(tile_program_counter),
            .program_load_we(tile_load_we),.program_load_addr(tile_load_addr),
            .program_load_data(tile_load_data),
            .dependency_load_we(tile_dep_load_we),.dependency_load_addr(tile_dep_load_addr),
            .dependency_load_data(tile_dep_load_data));
    end else begin : g_no_tile_program
        assign tile_cmd_valid=1'b0;
        assign tile_cmd_word='0;
        assign tile_program_done=1'b0;
        assign tile_program_error=1'b0;
        assign tile_program_counter='0;
    end endgenerate

    wire [3:0] d_awid,d_bid,d_arid,d_rid;
    wire [31:0] d_awaddr,d_araddr;
    wire [7:0] d_awlen,d_arlen;
    wire [2:0] d_awsize,d_arsize;
    wire [1:0] d_awburst,d_arburst,d_bresp,d_rresp;
    wire d_awvalid,d_awready,d_wlast,d_wvalid,d_wready,d_bvalid,d_bready;
    wire [AXI_DATA_W-1:0] d_wdata,d_rdata;
    wire [AXI_DATA_W/8-1:0] d_wstrb;
    wire d_arvalid,d_arready,d_rlast,d_rvalid,d_rready;

    wire [3:0] x_awid,x_bid,x_arid,x_rid;
    wire [31:0] x_awaddr,x_araddr;
    wire [7:0] x_awlen,x_arlen;
    wire [2:0] x_awsize,x_arsize;
    wire [1:0] x_awburst,x_arburst,x_bresp,x_rresp;
    wire x_awvalid,x_awready,x_wlast,x_wvalid,x_wready,x_bvalid,x_bready;
    wire [AXI_DATA_W-1:0] x_wdata,x_rdata;
    wire [AXI_DATA_W/8-1:0] x_wstrb;
    wire x_arvalid,x_arready,x_rlast,x_rvalid,x_rready;

    assign x_awid=dma_busy?d_awid:host_awid;
    assign x_awaddr=dma_busy?d_awaddr:host_awaddr;
    assign x_awlen=dma_busy?d_awlen:host_awlen;
    assign x_awsize=dma_busy?d_awsize:host_awsize;
    assign x_awburst=dma_busy?d_awburst:host_awburst;
    assign x_awvalid=dma_busy?d_awvalid:host_awvalid;
    assign host_awready=dma_busy?1'b0:x_awready;
    assign d_awready=dma_busy?x_awready:1'b0;
    assign x_wdata=dma_busy?d_wdata:host_wdata;
    assign x_wstrb=dma_busy?d_wstrb:host_wstrb;
    assign x_wlast=dma_busy?d_wlast:host_wlast;
    assign x_wvalid=dma_busy?d_wvalid:host_wvalid;
    assign host_wready=dma_busy?1'b0:x_wready;
    assign d_wready=dma_busy?x_wready:1'b0;
    assign x_bready=dma_busy?d_bready:host_bready;
    assign d_bid=x_bid; assign d_bresp=x_bresp; assign d_bvalid=dma_busy?x_bvalid:1'b0;
    assign host_bid=dma_busy?4'b0:x_bid; assign host_bresp=dma_busy?2'b0:x_bresp;
    assign host_bvalid=dma_busy?1'b0:x_bvalid;

    assign x_arid=tile_loader_busy?tile_arid:(dma_busy?d_arid:host_arid);
    assign x_araddr=tile_loader_busy?tile_araddr:(dma_busy?d_araddr:host_araddr);
    assign x_arlen=tile_loader_busy?tile_arlen:(dma_busy?d_arlen:host_arlen);
    assign x_arsize=tile_loader_busy?tile_arsize:(dma_busy?d_arsize:host_arsize);
    assign x_arburst=tile_loader_busy?tile_arburst:(dma_busy?d_arburst:host_arburst);
    assign x_arvalid=tile_loader_busy?tile_arvalid:(dma_busy?d_arvalid:host_arvalid);
    assign host_arready=(tile_loader_busy||dma_busy)?1'b0:x_arready;
    assign d_arready=(dma_busy&&!tile_loader_busy)?x_arready:1'b0;
    assign tile_arready=tile_loader_busy?x_arready:1'b0;
    assign x_rready=tile_loader_busy?tile_rready:(dma_busy?d_rready:host_rready);
    assign d_rid=x_rid; assign d_rdata=x_rdata; assign d_rresp=x_rresp;
    assign d_rlast=x_rlast; assign d_rvalid=(dma_busy&&!tile_loader_busy)?x_rvalid:1'b0;
    assign tile_rid=x_rid; assign tile_rdata=x_rdata; assign tile_rresp=x_rresp;
    assign tile_rlast=x_rlast; assign tile_rvalid=tile_loader_busy?x_rvalid:1'b0;
    assign host_rid=(dma_busy||tile_loader_busy)?4'b0:x_rid;
    assign host_rdata=(dma_busy||tile_loader_busy)?'0:x_rdata;
    assign host_rresp=(dma_busy||tile_loader_busy)?2'b0:x_rresp;
    assign host_rlast=(dma_busy||tile_loader_busy)?1'b0:x_rlast;
    assign host_rvalid=(dma_busy||tile_loader_busy)?1'b0:x_rvalid;

    supra2_block_dma #(.AXI_DATA_W(AXI_DATA_W)) u_dma(
        .clk(clk),.rst_n(rst_n),.start(dma_start),.busy(dma_busy),.done(done),.dma_error(dma_error_core),
        .block_start(block_start),.block_done(block_done),
        .ld_en(ld_en),.ld_target(ld_target),.ld_addr(ld_addr),.ld_data(ld_data),
        .wt_wr_en(wt_wr_en),.wt_wr_beat(wt_wr_beat),.wt_wr_data(wt_wr_data),
        .sb_wr_en(sb_wr_en),.sb_wr_sel(sb_wr_sel),.sb_wr_addr(sb_wr_addr),.sb_wr_data(sb_wr_data),
        .wload_req(wload_req),.wload_stage(wload_stage),.wload_ack(wload_ack),
        .result_rd_beat(result_rd_beat),.result_rd_data(result_rd_data),
        .m_axi_awid(d_awid),.m_axi_awaddr(d_awaddr),.m_axi_awlen(d_awlen),.m_axi_awsize(d_awsize),
        .m_axi_awburst(d_awburst),.m_axi_awvalid(d_awvalid),.m_axi_awready(d_awready),
        .m_axi_wdata(d_wdata),.m_axi_wstrb(d_wstrb),.m_axi_wlast(d_wlast),.m_axi_wvalid(d_wvalid),.m_axi_wready(d_wready),
        .m_axi_bid(d_bid),.m_axi_bresp(d_bresp),.m_axi_bvalid(d_bvalid),.m_axi_bready(d_bready),
        .m_axi_arid(d_arid),.m_axi_araddr(d_araddr),.m_axi_arlen(d_arlen),.m_axi_arsize(d_arsize),
        .m_axi_arburst(d_arburst),.m_axi_arvalid(d_arvalid),.m_axi_arready(d_arready),
        .m_axi_rid(d_rid),.m_axi_rdata(d_rdata),.m_axi_rresp(d_rresp),.m_axi_rlast(d_rlast),
        .m_axi_rvalid(d_rvalid),.m_axi_rready(d_rready));

    supra2_block_top #(.PROGRAMMED_STAGES(USE_STAGE_PROGRAM),
        .PROGRAMMED_TILES(USE_TILE_PROGRAM),
        .WEIGHT_PREFETCH(WEIGHT_PREFETCH && !USE_TILE_PROGRAM)) u_block(
        .clk(clk),.rst_n(rst_n),.start(block_start),.start_stage(5'd0),.start_head(4'd0),
        .start_ni(16'd0),.stop_ni(16'd0),.busy(block_busy),.done(block_done),
        .stage_cmd_valid(stage_cmd_valid),.stage_cmd_id(stage_cmd_id),
        .stage_cmd_ready(stage_cmd_ready),
        .tile_cmd_valid(tile_cmd_valid),.tile_cmd_word(tile_cmd_word),
        .tile_cmd_ready(tile_cmd_ready),.tile_cmd_done(tile_cmd_done),
        .ld_en(ld_en),.ld_target(ld_target),.ld_addr(ld_addr),.ld_data(ld_data),
        .ld_vec_en(1'b0),.ld_vec_target(4'd0),.ld_vec_beat(16'd0),.ld_vec_data(512'd0),
        .wt_wr_en(wt_wr_en),.wt_wr_beat(wt_wr_beat),.wt_wr_data(wt_wr_data),
        .sb_wr_en(sb_wr_en),.sb_wr_sel(sb_wr_sel),.sb_wr_addr(sb_wr_addr),.sb_wr_data(sb_wr_data),
        .param_vec_wr_en(1'b0),.param_vec_wr_beat(8'd0),.param_vec_wr_data(512'd0),
        .result_rd_beat(result_rd_beat),.result_rd_data(result_rd_data),
        .result_vec_rd_beat(13'b0),.result_vec_rd_data(),
        .wload_req(wload_req),.wload_stage(wload_stage),.wload_ack(wload_ack),.stage_id());

    generate if (EXTERNAL_AXI) begin : g_external_axi
        assign mem_awid=x_awid; assign mem_awaddr=x_awaddr;
        assign mem_awlen=x_awlen; assign mem_awsize=x_awsize;
        assign mem_awburst=x_awburst; assign mem_awvalid=x_awvalid;
        assign x_awready=mem_awready;
        assign mem_wdata=x_wdata; assign mem_wstrb=x_wstrb;
        assign mem_wlast=x_wlast; assign mem_wvalid=x_wvalid;
        assign x_wready=mem_wready;
        assign x_bid=mem_bid; assign x_bresp=mem_bresp;
        assign x_bvalid=mem_bvalid; assign mem_bready=x_bready;
        assign mem_arid=x_arid; assign mem_araddr=x_araddr;
        assign mem_arlen=x_arlen; assign mem_arsize=x_arsize;
        assign mem_arburst=x_arburst; assign mem_arvalid=x_arvalid;
        assign x_arready=mem_arready;
        assign x_rid=mem_rid; assign x_rdata=mem_rdata;
        assign x_rresp=mem_rresp; assign x_rlast=mem_rlast;
        assign x_rvalid=mem_rvalid; assign mem_rready=x_rready;

        assign app_addr='0; assign app_cmd='0; assign app_en=1'b0;
        assign app_hi_pri=1'b0; assign app_wdf_data='0;
        assign app_wdf_end=1'b0; assign app_wdf_mask='0;
        assign app_wdf_wren=1'b0;
    end else begin : g_native_mig
        assign mem_awid='0; assign mem_awaddr='0; assign mem_awlen='0;
        assign mem_awsize='0; assign mem_awburst='0; assign mem_awvalid=1'b0;
        assign mem_wdata='0; assign mem_wstrb='0; assign mem_wlast=1'b0;
        assign mem_wvalid=1'b0; assign mem_bready=1'b0;
        assign mem_arid='0; assign mem_araddr='0; assign mem_arlen='0;
        assign mem_arsize='0; assign mem_arburst='0; assign mem_arvalid=1'b0;
        assign mem_rready=1'b0;
        axi_to_mig_bridge #(.AXI_DATA_W(AXI_DATA_W),.AXI_ADDR_W(32),.AXI_ID_W(4),
        .MIG_DATA_W(64),.MIG_ADDR_W(MIG_ADDR_W),.MIG_MASK_W(8),
        .RATIO(AXI_DATA_W/64),.READ_FIFO_BEATS(8)) u_bridge(
        .clk(clk),.rst_n(rst_n),
        .s_axi_awid(x_awid),.s_axi_awaddr(x_awaddr),.s_axi_awlen(x_awlen),.s_axi_awsize(x_awsize),
        .s_axi_awburst(x_awburst),.s_axi_awvalid(x_awvalid),.s_axi_awready(x_awready),
        .s_axi_wdata(x_wdata),.s_axi_wstrb(x_wstrb),.s_axi_wlast(x_wlast),.s_axi_wvalid(x_wvalid),.s_axi_wready(x_wready),
        .s_axi_bid(x_bid),.s_axi_bresp(x_bresp),.s_axi_bvalid(x_bvalid),.s_axi_bready(x_bready),
        .s_axi_arid(x_arid),.s_axi_araddr(x_araddr),.s_axi_arlen(x_arlen),.s_axi_arsize(x_arsize),
        .s_axi_arburst(x_arburst),.s_axi_arvalid(x_arvalid),.s_axi_arready(x_arready),
        .s_axi_rid(x_rid),.s_axi_rdata(x_rdata),.s_axi_rresp(x_rresp),.s_axi_rlast(x_rlast),
        .s_axi_rvalid(x_rvalid),.s_axi_rready(x_rready),
        .app_addr(app_addr),.app_cmd(app_cmd),.app_en(app_en),.app_hi_pri(app_hi_pri),
        .app_wdf_data(app_wdf_data),.app_wdf_end(app_wdf_end),.app_wdf_mask(app_wdf_mask),
        .app_wdf_wren(app_wdf_wren),.app_rd_data(app_rd_data),.app_rd_data_end(app_rd_data_end),
        .app_rd_data_valid(app_rd_data_valid),.app_rdy(app_rdy),.app_wdf_rdy(app_wdf_rdy));
    end endgenerate

    assign busy=dma_busy|block_busy|tile_loader_busy;
endmodule
`default_nettype wire
