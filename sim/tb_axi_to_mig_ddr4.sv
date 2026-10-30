// AXI bridge connected to generated ZU7EV MIG and the official DDR4 pin model.
`timescale 1ps/1ps
module tb_axi_to_mig_ddr4;
    localparam int AXI_BEATS = 16;
    localparam int AXI_DATA_W = 512;
    localparam int MIG_DATA_W = 64;
    localparam int MIG_ADDR_W = 29;
    localparam real SYSCLK_PERIOD_PS = 13501.0;

    import arch_package::*;
    import supra2_dit_ddr_layout::*;
    localparam UTYPE_density CONFIGURED_DENSITY = _4G;

    logic sys_rst = 1'b0;
    logic en_model = 1'b0;
    wire model_enable = en_model;
    wire c0_init_calib_complete;
    wire c0_ddr4_ui_clk, c0_ddr4_ui_clk_sync_rst, dbg_clk;
    wire c0_ddr4_act_n, c0_ddr4_reset_n;
    wire [16:0] c0_ddr4_adr;
    wire [1:0] c0_ddr4_ba, c0_ddr4_bg;
    wire [0:0] c0_ddr4_cke, c0_ddr4_odt, c0_ddr4_cs_n;
    wire [0:0] c0_ddr4_ck_t_int, c0_ddr4_ck_c_int;
    wire [0:0] c0_ddr4_dm_dbi_n, c0_ddr4_dqs_c, c0_ddr4_dqs_t;
    wire [7:0] c0_ddr4_dq;
    wire c0_ddr4_ck_t, c0_ddr4_ck_c;
    logic sys_clk = 1'b0;
    wire c0_sys_clk_p = sys_clk;
    wire c0_sys_clk_n = ~sys_clk;
    always #(SYSCLK_PERIOD_PS/2.0) sys_clk = ~sys_clk;

    logic [3:0] axi_awid = 4'h3;
    logic [31:0] axi_awaddr = DDR_BASE + BLOCK_STAGE_WEIGHT_OFFSET[0][0];
    logic [7:0] axi_awlen = AXI_BEATS-1;
    logic [2:0] axi_awsize = 3'd6;
    logic [1:0] axi_awburst = 2'b01;
    logic axi_awvalid = 0, axi_wvalid = 0, axi_wlast = 0;
    logic [AXI_DATA_W-1:0] axi_wdata = 0;
    logic [AXI_DATA_W/8-1:0] axi_wstrb = '1;
    wire axi_awready, axi_wready;
    wire [3:0] axi_bid;
    wire [1:0] axi_bresp;
    wire axi_bvalid;
    logic axi_bready = 1;

    logic runtime_start = 0;
    wire runtime_busy, runtime_done, runtime_error;
    wire context_start, step_prep_start, block_prepare_start, block_start;
    wire [5:0] step_index;
    wire branch_index;
    wire [3:0] block_index;
    logic context_done = 0, step_prep_done = 0;
    logic block_prepare_done = 0, block_done = 0;
    wire final_start, cfg_combine_start, euler_start;
    logic final_done = 0, cfg_combine_done = 0, euler_done = 0;
    logic dicache_enable = 1'b0;
    logic [31:0] dicache_threshold_q16 = 32'd6554;
    logic [31:0] dicache_probe_error_q16 = '0;
    logic dicache_cache_refresh_done = 1'b0;
    logic dicache_cache_refresh_branch_index = 1'b0;
    wire dicache_probe_start, dicache_reuse_residual, dicache_refresh_residual;
    logic stage_request = 0;
    wire stage_request_ready;
    wire [2:0] stage_request_index;
    wire [1:0] stage_component;
    wire tile_select;
    wire [15:0] n_tile_index, k_tile_index;
    wire [AXI_DATA_W-1:0] dma_stream_data;
    wire dma_stream_valid, dma_stream_last, dma_done;
    logic vector_load_req = 1'b0;
    wire vector_load_ready, vector_load_valid, vector_load_last, vector_load_done;
    wire [3:0] vector_load_target_out;
    wire [15:0] vector_load_beat;
    wire [AXI_DATA_W-1:0] vector_load_data;
    logic fetch_launched = 0;
    wire [3:0] axi_arid;
    wire [31:0] axi_araddr;
    wire [7:0] axi_arlen;
    wire [2:0] axi_arsize;
    wire [1:0] axi_arburst;
    wire axi_arvalid, axi_arready;
    wire [3:0] axi_rid;
    wire [1:0] axi_rresp;
    wire axi_rvalid, axi_rlast, axi_rready;
    wire [AXI_DATA_W-1:0] axi_rdata;

    wire [MIG_ADDR_W-1:0] app_addr;
    wire [2:0] app_cmd;
    wire app_en, app_hi_pri;
    wire [MIG_DATA_W-1:0] app_wdf_data, app_rd_data;
    wire app_wdf_end, app_wdf_wren, app_rd_data_end, app_rd_data_valid;
    wire [MIG_DATA_W/8-1:0] app_wdf_mask;
    wire app_rdy, app_wdf_rdy;

    logic [63:0] cycle_count = 0;
    real ui_period_ps = 0.0;
    integer beat;
    integer read_burst_count = 0;
    integer weight_burst_count = 0;
    integer vector_burst_count = 0;
    function automatic logic [AXI_DATA_W-1:0] pattern(input int index);
        logic [AXI_DATA_W-1:0] value;
        int word_index;
        begin
            value = '0;
            for (int lane = 0; lane < AXI_DATA_W/64; lane++) begin
                word_index = index*(AXI_DATA_W/64) + lane;
                value[lane*64 +: 64] =
                    (64'h5a30_c000_0000_0000 + 64'(word_index)) ^
                    (64'ha501_0000_0000_0000 ^ 64'(word_index*32'h1021));
            end
            return value;
        end
    endfunction

    always @(posedge c0_ddr4_ui_clk)
        if (!c0_ddr4_ui_clk_sync_rst) cycle_count <= cycle_count + 1;

    always @(posedge c0_ddr4_ui_clk) begin
        if (!c0_ddr4_ui_clk_sync_rst && axi_arvalid && axi_arready) begin
            read_burst_count <= read_burst_count + 1;
            if (u_runtime_dma.reader_vector_owner) begin
                vector_burst_count <= vector_burst_count + 1;
                if (axi_araddr != axi_awaddr || axi_arlen != AXI_BEATS-1 ||
                    axi_arsize != 3'd6 || axi_arburst != 2'b01)
                    $fatal(1, "vector DMA issued unexpected burst addr=%h expected=%h len=%0d size=%0d burst=%0d",
                           axi_araddr,axi_awaddr,axi_arlen,axi_arsize,axi_arburst);
            end else if (axi_araddr == axi_awaddr) begin
                weight_burst_count <= weight_burst_count + 1;
                if (axi_arlen != AXI_BEATS-1 || axi_arsize != 3'd6 ||
                    axi_arburst != 2'b01)
                    $fatal(1, "weight DMA issued unexpected burst len=%0d size=%0d burst=%0d",
                           axi_arlen,axi_arsize,axi_arburst);
            end
        end
    end

    supra2_dit_runtime_dma u_runtime_dma (
        .clk(c0_ddr4_ui_clk), .rst_n(~c0_ddr4_ui_clk_sync_rst),
        .start(runtime_start), .busy(runtime_busy), .done(runtime_done),
        .context_start(context_start), .context_done(context_done),
        .step_prep_start(step_prep_start), .step_prep_done(step_prep_done),
        .step_timestep_fp32(), .step_timestep_valid(), .step_timestep_ready(1'b1),
        .adaln_table_enable(1'b0), .adaln_data(), .adaln_valid(),
        .adaln_ready(1'b1), .adaln_last(), .adaln_done(),
        .block_prepare_start(block_prepare_start),
        .block_prepare_done(block_prepare_done), .block_start(block_start),
        .step_index(step_index), .branch_index(branch_index),
        .block_index(block_index), .block_done(block_done),
        .final_start(final_start), .final_done(final_done),
        .final_param_dma_enable(1'b0), .final_param_valid(),
        .final_param_kind(), .final_param_beat(), .final_param_data(),
        .final_param_done(),
        .vector_load_req(vector_load_req), .vector_load_ready(vector_load_ready),
        .vector_load_address(axi_awaddr), .vector_load_beats(32'(AXI_BEATS)),
        .vector_load_target(4'd0), .vector_load_base_beat(16'd20),
        .vector_load_valid(vector_load_valid),
        .vector_load_target_out(vector_load_target_out),
        .vector_load_beat(vector_load_beat), .vector_load_data(vector_load_data),
        .vector_load_last(vector_load_last), .vector_load_done(vector_load_done),
        .qlinear_load_req(1'b0), .qlinear_load_ready(),
        .qlinear_load_address(32'd0), .qlinear_load_beats(32'd0),
        .qlinear_load_valid(), .qlinear_load_data_ready(1'b1),
        .qlinear_load_data(), .qlinear_load_last(), .qlinear_load_done(),
        .cfg_combine_start(cfg_combine_start), .cfg_combine_done(cfg_combine_done),
        .euler_start(euler_start), .euler_done(euler_done),
        .dicache_enable(dicache_enable), .dicache_threshold_q16(dicache_threshold_q16),
        .dicache_probe_error_q16(dicache_probe_error_q16),
        .dicache_cache_refresh_done(dicache_cache_refresh_done),
        .dicache_cache_refresh_branch_index(dicache_cache_refresh_branch_index),
        .dicache_reuse_done(1'b0),
        .dicache_probe_start(dicache_probe_start),
        .dicache_reuse_residual(dicache_reuse_residual),
        .dicache_refresh_residual(dicache_refresh_residual),
        .stage_request(stage_request), .stage_request_ready(stage_request_ready),
        .stage_request_index(stage_request_index), .stage_component(stage_component),
        .tile_select(tile_select), .n_tile_index(n_tile_index),
        .k_tile_index(k_tile_index), .stage_data(dma_stream_data),
        .stage_valid(dma_stream_valid), .stage_ready(1'b1),
        .stage_last(dma_stream_last), .stage_dma_done(dma_done),
        .dma_error(runtime_error), .m_axi_arid(axi_arid),
        .m_axi_araddr(axi_araddr), .m_axi_arlen(axi_arlen),
        .m_axi_arsize(axi_arsize), .m_axi_arburst(axi_arburst),
        .m_axi_arvalid(axi_arvalid), .m_axi_arready(axi_arready),
        .m_axi_rid(axi_rid), .m_axi_rdata(axi_rdata),
        .m_axi_rresp(axi_rresp), .m_axi_rlast(axi_rlast),
        .m_axi_rvalid(axi_rvalid), .m_axi_rready(axi_rready)
    );

    assign stage_request_index = 3'd0;
    assign stage_component = 2'd0;
    assign tile_select = 1'b1;
    assign n_tile_index = 16'd0;
    assign k_tile_index = 16'd0;

    always_ff @(posedge c0_ddr4_ui_clk or posedge c0_ddr4_ui_clk_sync_rst) begin
        if (c0_ddr4_ui_clk_sync_rst) begin
            context_done <= 1'b0;
            step_prep_done <= 1'b0;
            block_prepare_done <= 1'b0;
            block_done <= 1'b0;
            final_done <= 1'b0;
            cfg_combine_done <= 1'b0;
            euler_done <= 1'b0;
            stage_request <= 1'b0;
            fetch_launched <= 1'b0;
        end else begin
            context_done <= context_start;
            step_prep_done <= step_prep_start;
            block_prepare_done <= block_prepare_start;
            final_done <= final_start;
            cfg_combine_done <= cfg_combine_start;
            euler_done <= euler_start;
            block_done <= 1'b0;
            if (block_start && !fetch_launched) begin
                stage_request <= 1'b1;
                fetch_launched <= 1'b1;
            end
            if (stage_request && stage_request_ready) stage_request <= 1'b0;
            if (dma_done) block_done <= 1'b1;
            if (block_start && fetch_launched) block_done <= 1'b1;
        end
    end

    axi_to_mig_bridge #(
        .AXI_DATA_W(AXI_DATA_W), .AXI_ADDR_W(32), .AXI_ID_W(4),
        .MIG_DATA_W(MIG_DATA_W), .MIG_ADDR_W(MIG_ADDR_W),
        .MIG_MASK_W(MIG_DATA_W/8), .RATIO(AXI_DATA_W/MIG_DATA_W),
        .READ_FIFO_BEATS(8)
    ) u_bridge (
        .clk(c0_ddr4_ui_clk), .rst_n(~c0_ddr4_ui_clk_sync_rst),
        .s_axi_awid(axi_awid), .s_axi_awaddr(axi_awaddr), .s_axi_awlen(axi_awlen),
        .s_axi_awsize(axi_awsize), .s_axi_awburst(axi_awburst),
        .s_axi_awvalid(axi_awvalid), .s_axi_awready(axi_awready),
        .s_axi_wdata(axi_wdata), .s_axi_wstrb(axi_wstrb), .s_axi_wlast(axi_wlast),
        .s_axi_wvalid(axi_wvalid), .s_axi_wready(axi_wready),
        .s_axi_bid(axi_bid), .s_axi_bresp(axi_bresp), .s_axi_bvalid(axi_bvalid),
        .s_axi_bready(axi_bready), .s_axi_arid(axi_arid), .s_axi_araddr(axi_araddr),
        .s_axi_arlen(axi_arlen), .s_axi_arsize(axi_arsize), .s_axi_arburst(axi_arburst),
        .s_axi_arvalid(axi_arvalid), .s_axi_arready(axi_arready),
        .s_axi_rid(axi_rid), .s_axi_rdata(axi_rdata), .s_axi_rresp(axi_rresp),
        .s_axi_rlast(axi_rlast), .s_axi_rvalid(axi_rvalid), .s_axi_rready(axi_rready),
        .app_addr(app_addr), .app_cmd(app_cmd), .app_en(app_en), .app_hi_pri(app_hi_pri),
        .app_wdf_data(app_wdf_data), .app_wdf_end(app_wdf_end),
        .app_wdf_mask(app_wdf_mask), .app_wdf_wren(app_wdf_wren),
        .app_rd_data(app_rd_data), .app_rd_data_end(app_rd_data_end),
        .app_rd_data_valid(app_rd_data_valid), .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy)
    );

    mig_ddr4_model u_mig (
        .sys_rst(sys_rst), .c0_sys_clk_p(c0_sys_clk_p), .c0_sys_clk_n(c0_sys_clk_n),
        .c0_init_calib_complete(c0_init_calib_complete),
        .c0_ddr4_act_n(c0_ddr4_act_n), .c0_ddr4_adr(c0_ddr4_adr),
        .c0_ddr4_ba(c0_ddr4_ba), .c0_ddr4_bg(c0_ddr4_bg), .c0_ddr4_cke(c0_ddr4_cke),
        .c0_ddr4_odt(c0_ddr4_odt), .c0_ddr4_cs_n(c0_ddr4_cs_n),
        .c0_ddr4_ck_t(c0_ddr4_ck_t_int), .c0_ddr4_ck_c(c0_ddr4_ck_c_int),
        .c0_ddr4_reset_n(c0_ddr4_reset_n), .c0_ddr4_dm_dbi_n(c0_ddr4_dm_dbi_n),
        .c0_ddr4_dq(c0_ddr4_dq), .c0_ddr4_dqs_c(c0_ddr4_dqs_c),
        .c0_ddr4_dqs_t(c0_ddr4_dqs_t), .c0_ddr4_ui_clk(c0_ddr4_ui_clk),
        .c0_ddr4_ui_clk_sync_rst(c0_ddr4_ui_clk_sync_rst), .dbg_clk(dbg_clk),
        .c0_ddr4_app_addr(app_addr), .c0_ddr4_app_cmd(app_cmd),
        .c0_ddr4_app_en(app_en), .c0_ddr4_app_hi_pri(app_hi_pri),
        .c0_ddr4_app_wdf_data(app_wdf_data), .c0_ddr4_app_wdf_end(app_wdf_end),
        .c0_ddr4_app_wdf_mask(app_wdf_mask), .c0_ddr4_app_wdf_wren(app_wdf_wren),
        .c0_ddr4_app_rd_data(app_rd_data), .c0_ddr4_app_rd_data_end(app_rd_data_end),
        .c0_ddr4_app_rd_data_valid(app_rd_data_valid), .c0_ddr4_app_rdy(app_rdy),
        .c0_ddr4_app_wdf_rdy(app_wdf_rdy), .dbg_bus()
    );

    assign c0_ddr4_ck_t = c0_ddr4_ck_t_int[0];
    assign c0_ddr4_ck_c = c0_ddr4_ck_c_int[0];
    reg [16:0] addr_sdram[1:0];
    reg [1:0] ba_sdram[1:0], bg_sdram[1:0];
    reg [16:0] DDR4_ADRMOD[0:0];
    wire [31:0] cmdName;
    always @(*) begin
        addr_sdram[0] = c0_ddr4_adr; addr_sdram[1] = c0_ddr4_adr;
        ba_sdram[0] = c0_ddr4_ba; ba_sdram[1] = c0_ddr4_ba;
        bg_sdram[0] = c0_ddr4_bg; bg_sdram[1] = c0_ddr4_bg;
        if (c0_ddr4_act_n)
            casez (addr_sdram[0][16:14])
                4'b0100, 4'b0101: DDR4_ADRMOD[0] = addr_sdram[0] & 18'h1C7FF;
                default: DDR4_ADRMOD[0] = addr_sdram[0];
            endcase
        else DDR4_ADRMOD[0] = addr_sdram[0];
    end
    assign cmdName = '0;
    DDR4_if #(.CONFIGURED_DQ_BITS(8)) ddr4_if[0:0]();
    ddr4_model #(.CONFIGURED_DQ_BITS(8), .CONFIGURED_DENSITY(CONFIGURED_DENSITY))
        u_ddr4_model(.model_enable(model_enable), .iDDR4(ddr4_if[0]));
    for (genvar s=0; s<8; s++) begin : g_dq
        tran t_dq(ddr4_if[0].DQ[s], c0_ddr4_dq[s]);
    end
    tran t_dqs(ddr4_if[0].DQS_t, c0_ddr4_dqs_t[0]);
    tran c_dqs(ddr4_if[0].DQS_c, c0_ddr4_dqs_c[0]);
    tran t_dm(ddr4_if[0].DM_n, c0_ddr4_dm_dbi_n[0]);
    assign ddr4_if[0].BG=bg_sdram[0]; assign ddr4_if[0].BA=ba_sdram[0];
    assign ddr4_if[0].ADDR_17=1'b0; assign ddr4_if[0].ADDR=DDR4_ADRMOD[0][13:0];
    assign ddr4_if[0].CS_n=c0_ddr4_cs_n[0];
    assign ddr4_if[0].CK={c0_ddr4_ck_t,c0_ddr4_ck_c};
    assign ddr4_if[0].ACT_n=c0_ddr4_act_n;
    assign ddr4_if[0].RAS_n_A16=DDR4_ADRMOD[0][16];
    assign ddr4_if[0].CAS_n_A15=DDR4_ADRMOD[0][15];
    assign ddr4_if[0].WE_n_A14=DDR4_ADRMOD[0][14];
    assign ddr4_if[0].CKE=c0_ddr4_cke[0]; assign ddr4_if[0].ODT=c0_ddr4_odt[0];
    assign ddr4_if[0].PARITY=1'b0; assign ddr4_if[0].TEN=1'b0;
    assign ddr4_if[0].ZQ=1'b1; assign ddr4_if[0].PWR=1'b1;
    assign ddr4_if[0].VREF_CA=1'b1; assign ddr4_if[0].VREF_DQ=1'b1;
    assign ddr4_if[0].RESET_n=c0_ddr4_reset_n;

    initial begin
        #200000; sys_rst=1'b1;
        #5000; en_model=1'b1;
        #200000; sys_rst=1'b0;
    end

    initial begin : run_test
        logic [63:0] start_cycle, elapsed;
        realtime ui_edge_a, ui_edge_b;
        wait(c0_init_calib_complete === 1'b1);
        @(posedge c0_ddr4_ui_clk); ui_edge_a=$realtime;
        @(posedge c0_ddr4_ui_clk); ui_edge_b=$realtime;
        ui_period_ps=ui_edge_b-ui_edge_a;
        repeat(10) @(negedge c0_ddr4_ui_clk);
        @(negedge c0_ddr4_ui_clk); axi_awvalid=1'b1;
        do @(posedge c0_ddr4_ui_clk); while (!axi_awready);
        @(negedge c0_ddr4_ui_clk); axi_awvalid=1'b0;
        for (beat=0; beat<AXI_BEATS; beat++) begin
            @(negedge c0_ddr4_ui_clk);
            axi_wvalid = 1'b1;
            axi_wlast = (beat==AXI_BEATS-1);
            axi_wdata = pattern(beat);
            do @(posedge c0_ddr4_ui_clk); while (!axi_wready);
        end
        @(negedge c0_ddr4_ui_clk); axi_wvalid=0; axi_wlast=0;
        do @(posedge c0_ddr4_ui_clk); while (!axi_bvalid);
        if (axi_bid!==axi_awid || axi_bresp!==0) $fatal(1,"AXI write response mismatch");

        @(negedge c0_ddr4_ui_clk); runtime_start=1;
        start_cycle=cycle_count;
        @(negedge c0_ddr4_ui_clk); runtime_start=0;
        for (beat=0; beat<AXI_BEATS; beat++) begin
            do @(negedge c0_ddr4_ui_clk); while (!dma_stream_valid);
            if (dma_stream_data!==pattern(beat) ||
                dma_stream_last!==(beat==AXI_BEATS-1))
                $fatal(1,"DMA/MIG readback mismatch beat=%0d got=%h expected=%h last=%b",
                       beat,dma_stream_data,pattern(beat),dma_stream_last);
            @(posedge c0_ddr4_ui_clk);
        end
        @(negedge c0_ddr4_ui_clk);
        do @(negedge c0_ddr4_ui_clk); while (!runtime_done);
        if (runtime_busy || runtime_error || weight_burst_count != 1 || !fetch_launched)
            $fatal(1,"runtime/DMA completion mismatch busy=%b error=%b weight_bursts=%0d fetched=%b",
                   runtime_busy,runtime_error,weight_burst_count,fetch_launched);
        wait(vector_load_ready);
        @(negedge c0_ddr4_ui_clk); vector_load_req=1'b1;
        @(negedge c0_ddr4_ui_clk); vector_load_req=1'b0;
        for (beat=0; beat<AXI_BEATS; beat++) begin
            do @(negedge c0_ddr4_ui_clk); while (!vector_load_valid);
            if (vector_load_target_out!=4'd0 || vector_load_beat!=16'(20+beat) ||
                vector_load_data!==pattern(beat) ||
                vector_load_last!==(beat==AXI_BEATS-1))
                $fatal(1,"vector DMA/MIG mismatch beat=%0d dst=%0d data=%h expected=%h last=%b",
                       beat,vector_load_beat,vector_load_data,pattern(beat),vector_load_last);
            @(posedge c0_ddr4_ui_clk);
        end
        do @(negedge c0_ddr4_ui_clk); while (!vector_load_done);
        if (vector_burst_count != 1)
            $fatal(1,"expected one vector DMA burst, got %0d",vector_burst_count);
        elapsed=cycle_count-start_cycle;
        $display("MIG_DDR4_RUNTIME_DMA_PASS axi_width=%0d tile_beats=%0d vector_beats=%0d bytes=%0d cycles=%0d ui_clk_mhz=%0.2f",
                 AXI_DATA_W,AXI_BEATS,AXI_BEATS,2*AXI_BEATS*(AXI_DATA_W/8),elapsed,
                 1000000.0/ui_period_ps);
        $finish;
    end
    initial begin
        #(64'd100000000000);
        $fatal(1,"MIG DDR4 runtime-DMA test timeout waiting for calibration/traffic");
    end
endmodule
