// Full actual-size block-13 payload through AXI bridge, generated MIG and DDR4 model.
`timescale 1ps/1ps
module tb_supra2_block_ddr4_traffic;
    localparam int AXI_DATA_W = 128;
    localparam int MIG_DATA_W = 64;
    localparam int MIG_ADDR_W = 29;
    localparam int BURST_MAX = 256;
    localparam real SYSCLK_PERIOD_PS = 13501.0;
    import arch_package::*;
    localparam UTYPE_density CONFIGURED_DENSITY = _4G;

    logic sys_rst = 1'b0, en_model = 1'b0;
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
    wire c0_sys_clk_p = sys_clk, c0_sys_clk_n = ~sys_clk;
    always #(SYSCLK_PERIOD_PS/2.0) sys_clk = ~sys_clk;

    logic [3:0] axi_awid = 4'h3, axi_arid = 4'h5;
    logic [31:0] axi_awaddr = 32'h0010_0000, axi_araddr = 32'h0010_0000;
    logic [7:0] axi_awlen = 0, axi_arlen = 0;
    logic [2:0] axi_awsize = 3'd4, axi_arsize = 3'd4;
    logic [1:0] axi_awburst = 2'b01, axi_arburst = 2'b01;
    logic axi_awvalid = 0, axi_wvalid = 0, axi_wlast = 0;
    logic [127:0] axi_wdata = 0;
    logic [15:0] axi_wstrb = '1;
    wire axi_awready, axi_wready;
    wire [3:0] axi_bid, axi_rid;
    wire [1:0] axi_bresp, axi_rresp;
    wire axi_bvalid, axi_arready, axi_rvalid, axi_rlast;
    wire [127:0] axi_rdata;
    logic axi_bready = 1, axi_arvalid = 0, axi_rready = 0;
    wire [MIG_ADDR_W-1:0] app_addr;
    wire [2:0] app_cmd;
    wire app_en, app_hi_pri;
    wire [MIG_DATA_W-1:0] app_wdf_data, app_rd_data;
    wire app_wdf_end, app_wdf_wren, app_rd_data_end, app_rd_data_valid;
    wire [MIG_DATA_W/8-1:0] app_wdf_mask;
    wire app_rdy, app_wdf_rdy;

    byte unsigned payload[];
    string payload_file;
    integer payload_bytes, fd, nread, burst, beat, lane, offset;
    logic [63:0] cycle_count = 0, write_start, write_cycles, read_start, read_cycles;
    realtime ui_period_ps, edge_a, edge_b;

    always @(posedge c0_ddr4_ui_clk)
        if (!c0_ddr4_ui_clk_sync_rst) cycle_count <= cycle_count + 1;

    axi_to_mig_bridge #(.AXI_DATA_W(128), .AXI_ADDR_W(32), .AXI_ID_W(4),
        .MIG_DATA_W(64), .MIG_ADDR_W(MIG_ADDR_W), .MIG_MASK_W(8),
        .RATIO(2), .READ_FIFO_BEATS(8)) u_bridge (
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
        .app_rd_data_valid(app_rd_data_valid), .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy));

    mig_ddr4_model u_mig (
        .sys_rst(sys_rst), .c0_sys_clk_p(c0_sys_clk_p), .c0_sys_clk_n(c0_sys_clk_n),
        .c0_init_calib_complete(c0_init_calib_complete), .c0_ddr4_act_n(c0_ddr4_act_n),
        .c0_ddr4_adr(c0_ddr4_adr), .c0_ddr4_ba(c0_ddr4_ba), .c0_ddr4_bg(c0_ddr4_bg),
        .c0_ddr4_cke(c0_ddr4_cke), .c0_ddr4_odt(c0_ddr4_odt), .c0_ddr4_cs_n(c0_ddr4_cs_n),
        .c0_ddr4_ck_t(c0_ddr4_ck_t_int), .c0_ddr4_ck_c(c0_ddr4_ck_c_int),
        .c0_ddr4_reset_n(c0_ddr4_reset_n), .c0_ddr4_dm_dbi_n(c0_ddr4_dm_dbi_n),
        .c0_ddr4_dq(c0_ddr4_dq), .c0_ddr4_dqs_c(c0_ddr4_dqs_c), .c0_ddr4_dqs_t(c0_ddr4_dqs_t),
        .c0_ddr4_ui_clk(c0_ddr4_ui_clk), .c0_ddr4_ui_clk_sync_rst(c0_ddr4_ui_clk_sync_rst),
        .dbg_clk(dbg_clk), .c0_ddr4_app_addr(app_addr), .c0_ddr4_app_cmd(app_cmd),
        .c0_ddr4_app_en(app_en), .c0_ddr4_app_hi_pri(app_hi_pri),
        .c0_ddr4_app_wdf_data(app_wdf_data), .c0_ddr4_app_wdf_end(app_wdf_end),
        .c0_ddr4_app_wdf_mask(app_wdf_mask), .c0_ddr4_app_wdf_wren(app_wdf_wren),
        .c0_ddr4_app_rd_data(app_rd_data), .c0_ddr4_app_rd_data_end(app_rd_data_end),
        .c0_ddr4_app_rd_data_valid(app_rd_data_valid), .c0_ddr4_app_rdy(app_rdy),
        .c0_ddr4_app_wdf_rdy(app_wdf_rdy), .dbg_bus());

    assign c0_ddr4_ck_t = c0_ddr4_ck_t_int[0];
    assign c0_ddr4_ck_c = c0_ddr4_ck_c_int[0];
    reg [16:0] addr_sdram[1:0];
    reg [1:0] ba_sdram[1:0], bg_sdram[1:0];
    reg [16:0] DDR4_ADRMOD[0:0];
    wire [31:0] cmdName;
    always @(*) begin
        addr_sdram[0]=c0_ddr4_adr; addr_sdram[1]=c0_ddr4_adr;
        ba_sdram[0]=c0_ddr4_ba; ba_sdram[1]=c0_ddr4_ba;
        bg_sdram[0]=c0_ddr4_bg; bg_sdram[1]=c0_ddr4_bg;
        if (c0_ddr4_act_n) begin
            casez (addr_sdram[0][16:14])
                4'b0100,4'b0101: DDR4_ADRMOD[0]=addr_sdram[0]&18'h1C7FF;
                default: DDR4_ADRMOD[0]=addr_sdram[0];
            endcase
        end else DDR4_ADRMOD[0]=addr_sdram[0];
    end
    assign cmdName='0;
    DDR4_if #(.CONFIGURED_DQ_BITS(8)) ddr4_if[0:0]();
    ddr4_model #(.CONFIGURED_DQ_BITS(8),.CONFIGURED_DENSITY(CONFIGURED_DENSITY))
        u_ddr4_model(.model_enable(model_enable),.iDDR4(ddr4_if[0]));
    for (genvar g=0;g<8;g++) begin : g_dq
        tran t_dq(ddr4_if[0].DQ[g],c0_ddr4_dq[g]);
    end
    tran t_dqs(ddr4_if[0].DQS_t,c0_ddr4_dqs_t[0]);
    tran c_dqs(ddr4_if[0].DQS_c,c0_ddr4_dqs_c[0]);
    tran t_dm(ddr4_if[0].DM_n,c0_ddr4_dm_dbi_n[0]);
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

    initial begin #200000; sys_rst=1; #5000; en_model=1; #200000; sys_rst=0; end

    initial begin : run_traffic
        logic [127:0] expected_word;
        integer words, words_in_burst, burst_base;
        if (!$value$plusargs("PAYLOAD=%s",payload_file) ||
            !$value$plusargs("BYTES=%d",payload_bytes))
            $fatal(1,"PAYLOAD and BYTES plusargs are required");
        if (payload_bytes<=0 || (payload_bytes%16)!=0)
            $fatal(1,"payload size must be positive and 16-byte aligned");
        payload=new[payload_bytes];
        fd=$fopen(payload_file,"rb");
        if (!fd) $fatal(1,"cannot open payload %s",payload_file);
        nread=$fread(payload,fd); $fclose(fd);
        if (nread!=payload_bytes) $fatal(1,"payload length mismatch %0d/%0d",nread,payload_bytes);
        words=payload_bytes/16;
        wait(c0_init_calib_complete===1'b1);
        @(posedge c0_ddr4_ui_clk); edge_a=$realtime;
        @(posedge c0_ddr4_ui_clk); edge_b=$realtime;
        ui_period_ps=edge_b-edge_a;
        repeat(10) @(negedge c0_ddr4_ui_clk);

        write_start=cycle_count;
        for (burst_base=0;burst_base<words;burst_base+=BURST_MAX) begin
            words_in_burst=((words-burst_base)>BURST_MAX)?BURST_MAX:(words-burst_base);
            axi_awaddr=32'h0010_0000+burst_base*16; axi_awlen=8'(words_in_burst-1);
            @(negedge c0_ddr4_ui_clk); axi_awvalid=1;
            do @(posedge c0_ddr4_ui_clk); while(!axi_awready);
            @(negedge c0_ddr4_ui_clk); axi_awvalid=0;
            for (beat=0;beat<words_in_burst;beat++) begin
                @(negedge c0_ddr4_ui_clk);
                for(lane=0;lane<16;lane++)
                    axi_wdata[lane*8+:8]=payload[(burst_base+beat)*16+lane];
                axi_wlast=(beat==words_in_burst-1); axi_wvalid=1;
                do @(posedge c0_ddr4_ui_clk); while(!axi_wready);
            end
            @(negedge c0_ddr4_ui_clk); axi_wvalid=0; axi_wlast=0;
            do @(posedge c0_ddr4_ui_clk); while(!axi_bvalid);
            if(axi_bid!==axi_awid||axi_bresp!==0) $fatal(1,"write response error");
        end
        @(negedge c0_ddr4_ui_clk); write_cycles=cycle_count-write_start;

        read_start=cycle_count;
        axi_rready=1;
        for (burst_base=0;burst_base<words;burst_base+=BURST_MAX) begin
            words_in_burst=((words-burst_base)>BURST_MAX)?BURST_MAX:(words-burst_base);
            axi_araddr=32'h0010_0000+burst_base*16; axi_arlen=8'(words_in_burst-1);
            @(negedge c0_ddr4_ui_clk); axi_arvalid=1;
            do @(posedge c0_ddr4_ui_clk); while(!axi_arready);
            @(negedge c0_ddr4_ui_clk); axi_arvalid=0;
            for (beat=0;beat<words_in_burst;beat++) begin
                do @(negedge c0_ddr4_ui_clk); while(!axi_rvalid);
                expected_word='0;
                for(lane=0;lane<16;lane++)
                    expected_word[lane*8+:8]=payload[(burst_base+beat)*16+lane];
                if(axi_rid!==axi_arid||axi_rresp!==0||axi_rdata!==expected_word||
                   axi_rlast!==(beat==words_in_burst-1))
                    $fatal(1,"DDR mismatch global beat=%0d",burst_base+beat);
                @(posedge c0_ddr4_ui_clk);
            end
        end
        @(negedge c0_ddr4_ui_clk); read_cycles=cycle_count-read_start;
        $display("SUPRA2_BLOCK13_DDR4_PASS bytes=%0d beats=%0d bursts=%0d write_cycles=%0d read_cycles=%0d ui_clk_mhz=%0.2f read_MBps=%0.2f",
            payload_bytes,words,(words+BURST_MAX-1)/BURST_MAX,write_cycles,read_cycles,
            1000000.0/ui_period_ps,(payload_bytes*1000000.0)/(read_cycles*ui_period_ps));
        $finish;
    end
    initial begin #(64'd1000000000000); $fatal(1,"DDR traffic timeout"); end
endmodule
