// One-block end-to-end bench: actual block-13 payload -> MIG DDR4 -> DMA-model
// load ports -> connected Supra2 block -> output writeback to DDR4.
`timescale 1ps/1ps
module tb_supra2_block_ddr4_system;
    import fp16_utils_pkg::*;
    import block13_ddr_layout::*;
    localparam int AXI_DATA_W=512, AXI_BYTES=AXI_DATA_W/8;
    localparam int MIG_DATA_W=64, MIG_ADDR_W=29;
    localparam int ROWS=256, FEAT=576, XN=ROWS*FEAT, CTXN=32*FEAT;
    localparam int OUT_BYTES=XN*2, BURST_MAX=4096/AXI_BYTES;
    localparam int STG_N [0:6] = '{1728, 576, 576, 1152, 576, 2304, 576};
    localparam int STG_K [0:6] = '{576, 576, 576, 576, 576, 576, 2304};
    localparam real SYSCLK_PERIOD_PS=13501.0;
    import arch_package::*;
    localparam UTYPE_density CONFIGURED_DENSITY=_4G;

    logic sys_rst=0, en_model=0;
    wire model_enable;
    wire c0_init_calib_complete;
    assign model_enable=en_model;
    wire c0_ddr4_ui_clk, c0_ddr4_ui_clk_sync_rst, dbg_clk;
    wire c0_ddr4_act_n, c0_ddr4_reset_n;
    wire [16:0] c0_ddr4_adr;
    wire [1:0] c0_ddr4_ba,c0_ddr4_bg;
    wire [0:0] c0_ddr4_cke,c0_ddr4_odt,c0_ddr4_cs_n;
    wire [0:0] c0_ddr4_ck_t_int,c0_ddr4_ck_c_int;
    wire [0:0] c0_ddr4_dm_dbi_n,c0_ddr4_dqs_c,c0_ddr4_dqs_t;
    wire [7:0] c0_ddr4_dq;
    wire c0_ddr4_ck_t,c0_ddr4_ck_c;
    logic sys_clk=0;
    wire c0_sys_clk_p=sys_clk,c0_sys_clk_n=~sys_clk;
    always #(SYSCLK_PERIOD_PS/2.0) sys_clk=~sys_clk;

    logic [3:0] axi_awid=4'h3,axi_arid=4'h5;
    logic [31:0] axi_awaddr=0,axi_araddr=0;
    logic [7:0] axi_awlen=0,axi_arlen=0;
    logic [2:0] axi_awsize=3'd6,axi_arsize=3'd6;
    logic [1:0] axi_awburst=2'b01,axi_arburst=2'b01;
    logic axi_awvalid=0,axi_wvalid=0,axi_wlast=0;
    logic [AXI_DATA_W-1:0] axi_wdata=0;
    logic [AXI_BYTES-1:0] axi_wstrb='1;
    wire axi_awready,axi_wready;
    wire [3:0] axi_bid,axi_rid;
    wire [1:0] axi_bresp,axi_rresp;
    wire axi_bvalid,axi_arready,axi_rvalid,axi_rlast;
    wire [AXI_DATA_W-1:0] axi_rdata;
    logic axi_bready=1,axi_arvalid=0,axi_rready=0;
    wire [MIG_ADDR_W-1:0] app_addr;
    wire [2:0] app_cmd;
    wire app_en,app_hi_pri;
    wire [MIG_DATA_W-1:0] app_wdf_data,app_rd_data;
    wire app_wdf_end,app_wdf_wren,app_rd_data_end,app_rd_data_valid;
    wire [MIG_DATA_W/8-1:0] app_wdf_mask;
    wire app_rdy,app_wdf_rdy;

    wire clk=c0_ddr4_ui_clk;
    wire rst_n=~c0_ddr4_ui_clk_sync_rst;
    logic start=0;
    wire busy,done,dma_error;
    supra2_block_ddr_system_top #(.AXI_DATA_W(AXI_DATA_W)) system_dut(
        .clk(clk),.rst_n(rst_n),.start(start),.busy(busy),.done(done),.dma_error(dma_error),
        .host_awid(axi_awid),.host_awaddr(axi_awaddr),.host_awlen(axi_awlen),.host_awsize(axi_awsize),
        .host_awburst(axi_awburst),.host_awvalid(axi_awvalid),.host_awready(axi_awready),
        .host_wdata(axi_wdata),.host_wstrb(axi_wstrb),.host_wlast(axi_wlast),.host_wvalid(axi_wvalid),.host_wready(axi_wready),
        .host_bid(axi_bid),.host_bresp(axi_bresp),.host_bvalid(axi_bvalid),.host_bready(axi_bready),
        .host_arid(axi_arid),.host_araddr(axi_araddr),.host_arlen(axi_arlen),.host_arsize(axi_arsize),
        .host_arburst(axi_arburst),.host_arvalid(axi_arvalid),.host_arready(axi_arready),
        .host_rid(axi_rid),.host_rdata(axi_rdata),.host_rresp(axi_rresp),.host_rlast(axi_rlast),
        .host_rvalid(axi_rvalid),.host_rready(axi_rready),
        .app_addr(app_addr),.app_cmd(app_cmd),.app_en(app_en),.app_hi_pri(app_hi_pri),
        .app_wdf_data(app_wdf_data),.app_wdf_end(app_wdf_end),.app_wdf_mask(app_wdf_mask),.app_wdf_wren(app_wdf_wren),
        .app_rd_data(app_rd_data),.app_rd_data_end(app_rd_data_end),.app_rd_data_valid(app_rd_data_valid),
        .app_rdy(app_rdy),.app_wdf_rdy(app_wdf_rdy));

    mig_ddr4_model u_mig (
        .sys_rst(sys_rst),.c0_sys_clk_p(c0_sys_clk_p),.c0_sys_clk_n(c0_sys_clk_n),
        .c0_init_calib_complete(c0_init_calib_complete),.c0_ddr4_act_n(c0_ddr4_act_n),.c0_ddr4_adr(c0_ddr4_adr),
        .c0_ddr4_ba(c0_ddr4_ba),.c0_ddr4_bg(c0_ddr4_bg),.c0_ddr4_cke(c0_ddr4_cke),.c0_ddr4_odt(c0_ddr4_odt),
        .c0_ddr4_cs_n(c0_ddr4_cs_n),.c0_ddr4_ck_t(c0_ddr4_ck_t_int),.c0_ddr4_ck_c(c0_ddr4_ck_c_int),
        .c0_ddr4_reset_n(c0_ddr4_reset_n),.c0_ddr4_dm_dbi_n(c0_ddr4_dm_dbi_n),.c0_ddr4_dq(c0_ddr4_dq),
        .c0_ddr4_dqs_c(c0_ddr4_dqs_c),.c0_ddr4_dqs_t(c0_ddr4_dqs_t),.c0_ddr4_ui_clk(c0_ddr4_ui_clk),
        .c0_ddr4_ui_clk_sync_rst(c0_ddr4_ui_clk_sync_rst),.dbg_clk(dbg_clk),.c0_ddr4_app_addr(app_addr),
        .c0_ddr4_app_cmd(app_cmd),.c0_ddr4_app_en(app_en),.c0_ddr4_app_hi_pri(app_hi_pri),
        .c0_ddr4_app_wdf_data(app_wdf_data),.c0_ddr4_app_wdf_end(app_wdf_end),.c0_ddr4_app_wdf_mask(app_wdf_mask),
        .c0_ddr4_app_wdf_wren(app_wdf_wren),.c0_ddr4_app_rd_data(app_rd_data),.c0_ddr4_app_rd_data_end(app_rd_data_end),
        .c0_ddr4_app_rd_data_valid(app_rd_data_valid),.c0_ddr4_app_rdy(app_rdy),.c0_ddr4_app_wdf_rdy(app_wdf_rdy),.dbg_bus());

    assign c0_ddr4_ck_t=c0_ddr4_ck_t_int[0];
    assign c0_ddr4_ck_c=c0_ddr4_ck_c_int[0];
    reg [16:0] addr_sdram[1:0];
    reg [1:0] ba_sdram[1:0],bg_sdram[1:0];
    reg [16:0] DDR4_ADRMOD[0:0];
    wire [31:0] cmdName;
    always @(*) begin
        addr_sdram[0]=c0_ddr4_adr; addr_sdram[1]=c0_ddr4_adr;
        ba_sdram[0]=c0_ddr4_ba; ba_sdram[1]=c0_ddr4_ba;
        bg_sdram[0]=c0_ddr4_bg; bg_sdram[1]=c0_ddr4_bg;
        if(c0_ddr4_act_n) begin
            casez(addr_sdram[0][16:14])
                4'b0100,4'b0101: DDR4_ADRMOD[0]=addr_sdram[0]&18'h1C7FF;
                default: DDR4_ADRMOD[0]=addr_sdram[0];
            endcase
        end else DDR4_ADRMOD[0]=addr_sdram[0];
    end
    assign cmdName='0;
    DDR4_if #(.CONFIGURED_DQ_BITS(8)) ddr4_if[0:0]();
    ddr4_model #(.CONFIGURED_DQ_BITS(8),.CONFIGURED_DENSITY(CONFIGURED_DENSITY))
        u_ddr4_model(.model_enable(model_enable),.iDDR4(ddr4_if[0]));
    for(genvar g=0;g<8;g++) begin:g_dq
        tran t_dq(ddr4_if[0].DQ[g],c0_ddr4_dq[g]);
    end
    tran t_dqs(ddr4_if[0].DQS_t,c0_ddr4_dqs_t[0]);
    tran c_dqs(ddr4_if[0].DQS_c,c0_ddr4_dqs_c[0]);
    tran t_dm(ddr4_if[0].DM_n,c0_ddr4_dm_dbi_n[0]);
    assign ddr4_if[0].BG=bg_sdram[0]; assign ddr4_if[0].BA=ba_sdram[0];
    assign ddr4_if[0].ADDR_17=1'b0; assign ddr4_if[0].ADDR=DDR4_ADRMOD[0][13:0];
    assign ddr4_if[0].CS_n=c0_ddr4_cs_n[0]; assign ddr4_if[0].CK={c0_ddr4_ck_t,c0_ddr4_ck_c};
    assign ddr4_if[0].ACT_n=c0_ddr4_act_n; assign ddr4_if[0].RAS_n_A16=DDR4_ADRMOD[0][16];
    assign ddr4_if[0].CAS_n_A15=DDR4_ADRMOD[0][15]; assign ddr4_if[0].WE_n_A14=DDR4_ADRMOD[0][14];
    assign ddr4_if[0].CKE=c0_ddr4_cke[0]; assign ddr4_if[0].ODT=c0_ddr4_odt[0];
    assign ddr4_if[0].PARITY=1'b0; assign ddr4_if[0].TEN=1'b0; assign ddr4_if[0].ZQ=1'b1;
    assign ddr4_if[0].PWR=1'b1; assign ddr4_if[0].VREF_CA=1'b1; assign ddr4_if[0].VREF_DQ=1'b1;
    assign ddr4_if[0].RESET_n=c0_ddr4_reset_n;

    byte unsigned payload[],read_buffer[],output_bytes[];
    logic [15:0] golden[0:XN-1];
    integer payload_fd,payload_got,payload_bytes;
    integer axi_ui_cycles=0,ui_clock_cycles=0,compute_busy_cycles=0;
    integer dma_expected_addr=0,dma_expected_beats=0;
    integer dma_output_beat_check=0;
    logic dma_read_tracking=0;
    integer dma_write_expected_beats=0;
    logic dma_write_tracking=0;
    logic performance_active=0;
    realtime ui_period_ps=0.0,perf_start_ps,perf_end_ps,edge_a,edge_b;
    logic [63:0] total_ui_cycles,setup_write_cycles;
    integer error_words=0;
    real peak=0.0,max_abs=0.0,sum_sq=0.0,rel_peak,rmse;
    string payload_file;

    always @(posedge c0_ddr4_ui_clk) begin
        if(!c0_ddr4_ui_clk_sync_rst) begin
            ui_clock_cycles<=ui_clock_cycles+1;
            if(performance_active) axi_ui_cycles<=axi_ui_cycles+1;
        end
    end
    always @(posedge clk) begin
        if(system_dut.u_block.busy && performance_active) compute_busy_cycles<=compute_busy_cycles+1;
    end
    always @(negedge clk) begin
        if(rst_n && system_dut.wload_ack)
            check_dma_loaded_stage(system_dut.u_dma.stage_index);
    end
    // Verify every RTL-DMA AXI read against the exact byte image that was
    // preloaded through the host AXI port. This separates DDR/bridge/address
    // errors from local weight/scalar scatter and block arithmetic errors.
    always @(posedge clk) begin : check_dma_read_payload
        integer payload_offset;
        if(!rst_n) begin
            dma_read_tracking=0;
            dma_expected_addr=0;
            dma_expected_beats=0;
        end else begin
            if(system_dut.d_arvalid && system_dut.d_arready) begin
                if(dma_read_tracking) $fatal(1,"DMA issued overlapping read bursts");
                if(system_dut.d_arsize!==3'd6 || system_dut.d_araddr[5:0]!=0)
                    $fatal(1,"DMA read is not a full-width aligned 512-bit beat addr=%08x size=%0d",
                        system_dut.d_araddr,system_dut.d_arsize);
                if((int'(system_dut.d_araddr[11:0])+((int'(system_dut.d_arlen)+1)*AXI_BYTES))>4096)
                    $fatal(1,"DMA read burst crosses 4-KiB boundary addr=%08x len=%0d",
                        system_dut.d_araddr,system_dut.d_arlen);
                dma_expected_addr=system_dut.d_araddr;
                dma_expected_beats=int'(system_dut.d_arlen)+1;
                dma_read_tracking=1;
            end
            if(system_dut.d_rvalid && system_dut.d_rready) begin
                if(!dma_read_tracking) $fatal(1,"DMA read data without an accepted AR");
                payload_offset=dma_expected_addr-32'h0010_0000;
                if(payload_offset<0 || payload_offset+AXI_BYTES>payload_bytes)
                    $fatal(1,"DMA read outside payload addr=%08x offset=%0d",dma_expected_addr,payload_offset);
                for(integer lane=0;lane<AXI_BYTES;lane++)
                    if(system_dut.d_rdata[lane*8+:8]!==payload[payload_offset+lane])
                        $fatal(1,"DMA DDR data mismatch addr=%08x lane=%0d got=%02x exp=%02x",
                            dma_expected_addr+lane,lane,system_dut.d_rdata[lane*8+:8],payload[payload_offset+lane]);
                dma_expected_addr=dma_expected_addr+AXI_BYTES;
                dma_expected_beats=dma_expected_beats-1;
                if(system_dut.d_rlast!==(dma_expected_beats==0))
                    $fatal(1,"DMA RLAST mismatch addr=%08x remaining=%0d",dma_expected_addr,dma_expected_beats);
                if(dma_expected_beats==0) dma_read_tracking=0;
            end
        end
    end
    always @(posedge clk) begin : check_dma_write_protocol
        if(!rst_n) begin
            dma_write_tracking=0;
            dma_write_expected_beats=0;
            dma_output_beat_check=0;
        end else if(performance_active) begin
            if(system_dut.d_awvalid && system_dut.d_awready) begin
                if(dma_write_tracking) $fatal(1,"DMA issued overlapping write bursts");
                if(system_dut.d_awsize!==3'd6 || system_dut.d_awaddr[5:0]!=0)
                    $fatal(1,"DMA write is not a full-width aligned 512-bit beat addr=%08x size=%0d",
                        system_dut.d_awaddr,system_dut.d_awsize);
                if((int'(system_dut.d_awaddr[11:0])+((int'(system_dut.d_awlen)+1)*AXI_BYTES))>4096)
                    $fatal(1,"DMA write burst crosses 4-KiB boundary addr=%08x len=%0d",
                        system_dut.d_awaddr,system_dut.d_awlen);
                dma_write_expected_beats=int'(system_dut.d_awlen)+1;
                dma_write_tracking=1;
            end
            if(system_dut.d_wvalid && system_dut.d_wready) begin
                if(!dma_write_tracking) $fatal(1,"DMA write data without an accepted AW");
                if(system_dut.d_wlast!==(dma_write_expected_beats==1))
                    $fatal(1,"DMA WLAST mismatch remaining=%0d",dma_write_expected_beats);
                for(integer byte_lane=0;byte_lane<AXI_BYTES;byte_lane++) begin
                    logic [7:0] expected_byte;
                    expected_byte=(byte_lane%2==0)
                        ? system_dut.u_block.x_buf[dma_output_beat_check*(AXI_DATA_W/16)+byte_lane/2][7:0]
                        : system_dut.u_block.x_buf[dma_output_beat_check*(AXI_DATA_W/16)+byte_lane/2][15:8];
                    if(system_dut.d_wdata[byte_lane*8+:8]!==expected_byte)
                        $fatal(1,"DMA output pack mismatch beat=%0d byte_lane=%0d got=%02x expected=%02x",
                            dma_output_beat_check,byte_lane,system_dut.d_wdata[byte_lane*8+:8],expected_byte);
                end
                dma_output_beat_check=dma_output_beat_check+1;
                dma_write_expected_beats=dma_write_expected_beats-1;
                if(dma_write_expected_beats==0) dma_write_tracking=0;
            end
        end
    end
    initial begin #200000; sys_rst=1; #5000; en_model=1; #200000; sys_rst=0; end

    function automatic real half_to_real(input logic [15:0] v);
        integer e; real m,r;
        begin
            e=v[14:10];
            if(e==0) m=v[9:0]/1024.0; else m=1.0+v[9:0]/1024.0;
            if(e==0) r=m*(2.0**-14); else if(e==31) r=0.0; else r=m*(2.0**(e-15));
            half_to_real=v[15]?-r:r;
        end
    endfunction

    task automatic check_dma_loaded_stage(input integer stage_idx);
        integer n,k,weight_offset,scale_offset,bias_offset,source_offset;
        logic [31:0] expected_fp32;
        begin
            weight_offset=COMPONENT_OFFSET[WEIGHT_COMPONENT[stage_idx]];
            scale_offset=COMPONENT_OFFSET[SCALE_COMPONENT[stage_idx]];
            bias_offset=COMPONENT_OFFSET[BIAS_COMPONENT[stage_idx]];
            for(n=0;n<STG_N[stage_idx];n++) begin
                expected_fp32={payload[scale_offset+4*n+3],payload[scale_offset+4*n+2],
                    payload[scale_offset+4*n+1],payload[scale_offset+4*n]};
                if(system_dut.u_block.u_eng.ws_fp32[n]!==expected_fp32)
                    $fatal(1,"DMA scale scatter mismatch stage=%0d n=%0d got=%08x exp=%08x component=%0d base=%08x vector=%0d bias=%0b",
                        stage_idx,n,system_dut.u_block.u_eng.ws_fp32[n],expected_fp32,
                        system_dut.u_dma.component_index,system_dut.u_dma.component_base,
                        system_dut.u_dma.vector_index,system_dut.u_dma.vector_is_bias);
                expected_fp32={payload[bias_offset+4*n+3],payload[bias_offset+4*n+2],
                    payload[bias_offset+4*n+1],payload[bias_offset+4*n]};
                if(system_dut.u_block.u_eng.bias_fp32[n]!==expected_fp32)
                    $fatal(1,"DMA bias scatter mismatch stage=%0d n=%0d",stage_idx,n);
                for(k=0;k<STG_K[stage_idx];k++) begin
                    source_offset=weight_offset+((n/32)*(STG_K[stage_idx]/32)+(k/32))*1024+(k%32)*32+(n%32);
                    if(system_dut.u_block.u_eng.wt_int8[n*STG_K[stage_idx]+k]!==payload[source_offset])
                        $fatal(1,"DMA weight scatter mismatch stage=%0d n=%0d k=%0d",stage_idx,n,k);
                end
            end
            $display("RTL_DMA_STAGE_DATA_EXACT stage=%0d n=%0d k=%0d",stage_idx,STG_N[stage_idx],STG_K[stage_idx]);
        end
    endtask

    task automatic write_region(input integer address,input byte unsigned src[],output logic [63:0] elapsed);
        integer words,base,n,burst,beat,lane; logic [63:0] start_cycle;
        begin
            if((src.size()%AXI_BYTES)!=0) $fatal(1,"Unaligned DDR preload bytes=%0d",src.size());
            words=src.size()/AXI_BYTES; start_cycle=ui_clock_cycles;
            for(base=0;base<words;base+=BURST_MAX) begin
                burst=((words-base)>BURST_MAX)?BURST_MAX:(words-base);
                axi_awaddr=address+base*AXI_BYTES; axi_awlen=8'(burst-1);
                @(negedge c0_ddr4_ui_clk); axi_awvalid=1;
                do @(posedge c0_ddr4_ui_clk); while(!axi_awready);
                @(negedge c0_ddr4_ui_clk); axi_awvalid=0;
                for(beat=0;beat<burst;beat++) begin
                    @(negedge c0_ddr4_ui_clk);
                    for(lane=0;lane<AXI_BYTES;lane++) axi_wdata[lane*8+:8]=src[(base+beat)*AXI_BYTES+lane];
                    axi_wlast=(beat==burst-1); axi_wvalid=1;
                    do @(posedge c0_ddr4_ui_clk); while(!axi_wready);
                end
                @(negedge c0_ddr4_ui_clk); axi_wvalid=0; axi_wlast=0;
                do @(posedge c0_ddr4_ui_clk); while(!axi_bvalid);
                if(axi_bid!==axi_awid||axi_bresp!==0) $fatal(1,"AXI write response error");
            end
            @(negedge c0_ddr4_ui_clk); elapsed=ui_clock_cycles-start_cycle;
        end
    endtask

    task automatic read_region(input integer address,input integer component,input integer bytes,input bit compare_payload);
        integer words,base,burst,beat,lane,idx; logic [AXI_DATA_W-1:0] expected;
        begin
            if((bytes%AXI_BYTES)!=0) $fatal(1,"Unaligned DDR region bytes=%0d",bytes);
            read_buffer=new[bytes]; words=bytes/AXI_BYTES; axi_rready=1;
            for(base=0;base<words;base+=BURST_MAX) begin
                burst=((words-base)>BURST_MAX)?BURST_MAX:(words-base);
                axi_araddr=address+base*AXI_BYTES; axi_arlen=8'(burst-1);
                @(negedge c0_ddr4_ui_clk); axi_arvalid=1;
                do @(posedge c0_ddr4_ui_clk); while(!axi_arready);
                @(negedge c0_ddr4_ui_clk); axi_arvalid=0;
                for(beat=0;beat<burst;beat++) begin
                    do @(negedge c0_ddr4_ui_clk); while(!axi_rvalid);
                    expected='0;
                    for(lane=0;lane<AXI_BYTES;lane++) begin
                        idx=(base+beat)*AXI_BYTES+lane;
                        read_buffer[idx]=axi_rdata[lane*8+:8];
                        expected[lane*8+:8]=compare_payload?payload[component+idx]:read_buffer[idx];
                    end
                    if(axi_rid!==axi_arid||axi_rresp!==0||axi_rdata!==expected||axi_rlast!==(beat==burst-1))
                        $fatal(1,"DDR read mismatch component=%0d beat=%0d",component,base+beat);
                    @(posedge c0_ddr4_ui_clk);
                end
            end
        end
    endtask

    initial begin : run_system
        integer i,fd;
        realtime ui_edge_a,ui_edge_b;
        $display("TB_DIAGNOSTIC_BUILD=20261025B global_watchdog_cycles=24000000 rdata_stall_cycles=100000");
        wait(c0_init_calib_complete===1'b1);
        @(posedge c0_ddr4_ui_clk); ui_edge_a=$realtime;
        @(posedge c0_ddr4_ui_clk); ui_edge_b=$realtime; ui_period_ps=ui_edge_b-ui_edge_a;
        if(!$value$plusargs("PAYLOAD=%s",payload_file)||!$value$plusargs("BYTES=%d",payload_bytes))
            $fatal(1,"PAYLOAD and BYTES plusargs required");
        if(payload_bytes!=PAYLOAD_BYTES) $fatal(1,"Payload size differs from generated block layout");
        payload=new[payload_bytes]; fd=$fopen(payload_file,"rb");
        if(!fd) $fatal(1,"Cannot open payload %s",payload_file);
        payload_got=$fread(payload,fd); $fclose(fd);
        if(payload_got!=payload_bytes) $fatal(1,"Payload read size mismatch %0d/%0d",payload_got,payload_bytes);
        $readmemh("D:/graduation_design/npu-main/supra2_img_fpga/build/supra2_block13_top/golden_y.hex",golden,0,XN-1);

        repeat(10) @(negedge c0_ddr4_ui_clk);
        write_region(32'h0010_0000,payload,setup_write_cycles);
        $display("BLOCK13_DDR_PRELOAD_DONE bytes=%0d ui_cycles=%0d",payload_bytes,setup_write_cycles);

        wait(rst_n===1'b1);
        @(negedge clk); performance_active=1; perf_start_ps=$realtime; start=1;
        @(negedge clk); start=0;
        wait(system_dut.block_start===1'b1); #1;
        for(i=0;i<XN;i++) begin
            if(system_dut.u_block.x_buf[i]!=={payload[COMPONENT_OFFSET[0]+2*i+1],payload[COMPONENT_OFFSET[0]+2*i]})
                $fatal(1,"DMA x_buf load mismatch element=%0d",i);
        end
        for(i=0;i<CTXN;i++) begin
            if(system_dut.u_block.ctx_buf[i]!=={payload[COMPONENT_OFFSET[1]+2*i+1],payload[COMPONENT_OFFSET[1]+2*i]})
                $fatal(1,"DMA ctx_buf load mismatch element=%0d",i);
        end
        for(i=0;i<8*FEAT;i++) begin
            if(system_dut.u_block.param_buf[i]!=={payload[COMPONENT_OFFSET[2+i/FEAT]+2*(i%FEAT)+1],payload[COMPONENT_OFFSET[2+i/FEAT]+2*(i%FEAT)]})
                $fatal(1,"DMA param_buf load mismatch element=%0d",i);
        end
        $display("RTL_DMA_STATIC_INPUTS_EXACT x=%0d ctx=%0d adaln=%0d",XN,CTXN,8*FEAT);
        fork : block_watchdog
            begin wait(done===1'b1); end
            begin
                repeat(24_000_000) @(posedge clk);
                $display("DMA_TIMEOUT dma_state=%0d block_stage=%0d gemm_state=%0d wload_req=%0b wload_stage=%0d arvalid=%0b arready=%0b araddr=%08x arlen=%0d rvalid=%0b rready=%0b awvalid=%0b awready=%0b wvalid=%0b wready=%0b bvalid=%0b bready=%0b component=%0d stage=%0d n_tile=%0d k_tile=%0d tile_addr=%08x tile_beats=%0d beats_left=%0d bridge_active=%0b issue_left=%0d rsp_left=%0d occupancy=%0d fifo_count=%0d app_rdy=%0b app_rd_valid=%0b app_addr=%08x app_cmd=%0d",
                    system_dut.u_dma.state,system_dut.u_block.stage,
                    system_dut.u_block.u_eng.state,system_dut.wload_req,
                    system_dut.wload_stage,system_dut.d_arvalid,
                    system_dut.d_arready,system_dut.d_araddr,system_dut.d_arlen,system_dut.d_rvalid,
                    system_dut.d_rready,system_dut.d_awvalid,
                    system_dut.d_awready,system_dut.d_wvalid,
                    system_dut.d_wready,system_dut.d_bvalid,
                    system_dut.d_bready,system_dut.u_dma.component_index,
                    system_dut.u_dma.stage_index,system_dut.u_dma.n_tile_index,
                    system_dut.u_dma.k_tile_index,system_dut.u_dma.tile_read_addr,
                    system_dut.u_dma.tile_beats_received,system_dut.u_dma.beats_left,
                    system_dut.g_native_mig.u_bridge.r_active,system_dut.g_native_mig.u_bridge.r_issue_words_left,
                    system_dut.g_native_mig.u_bridge.r_rsp_words_left,system_dut.g_native_mig.u_bridge.r_word_occupancy,
                    system_dut.g_native_mig.u_bridge.r_fifo_count,system_dut.app_rdy,system_dut.app_rd_data_valid,
                    system_dut.app_addr,system_dut.app_cmd);
                $fatal(1,"DMA/block timeout");
            end
            begin : dma_rdata_watchdog
                integer rdata_stall_cycles;
                rdata_stall_cycles=0;
                forever begin
                    @(posedge clk);
                    if(system_dut.u_dma.state==5'd8 && !system_dut.d_rvalid) begin
                        rdata_stall_cycles=rdata_stall_cycles+1;
                        if(rdata_stall_cycles==100_000) begin
                            $display("DMA_RDATA_STALL dma_state=%0d block_stage=%0d component=%0d stage=%0d n_tile=%0d k_tile=%0d tile_addr=%08x tile_beats=%0d arvalid=%0b arready=%0b araddr=%08x arlen=%0d rvalid=%0b rready=%0b bridge_active=%0b issue_left=%0d rsp_left=%0d occupancy=%0d fifo_count=%0d app_rdy=%0b app_rd_valid=%0b app_addr=%08x app_cmd=%0d",
                                system_dut.u_dma.state,system_dut.u_block.stage,
                                system_dut.u_dma.component_index,system_dut.u_dma.stage_index,
                                system_dut.u_dma.n_tile_index,system_dut.u_dma.k_tile_index,
                                system_dut.u_dma.tile_read_addr,system_dut.u_dma.tile_beats_received,
                                system_dut.d_arvalid,system_dut.d_arready,system_dut.d_araddr,
                                system_dut.d_arlen,system_dut.d_rvalid,system_dut.d_rready,
                                system_dut.g_native_mig.u_bridge.r_active,
                                system_dut.g_native_mig.u_bridge.r_issue_words_left,
                                system_dut.g_native_mig.u_bridge.r_rsp_words_left,
                                system_dut.g_native_mig.u_bridge.r_word_occupancy,
                                system_dut.g_native_mig.u_bridge.r_fifo_count,system_dut.app_rdy,
                                system_dut.app_rd_data_valid,system_dut.app_addr,system_dut.app_cmd);
                            $fatal(1,"DMA RDATA stall watchdog");
                        end
                    end else rdata_stall_cycles=0;
                end
            end
        join_any
        disable block_watchdog;
        perf_end_ps=$realtime; performance_active=0;
        total_ui_cycles=axi_ui_cycles;
        if(dma_error) $fatal(1,"RTL DMA reported an AXI or descriptor error");
        if(dma_read_tracking || dma_write_tracking || (dma_write_expected_beats!=0))
            $fatal(1,"DMA ended with an incomplete AXI burst read=%0b write=%0b remaining_w=%0d",
                dma_read_tracking,dma_write_tracking,dma_write_expected_beats);
        for(i=0;i<XN;i++) begin
            if($isunknown(system_dut.u_block.x_buf[i])||$isunknown(golden[i])) error_words++;
            else begin
                real got,refv,err;
                got=half_to_real(system_dut.u_block.x_buf[i]); refv=half_to_real(golden[i]);
                err=(got>=refv)?got-refv:refv-got;
                if(err>max_abs) max_abs=err;
                if((refv>=0?refv:-refv)>peak) peak=(refv>=0)?refv:-refv;
                sum_sq=sum_sq+err*err;
            end
        end
        if(error_words!=0) $fatal(1,"block result has X/Z words=%0d",error_words);
        rel_peak=max_abs/((peak>1.0e-20)?peak:1.0); rmse=$sqrt(sum_sq/XN);
        if(rel_peak>0.02) $fatal(1,"block golden gate failed rel_peak=%0.6f",rel_peak);

        output_bytes=new[OUT_BYTES];
        for(i=0;i<XN;i++) begin
            output_bytes[i*2]=system_dut.u_block.x_buf[i][7:0];
            output_bytes[i*2+1]=system_dut.u_block.x_buf[i][15:8];
        end
        read_region(32'h0080_0000,0,OUT_BYTES,1'b0);
        for(i=0;i<OUT_BYTES;i++) if(read_buffer[i]!==output_bytes[i])
            $fatal(1,"DDR output writeback mismatch byte=%0d",i);
        $display("SUPRA2_BLOCK13_DDR_RTL_DMA_PASS compute_cycles=%0d ui_cycles=%0d elapsed_ms=%0.4f input_bytes=%0d output_bytes=%0d preload_write_cycles=%0d effective_MBps=%0.2f rel_peak=%0.6f rmse=%0.6f",
            compute_busy_cycles,total_ui_cycles,(perf_end_ps-perf_start_ps)/1.0e9,
            PAYLOAD_BYTES,OUT_BYTES,setup_write_cycles,
            ((PAYLOAD_BYTES+OUT_BYTES)*1.0e6)/((perf_end_ps-perf_start_ps)),rel_peak,rmse);
        $finish;
    end
    initial begin #(64'd100000000000000); $fatal(1,"DDR system simulation timeout"); end
endmodule
