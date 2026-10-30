// 基于 tb_supra2_block_ddr4_system：使用真实 block-13 输入与 golden 数据，
// 路径为主机预装载 -> 板级 MIG AXI512/DDR64 -> 实际 DMA/block -> DDR 回读。
// 编译导出的 ddr4_pl_axi_8 RTL，不使用 IP 黑盒；
// 本 testbench 之前先编译 ddr4_axi512_model/ddr4_sdram_model_wrapper.sv。
// 完整运行：+PAYLOAD=<binary> +BYTES=<package bytes> [+GOLDEN=<hex>]。
// 仅校准：+CALIB_ONLY=1，无需输入和 golden 文件。
// 可选：校准完成后导出命令引脚轨迹，使用 +DDR_COMMAND_TRACE=<output.csv>。
`timescale 1ps/1ps
module tb_supra2_block_ddr4_axi512_system #(parameter bit USE_TILE_PROGRAM=1'b0);
    import block13_ddr_layout::*;
    localparam int AXI_DATA_W=512, AXI_BYTES=64;
    localparam int ROWS=256, FEAT=576, XN=ROWS*FEAT, CTXN=32*FEAT;
    localparam int OUT_BYTES=XN*2, BURST_MAX=4096/AXI_BYTES;
    localparam int STG_N [0:6]='{1728,576,576,1152,576,2304,576};
    localparam int STG_K [0:6]='{576,576,576,576,576,576,2304};
    localparam int MAX_HOST_STALL=100_000, MAX_PERF_CYCLES=24_000_000;
    localparam real SYSCLK_PERIOD_PS=3332.0;
    localparam logic [31:0] DDR_BASE=32'h0010_0000, OUTPUT_BASE=32'h0080_0000;
    localparam logic [31:0] TILE_PROGRAM_BASE=32'h0090_0000, TILE_DEPENDENCY_BASE=32'h0091_0000;

    logic sys_clk=0, sys_rst=1, en_model=0;
    always #(SYSCLK_PERIOD_PS/2.0) sys_clk=~sys_clk;
    initial begin #5000; en_model=1; #400000; sys_rst=0; end
    wire c0_init_calib_complete;       // MIG 校准完成标志；为 1 后开始记录命令。
    wire c0_ddr4_ui_clk;               // MIG 的用户接口时钟，供 AXI/RTL 逻辑使用。
    wire c0_ddr4_ui_clk_sync_rst;      // 与用户接口时钟同步的复位。
    wire c0_ddr4_act_n;                // DDR4 ACT_n 命令引脚：低电平表示 ACT。
    wire c0_ddr4_reset_n;              // DDR4 器件复位引脚，低电平复位。
    wire [16:0] c0_ddr4_adr;           // DDR4 地址/命令总线；ACT 时作行地址，其他命令含 RAS/CAS/WE。
    wire [1:0] c0_ddr4_ba;             // DDR4 BA[1:0]，选择 bank。
    wire [0:0] c0_ddr4_bg;             // DDR4 BG[0]，选择 bank group；本板只连出一位。
    wire [0:0] c0_ddr4_cke;            // DDR4 时钟使能；低电平时器件不执行普通命令。
    wire [0:0] c0_ddr4_odt;            // DDR4 片上终端使能；当前命令时序检查器不使用它。
    wire [0:0] c0_ddr4_cs_n;           // DDR4 rank 片选；高电平表示本 rank 未被选中。
    wire [0:0] c0_ddr4_ck_t, c0_ddr4_ck_c; // DDR4 差分时钟正/负端；用 CK_T 上升沿计数。
    wire [63:0] c0_ddr4_dq;            // rank 的 64-bit 双向数据总线，四颗 x16 器件并联。
    wire [7:0] c0_ddr4_dm_dbi_n;       // 数据掩码/数据总线反相引脚；当前只记录命令，不建模其功能。
    wire [7:0] c0_ddr4_dqs_c;          // 8 个 byte lane 的 DQS 差分时钟负端。
    wire [7:0] c0_ddr4_dqs_t;          // 8 个 byte lane 的 DQS 差分时钟正端。
    wire [511:0] mig_debug;
    wire clk=c0_ddr4_ui_clk, rst_n=~c0_ddr4_ui_clk_sync_rst;
    logic start=0;
    wire busy,done,dma_error;

    logic [3:0] axi_awid=4'h3,axi_arid=4'h5;
    logic [31:0] axi_awaddr=0,axi_araddr=0;
    logic [7:0] axi_awlen=0,axi_arlen=0;
    logic [2:0] axi_awsize=3'd6,axi_arsize=3'd6;
    logic [1:0] axi_awburst=2'b01,axi_arburst=2'b01;
    logic axi_awvalid=0,axi_wvalid=0,axi_wlast=0;
    logic [511:0] axi_wdata=0;
    logic [63:0] axi_wstrb='1;
    wire axi_awready,axi_wready;
    wire [3:0] axi_bid,axi_rid;
    wire [1:0] axi_bresp,axi_rresp;
    wire axi_bvalid,axi_arready,axi_rvalid,axi_rlast;
    wire [511:0] axi_rdata;
    logic axi_bready=0,axi_arvalid=0,axi_rready=0;

    wire [3:0] mem_awid,mem_arid,mem_bid,mem_rid;
    wire [31:0] mem_awaddr,mem_araddr;
    wire [7:0] mem_awlen,mem_arlen;
    wire [2:0] mem_awsize,mem_arsize;
    wire [1:0] mem_awburst,mem_arburst,mem_bresp,mem_rresp;
    wire [511:0] mem_wdata,mem_rdata;
    wire [63:0] mem_wstrb;
    wire mem_awvalid,mem_awready,mem_wvalid,mem_wready,mem_wlast,mem_bvalid,mem_bready;
    wire mem_arvalid,mem_arready,mem_rvalid,mem_rready,mem_rlast;

    supra2_block_ddr_system_top #(.AXI_DATA_W(512),.EXTERNAL_AXI(1'b1),
        .USE_STAGE_PROGRAM(!USE_TILE_PROGRAM),.USE_TILE_PROGRAM(USE_TILE_PROGRAM)) system_dut (
        .clk(clk),.rst_n(rst_n),.start(start),.busy(busy),.done(done),.dma_error(dma_error),
        .host_awid(axi_awid),.host_awaddr(axi_awaddr),.host_awlen(axi_awlen),.host_awsize(axi_awsize),
        .host_awburst(axi_awburst),.host_awvalid(axi_awvalid),.host_awready(axi_awready),
        .host_wdata(axi_wdata),.host_wstrb(axi_wstrb),.host_wlast(axi_wlast),.host_wvalid(axi_wvalid),.host_wready(axi_wready),
        .host_bid(axi_bid),.host_bresp(axi_bresp),.host_bvalid(axi_bvalid),.host_bready(axi_bready),
        .host_arid(axi_arid),.host_araddr(axi_araddr),.host_arlen(axi_arlen),.host_arsize(axi_arsize),
        .host_arburst(axi_arburst),.host_arvalid(axi_arvalid),.host_arready(axi_arready),
        .host_rid(axi_rid),.host_rdata(axi_rdata),.host_rresp(axi_rresp),.host_rlast(axi_rlast),
        .host_rvalid(axi_rvalid),.host_rready(axi_rready),
        // The legacy native64 bridge is eliminated by EXTERNAL_AXI=1.
        .app_addr(),.app_cmd(),.app_en(),.app_hi_pri(),.app_wdf_data(),
        .app_wdf_end(),.app_wdf_mask(),.app_wdf_wren(),.app_rd_data(64'b0),
        .app_rd_data_end(1'b0),.app_rd_data_valid(1'b0),.app_rdy(1'b0),.app_wdf_rdy(1'b0),
        .mem_awid(mem_awid),.mem_awaddr(mem_awaddr),.mem_awlen(mem_awlen),.mem_awsize(mem_awsize),
        .mem_awburst(mem_awburst),.mem_awvalid(mem_awvalid),.mem_awready(mem_awready),
        .mem_wdata(mem_wdata),.mem_wstrb(mem_wstrb),.mem_wlast(mem_wlast),.mem_wvalid(mem_wvalid),.mem_wready(mem_wready),
        .mem_bid(mem_bid),.mem_bresp(mem_bresp),.mem_bvalid(mem_bvalid),.mem_bready(mem_bready),
        .mem_arid(mem_arid),.mem_araddr(mem_araddr),.mem_arlen(mem_arlen),.mem_arsize(mem_arsize),
        .mem_arburst(mem_arburst),.mem_arvalid(mem_arvalid),.mem_arready(mem_arready),
        .mem_rid(mem_rid),.mem_rdata(mem_rdata),.mem_rresp(mem_rresp),.mem_rlast(mem_rlast),
        .mem_rvalid(mem_rvalid),.mem_rready(mem_rready)
    );

    // 参数来自板级 XCI：MT40A256M16GE-075E、DDR64、AXI512、
    // address[30:0]、BG[0:0]、DDR 周期 833 ps、输入时钟周期 3332 ps。
    ddr4_pl_axi u_mig (
        .sys_rst(sys_rst),.c0_sys_clk_p(sys_clk),.c0_sys_clk_n(~sys_clk),
        .c0_init_calib_complete(c0_init_calib_complete),.c0_ddr4_ui_clk(c0_ddr4_ui_clk),
        .c0_ddr4_ui_clk_sync_rst(c0_ddr4_ui_clk_sync_rst),.c0_ddr4_aresetn(rst_n),
        .addn_ui_clkout1(),.dbg_clk(),.dbg_bus(mig_debug),
        .c0_ddr4_act_n(c0_ddr4_act_n),.c0_ddr4_adr(c0_ddr4_adr),
        .c0_ddr4_ba(c0_ddr4_ba),.c0_ddr4_bg(c0_ddr4_bg),
        .c0_ddr4_cke(c0_ddr4_cke),.c0_ddr4_odt(c0_ddr4_odt),.c0_ddr4_cs_n(c0_ddr4_cs_n),
        .c0_ddr4_ck_t(c0_ddr4_ck_t),.c0_ddr4_ck_c(c0_ddr4_ck_c),.c0_ddr4_reset_n(c0_ddr4_reset_n),
        .c0_ddr4_dm_dbi_n(c0_ddr4_dm_dbi_n),.c0_ddr4_dq(c0_ddr4_dq),
        .c0_ddr4_dqs_c(c0_ddr4_dqs_c),.c0_ddr4_dqs_t(c0_ddr4_dqs_t),
        .c0_ddr4_s_axi_awid(mem_awid),.c0_ddr4_s_axi_awaddr(mem_awaddr[30:0]),
        .c0_ddr4_s_axi_awlen(mem_awlen),.c0_ddr4_s_axi_awsize(mem_awsize),.c0_ddr4_s_axi_awburst(mem_awburst),
        .c0_ddr4_s_axi_awlock(1'b0),.c0_ddr4_s_axi_awcache(4'b0011),
        .c0_ddr4_s_axi_awprot(3'b000),.c0_ddr4_s_axi_awqos(4'b0000),
        .c0_ddr4_s_axi_awvalid(mem_awvalid),.c0_ddr4_s_axi_awready(mem_awready),
        .c0_ddr4_s_axi_wdata(mem_wdata),.c0_ddr4_s_axi_wstrb(mem_wstrb),
        .c0_ddr4_s_axi_wlast(mem_wlast),.c0_ddr4_s_axi_wvalid(mem_wvalid),.c0_ddr4_s_axi_wready(mem_wready),
        .c0_ddr4_s_axi_bid(mem_bid),.c0_ddr4_s_axi_bresp(mem_bresp),
        .c0_ddr4_s_axi_bvalid(mem_bvalid),.c0_ddr4_s_axi_bready(mem_bready),
        .c0_ddr4_s_axi_arid(mem_arid),.c0_ddr4_s_axi_araddr(mem_araddr[30:0]),
        .c0_ddr4_s_axi_arlen(mem_arlen),.c0_ddr4_s_axi_arsize(mem_arsize),.c0_ddr4_s_axi_arburst(mem_arburst),
        .c0_ddr4_s_axi_arlock(1'b0),.c0_ddr4_s_axi_arcache(4'b0011),
        .c0_ddr4_s_axi_arprot(3'b000),.c0_ddr4_s_axi_arqos(4'b0000),
        .c0_ddr4_s_axi_arvalid(mem_arvalid),.c0_ddr4_s_axi_arready(mem_arready),
        .c0_ddr4_s_axi_rid(mem_rid),.c0_ddr4_s_axi_rdata(mem_rdata),.c0_ddr4_s_axi_rresp(mem_rresp),
        .c0_ddr4_s_axi_rlast(mem_rlast),.c0_ddr4_s_axi_rvalid(mem_rvalid),.c0_ddr4_s_axi_rready(mem_rready)
    );
    for (genvar device=0;device<4;device++) begin : g_ddr_device
        ddr4_x16_model_wrapper memory_device (
            .model_enable(en_model),.reset_n(c0_ddr4_reset_n),.act_n(c0_ddr4_act_n),
            .adr(c0_ddr4_adr),.ba(c0_ddr4_ba),.bg(c0_ddr4_bg[0]),
            .cke(c0_ddr4_cke[0]),.odt(c0_ddr4_odt[0]),.cs_n(c0_ddr4_cs_n[0]),
            .ck_t(c0_ddr4_ck_t[0]),.ck_c(c0_ddr4_ck_c[0]),
            .dq(c0_ddr4_dq[device*16+:16]),.dqs_t(c0_ddr4_dqs_t[device*2+:2]),
            .dqs_c(c0_ddr4_dqs_c[device*2+:2]),.dm_dbi_n(c0_ddr4_dm_dbi_n[device*2+:2])
        );
    end

    // 逻辑 x64 rank 只记录一份共用命令总线。四颗 x16 器件接收相同命令，
    // 若逐颗记录，会把每条 ACT/RD/WR/PRE 重复计算四次。
    // CK 0 指校准完成后的第一个 DDR 上升沿，不是 MIG 初始化起点。
    integer ddr_trace_fd=0;             // 打开的 CSV 文件句柄；0 表示未启用轨迹输出。
    longint unsigned ddr_trace_ck=0;    // 校准后的 DDR CK 计数器，作为 CSV 的 ck 列。
    string ddr_trace_path;              // 加号参数 DDR_COMMAND_TRACE 指定的输出文件名。
    initial begin
        if ($value$plusargs("DDR_COMMAND_TRACE=%s",ddr_trace_path)) begin
            ddr_trace_fd=$fopen(ddr_trace_path,"w");
            if (ddr_trace_fd==0) $fatal(1,"cannot open DDR command trace %s",ddr_trace_path);
            $fwrite(ddr_trace_fd,"ck,act_n,adr,ba,bg,cs_n,cke\n");
        end
    end
    always @(posedge c0_ddr4_ck_t[0]) begin
        if (c0_init_calib_complete===1'b1) begin
            // 忽略未选中和 NOP 周期；回放器会直接跳到下一条命令所在 CK，
            // 并在该时间点结算尚未完成的自动 PRE。
            if (ddr_trace_fd!=0 && c0_ddr4_cs_n[0]===1'b0 &&
                c0_ddr4_cke[0]===1'b1 &&
                (c0_ddr4_act_n===1'b0 || c0_ddr4_adr[16:14]!==3'b111))
                $fwrite(ddr_trace_fd,"%0d,%0d,%05h,%0d,%0d,%0d,%0d\n",
                    ddr_trace_ck,c0_ddr4_act_n,c0_ddr4_adr,c0_ddr4_ba,
                    c0_ddr4_bg[0],c0_ddr4_cs_n[0],c0_ddr4_cke[0]);
            ddr_trace_ck<=ddr_trace_ck+1;
        end
    end

    byte unsigned payload[],read_buffer[],output_bytes[],tile_program_blob[],tile_dependency_blob[];
    byte unsigned tile_program_raw[],tile_dependency_raw[];
    logic [15:0] golden[0:XN-1];
    integer payload_bytes,payload_got,tile_program_bytes,tile_dependency_bytes;
    string payload_file,golden_file,tile_program_file,tile_dependency_file;
    integer calib_only=0;
    bit calibration_ready=0,performance_active=0,block_seen=0,block_finished=0;
    longint unsigned ui_clock_cycles=0,total_ui_cycles=0,setup_write_cycles=0,tile_program_write_cycles=0,tile_dependency_write_cycles=0;
    longint unsigned instruction_write_cycles=0;
    longint unsigned program_fetch_cycles=0,input_cycles=0,block_cycles=0,output_cycles=0,weight_wait_cycles=0;
    longint unsigned compute_busy_cycles=0,stage_cycles[0:20],stage_weight_wait[0:20];
    longint unsigned weight_prefetch_overlap_cycles=0, frontend_array_overlap_cycles=0;
    longint unsigned drain_array_overlap_cycles=0;
    longint unsigned dma_read_bytes=0,dma_write_bytes=0;
    logic [6:0] stages_checked=0;
    realtime ui_period_ps=0.0,perf_start_ps=0.0,perf_end_ps=0.0;
    integer dma_expected_addr=0,dma_expected_beats=0,dma_output_beat_check=0;
    integer dma_write_addr=0,dma_write_expected_beats=0,dma_b_count=0,dma_aw_count=0;
    bit dma_read_tracking=0,dma_write_tracking=0,dma_b_pending=0;
    integer rdata_stall_cycles=0,error_words=0;
    bit tile_read_pending=0;
    logic [31:0] tile_read_expected_addr=0;
    integer tile_ddr_read_beats=0;
    real peak=0.0,max_abs=0.0,sum_sq=0.0,rel_peak,rmse;

    function automatic real half_to_real(input logic [15:0] v);
        integer e; real m,r;
        begin
            e=v[14:10]; m=(e==0)?v[9:0]/1024.0:1.0+v[9:0]/1024.0;
            r=(e==0)?m*(2.0**-14):m*(2.0**(e-15));
            half_to_real=v[15]?-r:r;
        end
    endfunction
    function automatic string stage_name(input integer stage);
        case(stage)
            0:stage_name="LN0"; 1:stage_name="QKV"; 2:stage_name="ATTN_SELF";
            3:stage_name="SPROJ"; 4:stage_name="RES0"; 5:stage_name="LN1";
            6:stage_name="CQ"; 7:stage_name="CKV"; 8:stage_name="ATTN_CROSS";
            9:stage_name="CPROJ"; 10:stage_name="RES1"; 11:stage_name="LN2";
            12:stage_name="MLP0"; 13:stage_name="GELU"; 14:stage_name="MLP2";
            15:stage_name="RES2"; 16:stage_name="FIN"; 17:stage_name="IDLE";
            18:stage_name="PROGRAM_WAIT"; 19:stage_name="TILE_WEIGHT_LOAD";
            20:stage_name="TILE_NOOP";
            default:stage_name="INVALID";
        endcase
    endfunction

    task automatic direct_axi_diagnostic(input string reason);
        $display("BOARD_MIG_AXI512_DIAG reason=%s ui=%0d init=%b ui_rst=%b dma=%0d block_stage=%0d gemm=%0d wload=%b/%0d component=%0d n_tile=%0d k_tile=%0d tile_addr=%h tile_beats=%0d",
            reason,ui_clock_cycles,c0_init_calib_complete,c0_ddr4_ui_clk_sync_rst,
            system_dut.u_dma.state,system_dut.u_block.stage,system_dut.u_block.u_eng.state,
            system_dut.wload_req,system_dut.wload_stage,system_dut.u_dma.component_index,
            system_dut.u_dma.n_tile_index,system_dut.u_dma.k_tile_index,
            system_dut.u_dma.tile_read_addr,system_dut.u_dma.tile_beats_received);
        $display("BOARD_MIG_AXI512_PORTS AR=%b/%b addr=%h len=%0d R=%b/%b id=%h resp=%h last=%b AW=%b/%b addr=%h len=%0d W=%b/%b last=%b B=%b/%b resp=%h dbg=%h",
            mem_arvalid,mem_arready,mem_araddr,mem_arlen,mem_rvalid,mem_rready,mem_rid,mem_rresp,mem_rlast,
            mem_awvalid,mem_awready,mem_awaddr,mem_awlen,mem_wvalid,mem_wready,mem_wlast,
            mem_bvalid,mem_bready,mem_bresp,mig_debug[31:0]);
    endtask

    // Independent of block start, payload IO, and UI clock availability.
    initial begin
        #(64'd1_000_000_000);
        if(!calibration_ready) begin
            direct_axi_diagnostic("calibration timeout: 1 ms simulated time");
            $fatal(1,"BOARD_MIG_AXI512_CALIB_TIMEOUT init=%b ui_rst=%b; calibration/UI measurement did not complete within 1 ms",
                c0_init_calib_complete,c0_ddr4_ui_clk_sync_rst);
        end
    end
    initial begin #(64'd100_000_000_000_000); $fatal(1,"Board DDR system absolute simulation timeout"); end

    // Partition actual transaction cycles, not sums of standalone operators.
    // Block interval includes descriptor/weight waits; overlap is counted separately.
    initial for(integer stage=0;stage<21;stage++) begin
        stage_cycles[stage]=0; stage_weight_wait[stage]=0;
    end
    always @(posedge clk) begin : performance_counters
        if(rst_n) begin
            ui_clock_cycles<=ui_clock_cycles+1;
            if(performance_active) begin
                total_ui_cycles<=total_ui_cycles+1;
                if(total_ui_cycles>=MAX_PERF_CYCLES) begin
                    direct_axi_diagnostic("24M-cycle transaction watchdog");
                    $fatal(1,"DMA/block/output transaction timeout");
                end
                if(USE_TILE_PROGRAM && system_dut.tile_loader_busy) program_fetch_cycles<=program_fetch_cycles+1;
                else if(!block_seen && !system_dut.block_start) input_cycles<=input_cycles+1;
                else if(!block_finished && !system_dut.block_done) begin
                    block_cycles<=block_cycles+1;
                    if($isunknown(system_dut.u_block.stage) || system_dut.u_block.stage>20)
                        $fatal(1,"Invalid block stage during measured interval");
                    if(system_dut.program_error)
                        $fatal(1,"Stage program decoder rejected an instruction");
                    if(system_dut.tile_program_error)
                        $fatal(1,"Tile program decoder rejected an instruction pc=%0d",system_dut.tile_program_counter);
                    stage_cycles[system_dut.u_block.stage]<=stage_cycles[system_dut.u_block.stage]+1;
                    if(system_dut.wload_req &&
                       (!system_dut.u_block.WEIGHT_PREFETCH ||
                        (system_dut.u_block.gemm_stage && system_dut.u_block.ss==0))) begin
                        weight_wait_cycles<=weight_wait_cycles+1;
                        stage_weight_wait[system_dut.u_block.stage]<=stage_weight_wait[system_dut.u_block.stage]+1;
                    end
                    if(system_dut.u_block.WEIGHT_PREFETCH && system_dut.wload_req &&
                       system_dut.u_block.gemm_stage && system_dut.u_block.ss==2 &&
                       system_dut.wload_stage!=system_dut.u_block.wload_of_stage)
                        weight_prefetch_overlap_cycles<=weight_prefetch_overlap_cycles+1;
                    if(system_dut.u_block.eng_array_en && system_dut.u_block.u_eng.fe_state!=0)
                        frontend_array_overlap_cycles<=frontend_array_overlap_cycles+1;
                    if(system_dut.u_block.eng_array_en && system_dut.u_block.u_eng.drain_active)
                        drain_array_overlap_cycles<=drain_array_overlap_cycles+1;
                end else output_cycles<=output_cycles+1;
                if(system_dut.block_start) block_seen<=1;
                if(system_dut.block_done) block_finished<=1;
                if(system_dut.block_busy) compute_busy_cycles<=compute_busy_cycles+1;
            end
            // Include host preload/readback as well as the measured DMA run.
            if((ui_clock_cycles%200_000)==199_999)
                $display("BOARD_MIG_AXI512_PROGRESS ui=%0d perf_active=%b perf_cycles=%0d input=%0d block=%0d output=%0d stage=%0d dma=%0d weight_wait=%0d read_bytes=%0d write_bytes=%0d",
                    ui_clock_cycles+1,performance_active,total_ui_cycles,input_cycles,block_cycles,output_cycles,
                    system_dut.u_block.stage,system_dut.u_dma.state,weight_wait_cycles,dma_read_bytes,dma_write_bytes);
            // Explicitly reject address truncation at the board's 31-bit AXI port.
            if((mem_arvalid && mem_araddr[31]!==1'b0) || (mem_awvalid && mem_awaddr[31]!==1'b0))
                $fatal(1,"Address exceeds board MIG 31-bit AXI address space");
        end
    end

    always @(posedge clk) begin : check_dma_read_payload
        integer payload_offset;
        if(rst_n && performance_active) begin
            if(system_dut.d_arvalid && system_dut.d_arready) begin
                if(dma_read_tracking) $fatal(1,"DMA issued overlapping AR bursts");
                if(system_dut.d_arid!==4'h5 || system_dut.d_arsize!==3'd6 ||
                    system_dut.d_arburst!==2'b01 || system_dut.d_araddr[5:0]!==0 ||
                    (int'(system_dut.d_araddr[11:0])+(int'(system_dut.d_arlen)+1)*AXI_BYTES)>4096)
                    $fatal(1,"DMA AR alignment/attributes/page-boundary error");
                dma_expected_addr=system_dut.d_araddr;
                dma_expected_beats=int'(system_dut.d_arlen)+1;
                dma_read_tracking=1;
            end
            if(system_dut.d_rvalid && system_dut.d_rready) begin
                if(!dma_read_tracking || system_dut.d_rid!==4'h5 || system_dut.d_rresp!==2'b00)
                    $fatal(1,"DMA R without AR or bad ID/response");
                payload_offset=dma_expected_addr-DDR_BASE;
                if(payload_offset<0 || payload_offset+AXI_BYTES>payload_bytes)
                    $fatal(1,"DMA read outside payload addr=%h",dma_expected_addr);
                for(integer lane=0;lane<AXI_BYTES;lane++)
                    if(system_dut.d_rdata[lane*8+:8]!==payload[payload_offset+lane])
                        $fatal(1,"DMA DDR payload mismatch addr=%h lane=%0d got=%h expected=%h",
                            dma_expected_addr,lane,system_dut.d_rdata[lane*8+:8],payload[payload_offset+lane]);
                dma_expected_addr=dma_expected_addr+AXI_BYTES;
                dma_expected_beats=dma_expected_beats-1;
                dma_read_bytes=dma_read_bytes+AXI_BYTES;
                if(system_dut.d_rlast!==(dma_expected_beats==0)) $fatal(1,"DMA RLAST mismatch");
                if(dma_expected_beats==0) dma_read_tracking=0;
            end
            if(dma_read_tracking && system_dut.d_rready && !system_dut.d_rvalid) begin
                rdata_stall_cycles=rdata_stall_cycles+1;
                if(rdata_stall_cycles>=100_000) begin
                    direct_axi_diagnostic("DMA RDATA stall: 100000 cycles");
                    $fatal(1,"Direct MIG AXI RDATA stall watchdog");
                end
            end else rdata_stall_cycles=0;
        end
    end

    always @(posedge clk) begin : check_dma_write_protocol
        logic [7:0] expected_byte;
        if(rst_n && performance_active) begin
            if(system_dut.d_awvalid && system_dut.d_awready) begin
                if(dma_write_tracking || dma_b_pending) $fatal(1,"DMA overlapping AW");
                if(system_dut.d_awid!==4'h3 || system_dut.d_awsize!==3'd6 ||
                    system_dut.d_awburst!==2'b01 || system_dut.d_awaddr[5:0]!==0 ||
                    system_dut.d_awaddr!==(OUTPUT_BASE+32'(dma_write_bytes)) ||
                    (int'(system_dut.d_awaddr[11:0])+(int'(system_dut.d_awlen)+1)*AXI_BYTES)>4096)
                    $fatal(1,"DMA AW alignment/sequence/page-boundary error");
                dma_write_addr=system_dut.d_awaddr;
                dma_write_expected_beats=int'(system_dut.d_awlen)+1;
                dma_write_tracking=1; dma_b_pending=1; dma_aw_count=dma_aw_count+1;
            end
            if(system_dut.d_wvalid && system_dut.d_wready) begin
                if(!dma_write_tracking || dma_output_beat_check>=OUT_BYTES/AXI_BYTES)
                    $fatal(1,"Unexpected DMA W");
                if(system_dut.d_wlast!==(dma_write_expected_beats==1) || system_dut.d_wstrb!==64'hffff_ffff_ffff_ffff)
                    $fatal(1,"DMA WLAST/WSTRB mismatch");
                for(integer lane=0;lane<AXI_BYTES;lane++) begin
                    expected_byte=(lane%2==0)
                        ?system_dut.u_block.x_buf[dma_output_beat_check*32+lane/2][7:0]
                        :system_dut.u_block.x_buf[dma_output_beat_check*32+lane/2][15:8];
                    if($isunknown(expected_byte) || system_dut.d_wdata[lane*8+:8]!==expected_byte)
                        $fatal(1,"DMA output pack mismatch beat=%0d lane=%0d",dma_output_beat_check,lane);
                end
                dma_output_beat_check=dma_output_beat_check+1;
                dma_write_addr=dma_write_addr+AXI_BYTES;
                dma_write_bytes=dma_write_bytes+AXI_BYTES;
                dma_write_expected_beats=dma_write_expected_beats-1;
                if(dma_write_expected_beats==0) dma_write_tracking=0;
            end
            if(system_dut.d_bvalid && system_dut.d_bready) begin
                if(!dma_b_pending || dma_write_tracking || system_dut.d_bid!==4'h3 || system_dut.d_bresp!==2'b00)
                    $fatal(1,"DMA B premature or bad ID/response");
                dma_b_pending=0; dma_b_count=dma_b_count+1;
            end
        end
    end

    always @(posedge clk) begin : check_tile_program_ddr_reads
        integer offset;
        if(!rst_n) begin
            tile_read_pending<=0;
            tile_read_expected_addr<=0;
            tile_ddr_read_beats<=0;
        end else if(USE_TILE_PROGRAM && performance_active) begin
            if(system_dut.tile_arvalid && system_dut.tile_arready) begin
                if(tile_read_pending || system_dut.tile_arid!==4'h9 ||
                   system_dut.tile_arsize!==3'd6 || system_dut.tile_arlen!==0 ||
                   system_dut.tile_arburst!==2'b01 || system_dut.tile_araddr[5:0]!==0)
                    $fatal(1,"Tile-program DDR AR protocol error addr=%h",system_dut.tile_araddr);
                if(!((system_dut.tile_araddr>=TILE_PROGRAM_BASE && system_dut.tile_araddr<TILE_PROGRAM_BASE+32'(((tile_program_blob.size()+63)/64)*64)) ||
                     (system_dut.tile_araddr>=TILE_DEPENDENCY_BASE && system_dut.tile_araddr<TILE_DEPENDENCY_BASE+32'(((tile_dependency_blob.size()+63)/64)*64))))
                    $fatal(1,"Tile-program fetch outside DDR image addr=%h",system_dut.tile_araddr);
                tile_read_expected_addr<=system_dut.tile_araddr;
                tile_read_pending<=1;
            end
            if(system_dut.tile_rvalid && system_dut.tile_rready) begin
                if(!tile_read_pending || system_dut.tile_rid!==4'h9 ||
                   system_dut.tile_rresp!==2'b00 || system_dut.tile_rlast!==1'b1 ||
                   $isunknown(system_dut.tile_rdata))
                    $fatal(1,"Tile-program DDR R protocol/X error addr=%h",tile_read_expected_addr);
                if(tile_read_expected_addr>=TILE_PROGRAM_BASE && tile_read_expected_addr<TILE_DEPENDENCY_BASE) begin
                    offset=tile_read_expected_addr-TILE_PROGRAM_BASE;
                    for(integer lane=0;lane<AXI_BYTES;lane++)
                        if(system_dut.tile_rdata[lane*8+:8]!==tile_program_blob[offset+lane])
                            $fatal(1,"DDR instruction fetch mismatch addr=%h lane=%0d",tile_read_expected_addr,lane);
                end else begin
                    offset=tile_read_expected_addr-TILE_DEPENDENCY_BASE;
                    for(integer lane=0;lane<AXI_BYTES;lane++)
                        if(system_dut.tile_rdata[lane*8+:8]!==tile_dependency_blob[offset+lane])
                            $fatal(1,"DDR dependency fetch mismatch addr=%h lane=%0d",tile_read_expected_addr,lane);
                end
                tile_read_pending<=0;
                tile_ddr_read_beats<=tile_ddr_read_beats+1;
            end
        end
    end

    task automatic check_dma_loaded_stage(input integer stage_idx);
        integer weight_offset,scale_offset,bias_offset,source_offset;
        logic [31:0] expected_fp32;
        begin
            if(stage_idx<0 || stage_idx>6 || stages_checked[stage_idx]) $fatal(1,"Invalid/duplicate stage ACK %0d",stage_idx);
            weight_offset=COMPONENT_OFFSET[WEIGHT_COMPONENT[stage_idx]];
            scale_offset=COMPONENT_OFFSET[SCALE_COMPONENT[stage_idx]];
            bias_offset=COMPONENT_OFFSET[BIAS_COMPONENT[stage_idx]];
            for(integer n=0;n<STG_N[stage_idx];n++) begin
                expected_fp32={payload[scale_offset+4*n+3],payload[scale_offset+4*n+2],payload[scale_offset+4*n+1],payload[scale_offset+4*n]};
                if(system_dut.u_block.u_eng.ws_fp32[n]!==expected_fp32) $fatal(1,"DMA scale scatter mismatch stage=%0d n=%0d",stage_idx,n);
                expected_fp32={payload[bias_offset+4*n+3],payload[bias_offset+4*n+2],payload[bias_offset+4*n+1],payload[bias_offset+4*n]};
                if(system_dut.u_block.u_eng.bias_fp32[n]!==expected_fp32) $fatal(1,"DMA bias scatter mismatch stage=%0d n=%0d",stage_idx,n);
                for(integer k=0;k<STG_K[stage_idx];k++) begin
                    source_offset=weight_offset+((n/32)*(STG_K[stage_idx]/32)+k/32)*1024+(k%32)*32+n%32;
                    if(system_dut.u_block.u_eng.wt_int8[n*STG_K[stage_idx]+k]!==payload[source_offset])
                        $fatal(1,"DMA weight scatter mismatch stage=%0d n=%0d k=%0d",stage_idx,n,k);
                end
            end
            stages_checked[stage_idx]=1;
            $display("RTL_DMA_STAGE_DATA_EXACT stage=%0d n=%0d k=%0d",stage_idx,STG_N[stage_idx],STG_K[stage_idx]);
        end
    endtask
    always @(negedge clk) if(rst_n && system_dut.wload_ack) check_dma_loaded_stage(system_dut.u_dma.stage_index);

    // Inputs change on negedge. Observe handshakes/data at posedge before NBA.
    // Each host transaction has a bounded wait even outside the compute phase.
    task automatic wait_host(input integer channel);
        integer waited; bit accepted;
        begin
            waited=0; accepted=0;
            while(!accepted) begin
                @(posedge clk); waited++;
                case(channel)
                    0:accepted=(axi_awready===1'b1);
                    1:accepted=(axi_wready===1'b1);
                    2:accepted=(axi_bvalid===1'b1);
                    3:accepted=(axi_arready===1'b1);
                    4:accepted=(axi_rvalid===1'b1);
                    default:$fatal(1,"Invalid host channel");
                endcase
                if(waited>=MAX_HOST_STALL && !accepted) begin
                    direct_axi_diagnostic($sformatf("host channel %0d timeout",channel));
                    $fatal(1,"Host AXI timeout channel=%0d",channel);
                end
            end
        end
    endtask
    task automatic write_region(input logic [31:0] address,input byte unsigned src[],output longint unsigned elapsed);
        integer words,base,burst; longint unsigned start_cycle;
        begin
            if(src.size()%AXI_BYTES || address[5:0]!==0)
                $fatal(1,"Unaligned host preload addr=%h bytes=%0d",address,src.size());
            words=src.size()/AXI_BYTES; base=0; start_cycle=ui_clock_cycles;
            while(base<words) begin
                burst=((words-base)>BURST_MAX)?BURST_MAX:words-base;
                if(burst>(4096-int'((address+32'(base*AXI_BYTES))&32'hfff))/AXI_BYTES)
                    burst=(4096-int'((address+32'(base*AXI_BYTES))&32'hfff))/AXI_BYTES;
                @(negedge clk); axi_awaddr=address+32'(base*AXI_BYTES); axi_awlen=8'(burst-1); axi_awvalid=1;
                wait_host(0);
                @(negedge clk); axi_awvalid=0;
                for(integer beat=0;beat<burst;beat++) begin
                    @(negedge clk);
                    for(integer lane=0;lane<AXI_BYTES;lane++) axi_wdata[lane*8+:8]=src[(base+beat)*AXI_BYTES+lane];
                    axi_wlast=(beat==burst-1); axi_wvalid=1; wait_host(1);
                end
                @(negedge clk); axi_wvalid=0; axi_wlast=0; axi_bready=1;
                wait_host(2);
                if(axi_bid!==axi_awid || axi_bresp!==2'b00) $fatal(1,"Host preload AXI B error");
                @(negedge clk); axi_bready=0; base=base+burst;
                if((base%16384)==0) $display("BOARD_MIG_AXI512_PRELOAD_PROGRESS bytes=%0d/%0d",base*AXI_BYTES,src.size());
            end
            elapsed=ui_clock_cycles-start_cycle;
        end
    endtask
    task automatic read_region(input logic [31:0] address,input integer component,input integer bytes,input bit compare_payload);
        integer words,base,burst,idx;
        begin
            if(bytes%AXI_BYTES || address[5:0]!==0) $fatal(1,"Unaligned host readback");
            read_buffer=new[bytes]; words=bytes/AXI_BYTES; base=0;
            while(base<words) begin
                burst=((words-base)>BURST_MAX)?BURST_MAX:words-base;
                if(burst>(4096-int'((address+32'(base*AXI_BYTES))&32'hfff))/AXI_BYTES)
                    burst=(4096-int'((address+32'(base*AXI_BYTES))&32'hfff))/AXI_BYTES;
                @(negedge clk); axi_araddr=address+32'(base*AXI_BYTES); axi_arlen=8'(burst-1); axi_arvalid=1;
                wait_host(3);
                @(negedge clk); axi_arvalid=0; axi_rready=1;
                for(integer beat=0;beat<burst;beat++) begin
                    wait_host(4);
                    if(axi_rid!==axi_arid || axi_rresp!==2'b00 || axi_rlast!==(beat==burst-1) || $isunknown(axi_rdata))
                        $fatal(1,"Host readback protocol/X error beat=%0d",base+beat);
                    for(integer lane=0;lane<AXI_BYTES;lane++) begin
                        idx=(base+beat)*AXI_BYTES+lane;
                        read_buffer[idx]=axi_rdata[lane*8+:8];
                        if(compare_payload && read_buffer[idx]!==payload[component+idx])
                            $fatal(1,"DDR payload readback mismatch byte=%0d",component+idx);
                    end
                end
                @(negedge clk); axi_rready=0; base=base+burst;
            end
        end
    endtask

    initial begin : run_system
        integer fd; realtime edge_a,edge_b;
        longint unsigned stage_sum;
        integer tile_program_got,tile_dependency_got;
        $display("TB_BOARD_MIG_AXI512_DDR64 board_ip=ddr4_pl_axi_8 DDR4_4G_X16 devices=4 input_clock_ps=3332 memory_clock_ps=833");
        if(!$value$plusargs("CALIB_ONLY=%d",calib_only)) calib_only=0;
        wait(c0_init_calib_complete===1'b1 && rst_n===1'b1);
        @(posedge clk); edge_a=$realtime;
        repeat(64) @(posedge clk);
        edge_b=$realtime; ui_period_ps=(edge_b-edge_a)/64.0;
        if(ui_period_ps<=0.0) $fatal(1,"Invalid measured UI period");
        calibration_ready=1;
        if(calib_only!=0) begin
            $display("BOARD_MIG_AXI512_DDR64_CALIB_PASS ui_period_ps=%0.3f ui_MHz=%0.6f init=%b devices=4 dq_per_device=16",
                ui_period_ps,1.0e6/ui_period_ps,c0_init_calib_complete);
            $finish;
        end
        $display("BOARD_MIG_AXI512_DDR64_READY ui_period_ps=%0.3f ui_MHz=%0.6f",ui_period_ps,1.0e6/ui_period_ps);
        if(!$value$plusargs("PAYLOAD=%s",payload_file) || !$value$plusargs("BYTES=%d",payload_bytes))
            $fatal(1,"Full run requires PAYLOAD and BYTES plusargs");
        if(payload_bytes!=PAYLOAD_BYTES) $fatal(1,"Payload size differs from real generated layout");
        payload=new[payload_bytes]; fd=$fopen(payload_file,"rb");
        if(!fd) $fatal(1,"Cannot open payload %s",payload_file);
        payload_got=$fread(payload,fd); $fclose(fd);
        if(payload_got!=payload_bytes) $fatal(1,"Payload read size mismatch %0d/%0d",payload_got,payload_bytes);
        if(USE_TILE_PROGRAM) begin
            if(!$value$plusargs("TILE_PROGRAM_BIN=%s",tile_program_file) ||
               !$value$plusargs("TILE_DEPENDENCY_BIN=%s",tile_dependency_file))
                $fatal(1,"Tile-program run requires compiler binary plusargs");
            if(!$value$plusargs("TILE_PROGRAM_WORDS=%d",tile_program_bytes) ||
               !$value$plusargs("TILE_DEPENDENCY_WORDS=%d",tile_dependency_bytes))
                $fatal(1,"Tile-program run requires instruction/dependency word counts");
            tile_program_bytes=tile_program_bytes*16;
            tile_dependency_bytes=tile_dependency_bytes*2;
            tile_program_raw=new[tile_program_bytes];
            tile_dependency_raw=new[tile_dependency_bytes];
            fd=$fopen(tile_program_file,"rb"); if(!fd) $fatal(1,"Cannot open %s",tile_program_file);
            tile_program_got=$fread(tile_program_raw,fd); $fclose(fd);
            fd=$fopen(tile_dependency_file,"rb"); if(!fd) $fatal(1,"Cannot open %s",tile_dependency_file);
            tile_dependency_got=$fread(tile_dependency_raw,fd); $fclose(fd);
            if(tile_program_got!=tile_program_bytes || tile_dependency_got!=tile_dependency_bytes)
                $fatal(1,"Compiler binary size mismatch program=%0d/%0d dependencies=%0d/%0d",
                    tile_program_got,tile_program_bytes,tile_dependency_got,tile_dependency_bytes);
            tile_program_blob=new[((tile_program_bytes+63)/64)*64];
            tile_dependency_blob=new[((tile_dependency_bytes+63)/64)*64];
            foreach(tile_program_blob[i]) tile_program_blob[i]=0;
            foreach(tile_dependency_blob[i]) tile_dependency_blob[i]=0;
            for(integer i=0;i<tile_program_bytes;i++) tile_program_blob[i]=tile_program_raw[i];
            for(integer i=0;i<tile_dependency_bytes;i++) tile_dependency_blob[i]=tile_dependency_raw[i];
        end
        for(integer i=0;i<XN;i++) golden[i]='x;
        if($value$plusargs("GOLDEN=%s",golden_file)) begin
            fd=$fopen(golden_file,"r");
            if(!fd) $fatal(1,"Cannot open golden %s",golden_file);
            $fclose(fd); $readmemh(golden_file,golden,0,XN-1);
        end else $readmemh("D:/graduation_design/npu-main/supra2_img_fpga/build/supra2_block13_top/golden_y.hex",golden,0,XN-1);
        for(integer i=0;i<XN;i++)
            if($isunknown(golden[i]) || golden[i][14:10]==5'h1f) $fatal(1,"Invalid/missing golden word %0d",i);

        repeat(10) @(negedge clk);
        write_region(DDR_BASE,payload,setup_write_cycles);
        $display("BOARD_MIG_AXI512_PRELOAD_DONE bytes=%0d cycles=%0d ms=%0.6f",payload_bytes,setup_write_cycles,setup_write_cycles*ui_period_ps/1.0e9);
        read_region(DDR_BASE,0,payload_bytes,1'b1);
        $display("BOARD_MIG_AXI512_PAYLOAD_READBACK_EXACT bytes=%0d",payload_bytes);

        if(USE_TILE_PROGRAM) begin
            write_region(TILE_PROGRAM_BASE,tile_program_blob,tile_program_write_cycles);
            write_region(TILE_DEPENDENCY_BASE,tile_dependency_blob,tile_dependency_write_cycles);
            instruction_write_cycles=tile_program_write_cycles+tile_dependency_write_cycles;
            read_region(TILE_PROGRAM_BASE,0,tile_program_blob.size(),1'b0);
            foreach(tile_program_blob[i])
                if(read_buffer[i]!==tile_program_blob[i]) $fatal(1,"DDR tile instruction readback mismatch byte=%0d",i);
            read_region(TILE_DEPENDENCY_BASE,0,tile_dependency_blob.size(),1'b0);
            foreach(tile_dependency_blob[i])
                if(read_buffer[i]!==tile_dependency_blob[i]) $fatal(1,"DDR dependency readback mismatch byte=%0d",i);
            $display("BOARD_MIG_AXI512_TILE_PROGRAM_DDR_ROUNDTRIP_PASS instructions=%0d dependencies=%0d program_base=%h dependency_base=%h",
                tile_program_bytes/16,tile_dependency_bytes/2,TILE_PROGRAM_BASE,TILE_DEPENDENCY_BASE);
        end

        @(negedge clk); performance_active=1; perf_start_ps=$realtime; start=1;
        @(negedge clk); start=0;
        wait(system_dut.block_start===1'b1); #1;
        for(integer i=0;i<XN;i++)
            if(system_dut.u_block.x_buf[i]!=={payload[COMPONENT_OFFSET[0]+2*i+1],payload[COMPONENT_OFFSET[0]+2*i]})
                $fatal(1,"DMA x_buf load mismatch element=%0d",i);
        for(integer i=0;i<CTXN;i++)
            if(system_dut.u_block.ctx_buf[i]!=={payload[COMPONENT_OFFSET[1]+2*i+1],payload[COMPONENT_OFFSET[1]+2*i]})
                $fatal(1,"DMA ctx_buf load mismatch element=%0d",i);
        for(integer i=0;i<8*FEAT;i++)
            if(system_dut.u_block.param_buf[i]!=={payload[COMPONENT_OFFSET[2+i/FEAT]+2*(i%FEAT)+1],payload[COMPONENT_OFFSET[2+i/FEAT]+2*(i%FEAT)]})
                $fatal(1,"DMA param_buf load mismatch element=%0d",i);
        $display("RTL_DMA_STATIC_INPUTS_EXACT x=%0d ctx=%0d adaln=%0d",XN,CTXN,8*FEAT);
        wait(done===1'b1);
        @(negedge clk); perf_end_ps=$realtime; performance_active=0;
        if(dma_error!==1'b0 || busy!==1'b0 || !block_seen || !block_finished || stages_checked!==7'h7f)
            $fatal(1,"Incomplete block/DMA execution busy=%b error=%b stages=%h",busy,dma_error,stages_checked);
        if(dma_read_tracking || dma_write_tracking || dma_b_pending || dma_write_expected_beats!=0 ||
            dma_read_bytes!=payload_bytes || dma_write_bytes!=OUT_BYTES || dma_b_count!=dma_aw_count ||
            dma_output_beat_check!=OUT_BYTES/AXI_BYTES)
            $fatal(1,"Incomplete DMA traffic read=%0d write=%0d AW=%0d B=%0d",dma_read_bytes,dma_write_bytes,dma_aw_count,dma_b_count);
        stage_sum=0;
        if(USE_TILE_PROGRAM && system_dut.tile_program_done!==1'b1)
            $fatal(1,"Tile program did not consume END");
        if(USE_TILE_PROGRAM && (tile_read_pending || tile_ddr_read_beats!=((tile_program_bytes/16+3)/4)+((tile_dependency_bytes/2+31)/32)))
            $fatal(1,"Tile DDR fetch count mismatch beats=%0d pending=%b",tile_ddr_read_beats,tile_read_pending);
        for(integer stage=0;stage<21;stage++) stage_sum=stage_sum+stage_cycles[stage];
        if(stage_sum!=block_cycles || program_fetch_cycles+input_cycles+block_cycles+output_cycles!=total_ui_cycles)
            $fatal(1,"Performance cycle partition mismatch fetch=%0d input=%0d block=%0d output=%0d total=%0d",
                program_fetch_cycles,input_cycles,block_cycles,output_cycles,total_ui_cycles);
        for(integer i=0;i<XN;i++) begin
            if($isunknown(system_dut.u_block.x_buf[i]) || system_dut.u_block.x_buf[i][14:10]==5'h1f) error_words++;
            else begin
                real got,refv,err;
                got=half_to_real(system_dut.u_block.x_buf[i]); refv=half_to_real(golden[i]);
                err=(got>=refv)?got-refv:refv-got;
                if(err>max_abs) max_abs=err;
                if((refv>=0?refv:-refv)>peak) peak=(refv>=0)?refv:-refv;
                sum_sq=sum_sq+err*err;
            end
        end
        if(error_words!=0) $fatal(1,"Block result has X/Z/nonfinite words=%0d",error_words);
        rel_peak=max_abs/((peak>1.0e-20)?peak:1.0); rmse=$sqrt(sum_sq/XN);
        if(rel_peak>0.02) $fatal(1,"Block golden gate failed rel_peak=%0.6f",rel_peak);
        output_bytes=new[OUT_BYTES];
        for(integer i=0;i<XN;i++) begin
            output_bytes[i*2]=system_dut.u_block.x_buf[i][7:0];
            output_bytes[i*2+1]=system_dut.u_block.x_buf[i][15:8];
        end
        read_region(OUTPUT_BASE,0,OUT_BYTES,1'b0);
        for(integer i=0;i<OUT_BYTES;i++) if(read_buffer[i]!==output_bytes[i]) $fatal(1,"DDR output writeback mismatch byte=%0d",i);
        $display("BOARD_MIG_AXI512_PHASES ui_period_ps=%0.3f ui_MHz=%0.6f input_cycles=%0d input_ms=%0.6f block_cycles=%0d block_ms=%0.6f output_cycles=%0d output_ms=%0.6f weight_wait_cycles=%0d weight_wait_ms=%0.6f",
            ui_period_ps,1.0e6/ui_period_ps,input_cycles,input_cycles*ui_period_ps/1.0e9,
            block_cycles,block_cycles*ui_period_ps/1.0e9,output_cycles,output_cycles*ui_period_ps/1.0e9,
            weight_wait_cycles,weight_wait_cycles*ui_period_ps/1.0e9);
        for(integer stage=0;stage<21;stage++)
            $display("BOARD_MIG_AXI512_STAGE id=%0d name=%s cycles=%0d ms=%0.6f weight_wait_cycles=%0d",
                stage,stage_name(stage),stage_cycles[stage],stage_cycles[stage]*ui_period_ps/1.0e9,stage_weight_wait[stage]);
        // Transaction interval excludes calibration, host preload/readback,
        // and final debug DDR readback. These are simulator UI-cycle metrics.
        $display("SUPRA2_OVERLAP_COUNTERS weight_prefetch=%0d frontend_array=%0d drain_array=%0d",weight_prefetch_overlap_cycles,frontend_array_overlap_cycles,drain_array_overlap_cycles);
        $display("SUPRA2_BLOCK13_DDR_AXI512_PASS ui_period_ps=%0.3f ui_MHz=%0.6f ui_cycles=%0d program_fetch_cycles=%0d tile_ddr_read_beats=%0d compute_busy_cycles=%0d elapsed_ms=%0.6f instruction_write_cycles=%0d instruction_write_plus_run_cycles=%0d read_bytes=%0d write_bytes=%0d preload_cycles=%0d effective_MBps=%0.3f rel_peak=%0.6f rmse=%0.6f",
            ui_period_ps,1.0e6/ui_period_ps,total_ui_cycles,program_fetch_cycles,tile_ddr_read_beats,compute_busy_cycles,(perf_end_ps-perf_start_ps)/1.0e9,
            instruction_write_cycles,total_ui_cycles+instruction_write_cycles,
            dma_read_bytes,dma_write_bytes,setup_write_cycles,
            ((dma_read_bytes+dma_write_bytes)*1.0e6)/(perf_end_ps-perf_start_ps),rel_peak,rmse);
        $finish;
    end
endmodule
