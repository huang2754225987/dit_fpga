`timescale 1ns/1ps
`default_nettype none

module tb_layer0_fake_ddr;
    parameter real CLK_PERIOD = 1000.0 / 197.511;
    parameter int UCODE_DEPTH = 262144;
    parameter int AXI_DATA_W  = 128;
    parameter int SEQ_LEN     = 32;
    parameter bit ENABLE_DETAILED_PROFILE = 1'b1;
    localparam real FAKE_DDR_READ_LATENCY_NS = 50.0;
    localparam real FAKE_DDR_BEAT_INTERVAL_NS = 10.0;
    localparam int FAKE_DDR_READ_LATENCY_CYCLES =
        $rtoi(FAKE_DDR_READ_LATENCY_NS / CLK_PERIOD + 0.5);
    localparam int FAKE_DDR_BEAT_INTERVAL_CYCLES =
        $rtoi(FAKE_DDR_BEAT_INTERVAL_NS / CLK_PERIOD + 0.5);

    logic clk, rst_n;
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---- AXI-Lite ----
    logic [31:0] s_axil_awaddr, s_axil_wdata, s_axil_araddr;
    logic        s_axil_awvalid, s_axil_wvalid, s_axil_arvalid, s_axil_bready, s_axil_rready;
    logic [3:0]  s_axil_wstrb;
    wire         s_axil_awready, s_axil_wready, s_axil_arready;
    wire  [31:0] s_axil_rdata;
    wire  [1:0]  s_axil_bresp, s_axil_rresp;
    wire         s_axil_bvalid, s_axil_rvalid;

    // ---- AXI4 Read ----
    wire  [3:0]  m_axi_arid;   wire  [31:0] m_axi_araddr;
    wire  [7:0]  m_axi_arlen;  wire  [2:0]  m_axi_arsize;
    wire  [1:0]  m_axi_arburst; wire m_axi_arvalid;
    wire         m_axi_rready;
    logic        m_axi_arready;
    logic [3:0]  m_axi_rid;
    logic [AXI_DATA_W-1:0] m_axi_rdata;
    logic [1:0]  m_axi_rresp;
    logic        m_axi_rlast;
    logic        m_axi_rvalid;

    // ---- AXI4 Write (tie off) ----
    wire  [3:0]  m_axi_awid;   wire  [31:0] m_axi_awaddr;
    wire  [7:0]  m_axi_awlen;  wire  [2:0]  m_axi_awsize;
    wire  [1:0]  m_axi_awburst; wire m_axi_awvalid;
    wire  [AXI_DATA_W-1:0] m_axi_wdata;
    wire  [AXI_DATA_W/8-1:0] m_axi_wstrb;
    wire         m_axi_wlast, m_axi_wvalid, m_axi_bready;

    // ---- UCODE preload ----
    logic        tb_ucode_wr_en;
    logic [$clog2(UCODE_DEPTH)-1:0] tb_ucode_wr_addr;
    logic [31:0] tb_ucode_wr_data;

    logic m_axi_awready, m_axi_wready, m_axi_bvalid;
    logic [3:0] m_axi_bid;
    logic [1:0] m_axi_bresp;
    logic refill_request, refill_complete = 0;
    bit refill_busy = 0;
    logic [31:0] refill_address;
    integer refill_bytes;
    integer image_fd = 0, image_size, refill_rc;
    byte unsigned refill_buffer [0:4095];
    byte unsigned activation_store [0:SEQ_LEN*4096-1];
    byte unsigned prefill_kv_store [0:24*2*SEQ_LEN*4096-1];
    byte unsigned decode_seed [0:4095];
    bit zero_fill_mode = 0;
    bit full24_mode = 0;
    bit full24_decode_mode = 0;
    bit decode_mode_active = 0;
    logic [31:0] zero_backed_base = 0, zero_backed_bytes = 0;
    logic [31:0] decode_layer_out_base = 0;
    bit flash_tile_bases = 0;
    logic [31:0] base_act_value, base_scale_value, base_out_value;
    string image_path, ucode_prefix, descriptor_path, package_dir;
    integer package_layers = 24, active_layer = 0, layer_stage = 0;
    integer stage_count = 3, watchdog = 300000000, stage_index;
    integer full24_start_stage = 0;
    longint partial_cycle_limit = 0;
    bit dump_stage0_checkpoint = 0;
    string resume_checkpoint = "";
    string l1_resume_checkpoint = "";
    string activation_resume_checkpoint = "";
    string kv_resume_checkpoint = "";
    string prefill_kv_source_checkpoint = "";
    bit checkpoint_each_stage = 0;
    longint cycles = 0, stage_start, total_execution = 0;
    longint prefill_execution_cycles = 0, decode_execution_cycles = 0;
    longint array_enabled_cycles = 0, gemm_commands = 0, useful_macs = 0;
    longint array_wall_cycles = 0, array_first_cycle = -1, array_last_cycle = -1;
    longint fetch_empty_cycles = 0, decode_wait_cycles = 0;
    longint scoreboard_wait_cycles = 0, barrier_wait_cycles = 0;
    longint token_loop_active_cycles = 0, exec_not_idle_cycles = 0;
    longint refill_inflight_cycles = 0, refill_systolic_overlap_cycles = 0;
    longint refill_same_page_misses = 0, refill_page_misses = 0;
    longint refill_requested_bytes = 0;
    longint fetch_wait_data_cycles = 0, dma_engine_busy_cycles = 0;
    longint tgld_busy_cycles = 0, softmax_busy_cycles = 0;
    longint accepted_instrs = 0, accepted_dma_rd = 0, accepted_gemm = 0;
    longint accepted_token_begin = 0, accepted_token_end = 0;
    longint resource_busy_cycles [0:7];
    longint blocked_by_resource_cycles [0:7];
    integer histogram [0:32];
    bit measuring = 0;
    integer completed = 0;
    integer report_fd, output_fd;
    logic [7:0] code_bytes [0:UCODE_DEPTH*4-1];
    logic [7:0] desc_bytes [0:2159];
    integer n_words;
    integer stable_pc_cycles = 0, last_pc = -1;
    integer debug_dma_count;
    integer debug_rope_count;
    integer debug_vld_count;
    integer fault_source_count;
    logic debug_dma;
    bit trace_l1_writes = 0;
    bit trace_route_skips = 0;
    wire [511:0] wrf_read_vector = {
      u_dut.wrf_rd_data15, u_dut.wrf_rd_data14,
      u_dut.wrf_rd_data13, u_dut.wrf_rd_data12,
      u_dut.wrf_rd_data11, u_dut.wrf_rd_data10,
      u_dut.wrf_rd_data9,  u_dut.wrf_rd_data8,
      u_dut.wrf_rd_data7,  u_dut.wrf_rd_data6,
      u_dut.wrf_rd_data5,  u_dut.wrf_rd_data4,
      u_dut.wrf_rd_data3,  u_dut.wrf_rd_data2,
      u_dut.wrf_rd_data1,  u_dut.wrf_rd_data0
    };
    logic [17:0] last_dma_wr_pc = '0;
    logic [31:0] last_dma_wr_instr = '0;

    axi_fake_ddr #(
      .READ_LATENCY(FAKE_DDR_READ_LATENCY_CYCLES),
      .BEAT_INTERVAL(FAKE_DDR_BEAT_INTERVAL_CYCLES)
    ) memory (
      .clk(clk), .rst_n(rst_n),
      .arid(m_axi_arid), .araddr(m_axi_araddr), .arlen(m_axi_arlen),
      .arsize(m_axi_arsize), .arburst(m_axi_arburst), .arvalid(m_axi_arvalid), .arready(m_axi_arready),
      .rid(m_axi_rid), .rdata(m_axi_rdata), .rresp(m_axi_rresp),
      .rlast(m_axi_rlast), .rvalid(m_axi_rvalid), .rready(m_axi_rready),
      .awid(m_axi_awid), .awaddr(m_axi_awaddr), .awlen(m_axi_awlen),
      .awsize(m_axi_awsize), .awburst(m_axi_awburst), .awvalid(m_axi_awvalid), .awready(m_axi_awready),
      .wdata(m_axi_wdata), .wstrb(m_axi_wstrb), .wlast(m_axi_wlast),
      .wvalid(m_axi_wvalid), .wready(m_axi_wready), .bid(m_axi_bid),
      .bresp(m_axi_bresp), .bvalid(m_axi_bvalid), .bready(m_axi_bready),
      .debug_pc(u_dut.u_fetch.pc_q), .debug_instr(u_dut.lc_dec_instr),
      .debug_writer_pc(last_dma_wr_pc),
      .debug_writer_instr(last_dma_wr_instr),
      .debug_stage(stage_index), .debug_cycle(cycles),
      .refill_request(refill_request), .refill_address(refill_address),
      .refill_bytes(refill_bytes), .refill_complete(refill_complete)
    );

    // Prepare the backing bytes in a TB-side staging buffer, then commit them
    // to the bounded FakeDDR cache only after the modeled burst-transfer time.
    // This keeps the old cache contents intact during refill and lets the DUT
    // continue computing from data already captured in its local buffers. The
    // refill task is asynchronous, so the DUT clock continues throughout.
    task automatic service_fake_ddr_refill(
        input logic [31:0] request_address,
        input integer request_bytes);
      if (request_bytes < 1 || request_bytes > 4096 || request_bytes % 16 != 0)
        $fatal(1, "FakeDDR invalid refill size=%0d", request_bytes);
      if (full24_mode && decode_mode_active &&
          {1'b0,request_address} >= {1'b0,decode_layer_out_base} &&
          {1'b0,request_address} + request_bytes <=
              {1'b0,decode_layer_out_base} + SEQ_LEN*4096) begin
        for (int b=0;b<request_bytes;b++)
          refill_buffer[b] = prefill_kv_store[
              (active_layer*2*SEQ_LEN*4096) +
              int'(request_address-decode_layer_out_base) + b];
      end else if (full24_mode && decode_mode_active &&
          {1'b0,request_address} >= {1'b0,decode_layer_out_base} + 32'h00040000 &&
          {1'b0,request_address} + request_bytes <=
              {1'b0,decode_layer_out_base} + 32'h00040000 + SEQ_LEN*4096) begin
        for (int b=0;b<request_bytes;b++)
          refill_buffer[b] = prefill_kv_store[
              (active_layer*2*SEQ_LEN*4096) + SEQ_LEN*4096 +
              int'(request_address-decode_layer_out_base-32'h00040000) + b];
      end else if (full24_mode &&
          {1'b0,request_address} >= {1'b0,base_act_value} &&
          {1'b0,request_address} + request_bytes <=
              {1'b0,base_act_value} + SEQ_LEN*4096) begin
        for (int b = 0; b < request_bytes; b++)
          refill_buffer[b] = activation_store[
              int'(request_address - base_act_value) + b];
      end else if (zero_fill_mode ||
          (zero_backed_bytes != 0 &&
           {1'b0,request_address} >= {1'b0,zero_backed_base} &&
           {1'b0,request_address} + request_bytes <=
               {1'b0,zero_backed_base} + zero_backed_bytes)) begin
        for (int b = 0; b < request_bytes; b++) refill_buffer[b] = 8'h00;
      end else begin
        if (!image_fd || {1'b0, request_address} + request_bytes > image_size)
          $fatal(1, "FakeDDR refill outside backing image addr=%h bytes=%0d", request_address, request_bytes);
        if ($fseek(image_fd, request_address, 0) != 0) $fatal(1, "FakeDDR seek failed");
        refill_rc = $fread(refill_buffer, image_fd, 0, request_bytes);
        if (refill_rc != request_bytes) $fatal(1, "FakeDDR short image refill");
      end

      for (int beat = 0; beat < request_bytes / 16; beat++) begin
        @(negedge clk);
      end
      // Atomic cache update: do not expose a partially refilled burst or
      // destroy the previous cached data while the new burst is in flight.
      memory.fill_block(refill_buffer, request_bytes);
      refill_complete = 1'b1;
      @(posedge clk);
      @(negedge clk);
      refill_complete = 1'b0;
      refill_busy = 1'b0;
    endtask

    always @(negedge clk) begin : refill_dispatch
      bit page_resident;
      if (!rst_n) begin
        refill_complete = 0;
        refill_busy = 0;
        refill_same_page_misses = 0;
        refill_page_misses = 0;
        refill_requested_bytes = 0;
      end else if (refill_request && !refill_busy) begin
        page_resident = 0;
        for (int bank = 0; bank < 2; bank++)
          if (memory.cache_page_valid[bank] &&
              memory.cache_page_base[bank] == {refill_address[31:12],12'b0})
            page_resident = 1;
        if (page_resident) refill_same_page_misses++;
        else refill_page_misses++;
        refill_requested_bytes += refill_bytes;
        refill_busy = 1;
        fork
          service_fake_ddr_refill(refill_address, refill_bytes);
        join_none
      end
    end
    always @(posedge clk) begin
      cycles <= cycles+1;
      if (rst_n && u_dut.wrf_wr_en && $isunknown({
          u_dut.wrf_wr_data0, u_dut.wrf_wr_data1,
          u_dut.wrf_wr_data2, u_dut.wrf_wr_data3,
          u_dut.wrf_wr_data4, u_dut.wrf_wr_data5,
          u_dut.wrf_wr_data6, u_dut.wrf_wr_data7})) begin
        $display("FAKE_DDR_UNKNOWN_WRF_WRITE cycle=%0d pc=%0d instr=%08h addr=%0d bank=%b data=%08h_%08h_%08h_%08h_%08h_%08h_%08h_%08h",
                 cycles, u_dut.u_fetch.pc_q, u_dut.lc_dec_instr,
                 u_dut.wrf_wr_addr, u_dut.wrf_bank_sel,
                 u_dut.wrf_wr_data7, u_dut.wrf_wr_data6,
                 u_dut.wrf_wr_data5, u_dut.wrf_wr_data4,
                 u_dut.wrf_wr_data3, u_dut.wrf_wr_data2,
                 u_dut.wrf_wr_data1, u_dut.wrf_wr_data0);
        $display("FAKE_DDR_UNKNOWN_WRF_SOURCE gld_state=%0d gld_busy=%b compact=%b prec=%b compact_k=%0d l1base=%h l1cnt=%0d page=%0d rd_en=%b rd_addr=%h rd_valid=%b rd_data=%h saved_l1=%h fast=%b commit=%h dma_valid=%b dma_addr=%h dma_len=%0d",
                 u_dut.u_gld_ctrl.state,u_dut.gld_busy,
                 u_dut.u_gld_ctrl.r_compact,u_dut.u_gld_ctrl.r_prec,
                 u_dut.u_gld_ctrl.r_compact_k,u_dut.u_gld_ctrl.r_l1_base,
                 u_dut.u_gld_ctrl.r_l1_cnt,u_dut.u_gld_ctrl.r_l1_page,
                 u_dut.l1_rd_en,u_dut.l1_rd_addr,u_dut.l1_rd_valid,
                 u_dut.l1_rd_data,u_dut.u_gld_ctrl.r_l1_buf,
                 u_dut.u_gld_ctrl.w_fast_commit,u_dut.u_gld_ctrl.w_commit_data,
                 u_dut.dma_rd_cmd_valid,u_dut.dma_rd_int_cmd_addr,
                 u_dut.dma_rd_int_cmd_len);
        $fatal(1, "FakeDDR unknown WRF write addr=%0d", u_dut.wrf_wr_addr);
      end
      if (rst_n && u_dut.arf_wr_en && $isunknown({
          u_dut.arf_wr_data0, u_dut.arf_wr_data1,
          u_dut.arf_wr_data2, u_dut.arf_wr_data3,
          u_dut.arf_wr_data4, u_dut.arf_wr_data5,
          u_dut.arf_wr_data6, u_dut.arf_wr_data7,
          u_dut.arf_wr_data8, u_dut.arf_wr_data9,
          u_dut.arf_wr_data10, u_dut.arf_wr_data11,
          u_dut.arf_wr_data12, u_dut.arf_wr_data13,
          u_dut.arf_wr_data14, u_dut.arf_wr_data15,
          u_dut.arf_wr_data16, u_dut.arf_wr_data17,
          u_dut.arf_wr_data18, u_dut.arf_wr_data19,
          u_dut.arf_wr_data20, u_dut.arf_wr_data21,
          u_dut.arf_wr_data22, u_dut.arf_wr_data23,
          u_dut.arf_wr_data24, u_dut.arf_wr_data25,
          u_dut.arf_wr_data26, u_dut.arf_wr_data27,
          u_dut.arf_wr_data28, u_dut.arf_wr_data29,
          u_dut.arf_wr_data30, u_dut.arf_wr_data31})) begin
        $display("FAKE_DDR_UNKNOWN_ARF_WRITE cycle=%0d pc=%0d instr=%08h addr=%0d bank=%b transpose=%b data=%08h_%08h_%08h_%08h",
                 cycles, u_dut.u_fetch.pc_q, u_dut.lc_dec_instr,
                 u_dut.arf_wr_addr, u_dut.arf_bank_sel,
                 u_dut.arf_wr_transpose,
                 u_dut.arf_wr_data3, u_dut.arf_wr_data2,
                 u_dut.arf_wr_data1, u_dut.arf_wr_data0);
        $display("FAKE_DDR_UNKNOWN_ARF_SOURCE state=%0d gld_busy=%b m=%0d k=%0d route_count=%0d group_row=%0d group_token=%0d token_start=%0d l1_base=%h l1_cnt=%0d transpose_reads=%0d at_k=%0d row_base=%0d col=%0d pending=%b response=%b l1_addr=%h l1_page=%0d l1_data=%h transpose_row0=%h",
                 u_dut.u_gld_ctrl.state,u_dut.gld_busy,
                 u_dut.u_gld_ctrl.r_a_m,u_dut.u_gld_ctrl.r_k_count,
                 u_dut.route_group_count,u_dut.group_route_row,
                 u_dut.route_group_token[u_dut.group_route_row],
                 u_dut.route_group_token_start,
                 u_dut.u_gld_ctrl.r_l1_base,u_dut.u_gld_ctrl.r_l1_cnt,
                 u_dut.u_gld_ctrl.a_transpose_reads,u_dut.u_gld_ctrl.r_at_k,
                 u_dut.u_gld_ctrl.r_at_row_base,u_dut.u_gld_ctrl.r_at_col,
                 u_dut.u_gld_ctrl.r_at_rd_pending,u_dut.l1_rd_valid,
                 u_dut.gld_l1_rd_addr,u_dut.gld_l1_rd_page,u_dut.l1_rd_data,
                 u_dut.u_gld_ctrl.a_transpose_buf[0][u_dut.u_gld_ctrl.r_at_k]);
        $fatal(1, "FakeDDR unknown ARF write addr=%0d", u_dut.arf_wr_addr);
      end
      if (trace_route_skips && rst_n && u_dut.lc_source_skip_valid &&
          u_dut.lc_dec_instr == 32'h0ed40005)
        $display("FAKE_DDR_SKIP_ROW4 cycle=%0d pc=%0d route_count=%0d group_row=%0d token_start=%0d skip_words=%0d",
                 cycles,u_dut.u_fetch.pc_q,u_dut.route_group_count,
                 u_dut.group_route_row,u_dut.route_group_token_start,
                 u_dut.lc_source_skip_count);
      if (trace_l1_writes && rst_n && u_dut.l1_wr_en && u_dut.l1_wr_addr[12:11] == 2'd1 &&
          u_dut.l1_wr_addr[10:0] >= 11'h790 &&
          u_dut.l1_wr_addr[10:0] <= 11'h79f)
        $display("FAKE_DDR_L1_TARGET_WRITE cycle=%0d pc=%0d addr=%h data=%h",
                 cycles,u_dut.u_fetch.pc_q,u_dut.l1_wr_addr,u_dut.l1_wr_data);
      if (trace_l1_writes && rst_n && u_dut.l1_wr_en && u_dut.l1_wr_addr[12:11] == 2'd2 &&
          u_dut.l1_wr_addr[10:0] <= 11'h1ff)
        $display("FAKE_DDR_L1_PAGE2_TARGET_WRITE cycle=%0d pc=%0d addr=%h data=%h",
                 cycles,u_dut.u_fetch.pc_q,u_dut.l1_wr_addr,u_dut.l1_wr_data);
      if (trace_l1_writes && rst_n && u_dut.vst_cmd_valid && u_dut.vst_cmd_page == 2'd2 &&
          u_dut.vst_cmd_addr >= 16'h1e00 && u_dut.vst_cmd_addr <= 16'h1e40)
        $display("FAKE_DDR_VST_PAGE2_CMD cycle=%0d pc=%0d addr=%h token_stride=%b route_row_valid=%b group_page=%0d",
                 cycles,u_dut.u_fetch.pc_q,u_dut.vst_cmd_addr,
                 u_dut.vst_cmd_token_stride_mode,u_dut.group_route_row_valid,
                 u_dut.vst_cmd_page);
      if (rst_n && u_dut.wrf_rd_en) begin
        for (int rf_word = 0; rf_word < 16; rf_word++) begin
          if (rf_word < (u_dut.u_gemm_ctrl.r_prec == 2'b00
                         ? 16 : u_dut.u_gemm_ctrl.r_nwords) &&
              $isunknown(wrf_read_vector[rf_word*32 +: 32])) begin
            $display("FAKE_DDR_UNKNOWN_WRF_READ cycle=%0d pc=%0d instr=%08h addr=%0d bank=%b word=%0d nwords=%0d data=%08h",
                     cycles, u_dut.u_fetch.pc_q, u_dut.lc_dec_instr,
                     u_dut.wrf_rd_addr, u_dut.wrf_rd_bank_sel, rf_word,
                     u_dut.u_gemm_ctrl.r_nwords,
                     wrf_read_vector[rf_word*32 +: 32]);
            $fatal(1, "FakeDDR unknown active WRF read addr=%0d word=%0d",
                   u_dut.wrf_rd_addr, rf_word);
          end
        end
      end
      if (rst_n && u_dut.arf_rd_en) begin
        for (int act_row = 0; act_row < 32; act_row++) begin
          if ((u_dut.u_gemm_ctrl.r_act_batch
               ? act_row < u_dut.u_gemm_ctrl.r_M : act_row == 0) &&
              $isunknown(u_dut.arf_rd_data[act_row*16 +: 16])) begin
            $display("FAKE_DDR_UNKNOWN_ARF_READ cycle=%0d pc=%0d instr=%08h addr=%0d row=%0d M=%0d batch=%b data=%04h",
                     cycles, u_dut.u_fetch.pc_q, u_dut.lc_dec_instr,
                     u_dut.arf_rd_addr, act_row,
                     u_dut.u_gemm_ctrl.r_M,
                     u_dut.u_gemm_ctrl.r_act_batch,
                     u_dut.arf_rd_data[act_row*16 +: 16]);
            $fatal(1, "FakeDDR unknown active ARF read addr=%0d row=%0d",
                   u_dut.arf_rd_addr, act_row);
          end
        end
      end
      if (rst_n && u_dut.dq_in_valid && u_dut.dq_in_ready &&
          $isunknown({u_dut.dq_in_data, u_dut.dq_scale_data,
                      u_dut.dq_zero_data})) begin
        $display("FAKE_DDR_UNKNOWN_DQ_INPUT cycle=%0d pc=%0d instr=%08h gather_k=%0d nbits=%0d data=%h scale0=%h zero0=%h sc_state=%0d sc_ready=%b wr_bank=%b rd_bank=%b group_sel=%b szm_rd=%b/%0d/%b/%h mem576=%h mem577=%h mem578=%h mem579=%h loader=%0d exp=%0d left=%0d rsz0=%h rsz1=%h cache=%b",
                 cycles, u_dut.u_fetch.pc_q, u_dut.lc_dec_instr,
                 u_dut.u_gemm_ctrl.r_gather_k,
                 u_dut.gemm_dq_nbits_code, u_dut.dq_in_data,
                 u_dut.dq_scale_data[15:0],
                 u_dut.dq_zero_data[15:0],
                 u_dut.u_scale_rd_ctrl.state,
                 u_dut.dq_sz_ready_by_bank,
                 u_dut.wrf_bank_sel,
                 u_dut.u_scale_rd_ctrl.rd_bank_sel,
                 u_dut.gemm_dq_group_sel,
                 u_dut.szm_rd_en, u_dut.szm_rd_addr,
                 u_dut.szm_rd_valid, u_dut.szm_rd_data,
                 u_dut.u_scale_zero_mem.mem[576],
                 u_dut.u_scale_zero_mem.mem[577],
                 u_dut.u_scale_zero_mem.mem[578],
                 u_dut.u_scale_zero_mem.mem[579],
                 u_dut.u_scale_loader.state,
                 u_dut.u_scale_loader.exp_cnt,
                 u_dut.u_scale_loader.beats_left,
                 u_dut.u_scale_rd_ctrl.r_sz[0][15:0],
                 u_dut.u_scale_rd_ctrl.r_sz[1][15:0],
                 u_dut.u_scale_rd_ctrl.cache_valid);
        $fatal(1, "FakeDDR unknown dequant input/parameters");
      end
      if (rst_n && u_dut.dq_out_valid && u_dut.dq_out_ready &&
          $isunknown(u_dut.dq_out_data)) begin
        $display("FAKE_DDR_UNKNOWN_DQ_OUTPUT cycle=%0d pc=%0d instr=%08h commit_k=%0d data0=%h",
                 cycles, u_dut.u_fetch.pc_q, u_dut.lc_dec_instr,
                 u_dut.u_gemm_ctrl.r_commit_k,
                 u_dut.dq_out_data[15:0]);
        $fatal(1, "FakeDDR unknown dequant output");
      end
      if (rst_n && u_dut.u_gemm_ctrl.sa_compute_active &&
          $isunknown({u_dut.sa_a_col[0], u_dut.sa_b_row[0],
                      u_dut.sa_acc[0][0]})) begin
        $display("FAKE_DDR_UNKNOWN_SA cycle=%0d pc=%0d instr=%08h gemm_state=%0d prec=%b K=%0d gather/commit=%0d/%0d feed_k=%0d dtype_fp16=%b clear=%b en=%b a0=%h b0=%h wt00=%h dq_valid/ready=%b/%b dq0=%h acc00=%h",
                 cycles, u_dut.u_fetch.pc_q, u_dut.lc_dec_instr,
                 u_dut.u_gemm_ctrl.state,
                 u_dut.u_gemm_ctrl.r_prec, u_dut.u_gemm_ctrl.r_K,
                 u_dut.u_gemm_ctrl.r_gather_k,
                 u_dut.u_gemm_ctrl.r_commit_k,
                 u_dut.u_gemm_ctrl.r_feed_k,
                 1'b1,
                 u_dut.sa_clear, u_dut.sa_en,
                 u_dut.sa_a_col[0], u_dut.sa_b_row[0],
                 u_dut.u_gemm_ctrl.wt_buf[0][0],
                 u_dut.dq_out_valid, u_dut.dq_out_ready,
                 u_dut.dq_out_data[15:0],
                 u_dut.sa_acc[0][0]);
        $fatal(1, "FakeDDR unknown systolic input/accumulator");
      end
      if (rst_n && u_dut.acc_wr_en && $isunknown({
          u_dut.acc_wr_data0, u_dut.acc_wr_data1,
          u_dut.acc_wr_data2, u_dut.acc_wr_data3,
          u_dut.acc_wr_data4, u_dut.acc_wr_data5,
          u_dut.acc_wr_data6, u_dut.acc_wr_data7,
          u_dut.acc_wr_data8, u_dut.acc_wr_data9,
          u_dut.acc_wr_data10, u_dut.acc_wr_data11,
          u_dut.acc_wr_data12, u_dut.acc_wr_data13,
          u_dut.acc_wr_data14, u_dut.acc_wr_data15,
          u_dut.acc_wr_data16, u_dut.acc_wr_data17,
          u_dut.acc_wr_data18, u_dut.acc_wr_data19,
          u_dut.acc_wr_data20, u_dut.acc_wr_data21,
          u_dut.acc_wr_data22, u_dut.acc_wr_data23,
          u_dut.acc_wr_data24, u_dut.acc_wr_data25,
          u_dut.acc_wr_data26, u_dut.acc_wr_data27,
          u_dut.acc_wr_data28, u_dut.acc_wr_data29,
          u_dut.acc_wr_data30, u_dut.acc_wr_data31})) begin
        $display("FAKE_DDR_UNKNOWN_ACC_WRITE cycle=%0d pc=%0d instr=%08h row=%0d bank=%b gemm_cmd_acc=%b gemm_cmd_k=%0d data0=%08h data1=%08h data2=%08h data3=%08h data4=%08h data5=%08h data6=%08h data7=%08h",
                 cycles, u_dut.u_fetch.pc_q, u_dut.lc_dec_instr,
                 u_dut.acc_wr_addr, u_dut.acc_bank_sel,
                 u_dut.gemm_cmd_acc, u_dut.gemm_cmd_k,
                 u_dut.acc_wr_data0, u_dut.acc_wr_data1,
                 u_dut.acc_wr_data2, u_dut.acc_wr_data3,
                 u_dut.acc_wr_data4, u_dut.acc_wr_data5,
                 u_dut.acc_wr_data6, u_dut.acc_wr_data7);
        $fatal(1, "FakeDDR unknown accumulator write row=%0d",
               u_dut.acc_wr_addr);
      end
      if (rst_n && u_dut.prf_wr_en && $isunknown(u_dut.prf_wr_data)) begin
        $display("FAKE_DDR_UNKNOWN_PRF_WRITE cycle=%0d pc=%0d instr=%08h addr=%0d data=%064h vv=%b sm=%b rms=%b rope=%b silu=%b vec=%b ac2v=%b",
                 cycles, u_dut.u_fetch.pc_q, u_dut.lc_dec_instr,
                 u_dut.prf_wr_addr, u_dut.prf_wr_data,
                 u_dut.vv_prf_wr_en, u_dut.sm_rf_wr_en,
                 u_dut.rn_rf_wr_en, u_dut.rp_rf_wr_en,
                 u_dut.si_rf_wr_en, u_dut.ve_rf_wr_en,
                 u_dut.ac2v_prf_wr_en);
        $fatal(1, "FakeDDR unknown PRF write addr=%0d", u_dut.prf_wr_addr);
      end
      if (rst_n && u_dut.l1_wr_en && $isunknown(u_dut.l1_wr_data)) begin
        $display("FAKE_DDR_UNKNOWN_L1_WRITE cycle=%0d pc=%0d instr=%08h addr=%04h vv_en=%b vv_addr=%04h vv_page=%0d vv_data=%064h dma_en=%b dma_addr=%04h dma_data=%064h",
                 cycles, u_dut.u_fetch.pc_q, u_dut.lc_dec_instr,
                 u_dut.l1_wr_addr, u_dut.vv_l1_wr_en,
                 u_dut.vv_l1_wr_addr, u_dut.vv_l1_wr_page,
                 u_dut.vv_l1_wr_data, u_dut.dma_rx_wr_en,
                 u_dut.dma_rx_wr_addr, u_dut.dma_rx_wr_data);
        $fatal(1, "FakeDDR unknown L1 write addr=%04h", u_dut.l1_wr_addr);
      end
      if (measuring && u_dut.lc_dec_valid && u_dut.fetch_instr_ready &&
          u_dut.lc_dec_instr[31:28] == 4'hD) begin
        last_dma_wr_pc <= u_dut.u_fetch.pc_q;
        last_dma_wr_instr <= u_dut.lc_dec_instr;
      end
      if (measuring && ENABLE_DETAILED_PROFILE) begin
        if (u_dut.lc_dec_valid && u_dut.fetch_instr_ready) begin
          accepted_instrs++;
          if (u_dut.lc_dec_instr[31:28] == 4'hC) accepted_dma_rd++;
          if (u_dut.lc_dec_instr[31:28] == 4'h5) accepted_gemm++;
          if (u_dut.lc_dec_instr[31:28] == 4'h0 &&
              u_dut.lc_dec_instr[15:12] == 4'h8)
            accepted_token_begin++;
          if (u_dut.lc_dec_instr[31:28] == 4'h0 &&
              u_dut.lc_dec_instr[15:12] == 4'h9)
            accepted_token_end++;
        end
        if (!u_dut.lc_dec_valid) fetch_empty_cycles++;
        if (u_dut.loop_active) token_loop_active_cycles++;
        if (!u_dut.exec_all_idle) exec_not_idle_cycles++;
        if (u_dut.u_fetch.state_q == 3'd2) fetch_wait_data_cycles++;
        if (u_dut.dma_rd_busy) dma_engine_busy_cycles++;
        if (u_dut.tgld_busy) tgld_busy_cycles++;
        if (u_dut.sm_busy) softmax_busy_cycles++;
        if (u_dut.lc_dec_valid && !u_dut.fetch_instr_ready) begin
          decode_wait_cycles++;
          if (!u_dut.u_decode.can_issue) scoreboard_wait_cycles++;
          if (u_dut.u_decode.barrier_active) barrier_wait_cycles++;
          for (int resource = 0; resource < 8; resource++)
            if (u_dut.res_busy[resource])
              blocked_by_resource_cycles[resource]++;
        end
        for (int resource = 0; resource < 8; resource++)
          if (u_dut.res_busy[resource]) resource_busy_cycles[resource]++;
        if (u_dut.sa_en) begin
          array_enabled_cycles++;
          if (array_first_cycle < 0) array_first_cycle = cycles;
          array_last_cycle = cycles;
        end
        if (u_dut.gemm_done && array_first_cycle >= 0) begin
          array_wall_cycles += array_last_cycle-array_first_cycle+1;
          array_first_cycle = -1; array_last_cycle = -1;
        end
        if (u_dut.gemm_cmd_valid && u_dut.u_gemm_ctrl.cmd_ready) begin
          gemm_commands++;
          useful_macs += int'(u_dut.cfg_m_active)*32*int'(u_dut.gemm_cmd_k);
          histogram[u_dut.cfg_m_active]++;
        end
      end
      if (measuring) begin
        if (last_pc == u_dut.u_fetch.pc_q) stable_pc_cycles++;
        else begin last_pc = u_dut.u_fetch.pc_q; stable_pc_cycles = 0; end
        if (stable_pc_cycles == 100000)
          $fatal(1, "No source PC progress for 100000 cycles at pc=%0d",
                 u_dut.u_fetch.pc_q);
        if (rst_n && fault_source_count < 8 &&
            (u_dut.route_error || u_dut.expert_gemm_weight_mismatch ||
             u_dut.gemm_k_range_fault || u_dut.scale_start_overrun ||
             u_dut.scale_cmd_current_expert_fault ||
             (u_dut.group_weight_cmd_valid && !u_dut.route_group_desc_active) ||
             (u_dut.rweight_cmd_valid &&
              (u_dut.rweight_cmd_slot < 1 || u_dut.rweight_cmd_slot > 4)) ||
             (u_dut.cfg_k_write_valid &&
              u_dut.cfg_k_write_value != 12'd32 &&
              u_dut.cfg_k_write_value != 12'd64 &&
              u_dut.cfg_k_write_value != 12'd128 &&
              u_dut.cfg_k_write_value != 12'd256) ||
             (u_dut.cfg_m_write_valid && u_dut.cfg_m_write_value > 6'd32) ||
             (u_dut.cfg_m_write_valid && u_dut.cfg_m_write_value == 0 &&
              !u_dut.route_group_desc_active) ||
             (u_dut.dma_rd_cmd_valid && u_dut.dma_rd_cmd_group_gather &&
              u_dut.rg_invalid_start) ||
             (u_dut.dma_rd_cmd_valid && u_dut.expert_dma_fault) ||
             (m_axi_rvalid && m_axi_rready && m_axi_rresp != 0) ||
             (m_axi_bvalid && m_axi_bready && m_axi_bresp != 0))) begin
          $display("FAKE_DDR_FAULT_SOURCE stage=%0d pc=%0d instr=%08h route=%b weight_mismatch=%b gemm_k=%b scale_overrun=%b scale_expert=%b group_desc=%b rweight=%b/%0d cfg_k=%b/%0d cfg_m=%b/%0d group_start=%b expert_dma=%b dma_sel=%0d dma_addr=%h axi_r=%b/%b axi_b=%b/%b loaded_w=%b/%0d/%0d/%0d/%0d desc_b=%0d/%h gld=%0d/%0d/%0d desc_c=%0d/%h gemm=%0d/%0d/%0d",
                   stage_index,u_dut.u_fetch.pc_q,u_dut.lc_dec_instr,
                   u_dut.route_error,u_dut.expert_gemm_weight_mismatch,
                   u_dut.gemm_k_range_fault,u_dut.scale_start_overrun,
                   u_dut.scale_cmd_current_expert_fault,
                   u_dut.group_weight_cmd_valid && !u_dut.route_group_desc_active,
                   u_dut.rweight_cmd_valid,u_dut.rweight_cmd_slot,
                   u_dut.cfg_k_write_valid,u_dut.cfg_k_write_value,
                   u_dut.cfg_m_write_valid,u_dut.cfg_m_write_value,
                   u_dut.dma_rd_cmd_valid && u_dut.dma_rd_cmd_group_gather &&
                     u_dut.rg_invalid_start,
                   u_dut.dma_rd_cmd_valid && u_dut.expert_dma_fault,
                   u_dut.dma_rd_cmd_expert_sel,u_dut.dma_rd_cmd_addr,
                   m_axi_rvalid,m_axi_rresp,m_axi_bvalid,m_axi_bresp,
                   u_dut.loaded_expert_w_valid,u_dut.loaded_expert_w_slot,
                   u_dut.loaded_expert_w_id,u_dut.loaded_expert_w_proj,
                   u_dut.loaded_expert_w_prec,u_dut.ed_rd_expert_b,
                   u_dut.ed_desc_b_attr,u_dut.gld_w_cmd_expert_sel,
                   u_dut.gld_w_cmd_proj,u_dut.gld_prec_eff,
                   u_dut.ed_rd_expert_c,u_dut.ed_desc_c_attr,
                   u_dut.gemm_cmd_expert_sel,u_dut.gemm_cmd_proj,
                   u_dut.gemm_prec_eff);
          fault_source_count++;
        end
        if (u_dut.execution_fault)
          $fatal(1, "NPU execution fault pc=%0d", u_dut.u_fetch.pc);
      end
    end

    top #(
        .ARRAY_DIM(32), .P_AXI_DATA_W(AXI_DATA_W),
        .P_AXI_ADDR_W(32), .P_AXI_ID_W(4), .P_UCODE_DEPTH(UCODE_DEPTH)
    ) u_dut (
        .clk(clk), .rst_n(rst_n),
        .s_axil_awaddr(s_axil_awaddr), .s_axil_awvalid(s_axil_awvalid), .s_axil_awready(s_axil_awready),
        .s_axil_wdata(s_axil_wdata), .s_axil_wstrb(s_axil_wstrb),
        .s_axil_wvalid(s_axil_wvalid), .s_axil_wready(s_axil_wready),
        .s_axil_bresp(s_axil_bresp), .s_axil_bvalid(s_axil_bvalid), .s_axil_bready(s_axil_bready),
        .s_axil_araddr(s_axil_araddr), .s_axil_arvalid(s_axil_arvalid), .s_axil_arready(s_axil_arready),
        .s_axil_rdata(s_axil_rdata), .s_axil_rresp(s_axil_rresp),
        .s_axil_rvalid(s_axil_rvalid), .s_axil_rready(s_axil_rready),
        .m_axi_arid(m_axi_arid), .m_axi_araddr(m_axi_araddr), .m_axi_arlen(m_axi_arlen),
        .m_axi_arsize(m_axi_arsize), .m_axi_arburst(m_axi_arburst),
        .m_axi_arvalid(m_axi_arvalid), .m_axi_arready(m_axi_arready),
        .m_axi_rid(m_axi_rid), .m_axi_rdata(m_axi_rdata),
        .m_axi_rresp(m_axi_rresp), .m_axi_rlast(m_axi_rlast),
        .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(m_axi_rready),
        .m_axi_awid(m_axi_awid), .m_axi_awaddr(m_axi_awaddr), .m_axi_awlen(m_axi_awlen),
        .m_axi_awsize(m_axi_awsize), .m_axi_awburst(m_axi_awburst),
        .m_axi_awvalid(m_axi_awvalid), .m_axi_awready(m_axi_awready),
        .m_axi_wdata(m_axi_wdata), .m_axi_wstrb(m_axi_wstrb),
        .m_axi_wlast(m_axi_wlast), .m_axi_wvalid(m_axi_wvalid), .m_axi_wready(m_axi_wready),
        .m_axi_bid(m_axi_bid), .m_axi_bresp(m_axi_bresp), .m_axi_bvalid(m_axi_bvalid), .m_axi_bready(m_axi_bready),
        .s_acc_flush_valid(), .s_acc_flush_addr(), .s_acc_flush_data(),
        .tb_ucode_wr_en(tb_ucode_wr_en), .tb_ucode_wr_addr(tb_ucode_wr_addr),
        .tb_ucode_wr_data(tb_ucode_wr_data)
    );

    task automatic axil_write(input logic [31:0] addr, input logic [31:0] data);
      bit aw_seen, w_seen;
      integer timeout;
      @(negedge clk);
      s_axil_awaddr=addr; s_axil_awvalid=1;
      s_axil_wdata=data; s_axil_wstrb=15; s_axil_wvalid=1;
      s_axil_bready=1; aw_seen=0; w_seen=0; timeout=0;
      while (!aw_seen || !w_seen) begin
        @(posedge clk);
        if (s_axil_awvalid && s_axil_awready) aw_seen=1;
        if (s_axil_wvalid && s_axil_wready) w_seen=1;
        @(negedge clk);
        if (aw_seen) s_axil_awvalid=0;
        if (w_seen) s_axil_wvalid=0;
        timeout=timeout+1;
        if (timeout > 1000) $fatal(1,"AXIL write timeout");
      end
      timeout=0;
      while (!s_axil_bvalid) begin
        @(negedge clk);
        timeout=timeout+1;
        if (timeout > 1000) $fatal(1,"AXIL response timeout");
      end
      @(posedge clk); @(negedge clk); s_axil_bready=0;
    endtask

    task automatic load_ucode(input string filename);
      integer fd, bytes_read, size;
      fd=$fopen(filename,"rb");
      if (!fd) $fatal(1,"Missing instruction file %s",filename);
      if ($fseek(fd,0,2)) $fatal(1,"Instruction seek failed");
      size=$ftell(fd);
      if (size<=0 || size%4 || size>UCODE_DEPTH*4) $fatal(1,"Bad instruction size %0d",size);
      if ($fseek(fd,0,0)) $fatal(1,"Instruction rewind failed");
      bytes_read=$fread(code_bytes,fd,0,size); $fclose(fd);
      if (bytes_read!=size) $fatal(1,"Short instruction read");
      n_words=size/4;
      for (int word=0; word<n_words; word++) begin
        @(negedge clk);
        tb_ucode_wr_en=1; tb_ucode_wr_addr=word;
        tb_ucode_wr_data={code_bytes[word*4+3],code_bytes[word*4+2],code_bytes[word*4+1],code_bytes[word*4]};
      end
      @(negedge clk); tb_ucode_wr_en=0;
      $display("FAKE_DDR_UCODE stage=%0d words=%0d",stage_index,n_words);
    endtask

    function automatic string package_layer_tag(input integer layer);
      if (layer<10) return $sformatf("0%0d",layer);
      return $sformatf("%0d",layer);
    endfunction

    task automatic load_expert_descriptors(input string filename);
      integer fd, bytes_read;
      logic [31:0] w0,w1,w2;
      fd=$fopen(filename,"rb");
      if (!fd) $fatal(1,"Missing descriptor table %s",filename);
      bytes_read=$fread(desc_bytes,fd); $fclose(fd);
      if (bytes_read!=2160) $fatal(1,"Bad descriptor table length %0d",bytes_read);
      for (int entry=0;entry<180;entry++) begin
        w0={desc_bytes[entry*12+3],desc_bytes[entry*12+2],desc_bytes[entry*12+1],desc_bytes[entry*12]};
        w1={desc_bytes[entry*12+7],desc_bytes[entry*12+6],desc_bytes[entry*12+5],desc_bytes[entry*12+4]};
        w2={desc_bytes[entry*12+11],desc_bytes[entry*12+10],desc_bytes[entry*12+9],desc_bytes[entry*12+8]};
        axil_write('h84,entry); axil_write('h88,w0); axil_write('h8c,w1); axil_write('h90,w2);
      end
      $display("FAKE_DDR_DESCRIPTORS layer=%0d entries=180",active_layer);
    endtask

    task automatic open_package_image(input integer layer);
      string filename;
      if (image_fd) $fclose(image_fd);
      filename=$sformatf("%s/layer_%s.bin",package_dir,package_layer_tag(layer));
      image_fd=$fopen(filename,"rb");
      if (!image_fd) $fatal(1,"Cannot open layer backing image %s",filename);
      if ($fseek(image_fd,0,2)) $fatal(1,"Layer image size seek failed");
      image_size=$ftell(image_fd);
      if (image_size<1) $fatal(1,"Empty layer backing image %s",filename);
      image_path=filename;
      $display("FAKE_DDR_LAYER_IMAGE layer=%0d bytes=%0d",layer,image_size);
    endtask

    task automatic capture_final_activation(input integer layer);
      integer token, group_index, byte_index;
      logic [31:0] output_address;
      logic [7:0] value;
      for (token=0;token<SEQ_LEN;token++) begin
        group_index=token/4;
        for (byte_index=0;byte_index<4096;byte_index++) begin
          output_address=base_out_value + (4+group_index)*65536 +
                         (token%4)*4096 + byte_index;
          value=memory.read_dirty_byte(output_address);
          if ($isunknown(value))
            $fatal(1,"Missing final activation layer=%0d token=%0d byte=%0d addr=%08h",
                   layer,token,byte_index,output_address);
          activation_store[token*4096+byte_index]=value;
        end
      end
      $display("FAKE_DDR_ACTIVATION_CHAIN layer=%0d bytes=%0d source_windows=4..11",
               layer,SEQ_LEN*4096);
    endtask

    task automatic capture_prefill_kv(input integer layer);
      integer projection, token, byte_index, source_window, destination;
      logic [31:0] source_address;
      logic [7:0] value;
      for (projection=0;projection<2;projection++) begin
        source_window = projection==0 ? 2 : 4;
        for (token=0;token<SEQ_LEN;token++) begin
          for (byte_index=0;byte_index<4096;byte_index++) begin
            source_address=base_out_value +
                (source_window + token/16)*65536 +
                (token%16)*4096 + byte_index;
            value=memory.read_dirty_byte(source_address);
            if ($isunknown(value))
              $fatal(1,"Missing prefill KV layer=%0d projection=%0d token=%0d byte=%0d addr=%08h",
                     layer,projection,token,byte_index,source_address);
            destination=(layer*2+projection)*SEQ_LEN*4096 +
                        token*4096 + byte_index;
            prefill_kv_store[destination]=value;
          end
        end
      end
      $display("FAKE_DDR_PREFILL_KV_CAPTURE layer=%0d tokens=%0d K_windows=2..3 V_windows=4..5 raw_K=1",
               layer,SEQ_LEN);
    endtask

    task automatic capture_decode_activation(input integer layer,
                                             input logic [31:0] output_base);
      logic [31:0] output_address;
      logic [7:0] value;
      for (int byte_index=0;byte_index<4096;byte_index++) begin
        output_address=output_base + 10*65536 + byte_index;
        value=memory.read_dirty_byte(output_address);
        if ($isunknown(value))
          $fatal(1,"Missing decode activation layer=%0d byte=%0d addr=%08h",
                 layer,byte_index,output_address);
        activation_store[(SEQ_LEN-1)*4096+byte_index]=value;
      end
      $display("FAKE_DDR_DECODE_CHAIN layer=%0d token=%0d bytes=4096",
               layer,SEQ_LEN-1);
    endtask

    task automatic dump_l1_checkpoint(input string path);
      integer l1_fd;
      logic [255:0] line_data;
      l1_fd=$fopen(path,"w");
      if (!l1_fd) $fatal(1,"Cannot create L1 checkpoint %s",path);
      for (int line=0;line<4608;line++) begin
        line_data='0;
        for (int lane=0;lane<8;lane++)
          line_data[lane*32 +: 32] =
            u_dut.u_l1_cache.bank_mem[((line & 3) << 3) + lane][line >> 2];
        $fdisplay(l1_fd,"%064h",line_data);
      end
      $fclose(l1_fd);
      $display("FAKE_DDR_L1_CHECKPOINT_DUMP file=%s lines=4608",path);
    endtask

    task automatic load_l1_checkpoint(input string path);
      integer l1_fd, scan_rc;
      logic [255:0] line_data;
      l1_fd=$fopen(path,"r");
      if (!l1_fd) $fatal(1,"Cannot open L1 checkpoint %s",path);
      for (int line=0;line<4608;line++) begin
        scan_rc=$fscanf(l1_fd,"%h",line_data);
        if (scan_rc != 1)
          $fatal(1,"Short/corrupt L1 checkpoint %s line=%0d",path,line);
        for (int lane=0;lane<8;lane++)
          u_dut.u_l1_cache.bank_mem[((line & 3) << 3) + lane][line >> 2] =
            line_data[lane*32 +: 32];
      end
      $fclose(l1_fd);
      $display("FAKE_DDR_L1_CHECKPOINT_LOAD file=%s lines=4608",path);
    endtask

    task automatic dump_activation_checkpoint(input string path);
      integer fd;
      fd=$fopen(path,"wb");
      if (!fd) $fatal(1,"Cannot create activation checkpoint %s",path);
      for (int i=0;i<SEQ_LEN*4096;i+=8)
        $fwrite(fd,"%c%c%c%c%c%c%c%c",activation_store[i],
                activation_store[i+1],activation_store[i+2],activation_store[i+3],
                activation_store[i+4],activation_store[i+5],activation_store[i+6],
                activation_store[i+7]);
      $fclose(fd);
      $display("FAKE_DDR_ACTIVATION_CHECKPOINT_DUMP file=%s bytes=%0d",path,SEQ_LEN*4096);
    endtask

    task automatic load_activation_checkpoint(input string path);
      integer fd, bytes_read;
      fd=$fopen(path,"rb");
      if (!fd) $fatal(1,"Cannot open activation checkpoint %s",path);
      bytes_read=$fread(activation_store,fd);
      $fclose(fd);
      if (bytes_read!=SEQ_LEN*4096)
        $fatal(1,"Short/corrupt activation checkpoint %s bytes=%0d",path,bytes_read);
      $display("FAKE_DDR_ACTIVATION_CHECKPOINT_LOAD file=%s bytes=%0d",path,bytes_read);
    endtask

    task automatic dump_kv_checkpoint(input string path);
      integer fd;
      byte unsigned b0,b1,b2,b3,b4,b5,b6,b7;
      fd=$fopen(path,"wb");
      if (!fd) $fatal(1,"Cannot create KV checkpoint %s",path);
      for (int i=0;i<24*2*SEQ_LEN*4096;i+=8) begin
        b0=$isunknown(prefill_kv_store[i])   ? 8'h00 : prefill_kv_store[i];
        b1=$isunknown(prefill_kv_store[i+1]) ? 8'h00 : prefill_kv_store[i+1];
        b2=$isunknown(prefill_kv_store[i+2]) ? 8'h00 : prefill_kv_store[i+2];
        b3=$isunknown(prefill_kv_store[i+3]) ? 8'h00 : prefill_kv_store[i+3];
        b4=$isunknown(prefill_kv_store[i+4]) ? 8'h00 : prefill_kv_store[i+4];
        b5=$isunknown(prefill_kv_store[i+5]) ? 8'h00 : prefill_kv_store[i+5];
        b6=$isunknown(prefill_kv_store[i+6]) ? 8'h00 : prefill_kv_store[i+6];
        b7=$isunknown(prefill_kv_store[i+7]) ? 8'h00 : prefill_kv_store[i+7];
        $fwrite(fd,"%c%c%c%c%c%c%c%c",b0,b1,b2,b3,b4,b5,b6,b7);
      end
      $fclose(fd);
      $display("FAKE_DDR_KV_CHECKPOINT_DUMP file=%s bytes=%0d",path,24*2*SEQ_LEN*4096);
    endtask

    task automatic load_kv_checkpoint(input string path);
      integer fd, bytes_read;
      fd=$fopen(path,"rb");
      if (!fd) $fatal(1,"Cannot open KV checkpoint %s",path);
      bytes_read=$fread(prefill_kv_store,fd);
      $fclose(fd);
      if (bytes_read!=24*2*SEQ_LEN*4096)
        $fatal(1,"Short/corrupt KV checkpoint %s bytes=%0d",path,bytes_read);
      $display("FAKE_DDR_KV_CHECKPOINT_LOAD file=%s bytes=%0d",path,bytes_read);
    endtask

    initial begin : main
      integer fd, rc;
      integer output_chunk_stages, output_chunk_bytes, output_base_address;
      trace_l1_writes = $test$plusargs("FAKE_DDR_TRACE_L1");
      trace_route_skips = $test$plusargs("FAKE_DDR_TRACE_ROUTE");
      for (int row=0;row<=32;row++) histogram[row]=0;
      for (int resource=0;resource<8;resource++) begin
        resource_busy_cycles[resource]=0;
        blocked_by_resource_cycles[resource]=0;
      end
      if ($test$plusargs("ZERO_FILL")) begin
        // Synthetic attention timing cases may touch padded L1 lanes. Give
        // every unwritten SRAM lane a deterministic value for that mode.
        for (int bank=0;bank<32;bank++)
          for (int row=0;row<1152;row++)
            u_dut.u_l1_cache.bank_mem[bank][row] = 32'b0;
      end
      rst_n=0; s_axil_awaddr=0; s_axil_awvalid=0;
      s_axil_wdata=0; s_axil_wstrb=0; s_axil_wvalid=0; s_axil_bready=0;
      s_axil_araddr=0; s_axil_arvalid=0; s_axil_rready=0;
      tb_ucode_wr_en=0; tb_ucode_wr_addr=0; tb_ucode_wr_data=0;
      full24_mode=$test$plusargs("FULL24");
      full24_decode_mode=$test$plusargs("FULL24_DECODE");
      if (full24_decode_mode && !$test$plusargs("FULL24"))
        $fatal(1,"FULL24_DECODE requires FULL24 mode");
      if (full24_mode) begin
        if (!$value$plusargs("PACKAGE_DIR=%s",package_dir))
          $fatal(1,"PACKAGE_DIR required in FULL24 mode");
        if ($value$plusargs("FULL24_LAYERS=%d",package_layers)) begin end
        if ($value$plusargs("FULL24_START_STAGE=%d",full24_start_stage)) begin end
        dump_stage0_checkpoint=$test$plusargs("FULL24_DUMP_STAGE0");
        if ($value$plusargs("FAKE_DDR_RESUME_FROM=%s",resume_checkpoint)) begin end
        if ($value$plusargs("FAKE_DDR_L1_RESUME_FROM=%s",l1_resume_checkpoint)) begin end
        if ($value$plusargs("FAKE_DDR_ACT_RESUME_FROM=%s",activation_resume_checkpoint)) begin end
        if ($value$plusargs("FAKE_DDR_KV_RESUME_FROM=%s",kv_resume_checkpoint)) begin end
        if ($value$plusargs("FAKE_DDR_PREFILL_KV_SOURCE_FROM=%s",prefill_kv_source_checkpoint)) begin end
        checkpoint_each_stage=$test$plusargs("FULL24_CHECKPOINT_EACH_STAGE");
        if (package_layers<1 || package_layers>24)
          $fatal(1,"FULL24_LAYERS must be in [1,24]");
        active_layer=0;
        image_path=$sformatf("%s/layer_%s.bin",package_dir,package_layer_tag(active_layer));
        ucode_prefix=$sformatf("%s/ucode_layer_%s",package_dir,package_layer_tag(active_layer));
        descriptor_path=$sformatf("%s/expert_desc_%s.bin",package_dir,package_layer_tag(active_layer));
        stage_count=package_layers*(full24_decode_mode ? 4 : 3);
      end else begin
        if (!$value$plusargs("IMAGE=%s",image_path)) $fatal(1,"IMAGE required");
        if (!$value$plusargs("UCODE_PREFIX=%s",ucode_prefix)) $fatal(1,"UCODE_PREFIX required");
        if (!$value$plusargs("EDESC=%s",descriptor_path)) $fatal(1,"EDESC required");
      end
      if ($value$plusargs("STAGES=%d",stage_count)) begin end
      if (full24_mode) stage_count=package_layers*(full24_decode_mode ? 4 : 3);
      output_chunk_stages = 0;
      output_chunk_bytes = 0;
      debug_dma_count = 0;
      debug_rope_count = 0;
      debug_vld_count = 0;
      fault_source_count = 0;
      debug_dma = $test$plusargs("DEBUG_DMA");
      if ($value$plusargs("OUT_BASE_CHUNK_STAGES=%d",output_chunk_stages)) begin end
      if ($value$plusargs("OUT_BASE_CHUNK_BYTES=%d",output_chunk_bytes)) begin end
      if ($value$plusargs("WATCHDOG=%d",watchdog)) begin end
      if ($value$plusargs("PARTIAL_CYCLE_LIMIT=%d",partial_cycle_limit)) begin end
      zero_fill_mode = $test$plusargs("ZERO_FILL");
      flash_tile_bases = $test$plusargs("FLASH_TILE_BASES");
      base_act_value = 'h410000;
      base_scale_value = 'h410000;
      base_out_value = 'h80000;
      if ($value$plusargs("BASE_ACT=%h",base_act_value)) begin end
      if ($value$plusargs("BASE_SCALE=%h",base_scale_value)) begin end
      if ($value$plusargs("BASE_OUT=%h",base_out_value)) begin end
      if ($value$plusargs("ZERO_BACKED_BASE=%h",zero_backed_base)) begin end
      if ($value$plusargs("ZERO_BACKED_BYTES=%h",zero_backed_bytes)) begin end
      if (stage_count<1 || stage_count>128 || watchdog<1 ||
          full24_start_stage<0 || full24_start_stage>=stage_count)
        $fatal(1,"Invalid run bounds");
      if (full24_mode && full24_start_stage>0 && resume_checkpoint=="")
        $fatal(1,"FULL24_START_STAGE requires FAKE_DDR_RESUME_FROM checkpoint");
      if (prefill_kv_source_checkpoint!="" &&
          !(full24_mode && full24_decode_mode && package_layers==1 &&
            full24_start_stage==package_layers*3 && kv_resume_checkpoint==""))
        $fatal(1,"FAKE_DDR_PREFILL_KV_SOURCE_FROM is only valid for one-layer decode resume without a KV checkpoint");
      image_fd=$fopen(image_path,"rb");
      if (!image_fd) $fatal(1,"Cannot open image");
      if ($fseek(image_fd,0,2)) $fatal(1,"Image size seek failed");
      image_size=$ftell(image_fd);
      if (image_size<1) $fatal(1,"Empty backing image");
      if (full24_mode) begin
        if (image_size<32'h00030000) $fatal(1,"Layer-0 image lacks its input activation region");
        if ($fseek(image_fd,32'h00010000,0)) $fatal(1,"Initial activation seek failed");
        rc=$fread(activation_store,image_fd,0,SEQ_LEN*4096);
        if (rc!=SEQ_LEN*4096) $fatal(1,"Short initial activation image read %0d",rc);
        if (full24_decode_mode)
          for (int b=0;b<4096;b++)
            decode_seed[b]=activation_store[(SEQ_LEN-1)*4096+b];
        $display("FAKE_DDR_FULL24 layers=%0d activation_bytes=%0d resident_read_cache=%0d decode=%0d",
                 package_layers,SEQ_LEN*4096,2*4096,full24_decode_mode);
      end
      report_fd=$fopen("layer0_stage_cycles.csv","w");
      $fdisplay(report_fd,"stage,execution_cycles,execution_ms,read_bytes,write_bytes,refills,cache_hits,same_page_misses,page_misses,refill_bytes,dirty_beats");
      $display("FAKE_DDR_CLOCK period_ns=%0.6f freq_mhz=%0.3f read_latency_ns=%0.3f beat_interval_ns=%0.3f",
               CLK_PERIOD,1000.0/CLK_PERIOD,
               FAKE_DDR_READ_LATENCY_CYCLES*CLK_PERIOD,
               FAKE_DDR_BEAT_INTERVAL_CYCLES*CLK_PERIOD);
      $display("FAKE_DDR_IMAGE bytes=%0d resident_read_cache=%0d dirty_limit_bytes=4194304",image_size,2*4096);
      $display("FAKE_DDR_MAP act=%08h scale=%08h out=%08h zero_backed=[%08h,%08h)",
               base_act_value,base_scale_value,base_out_value,
               zero_backed_base,zero_backed_base+zero_backed_bytes);
      if (zero_fill_mode) $display("FAKE_DDR_ZERO_FILL enabled");
      repeat(10) @(negedge clk); rst_n=1;
      repeat(5) @(negedge clk);
      if (!full24_mode) load_expert_descriptors(descriptor_path);
      if (full24_mode && full24_start_stage>0) begin
        active_layer=(full24_start_stage<package_layers*3)
                   ? full24_start_stage/3
                   : full24_start_stage-package_layers*3;
        if (active_layer>0 || full24_start_stage>=package_layers*3)
          open_package_image(active_layer);
        descriptor_path=$sformatf("%s/expert_desc_%s.bin",package_dir,
                                  package_layer_tag(active_layer));
        load_expert_descriptors(descriptor_path);
        if (prefill_kv_source_checkpoint != "") begin
          memory.load_writes(prefill_kv_source_checkpoint);
          capture_prefill_kv(0);
        end
        memory.load_writes(resume_checkpoint);
        if (l1_resume_checkpoint != "")
          load_l1_checkpoint(l1_resume_checkpoint);
        if (activation_resume_checkpoint != "")
          load_activation_checkpoint(activation_resume_checkpoint);
        if (kv_resume_checkpoint != "")
          load_kv_checkpoint(kv_resume_checkpoint);
        else if (full24_decode_mode && full24_start_stage==1)
          // Legacy stage-0 checkpoints predate the per-stage snapshots and
          // were written before the TB copied K/V into its decode backing store.
          capture_prefill_kv(0);
      end
      for (stage_index=(full24_mode ? full24_start_stage : 0);
           stage_index<stage_count;stage_index++) begin
        if (full24_mode) begin
          if (stage_index<package_layers*3) begin
            active_layer=stage_index/3;
            layer_stage=stage_index%3;
            decode_mode_active=0;
          end else begin
            active_layer=stage_index-package_layers*3;
            layer_stage=3;
            decode_mode_active=1;
            decode_layer_out_base=base_out_value;
            if (stage_index==package_layers*3) begin
              for (int b=0;b<4096;b++)
                activation_store[(SEQ_LEN-1)*4096+b]=decode_seed[b];
              $display("FAKE_DDR_DECODE_SEED_RESTORE token=%0d bytes=4096",
                       SEQ_LEN-1);
            end
          end
          if (layer_stage==0 || layer_stage==3) begin
            if (stage_index!=0) open_package_image(active_layer);
            ucode_prefix=$sformatf("%s/ucode_layer_%s",package_dir,package_layer_tag(active_layer));
            descriptor_path=$sformatf("%s/expert_desc_%s.bin",package_dir,package_layer_tag(active_layer));
            load_expert_descriptors(descriptor_path);
          end
        end else begin
          layer_stage=stage_index;
        end
        if (full24_mode && layer_stage==3)
          load_ucode($sformatf("%s/ucode_decode_layer_%s_position_32.bin",
                               package_dir,package_layer_tag(active_layer)));
        else
          load_ucode(layer_stage < 10
            ? $sformatf("%s.stage0%0d.bin",ucode_prefix,layer_stage)
            : $sformatf("%s.stage%0d.bin",ucode_prefix,layer_stage));
        axil_write('h0c,n_words); axil_write('h10,0);
        axil_write('h14, flash_tile_bases ? 'h600000 : 0);
        output_base_address = base_out_value;
        if (full24_mode && layer_stage==3)
          output_base_address = decode_layer_out_base;
        else if (output_chunk_stages > 0)
          output_base_address = base_out_value +
              (stage_index / output_chunk_stages) * output_chunk_bytes;
        axil_write('h1c,output_base_address);
        axil_write('h10,base_act_value);
        axil_write('h48,base_scale_value);
        axil_write('h4c,1); axil_write('h50,32); axil_write('h54,32);
        axil_write('h78,0); axil_write('h7c,64);
        @(negedge clk);
        last_pc = -1;
        stable_pc_cycles = 0;
        measuring=1;
        stage_start=cycles;
        axil_write('h00,1);
        while (!u_dut.npu_done_latch) begin
          @(negedge clk);
          if (cycles-stage_start>watchdog) $fatal(1,"Stage END timeout pc=%0d",u_dut.u_fetch.pc);
          if ((cycles-stage_start)%100000==0)
            $display("FAKE_DDR_PROGRESS stage=%0d cycles=%0d pc=%0d refills=%0d cache_hits=%0d same_page_misses=%0d page_misses=%0d refill_bytes=%0d refill_busy=%b sa_en=%b overlap_cycles=%0d",
                     stage_index,cycles-stage_start,u_dut.u_fetch.pc_q,
                     memory.refill_count,memory.cache_hit_count,
                     refill_same_page_misses,refill_page_misses,
                     refill_requested_bytes,refill_busy,
                     u_dut.sa_en,refill_systolic_overlap_cycles);
          if (partial_cycle_limit>0 && cycles-stage_start>=partial_cycle_limit) begin
            measuring=0;
            $display("FAKE_DDR_PARTIAL_STAGE stage=%0d cycles=%0d limit=%0d",
                     stage_index,cycles-stage_start,partial_cycle_limit);
            $display("FAKE_DDR_PARTIAL_REFILL refills=%0d cache_hits=%0d refill_active_cycles=%0d systolic_overlap_cycles=%0d",
                     memory.refill_count,memory.cache_hit_count,
                     refill_inflight_cycles,refill_systolic_overlap_cycles);
            completed=1;
            $finish;
          end
        end
        while (!u_dut.exec_all_idle || memory.reading || memory.writing || m_axi_bvalid) begin
          @(negedge clk);
          if (cycles-stage_start>watchdog) $fatal(1,"Stage idle timeout");
        end
        measuring=0; total_execution+=cycles-stage_start;
        if (full24_mode && layer_stage==3)
          decode_execution_cycles+=cycles-stage_start;
        else if (full24_mode)
          prefill_execution_cycles+=cycles-stage_start;
        $display("FAKE_DDR_STAGE_DONE layer=%0d stage=%0d cycles=%0d refills=%0d cache_hits=%0d same_page_misses=%0d page_misses=%0d refill_bytes=%0d",
                 active_layer,layer_stage,cycles-stage_start,
                 memory.refill_count,memory.cache_hit_count,
                 refill_same_page_misses,refill_page_misses,
                 refill_requested_bytes);
        $fdisplay(report_fd,"%0d,%0d,%0.6f,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",stage_index,cycles-stage_start,
                  (cycles-stage_start)*CLK_PERIOD/1e6,memory.read_bytes,memory.write_bytes,
                  memory.refill_count,memory.cache_hit_count,
                  refill_same_page_misses,refill_page_misses,
                  refill_requested_bytes,memory.dirty_count);
        $fflush(report_fd);
        if (full24_mode && dump_stage0_checkpoint && stage_index==0) begin
          memory.dump_writes("fake_ddr_stage0_checkpoint.hex");
          dump_l1_checkpoint("fake_ddr_stage0_l1_checkpoint.hex");
        end
        if (full24_mode && layer_stage==0 && full24_decode_mode)
          capture_prefill_kv(active_layer);
        if (full24_mode && layer_stage==2 && active_layer+1<package_layers)
          capture_final_activation(active_layer);
        if (full24_mode && layer_stage==3)
          capture_decode_activation(active_layer,decode_layer_out_base);
        if (full24_mode && checkpoint_each_stage) begin
          string checkpoint_stem;
          checkpoint_stem=$sformatf("fake_ddr_resume_after_stage_%03d",stage_index);
          memory.dump_writes($sformatf("%s.ddr.hex",checkpoint_stem));
          dump_l1_checkpoint($sformatf("%s.l1.hex",checkpoint_stem));
          dump_activation_checkpoint($sformatf("%s.activation.bin",checkpoint_stem));
          dump_kv_checkpoint($sformatf("%s.kv.bin",checkpoint_stem));
          $display("FAKE_DDR_RESUME_CHECKPOINT stage_completed=%0d next_stage=%0d prefix=%s",
                   stage_index,stage_index+1,checkpoint_stem);
        end
      end
      memory.dump_writes("layer0_dirty_output.hex");
      $fclose(report_fd); $fclose(image_fd);
      $display("FAKE_DDR_DONE execution_cycles=%0d execution_ms=%0.6f gemms=%0d macs=%0d array_enabled=%0d",
               total_execution,total_execution*CLK_PERIOD/1e6,gemm_commands,useful_macs,array_enabled_cycles);
      $display("FAKE_DDR_CACHE_STATS refills=%0d hits=%0d resident_bytes=%0d",
               memory.refill_count,memory.cache_hit_count,2*4096);
      if (full24_mode)
        $display("FAKE_DDR_FULL24_PREFILL_TTFT layers=%0d tokens=%0d cycles=%0d milliseconds=%0.6f",
                 package_layers,SEQ_LEN,prefill_execution_cycles,
                 prefill_execution_cycles*CLK_PERIOD/1e6);
      else
        $display("FAKE_DDR_LAYER0_PREFILL_TTFT tokens=%0d cycles=%0d milliseconds=%0.6f",
                 SEQ_LEN,total_execution,total_execution*CLK_PERIOD/1e6);
      if (full24_mode && full24_decode_mode)
        $display("FAKE_DDR_FULL24_DECODE_TPOT layers=%0d prompt_tokens=%0d generated_tokens=1 cycles=%0d seconds=%0.9f kv_history=real_prefill_qkv",
                 package_layers,SEQ_LEN,decode_execution_cycles,
                 decode_execution_cycles*CLK_PERIOD/1e9);
      if (ENABLE_DETAILED_PROFILE) begin
        $display("FAKE_DDR_ARRAY wall_cycles=%0d pure_efficiency=%0.6f",array_wall_cycles,
                 real'(useful_macs)/(1024.0*array_wall_cycles));
        $display("FAKE_DDR_PROFILE fetch_empty=%0d decode_wait=%0d scoreboard_wait=%0d barrier_wait=%0d",
                 fetch_empty_cycles,decode_wait_cycles,scoreboard_wait_cycles,barrier_wait_cycles);
        $display("FAKE_DDR_PROFILE_DETAIL token_loop=%0d exec_not_idle=%0d fetch_wait_data=%0d dma_engine=%0d tgld=%0d softmax=%0d",
                 token_loop_active_cycles,exec_not_idle_cycles,fetch_wait_data_cycles,
                 dma_engine_busy_cycles,tgld_busy_cycles,softmax_busy_cycles);
        $display("FAKE_DDR_ISSUED instructions=%0d dma_rd=%0d gemm=%0d token_begin=%0d token_end=%0d",
                 accepted_instrs,accepted_dma_rd,accepted_gemm,
                 accepted_token_begin,accepted_token_end);
        for (int resource=0;resource<8;resource++)
          $display("FAKE_DDR_RESOURCE index=%0d busy=%0d blocked_issue=%0d",
                   resource,resource_busy_cycles[resource],
                   blocked_by_resource_cycles[resource]);
        for (int row=0;row<=32;row++) if (histogram[row])
          $display("FAKE_DDR_M_HIST M=%0d gemms=%0d",row,histogram[row]);
      end else begin
        $display("FAKE_DDR_PROFILE disabled; cycle/refill/fault/watchdog checks remain active");
      end
      $display("FAKE_DDR_REFILL_OVERLAP refill_active_cycles=%0d systolic_active_cycles=%0d",
               refill_inflight_cycles,refill_systolic_overlap_cycles);
      $display("FAKE_DDR_CONTROL_FLOW_PASS numerical_reference=NOT_CHECKED MIG_timing=NOT_MODELED");
      completed=1; $finish;
    end

    always @(posedge clk) begin : debug_dma_trace
      if (rst_n && measuring && refill_busy) begin
        refill_inflight_cycles++;
        if (u_dut.sa_en) refill_systolic_overlap_cycles++;
      end
      if (rst_n && debug_dma && u_dut.dma_rd_cmd_valid &&
          debug_dma_count < 24) begin
        $display("FAKE_DDR_DMA_TRACE pc=%0d instr=%h token_iter=%0d direct=%b wide=%b base_sel=%0d base_hi=%b addr=%h physical=%h len=%0d",
                 u_dut.u_fetch.pc, u_dut.lc_dec_instr,
                 u_dut.token_loop_iter,
                 u_dut.dma_rd_cmd_out_token_group,
                 u_dut.dma_rd_cmd_head_gather_wide,
                 u_dut.dma_rd_cmd_out_token_base,
                 u_dut.dma_rd_cmd_out_token_addr_hi,
                 u_dut.dma_rd_cmd_addr, u_dut.dma_rd_int_cmd_addr,
                 u_dut.dma_rd_int_cmd_len);
        debug_dma_count = debug_dma_count + 1;
      end
      if (rst_n && debug_dma && u_dut.rp_busy &&
          u_dut.u_rope.state == 3'd1 &&
          debug_rope_count < 16) begin
        $display("FAKE_DDR_ROPE_TRACE q_even=%h q_odd=%h cos=%h sin=%h",
                 u_dut.prf_rd_data_a[15:0],
                 u_dut.prf_rd_data_a[31:16],
                 u_dut.prf_rd_data_b[15:0],
                 u_dut.prf_rd_data_b[16*16 +: 16]);
        debug_rope_count = debug_rope_count + 1;
      end
      if (rst_n && debug_dma && u_dut.vv_prf_wr_en &&
          debug_vld_count < 24) begin
        $display("FAKE_DDR_VLD_TRACE reg=%0d even=%h odd=%h",
                 u_dut.vv_prf_wr_addr,
                 u_dut.vv_prf_wr_data[15:0],
                 u_dut.vv_prf_wr_data[16*16 +: 16]);
        debug_vld_count = debug_vld_count + 1;
      end
    end
endmodule
`default_nettype wire

