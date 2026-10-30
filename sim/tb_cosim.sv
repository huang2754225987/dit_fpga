// =============================================================================
// tb_cosim.sv - Testbench for npu_cosim_top (NPU + BRAM, no DDR4)
// =============================================================================
// Uses BRAM port B backdoor for instant weight preload (1 cycle per 256-bit word)
// No DDR4 calibration needed - BRAM is instant-on.
// =============================================================================
`timescale 1ns/1ps

module tb_cosim;

    parameter int CLK_PERIOD   = 5;      // 200 MHz
    parameter int UCODE_DEPTH  = 262144;  // 256K for H=2048 with LOOP

    // Clock and reset
    logic clk, rst_n;
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // BRAM clock = main clock
    wire b_clk;
    assign b_clk = clk;

    // ── AXI-Lite (TB drives) ──
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

    // ── BRAM Port B backdoor ──
    logic         b_en;
    logic [31:0]  b_addr;
    logic [255:0] b_din;
    logic [31:0]  b_we;
    logic         b_rst;

    // ── UCODE preload ──
    logic         tb_ucode_wr_en;
    logic [17:0]  tb_ucode_wr_addr; // $clog2(262144) = 18 bits
    logic [31:0]  tb_ucode_wr_data;

    // ── NPU sim ports ──
    wire         s_acc_flush_valid;
    wire [31:0]  s_acc_flush_addr;
    wire [31:0]  s_acc_flush_data;

    // ── Prefill buffer (declared before tasks) ──
    byte prefill_buf[0:1048576]; // 1MB max

    // ── DUT ──
    npu_cosim_top u_dut (
        .clk(clk), .rst_n(rst_n),
        .s_axil_awaddr(s_axil_awaddr), .s_axil_awvalid(s_axil_awvalid),
        .s_axil_awready(s_axil_awready),
        .s_axil_wdata(s_axil_wdata), .s_axil_wstrb(s_axil_wstrb),
        .s_axil_wvalid(s_axil_wvalid), .s_axil_wready(s_axil_wready),
        .s_axil_bresp(s_axil_bresp), .s_axil_bvalid(s_axil_bvalid),
        .s_axil_bready(s_axil_bready),
        .s_axil_araddr(s_axil_araddr), .s_axil_arvalid(s_axil_arvalid),
        .s_axil_arready(s_axil_arready),
        .s_axil_rdata(s_axil_rdata), .s_axil_rresp(s_axil_rresp),
        .s_axil_rvalid(s_axil_rvalid), .s_axil_rready(s_axil_rready),
        .b_clk(b_clk), .b_en(b_en), .b_addr(b_addr),
        .b_din(b_din), .b_we(b_we), .b_rst(b_rst),
        .s_acc_flush_valid(s_acc_flush_valid),
        .s_acc_flush_addr(s_acc_flush_addr),
        .s_acc_flush_data(s_acc_flush_data),
        .tb_ucode_wr_en(tb_ucode_wr_en),
        .tb_ucode_wr_addr(tb_ucode_wr_addr),
        .tb_ucode_wr_data(tb_ucode_wr_data)
    );

    // =========================================================================
    // AXI-Lite write task
    // =========================================================================
    task axil_write(input [31:0] addr, input [31:0] data);
    begin
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
        while (!s_axil_bvalid) @(posedge clk);
        s_axil_bready  <= 1'b1;
        @(posedge clk);
        s_axil_bready  <= 1'b0;
    end
    endtask

    // =========================================================================
    // AXI-Lite read task
    // =========================================================================
    task axil_read(input [31:0] addr, output [31:0] data);
    begin
        @(posedge clk);
        s_axil_araddr  <= addr;
        s_axil_arvalid <= 1'b1;
        s_axil_rready  <= 1'b1;
        while (!s_axil_arready) @(posedge clk);
        s_axil_arvalid <= 1'b0;
        while (!s_axil_rvalid) @(posedge clk);
        data = s_axil_rdata;
        s_axil_rready <= 1'b0;
    end
    endtask

    // =========================================================================
    // BRAM backdoor write (1 cycle per 256-bit word - instant!)
    // =========================================================================
    task bram_write(input [31:0] addr, input [255:0] data);
    begin
        @(posedge b_clk);
        b_en   <= 1'b1;
        b_addr <= addr;
        b_din  <= data;
        b_we   <= 32'hFFFFFFFF;
        @(posedge b_clk);
        b_en <= 1'b0;
        b_we <= 32'h00000000;
    end
    endtask

    // =========================================================================
    // Pre-fill BRAM with weight data via port B backdoor
    // =========================================================================
    task prefill_bram(input string filename, input [31:0] base_addr, input int max_bytes);
        integer fd, r, total, i;
        logic [255:0] wdata256;
    begin
        fd = $fopen(filename, "rb");
        if (fd == 0) begin
            $display("[TB] WARNING: Cannot open %s - skipping", filename);
            return;
        end
        total = 0;
        while (!$feof(fd) && total < max_bytes) begin
            r = $fread(prefill_buf[total], fd);
            if (r > 0) total = total + r;
            else break;
        end
        $fclose(fd);
        $display("[TB] BRAM Pre-fill: %s -> 0x%08x, %0d bytes", filename, base_addr, total);

        for (i = 0; i + 31 < total; i += 32) begin
            wdata256 = {prefill_buf[i+31], prefill_buf[i+30],
                       prefill_buf[i+29], prefill_buf[i+28],
                       prefill_buf[i+27], prefill_buf[i+26],
                       prefill_buf[i+25], prefill_buf[i+24],
                       prefill_buf[i+23], prefill_buf[i+22],
                       prefill_buf[i+21], prefill_buf[i+20],
                       prefill_buf[i+19], prefill_buf[i+18],
                       prefill_buf[i+17], prefill_buf[i+16],
                       prefill_buf[i+15], prefill_buf[i+14],
                       prefill_buf[i+13], prefill_buf[i+12],
                       prefill_buf[i+11], prefill_buf[i+10],
                       prefill_buf[i+9],  prefill_buf[i+8],
                       prefill_buf[i+7],  prefill_buf[i+6],
                       prefill_buf[i+5],  prefill_buf[i+4],
                       prefill_buf[i+3],  prefill_buf[i+2],
                       prefill_buf[i+1],  prefill_buf[i+0]};
            bram_write(base_addr + i, wdata256);
        end
    end
    endtask

    integer ucode_gemm_expected = 0;
    integer ucode_expert_gemm_expected = 0;
    integer ucode_loaded_count = 0;
    string  ucode_filename;
    string  edesc_filename;

    // =========================================================================
    // Load ucode from file
    // =========================================================================
    task load_ucode(input string filename);
        integer fd, r, total, i;
        logic [31:0] word;
    begin
        fd = $fopen(filename, "rb");
        if (fd == 0) begin
            $display("[TB] ERROR: Cannot open %s", filename);
            $finish;
        end
        total = 0;
        ucode_gemm_expected = 0;
        while (!$feof(fd) && total < UCODE_DEPTH) begin
            r = $fread(word, fd);
            if (r == 4) begin
                tb_ucode_wr_en   <= 1'b1;
                tb_ucode_wr_addr <= total[$clog2(UCODE_DEPTH)-1:0];
                tb_ucode_wr_data <= word;
                if (word[31:28] == 4'h5) begin
                    ucode_gemm_expected = ucode_gemm_expected + 1;
                    if (word[19:16] != 4'h0)
                        ucode_expert_gemm_expected = ucode_expert_gemm_expected + 1;
                end
                @(posedge clk);
                total = total + 1;
            end else break;
        end
        $fclose(fd);
        tb_ucode_wr_en <= 1'b0;
        ucode_loaded_count = total;
        $display("[TB] Loaded %0d ucode instructions from %s (%0d GEMMs)",
                 total, filename, ucode_gemm_expected);
    $fflush;
    end
    endtask

    // =========================================================================
    // Task: load the 180-entry expert descriptor table (12B/entry LE:
    // w0=weight_base, w1=scale_base, w2=attr) via REG_EDESC_IDX/W0/W1/W2.
    // Replaces the deprecated expert_prec_entries.txt / REG_EXPERT_PREC_CTRL flow.
    // =========================================================================
    task load_expert_desc(string filename);
        integer fd, ei, rdsz;
        logic [7:0]  edesc_bytes [0:180*12-1];
        logic [31:0] ew0, ew1, ew2;
        begin
            fd = $fopen(filename, "rb");
            if (fd == 0) begin
                $display("[TB] WARNING: Cannot open %s - skipping expert desc load", filename);
            end else begin
                rdsz = $fread(edesc_bytes, fd);
                $fclose(fd);
                if (rdsz < 180*12)
                    $display("[TB] WARNING: %s short read (%0d < %0d bytes)", filename, rdsz, 180*12);
                for (ei = 0; ei < 180; ei++) begin
                    ew0 = {edesc_bytes[ei*12+3],  edesc_bytes[ei*12+2],
                           edesc_bytes[ei*12+1],  edesc_bytes[ei*12+0]};
                    ew1 = {edesc_bytes[ei*12+7],  edesc_bytes[ei*12+6],
                           edesc_bytes[ei*12+5],  edesc_bytes[ei*12+4]};
                    ew2 = {edesc_bytes[ei*12+11], edesc_bytes[ei*12+10],
                           edesc_bytes[ei*12+9],  edesc_bytes[ei*12+8]};
                    axil_write(32'h84, ei);   // REG_EDESC_IDX
                    axil_write(32'h88, ew0);  // REG_EDESC_W0 weight_base
                    axil_write(32'h8C, ew1);  // REG_EDESC_W1 scale_base
                    axil_write(32'h90, ew2);  // REG_EDESC_W2 attr
                end
                $display("[TB] Loaded 180 expert descriptors from %s", filename);
            end
        end
    endtask

    // =========================================================================
    // Main - Full 25-layer TTFT simulation
    // =========================================================================
    integer cycle_cnt = 0;
    integer gemm_active_cycles = 0;  // cycles where GEMM is not IDLE
    integer gemm_done_count = 0;     // count GEMM completions
    integer layer_start;
    integer layer_cycles;
    integer total_cycles;
    integer layer_idx;
    integer poll_cnt;
    integer expert_prec_check_count = 0;
    integer dequant_width_check_count = 0;
    logic done;
    logic [31:0] reg_val;
    logic expert_gemm_check_pending = 0;
    logic [1:0] expert_gemm_expected_prec = 0;
    logic scale_load_completed = 0;
    logic [5:0] tb_selected_expert;
    logic [1:0] tb_table_prec;

    // ---- Decode (TPOT) mode state ----
    integer decode_mode = 0;
    integer cfg_m_plusarg;
    integer n_tokens = 8;
    integer tok_start;
    integer tok_cycles;
    integer tok_total = 0;
    integer tok_first = 0;
    string  tok_ucode;
    integer kv_pass = 0;
    integer kv_fail = 0;
    integer kv_xonly = 0;   // compared words that were all-X (vacuous, not counted as PASS)
    integer cur_tok = -1;          // token index whose ucode is running

    // =========================================================================
    // PERF (Task 0): cycle decomposition counters — tb-only observation,
    // no RTL change. Enabled between START_ACCEPT and ALL_IDLE of each layer.
    // =========================================================================
    integer perf_gemm_state [0:7];
    integer perf_gather_prec [0:3];
    integer perf_dma_rd, perf_dma_rd_gemm_ov, perf_dma_rd_exposed;
    integer perf_sc_load, perf_sc_exposed;
    integer perf_topk, perf_gld_w, perf_gld_a, perf_gld_exposed, perf_dma_wr;
    integer perf_barrier, perf_barrier_sc, perf_barrier_dma, perf_barrier_gemm;
    integer perf_stall [0:15];
    integer perf_issue [0:15];
    integer perf_stall_nop_subop [0:15];
    integer perf_stall_resource [0:7];
    integer perf_stall_gemm_wrf, perf_stall_gemm_arf;
    integer perf_idle_bubble;
    integer pi, pj;
    logic   perf_en;

    initial begin
        perf_en = 0;
        for (pi = 0; pi < 8;  pi++) perf_gemm_state[pi] = 0;
        for (pi = 0; pi < 4;  pi++) perf_gather_prec[pi] = 0;
        for (pi = 0; pi < 16; pi++) begin
            perf_stall[pi] = 0;
            perf_issue[pi] = 0;
            perf_stall_nop_subop[pi] = 0;
        end
        for (pi = 0; pi < 8; pi++) perf_stall_resource[pi] = 0;
        perf_dma_rd = 0; perf_dma_rd_gemm_ov = 0; perf_dma_rd_exposed = 0;
        perf_sc_load = 0; perf_sc_exposed = 0;
        perf_topk = 0; perf_gld_w = 0; perf_gld_a = 0; perf_gld_exposed = 0;
        perf_dma_wr = 0; perf_barrier = 0; perf_barrier_sc = 0;
        perf_barrier_dma = 0; perf_barrier_gemm = 0;
        perf_stall_gemm_wrf = 0; perf_stall_gemm_arf = 0;
        perf_idle_bubble = 0;
    end

    wire perf_gemm_busy = (u_dut.u_npu.u_gemm_ctrl.state != 4'd0);
    wire perf_gld_busy  = (u_dut.u_npu.u_gld_ctrl.state != 4'd0);
    wire perf_any_busy  = perf_gemm_busy | perf_gld_busy |
                          u_dut.u_npu.dma_rd_busy | u_dut.u_npu.dma_wr_busy |
                          u_dut.u_npu.sc_load_active | u_dut.u_npu.topk_busy_r |
                          u_dut.u_npu.uc_load_active |
                          u_dut.u_npu.u_scoreboard.res_busy[3];

    always @(posedge clk) begin
        if (rst_n && perf_en) begin
            perf_gemm_state[u_dut.u_npu.u_gemm_ctrl.state] <=
                perf_gemm_state[u_dut.u_npu.u_gemm_ctrl.state] + 1;
            if (u_dut.u_npu.u_gemm_ctrl.state == 4'd3)  // ST_GATHER
                perf_gather_prec[u_dut.u_npu.u_gemm_ctrl.r_prec] <=
                    perf_gather_prec[u_dut.u_npu.u_gemm_ctrl.r_prec] + 1;

            if (u_dut.u_npu.dma_rd_busy) begin
                perf_dma_rd <= perf_dma_rd + 1;
                if (perf_gemm_busy) perf_dma_rd_gemm_ov <= perf_dma_rd_gemm_ov + 1;
                else                perf_dma_rd_exposed <= perf_dma_rd_exposed + 1;
            end
            if (u_dut.u_npu.sc_load_active) begin
                perf_sc_load <= perf_sc_load + 1;
                if (!perf_gemm_busy && !u_dut.u_npu.dma_rd_busy)
                    perf_sc_exposed <= perf_sc_exposed + 1;
            end
            if (u_dut.u_npu.topk_busy_r) perf_topk <= perf_topk + 1;
            if (perf_gld_busy) begin
                if (u_dut.u_npu.u_gld_ctrl.state <= 4'd3)
                    perf_gld_w <= perf_gld_w + 1;
                else if (u_dut.u_npu.u_gld_ctrl.state <= 4'd6)
                    perf_gld_a <= perf_gld_a + 1;
                if (!perf_gemm_busy && !u_dut.u_npu.dma_rd_busy)
                    perf_gld_exposed <= perf_gld_exposed + 1;
            end
            if (u_dut.u_npu.dma_wr_busy) perf_dma_wr <= perf_dma_wr + 1;

            if (u_dut.u_npu.barrier_active) begin
                perf_barrier <= perf_barrier + 1;
                if (u_dut.u_npu.sc_load_active)
                    perf_barrier_sc <= perf_barrier_sc + 1;
                else if (u_dut.u_npu.dma_rd_busy)
                    perf_barrier_dma <= perf_barrier_dma + 1;
                else if (perf_gemm_busy)
                    perf_barrier_gemm <= perf_barrier_gemm + 1;
            end

            // Decode head-of-line stall attributed by opcode
            if (u_dut.u_npu.lc_dec_valid && !u_dut.u_npu.fetch_instr_ready) begin
                perf_stall[u_dut.u_npu.lc_dec_instr[31:28]] <=
                    perf_stall[u_dut.u_npu.lc_dec_instr[31:28]] + 1;
                if (u_dut.u_npu.lc_dec_instr[31:28] == 4'd0)
                    perf_stall_nop_subop[
                        u_dut.u_npu.lc_dec_instr[15:12]] <=
                        perf_stall_nop_subop[
                            u_dut.u_npu.lc_dec_instr[15:12]] + 1;
                for (pj = 0; pj < 8; pj++) begin
                    if (u_dut.u_npu.u_decode.needed_resources[pj] &&
                        u_dut.u_npu.u_scoreboard.res_busy[pj])
                        perf_stall_resource[pj] <=
                            perf_stall_resource[pj] + 1;
                end
                if (u_dut.u_npu.lc_dec_instr[31:28] == 4'd5) begin
                    if (!u_dut.u_npu.wrf_data_valid)
                        perf_stall_gemm_wrf <= perf_stall_gemm_wrf + 1;
                    if (!u_dut.u_npu.arf_data_valid)
                        perf_stall_gemm_arf <= perf_stall_gemm_arf + 1;
                end
            end
            if (u_dut.u_npu.lc_dec_valid && u_dut.u_npu.fetch_instr_ready)
                perf_issue[u_dut.u_npu.lc_dec_instr[31:28]] <=
                    perf_issue[u_dut.u_npu.lc_dec_instr[31:28]] + 1;

            if (!perf_any_busy) perf_idle_bubble <= perf_idle_bubble + 1;
        end
    end

    // KV cache monitor: K slot region BRAM bytes [0xC000,0xD000),
    // V slot region [0xE000,0xF000). BRAM truncates AXI addr to 21 bits.
    logic [255:0] kv_k_wdata [0:7];   // first W beat of each slot append
    logic [7:0]   kv_k_wseen;
    logic [255:0] kv_v_wdata [0:7];
    logic [7:0]   kv_v_wseen;
    logic         wr_armed;           // next W beat starts an append burst
    integer       wr_slot;
    logic         wr_is_v;
    integer       rd_beat;
    logic         rd_active;

    always_comb begin
        case (u_dut.u_npu.gemm_cmd_expert_sel)
            4'd1: tb_selected_expert = u_dut.u_npu.topk_idx_arr[0];
            4'd2: tb_selected_expert = u_dut.u_npu.topk_idx_arr[1];
            4'd3: tb_selected_expert = u_dut.u_npu.topk_idx_arr[2];
            4'd4: tb_selected_expert = u_dut.u_npu.topk_idx_arr[3];
            default: tb_selected_expert = 6'd0;
        endcase
        // Expert precision now lives in the descriptor table attr[1:0]
        // (mem bits [65:64]). expert_prec_table was removed from top.sv.
        tb_table_prec = u_dut.u_npu.u_expert_desc_table.mem[
            tb_selected_expert * 3 + u_dut.u_npu.gemm_cmd_proj][65:64];
    end

    initial begin
        rst_n = 0;
        s_axil_awaddr = 0; s_axil_awvalid = 0;
        s_axil_wdata = 0; s_axil_wstrb = 0; s_axil_wvalid = 0;
        s_axil_bready = 0;
        s_axil_araddr = 0; s_axil_arvalid = 0; s_axil_rready = 0;
        b_en = 0; b_addr = 0; b_din = 0; b_we = 0; b_rst = 1;
        tb_ucode_wr_en = 0; tb_ucode_wr_addr = 0; tb_ucode_wr_data = 0;
        cycle_cnt = 0;
        total_cycles = 0;
        layer_idx = 0;
        poll_cnt = 0;

        repeat(10) @(posedge clk);
        rst_n = 1;
        b_rst = 0;
        repeat(5) @(posedge clk);

        // Small A/B regression for Python static-linear FP16 K-tile
        // scheduling.  K-tile 0 has weight=1.0, K-tile 1 has weight=2.0,
        // and all activations are 1.0, so every accumulated output is 96.0.
        if ($test$plusargs("STATIC_FP16_PIPELINE_TEST")) begin : static_fp16_pipeline_test
            integer fp_i, fp_tile, fp_col, fp_start, fp_elapsed, fp_mismatches;
            logic [31:0] fp_value;
            logic [255:0] fp_line;
            string fp_ucode;
            fp_mismatches = 0;
            fp_line = {8{32'h3C00_3C00}};
            for (fp_i = 0; fp_i < 128; fp_i++)
                bram_write(fp_i * 32, fp_line);
            for (fp_tile = 0; fp_tile < 64; fp_tile++) begin
                fp_line = (fp_tile[0] == 1'b0)
                        ? {8{32'h3C00_3C00}} : {8{32'h4000_4000}};
                for (fp_i = 0; fp_i < 64; fp_i++)
                    bram_write(32'h0001_0000 + fp_tile * 2048 + fp_i * 32,
                               fp_line);
            end

            if (!$value$plusargs("UCODE_FILE=%s", fp_ucode))
                $fatal(1, "[TB][STATIC_FP16] UCODE_FILE is required");
            load_ucode(fp_ucode);
            repeat(5) @(posedge clk);
            axil_write(32'h0C, ucode_loaded_count);
            axil_write(32'h10, 32'h0000_0000); // DDR_BASE_ACT
            axil_write(32'h14, 32'h0000_0000); // DDR_BASE_WGT
            axil_write(32'h1C, 32'h0010_0000); // DDR_BASE_OUT
            axil_write(32'h48, 32'h0000_0000); // DDR_BASE_SCALE
            axil_write(32'h4C, 32'd32);       // CFG_M
            axil_write(32'h50, 32'd32);       // CFG_N
            axil_write(32'h54, 32'd32);       // CFG_K
            fp_start = cycle_cnt;
            axil_write(32'h00, 32'h0000_0001);
            repeat(2) @(posedge clk);
            while (cycle_cnt - fp_start < 1000000) begin
                @(posedge clk);
                if (u_dut.u_npu.npu_done_latch) begin
                    while (!u_dut.u_npu.all_idle &&
                           cycle_cnt - fp_start < 1000000)
                        @(posedge clk);
                    break;
                end
            end
            fp_elapsed = cycle_cnt - fp_start;
            if (!u_dut.u_npu.npu_done_latch || !u_dut.u_npu.all_idle)
                $fatal(1, "[TB][STATIC_FP16] timeout after %0d cycles", fp_elapsed);
            if (gemm_done_count != 64)
                $fatal(1, "[TB][STATIC_FP16] GEMM count %0d != 64", gemm_done_count);
            for (fp_col = 0; fp_col < 32; fp_col++) begin
                if (u_dut.u_npu.acc_bank_sel)
                    fp_value = u_dut.u_npu.u_acc_buffer.bank0_mem[fp_col][0];
                else
                    fp_value = u_dut.u_npu.u_acc_buffer.bank1_mem[fp_col][0];
                if (fp_value !== 32'h4540_0000) begin
                    fp_mismatches++;
                    if (fp_mismatches <= 8)
                        $display("[TB][STATIC_FP16] mismatch col=%0d got=%08x expected=45400000",
                                 fp_col, fp_value);
                end
            end
            if (fp_mismatches != 0)
                $fatal(1, "[TB][STATIC_FP16] FAIL mismatches=%0d", fp_mismatches);
            $display("[TB][STATIC_FP16] PASS cycles=%0d gemms=64 outputs=32 mismatch=0",
                     fp_elapsed);
            $finish;
        end

        // ================= AC2V 通路测试 (P1: acc->PRF->L1->DMA->DDR) =================
        // GEMM(fp16, K=32, 单 K-tile) 后，AC2V×32 + VST×32 + DMA_WR×2 把整个
        // 32x32 acc tile 搬到 DDR；与 acc_buffer 结果集（acc_flush host 口同一
        // 读口来源）逐元素 fp32->fp16(RNE) 比对，要求 bit-exact。
        if ($test$plusargs("AC2V_TEST")) begin : ac2v_test
            integer ac2v_mismatch;
            integer ac2v_checked;
            $display("[TB][AC2V] Pre-filling BRAM (act@0x0, wgt@0x10000)...");
            prefill_bram("D:/graduation_design/npu-main/sim/modelsim/ac2v_act.bin",
                         32'h0000_0000, 4096);
            prefill_bram("D:/graduation_design/npu-main/sim/modelsim/ac2v_wgt.bin",
                         32'h0001_0000, 4096);
            if (!$value$plusargs("UCODE_FILE=%s", ucode_filename))
                ucode_filename = "D:/graduation_design/npu-main/sim/modelsim/ac2v_cosim.bin";
            load_ucode(ucode_filename);
            repeat(5) @(posedge clk);
            axil_write(32'h0C, ucode_loaded_count);
            axil_write(32'h10, 32'h0000_0000);   // DDR_BASE_ACT
            axil_write(32'h14, 32'h0000_0000);   // DDR_BASE_WGT
            axil_write(32'h1C, 32'h0010_0000);   // DDR_BASE_OUT = 1MB
            axil_write(32'h48, 32'h0000_0000);   // DDR_BASE_SCALE
            axil_write(32'h4C, 32'd32);          // CFG_M
            axil_write(32'h50, 32'd32);          // CFG_N
            axil_write(32'h54, 32'd32);          // CFG_K
            $display("[TB][AC2V] Starting NPU...");
            axil_write(32'h00, 32'h0000_0001);   // CTRL.start
            done = 0;
            while (cycle_cnt < 10000000) begin
                @(posedge clk);
                if (u_dut.u_npu.npu_done_latch) begin
                    while (!u_dut.u_npu.all_idle && cycle_cnt < 10000000)
                        @(posedge clk);
                    done = 1;
                    break;
                end
            end
            if (!done)
                $fatal(1, "[TB][AC2V] FAIL: TIMEOUT");
            $display("[TB][AC2V] NPU done at cycle %0d, comparing DDR out vs acc_buffer...",
                     cycle_cnt);
            // DMA_WR#0: L1 0x8000 -> DDR 0x108000 (rows 0..15)
            // DMA_WR#1: L1 0x8400 -> DDR 0x108400 (rows 16..31)
            // 结果集 = 非活动集 = ~acc_bank_sel；bank 布局 bankX_mem[col][row]
            ac2v_mismatch = 0;
            ac2v_checked  = 0;
            for (integer r = 0; r < 32; r++) begin
                for (integer j = 0; j < 32; j++) begin
                    logic [31:0] acc_w;
                    logic [15:0] exp_h, act_h;
                    integer      byte_addr, ln, sl;
                    acc_w = u_dut.u_npu.acc_bank_sel
                            ? u_dut.u_npu.u_acc_buffer.bank0_mem[j][r]
                            : u_dut.u_npu.u_acc_buffer.bank1_mem[j][r];
                    exp_h     = fp16_utils_pkg::fp32_to_fp16(acc_w);
                    byte_addr = 32'h0010_8000 + r * 64 + j * 2;
                    ln        = byte_addr >> 5;         // BRAM 32B 行号
                    sl        = byte_addr[4:0] >> 1;    // 行内 16-bit 槽
                    act_h     = u_dut.u_bram.mem[ln][sl*16 +: 16];
                    ac2v_checked = ac2v_checked + 1;
                    if (act_h !== exp_h) begin
                        ac2v_mismatch = ac2v_mismatch + 1;
                        if (ac2v_mismatch <= 8 || j == 0 || j == 16)
                            $display("[TB][AC2V] MISMATCH row=%0d col=%0d exp=%04x got=%04x (acc_fp32=%08x)",
                                     r, j, exp_h, act_h, acc_w);
                    end
                end
            end
            if (ac2v_mismatch == 0)
                $display("[TB][AC2V] PASS checked=%0d mismatch=0 (acc->PRF->L1->DMA->DDR fp16 逐元素一致)",
                         ac2v_checked);
            else
                $fatal(1, "[TB][AC2V] FAIL checked=%0d mismatch=%0d",
                       ac2v_checked, ac2v_mismatch);
            $finish;
        end

        $display("[TB] === NPU Full TTFT Simulation (25 layers) ===");

        // 1. Pre-fill BRAM with weight data via port B backdoor
        $display("[TB] Pre-filling BRAM...");
        prefill_bram("D:/graduation_design/npu-main/sim/vivado/mig_msim/ddr4_0/modelsim/wq.bin",
                     32'h00001000, 32768);
        prefill_bram("D:/graduation_design/npu-main/activation.bin", 32'h00000000, 8192);
        prefill_bram("D:/graduation_design/npu-main/input_ln.bin", 32'h00008200, 256);
        prefill_bram("D:/graduation_design/npu-main/post_attn_ln.bin", 32'h00008400, 256);
        $display("[TB] BRAM pre-fill complete!");

        // 2. Load ucode
        $display("[TB] Loading ucode...");
        if (!$value$plusargs("UCODE_FILE=%s", ucode_filename))
            ucode_filename = "D:/graduation_design/npu-main/ucode.bin";
        load_ucode(ucode_filename);
        repeat(5) @(posedge clk);

        // 3. Configure registers (same for all layers)
        $display("[TB] Configuring registers...");
        axil_write(32'h0C, ucode_loaded_count);
        $display("[TB] UCODE_LEN written: %0d", ucode_loaded_count);
        axil_write(32'h10, 32'h0000_0000);   // DDR_BASE_ACT
        axil_write(32'h14, 32'h0000_0000);   // DDR_BASE_WGT
        axil_write(32'h1C, 32'h0040_0000);   // DDR_BASE_OUT
        axil_write(32'h48, 32'h0000_0000);   // DDR_BASE_SCALE
        axil_write(32'h4C, 32'd32);          // CFG_M
        if ($value$plusargs("CFG_M=%d", cfg_m_plusarg)) begin
            axil_write(32'h4C, cfg_m_plusarg);   // CFG_M override (e.g. LM head M=1)
            $display("[TB] CFG_M overridden by plusarg: %0d", cfg_m_plusarg);
        end
        axil_write(32'h50, 32'd32);          // CFG_N
        axil_write(32'h54, 32'd64);          // CFG_K = actual K_tile = 64 (was 128: 2x GLD over-read, task-0 finding)
        $display("[TB] Loading expert descriptor table...");
        if (!$value$plusargs("EDESC_FILE=%s", edesc_filename))
            edesc_filename = "D:/graduation_design/npu-main/sim/modelsim/expert_desc_cosim.bin";
        load_expert_desc(edesc_filename);
        $display("[TB] All registers written, starting NPU...");
        repeat(5) @(posedge clk);

        $display("[MARK] COLD_SETUP_DONE cycle=%0d (BRAM preload + ucode + regs + descriptors; TTFT_cold_sim overhead)", cycle_cnt);
        // Optional deterministic BRAM fill (non-X data so KV observability
        // compares real bytes instead of X===X). Excluded from token cycles.
        if ($test$plusargs("FILL_BRAM")) begin
            $display("[TB] Filling BRAM with deterministic pattern...");
            for (integer fi = 0; fi < 65536; fi++) begin
                logic [255:0] fw;
                for (integer fj = 0; fj < 8; fj++) begin
                    if ($test$plusargs("FILL_FP16_ONE"))
                        fw[fj*32 +: 32] = 32'h3C00_3C00;
                    else
                        fw[fj*32 +: 32] = 32'h3C00_0000 + fi*8 + fj;
                end
                bram_write(fi*32, fw);
            end
            $display("[TB] BRAM fill done");
        end
        // ================= DECODE MODE (TPOT, M5/M6) =================
        if ($value$plusargs("DECODE_MODE=%d", decode_mode)) ;
        if (!$value$plusargs("N_TOKENS=%d", n_tokens)) n_tokens = 8;
        if (decode_mode) begin
            $display("[TB] === DECODE MODE: %0d tokens, no reset/reload, KV cache resident ===", n_tokens);
            for (integer t = 0; t < n_tokens; t++) begin
                // Host-in-the-loop ucode swap (NPU idle, no reset, data BRAM kept)
                $sformat(tok_ucode, "D:/graduation_design/npu-main/sim/decode/ucode_decode_t%0d.bin", t);
                load_ucode(tok_ucode);
                axil_write(32'h0C, ucode_loaded_count);
                repeat(5) @(posedge clk);
                cur_tok = t;
                axil_write(32'h00, 32'h0000_0001);   // CTRL.start
                repeat(2) @(posedge clk);
                tok_start = cycle_cnt;
                $display("[MARK] DECODE_START token=%0d cycle=%0d", t, cycle_cnt);
                $fflush;
                done = 0;
                while (cycle_cnt - tok_start < 100000000) begin
                    @(posedge clk);
                    if ((cycle_cnt - tok_start) % 100000 == 0 && cycle_cnt != tok_start) begin
                        $display("[TB] token %0d: %0d cyc", t, cycle_cnt - tok_start);
                        $fflush;
                    end
                    if (u_dut.u_npu.npu_done_latch) begin
                        while (!u_dut.u_npu.all_idle && (cycle_cnt - tok_start < 100000000))
                            @(posedge clk);
                        tok_cycles = cycle_cnt - tok_start;
                        if (t == 0) tok_first = tok_cycles;
                        else        tok_total = tok_total + tok_cycles;
                        $display("[MARK] DECODE_DONE token=%0d cycle=%0d cycles=%0d (AC-07 raw)",
                                 t, cycle_cnt, tok_cycles);
                        $fflush;
                        done = 1;
                        break;
                    end
                end
                if (!done) begin
                    $display("[TB] Decode token %0d TIMEOUT", t);
                    $finish;
                end
            end
            $display("========================================");
            $display("[TB] TPOT SUMMARY (one-layer component, AC-08)");
            $display("  Tokens: %0d (first/warm-up separated)", n_tokens);
            $display("  First token cycles: %0d", tok_first);
            if (n_tokens > 1)
                $display("  Steady-state mean: %.1f cycles/token over %0d tokens",
                         real'(tok_total)/real'(n_tokens-1), n_tokens-1);
            $display("  KV observability: %0d PASS, %0d FAIL, %0d XONLY(vacuous)", kv_pass, kv_fail, kv_xonly);
            $display("  GEMM done count: %0d", gemm_done_count);
            $display("  NPU freq: 200 MHz (5 ns/cycle)");
            $display("========================================");
            $fflush;
            $finish;
        end
        // 4. Run 25 layers (24 transformer + LM head)
        for (layer_idx = 0; layer_idx < 1; layer_idx++) begin  // reduced to 1 for test
            // Start NPU (writing CTRL=1 clears done_latch and restarts)
            axil_write(32'h00, 32'h0000_0001);   // CTRL.start
            repeat(2) @(posedge clk);
            $display("[MARK] START_ACCEPT layer=%0d cycle=%0d (AC-06 t0)", layer_idx, cycle_cnt);

            layer_start = cycle_cnt;
            perf_en = 1;
            done = 0;

            // Wait for DONE (per-layer watchdog: 10M cycles for K-tiled H=2048)
            while (cycle_cnt - layer_start < 100000000) begin
                @(posedge clk);
                // Periodic status every 10K cycles
               if ((cycle_cnt - layer_start) % 10000 == 0 && cycle_cnt != layer_start) begin
                   $display("[TB] %0d cyc", cycle_cnt - layer_start);
               end
                if (u_dut.u_npu.npu_done_latch) begin
                    // PC reached END, but GEMMs may still be running
                    $display("[TB] Layer %0d PC_DONE: %0d cycles, waiting for all_idle...",
                             layer_idx, cycle_cnt - layer_start);
                    $display("[MARK] LAYER_PC_DONE layer=%0d cycle=%0d rel=%0d", layer_idx, cycle_cnt, cycle_cnt - layer_start);
                    // Wait for all resources to be idle (all GEMMs complete)
                    while (!u_dut.u_npu.all_idle && (cycle_cnt - layer_start < 100000000)) begin
                        @(posedge clk);
                        if ((cycle_cnt - layer_start) % 50000 == 0 && cycle_cnt != layer_start) begin
                            $display("[TB]   %0d cyc all_idle=%b gemm_st=%0d res=%08b",
                                     cycle_cnt - layer_start, u_dut.u_npu.all_idle,
                                     u_dut.u_npu.u_gemm_ctrl.state,
                                     u_dut.u_npu.u_scoreboard.res_busy);
                        end
                    end
                    layer_cycles = cycle_cnt - layer_start;
                    total_cycles = total_cycles + layer_cycles;
                    $display("[TB] Layer %0d ALL_IDLE: %0d cycles (total=%0d, sim_time=%.3f ns)",
                             layer_idx, layer_cycles, total_cycles, $realtime);
                    $display("[MARK] FIRST_RESULT_VALID layer=%0d cycle=%0d TTFT_compute_cycles=%0d (one-layer component, AC-08)", layer_idx, cycle_cnt, layer_cycles);
                    perf_en = 0;
                    done = 1;
                    break;
                end
            end

            if (!done) begin
                $display("[TB] Layer %0d TIMEOUT after %0d cycles!", layer_idx, cycle_cnt - layer_start);
                $display("[TB] Partial results: %0d layers, %0d cycles", layer_idx, total_cycles);
                $finish;
            end
        end

        // Results
        $display("");
        $display("========================================");
        $display("[TB] FULL H=2048 STRUCTURAL/TIMING Test (PLACEHOLDER WEIGHTS)");
        $display("  Layers executed: %0d", layer_idx);
        $display("  Total cycles: %0d", total_cycles);
        $display("  Sim time: %.3f ns = %.3f ms", $realtime, $realtime/1e6);
        $display("  NPU freq: 200 MHz (5 ns/cycle)");
        $display("  TTFT: %.3f ms", total_cycles * 5.0 / 1e6);
        $display("  GEMM active cycles: %0d (%.1f%% of total)", gemm_active_cycles, real'(gemm_active_cycles)/real'(total_cycles)*100.0);
        $display("  GEMM done count: %0d (expected %0d)", gemm_done_count, ucode_gemm_expected);
        $display("  Expert precision checks: %0d", expert_prec_check_count);
        $display("  Dequant width checks: %0d", dequant_width_check_count);
        if ($test$plusargs("DUMP_ACC_ROW0")) begin
            for (integer dump_col = 0; dump_col < 32; dump_col++) begin
                if (u_dut.u_npu.acc_bank_sel)
                    $display("[TB][ACC_DUMP] col=%0d value=%08x", dump_col,
                             u_dut.u_npu.u_acc_buffer.bank0_mem[dump_col][0]);
                else
                    $display("[TB][ACC_DUMP] col=%0d value=%08x", dump_col,
                             u_dut.u_npu.u_acc_buffer.bank1_mem[dump_col][0]);
            end
        end
        if (gemm_done_count != ucode_gemm_expected)
            $error("[TB] GEMM count mismatch: got %0d expected %0d",
                   gemm_done_count, ucode_gemm_expected);
        if (ucode_expert_gemm_expected > 0 &&
            (expert_prec_check_count == 0 || dequant_width_check_count == 0))
            $error("[TB] Dynamic precision assertions had zero coverage");
        if (gemm_done_count > 0)
            $display("  Avg cyc/GEMM: %.1f", real'(gemm_active_cycles)/real'(gemm_done_count));
        $display("---- PERF decomposition (Task 0, one-layer window) ----");
        $display("  GEMM state cyc: IDLE=%0d SETUP=%0d CLEAR=%0d GATHER=%0d FEED=%0d DRAIN=%0d STORE=%0d DONE=%0d",
                 perf_gemm_state[0], perf_gemm_state[1], perf_gemm_state[2], perf_gemm_state[3],
                 perf_gemm_state[4], perf_gemm_state[5], perf_gemm_state[6], perf_gemm_state[7]);
        $display("  GATHER by prec: FP16=%0d INT1=%0d INT2=%0d INT3=%0d",
                 perf_gather_prec[0], perf_gather_prec[1], perf_gather_prec[2], perf_gather_prec[3]);
        $display("  DMA_RD busy=%0d (overlap GEMM=%0d, exposed=%0d)",
                 perf_dma_rd, perf_dma_rd_gemm_ov, perf_dma_rd_exposed);
        $display("  SCALE load=%0d (exposed, no GEMM/DMA=%0d)", perf_sc_load, perf_sc_exposed);
        $display("  TOPK busy=%0d", perf_topk);
        $display("  GLD_W=%0d GLD_A=%0d GLD exposed(no GEMM/DMA)=%0d",
                 perf_gld_w, perf_gld_a, perf_gld_exposed);
        $display("  DMA_WR busy=%0d", perf_dma_wr);
        $display("  BARRIER wait=%0d (on sc_load=%0d, on dma_rd=%0d, on gemm=%0d, other=%0d)",
                 perf_barrier, perf_barrier_sc, perf_barrier_dma, perf_barrier_gemm,
                 perf_barrier - perf_barrier_sc - perf_barrier_dma - perf_barrier_gemm);
        $display("  Decode stall by opcode: NOP=%0d VLD=%0d VST=%0d GLD_W=%0d GLD_A=%0d GEMM=%0d DMA_RD=%0d DMA_WR=%0d BARRIER=%0d other=%0d",
                 perf_stall[0], perf_stall[1], perf_stall[2], perf_stall[3], perf_stall[4],
                 perf_stall[5], perf_stall[12], perf_stall[13], perf_stall[14],
                 perf_stall[6]+perf_stall[7]+perf_stall[8]+perf_stall[9]+perf_stall[10]+perf_stall[11]+perf_stall[15]);
        $display("    NOP sub-op stalls: TOPK=%0d SCALE=%0d RWEIGHT=%0d SET_CFG_K=%0d SET_CFG_M=%0d STATIC_SCALE=%0d",
                 perf_stall_nop_subop[1], perf_stall_nop_subop[2],
                 perf_stall_nop_subop[7], perf_stall_nop_subop[14],
                 perf_stall_nop_subop[15], perf_stall_nop_subop[11]);
        $display("    Needed-resource conflict cycles (overlapping): res0=%0d res1=%0d res2=%0d res3=%0d res4=%0d res5=%0d res6=%0d res7=%0d",
                 perf_stall_resource[0], perf_stall_resource[1],
                 perf_stall_resource[2], perf_stall_resource[3],
                 perf_stall_resource[4], perf_stall_resource[5],
                 perf_stall_resource[6], perf_stall_resource[7]);
        $display("    GEMM stall: wait WRF=%0d wait ARF=%0d", perf_stall_gemm_wrf, perf_stall_gemm_arf);
        $display("  Idle bubble (no engine busy)=%0d", perf_idle_bubble);
        $display("----------------------------------------------");
        $display("========================================");
        $finish;
    end

   // Cycle counter with GEMM metrics
   always @(posedge clk) begin
       if (rst_n) begin
           cycle_cnt <= cycle_cnt + 1;
           if (u_dut.u_npu.u_gemm_ctrl.state != 3'd0)
               gemm_active_cycles <= gemm_active_cycles + 1;
           if (u_dut.u_npu.u_gemm_ctrl.done)
               gemm_done_count <= gemm_done_count + 1;

           // NOP_SCALE must complete before the first expert GLD.W can issue.
           if (u_dut.u_npu.scale_cmd_valid)
               scale_load_completed <= 1'b0;
           if (u_dut.u_npu.sc_load_done)
               scale_load_completed <= 1'b1;
           if (u_dut.u_npu.gld_w_cmd_valid &&
               u_dut.u_npu.gld_w_cmd_expert_sel != 0 && !scale_load_completed)
               $error("[TB] Expert GLD.W issued before scale DMA completed");

           // Check table lookup at dispatch, then the value latched by gemm_ctrl
           // on the following clock.
           if (expert_gemm_check_pending) begin
               if (u_dut.u_npu.u_gemm_ctrl.r_prec !== expert_gemm_expected_prec)
                   $error("[TB] GEMM r_prec mismatch: got %0d expected %0d",
                          u_dut.u_npu.u_gemm_ctrl.r_prec, expert_gemm_expected_prec);
               expert_prec_check_count <= expert_prec_check_count + 1;
           end
           expert_gemm_check_pending <= u_dut.u_npu.gemm_cmd_valid &&
                                        (u_dut.u_npu.gemm_cmd_expert_sel != 0);
           if (u_dut.u_npu.gemm_cmd_valid &&
               u_dut.u_npu.gemm_cmd_expert_sel != 0) begin
               if (u_dut.u_npu.gemm_prec_eff !== tb_table_prec)
                   $error("[TB] Expert table lookup mismatch: got %0d expected %0d",
                          u_dut.u_npu.gemm_prec_eff, tb_table_prec);
               expert_gemm_expected_prec <= tb_table_prec;
           end

           if (u_dut.u_npu.dq_in_valid) begin
               if (u_dut.u_npu.gemm_dq_nbits_code !==
                   {1'b0, u_dut.u_npu.u_gemm_ctrl.r_prec})
                   $error("[TB] Dequant width mismatch: nbits=%0d prec=%0d",
                          u_dut.u_npu.gemm_dq_nbits_code,
                          u_dut.u_npu.u_gemm_ctrl.r_prec);
               dequant_width_check_count <= dequant_width_check_count + 1;
           end
       end
   end

    // ---- KV cache observability monitor (AC-07 / T-502) ----
    // Append capture: AW into K/V slot region arms capture of first W beat.
    // Read-back check: a later token's K-cache read burst from base 0xC000
    // must return slot s's appended word at beat s*8.
    always @(posedge clk) begin
        if (rst_n && decode_mode) begin
            // --- write channel ---
            if (u_dut.m_axi_awvalid && u_dut.m_axi_awready) begin
                automatic logic [20:0] a = u_dut.m_axi_awaddr[20:0];
                if (a >= 21'h0C000 && a < 21'h0D000) begin
                    wr_armed <= 1'b1; wr_is_v <= 1'b0;
                    wr_slot  <= (a - 21'h0C000) >> 8;
                end else if (a >= 21'h0E000 && a < 21'h0F000) begin
                    wr_armed <= 1'b1; wr_is_v <= 1'b1;
                    wr_slot  <= (a - 21'h0E000) >> 8;
                end
            end
            if (wr_armed && u_dut.m_axi_wvalid && u_dut.m_axi_wready) begin
                if (!wr_is_v) begin
                    kv_k_wdata[wr_slot[2:0]] <= u_dut.m_axi_wdata;
                    kv_k_wseen[wr_slot[2:0]] <= 1'b1;
                    $display("[KV] token %0d appended K slot %0d w0=%h",
                             cur_tok, wr_slot, u_dut.m_axi_wdata[63:0]);
                end else begin
                    kv_v_wdata[wr_slot[2:0]] <= u_dut.m_axi_wdata;
                    kv_v_wseen[wr_slot[2:0]] <= 1'b1;
                    $display("[KV] token %0d appended V slot %0d", cur_tok, wr_slot);
                end
                wr_armed <= 1'b0;
            end
            // --- read channel: K-cache burst from base 0xC000 ---
            if (u_dut.m_axi_arvalid && u_dut.m_axi_arready) begin
                if (u_dut.m_axi_araddr[20:0] == 21'h0C000) begin
                    rd_active <= 1'b1; rd_beat <= 0;
                end
            end
            if (rd_active && u_dut.m_axi_rvalid && u_dut.m_axi_rready) begin
                for (integer s = 0; s < 8; s++) begin
                    if (rd_beat == s*8 && kv_k_wseen[s] && s < cur_tok) begin
                        if (u_dut.m_axi_rdata === {256{1'bx}}) begin
                            // All-X word: unwritten and written-all-X are
                            // indistinguishable -> NOT evidence. Count aside.
                            $display("[KV] XONLY: token %0d slot %0d read is all-X (vacuous)", cur_tok, s);
                            kv_xonly <= kv_xonly + 1;
                        end else if (u_dut.m_axi_rdata === kv_k_wdata[s]) begin
                            $display("[KV] PASS: token %0d attention read observes token %0d K-append (beat %0d)",
                                     cur_tok, s, rd_beat);
                            kv_pass <= kv_pass + 1;
                            $fflush;
                        end else begin
                            $display("[KV] FAIL: token %0d read slot %0d got %h exp %h",
                                     cur_tok, s, u_dut.m_axi_rdata[63:0], kv_k_wdata[s][63:0]);
                            kv_fail <= kv_fail + 1;
                        end
                    end
                end
                if (u_dut.m_axi_rlast) rd_active <= 1'b0;
                rd_beat <= rd_beat + 1;
            end
        end
    end

    // Hard watchdog (30 seconds sim time = 6 billion cycles)
    initial begin
        #30s;
        $display("[TB] HARD WATCHDOG - killing sim after 30s");
        $finish;
    end

endmodule
