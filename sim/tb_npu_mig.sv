// =============================================================================
// tb_npu_mig.sv - NPU + MIG DDR4 Co-Simulation Testbench
// =============================================================================
// Integrates: NPU top + AXI-to-MIG bridge + MIG DDR4 IP + DDR4 memory model
// Pre-fills DDR4 with real weight/activation/scale data before NPU start.
// =============================================================================
`timescale 1ns/1ps

module tb_npu_mig;

    // Parameters
    parameter int CLK_PERIOD    = 5;    // 200 MHz NPU clock
    parameter int UCODE_DEPTH   = 262144;
    parameter int AXI_DATA_W    = 128;
    parameter int AXI_ADDR_W    = 32;
    parameter int AXI_ID_W      = 4;
    parameter int MIG_DATA_W    = 64;
    parameter int MIG_ADDR_W    = 29;
    parameter int MIG_MASK_W    = 8;

    // DDR4 model parameters (from MIG example)
    // Must match C0.DDR4_TimePeriod in the generated ddr4_0 XCI.
    localparam real    tCK = 750;   // MT40A512M8RH-075E, DDR4-2666
    localparam integer DQ_WIDTH = 8;
    localparam integer ADDR_WIDTH = 17;
    import arch_package::*;
    parameter UTYPE_density CONFIGURED_DENSITY = _4G;
    localparam integer RANK_WIDTH = 1;
    localparam integer NUM_PHYSICAL_PARTS = 1;

    // DDR4 command encodings
    localparam logic [3:0] MRS  = 4'b0000;
    localparam logic [3:0] REF   = 4'b0001;
    localparam logic [3:0] PRE   = 4'b0010;
    localparam logic [3:0] ACT   = 4'b0011;
    localparam logic [3:0] WR    = 4'b0100;
    localparam logic [3:0] RD    = 4'b0101;
    localparam logic [3:0] ZQC   = 4'b0110;
    localparam logic [3:0] NOP   = 4'b0111;

    // =========================================================================
    // AXI-Lite signals (TB → NPU)
    // =========================================================================
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

    // =========================================================================
    // AXI4 Master signals (NPU → bridge)
    // =========================================================================
    wire  [3:0]            m_axi_arid;
    wire  [31:0]           m_axi_araddr;
    wire  [7:0]            m_axi_arlen;
    wire  [2:0]            m_axi_arsize;
    wire  [1:0]            m_axi_arburst;
    wire                   m_axi_arvalid;
    wire                   m_axi_arready;
    wire  [3:0]            m_axi_rid;
    wire  [AXI_DATA_W-1:0] m_axi_rdata;
    wire  [1:0]            m_axi_rresp;
    wire                   m_axi_rlast;
    wire                   m_axi_rvalid;
    wire                   m_axi_rready;
    wire  [3:0]            m_axi_awid;
    wire  [31:0]           m_axi_awaddr;
    wire  [7:0]            m_axi_awlen;
    wire  [2:0]            m_axi_awsize;
    wire  [1:0]            m_axi_awburst;
    wire                   m_axi_awvalid;
    wire                   m_axi_awready;
    wire  [AXI_DATA_W-1:0] m_axi_wdata;
    wire  [AXI_DATA_W/8-1:0] m_axi_wstrb;
    wire                   m_axi_wlast;
    wire                   m_axi_wvalid;
    wire                   m_axi_wready;
    wire  [3:0]            m_axi_bid;
    wire  [1:0]            m_axi_bresp;
    wire                   m_axi_bvalid;
    wire                   m_axi_bready;

    // AXI stimulus used only by +MIG_SMOKE_ONLY.  The smoke test forces the
    // NPU-facing AXI nets to these values without starting an NPU program.
    logic [3:0]             smoke_axi_arid;
    logic [31:0]            smoke_axi_araddr;
    logic [7:0]             smoke_axi_arlen;
    logic [2:0]             smoke_axi_arsize;
    logic [1:0]             smoke_axi_arburst;
    logic                   smoke_axi_arvalid;
    logic                   smoke_axi_rready;
    logic [3:0]             smoke_axi_awid;
    logic [31:0]            smoke_axi_awaddr;
    logic [7:0]             smoke_axi_awlen;
    logic [2:0]             smoke_axi_awsize;
    logic [1:0]             smoke_axi_awburst;
    logic                   smoke_axi_awvalid;
    logic [AXI_DATA_W-1:0]  smoke_axi_wdata;
    logic [AXI_DATA_W/8-1:0] smoke_axi_wstrb;
    logic                   smoke_axi_wlast;
    logic                   smoke_axi_wvalid;
    logic                   smoke_axi_bready;

    // =========================================================================
    // MIG app interface — muxed between bridge and TB prefill
    // =========================================================================
    // Bridge outputs
    wire [MIG_ADDR_W-1:0]  br_app_addr;
    wire [2:0]             br_app_cmd;
    wire                   br_app_en;
    wire                   br_app_hi_pri;
    wire [MIG_DATA_W-1:0]  br_app_wdf_data;
    wire                   br_app_wdf_end;
    wire [MIG_MASK_W-1:0]  br_app_wdf_mask;
    wire                   br_app_wdf_wren;

    // TB prefill outputs
    logic [MIG_ADDR_W-1:0] tb_app_addr;
    logic [2:0]            tb_app_cmd;
    logic                  tb_app_en;
    logic                  tb_app_hi_pri;
    logic [MIG_DATA_W-1:0] tb_app_wdf_data;
    logic                  tb_app_wdf_end;
    logic [MIG_MASK_W-1:0] tb_app_wdf_mask;
    logic                  tb_app_wdf_wren;

    // Prefill mode control
    logic                  prefill_mode = 1'b0;

    // MIG inputs (muxed)
    wire [MIG_ADDR_W-1:0]  app_addr    = prefill_mode ? tb_app_addr    : br_app_addr;
    wire [2:0]             app_cmd     = prefill_mode ? tb_app_cmd     : br_app_cmd;
    wire                   app_en      = prefill_mode ? tb_app_en      : br_app_en;
    wire                   app_hi_pri  = prefill_mode ? tb_app_hi_pri  : br_app_hi_pri;
    wire [MIG_DATA_W-1:0]  app_wdf_data = prefill_mode ? tb_app_wdf_data : br_app_wdf_data;
    wire                   app_wdf_end = prefill_mode ? tb_app_wdf_end : br_app_wdf_end;
    wire [MIG_MASK_W-1:0]  app_wdf_mask = prefill_mode ? tb_app_wdf_mask : br_app_wdf_mask;
    wire                   app_wdf_wren = prefill_mode ? tb_app_wdf_wren : br_app_wdf_wren;

    // MIG outputs (shared, read-only by both bridge and TB)
    wire [MIG_DATA_W-1:0]  app_rd_data;
    wire                   app_rd_data_end;
    wire                   app_rd_data_valid;
    wire                   app_rdy;
    wire                   app_wdf_rdy;

    wire                   c0_init_calib_complete;
    wire                   c0_ddr4_ui_clk;
    wire                   c0_ddr4_ui_clk_sync_rst;
    wire                   dbg_clk;

    // NPU clock = MIG ui_clk (single clock domain — no CDC needed)
    wire clk;
    assign clk = c0_ddr4_ui_clk;

    // Lightweight AXI-to-MIG timing counter.  It is used only by the
    // +MIG_SMOKE_ONLY calibration path and does not participate in NPU
    // functional checking or in the full DDR image prefill.
    integer mig_timing_cycle;
    always @(posedge clk) begin
        if (c0_init_calib_complete)
            mig_timing_cycle <= mig_timing_cycle + 1;
        else
            mig_timing_cycle <= 0;
    end

    // Measure ui_clk frequency (after calibration)
    real calib_first_edge, calib_second_edge;
    initial begin
        wait(c0_init_calib_complete === 1'b1);
        @(posedge c0_ddr4_ui_clk);
        calib_first_edge = $realtime;
        @(posedge c0_ddr4_ui_clk);
        calib_second_edge = $realtime;
        $display("[TB] ui_clk period (post-calib) = %0.3f ns (%0.3f MHz)",
                 calib_second_edge - calib_first_edge,
                 1000.0 / (calib_second_edge - calib_first_edge));
    end

    // Sample acc_bank_sel at every gemm_done: the STORE of the GEMM that just
    // finished wrote into the bank that was ACTIVE before this edge (bank_sel
    // flips ON gemm_done), so the sampled value identifies the result bank.
    logic last_gemm_acc_bank;
    always @(posedge clk) begin
        if (u_npu.gemm_done)
            last_gemm_acc_bank <= u_npu.acc_bank_sel;
    end

    // =========================================================================
    // Optional instruction-latency event trace
    // =========================================================================
    // Enable with +INSTR_LATENCY_TRACE.  This monitor observes decode accepts
    // (not fetch attempts), so an ISSUE event marks the cycle an instruction
    // actually acquires its execution resource.  Completion is emitted from
    // the same done signals consumed by the NPU scoreboard.  The companion
    // Python parser pairs ISSUE/DONE by resource and produces CSV reports.
    //
    // The fetch PC is a best-effort source-PC tag.  For hardware-loop replay,
    // use the dynamic sequence number and raw instruction as the unique key:
    // the loop controller replays buffered instructions while fetch PC stays
    // at the loop body boundary.
    integer instr_trace_fd;
    integer instr_trace_seq;
    integer instr_trace_cycle;
    integer instr_trace_barrier_seq;
    integer instr_trace_barrier_start;
    logic   instr_trace_on;
    string  instr_trace_op;
    string  instr_trace_res;

    initial begin
        string instr_trace_file;
        instr_trace_on = $test$plusargs("INSTR_LATENCY_TRACE");
        instr_trace_seq = 0;
        instr_trace_cycle = 0;
        instr_trace_barrier_seq = -1;
        instr_trace_barrier_start = -1;
        if (instr_trace_on) begin
            if (!$value$plusargs("INSTR_TRACE_FILE=%s", instr_trace_file))
                instr_trace_file = "instruction_events.log";
            instr_trace_fd = $fopen(instr_trace_file, "w");
            if (instr_trace_fd == 0)
                $fatal(1, "Cannot open instruction trace file: %0s", instr_trace_file);
            $fwrite(instr_trace_fd, "# type|cycle|seq|fetch_pc|raw|opcode|resource\n");
        end
    end

    always @(posedge clk) begin
        if (instr_trace_on) begin
            instr_trace_cycle = instr_trace_cycle + 1;

            // Completions are recorded before new issues.  A resource is
            // single-issue by construction of the scoreboard, so its parser
            // queue has at most one active instruction in current RTL.
            if (u_npu.dma_rd_done)
                $fwrite(instr_trace_fd, "DONE|%0d|-1|-1|-|DONE|DMA_RD\n", instr_trace_cycle);
            if (u_npu.dma_wr_done)
                $fwrite(instr_trace_fd, "DONE|%0d|-1|-1|-|DONE|DMA_WR\n", instr_trace_cycle);
            if (u_npu.gemm_done)
                $fwrite(instr_trace_fd, "DONE|%0d|-1|-1|-|DONE|GEMM\n", instr_trace_cycle);
            if (u_npu.weight_rf_done)
                $fwrite(instr_trace_fd, "DONE|%0d|-1|-1|-|DONE|WRF\n", instr_trace_cycle);
            if (u_npu.act_rf_done)
                $fwrite(instr_trace_fd, "DONE|%0d|-1|-1|-|DONE|ARF\n", instr_trace_cycle);
            if (u_npu.postproc_done)
                $fwrite(instr_trace_fd, "DONE|%0d|-1|-1|-|DONE|POSTPROC\n", instr_trace_cycle);
            if (u_npu.topk_done_sig)
                $fwrite(instr_trace_fd, "DONE|%0d|-1|-1|-|DONE|TOPK\n", instr_trace_cycle);
            if (u_npu.sc_load_done)
                $fwrite(instr_trace_fd, "DONE|%0d|-1|-1|-|DONE|SCALE\n", instr_trace_cycle);

            if (instr_trace_barrier_seq >= 0 && u_npu.exec_all_idle) begin
                $fwrite(instr_trace_fd, "DONE|%0d|%0d|-1|-|DONE|BARRIER\n",
                        instr_trace_cycle, instr_trace_barrier_seq);
                instr_trace_barrier_seq = -1;
                instr_trace_barrier_start = -1;
            end

            if (u_npu.lc_dec_valid && u_npu.fetch_instr_ready) begin
                case (u_npu.lc_dec_instr[31:28])
                    4'h0: begin
                        case (u_npu.lc_dec_instr[15:12])
                            4'h1: begin instr_trace_op = "TOPK";         instr_trace_res = "TOPK";     end
                            4'h2: begin instr_trace_op = "SCALE";        instr_trace_res = "SCALE";    end
                            4'h5: begin instr_trace_op = "AC2V";         instr_trace_res = "POSTPROC"; end
                            4'h6: begin instr_trace_op = "ROUTE";        instr_trace_res = "TOPK";     end
                            4'h7: begin instr_trace_op = "RWEIGHT";      instr_trace_res = "POSTPROC"; end
                            4'hC: begin instr_trace_op = "TGLD";         instr_trace_res = "WRF";      end
                            4'hD: begin instr_trace_op = "BCAST";        instr_trace_res = "POSTPROC"; end
                            4'h3: begin instr_trace_op = "GROUP_WEIGHT"; instr_trace_res = "POSTPROC"; end
                            4'h4: begin instr_trace_op = "LOOP";         instr_trace_res = "NONE";     end
                            4'h8: begin instr_trace_op = "TOKEN_BEGIN";  instr_trace_res = "NONE";     end
                            4'h9: begin instr_trace_op = "TOKEN_END";    instr_trace_res = "NONE";     end
                            4'hA: begin instr_trace_op = "SET_WPAGE";    instr_trace_res = "NONE";     end
                            4'hB: begin instr_trace_op = "STATIC_SCALE"; instr_trace_res = "SCALE";    end
                            default: begin instr_trace_op = "NOP";       instr_trace_res = "NONE";     end
                        endcase
                    end
                    4'h1: begin instr_trace_op = "VLD";     instr_trace_res = "POSTPROC"; end
                    4'h2: begin instr_trace_op = "VST";     instr_trace_res = "POSTPROC"; end
                    4'h3: begin instr_trace_op = "GLD_W";   instr_trace_res = "WRF";      end
                    4'h4: begin instr_trace_op = "GLD_A";   instr_trace_res = "ARF";      end
                    4'h5: begin instr_trace_op = "GEMM";    instr_trace_res = "GEMM";     end
                    4'h6: begin instr_trace_op = "SOFTMAX"; instr_trace_res = "POSTPROC"; end
                    4'h7: begin instr_trace_op = "RMSNORM"; instr_trace_res = "POSTPROC"; end
                    4'h8: begin instr_trace_op = "ROPE";    instr_trace_res = "POSTPROC"; end
                    4'h9: begin instr_trace_op = "SILU";    instr_trace_res = "POSTPROC"; end
                    4'hA: begin instr_trace_op = "ADD";     instr_trace_res = "POSTPROC"; end
                    4'hB: begin instr_trace_op = "MUL";     instr_trace_res = "POSTPROC"; end
                    4'hC: begin instr_trace_op = "DMA_RD";  instr_trace_res = "DMA_RD";   end
                    4'hD: begin instr_trace_op = "DMA_WR";  instr_trace_res = "DMA_WR";   end
                    4'hE: begin instr_trace_op = "BARRIER"; instr_trace_res = "BARRIER";  end
                    default: begin instr_trace_op = "END";  instr_trace_res = "NONE";     end
                endcase

                $fwrite(instr_trace_fd, "ISSUE|%0d|%0d|%0d|%08x|%0s|%0s\n",
                        instr_trace_cycle, instr_trace_seq, u_npu.u_fetch.pc,
                        u_npu.lc_dec_instr, instr_trace_op, instr_trace_res);
                if (instr_trace_res == "NONE")
                    $fwrite(instr_trace_fd, "DONE|%0d|%0d|-1|-|DONE|NONE\n",
                            instr_trace_cycle, instr_trace_seq);
                else if (instr_trace_res == "BARRIER") begin
                    if (u_npu.exec_all_idle)
                        $fwrite(instr_trace_fd, "DONE|%0d|%0d|-1|-|DONE|BARRIER\n",
                                instr_trace_cycle, instr_trace_seq);
                    else begin
                        instr_trace_barrier_seq = instr_trace_seq;
                        instr_trace_barrier_start = instr_trace_cycle;
                    end
                end
                instr_trace_seq = instr_trace_seq + 1;
            end
        end
    end

    // MIG_EXPERT multi-N-tile: snapshot each N-tile's 32-lane acc result at
    // the gemm_done of its last K-tile (every exp_snap_ktiles-th gemm_done).
    // The ST_STORE phase completes before ST_DONE raises gemm_done, so the
    // acc_buffer bank content is stable at this edge; skip_store GEMMs never
    // touch acc_buffer, so a snapshot survives until the same bank is stored
    // again two tiles later. exp_snap_ktiles=0 disables the monitor.
    integer exp_snap_ktiles;
    integer exp_gd_cnt;
    logic [31:0] exp_tile_acc [0:7][0:31];
    integer mac_util_m, mac_util_n, mac_util_k;
    integer mac_util_active_cycles;
    integer mac_util_gemm_wall_cycles;
    integer mac_util_gld_overlap_cycles, mac_util_dma_overlap_cycles;
    integer mac_util_dequant_feed_overlap_cycles;
    integer group_dma_busy_cycles, group_gld_busy_cycles;
    integer group_gemm_busy_cycles, group_barrier_busy_cycles;
    integer group_fetch_wait_cycles;
    integer group_decode_stall_by_opcode [0:15];
    integer group_scale_stall_cycles, group_other_nop_stall_cycles;
    logic group_stall_monitor_enable;
    longint unsigned mac_util_useful_ops, mac_util_peak_ops;
    longint unsigned mac_util_wall_capacity_ops;
    longint unsigned mac_util_enabled_pe_slots;
    integer mac_util_gemm_count, mac_util_max_m;
    integer grouped_gemm_cmd_count;
    logic [3:0] grouped_last_gemm_expert;
    logic [1:0] grouped_last_gemm_proj;
    logic [15:0] grouped_last_gemm_k;
    integer group_spill_gld_a_count;
    integer group_extension_gld_a_count;
    integer group_explicit_k_dma_count;
    integer group_explicit_k_dma_overlap_cycles;
    logic group_explicit_k_dma_inflight;
    logic group_explicit_k_dma_seen_busy;
    logic expert_group_monitor_enable;
    logic [2:0] dequant_precision_seen;
    logic [15:0] group_dma_context_valid;
    logic [15:0] group_dma_context_addr [0:15];
    logic [15:0] group_dma_context_k [0:15];
    logic [3:0] group_dma_context_slot [0:15];
    logic [1:0] group_dma_context_proj [0:15];
    logic [5:0] group_dma_context_expert_iter [0:15];
    logic group_prev_dma_valid;
    logic [15:0] group_prev_dma_nt;
    logic [3:0] group_prev_dma_bank;
    logic [1:0] group_prev_dma_proj;
    logic [5:0] group_prev_dma_expert_iter;
    logic [31:0] group_dma_chunk_mask;
    logic [3:0] group_dma_chunk_count;
    logic group_weight_context_valid;
    logic [1:0] group_weight_context_proj;
    logic [5:0] group_weight_context_chunk;
    integer group_context_init_idx;
    logic deq_context_seen;
    logic deq_context_group_sel;
    logic [511:0] deq_scale_context, deq_zero_context;
    localparam integer GROUP_DEMO_DIM = 512;
    localparam integer GROUP_DEMO_INTERMEDIATE_DIM = 1024;
    localparam integer GROUP_DEMO_GATE_UP_K = 256;
    localparam integer GROUP_DEMO_DOWN_K = 256;
    localparam integer GROUP_DEMO_DOWN_TAIL_K = 256;
    localparam integer GROUP_DEMO_DOWN_CHUNK_COUNT = 4;
    localparam integer GROUP_DEMO_EXPECTED_K_OVERRIDE_COUNT =
        (GROUP_DEMO_DOWN_TAIL_K < GROUP_DEMO_DOWN_K)
            ? 4 * (GROUP_DEMO_DIM / 32) : 0;
    localparam integer GROUP_DEMO_GATE_UP_GEMMS =
        2 * (GROUP_DEMO_INTERMEDIATE_DIM / 32) *
        (GROUP_DEMO_DIM / GROUP_DEMO_GATE_UP_K);
    localparam integer GROUP_DEMO_DOWN_GEMMS =
        (GROUP_DEMO_DIM / 32) * GROUP_DEMO_DOWN_CHUNK_COUNT;
    localparam integer GROUP_DEMO_GEMMS_PER_EXPERT =
        GROUP_DEMO_GATE_UP_GEMMS + GROUP_DEMO_DOWN_GEMMS;
    localparam integer GROUP_DEMO_DEQUANT_OVERLAP_PER_EXPERT =
        GROUP_DEMO_GATE_UP_GEMMS * (GROUP_DEMO_GATE_UP_K - 1) +
        (GROUP_DEMO_DIM / 32) *
         (3 * (GROUP_DEMO_DOWN_K - 1) +
         (GROUP_DEMO_DOWN_TAIL_K - 1));
    localparam integer GROUP_DEMO_MAC_K_SUM_PER_EXPERT =
        GROUP_DEMO_GATE_UP_GEMMS * GROUP_DEMO_GATE_UP_K +
        (GROUP_DEMO_DIM / 32) *
        (3 * GROUP_DEMO_DOWN_K + GROUP_DEMO_DOWN_TAIL_K);
    localparam integer GROUP_DEMO_ACTIVATION_END =
        16'h2000 + (GROUP_DEMO_DIM/32)*2048;
    localparam integer GROUP_DEMO_PAGE1_CHUNKS =
        2;
    localparam integer GROUP_DEMO_SPILL_CHUNKS = 1;
    localparam integer GROUP_DEMO_EXTENSION_CHUNKS = 1;
    localparam integer GROUP_DEMO_SPILL_BYTES =
        32 * GROUP_DEMO_DOWN_K * 2;
    localparam integer GROUP_DEMO_EXTENSION_BYTES =
        32 * GROUP_DEMO_DOWN_K * 2;
    localparam logic [15:0] GROUP_DEMO_SPILL_BASE = 16'hA000;

    logic [7:0] group_golden_bytes [0:32*GROUP_DEMO_DIM*2-1];
    initial begin
        if (GROUP_DEMO_SPILL_BASE < GROUP_DEMO_ACTIVATION_END ||
            GROUP_DEMO_SPILL_BASE % 2048 != 0 ||
            GROUP_DEMO_SPILL_BASE + GROUP_DEMO_SPILL_BYTES > 65536 ||
            GROUP_DEMO_EXTENSION_BYTES > 16'h4000 ||
            (GROUP_DEMO_PAGE1_CHUNKS + GROUP_DEMO_SPILL_CHUNKS +
             GROUP_DEMO_EXTENSION_CHUNKS) !=
                GROUP_DEMO_DOWN_CHUNK_COUNT ||
            (4*GROUP_DEMO_DOWN_K) !=
                GROUP_DEMO_INTERMEDIATE_DIM)
            $fatal(1, "[TB][GROUPED_DEMO] L1 weight/spill layout is invalid or incomplete");
        exp_snap_ktiles = 0;
        exp_gd_cnt = 0;
        mac_util_m = 0;
        mac_util_n = 0;
        mac_util_k = 0;
        mac_util_active_cycles = 0;
        mac_util_gemm_wall_cycles = 0;
        mac_util_gld_overlap_cycles = 0;
        mac_util_dma_overlap_cycles = 0;
        mac_util_dequant_feed_overlap_cycles = 0;
        group_stall_monitor_enable = 1'b0;
        group_dma_busy_cycles = 0;
        group_gld_busy_cycles = 0;
        group_gemm_busy_cycles = 0;
        group_barrier_busy_cycles = 0;
        group_fetch_wait_cycles = 0;
        mac_util_useful_ops = 0;
        mac_util_peak_ops = 0;
        mac_util_wall_capacity_ops = 0;
        mac_util_enabled_pe_slots = 0;
        mac_util_gemm_count = 0;
        mac_util_max_m = 0;
        group_spill_gld_a_count = 0;
        group_extension_gld_a_count = 0;
        dequant_precision_seen = '0;
        group_dma_context_valid = '0;
        for (group_context_init_idx = 0; group_context_init_idx < 16;
             group_context_init_idx = group_context_init_idx + 1) begin
            group_dma_context_addr[group_context_init_idx] = '0;
            group_dma_context_slot[group_context_init_idx] = '0;
            group_dma_context_proj[group_context_init_idx] = '0;
            group_dma_context_expert_iter[group_context_init_idx] = '0;
        end
        group_prev_dma_valid = 1'b0;
        group_prev_dma_nt = '0;
        group_prev_dma_bank = '0;
        group_prev_dma_proj = '0;
        group_prev_dma_expert_iter = '0;
        group_dma_chunk_mask = '0;
        group_dma_chunk_count = '0;
        group_explicit_k_dma_count = 0;
        group_explicit_k_dma_overlap_cycles = 0;
        group_explicit_k_dma_inflight = 1'b0;
        group_explicit_k_dma_seen_busy = 1'b0;
        group_weight_context_valid = 1'b0;
        group_weight_context_proj = '0;
        group_weight_context_chunk = '0;
    end

    // A GEMM must consume stable scale/zero data within each selected group
    // from the same active ping-pong bank as its packed weights. A 256-K GEMM
    // legitimately switches once from scale group 0 to group 1 halfway through
    // K, so validate stability per group rather than across the whole gather.
    always @(posedge clk) begin : dequant_context_guard
        integer expected_scale_addr;
        logic [511:0] expected_scale_data;
        logic [511:0] expected_zero_data;
        if (!u_npu.rst_int_n) begin
            deq_context_seen <= 1'b0;
            deq_context_group_sel <= 1'b0;
            deq_scale_context <= '0;
            deq_zero_context <= '0;
        end else begin
            if (u_npu.gemm_cmd_accept)
                deq_context_seen <= 1'b0;
            if (u_npu.dq_in_valid && u_npu.dq_in_ready) begin
                if (u_npu.dq_sz_ready !== 1'b1 ||
                    u_npu.u_scale_rd_ctrl.ready[u_npu.wrf_bank_sel] !== 1'b1 ||
                    u_npu.u_scale_rd_ctrl.rd_bank_sel !== u_npu.wrf_bank_sel)
                    $fatal(1, "[TB][DEQUANT] active WRF bank has no matching ready scale/zero context");
                if (expert_group_monitor_enable &&
                    u_npu.gemm_cmd_k == 16'd256) begin
                    if (u_npu.u_scale_rd_ctrl.read_count != 4'd8 ||
                        u_npu.gemm_dq_group_sel !==
                            u_npu.u_gemm_ctrl.r_gather_k[7])
                        $fatal(1, "[TB][DEQUANT] 256-K scale group geometry mismatch: beats=%0d k=%0d group=%0d",
                               u_npu.u_scale_rd_ctrl.read_count,
                               u_npu.u_gemm_ctrl.r_gather_k,
                               u_npu.gemm_dq_group_sel);
                    expected_scale_addr = u_npu.u_scale_rd_ctrl.base +
                        (u_npu.gemm_dq_group_sel ? 4 : 0);
                    expected_scale_data = {
                        u_npu.u_scale_zero_mem.mem[expected_scale_addr+1],
                        u_npu.u_scale_zero_mem.mem[expected_scale_addr]};
                    expected_zero_data = {
                        u_npu.u_scale_zero_mem.mem[expected_scale_addr+3],
                        u_npu.u_scale_zero_mem.mem[expected_scale_addr+2]};
                    if (u_npu.dq_scale_data !== expected_scale_data ||
                        u_npu.dq_zero_data !== expected_zero_data)
                        $fatal(1, "[TB][DEQUANT] 256-K scale/zero group %0d does not match DDR-loaded scale SRAM at beat %0d",
                               u_npu.gemm_dq_group_sel, expected_scale_addr);
                end
                if (deq_context_seen &&
                    u_npu.gemm_dq_group_sel == deq_context_group_sel &&
                    (u_npu.dq_scale_data !== deq_scale_context ||
                     u_npu.dq_zero_data !== deq_zero_context))
                    $fatal(1, "[TB][DEQUANT] scale/zero context changed within one GEMM scale group");
                if (!deq_context_seen ||
                    u_npu.gemm_dq_group_sel != deq_context_group_sel) begin
                    deq_context_group_sel <= u_npu.gemm_dq_group_sel;
                    deq_scale_context <= u_npu.dq_scale_data;
                    deq_zero_context <= u_npu.dq_zero_data;
                    deq_context_seen <= 1'b1;
                end
            end
        end
    end

    // A completed prefetch may wait for the in-flight GEMM to retire before
    // its bank is published. The producer resource must remain held during
    // that window so a following GLD cannot overwrite the pending bank.
    always @(posedge clk) begin
        if (u_npu.rst_int_n) begin
            if (u_npu.wrf_pending_flip && !u_npu.gemm_done &&
                u_npu.weight_rf_done)
                $fatal(1, "[TB][PINGPONG] WRF write resource released before pending bank flip");
            if (u_npu.arf_pending_flip && !u_npu.gemm_done &&
                u_npu.act_rf_done)
                $fatal(1, "[TB][PINGPONG] ARF write resource released before pending bank flip");
            if (u_npu.wrf_pending_flip && u_npu.gld_w_cmd_valid)
                $fatal(1, "[TB][PINGPONG] GLD.W issued while prior WRF bank awaits publication");
            if (u_npu.arf_pending_flip && u_npu.gld_a_cmd_valid)
                $fatal(1, "[TB][PINGPONG] GLD.A issued while prior ARF bank awaits publication");
        end
    end

    logic pingpong_gemm_context_valid;
    logic pingpong_wrf_bank_at_start, pingpong_arf_bank_at_start;
    always @(posedge clk or negedge u_npu.rst_int_n) begin
        if (!u_npu.rst_int_n) begin
            pingpong_gemm_context_valid <= 1'b0;
            pingpong_wrf_bank_at_start <= 1'b0;
            pingpong_arf_bank_at_start <= 1'b0;
        end else begin
            if (u_npu.gemm_cmd_accept && u_npu.u_gemm_ctrl.cmd_ready) begin
                pingpong_gemm_context_valid <= 1'b1;
                pingpong_wrf_bank_at_start <= u_npu.wrf_bank_sel;
                pingpong_arf_bank_at_start <= u_npu.arf_bank_sel;
            end
            if (pingpong_gemm_context_valid && u_npu.u_gemm_ctrl.busy &&
                (u_npu.wrf_bank_sel !== pingpong_wrf_bank_at_start ||
                 u_npu.arf_bank_sel !== pingpong_arf_bank_at_start))
                $fatal(1, "[TB][PINGPONG] active RF bank changed during GEMM");
            if (u_npu.gemm_done)
                pingpong_gemm_context_valid <= 1'b0;
        end
    end

    // Check every explicit K-chunk DMA -> GLD.W -> GLD.A handoff. These
    // instructions are no longer driven by a generic K-loop, so derive the
    // chunk index from the descriptor-relative DMA tile address instead of
    // loop_iter. This also audits the alternating L1 staging banks.
    always @(posedge clk) begin : grouped_chunk_prefetch_guard
        integer k_tiles;
        integer k_block;
        integer k_start;
        integer ntile;
        integer expected_group;
        integer expected_chunk;
        integer expected_addr;
        integer expected_chunks;
        integer chunk_idx;
        integer ctx_bank;
        integer ctx_reset_idx;
        logic same_group;
        logic [1:0] expected_page;

        if (!u_npu.rst_int_n) begin
            group_dma_context_valid <= '0;
            for (ctx_reset_idx = 0; ctx_reset_idx < 16;
                 ctx_reset_idx = ctx_reset_idx + 1) begin
                group_dma_context_addr[ctx_reset_idx] <= '0;
                group_dma_context_k[ctx_reset_idx] <= '0;
                group_dma_context_slot[ctx_reset_idx] <= '0;
                group_dma_context_proj[ctx_reset_idx] <= '0;
                group_dma_context_expert_iter[ctx_reset_idx] <= '0;
            end
            group_prev_dma_valid <= 1'b0;
            group_prev_dma_nt <= '0;
            group_prev_dma_bank <= '0;
            group_prev_dma_proj <= '0;
            group_prev_dma_expert_iter <= '0;
            group_dma_chunk_mask <= '0;
            group_dma_chunk_count <= '0;
            group_explicit_k_dma_count <= 0;
            group_weight_context_valid <= 1'b0;
            group_weight_context_proj <= '0;
            group_weight_context_chunk <= '0;
        end else if (expert_group_monitor_enable) begin
            if (u_npu.dma_rd_cmd_valid &&
                u_npu.dma_rd_cmd_expert_sel != 4'd0 &&
                !u_npu.dma_rd_cmd_group_gather) begin
                k_block = (u_npu.dma_rd_cmd_proj == 2'd2)
                        ? (u_npu.dma_rd_cmd_k_override
                           ? u_npu.dma_rd_cmd_transfer_k
                           : u_npu.cfg_k_active)
                        : GROUP_DEMO_GATE_UP_K;
                k_tiles = (u_npu.dma_rd_cmd_proj == 2'd2)
                        ? GROUP_DEMO_INTERMEDIATE_DIM / 32
                        : GROUP_DEMO_DIM / 32;
                k_start = u_npu.dma_rd_cmd_addr % k_tiles;
                ntile = u_npu.dma_rd_cmd_addr / k_tiles;
                chunk_idx = (u_npu.dma_rd_cmd_proj == 2'd2)
                          ? k_start / (GROUP_DEMO_DOWN_K / 32)
                          : k_start / (k_block / 32);
                expected_chunks = (u_npu.dma_rd_cmd_proj == 2'd2)
                                ? GROUP_DEMO_DOWN_CHUNK_COUNT
                                : k_tiles / (k_block / 32);
                if (u_npu.loop_active || u_npu.dma_rd_cmd_expert_sel != 4'd1 ||
                    u_npu.dma_rd_cmd_proj > 2'd2 ||
                    u_npu.dma_rd_cmd_vd > 4'd1 ||
                    (u_npu.dma_rd_cmd_proj != 2'd2 &&
                     u_npu.dma_rd_cmd_k_override) ||
                    (k_block != 128 && k_block != 256) ||
                    (u_npu.dma_rd_cmd_proj == 2'd2 &&
                     ((k_start % (GROUP_DEMO_DOWN_K / 32)) != 0 ||
                      k_block != GROUP_DEMO_DOWN_K)) ||
                    (k_start % (k_block / 32)) != 0 ||
                    chunk_idx >= expected_chunks)
                    $fatal(1, "[TB][GROUPED_DEMO] invalid explicit K-DMA addr=%0d expert=%0d proj=%0d bank=%0d loop=%0d",
                           u_npu.dma_rd_cmd_addr,
                           u_npu.dma_rd_cmd_expert_sel,
                           u_npu.dma_rd_cmd_proj, u_npu.dma_rd_cmd_vd,
                           u_npu.loop_active);
                if (u_npu.dma_rd_cmd_k_override) begin
                    if (u_npu.dma_rd_cmd_proj != 2'd2 ||
                        u_npu.dma_rd_cmd_transfer_k == u_npu.cfg_k_active)
                        $fatal(1, "[TB][GROUPED_DEMO] K override must prefetch a differently sized Down chunk");
                    group_explicit_k_dma_count <= group_explicit_k_dma_count + 1;
                end
                same_group = group_prev_dma_valid &&
                    group_prev_dma_expert_iter == u_npu.expert_loop_iter &&
                    group_prev_dma_proj == u_npu.dma_rd_cmd_proj &&
                    group_prev_dma_nt == ntile;
                if (!same_group && group_prev_dma_valid) begin
                    k_tiles = (group_prev_dma_proj == 2'd2)
                            ? GROUP_DEMO_INTERMEDIATE_DIM / 32
                            : GROUP_DEMO_DIM / 32;
                    k_block = (group_prev_dma_proj == 2'd2)
                            ? GROUP_DEMO_DOWN_K : GROUP_DEMO_GATE_UP_K;
                    expected_chunks = (group_prev_dma_proj == 2'd2)
                                    ? GROUP_DEMO_DOWN_CHUNK_COUNT
                                    : k_tiles / (k_block / 32);
                    if (group_dma_chunk_count != expected_chunks)
                        $fatal(1, "[TB][GROUPED_DEMO] K chunks missing at expert=%0d proj=%0d nt=%0d count=%0d expected=%0d",
                               group_prev_dma_expert_iter, group_prev_dma_proj,
                               group_prev_dma_nt, group_dma_chunk_count,
                               expected_chunks);
                end
                if (!same_group && u_npu.dma_rd_cmd_vd !=
                    (u_npu.dma_rd_cmd_proj == 2'd2
                        ? (4'd1 ^ ((ntile * GROUP_DEMO_DOWN_CHUNK_COUNT) & 1))
                        : 4'd1))
                    $fatal(1, "[TB][GROUPED_DEMO] first K-DMA for expert=%0d proj=%0d nt=%0d selected the wrong ping-pong bank: %0d",
                           u_npu.expert_loop_iter, u_npu.dma_rd_cmd_proj,
                           ntile, u_npu.dma_rd_cmd_vd);
                if (group_prev_dma_valid &&
                    group_prev_dma_expert_iter == u_npu.expert_loop_iter &&
                    group_prev_dma_proj == u_npu.dma_rd_cmd_proj &&
                    group_prev_dma_nt == ntile &&
                    group_prev_dma_bank != (u_npu.dma_rd_cmd_vd ^ 4'd1))
                    $fatal(1, "[TB][GROUPED_DEMO] DMA banks did not ping-pong at expert=%0d proj=%0d nt=%0d",
                           u_npu.expert_loop_iter, u_npu.dma_rd_cmd_proj,
                           ntile);
                if (same_group) begin
                    if (group_dma_chunk_mask[chunk_idx])
                        $fatal(1, "[TB][GROUPED_DEMO] duplicate K chunk %0d at expert=%0d proj=%0d nt=%0d",
                               chunk_idx, u_npu.expert_loop_iter,
                               u_npu.dma_rd_cmd_proj, ntile);
                    group_dma_chunk_mask <= group_dma_chunk_mask |
                                            (32'b1 << chunk_idx);
                    group_dma_chunk_count <= group_dma_chunk_count + 1'b1;
                end else begin
                    group_dma_chunk_mask <= 32'b1 << chunk_idx;
                    group_dma_chunk_count <= 4'd1;
                end
                group_prev_dma_valid <= 1'b1;
                group_prev_dma_nt <= ntile;
                group_prev_dma_bank <= u_npu.dma_rd_cmd_vd;
                group_prev_dma_proj <= u_npu.dma_rd_cmd_proj;
                group_prev_dma_expert_iter <= u_npu.expert_loop_iter;
                group_dma_context_valid[u_npu.dma_rd_cmd_vd] <= 1'b1;
                group_dma_context_addr[u_npu.dma_rd_cmd_vd] <=
                    u_npu.dma_rd_cmd_addr;
                group_dma_context_k[u_npu.dma_rd_cmd_vd] <=
                    u_npu.dma_rd_cmd_k_override
                        ? u_npu.dma_rd_cmd_transfer_k : u_npu.cfg_k_active;
                group_dma_context_slot[u_npu.dma_rd_cmd_vd] <=
                    u_npu.dma_rd_cmd_expert_sel;
                group_dma_context_proj[u_npu.dma_rd_cmd_vd] <=
                    u_npu.dma_rd_cmd_proj;
                group_dma_context_expert_iter[u_npu.dma_rd_cmd_vd] <=
                    u_npu.expert_loop_iter;
            end

            if (u_npu.gld_w_cmd_valid &&
                u_npu.gld_w_cmd_expert_sel != 4'd0) begin
                ctx_bank = u_npu.gld_w_cmd_addr[15:12];
                k_tiles = (group_dma_context_proj[ctx_bank] == 2'd2)
                        ? GROUP_DEMO_INTERMEDIATE_DIM / 32
                        : GROUP_DEMO_DIM / 32;
                k_block = (group_dma_context_proj[ctx_bank] == 2'd2)
                        ? group_dma_context_k[ctx_bank]
                        : GROUP_DEMO_GATE_UP_K;
                k_start = group_dma_context_addr[ctx_bank] % k_tiles;
                expected_group = k_start / 4; // one scale group per 128 K values
                if (!group_dma_context_valid[ctx_bank] ||
                    group_dma_context_slot[ctx_bank] !=
                        u_npu.gld_w_cmd_expert_sel ||
                    group_dma_context_proj[ctx_bank] !=
                        u_npu.gld_w_cmd_proj ||
                    group_dma_context_expert_iter[ctx_bank] !=
                        u_npu.expert_loop_iter ||
                    u_npu.cfg_k_active != k_block ||
                    expected_group != u_npu.gld_w_cmd_group)
                    $fatal(1, "[TB][GROUPED_DEMO] GLD.W mismatches DMA context: ddr_tile=%0d bank=%0d proj=%0d scale_group=%0d",
                           group_dma_context_addr[ctx_bank], ctx_bank,
                           u_npu.gld_w_cmd_proj, u_npu.gld_w_cmd_group);
                group_weight_context_valid <= 1'b1;
                group_dma_context_valid[ctx_bank] <= 1'b0;
                group_weight_context_proj <= group_dma_context_proj[ctx_bank];
                group_weight_context_chunk <=
                    (group_dma_context_proj[ctx_bank] == 2'd2)
                        ? k_start : k_start / (k_block / 32);
            end

            if (u_npu.gld_a_cmd_valid && u_npu.gld_a_cmd_transpose) begin
                if (!group_weight_context_valid)
                    $fatal(1, "[TB][GROUPED_DEMO] GLD.A has no matching weight K-chunk context");
                expected_chunk = group_weight_context_chunk;
                if (group_weight_context_proj == 2'd2) begin
                    if (expected_chunk < 16) begin
                        expected_page = 2'd1;
                        expected_addr = 16'h8000 +
                            (expected_chunk / (GROUP_DEMO_DOWN_K / 32)) *
                            (32*GROUP_DEMO_DOWN_K*2);
                    end else if (expected_chunk < 24) begin
                        expected_page = 2'd0;
                        expected_addr = GROUP_DEMO_SPILL_BASE +
                            ((expected_chunk - 16) /
                             (GROUP_DEMO_DOWN_K / 32)) *
                            (32*GROUP_DEMO_DOWN_K*2);
                    end else begin
                        expected_page = 2'd2;
                        expected_addr = ((expected_chunk - 24) /
                                         (GROUP_DEMO_DOWN_K / 32)) *
                                        (32*GROUP_DEMO_DOWN_K*2);
                    end
                end else begin
                    expected_page = 2'd0;
                    expected_addr = 16'h2000 + expected_chunk *
                                    (32*GROUP_DEMO_GATE_UP_K*2);
                end
                if (u_npu.gld_a_cmd_page !== expected_page ||
                    u_npu.gld_a_cmd_addr !== 16'(expected_addr))
                    $fatal(1, "[TB][GROUPED_DEMO] GLD.A K-chunk mismatch: proj=%0d chunk=%0d page=%0d addr=%h expected_page=%0d expected_addr=%h",
                           group_weight_context_proj, expected_chunk,
                           u_npu.gld_a_cmd_page, u_npu.gld_a_cmd_addr,
                           expected_page, expected_addr);
            end
        end
    end

    // Count only cycles where an explicitly sized mixed-K DMA is active at
    // the same time as a GEMM. This separates real overlap from mere issue.
    always @(posedge clk) begin
        if (!u_npu.rst_int_n) begin
            group_explicit_k_dma_inflight <= 1'b0;
            group_explicit_k_dma_seen_busy <= 1'b0;
            group_explicit_k_dma_overlap_cycles <= 0;
        end else if (expert_group_monitor_enable) begin
            if (u_npu.dma_rd_cmd_valid && u_npu.dma_rd_cmd_k_override) begin
                group_explicit_k_dma_inflight <= 1'b1;
                group_explicit_k_dma_seen_busy <= 1'b0;
            end else if (group_explicit_k_dma_inflight) begin
                if (u_npu.dma_rd_busy) begin
                    group_explicit_k_dma_seen_busy <= 1'b1;
                    if (u_npu.u_gemm_ctrl.busy)
                        group_explicit_k_dma_overlap_cycles <=
                            group_explicit_k_dma_overlap_cycles + 1;
                end else if (group_explicit_k_dma_seen_busy) begin
                    group_explicit_k_dma_inflight <= 1'b0;
                    group_explicit_k_dma_seen_busy <= 1'b0;
                end
            end
        end
    end

    // The 512x1024 grouped demo stores K chunks 0-1 in page 1, chunk 2 in the
    // page-0 spill at 0xA000, and chunk 3 in the 16-KiB page-2 extension.
    // Alternating Down N tiles keep each next tile's first chunk resident.
    always @(posedge clk) begin
        if (!u_npu.rst_int_n) begin
            group_spill_gld_a_count <= 0;
            group_extension_gld_a_count <= 0;
        end else if (expert_group_monitor_enable && u_npu.gld_a_cmd_valid &&
                     u_npu.gld_a_cmd_transpose &&
                     u_npu.gld_a_cmd_page == 2'd0 &&
                     u_npu.gld_a_cmd_addr >= GROUP_DEMO_SPILL_BASE &&
                     u_npu.gld_a_cmd_addr < GROUP_DEMO_SPILL_BASE +
                                              GROUP_DEMO_SPILL_BYTES) begin
            if (((u_npu.gld_a_cmd_addr - GROUP_DEMO_SPILL_BASE) %
                 (32*GROUP_DEMO_DOWN_K*2)) != 0)
                $fatal(1, "[TB][GROUPED_DEMO] spill activation address is not K-chunk aligned: %h",
                       u_npu.gld_a_cmd_addr);
            group_spill_gld_a_count <= group_spill_gld_a_count + 1;
        end else if (expert_group_monitor_enable && u_npu.gld_a_cmd_valid &&
                     u_npu.gld_a_cmd_transpose &&
                     u_npu.gld_a_cmd_page == 2'd2) begin
            if ((u_npu.gld_a_cmd_addr % (32*GROUP_DEMO_DOWN_K*2)) != 0 ||
                u_npu.gld_a_cmd_addr + GROUP_DEMO_EXTENSION_BYTES > 16'h4000)
                $fatal(1, "[TB][GROUPED_DEMO] extension activation address is invalid: %h",
                       u_npu.gld_a_cmd_addr);
            group_extension_gld_a_count <= group_extension_gld_a_count + 1;
        end
    end

    // Report compute-phase array utilization and end-to-end GEMM occupancy.
    // The latter includes setup, weight dequant/gather, feed/drain and store,
    // exposing time when the array cannot perform useful MACs.
    always @(posedge clk) begin
        if (!u_npu.rst_int_n) begin
            mac_util_m <= 0;
            mac_util_n <= 0;
            mac_util_k <= 0;
            mac_util_active_cycles <= 0;
            mac_util_gemm_wall_cycles <= 0;
            mac_util_gld_overlap_cycles <= 0;
            mac_util_dma_overlap_cycles <= 0;
            mac_util_dequant_feed_overlap_cycles <= 0;
            group_dma_busy_cycles <= 0;
            group_gld_busy_cycles <= 0;
            group_gemm_busy_cycles <= 0;
            group_barrier_busy_cycles <= 0;
            group_fetch_wait_cycles <= 0;
            group_scale_stall_cycles <= 0;
            group_other_nop_stall_cycles <= 0;
            for (integer op = 0; op < 16; op = op + 1)
                group_decode_stall_by_opcode[op] <= 0;
            mac_util_useful_ops <= 0;
            mac_util_peak_ops <= 0;
            mac_util_wall_capacity_ops <= 0;
            mac_util_enabled_pe_slots <= 0;
            mac_util_gemm_count <= 0;
            mac_util_max_m <= 0;
            grouped_gemm_cmd_count <= 0;
            grouped_last_gemm_expert <= '0;
            grouped_last_gemm_proj <= '0;
            grouped_last_gemm_k <= '0;
            dequant_precision_seen <= '0;
        end else begin
            if (group_stall_monitor_enable) begin
                if (u_npu.dma_rd_busy)
                    group_dma_busy_cycles <= group_dma_busy_cycles + 1;
                if (u_npu.gld_busy)
                    group_gld_busy_cycles <= group_gld_busy_cycles + 1;
                if (u_npu.u_gemm_ctrl.busy)
                    group_gemm_busy_cycles <= group_gemm_busy_cycles + 1;
                if (u_npu.u_decode.barrier_active)
                    group_barrier_busy_cycles <= group_barrier_busy_cycles + 1;
                if (u_npu.lc_dec_valid &&
                    !u_npu.fetch_instr_ready) begin
                    group_fetch_wait_cycles <= group_fetch_wait_cycles + 1;
                    group_decode_stall_by_opcode[
                        u_npu.lc_dec_instr[31:28]] <=
                        group_decode_stall_by_opcode[
                            u_npu.lc_dec_instr[31:28]] + 1;
                    if (u_npu.lc_dec_instr[31:28] == 4'h0 &&
                        u_npu.lc_dec_instr[15:12] == 4'h2)
                        group_scale_stall_cycles <=
                            group_scale_stall_cycles + 1;
                    else if (u_npu.lc_dec_instr[31:28] == 4'h0)
                        group_other_nop_stall_cycles <=
                            group_other_nop_stall_cycles + 1;
                end
            end
            if (u_npu.weight_rf_done && !u_npu.tgld_done &&
                !u_npu.wrf_scale_ready_inactive)
                $fatal(1, "[TB][WRF_PUBLISH] weight bank released before its scale/zero metadata is ready");
            if (expert_group_monitor_enable && u_npu.dma_rd_busy &&
                u_npu.dma_rd_l1_bank_valid && u_npu.gld_w_cmd_valid &&
                u_npu.gld_w_cmd_addr[15:12] == u_npu.dma_rd_l1_bank)
                $fatal(1, "[TB][L1_BANK_HAZARD] GLD.W read bank %0d while expert DMA writes it",
                       u_npu.dma_rd_l1_bank);
            if (expert_group_monitor_enable && u_npu.dq_in_valid &&
                u_npu.dq_in_ready) begin
                if ($isunknown({u_npu.dq_in_data,
                                u_npu.dq_scale_data,
                                u_npu.dq_zero_data}))
                    $fatal(1, "[TB][GROUPED_X_DQ_INPUT] unknown weight/quant input: gK=%0d rf_addr=%0d nbits=%0d data=%h scale=%h zero=%h",
                           u_npu.u_gemm_ctrl.r_gather_k,
                           u_npu.wrf_rd_addr,
                           u_npu.gemm_dq_nbits_code, u_npu.dq_in_data,
                           u_npu.dq_scale_data, u_npu.dq_zero_data);
                for (int row = 0; row < 32; row++) begin
                    if ((u_npu.u_gemm_ctrl.r_act_batch &&
                         row < u_npu.u_gemm_ctrl.r_M) ||
                        (!u_npu.u_gemm_ctrl.r_act_batch && row == 0)) begin
                        if ($isunknown(u_npu.arf_rd_data[row*16 +: 16]))
                            $fatal(1, "[TB][GROUPED_X_DQ_INPUT] unknown active activation row: gK=%0d row=%0d M=%0d batch=%b arf_addr=%0d act=%h",
                                   u_npu.u_gemm_ctrl.r_gather_k, row,
                                   u_npu.u_gemm_ctrl.r_M,
                                   u_npu.u_gemm_ctrl.r_act_batch,
                                   u_npu.arf_rd_addr, u_npu.arf_rd_data);
                    end
                end
            end
            if (expert_group_monitor_enable && u_npu.dq_out_valid &&
                u_npu.u_gemm_ctrl.int_gather &&
                $isunknown({u_npu.dq_out_data,
                            u_npu.u_gemm_ctrl.act_d2}))
                $fatal(1, "[TB][GROUPED_X_DQ_COMMIT] unknown dequant/activation commit: commit_k=%0d gather_k=%0d dequant=%h act_d2=%h",
                       u_npu.u_gemm_ctrl.r_commit_k,
                       u_npu.u_gemm_ctrl.r_gather_k,
                       u_npu.dq_out_data, u_npu.u_gemm_ctrl.act_d2);
            if (expert_group_monitor_enable &&
                u_npu.u_gemm_ctrl.sa_compute_active) begin
                for (int lane = 0; lane < 32; lane++) begin
                    if ($isunknown({u_npu.sa_a_col[lane],
                                    u_npu.sa_b_row[lane]}))
                        $fatal(1, "[TB][GROUPED_X_SA_FEED] unknown systolic input: state=%0d feed_k=%0d lane=%0d a=%h b=%h",
                               u_npu.u_gemm_ctrl.state,
                               u_npu.u_gemm_ctrl.r_feed_k, lane,
                               u_npu.sa_a_col[lane],
                               u_npu.sa_b_row[lane]);
                end
                for (int pe_row = 0; pe_row < 32; pe_row++) begin
                    for (int pe_col = 0; pe_col < 32; pe_col++) begin
                        if ($isunknown(u_npu.sa_acc[pe_row][pe_col]))
                            $fatal(1, "[TB][GROUPED_X_SA_ACCUM] unknown PE accumulator: command=%0d expert=%0d proj=%0d cmd_k=%0d state=%0d feed_k=%0d row=%0d col=%0d a=%h b=%h acc=%h accumulate=%b clear/en=%b/%b",
                                   grouped_gemm_cmd_count,
                                   grouped_last_gemm_expert,
                                   grouped_last_gemm_proj,
                                   grouped_last_gemm_k,
                                   u_npu.u_gemm_ctrl.state,
                                   u_npu.u_gemm_ctrl.r_feed_k,
                                   pe_row, pe_col,
                                   u_npu.sa_a_col[pe_row],
                                   u_npu.sa_b_row[pe_col],
                                   u_npu.sa_acc[pe_row][pe_col],
                                   u_npu.u_gemm_ctrl.r_acc_accumulate,
                                   u_npu.sa_clear, u_npu.sa_en);
                    end
                end
            end
            if (expert_group_monitor_enable && u_npu.acc_wr_en &&
                $isunknown({u_npu.acc_wr_data0,  u_npu.acc_wr_data1,
                            u_npu.acc_wr_data2,  u_npu.acc_wr_data3,
                            u_npu.acc_wr_data4,  u_npu.acc_wr_data5,
                            u_npu.acc_wr_data6,  u_npu.acc_wr_data7,
                            u_npu.acc_wr_data8,  u_npu.acc_wr_data9,
                            u_npu.acc_wr_data10, u_npu.acc_wr_data11,
                            u_npu.acc_wr_data12, u_npu.acc_wr_data13,
                            u_npu.acc_wr_data14, u_npu.acc_wr_data15,
                            u_npu.acc_wr_data16, u_npu.acc_wr_data17,
                            u_npu.acc_wr_data18, u_npu.acc_wr_data19,
                            u_npu.acc_wr_data20, u_npu.acc_wr_data21,
                            u_npu.acc_wr_data22, u_npu.acc_wr_data23,
                            u_npu.acc_wr_data24, u_npu.acc_wr_data25,
                            u_npu.acc_wr_data26, u_npu.acc_wr_data27,
                            u_npu.acc_wr_data28, u_npu.acc_wr_data29,
                            u_npu.acc_wr_data30, u_npu.acc_wr_data31}))
                $fatal(1, "[TB][GROUPED_X_GEMM_STORE] GEMM stored X: row=%0d acc_bank=%0d command=%0d expert=%0d proj=%0d cmd_k=%0d latched_m/n/k=%0d/%0d/%0d prec=%0d accumulate=%b active_rows=%0d sa_clear/en=%b/%b feed_k/max=%0d/%0d gather/commit=%0d/%0d data[31:0]=%h",
                       u_npu.acc_wr_addr, u_npu.acc_bank_sel,
                       grouped_gemm_cmd_count, grouped_last_gemm_expert,
                       grouped_last_gemm_proj,
                       grouped_last_gemm_k,
                       u_npu.u_gemm_ctrl.r_M, u_npu.u_gemm_ctrl.r_N,
                       u_npu.u_gemm_ctrl.r_K, u_npu.u_gemm_ctrl.r_prec,
                       u_npu.u_gemm_ctrl.r_acc_accumulate,
                       u_npu.sa_active_rows, u_npu.sa_clear, u_npu.sa_en,
                       u_npu.u_gemm_ctrl.r_feed_k,
                       u_npu.u_gemm_ctrl.r_feed_max,
                       u_npu.u_gemm_ctrl.r_gather_k,
                       u_npu.u_gemm_ctrl.r_commit_k,
                       {u_npu.acc_wr_data31, u_npu.acc_wr_data30,
                        u_npu.acc_wr_data29, u_npu.acc_wr_data28,
                        u_npu.acc_wr_data27, u_npu.acc_wr_data26,
                        u_npu.acc_wr_data25, u_npu.acc_wr_data24,
                        u_npu.acc_wr_data23, u_npu.acc_wr_data22,
                        u_npu.acc_wr_data21, u_npu.acc_wr_data20,
                        u_npu.acc_wr_data19, u_npu.acc_wr_data18,
                        u_npu.acc_wr_data17, u_npu.acc_wr_data16,
                        u_npu.acc_wr_data15, u_npu.acc_wr_data14,
                        u_npu.acc_wr_data13, u_npu.acc_wr_data12,
                        u_npu.acc_wr_data11, u_npu.acc_wr_data10,
                        u_npu.acc_wr_data9,  u_npu.acc_wr_data8,
                        u_npu.acc_wr_data7,  u_npu.acc_wr_data6,
                        u_npu.acc_wr_data5,  u_npu.acc_wr_data4,
                        u_npu.acc_wr_data3,  u_npu.acc_wr_data2,
                        u_npu.acc_wr_data1,  u_npu.acc_wr_data0});
            if (expert_group_monitor_enable && u_npu.ac2v_prf_wr_en &&
                $isunknown(u_npu.ac2v_prf_wr_data))
                $fatal(1, "[TB][GROUPED_X_AC2V] accumulator drain emitted X: row=%0d set=%0d vd=%0d fp32_lane0=%h fp16_lane0=%h",
                       u_npu.u_acc_drain_ctrl.r_row,
                       u_npu.u_acc_drain_ctrl.r_set,
                       u_npu.ac2v_prf_wr_addr,
                       u_npu.u_acc_drain_ctrl.row_fp32[0],
                       u_npu.u_acc_drain_ctrl.row_fp16[0]);
            if (expert_group_monitor_enable && u_npu.vv_prf_wr_en &&
                $isunknown(u_npu.vv_prf_wr_data))
                $fatal(1, "[TB][GROUPED_X_VLD] L1 read emitted X into PRF: line=%h beat=%0d token=%0d prf=%0d data=%h",
                       u_npu.vv_l1_rd_addr,
                       u_npu.u_vld_vst.r_beat_cnt,
                       u_npu.route_group_token[u_npu.group_route_row],
                       u_npu.vv_prf_wr_addr,
                       u_npu.vv_prf_wr_data);
            if (expert_group_monitor_enable && u_npu.ve_rf_wr_en &&
                $isunknown(u_npu.ve_rf_wr_data))
                $fatal(1, "[TB][GROUPED_X_VEC] vector engine emitted X: instruction=%h vd=%0d token=%0d row=%0d data=%h",
                       u_npu.lc_dec_instr, u_npu.ve_rf_wr_addr,
                       u_npu.route_group_token[u_npu.group_route_row],
                       u_npu.group_route_row, u_npu.ve_rf_wr_data);
            if (expert_group_monitor_enable && u_npu.vv_l1_wr_en &&
                $isunknown(u_npu.vv_l1_wr_data))
                $fatal(1, "[TB][GROUPED_X_WRITE] VST emitted X data: page=%0d line=%h row=%0d token=%0d prf=%0d prf_data=%h vec_buf=%h",
                       u_npu.vv_l1_wr_page, u_npu.vv_l1_wr_addr,
                       u_npu.u_vld_vst.r_beat_cnt,
                       u_npu.route_group_token[u_npu.group_route_row],
                       u_npu.vv_prf_rd_addr,
                       u_npu.prf_rd_data_a,
                       u_npu.u_vld_vst.r_vec_buf);
            if (expert_group_monitor_enable && u_npu.dq_in_valid &&
                u_npu.dq_in_ready) begin
                case (u_npu.gemm_dq_nbits_code)
                    3'b001: dequant_precision_seen[0] <= 1'b1;
                    3'b010: dequant_precision_seen[1] <= 1'b1;
                    3'b011: dequant_precision_seen[2] <= 1'b1;
                    default: $fatal(1, "[TB][GROUPED_DEMO] unsupported dequant precision code %b",
                                    u_npu.gemm_dq_nbits_code);
                endcase
            end
            if (u_npu.u_gemm_ctrl.int_gather) begin
                if (u_npu.u_gemm_ctrl.r_commit_k > u_npu.u_gemm_ctrl.r_gather_k)
                    $fatal(1, "[TB][DEQUANT_OVERLAP] commit counter advanced beyond accepted inputs");
                else if ((u_npu.u_gemm_ctrl.r_gather_k -
                          u_npu.u_gemm_ctrl.r_commit_k) > 8'd2)
                    $fatal(1, "[TB][DEQUANT_OVERLAP] accept/commit counters exceed the 2-cycle pipeline window");
            end
            if (u_npu.gemm_done &&
                u_npu.u_gemm_ctrl.r_prec != 2'b00 &&
                (u_npu.u_gemm_ctrl.r_gather_k != u_npu.u_gemm_ctrl.r_K[8:0] ||
                 u_npu.u_gemm_ctrl.r_commit_k != u_npu.u_gemm_ctrl.r_K[8:0]))
                $fatal(1, "[TB][DEQUANT_OVERLAP] GEMM retired with accepts=%0d commits=%0d K=%0d",
                       u_npu.u_gemm_ctrl.r_gather_k,
                       u_npu.u_gemm_ctrl.r_commit_k,
                       u_npu.u_gemm_ctrl.r_K[8:0]);
            if (u_npu.u_gemm_ctrl.int_gather && u_npu.sa_en &&
                u_npu.u_gemm_ctrl.r_feed_k >= u_npu.u_gemm_ctrl.r_commit_k)
                $fatal(1, "[TB][DEQUANT_OVERLAP] systolic feed reached an uncommitted K row");
            if (u_npu.gemm_cmd_accept && u_npu.u_gemm_ctrl.cmd_ready) begin
                grouped_gemm_cmd_count <= grouped_gemm_cmd_count + 1;
                grouped_last_gemm_expert <= u_npu.gemm_cmd_expert_sel;
                grouped_last_gemm_proj <= u_npu.gemm_cmd_proj;
                grouped_last_gemm_k <= u_npu.gemm_cmd_k;
                // Route-group GEMMs use a dynamic M override derived from
                // route_group_count; the host CFG_M register remains unchanged.
                if (expert_group_monitor_enable &&
                    u_npu.gemm_cmd_k != 16'(
                        (u_npu.gemm_cmd_proj == 2'd2)
                            ? ((group_weight_context_chunk == 6'd24)
                                ? GROUP_DEMO_DOWN_TAIL_K
                                : GROUP_DEMO_DOWN_K)
                            : GROUP_DEMO_GATE_UP_K))
                    $fatal(1, "[TB][GROUPED_MAC_UTIL] proj=%0d K=%0d expected=%0d",
                           u_npu.gemm_cmd_proj, u_npu.gemm_cmd_k,
                           (u_npu.gemm_cmd_proj == 2'd2)
                               ? ((group_weight_context_chunk == 6'd24)
                                    ? GROUP_DEMO_DOWN_TAIL_K
                                    : GROUP_DEMO_DOWN_K)
                               : GROUP_DEMO_GATE_UP_K);
                mac_util_m <= (u_npu.cfg_m_active > 32)
                            ? 32 : u_npu.cfg_m_active;
                if (u_npu.cfg_m_active > mac_util_max_m)
                    mac_util_max_m <= (u_npu.cfg_m_active > 32)
                                    ? 32 : u_npu.cfg_m_active;
                mac_util_n <= (u_npu.cfg_N > 32) ? 32 : u_npu.cfg_N;
                mac_util_k <= u_npu.gemm_cmd_k;
                mac_util_active_cycles <= 0;
                mac_util_gemm_wall_cycles <= 0;
                mac_util_gemm_count <= mac_util_gemm_count + 1;
            end
            if (u_npu.u_gemm_ctrl.busy)
                mac_util_gemm_wall_cycles <= mac_util_gemm_wall_cycles + 1;
            if (u_npu.u_gemm_ctrl.busy &&
                u_npu.u_gemm_ctrl.sa_compute_active) begin
                if (expert_group_monitor_enable &&
                    u_npu.sa_active_rows !== 6'(mac_util_m))
                    $fatal(1, "[TB][GROUPED_MAC_UTIL] active PE rows %0d != GEMM M %0d",
                           u_npu.sa_active_rows, mac_util_m);
                mac_util_active_cycles <= mac_util_active_cycles + 1;
                mac_util_enabled_pe_slots <= mac_util_enabled_pe_slots +
                    (longint'(u_npu.sa_active_rows) * 32);
            end
            if (u_npu.u_gemm_ctrl.busy && u_npu.gld_busy)
                mac_util_gld_overlap_cycles <= mac_util_gld_overlap_cycles + 1;
            if (u_npu.u_gemm_ctrl.busy && u_npu.dma_rd_busy)
                mac_util_dma_overlap_cycles <= mac_util_dma_overlap_cycles + 1;
            if (u_npu.u_gemm_ctrl.int_gather &&
                u_npu.u_gemm_ctrl.sa_compute_active)
                mac_util_dequant_feed_overlap_cycles <=
                    mac_util_dequant_feed_overlap_cycles + 1;
            if (u_npu.gemm_done) begin
                mac_util_useful_ops <= mac_util_useful_ops +
                    (longint'(mac_util_m) * longint'(mac_util_n) * longint'(mac_util_k));
                mac_util_peak_ops <= mac_util_peak_ops +
                    (longint'(mac_util_active_cycles) * 32 * 32);
                mac_util_wall_capacity_ops <= mac_util_wall_capacity_ops +
                    (longint'(mac_util_gemm_wall_cycles + 1) * 32 * 32);
            end
        end
    end

    always @(posedge clk) begin
        if (exp_snap_ktiles > 0 && u_npu.gemm_done) begin
            if (((exp_gd_cnt + 1) % exp_snap_ktiles) == 0) begin
                integer t;
                t = exp_gd_cnt / exp_snap_ktiles;
                if (t < 8)
                    for (int jj = 0; jj < 32; jj++)
                        exp_tile_acc[t][jj] <= u_npu.acc_bank_sel
                            ? u_npu.u_acc_buffer.bank1_mem[jj][0]
                            : u_npu.u_acc_buffer.bank0_mem[jj][0];
            end
            exp_gd_cnt <= exp_gd_cnt + 1;
        end
    end

    // =========================================================================
    // DDR4 physical pins (MIG ↔ memory model)
    // =========================================================================
    wire                 c0_ddr4_act_n;
    wire  [16:0]         c0_ddr4_adr;
    wire  [1:0]          c0_ddr4_ba;
    wire  [1:0]          c0_ddr4_bg;
    wire  [0:0]          c0_ddr4_cke;
    wire  [0:0]          c0_ddr4_odt;
    wire  [0:0]          c0_ddr4_cs_n;
    wire  [0:0]          c0_ddr4_ck_t_int;
    wire  [0:0]          c0_ddr4_ck_c_int;
    wire                 c0_ddr4_ck_t;
    wire                 c0_ddr4_ck_c;
    wire                 c0_ddr4_reset_n;
    wire  [0:0]          c0_ddr4_dm_dbi_n;
    wire  [7:0]          c0_ddr4_dq;
    wire  [0:0]          c0_ddr4_dqs_c;
    wire  [0:0]          c0_ddr4_dqs_t;

    // MIG clock
    wire                 c0_sys_clk_p;
    wire                 c0_sys_clk_n;
    logic                sys_rst;
    logic                en_model;
    wire                 model_enable;

    // =========================================================================
    // UCODE preload port
    // =========================================================================
    logic                               tb_ucode_wr_en;
    logic [$clog2(UCODE_DEPTH)-1:0]     tb_ucode_wr_addr;
    logic [31:0]                        tb_ucode_wr_data;

    // =========================================================================
    // NPU reset (declared early — used in port connections below)
    // =========================================================================
    logic rst_n;

    // =========================================================================
    // NPU acc_flush output (sim-only)
    // =========================================================================
    wire        s_acc_flush_valid;
    wire [31:0] s_acc_flush_addr;
    wire [31:0] s_acc_flush_data;

    // =========================================================================
    // DUT: NPU Top
    // =========================================================================
    top #(
        .ARRAY_DIM     (32),
        .P_AXI_DATA_W  (AXI_DATA_W),
        .P_AXI_ADDR_W  (32),
        .P_AXI_ID_W    (4),
        .P_UCODE_DEPTH (UCODE_DEPTH)
    ) u_npu (
        .clk              (clk),
        .rst_n            (rst_n),

        // AXI-Lite
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

        // AXI4 Master (to bridge)
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

        // Sim ports
        .s_acc_flush_valid  (s_acc_flush_valid),
        .s_acc_flush_addr   (s_acc_flush_addr),
        .s_acc_flush_data   (s_acc_flush_data),

        .tb_ucode_wr_en   (tb_ucode_wr_en),
        .tb_ucode_wr_addr (tb_ucode_wr_addr),
        .tb_ucode_wr_data (tb_ucode_wr_data)
    );

    // =========================================================================
    // AXI-to-MIG Bridge (runs on MIG ui_clk — AXI handshake handles CDC)
    // =========================================================================
    axi_to_mig_bridge #(
        .AXI_DATA_W (AXI_DATA_W),
        .AXI_ADDR_W (32),
        .AXI_ID_W   (4),
        .MIG_DATA_W (MIG_DATA_W),
        .MIG_ADDR_W (MIG_ADDR_W),
        .MIG_MASK_W (MIG_MASK_W)
    ) u_bridge (
        .clk               (c0_ddr4_ui_clk),
        .rst_n             (~c0_ddr4_ui_clk_sync_rst),

        // AXI4 Slave (from NPU)
        .s_axi_awid        (m_axi_awid),
        .s_axi_awaddr      (m_axi_awaddr),
        .s_axi_awlen       (m_axi_awlen),
        .s_axi_awsize      (m_axi_awsize),
        .s_axi_awburst     (m_axi_awburst),
        .s_axi_awvalid     (m_axi_awvalid),
        .s_axi_awready     (m_axi_awready),

        .s_axi_wdata       (m_axi_wdata),
        .s_axi_wstrb       (m_axi_wstrb),
        .s_axi_wlast       (m_axi_wlast),
        .s_axi_wvalid      (m_axi_wvalid),
        .s_axi_wready      (m_axi_wready),   // ← correct: bridge wready → NPU

        .s_axi_bid         (m_axi_bid),
        .s_axi_bresp       (m_axi_bresp),
        .s_axi_bvalid      (m_axi_bvalid),
        .s_axi_bready      (m_axi_bready),

        .s_axi_arid        (m_axi_arid),
        .s_axi_araddr      (m_axi_araddr),
        .s_axi_arlen       (m_axi_arlen),
        .s_axi_arsize      (m_axi_arsize),
        .s_axi_arburst     (m_axi_arburst),
        .s_axi_arvalid     (m_axi_arvalid),
        .s_axi_arready     (m_axi_arready),

        .s_axi_rid         (m_axi_rid),
        .s_axi_rdata       (m_axi_rdata),
        .s_axi_rresp       (m_axi_rresp),
        .s_axi_rlast       (m_axi_rlast),
        .s_axi_rvalid      (m_axi_rvalid),
        .s_axi_rready      (m_axi_rready),

        // MIG app interface (bridge outputs → mux)
        .app_addr           (br_app_addr),
        .app_cmd            (br_app_cmd),
        .app_en             (br_app_en),
        .app_hi_pri         (br_app_hi_pri),
        .app_wdf_data       (br_app_wdf_data),
        .app_wdf_end        (br_app_wdf_end),
        .app_wdf_mask       (br_app_wdf_mask),
        .app_wdf_wren       (br_app_wdf_wren),
        .app_rd_data        (app_rd_data),
        .app_rd_data_end    (app_rd_data_end),
        .app_rd_data_valid  (app_rd_data_valid),
        .app_rdy            (app_rdy),
        .app_wdf_rdy        (app_wdf_rdy)
    );

    // =========================================================================
    // MIG DDR4 IP (ddr4_0)
    // =========================================================================
    ddr4_0 u_ddr4 (
        .sys_rst               (sys_rst),
        .c0_sys_clk_p          (c0_sys_clk_p),
        .c0_sys_clk_n          (c0_sys_clk_n),
        .c0_init_calib_complete(c0_init_calib_complete),

        .c0_ddr4_act_n        (c0_ddr4_act_n),
        .c0_ddr4_adr          (c0_ddr4_adr),
        .c0_ddr4_ba           (c0_ddr4_ba),
        .c0_ddr4_bg           (c0_ddr4_bg),
        .c0_ddr4_cke          (c0_ddr4_cke),
        .c0_ddr4_odt          (c0_ddr4_odt),
        .c0_ddr4_cs_n         (c0_ddr4_cs_n),
        .c0_ddr4_ck_t         (c0_ddr4_ck_t_int),
        .c0_ddr4_ck_c         (c0_ddr4_ck_c_int),
        .c0_ddr4_reset_n      (c0_ddr4_reset_n),
        .c0_ddr4_dm_dbi_n     (c0_ddr4_dm_dbi_n),
        .c0_ddr4_dq           (c0_ddr4_dq),
        .c0_ddr4_dqs_c        (c0_ddr4_dqs_c),
        .c0_ddr4_dqs_t        (c0_ddr4_dqs_t),

        .c0_ddr4_ui_clk       (c0_ddr4_ui_clk),
        .c0_ddr4_ui_clk_sync_rst(c0_ddr4_ui_clk_sync_rst),
        .dbg_clk              (dbg_clk),

        .c0_ddr4_app_addr     (app_addr),
        .c0_ddr4_app_cmd      (app_cmd),
        .c0_ddr4_app_en       (app_en),
        .c0_ddr4_app_hi_pri   (app_hi_pri),
        .c0_ddr4_app_wdf_data (app_wdf_data),
        .c0_ddr4_app_wdf_end  (app_wdf_end),
        .c0_ddr4_app_wdf_mask (app_wdf_mask),
        .c0_ddr4_app_wdf_wren (app_wdf_wren),
        .c0_ddr4_app_rd_data  (app_rd_data),
        .c0_ddr4_app_rd_data_end(app_rd_data_end),
        .c0_ddr4_app_rd_data_valid(app_rd_data_valid),
        .c0_ddr4_app_rdy      (app_rdy),
        .c0_ddr4_app_wdf_rdy  (app_wdf_rdy),

        .dbg_bus              ()
    );

    // =========================================================================
    // DDR4 Memory Model (from MIG example sim_tb_top)
    // =========================================================================
    assign c0_ddr4_ck_t = c0_ddr4_ck_t_int[0];
    assign c0_ddr4_ck_c = c0_ddr4_ck_c_int[0];

    reg  [16:0]          c0_ddr4_adr_sdram[1:0];
    reg  [1:0]           c0_ddr4_ba_sdram[1:0];
    reg  [1:0]           c0_ddr4_bg_sdram[1:0];

    always @(*) begin
        c0_ddr4_adr_sdram[0] <= c0_ddr4_adr;
        c0_ddr4_adr_sdram[1] <= c0_ddr4_adr;
        c0_ddr4_ba_sdram[0]  <= c0_ddr4_ba;
        c0_ddr4_ba_sdram[1]  <= c0_ddr4_ba;
        c0_ddr4_bg_sdram[0]  <= c0_ddr4_bg;
        c0_ddr4_bg_sdram[1]  <= c0_ddr4_bg;
    end

    reg [31:0] cmdName;
    reg [ADDR_WIDTH-1:0] DDR4_ADRMOD[RANK_WIDTH-1:0];

    always @(*) begin
        if (c0_ddr4_cs_n == 1'b1)
            cmdName = "DSEL";
        else if (c0_ddr4_act_n)
            casez (DDR4_ADRMOD[0][16:14])
                MRS:  cmdName = "MRS";
                REF:  cmdName = "REF";
                PRE:  cmdName = "PRE";
                WR:   cmdName = "WR";
                RD:   cmdName = "RD";
                ZQC:  cmdName = "ZQC";
                NOP:  cmdName = "NOP";
                default: cmdName = "***";
            endcase
        else
            cmdName = "ACT";
    end

    always @(*) begin
        if (c0_ddr4_act_n)
            casez (c0_ddr4_adr_sdram[0][16:14])
                WR, RD: DDR4_ADRMOD[0] = c0_ddr4_adr_sdram[0] & 18'h1C7FF;
                default: DDR4_ADRMOD[0] = c0_ddr4_adr_sdram[0];
            endcase
        else
            DDR4_ADRMOD[0] = c0_ddr4_adr_sdram[0];
    end

    DDR4_if #(.CONFIGURED_DQ_BITS(8)) iDDR4[0:0]();

    ddr4_model #(
        .CONFIGURED_DQ_BITS(8),
        .CONFIGURED_DENSITY(CONFIGURED_DENSITY)
    ) ddr4_model_inst (
        .model_enable(model_enable),
        .iDDR4(iDDR4[0])
    );

    genvar s;
    generate
        for (s = 0; s < 8; s++) begin: tranDQ
            tran bidiDQ(iDDR4[0].DQ[s], c0_ddr4_dq[s]);
        end
    endgenerate

    tran bidiDQS(iDDR4[0].DQS_t, c0_ddr4_dqs_t[0]);
    tran bidiDQS_(iDDR4[0].DQS_c, c0_ddr4_dqs_c[0]);
    tran bidiDM(iDDR4[0].DM_n, c0_ddr4_dm_dbi_n[0]);

    assign iDDR4[0].BG       = c0_ddr4_bg_sdram[0];
    assign iDDR4[0].BA       = c0_ddr4_ba_sdram[0];
    assign iDDR4[0].ADDR_17  = 1'b0;
    assign iDDR4[0].ADDR     = DDR4_ADRMOD[0][13:0];
    assign iDDR4[0].CS_n    = c0_ddr4_cs_n[0];
    assign iDDR4[0].CK      = {c0_ddr4_ck_t, c0_ddr4_ck_c};
    assign iDDR4[0].ACT_n   = c0_ddr4_act_n;
    assign iDDR4[0].RAS_n_A16 = DDR4_ADRMOD[0][16];
    assign iDDR4[0].CAS_n_A15 = DDR4_ADRMOD[0][15];
    assign iDDR4[0].WE_n_A14  = DDR4_ADRMOD[0][14];
    assign iDDR4[0].CKE      = c0_ddr4_cke[0];
    assign iDDR4[0].ODT      = c0_ddr4_odt[0];
    assign iDDR4[0].PARITY   = 1'b0;
    assign iDDR4[0].TEN      = 1'b0;
    assign iDDR4[0].ZQ       = 1'b1;
    assign iDDR4[0].PWR      = 1'b1;
    assign iDDR4[0].VREF_CA  = 1'b1;
    assign iDDR4[0].VREF_DQ  = 1'b1;
    assign iDDR4[0].RESET_n  = c0_ddr4_reset_n;

    // =========================================================================
    // MIG differential clock (74 MHz — gives ui_clk ≈ 200 MHz with DDR4-1600)
    // =========================================================================
    localparam real SYSCLK_PERIOD_NS = 13.501;  // 74 MHz
    reg sys_clk_i = 1'b0;
    always sys_clk_i = #(SYSCLK_PERIOD_NS / 2.0) ~sys_clk_i;
    assign c0_sys_clk_p = sys_clk_i;
    assign c0_sys_clk_n = ~sys_clk_i;

    // =========================================================================
    // Reset and clock domain handling
    // =========================================================================
    // rst_n declared above (before NPU instance)

    initial begin
        sys_rst = 1'b0;
        #200;
        sys_rst = 1'b1;
        en_model = 1'b0;
        #5 en_model = 1'b1;
        #200;
        sys_rst = 1'b0;
        #100;
    end

    assign model_enable = en_model;

    initial begin
        rst_n = 1'b0;
        wait(c0_init_calib_complete === 1'b1);
        #100;
        rst_n = 1'b1;
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
        while (!s_axil_bvalid) @(posedge clk);
        s_axil_bready <= 1'b1;
        @(posedge clk);
        s_axil_bready <= 1'b0;
    endtask

    // =========================================================================
    // Load UCODE from binary file
    // =========================================================================
    logic [7:0] ucode_mem [0:UCODE_DEPTH*4-1];
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

    // Variant that lets the caller choose the ucode file (MIG_REAL_COMPUTE).
    task load_ucode_file(input string fname);
        integer fd, r;
        fd = $fopen(fname, "rb");
        if (fd == 0) begin
            $fatal(1, "Cannot open ucode file %0s", fname);
        end
        ucode_size = 0;
        while (!$feof(fd)) begin
            r = $fread(ucode_mem[ucode_size], fd);
            if (r > 0) ucode_size = ucode_size + r;
        end
        $fclose(fd);
        $display("[TB] Loaded %0d bytes from %s (%0d instructions)", ucode_size, fname, ucode_size/4);

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
    // DDR4 MIG app write task — write one 64-bit word
    // =========================================================================
    task ddr4_mig_write(input logic [MIG_ADDR_W-1:0] waddr,
                        input logic [MIG_DATA_W-1:0] wdata);
        bit cmd_pending, data_pending;
        @(posedge c0_ddr4_ui_clk);
        tb_app_addr    <= waddr;
        tb_app_cmd     <= 3'b000;  // write
        tb_app_en      <= 1'b1;
        tb_app_hi_pri  <= 1'b0;
        tb_app_wdf_data <= wdata;
        tb_app_wdf_end  <= 1'b1;
        tb_app_wdf_mask <= '0;
        tb_app_wdf_wren <= 1'b1;
        // Command and write-data channels are independent.  Deassert each
        // valid immediately after its own handshake so backpressure cannot
        // duplicate one half of a continuous write stream.
        cmd_pending = 1'b1;
        data_pending = 1'b1;
        while (cmd_pending || data_pending) begin
            @(posedge c0_ddr4_ui_clk);
            if (cmd_pending && app_rdy) begin
                tb_app_en <= 1'b0;
                cmd_pending = 1'b0;
            end
            if (data_pending && app_wdf_rdy) begin
                tb_app_wdf_wren <= 1'b0;
                tb_app_wdf_end <= 1'b0;
                data_pending = 1'b0;
            end
        end
    endtask

    // =========================================================================
    // DDR4 MIG app read task — read one 64-bit word
    // =========================================================================
    task ddr4_mig_read(input  logic [MIG_ADDR_W-1:0] raddr,
                       output logic [MIG_DATA_W-1:0] rdata);
        @(posedge c0_ddr4_ui_clk);
        tb_app_addr <= raddr;
        tb_app_cmd  <= 3'b001;  // read
        tb_app_en   <= 1'b1;
        do @(posedge c0_ddr4_ui_clk); while (!app_rdy);
        tb_app_en <= 1'b0;
        do @(posedge c0_ddr4_ui_clk); while (!app_rd_data_valid);
        rdata = app_rd_data;
    endtask

    // =========================================================================
    // AXI-side bridge smoke tasks (128-bit AXI beat <-> two 64-bit MIG beats)
    // =========================================================================
    task axi_smoke_read(input  logic [31:0] addr,
                        output logic [AXI_DATA_W-1:0] data);
        @(posedge clk);
        smoke_axi_arid    = 4'hA;
        smoke_axi_araddr  = addr;
        smoke_axi_arlen   = 8'd0;
        smoke_axi_arsize  = 3'd4;  // 16 bytes
        smoke_axi_arburst = 2'b01; // INCR
        smoke_axi_arvalid = 1'b1;
        smoke_axi_rready  = 1'b1;

        do @(posedge clk); while (!m_axi_arready);
        smoke_axi_arvalid = 1'b0;

        do @(posedge clk); while (!m_axi_rvalid);
        data = m_axi_rdata;
        if (!m_axi_rlast)
            $fatal(1, "AXI bridge read response missing RLAST");
        @(posedge clk);
        smoke_axi_rready = 1'b0;
    endtask

    task axi_smoke_write(input logic [31:0] addr,
                         input logic [AXI_DATA_W-1:0] data);
        integer start_cycle;
        start_cycle = mig_timing_cycle;
        @(posedge clk);
        smoke_axi_awid    = 4'h5;
        smoke_axi_awaddr  = addr;
        smoke_axi_awlen   = 8'd0;
        smoke_axi_awsize  = 3'd4;  // 16 bytes
        smoke_axi_awburst = 2'b01; // INCR
        smoke_axi_awvalid = 1'b1;

        do @(posedge clk); while (!m_axi_awready);
        smoke_axi_awvalid = 1'b0;

        smoke_axi_wdata  = data;
        smoke_axi_wstrb  = '1;
        smoke_axi_wlast  = 1'b1;
        smoke_axi_wvalid = 1'b1;
        do @(posedge clk); while (!m_axi_wready);
        smoke_axi_wvalid = 1'b0;

        smoke_axi_bready = 1'b1;
        do @(posedge clk); while (!m_axi_bvalid);
        if (m_axi_bresp != 2'b00)
            $fatal(1, "AXI bridge write returned BRESP=%0b", m_axi_bresp);
        @(posedge clk);
        smoke_axi_bready = 1'b0;
        $display("MIG_TIMING|AXI_WRITE_16B|bytes=16|cycles=%0d", mig_timing_cycle - start_cycle);
    endtask

    // Burst versions deliberately exercise the exact AXI path used by
    // axi_dma_rd/axi_dma_wr: one outstanding burst, 16-byte AXI beats, and
    // the bridge's two 64-bit MIG app transactions per beat.
    task axi_smoke_read_burst(input logic [31:0] addr, input integer beats);
        integer start_cycle, beat;
        start_cycle = mig_timing_cycle;
        @(posedge clk);
        smoke_axi_arid    = 4'hC;
        smoke_axi_araddr  = addr;
        smoke_axi_arlen   = beats - 1;
        smoke_axi_arsize  = 3'd4;
        smoke_axi_arburst = 2'b01;
        smoke_axi_arvalid = 1'b1;
        smoke_axi_rready  = 1'b0;
        do @(posedge clk); while (!m_axi_arready);
        smoke_axi_arvalid = 1'b0;
        for (beat = 0; beat < beats; beat = beat + 1) begin
            do @(negedge clk); while (!m_axi_rvalid);
            if (m_axi_rdata !== {8{beat[15:0]}})
                $fatal(1, "AXI read burst data mismatch at beat %0d: expected=%h actual=%h",
                       beat, {8{beat[15:0]}}, m_axi_rdata);
            if ((beat == beats - 1) != m_axi_rlast)
                $fatal(1, "AXI read burst RLAST mismatch at beat %0d", beat);
            if ((beat % 8) == 0) begin
                repeat (3) begin
                    @(negedge clk);
                    if (!m_axi_rvalid ||
                        m_axi_rdata !== {8{beat[15:0]}} ||
                        m_axi_rlast !== (beat == beats - 1))
                        $fatal(1, "AXI R response changed under backpressure at beat %0d",
                               beat);
                end
            end
            smoke_axi_rready = 1'b1;
            @(posedge clk);
            @(negedge clk);
            smoke_axi_rready = 1'b0;
        end
        $display("MIG_TIMING|AXI_READ_BURST|bytes=%0d|beats=%0d|cycles=%0d",
                 beats * (AXI_DATA_W / 8), beats, mig_timing_cycle - start_cycle);
    endtask

    task axi_smoke_write_burst(input logic [31:0] addr, input integer beats);
        integer start_cycle, beat;
        start_cycle = mig_timing_cycle;
        @(posedge clk);
        smoke_axi_awid    = 4'h6;
        smoke_axi_awaddr  = addr;
        smoke_axi_awlen   = beats - 1;
        smoke_axi_awsize  = 3'd4;
        smoke_axi_awburst = 2'b01;
        smoke_axi_awvalid = 1'b1;
        do @(posedge clk); while (!m_axi_awready);
        smoke_axi_awvalid = 1'b0;
        for (beat = 0; beat < beats; beat = beat + 1) begin
            smoke_axi_wdata  = {8{beat[15:0]}};
            smoke_axi_wstrb  = '1;
            smoke_axi_wlast  = (beat == beats - 1);
            smoke_axi_wvalid = 1'b1;
            do @(posedge clk); while (!m_axi_wready);
            smoke_axi_wvalid = 1'b0;
        end
        smoke_axi_bready = 1'b1;
        do @(posedge clk); while (!m_axi_bvalid);
        if (m_axi_bresp != 2'b00)
            $fatal(1, "AXI write burst returned BRESP=%0b", m_axi_bresp);
        @(posedge clk);
        smoke_axi_bready = 1'b0;
        $display("MIG_TIMING|AXI_WRITE_BURST|bytes=%0d|beats=%0d|cycles=%0d",
                 beats * (AXI_DATA_W / 8), beats, mig_timing_cycle - start_cycle);
    endtask

    // =========================================================================
    // Stream a bounded slice of a binary file into DDR4.  The buffer remains
    // fixed-size even when the source image is tens of MiB.
    // =========================================================================
    // Keep the shared file buffer small: ModelSim allocates unpacked arrays
    // eagerly, and the fast writer already pipelines each chunk independently.
    localparam integer PREFILL_CHUNK_BYTES = 1 * 1024 * 1024;
    localparam integer PREFILL_CHUNK_CYCLE_LIMIT = 4_000_000;
    logic [7:0] prefill_buf [0:PREFILL_CHUNK_BYTES-1];
    logic [7:0] expected_buf [0:15];

    task prefill_ddr4_stream(input string filename,
                             input integer file_offset,
                             input logic [31:0] byte_addr,
                             input integer max_bytes,
                             input bit require_exact,
                             output integer total);
        integer fd, r, request_bytes, i;
        longint unsigned address_limit;
        logic [MIG_DATA_W-1:0] wdata64;
        logic [MIG_ADDR_W-1:0] mig_addr;

        total = 0;
        address_limit = 64'd1 << MIG_ADDR_W;
        if (file_offset < 0 || max_bytes <= 0)
            $fatal(1, "Invalid stream request: offset=%0d bytes=%0d", file_offset, max_bytes);
        if ((byte_addr + max_bytes) > address_limit)
            $fatal(1, "DDR stream range exceeds %0d-bit address window", MIG_ADDR_W);

        fd = $fopen(filename, "rb");
        if (fd == 0)
            $fatal(1, "Cannot open DDR image %s", filename);
        if ($fseek(fd, file_offset, 0) != 0)
            $fatal(1, "Cannot seek %s to byte %0d", filename, file_offset);

        while (total < max_bytes) begin
            request_bytes = max_bytes - total;
            if (request_bytes > PREFILL_CHUNK_BYTES)
                request_bytes = PREFILL_CHUNK_BYTES;
            r = $fread(prefill_buf, fd, 0, request_bytes);
            if (r <= 0)
                break;

            for (i = 0; i < r; i += 8) begin
                wdata64 = '0;
                for (int j = 0; j < 8 && (i + j) < r; j++)
                    wdata64[j*8 +: 8] = prefill_buf[i + j];
                mig_addr = byte_addr + total + i;
                ddr4_mig_write(mig_addr, wdata64);
            end
            total = total + r;
            $display("[TB][MIG_REAL] progress: %0d / %0d bytes (%.1f%%) written to 0x%08x @%0t",
                     total, max_bytes, (total * 100.0) / max_bytes,
                     byte_addr + total, $time);
            if (r < request_bytes)
                break;
        end
        $fclose(fd);

        if (require_exact && total != max_bytes)
            $fatal(1, "Short DDR image read: requested=%0d actual=%0d", max_bytes, total);
        $display("[TB][MIG_REAL] streamed %0d bytes from %s[%0d] to DDR byte 0x%08x",
                 total, filename, file_offset, byte_addr);
    endtask

    task prefill_ddr4(input string filename,
                      input logic [31:0] byte_addr,
                      input integer max_bytes);
        integer total;
        prefill_ddr4_stream(filename, 0, byte_addr, max_bytes, 1'b0, total);
    endtask

    // Pipeline contiguous MIG app writes.  This remains a real MIG front-door
    // transfer: command and write-data channels simply stay occupied whenever
    // their respective ready signals allow it.  It avoids the artificial idle
    // clock inserted by ddr4_mig_write between every 64-bit word.
    task prefill_ddr4_stream_fast(input string filename,
                                  input integer file_offset,
                                  input logic [31:0] byte_addr,
                                  input integer max_bytes,
                                  input bit require_exact,
                                  output integer total);
        integer fd, r, request_bytes, base, cmd_i, data_i;
        integer chunk_cycles, last_reported_data;
        logic [MIG_DATA_W-1:0] wdata64;
        logic [MIG_ADDR_W-1:0] mig_addr;
        longint unsigned address_limit;

        total = 0;
        address_limit = 64'd1 << MIG_ADDR_W;
        if (file_offset < 0 || max_bytes <= 0)
            $fatal(1, "Invalid fast stream request: offset=%0d bytes=%0d", file_offset, max_bytes);
        if ((byte_addr + max_bytes) > address_limit)
            $fatal(1, "Fast DDR stream range exceeds %0d-bit address window", MIG_ADDR_W);
        fd = $fopen(filename, "rb");
        if (fd == 0) $fatal(1, "Cannot open DDR image %s", filename);
        if ($fseek(fd, file_offset, 0) != 0)
            $fatal(1, "Cannot seek %s to byte %0d", filename, file_offset);

        while (total < max_bytes) begin
            request_bytes = max_bytes - total;
            if (request_bytes > PREFILL_CHUNK_BYTES)
                request_bytes = PREFILL_CHUNK_BYTES;
            r = $fread(prefill_buf, fd, 0, request_bytes);
            if (r <= 0) break;
            cmd_i = 0;
            data_i = 0;
            chunk_cycles = 0;
            last_reported_data = 0;
            tb_app_en <= 1'b0;
            tb_app_wdf_wren <= 1'b0;
            while (cmd_i < r || data_i < r || tb_app_en || tb_app_wdf_wren) begin
                @(posedge c0_ddr4_ui_clk);
                chunk_cycles = chunk_cycles + 1;
                if (tb_app_en && app_rdy) begin
                    tb_app_en <= 1'b0;
                    cmd_i = cmd_i + 8;
                end
                if (tb_app_wdf_wren && app_wdf_rdy) begin
                    tb_app_wdf_wren <= 1'b0;
                    tb_app_wdf_end <= 1'b0;
                    data_i = data_i + 8;
                end
                // When ready is high the preceding valid is consumed at this
                // edge, so present the following word for the next edge.
                if ((!tb_app_en || app_rdy) && cmd_i < r) begin
                    mig_addr = byte_addr + total + cmd_i;
                    tb_app_addr <= mig_addr;
                    tb_app_cmd <= 3'b000;
                    tb_app_hi_pri <= 1'b0;
                    tb_app_en <= 1'b1;
                end
                if ((!tb_app_wdf_wren || app_wdf_rdy) && data_i < r) begin
                    wdata64 = '0;
                    for (int j = 0; j < 8 && (data_i + j) < r; j++)
                        wdata64[j*8 +: 8] = prefill_buf[data_i + j];
                    tb_app_wdf_data <= wdata64;
                    tb_app_wdf_end <= 1'b1;
                    tb_app_wdf_mask <= '0;
                    tb_app_wdf_wren <= 1'b1;
                end
                if (data_i >= last_reported_data + 65536) begin
                    last_reported_data = data_i;
                    $display("[TB][MIG_REAL_FAST] chunk progress: data=%0d/%0d cmd=%0d cycles=%0d app_rdy=%b wdf_rdy=%b @%0t",
                             data_i, r, cmd_i, chunk_cycles, app_rdy, app_wdf_rdy, $time);
                end
                if (chunk_cycles >= PREFILL_CHUNK_CYCLE_LIMIT)
                    $fatal(1, "Fast DDR prefill stalled: data=%0d/%0d cmd=%0d cycles=%0d app_rdy=%b wdf_rdy=%b",
                           data_i, r, cmd_i, chunk_cycles, app_rdy, app_wdf_rdy);
            end
            total = total + r;
            $display("[TB][MIG_REAL_FAST] progress: %0d / %0d bytes (%.1f%%) @%0t",
                     total, max_bytes, (total * 100.0) / max_bytes, $time);
            if (r < request_bytes) break;
        end
        $fclose(fd);
        if (require_exact && total != max_bytes)
            $fatal(1, "Short fast DDR image read: requested=%0d actual=%0d", max_bytes, total);
        $display("[TB][MIG_REAL_FAST] streamed %0d bytes from %s[%0d] to DDR byte 0x%08x",
                 total, filename, file_offset, byte_addr);
    endtask

    // Check image words through the MIG app read port after prefill. A stride
    // of 4 KiB is a fast page-level spot check; a stride of 8 bytes compares
    // every byte in an 8-byte-aligned image.
    task verify_ddr4_image(input string filename,
                           input integer image_bytes,
                           input integer sample_stride_bytes,
                           output integer sample_count);
        integer fd, sample_offset, read_count, last_offset, expected_samples;
        logic [7:0] expected_bytes [0:7];
        logic [MIG_ADDR_W-1:0] sample_addr;
        logic [MIG_DATA_W-1:0] expected_word, actual_word;

        if (image_bytes < 8 || (image_bytes % 8) != 0 ||
            sample_stride_bytes < 8 ||
            (sample_stride_bytes % 8) != 0)
            $fatal(1, "Invalid DDR image verification geometry: bytes=%0d stride=%0d",
                   image_bytes, sample_stride_bytes);
        fd = $fopen(filename, "rb");
        if (fd == 0)
            $fatal(1, "Cannot open DDR image for readback verification: %s", filename);
        sample_count = 0;
        for (sample_offset = 0;
             sample_offset + 8 <= image_bytes;
             sample_offset = sample_offset + sample_stride_bytes) begin
            if ($fseek(fd, sample_offset, 0) != 0)
                $fatal(1, "Cannot seek DDR image to verification offset %0d", sample_offset);
            read_count = $fread(expected_bytes, fd, 0, 8);
            if (read_count != 8)
                $fatal(1, "Short DDR image verification read at offset %0d: %0d bytes",
                       sample_offset, read_count);
            expected_word = {expected_bytes[7], expected_bytes[6],
                             expected_bytes[5], expected_bytes[4],
                             expected_bytes[3], expected_bytes[2],
                             expected_bytes[1], expected_bytes[0]};
            sample_addr = sample_offset;
            ddr4_mig_read(sample_addr, actual_word);
            if (actual_word !== expected_word)
                $fatal(1, "DDR image page sample mismatch at byte 0x%08x: expected=%h actual=%h",
                       sample_offset, expected_word, actual_word);
            sample_count = sample_count + 1;
            if ((sample_count % 1024) == 0)
                $display("[TB][MIG_EXPERT] DDR image readback progress: %0d samples, next byte 0x%08x",
                         sample_count, sample_offset + sample_stride_bytes);
        end
        last_offset = (image_bytes - 8) & ~7;
        if ((last_offset % sample_stride_bytes) != 0) begin
            if ($fseek(fd, last_offset, 0) != 0)
                $fatal(1, "Cannot seek DDR image to final verification offset %0d",
                       last_offset);
            read_count = $fread(expected_bytes, fd, 0, 8);
            if (read_count != 8)
                $fatal(1, "Short DDR image final verification read: %0d bytes",
                       read_count);
            expected_word = {expected_bytes[7], expected_bytes[6],
                             expected_bytes[5], expected_bytes[4],
                             expected_bytes[3], expected_bytes[2],
                             expected_bytes[1], expected_bytes[0]};
            sample_addr = last_offset;
            ddr4_mig_read(sample_addr, actual_word);
            if (actual_word !== expected_word)
                $fatal(1, "DDR image final sample mismatch at byte 0x%08x: expected=%h actual=%h",
                       last_offset, expected_word, actual_word);
            sample_count = sample_count + 1;
        end
        expected_samples = (image_bytes + sample_stride_bytes - 1) /
                           sample_stride_bytes;
        if ((last_offset % sample_stride_bytes) != 0)
            expected_samples = expected_samples + 1;
        if (sample_count != expected_samples)
            $fatal(1, "DDR image readback sample coverage mismatch: actual=%0d expected=%0d",
                   sample_count, expected_samples);
        $fclose(fd);
        if (sample_stride_bytes == 8)
            $display("[TB][MIG_EXPERT] FULL DDR image readback PASS: %0d words (%0d bytes)",
                     sample_count, image_bytes);
        else
            $display("[TB][MIG_EXPERT] DDR image page readback PASS: %0d samples, stride=%0d bytes, final image word included",
                     sample_count, sample_stride_bytes);
    endtask

    task read_expected64(input string filename,
                         input integer file_offset,
                         output logic [63:0] expected);
        integer fd, r;
        fd = $fopen(filename, "rb");
        if (fd == 0)
            $fatal(1, "Cannot open expected image %s", filename);
        if ($fseek(fd, file_offset, 0) != 0)
            $fatal(1, "Cannot seek expected image to %0d", file_offset);
        r = $fread(expected_buf, fd, 0, 8);
        $fclose(fd);
        if (r != 8)
            $fatal(1, "Short expected read at file byte %0d", file_offset);
        for (int j = 0; j < 8; j++)
            expected[j*8 +: 8] = expected_buf[j];
    endtask

    task read_expected128(input string filename,
                          input integer file_offset,
                          output logic [127:0] expected);
        integer fd, r;
        fd = $fopen(filename, "rb");
        if (fd == 0)
            $fatal(1, "Cannot open expected image %s", filename);
        if ($fseek(fd, file_offset, 0) != 0)
            $fatal(1, "Cannot seek expected image to %0d", file_offset);
        r = $fread(expected_buf, fd, 0, 16);
        $fclose(fd);
        if (r != 16)
            $fatal(1, "Short expected read at file byte %0d", file_offset);
        for (int j = 0; j < 16; j++)
            expected[j*8 +: 8] = expected_buf[j];
    endtask

    // FP16 bits -> real (for TopK weight comparison against router_topk_ref).
    function automatic real fp16_to_real(input logic [15:0] h);
        real m;
        integer e;
        begin
            if (h[14:10] == 5'd0) begin
                m = h[9:0] / 1024.0;
                fp16_to_real = m * (2.0 ** (-14));
            end else if (h[14:10] == 5'h1F) begin
                fp16_to_real = 0.0;   // inf/nan not expected here
            end else begin
                e = h[14:10] - 15;
                m = 1.0 + h[9:0] / 1024.0;
                fp16_to_real = m * (2.0 ** e);
            end
            if (h[15]) fp16_to_real = -fp16_to_real;
        end
    endfunction

    // =========================================================================
    // Main test sequence
    // =========================================================================
    initial begin
        if (!$test$plusargs("MIG_LAYER0") && !$test$plusargs("NO_VCD")) begin
            $dumpfile("npu_mig_sim.vcd");
            $dumpvars(0, tb_npu_mig);
        end

        // Initialize AXI-Lite
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

        // Initialize prefill signals
        tb_app_addr     = 0;
        tb_app_cmd      = 0;
        tb_app_en       = 0;
        tb_app_hi_pri   = 0;
        tb_app_wdf_data = 0;
        tb_app_wdf_end  = 0;
        tb_app_wdf_mask = 0;
        tb_app_wdf_wren = 0;

        // Wait for MIG calibration
        $display("[TB] Waiting for MIG calibration...");
        wait(c0_init_calib_complete === 1'b1);
        $display("[TB] MIG calibration complete!");
        repeat(20) @(posedge c0_ddr4_ui_clk);

        // Minimal connection test.  It deliberately avoids loading weights or
        // starting the NPU; this mode only proves MIG + DDR4 model read/write.
        if ($test$plusargs("MIG_SMOKE_ONLY")) begin : mig_smoke
            logic [MIG_DATA_W-1:0] smoke_rdata;
            logic [AXI_DATA_W-1:0] smoke_axi_rdata;
            localparam logic [MIG_ADDR_W-1:0] SMOKE_ADDR = 29'h0000_0100;
            localparam logic [MIG_DATA_W-1:0] SMOKE_DATA = 64'h0123_4567_89AB_CDEF;
            localparam logic [MIG_DATA_W-1:0] SMOKE_DATA_HI = 64'hFEDC_BA98_7654_3210;
            localparam logic [31:0] AXI_WRITE_ADDR = 32'h0000_0180;
            localparam logic [AXI_DATA_W-1:0] AXI_WRITE_DATA =
                128'hA5A5_5A5A_1122_3344_5566_7788_CCDD_EEFF;

            smoke_axi_arid = '0;
            smoke_axi_araddr = '0;
            smoke_axi_arlen = '0;
            smoke_axi_arsize = '0;
            smoke_axi_arburst = '0;
            smoke_axi_arvalid = 1'b0;
            smoke_axi_rready = 1'b0;
            smoke_axi_awid = '0;
            smoke_axi_awaddr = '0;
            smoke_axi_awlen = '0;
            smoke_axi_awsize = '0;
            smoke_axi_awburst = '0;
            smoke_axi_awvalid = 1'b0;
            smoke_axi_wdata = '0;
            smoke_axi_wstrb = '0;
            smoke_axi_wlast = 1'b0;
            smoke_axi_wvalid = 1'b0;
            smoke_axi_bready = 1'b0;

            // Override only the NPU-produced AXI request signals.  Bridge
            // responses remain the real signals observed by the tasks.
            force m_axi_arid    = smoke_axi_arid;
            force m_axi_araddr  = smoke_axi_araddr;
            force m_axi_arlen   = smoke_axi_arlen;
            force m_axi_arsize  = smoke_axi_arsize;
            force m_axi_arburst = smoke_axi_arburst;
            force m_axi_arvalid = smoke_axi_arvalid;
            force m_axi_rready  = smoke_axi_rready;
            force m_axi_awid    = smoke_axi_awid;
            force m_axi_awaddr  = smoke_axi_awaddr;
            force m_axi_awlen   = smoke_axi_awlen;
            force m_axi_awsize  = smoke_axi_awsize;
            force m_axi_awburst = smoke_axi_awburst;
            force m_axi_awvalid = smoke_axi_awvalid;
            force m_axi_wdata   = smoke_axi_wdata;
            force m_axi_wstrb   = smoke_axi_wstrb;
            force m_axi_wlast   = smoke_axi_wlast;
            force m_axi_wvalid  = smoke_axi_wvalid;
            force m_axi_bready  = smoke_axi_bready;

            prefill_mode = 1'b1;
            repeat(10) @(posedge c0_ddr4_ui_clk);
            ddr4_mig_write(SMOKE_ADDR, SMOKE_DATA);
            ddr4_mig_write(SMOKE_ADDR + 8, SMOKE_DATA_HI);
            repeat(20) @(posedge c0_ddr4_ui_clk);
            ddr4_mig_read(SMOKE_ADDR, smoke_rdata);

            if (smoke_rdata !== SMOKE_DATA) begin
                $display("[TB][MIG_SMOKE] FAIL addr=0x%08x expected=0x%016x actual=0x%016x",
                         SMOKE_ADDR, SMOKE_DATA, smoke_rdata);
                $fatal(1, "MIG DDR4 readback mismatch");
            end
            $display("[TB][MIG_SMOKE] PASS addr=0x%08x data=0x%016x",
                     SMOKE_ADDR, smoke_rdata);

            // Switch ownership to the real AXI-to-MIG bridge and verify that
            // one 128-bit AXI read assembles the two native MIG words.
            prefill_mode = 1'b0;
            repeat(20) @(posedge c0_ddr4_ui_clk);
            axi_smoke_read(SMOKE_ADDR, smoke_axi_rdata);
            if (smoke_axi_rdata !== {SMOKE_DATA_HI, SMOKE_DATA}) begin
                $display("[TB][MIG_AXI_SMOKE] READ FAIL expected=0x%032x actual=0x%032x",
                         {SMOKE_DATA_HI, SMOKE_DATA}, smoke_axi_rdata);
                $fatal(1, "AXI-to-MIG bridge read mismatch");
            end
            $display("[TB][MIG_AXI_SMOKE] READ PASS addr=0x%08x data=0x%032x",
                     SMOKE_ADDR, smoke_axi_rdata);

            // Exercise the reverse path as well: AXI write -> two MIG writes
            // -> DDR4 model -> AXI readback.
            axi_smoke_write(AXI_WRITE_ADDR, AXI_WRITE_DATA);
            axi_smoke_read(AXI_WRITE_ADDR, smoke_axi_rdata);
            if (smoke_axi_rdata !== AXI_WRITE_DATA) begin
                $display("[TB][MIG_AXI_SMOKE] WRITE/READ FAIL expected=0x%032x actual=0x%032x",
                         AXI_WRITE_DATA, smoke_axi_rdata);
                $fatal(1, "AXI-to-MIG bridge write/read mismatch");
            end
            $display("[TB][MIG_AXI_SMOKE] WRITE/READ PASS addr=0x%08x data=0x%032x",
                     AXI_WRITE_ADDR, smoke_axi_rdata);

            // Timing calibration for the DMA model: 64 AXI beats is exactly
            // the MAX_BURST_LEN selected by npu_top (1 KiB at 128-bit AXI).
            axi_smoke_write_burst(32'h0000_0200, 64);
            axi_smoke_read_burst(32'h0000_0200, 64);
            $finish;
        end

        // Load a real image slice through the native MIG front door, then
        // verify deterministic samples through both native MIG and the actual
        // NPU-facing AXI-to-MIG bridge.
        if ($test$plusargs("MIG_REAL_IMAGE_CHECK")) begin : mig_real_image_check
            string real_image;
            integer real_file_offset, real_ddr_base, real_bytes, loaded_bytes;
            integer check_offset, word_count;
            logic [31:0] random_state;
            logic [63:0] expected64, actual64;
            logic [127:0] expected128, actual128;

            if (!$value$plusargs("REAL_IMAGE=%s", real_image))
                $fatal(1, "+REAL_IMAGE=<path> is required");
            if (!$value$plusargs("REAL_FILE_OFFSET=%d", real_file_offset))
                $fatal(1, "+REAL_FILE_OFFSET=<decimal> is required");
            if (!$value$plusargs("REAL_DDR_BASE=%d", real_ddr_base))
                $fatal(1, "+REAL_DDR_BASE=<decimal> is required");
            if (!$value$plusargs("REAL_BYTES=%d", real_bytes))
                $fatal(1, "+REAL_BYTES=<decimal> is required");
            if (real_bytes < 16 || (real_ddr_base & 15) != 0)
                $fatal(1, "Real-image range must be >=16 bytes and 16-byte aligned");

            // Initialize and take control of the bridge's AXI request side.
            smoke_axi_arid = '0;
            smoke_axi_araddr = '0;
            smoke_axi_arlen = '0;
            smoke_axi_arsize = '0;
            smoke_axi_arburst = '0;
            smoke_axi_arvalid = 1'b0;
            smoke_axi_rready = 1'b0;
            smoke_axi_awid = '0;
            smoke_axi_awaddr = '0;
            smoke_axi_awlen = '0;
            smoke_axi_awsize = '0;
            smoke_axi_awburst = '0;
            smoke_axi_awvalid = 1'b0;
            smoke_axi_wdata = '0;
            smoke_axi_wstrb = '0;
            smoke_axi_wlast = 1'b0;
            smoke_axi_wvalid = 1'b0;
            smoke_axi_bready = 1'b0;
            force m_axi_arid    = smoke_axi_arid;
            force m_axi_araddr  = smoke_axi_araddr;
            force m_axi_arlen   = smoke_axi_arlen;
            force m_axi_arsize  = smoke_axi_arsize;
            force m_axi_arburst = smoke_axi_arburst;
            force m_axi_arvalid = smoke_axi_arvalid;
            force m_axi_rready  = smoke_axi_rready;
            force m_axi_awid    = smoke_axi_awid;
            force m_axi_awaddr  = smoke_axi_awaddr;
            force m_axi_awlen   = smoke_axi_awlen;
            force m_axi_awsize  = smoke_axi_awsize;
            force m_axi_awburst = smoke_axi_awburst;
            force m_axi_awvalid = smoke_axi_awvalid;
            force m_axi_wdata   = smoke_axi_wdata;
            force m_axi_wstrb   = smoke_axi_wstrb;
            force m_axi_wlast   = smoke_axi_wlast;
            force m_axi_wvalid  = smoke_axi_wvalid;
            force m_axi_bready  = smoke_axi_bready;

            prefill_mode = 1'b1;
            repeat(10) @(posedge c0_ddr4_ui_clk);
            prefill_ddr4_stream(real_image, real_file_offset, real_ddr_base,
                                real_bytes, 1'b1, loaded_bytes);
            if (loaded_bytes != real_bytes)
                $fatal(1, "Real image load count mismatch");

            // Required native checks: first, middle, last, a 4 KiB boundary,
            // and 64 deterministic pseudo-random aligned words.
            for (int fixed_index = 0; fixed_index < 4; fixed_index++) begin
                case (fixed_index)
                    0: check_offset = 0;
                    1: check_offset = ((real_bytes / 2) / 8) * 8;
                    2: check_offset = ((real_bytes - 8) / 8) * 8;
                    default: check_offset = (real_bytes >= 4096) ? 4088 : 0;
                endcase
                read_expected64(real_image, real_file_offset + check_offset, expected64);
                ddr4_mig_read(real_ddr_base + check_offset, actual64);
                if (actual64 !== expected64)
                    $fatal(1, "Native real-image mismatch offset=%0d expected=%016x actual=%016x",
                           check_offset, expected64, actual64);
            end

            random_state = 32'h4d49_4731;
            word_count = real_bytes / 8;
            for (int sample = 0; sample < 64; sample++) begin
                random_state = random_state * 32'd1664525 + 32'd1013904223;
                check_offset = (random_state % word_count) * 8;
                read_expected64(real_image, real_file_offset + check_offset, expected64);
                ddr4_mig_read(real_ddr_base + check_offset, actual64);
                if (actual64 !== expected64)
                    $fatal(1, "Random native mismatch sample=%0d offset=%0d", sample, check_offset);
            end
            $display("[TB][MIG_REAL][NATIVE] PASS fixed=4 random=64 bytes=%0d", real_bytes);

            prefill_mode = 1'b0;
            repeat(20) @(posedge c0_ddr4_ui_clk);
            for (int axi_index = 0; axi_index < 4; axi_index++) begin
                case (axi_index)
                    0: check_offset = 0;
                    1: check_offset = ((real_bytes / 2) / 16) * 16;
                    2: check_offset = ((real_bytes - 16) / 16) * 16;
                    default: check_offset = (real_bytes >= 4096) ? 4080 : 0;
                endcase
                read_expected128(real_image, real_file_offset + check_offset, expected128);
                axi_smoke_read(real_ddr_base + check_offset, actual128);
                if (actual128 !== expected128)
                    $fatal(1, "AXI real-image mismatch offset=%0d expected=%032x actual=%032x",
                           check_offset, expected128, actual128);
            end
            $display("[TB][MIG_REAL][AXI] PASS fixed=4 bytes=%0d", real_bytes);
            $display("[TB][MIG_REAL] PASS image=%s file_offset=%0d ddr_base=0x%08x bytes=%0d",
                     real_image, real_file_offset, real_ddr_base, real_bytes);
            $finish;
        end

        // ================================================================
        // MIG_REAL_COMPUTE: run the real q_proj N-tile0 GEMM end-to-end.
        // Loads the real activation (DDR 0x0), the real q_proj N-tile0 WRF
        // (DDR 0x10000) and the real scale/zero (DDR 0x410000) through the
        // MIG front door, loads the real ucode, triggers LOAD_SCALE, starts
        // the NPU, waits for done and compares the acc output vs the Python
        // golden file (sim/modelsim/real_ntile0_golden.bin).
        // ================================================================
        if ($test$plusargs("MIG_REAL_COMPUTE")) begin : mig_real_compute
            string real_image;
            string ucode_file, golden_file;
            integer loaded_bytes;
            real npu_val, gold_val, max_abs0, max_abs1, d0, d1;
            real ma_r0c, ma_r1c, ma_c0r, ma_c1r;
            integer fd_gold, g;
            integer cycle_cnt;
            logic [31:0] npu0 [0:31];
            logic [31:0] npu1 [0:31];
            logic ever_gld_w, ever_gld_a, ever_wrf_wr, ever_arf_wr;
            logic ever_dma_rd, ever_dma_rx_wr;
            integer gldw_cnt, wrfwr_cnt, glda_cnt, arfwr_cnt, dmard_cnt;
            integer dma_rx_sample;
            integer dq_fire_cnt;
            integer store_dump_cnt;
            logic prev_wrf_bank_sel;
            integer ev_trace_cnt;
            integer sz_seen_cnt;
            integer sz_seen_addr [0:31];
            integer sz_seen_base [0:31];
            integer sz_seen_iss  [0:31];

            if (!$value$plusargs("REAL_IMAGE=%s", real_image))
                $fatal(1, "+REAL_IMAGE=<path> is required");
            if (!$value$plusargs("UCODE_FILE=%s", ucode_file))
                ucode_file = "mig_real_qproj_ntile0.bin";
            if (!$value$plusargs("GOLDEN_FILE=%s", golden_file))
                golden_file = "real_ntile0_golden.bin";

            prefill_mode = 1'b1;
            repeat(10) @(posedge c0_ddr4_ui_clk);

            $display("[TB][MIG_REAL_COMPUTE] streaming real layer-0 q_proj inputs to DDR...");
            prefill_ddr4_stream(real_image, 0, 32'h0000_0000, 4096, 1'b1, loaded_bytes);
            prefill_ddr4_stream(real_image, 32'h0001_0000, 32'h0001_0000,
                                65536, 1'b1, loaded_bytes);
            prefill_ddr4_stream(real_image, 32'h0041_0000, 32'h0041_0000,
                                2048, 1'b1, loaded_bytes);

            $display("[TB][MIG_REAL_COMPUTE] DDR pre-fill complete");
            repeat(10) @(posedge c0_ddr4_ui_clk);
            prefill_mode = 1'b0;
            repeat(20) @(posedge clk);

            $display("[TB][MIG_REAL_COMPUTE] Loading ucode (%0s)...", ucode_file);
            load_ucode_file(ucode_file);
            repeat(5) @(posedge clk);

            $display("[TB][MIG_REAL_COMPUTE] Configuring registers...");
            axil_write(32'h0C, ucode_size / 4); // UCODE_LEN = instruction count
            axil_write(32'h10, 32'h0000_0000);   // DDR_BASE_ACT
            axil_write(32'h14, 32'h0000_0000);   // DDR_BASE_WGT (imm carries 0x10000)
            axil_write(32'h1C, 32'h0008_0000);   // DDR_BASE_OUT (scratch)
            axil_write(32'h48, 32'h0041_0000);   // DDR_BASE_SCALE
            axil_write(32'h4C, 32'd1);           // CFG_M = seq_len
            axil_write(32'h50, 32'd32);          // CFG_N = 32 (one N-tile)
            axil_write(32'h54, 32'd32);          // CFG_K (per K-tile)
            axil_write(32'h78, 32'd4);           // DEQUANT_CFG: nbits=4
            axil_write(32'h7C, 32'd64);          // DEQUANT_SCALE_LEN: 64 beats
            repeat(5) @(posedge clk);

            $display("[TB][MIG_REAL_COMPUTE] Triggering LOAD_SCALE...");
            axil_write(32'h58, 32'h0000_0008);   // HOST_CMD bit3 = LOAD_SCALE
            cycle_cnt = 0;
            while (!u_npu.sc_load_done && cycle_cnt < 10000) begin
                @(posedge clk); cycle_cnt = cycle_cnt + 1;
            end
            $display("[TB][MIG_REAL_COMPUTE] scale DMA done after %0d cycles",
                     cycle_cnt);
            repeat(5) @(posedge clk);

            $display("[TB][MIG_REAL_COMPUTE] Starting NPU...");
            axil_write(32'h00, 32'h0000_0001);   // START

            cycle_cnt = 0;
            gldw_cnt = 0; wrfwr_cnt = 0; glda_cnt = 0; arfwr_cnt = 0; dmard_cnt = 0;
            dma_rx_sample = 0;
            store_dump_cnt = 0;
            prev_wrf_bank_sel = 1'b0;
            ev_trace_cnt = 0;
            dq_fire_cnt = 0;
            sz_seen_cnt = 0;
            for (int si = 0; si < 32; si++) begin
                sz_seen_addr[si] = 0;
                sz_seen_base[si] = 0;
                sz_seen_iss[si]  = 0;
            end
            ever_gld_w = 1'b0; ever_gld_a = 1'b0; ever_wrf_wr = 1'b0;
            ever_arf_wr = 1'b0; ever_dma_rd = 1'b0; ever_dma_rx_wr = 1'b0;
            while (!u_npu.npu_done_latch && cycle_cnt < 2000000) begin
                @(posedge clk); cycle_cnt = cycle_cnt + 1;
                if (u_npu.u_decode.gld_w_cmd_valid) begin
                    gldw_cnt = gldw_cnt + 1; ever_gld_w = 1'b1;
                end
                if (u_npu.u_decode.gld_a_cmd_valid) begin
                    glda_cnt = glda_cnt + 1; ever_gld_a = 1'b1;
                end
                if (u_npu.u_gld_ctrl.wrf_wr_en) begin
                    wrfwr_cnt = wrfwr_cnt + 1; ever_wrf_wr = 1'b1;
                end
                if (u_npu.u_gld_ctrl.arf_wr_en) begin
                    arfwr_cnt = arfwr_cnt + 1; ever_arf_wr = 1'b1;
                end
                if (u_npu.u_decode.dma_rd_cmd_valid) begin
                    dmard_cnt = dmard_cnt + 1; ever_dma_rd = 1'b1;
                end
                if (u_npu.dma_rx_wr_en) ever_dma_rx_wr = 1'b1;
                // Sample first few DMA L1 writes (addr + data) to isolate the
                // DMA->L1 path from the GLD path.
                if (u_npu.dma_rx_wr_en && dma_rx_sample < 4) begin
                    $display("[TB][MIG_REAL_COMPUTE] DMA_WR sample%0d l1addr=0x%03x data=%032x",
                             dma_rx_sample, u_npu.r_dma_rx_addr,
                             u_npu.dma_rx_wr_data);
                    dma_rx_sample = dma_rx_sample + 1;
                end
                // Cycle-accurate ordering trace: GLD issue, GLD done, GEMM
                // issue/done, DMA done, WRF bank_sel flip. Cap at 60 events.
                if (ev_trace_cnt < 60 && (u_npu.gld_w_cmd_valid ||
                                          u_npu.weight_rf_done ||
                                          u_npu.gemm_cmd_valid ||
                                          u_npu.gemm_done ||
                                          u_npu.dma_rd_done ||
                                          (u_npu.wrf_bank_sel != prev_wrf_bank_sel))) begin
                    $display("[TB][TRACE] cyc=%0d %s%s%s%s%s%s wrf_bs=%b pc=%0d",
                             cycle_cnt,
                             u_npu.gld_w_cmd_valid   ? "GLDW " : "",
                             u_npu.weight_rf_done    ? "GLDWDONE " : "",
                             u_npu.gemm_cmd_valid    ? "GEMMISS " : "",
                             u_npu.gemm_done         ? "GEMMDONE " : "",
                             u_npu.dma_rd_done       ? "DMARDONE " : "",
                             (u_npu.wrf_bank_sel != prev_wrf_bank_sel) ? "FLIP " : "",
                             u_npu.wrf_bank_sel, u_npu.u_fetch.pc);
                    prev_wrf_bank_sel = u_npu.wrf_bank_sel;
                    ev_trace_cnt = ev_trace_cnt + 1;
                end
                // At each GEMM completion (full drain done, acc stable), dump the
                // accumulated sa_acc row0 so we can compare per-tile partial
                // sums vs golden (locates any missing/extra K-term in the MAC).
                if (u_npu.u_decode.gemm_cmd_valid && store_dump_cnt < 16)
                    $display("[TB][MIG_REAL_COMPUTE] GEMMISS flag cyc=%0d acc=%0d skip_store=%0d prec=%0d",
                             cycle_cnt, u_npu.u_decode.gemm_cmd_acc,
                             u_npu.u_decode.gemm_cmd_skip_store, u_npu.u_decode.gemm_cmd_prec);
                if (u_npu.sa_clear)
                    $display("[TB][MIG_REAL_COMPUTE] SACLEAR cyc=%0d gemm_busy=%0d acc_accumulate=%0d",
                             cycle_cnt, u_npu.u_gemm_ctrl.busy, u_npu.gemm_cmd_acc);
                if (u_npu.gemm_done && store_dump_cnt < 8) begin
                    string s;
                    s = "";
                    for (int gj2 = 0; gj2 < 32; gj2++)
                        $sformat(s, "%s %f", s, $bitstoshortreal(u_npu.sa_acc[0][gj2]));
                    $display("[TB][MIG_REAL_COMPUTE] SAACC gemm_done dump%0d cyc=%0d =%0s",
                             store_dump_cnt, cycle_cnt, s);
                    store_dump_cnt = store_dump_cnt + 1;
                end
                if (u_npu.l1_wr_en &&
                    u_npu.l1_wr_addr >= 11'h178 && u_npu.l1_wr_addr <= 11'h18F)
                    $display("[TB][MIG_REAL_COMPUTE] L1WATCH cyc=%0d addr=0x%03x data=%064x src_en(vv,dma)=(%b,%b)",
                             cycle_cnt, u_npu.l1_wr_addr, u_npu.l1_wr_data,
                             u_npu.vv_l1_wr_en, u_npu.dma_rx_wr_en);
                // Capture the first dequant output fire (32 lanes, first K of
                // the first GEMM) to compare against the Python golden deq.
                // Capture scale_rd_ctrl trigger context: on each GLD.W start,
                // record group/expert/proj + the 4 read addresses issued.
                if (u_npu.gld_w_cmd_valid && gldw_cnt <= 4) begin
                    $display("[TB][MIG_REAL_COMPUTE] GLDW%0d trg grp=%0d exp=%0d proj=%0d",
                             gldw_cnt, u_npu.gld_w_cmd_group,
                             u_npu.gld_w_cmd_expert_sel, u_npu.gld_w_cmd_proj);
                end
                if (u_npu.u_scale_rd_ctrl.state == 2'd1) begin  // S_READ
                    if (sz_seen_cnt < 32) begin
                        sz_seen_addr[sz_seen_cnt] = u_npu.u_scale_rd_ctrl.rd_addr;
                        sz_seen_base[sz_seen_cnt] = u_npu.u_scale_rd_ctrl.base;
                        sz_seen_iss [sz_seen_cnt] = u_npu.u_scale_rd_ctrl.issued;
                        sz_seen_cnt = sz_seen_cnt + 1;
                    end
                end
                // Capture dequant fire details for the first fire.
                // At the first fire of GEMM1 (fire32), snapshot the WRF bank_sel
                // and the L1 word GLD_W tile1 reads from (0x100), so we can see
                // exactly which data set GEMM1's K0 consumes.
                if (u_npu.dq_out_valid && dq_fire_cnt == 32) begin
                    $display("[TB][MIG_REAL_COMPUTE] fire32 snapshot wrf_bank_sel=%b acc_bank_sel=%b",
                             u_npu.wrf_bank_sel, u_npu.acc_bank_sel);
                    $display("[TB][MIG_REAL_COMPUTE] fire32 L1@0x100 b0..b7 = %08x %08x %08x %08x %08x %08x %08x %08x",
                             u_npu.u_l1_cache.bank_mem[0][64],
                             u_npu.u_l1_cache.bank_mem[1][64],
                             u_npu.u_l1_cache.bank_mem[2][64],
                             u_npu.u_l1_cache.bank_mem[3][64],
                             u_npu.u_l1_cache.bank_mem[4][64],
                             u_npu.u_l1_cache.bank_mem[5][64],
                             u_npu.u_l1_cache.bank_mem[6][64],
                             u_npu.u_l1_cache.bank_mem[7][64]);
                    $display("[TB][MIG_REAL_COMPUTE] fire32 WRF banks  b0..b15@K0 = %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x",
                             u_npu.u_weight_rf.banks[0][0], u_npu.u_weight_rf.banks[1][0],
                             u_npu.u_weight_rf.banks[2][0], u_npu.u_weight_rf.banks[3][0],
                             u_npu.u_weight_rf.banks[4][0], u_npu.u_weight_rf.banks[5][0],
                             u_npu.u_weight_rf.banks[6][0], u_npu.u_weight_rf.banks[7][0],
                             u_npu.u_weight_rf.banks[8][0], u_npu.u_weight_rf.banks[9][0],
                             u_npu.u_weight_rf.banks[10][0], u_npu.u_weight_rf.banks[11][0],
                             u_npu.u_weight_rf.banks[12][0], u_npu.u_weight_rf.banks[13][0],
                             u_npu.u_weight_rf.banks[14][0], u_npu.u_weight_rf.banks[15][0]);
                    $display("[TB][MIG_REAL_COMPUTE] fire32 WRF banks_p b0..b15@K0 = %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x",
                             u_npu.u_weight_rf.banks_p[0][0], u_npu.u_weight_rf.banks_p[1][0],
                             u_npu.u_weight_rf.banks_p[2][0], u_npu.u_weight_rf.banks_p[3][0],
                             u_npu.u_weight_rf.banks_p[4][0], u_npu.u_weight_rf.banks_p[5][0],
                             u_npu.u_weight_rf.banks_p[6][0], u_npu.u_weight_rf.banks_p[7][0],
                             u_npu.u_weight_rf.banks_p[8][0], u_npu.u_weight_rf.banks_p[9][0],
                             u_npu.u_weight_rf.banks_p[10][0], u_npu.u_weight_rf.banks_p[11][0],
                             u_npu.u_weight_rf.banks_p[12][0], u_npu.u_weight_rf.banks_p[13][0],
                             u_npu.u_weight_rf.banks_p[14][0], u_npu.u_weight_rf.banks_p[15][0]);
                end
                // Full capture: stamp EVERY dequant fire with cycle and the
                // GEMM internal gather/feed K counters, so we can map fire N to
                // (gemm index, K index) unambiguously.
                if (u_npu.dq_out_valid && dq_fire_cnt < 160) begin
                    $display("[TB][MIG_REAL_COMPUTE] DEQ cyc=%0d fire%0d gK=%0d fK=%0d gemmstate=%0d lanes[0:31]=%064h",
                             cycle_cnt, dq_fire_cnt,
                             u_npu.u_gemm_ctrl.r_gather_k,
                             u_npu.u_gemm_ctrl.r_feed_k,
                             u_npu.u_gemm_ctrl.state,
                             u_npu.dq_out_data);
                    // At each GEMM's first K (gK==0), dump the active ARF set so
                    // we can compare the activation the MAC consumed vs golden.
                    if (u_npu.u_gemm_ctrl.r_gather_k == 0) begin
                        string s;
                        $display("[TB][MIG_REAL_COMPUTE] ARFDUMP fire%0d arf_bank_sel=%b (banks, K0..K15 cols0)",
                                 dq_fire_cnt, u_npu.arf_bank_sel);
                        for (int e = 0; e < 32; e++) begin
                            s = "";
                            for (int bi = 0; bi < 16; bi++)
                                $sformat(s, "%s %04x", s, u_npu.u_act_rf.banks[bi][e]);
                            $display("[TB][MIG_REAL_COMPUTE] ARFDUMP banks K%0d =%0s", e, s);
                            s = "";
                            for (int bi = 0; bi < 16; bi++)
                                $sformat(s, "%s %04x", s, u_npu.u_act_rf.banks_p[bi][e]);
                            $display("[TB][MIG_REAL_COMPUTE] ARFDUMP banks_p K%0d =%0s", e, s);
                        end
                    end
                    dq_fire_cnt = dq_fire_cnt + 1;
                end
                if (cycle_cnt % 200000 == 0) begin
                    $display($sformatf("[TB][MIG_REAL_COMPUTE] diag cycle=%0d fetch_pc=%0d fetch_state=%0d instr_valid=%b instr_ready=%b barrier=%b res_busy=%02x lc_state=%0d lc_valid=%b lc_instr=%08x dma_rd_busy=%b dma_wr_busy=%b sc_load_active=%b gemm_busy=%b",
                             cycle_cnt, u_npu.u_fetch.pc,
                             u_npu.u_fetch.state_q,
                             u_npu.u_fetch.instr_valid,
                             u_npu.u_decode.instr_ready,
                             u_npu.u_decode.barrier_active,
                             u_npu.res_busy,
                             u_npu.u_loop_ctrl.state,
                             u_npu.u_loop_ctrl.dec_valid,
                             u_npu.u_loop_ctrl.dec_instr,
                             u_npu.dma_rd_busy, u_npu.dma_wr_busy,
                             u_npu.sc_load_active, u_npu.u_gemm_ctrl.busy));
                end
            end
            if (!u_npu.npu_done_latch) begin
                $display($sformatf("[TB][MIG_REAL_COMPUTE] TIMEOUT diag: pc=%0d dma_rd_busy=%b dma_wr_busy=%b dma_rd_done=%b fetch_state=%0d",
                         u_npu.u_fetch.pc, u_npu.dma_rd_busy, u_npu.dma_wr_busy,
                         u_npu.dma_rd_done, u_npu.u_fetch.state_q));
                $fatal(1, "MIG_REAL_COMPUTE timeout after %0d cycles", cycle_cnt);
            end
            $display("[TB][MIG_REAL_COMPUTE] COUNTS gld_w=%0d gld_a=%0d wrf_wr=%0d arf_wr=%0d dma_rd=%0d",
                     gldw_cnt, glda_cnt, wrfwr_cnt, arfwr_cnt, dmard_cnt);
            $display("[TB][MIG_REAL_COMPUTE] NPU DONE at cycle %0d", cycle_cnt);
            repeat(20) @(posedge clk);
            $display("[TB][MIG_REAL_COMPUTE] SZTRACE seen=%0d addr_base_pairs: %0d/%0d %0d/%0d %0d/%0d %0d/%0d %0d/%0d %0d/%0d %0d/%0d %0d/%0d",
                     sz_seen_cnt,
                     sz_seen_addr[0], sz_seen_base[0], sz_seen_addr[1], sz_seen_base[1],
                     sz_seen_addr[2], sz_seen_base[2], sz_seen_addr[3], sz_seen_base[3],
                     sz_seen_addr[4], sz_seen_base[4], sz_seen_addr[5], sz_seen_base[5],
                     sz_seen_addr[6], sz_seen_base[6], sz_seen_addr[7], sz_seen_base[7]);

            // ---- ever-asserted flags (sampled in the main loop) ---------------
            begin
                $display("[TB][MIG_REAL_COMPUTE] EVER gld_w_cmd_valid=%b gld_a_cmd_valid=%b wrf_wr_en=%b arf_wr_en=%b",
                         ever_gld_w, ever_gld_a, ever_wrf_wr, ever_arf_wr);
                $display("[TB][MIG_REAL_COMPUTE] EVER dma_rd_cmd_valid=%b dma_rx_wr_en=%b",
                         ever_dma_rd, ever_dma_rx_wr);
                $display("[TB][MIG_REAL_COMPUTE] gld_ctrl state=%0d busy=%b",
                         u_npu.u_gld_ctrl.state, u_npu.u_gld_ctrl.busy);
                $display("[TB][MIG_REAL_COMPUTE] bank_sel wrf=%b arf=%b acc=%b deq_nbits=%0d gemm_prec=%0d",
                         u_npu.wrf_bank_sel, u_npu.arf_bank_sel,
                         u_npu.acc_bank_sel, u_npu.reg_deq_nbits_code,
                         u_npu.gemm_prec_eff);
                // Cross-check DDR contents vs L1: read back DDR 0x0 (activation)
                // and 0x10000 (weight) through the native MIG port.
                begin : ddr_check
                    logic [63:0] w0, w1, w2, w3;
                    prefill_mode = 1'b1;
                    repeat(10) @(posedge c0_ddr4_ui_clk);
                    ddr4_mig_read(29'h0000_0000, w0);
                    ddr4_mig_read(29'h0000_0008, w1);
                    ddr4_mig_read(29'h0000_0010, w2);
                    ddr4_mig_read(29'h0000_0018, w3);
                    $display("[TB][MIG_REAL_COMPUTE] DDR act@0x0 = %016x %016x %016x %016x",
                             w0, w1, w2, w3);
                    ddr4_mig_read(29'h0001_0000, w0);
                    ddr4_mig_read(29'h0001_0008, w1);
                    ddr4_mig_read(29'h0001_0010, w2);
                    ddr4_mig_read(29'h0001_0018, w3);
                    $display("[TB][MIG_REAL_COMPUTE] DDR wgt@0x10000 = %016x %016x %016x %016x",
                             w0, w1, w2, w3);
                    prefill_mode = 1'b0;
                    repeat(10) @(posedge clk);
                end
                // scale_zero_mem: dump beats 0..7 (256-bit each) as fp16 lanes
                begin : sz_dump
                    string s;
                    for (int bi = 0; bi < 8; bi++) begin
                        s = "";
                        for (int li = 0; li < 8; li++)
                            $sformat(s, "%s %04x", s,
                                     u_npu.u_scale_zero_mem.mem[bi][li*16 +: 16]);
                        $display("[TB][MIG_REAL_COMPUTE] DBG szmem[%0d]=%0s", bi, s);
                    end
                    $display("[TB][MIG_REAL_COMPUTE] DBG gld_w_cmd_group=%0d sz_base=%0d sz_rd_addr=%0d",
                             u_npu.gld_w_cmd_group, u_npu.u_scale_rd_ctrl.base,
                             u_npu.szm_rd_addr);
                end
            end

            // ---- debug dumps -------------------------------------------------
            begin : dbg_dump
                string s;
                // L1 bank1/bank2 weight locations: tile0/2 -> 0x80, tile1/3 -> 0x100.
                s = "";
                for (int bi = 0; bi < 8; bi++)
                    $sformat(s, "%s %08x", s, u_npu.u_l1_cache.bank_mem[bi][32]);
                $display("[TB][MIG_REAL_COMPUTE] DBG l1 w0x80(32)  =%0s", s);
                s = "";
                for (int bi = 0; bi < 8; bi++)
                    $sformat(s, "%s %08x", s, u_npu.u_l1_cache.bank_mem[bi][64]);
                $display("[TB][MIG_REAL_COMPUTE] DBG l1 w0x100(64) =%0s", s);
                s = "";
                for (int bi = 0; bi < 8; bi++)
                    $sformat(s, "%s %08x", s, u_npu.u_l1_cache.bank_mem[bi][96]);
                $display("[TB][MIG_REAL_COMPUTE] DBG l1 act0x180(96)=%0s", s);
                // WRF both sets bank 0..7 @K0
                s = "";
                for (int bi = 0; bi < 8; bi++)
                    $sformat(s, "%s %08x", s, u_npu.u_weight_rf.banks[bi][0]);
                $display("[TB][MIG_REAL_COMPUTE] DBG wrf.banks[0:7]@K0 =%0s", s);
                s = "";
                for (int bi = 0; bi < 8; bi++)
                    $sformat(s, "%s %08x", s, u_npu.u_weight_rf.banks_p[bi][0]);
                $display("[TB][MIG_REAL_COMPUTE] DBG wrf.banks_p[0:7]@K0 =%0s", s);
                // ARF both sets
                s = "";
                for (int bi = 0; bi < 8; bi++)
                    $sformat(s, "%s %04x", s, u_npu.u_act_rf.banks[bi][0]);
                $display("[TB][MIG_REAL_COMPUTE] DBG arf.banks[0:7]@K0 =%0s", s);
                s = "";
                for (int bi = 0; bi < 8; bi++)
                    $sformat(s, "%s %04x", s, u_npu.u_act_rf.banks_p[bi][0]);
                $display("[TB][MIG_REAL_COMPUTE] DBG arf.banks_p[0:7]@K0 =%0s", s);
            end

            // Read back the GEMM result. The last GEMM's store writes the
            // 32-lane accumulator into the acc_buffer bank that was active at
            // that time; bank_sel flips on gemm_done, and last_gemm_acc_bank
            // (sampled at the final gemm_done) identifies the result bank.
            // acc_buffer stores column banks: bankX_mem[col][row] =
            // sa_acc[row][col]. With CFG_M=1 the only meaningful row is 0, so
            // output lane gj is bankX_mem[gj][0] (col-major). Judge on this
            // single interpretation only: the row-major alias reads
            // never-accumulated entries and can false-PASS.
            max_abs0 = 0.0; max_abs1 = 0.0;
            begin : readback
                ma_r0c = 0.0; ma_r1c = 0.0; ma_c0r = 0.0; ma_c1r = 0.0;
                fd_gold = $fopen(golden_file, "rb");
                if (fd_gold == 0)
                    $fatal(1, "Cannot open golden file %0s", golden_file);
                for (g = 0; g < 32; g++) begin
                    logic [31:0] w32;
                    logic [7:0]  gb [0:3];   // little-endian fp32 bytes
                    integer fr;
                    real gv, nv, dd;
                    fr = $fread(gb, fd_gold);
                    if (fr != 4)
                        $fatal(1, "Golden file too short at word %0d (fr=%0d)", g, fr);
                    w32 = {gb[3], gb[2], gb[1], gb[0]};   // LE -> IEEE bit pattern
                    gv = $bitstoshortreal(w32);
                    // col-major, result bank selected by last_gemm_acc_bank
                    nv = last_gemm_acc_bank
                         ? $bitstoshortreal(u_npu.u_acc_buffer.bank1_mem[g][0])
                         : $bitstoshortreal(u_npu.u_acc_buffer.bank0_mem[g][0]);
                    dd = (nv > gv) ? (nv - gv) : (gv - nv); if (dd > ma_c0r) ma_c0r = dd;
                end
                $fclose(fd_gold);
                max_abs0 = ma_c0r;
                $display("[TB][MIG_REAL_COMPUTE] max_abs(col-major bank%0d)=%e",
                         last_gemm_acc_bank, ma_c0r);
            end
            $display("[TB][MIG_REAL_COMPUTE] best out_bank0[0:8] = %f %f %f %f %f %f %f %f",
                     $bitstoshortreal(u_npu.u_acc_buffer.bank0_mem[0][0]),
                     $bitstoshortreal(u_npu.u_acc_buffer.bank0_mem[1][0]),
                     $bitstoshortreal(u_npu.u_acc_buffer.bank0_mem[2][0]),
                     $bitstoshortreal(u_npu.u_acc_buffer.bank0_mem[3][0]),
                     $bitstoshortreal(u_npu.u_acc_buffer.bank0_mem[4][0]),
                     $bitstoshortreal(u_npu.u_acc_buffer.bank0_mem[5][0]),
                     $bitstoshortreal(u_npu.u_acc_buffer.bank0_mem[6][0]),
                     $bitstoshortreal(u_npu.u_acc_buffer.bank0_mem[7][0]));
            $display("[TB][MIG_REAL_COMPUTE] RTL  acc1 col-major[0:8] = %f %f %f %f %f %f %f %f",
                     $bitstoshortreal(u_npu.u_acc_buffer.bank1_mem[0][0]),
                     $bitstoshortreal(u_npu.u_acc_buffer.bank1_mem[1][0]),
                     $bitstoshortreal(u_npu.u_acc_buffer.bank1_mem[2][0]),
                     $bitstoshortreal(u_npu.u_acc_buffer.bank1_mem[3][0]),
                     $bitstoshortreal(u_npu.u_acc_buffer.bank1_mem[4][0]),
                     $bitstoshortreal(u_npu.u_acc_buffer.bank1_mem[5][0]),
                     $bitstoshortreal(u_npu.u_acc_buffer.bank1_mem[6][0]),
                     $bitstoshortreal(u_npu.u_acc_buffer.bank1_mem[7][0]));
            // Dump all four (bank x orientation) candidate outputs to a file
            // so Python can pick the best match precisely.
            begin : dump_out
                integer fd_out;
                fd_out = $fopen("real_ntile0_rtl_out.bin", "wb");
                // layout: [bank0 col-major 32][bank1 col-major 32]
                //         [bank0 row-major 32][bank1 row-major 32]
                for (int bnk = 0; bnk < 2; bnk++) begin
                    for (int oi = 0; oi < 32; oi++) begin
                        logic [31:0] ow;
                        if (bnk == 0) ow = u_npu.u_acc_buffer.bank0_mem[oi][0];
                        else          ow = u_npu.u_acc_buffer.bank1_mem[oi][0];
                        $fwrite(fd_out, "%c%c%c%c", ow[7:0], ow[15:8],
                                ow[23:16], ow[31:24]);
                    end
                end
                for (int bnk = 0; bnk < 2; bnk++) begin
                    for (int oi = 0; oi < 32; oi++) begin
                        logic [31:0] ow;
                        if (bnk == 0) ow = u_npu.u_acc_buffer.bank0_mem[0][oi];
                        else          ow = u_npu.u_acc_buffer.bank1_mem[0][oi];
                        $fwrite(fd_out, "%c%c%c%c", ow[7:0], ow[15:8],
                                ow[23:16], ow[31:24]);
                    end
                end
                $fclose(fd_out);
                $display("[TB][MIG_REAL_COMPUTE] wrote real_ntile0_rtl_out.bin with 2 banks x 2 orientations x 32 fp32");
            end
            if (max_abs0 <= 1.0e-2)
                $display("[TB][MIG_REAL_COMPUTE] PASS (AC-05) DONE cycle=%0d bank=%0d max_abs=%e",
                         cycle_cnt, last_gemm_acc_bank, max_abs0);
            else
                $fatal(1, "[TB][MIG_REAL_COMPUTE] FAIL: col-major bank%0d max_abs=%e exceeds 1e-2",
                       last_gemm_acc_bank, max_abs0);

            // ---- AC2V 通路检查 (+AC2V_CHECK, P1) ----
            // ucode 尾部已执行 AC2V×32 + VST×32 + DMA_WR×2，把 acc 结果集经
            // PRF/L1(0x8000..) 搬到 DDR_BASE_OUT+0x8000/0x8400。用 native MIG
            // 口读回，与 acc_buffer 结果集（acc_flush host 口同一读口来源）
            // 逐元素 fp32->fp16(RNE) 比对。kt4 为 M=1，仅 row 0 有效；
            // golden 含 X 的条目跳过计数。
            if ($test$plusargs("AC2V_CHECK")) begin : ac2v_check
                integer ac2v_mismatch, ac2v_checked, ac2v_xskip;
                logic [63:0] w64 [0:7];
                ac2v_mismatch = 0; ac2v_checked = 0; ac2v_xskip = 0;
                prefill_mode = 1'b1;
                repeat(10) @(posedge c0_ddr4_ui_clk);
                for (integer r = 0; r < 32; r++) begin
                    // DDR 行基址: DDR_BASE_OUT(0x80000) + 0x8000 + r*64
                    for (integer k = 0; k < 8; k++)
                        ddr4_mig_read(29'h0008_8000 + r*64 + k*8, w64[k]);
                    for (integer j = 0; j < 32; j++) begin
                        logic [31:0] acc_w;
                        logic [15:0] exp_h, act_h;
                        acc_w = last_gemm_acc_bank
                                ? u_npu.u_acc_buffer.bank1_mem[j][r]
                                : u_npu.u_acc_buffer.bank0_mem[j][r];
                        if (^acc_w === 1'bx) begin
                            ac2v_xskip = ac2v_xskip + 1;
                        end else begin
                            exp_h = fp16_utils_pkg::fp32_to_fp16(acc_w);
                            act_h = w64[j>>2][(j%4)*16 +: 16];
                            ac2v_checked = ac2v_checked + 1;
                            if (act_h !== exp_h) begin
                                ac2v_mismatch = ac2v_mismatch + 1;
                                if (ac2v_mismatch <= 8)
                                    $display("[TB][AC2V_CHECK] MISMATCH row=%0d col=%0d exp=%04x got=%04x (acc_fp32=%08x)",
                                             r, j, exp_h, act_h, acc_w);
                            end
                        end
                    end
                end
                prefill_mode = 1'b0;
                repeat(10) @(posedge clk);
                if (ac2v_mismatch == 0 && ac2v_checked > 0)
                    $display("[TB][AC2V_CHECK] PASS checked=%0d mismatch=0 xskip=%0d (acc->PRF->L1->DMA->DDR fp16 逐元素一致)",
                             ac2v_checked, ac2v_xskip);
                else
                    $fatal(1, "[TB][AC2V_CHECK] FAIL checked=%0d mismatch=%0d xskip=%0d",
                           ac2v_checked, ac2v_mismatch, ac2v_xskip);
            end
            $finish;
        end

        // ================================================================
        // MIG_BATCH_GEMM: verify one independent activation per systolic row.
        // The fixture contains an M=32, K=32 activation tile in K-major order
        // and one FP16 KxN weight tile. This isolates the batched ARF/GEMM data
        // path from Top-4 routing so every output can be compared directly.
        // ================================================================
        if ($test$plusargs("MIG_BATCH_GEMM")) begin : mig_batch_gemm
            string act_file, weight_file, ucode_file, golden_file;
            integer loaded_bytes, cycle_cnt, fd_gold, fr, row, col, batch_m;
            integer batch_k;
            integer expected_gemms;
            logic [7:0] gb [0:3];
            logic [31:0] w32;
            real gv, nv, dd, max_abs;

            if (!$value$plusargs("BATCH_ACT_FILE=%s", act_file))
                act_file = "D:/graduation_design/npu-main/sim/modelsim/m32_batch_gemm/activations_kmajor.bin";
            if (!$value$plusargs("BATCH_WEIGHT_FILE=%s", weight_file))
                weight_file = "D:/graduation_design/npu-main/sim/modelsim/m32_batch_gemm/weights_kmajor.bin";
            if (!$value$plusargs("BATCH_UCODE_FILE=%s", ucode_file))
                ucode_file = "D:/graduation_design/npu-main/sim/modelsim/m32_batch_gemm/m32_batch_gemm_ucode.bin";
            if (!$value$plusargs("BATCH_GOLDEN_FILE=%s", golden_file))
                golden_file = "D:/graduation_design/npu-main/sim/modelsim/m32_batch_gemm/golden_fp32.bin";
            if (!$value$plusargs("BATCH_M=%d", batch_m)) batch_m = 32;
            if (!$value$plusargs("BATCH_K=%d", batch_k)) batch_k = 32;
            if (!$value$plusargs("BATCH_EXPECT_GEMMS=%d", expected_gemms))
                expected_gemms = 2;
            if (batch_m != 1 && batch_m != 32)
                $fatal(1, "MIG_BATCH_GEMM supports comparison M=1 or M=32, got %0d", batch_m);

            prefill_mode = 1'b1;
            repeat (10) @(posedge c0_ddr4_ui_clk);
            prefill_ddr4_stream_fast(act_file, 0, 32'h0000_0000,
                                     4096, 1'b1, loaded_bytes);
            if (loaded_bytes != 4096) $fatal(1, "batch activation prefill short");
            prefill_ddr4_stream_fast(weight_file, 0, 32'h0001_0000,
                                     6144, 1'b1, loaded_bytes);
            if (loaded_bytes != 6144) $fatal(1, "batch weight prefill short");
            repeat (10) @(posedge c0_ddr4_ui_clk);
            prefill_mode = 1'b0;

            load_ucode_file(ucode_file);
            repeat (5) @(posedge clk);
            axil_write(32'h0C, ucode_size / 4);
            axil_write(32'h10, 32'h0000_0000);   // activation DDR base
            axil_write(32'h14, 32'h0001_0000);   // weight DDR base
            axil_write(32'h4C, batch_m);         // CFG_M
            axil_write(32'h50, 32'd32);          // CFG_N
            axil_write(32'h54, batch_k);          // CFG_K
            axil_write(32'h00, 32'h0000_0001);   // START

            cycle_cnt = 0;
            while (!u_npu.npu_done_latch && cycle_cnt < 1000000) begin
                @(posedge clk);
                cycle_cnt++;
            end
            if (!u_npu.npu_done_latch)
                $fatal(1, "MIG_BATCH_GEMM timeout pc=%0d", u_npu.u_fetch.pc);
            if (mac_util_gemm_count != expected_gemms)
                $fatal(1, "MIG_BATCH_GEMM expected %0d GEMMs, got %0d",
                       expected_gemms, mac_util_gemm_count);
            $display("[TB][MIG_BATCH_GEMM] NPU DONE cycle=%0d", cycle_cnt);
            $display("[TB][MAC_UTIL] useful_MAC=%0d array_capacity_MAC_slots=%0d array_utilization=%0.3f%% enabled_PE_MAC_slots=%0d enabled_PE_utilization=%0.3f%%",
                     mac_util_useful_ops, mac_util_peak_ops,
                     100.0 * $itor(mac_util_useful_ops) / $itor(mac_util_peak_ops),
                     mac_util_enabled_pe_slots,
                     100.0 * $itor(mac_util_useful_ops) /
                     $itor(mac_util_enabled_pe_slots));
            $display("[TB][MULTIBUF_OVERLAP] GEMM_GLD_cycles=%0d GEMM_DMA_cycles=%0d",
                     mac_util_gld_overlap_cycles, mac_util_dma_overlap_cycles);

            fd_gold = $fopen(golden_file, "rb");
            if (fd_gold == 0) $fatal(1, "Cannot open batch golden file %0s", golden_file);
            max_abs = 0.0;
            for (row = 0; row < batch_m; row++) begin
                for (col = 0; col < 32; col++) begin
                    fr = $fread(gb, fd_gold);
                    if (fr != 4) $fatal(1, "Batch golden short at row=%0d col=%0d", row, col);
                    w32 = {gb[3], gb[2], gb[1], gb[0]};
                    gv = $bitstoshortreal(w32);
                    nv = last_gemm_acc_bank
                       ? $bitstoshortreal(u_npu.u_acc_buffer.bank1_mem[col][row])
                       : $bitstoshortreal(u_npu.u_acc_buffer.bank0_mem[col][row]);
                    dd = (nv > gv) ? nv - gv : gv - nv;
                    if (dd > max_abs) max_abs = dd;
                end
            end
            $fclose(fd_gold);
            if (max_abs > 1.0e-3)
                $fatal(1, "MIG_BATCH_GEMM numerical mismatch max_abs=%e", max_abs);
            if (batch_m == 32)
                $display("M32_BATCH_NUMERIC_PASS max_abs=%e", max_abs);
            else
                $display("M1_SINGLE_NUMERIC_PASS max_abs=%e", max_abs);
            $finish;
        end

        // ================================================================
        // MIG_EXPERT: run ONE N-tile of a routed-expert projection GEMM
        // (default: slot0 = expert 33, gate_proj, 2-bit) end-to-end through
        // the expert path: expert-described DMA_RD (indirect addressing) ->
        // NOP_SCALE (12-tile expert scale load) -> GLD_W (descriptor
        // precision override) -> GEMM -> acc_buffer vs Python golden
        // (verify_expert_gemm_golden.py).
        //
        // TopK is NOT re-run here: the tb forces topk_idx_arr = the four
        // image experts [33,52,22,47] and topk_done_latched (the router+topk
        // chain is already verified by MIG_ROUTER+ROUTER_CHECK_TOPK). All
        // expert-addressed instructions gate on topk_done_latched.
        // ================================================================
        if ($test$plusargs("MIG_EXPERT")) begin : mig_expert
            string expert_image, expert_ucode, expert_golden, edesc_file;
            integer loaded_bytes, cycle_cnt, expert_timeout_cycles;
            integer same_pc_stall_count, last_pc;
            integer gldw_cnt, gemm_cnt, dmard_cnt;
            integer fd_e, fd_gold, g, fr, ei, s, p;
            integer rdsz;
            logic [7:0]  edesc_bytes [0:180*12-1];
            logic [7:0]  sc_bytes    [0:2048-1];
            logic [31:0] ew0, ew1, ew2, w32;
            logic [7:0]  gb [0:3];
            logic [255:0] exp_beat;
            real gv, nv, dd, max_abs;
            integer sc_mismatch;
            // Expert window layout (real_layer_L0_manifest.json).
            localparam integer SLOT_BASE  = 53465088;   // window_base
            localparam integer SLOT_STRIDE = 8921088;   // slot stride
            localparam integer SC_OFF [0:2] = '{2883584, 5857280, 8830976};
            // slot0 gate weight bytes to stream: n_ktiles*1024 (+4KB DMA
            // over-read). Sized by +EXPERT_KTILES so small-K runs skip the
            // full 64KB stream (MIG front-door writes are slow).
            integer expert_ktiles, expert_gemm_k, expert_cfg_k, expert_final_cfg_k;
            integer expert_gemms_per_proj, wgt_bytes;
            integer expert_ntiles, expert_nt_start, expert_nt_stride, nt_i;
            // Chain mode (EXPERT_FORCE_TOPK=0): ucode runs router GEMM +
            // NOP_TOPK itself; tb streams the router act/weights and does NOT
            // force topk_idx_arr. TopK result is checked against
            // router_topk_ref.txt after NPU done.
            integer force_topk, stream_router, expert_topk_check;
            string  act_src;
            integer expert_slot, expert_proj, wgt_base, sc_base;
            // Compact all-resident MoE demo controls.  The legacy mode keeps
            // its sparse prefill behavior; EXPERT_PREFILL_ALL instead streams
            // one complete small image containing all 60 x 3 projections.
            integer expert_prefill_all, expert_image_bytes, expert_demo, expert_fast_prefill;
            integer expert_verify_image, expert_verify_full_image;
            integer image_verify_samples;
            integer expert_group_demo, expert_group_rows;
            string expert_group_golden;
            integer expert_demo_golden;
            integer expert_expect_gemm, expected_total_gemms;
            integer route0, route1, route2, route3;
            integer demo_desc_bad;
            integer fd_ref, tk, rc;
            integer group_fd, group_golden_size, group_row, group_col;
            integer group_byte_addr, group_line_addr, group_bank, group_bank_row;
            integer group_golden_index, group_mismatches;
            integer ref_idx [0:3];
            logic [15:0] ref_w [0:3];
            logic [15:0] group_rtl_bits, group_ref_bits;
            logic [31:0] group_word;
            string ref_tag;
            real w_rtl, w_ref, w_err, w_maxerr;
            real group_rtl_value, group_ref_value, group_abs_err, group_max_abs_err;
            logic topk_idx_ok;

            expert_group_monitor_enable = 1'b0;
            if (!$value$plusargs("REAL_IMAGE=%s", expert_image))
                expert_image = "D:/graduation_design/npu-main/sim/modelsim/real_layer_L0.bin";
            if (!$value$plusargs("EXPERT_UCODE=%s", expert_ucode))
                expert_ucode = "D:/graduation_design/npu-main/sim/modelsim/mig_expert_sel1_gate_nt0_kt4.bin";
            if (!$value$plusargs("EXPERT_GOLDEN=%s", expert_golden))
                expert_golden = "D:/graduation_design/npu-main/sim/modelsim/expert33_gate_ntile0_golden_kt4.bin";
            if (!$value$plusargs("EDESC_FILE=%s", edesc_file))
                edesc_file = "D:/graduation_design/npu-main/sim/modelsim/expert_desc.bin";
            if (!$value$plusargs("EXPERT_KTILES=%d", expert_ktiles))
                expert_ktiles = 4;
            if (!$value$plusargs("EXPERT_GEMM_K=%d", expert_gemm_k))
                expert_gemm_k = 32;
            if (!$value$plusargs("EXPERT_CFG_K=%d", expert_cfg_k))
                expert_cfg_k = expert_gemm_k;
            if (!$value$plusargs("EXPERT_FINAL_CFG_K=%d", expert_final_cfg_k))
                expert_final_cfg_k = expert_cfg_k;
            if (!$value$plusargs("EXPERT_GEMMS_PER_PROJ=%d", expert_gemms_per_proj))
                expert_gemms_per_proj = expert_ktiles;
            if ((expert_gemm_k != 32 && expert_gemm_k != 64 &&
                 expert_gemm_k != 128 && expert_gemm_k != 256) ||
                expert_gemms_per_proj * (expert_gemm_k / 32) != expert_ktiles)
                $fatal(1, "invalid expert GEMM K tiling: physical=%0d gemm_k=%0d chunks=%0d",
                       expert_ktiles, expert_gemm_k, expert_gemms_per_proj);
            if (!$value$plusargs("EXPERT_NTILES=%d", expert_ntiles))
                expert_ntiles = 1;
            if (!$value$plusargs("EXPERT_NT_START=%d", expert_nt_start))
                expert_nt_start = 0;
            // Bytes per N-tile in the image weight region
            // (K-tiles-per-N-tile * 1024): 65536 for K=2048, 45056 for K=1408.
            if (!$value$plusargs("EXPERT_NT_STRIDE=%d", expert_nt_stride))
                expert_nt_stride = 65536;
            if (!$value$plusargs("EXPERT_FORCE_TOPK=%d", force_topk))
                force_topk = 1;   // default: legacy forced-topk standalone mode
            if (!$value$plusargs("EXPERT_STREAM_ROUTER=%d", stream_router))
                stream_router = 0;
            if (!$value$plusargs("EXPERT_TOPK_CHECK=%d", expert_topk_check))
                expert_topk_check = 1;
            if (!$value$plusargs("EXPERT_PREFILL_ALL=%d", expert_prefill_all))
                expert_prefill_all = 0;
            if (!$value$plusargs("EXPERT_FAST_PREFILL=%d", expert_fast_prefill))
                expert_fast_prefill = 0;
            if (!$value$plusargs("EXPERT_IMAGE_BYTES=%d", expert_image_bytes))
                expert_image_bytes = 0;
            if (!$value$plusargs("EXPERT_VERIFY_IMAGE=%d", expert_verify_image))
                expert_verify_image = 0;
            if (!$value$plusargs("EXPERT_VERIFY_IMAGE_FULL=%d", expert_verify_full_image))
                expert_verify_full_image = 0;
            if (expert_verify_full_image)
                expert_verify_image = 1;
            if (!$value$plusargs("EXPERT_DEMO=%d", expert_demo))
                expert_demo = 0;
            if (!$value$plusargs("EXPERT_DEMO_GOLDEN=%d", expert_demo_golden))
                expert_demo_golden = 0;
            if (!$value$plusargs("EXPERT_GROUP_DEMO=%d", expert_group_demo))
                expert_group_demo = 0;
            expert_group_monitor_enable = (expert_group_demo != 0);
            if (!$value$plusargs("EXPERT_GROUP_ROWS=%d", expert_group_rows))
                expert_group_rows = 32;
            if (expert_group_rows < 1 || expert_group_rows > 32)
                $fatal(1, "EXPERT_GROUP_ROWS must be in [1,32]");
            if (!$value$plusargs("EXPERT_GROUP_GOLDEN=%s", expert_group_golden))
                expert_group_golden = "D:/graduation_design/npu-main/sim/modelsim/moe60_demo/moe60_grouped_golden_fp16.bin";
            if (!$value$plusargs("EXPERT_EXPECT_GEMM=%d", expert_expect_gemm))
                expert_expect_gemm = 0;
            if (!$value$plusargs("EXPERT_TIMEOUT_CYCLES=%d", expert_timeout_cycles))
                expert_timeout_cycles = 2000000;
            if (!$value$plusargs("EXPERT_ROUTE0=%d", route0)) route0 = 33;
            if (!$value$plusargs("EXPERT_ROUTE1=%d", route1)) route1 = 52;
            if (!$value$plusargs("EXPERT_ROUTE2=%d", route2)) route2 = 22;
            if (!$value$plusargs("EXPERT_ROUTE3=%d", route3)) route3 = 47;
            if (!$value$plusargs("EXPERT_ACT_FILE=%s", act_src))
                act_src = "";     // default: activation = image byte 0
            // Expert-sweep parameterization (task: other slots / projections).
            // wgt_base / sc_base default to slot0-gate; the scale-zero check
            // reads scale_zero_mem using the active compact/wide descriptor stride.
            if (!$value$plusargs("EXPERT_SLOT=%d", expert_slot))
                expert_slot = 0;
            if (!$value$plusargs("EXPERT_PROJ=%d", expert_proj))
                expert_proj = 0;
            if (!$value$plusargs("EXPERT_WGT_BASE=%d", wgt_base))
                wgt_base = SLOT_BASE;
            if (!$value$plusargs("EXPERT_SC_BASE=%d", sc_base))
                sc_base = SLOT_BASE + SC_OFF[0];
            wgt_bytes = expert_ktiles * 1024 + 4096;

            prefill_mode = 1'b1;
            repeat(10) @(posedge c0_ddr4_ui_clk);

            if (expert_prefill_all) begin
                if (expert_image_bytes <= 0)
                    $fatal(1, "EXPERT_PREFILL_ALL requires +EXPERT_IMAGE_BYTES=<positive bytes>");
                $display("[TB][MIG_EXPERT] starting all-resident image prefill: %0d bytes",
                         expert_image_bytes);
                if (expert_fast_prefill)
                    prefill_ddr4_stream_fast(expert_image, 0, 32'h0000_0000,
                                              expert_image_bytes, 1'b1, loaded_bytes);
                else
                    prefill_ddr4_stream(expert_image, 0, 32'h0000_0000,
                                        expert_image_bytes, 1'b1, loaded_bytes);
            end else begin
                $display("[TB][MIG_EXPERT] streaming act + slot0 gate weight (%0d B x %0d N-tile(s) from nt%0d) + scale tiles to DDR...",
                         wgt_bytes, expert_ntiles, expert_nt_start);
                if (act_src != "")
                    prefill_ddr4_stream(act_src, 0, 32'h0000_0000, 4096, 1'b1, loaded_bytes);
                else
                    prefill_ddr4_stream(expert_image, 0, 32'h0000_0000, 4096, 1'b1, loaded_bytes);
                if (stream_router) begin
                // Router fp16 weights (60x2048) -> DDR 0x10000, matching the
                // MIG_ROUTER layout; the chain ucode's router GEMM reads them.
                prefill_ddr4_stream(expert_image, 32'd17379328,
                                    32'h0001_0000, 262144, 1'b1, loaded_bytes);
                end
                for (nt_i = expert_nt_start; nt_i < expert_nt_start + expert_ntiles; nt_i++)
                    prefill_ddr4_stream(expert_image,
                                        wgt_base + nt_i*expert_nt_stride,
                                        wgt_base + nt_i*expert_nt_stride,
                                        wgt_bytes, 1'b1, loaded_bytes);
            // NOP_SCALE loads all 12 (slot,proj) scale tiles per N-tile, but
            // this test only consumes the selected slot/proj tile. Stream
            // the other 11 only under
            // +EXPERT_SCALE_FULL (unstreamed beats land X in scale_zero_mem
            // and are never read). Each N-tile's scale block is per-proj
            // compact: gate/up 64 words (2048 B), down 44 words (1408 B)
            // (K=1408 -> 11 groups; matches the compact image layout and the
            // descriptor-driven scale_loader stride).
                if ($test$plusargs("EXPERT_SCALE_FULL")) begin
                for (s = 0; s < 4; s++)
                    for (p = 0; p < 3; p++)
                        prefill_ddr4_stream(expert_image,
                                            SLOT_BASE + s*SLOT_STRIDE + SC_OFF[p]
                                                + expert_nt_start*((p==2)?1408:2048),
                                            SLOT_BASE + s*SLOT_STRIDE + SC_OFF[p]
                                                + expert_nt_start*((p==2)?1408:2048),
                                            expert_ntiles*((p==2)?1408:2048), 1'b1, loaded_bytes);
                end else begin
                prefill_ddr4_stream(expert_image,
                                    sc_base + expert_nt_start*((expert_proj==2)?1408:2048),
                                    sc_base + expert_nt_start*((expert_proj==2)?1408:2048),
                                    expert_ntiles*((expert_proj==2)?1408:2048), 1'b1, loaded_bytes);
                end
            end

            $display("[TB][MIG_EXPERT] DDR pre-fill complete");
            if (expert_prefill_all && expert_verify_image) begin
                // Let accepted write commands retire before issuing readbacks.
                repeat(32) @(posedge c0_ddr4_ui_clk);
                verify_ddr4_image(expert_image, expert_image_bytes,
                                  expert_verify_full_image ? 8 : 4096,
                                  image_verify_samples);
                if (image_verify_samples <= 0)
                    $fatal(1, "DDR image readback verification produced no samples");
            end
            repeat(10) @(posedge c0_ddr4_ui_clk);
            prefill_mode = 1'b0;
            repeat(20) @(posedge clk);

            // ---- Load all 180 expert descriptors through the AXI-Lite protocol ----
            fd_e = $fopen(edesc_file, "rb");
            if (fd_e == 0) $fatal(1, "Cannot open %0s", edesc_file);
            rdsz = $fread(edesc_bytes, fd_e);
            $fclose(fd_e);
            if (rdsz != 180*12) $fatal(1, "expert_desc.bin short: %0d bytes", rdsz);
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
            $display("[TB][MIG_EXPERT] loaded 180 expert descriptors");
            if (expert_demo) begin
                demo_desc_bad = 0;
                for (ei = 0; ei < 180; ei++) begin
                    if (u_npu.u_expert_desc_table.mem[ei] !==
                        {edesc_bytes[ei*12+11], edesc_bytes[ei*12+10],
                         edesc_bytes[ei*12+9],  edesc_bytes[ei*12+8],
                         edesc_bytes[ei*12+7],  edesc_bytes[ei*12+6],
                         edesc_bytes[ei*12+5],  edesc_bytes[ei*12+4],
                         edesc_bytes[ei*12+3],  edesc_bytes[ei*12+2],
                         edesc_bytes[ei*12+1],  edesc_bytes[ei*12+0]})
                        demo_desc_bad = demo_desc_bad + 1;
                end
                if (demo_desc_bad != 0)
                    $fatal(1, "[TB][MIG_EXPERT] demo descriptor load mismatch: %0d entries", demo_desc_bad);
                $display("[TB][MIG_EXPERT] DEMO_DESC_PASS all 180 descriptor entries loaded");
            end

            // ---- Force TopK results (image slots = experts 33/52/22/47) ----
            // Chain mode (EXPERT_FORCE_TOPK=0): the ucode's router GEMM +
            // NOP_TOPK produces topk_idx_arr autonomously; no force.
            if (force_topk) begin
                force u_npu.topk_idx_arr[0] = route0[5:0];
                force u_npu.topk_idx_arr[1] = route1[5:0];
                force u_npu.topk_idx_arr[2] = route2[5:0];
                force u_npu.topk_idx_arr[3] = route3[5:0];
                force u_npu.topk_done_latched = 1'b1;
                $display("[TB][MIG_EXPERT] topk forced: idx=%0d/%0d/%0d/%0d done=1",
                         route0, route1, route2, route3);
            end else
                $display("[TB][MIG_EXPERT] chain mode: topk NOT forced (ucode runs router+NOP_TOPK)");

            $display("[TB][MIG_EXPERT] Loading ucode (%0s)...", expert_ucode);
            load_ucode_file(expert_ucode);
            repeat(5) @(posedge clk);

            $display("[TB][MIG_EXPERT] Configuring registers...");
            axil_write(32'h0C, ucode_size / 4); // UCODE_LEN
            axil_write(32'h10, 32'h0000_0000);   // DDR_BASE_ACT
            axil_write(32'h14, 32'h0000_0000);   // DDR_BASE_WGT (desc carries base)
            axil_write(32'h1C, 32'h0008_0000);   // DDR_BASE_OUT (scratch)
            axil_write(32'h48, 32'h0041_0000);   // DDR_BASE_SCALE (unused: NOP_SCALE)
            axil_write(32'h4C, 32'd1);           // CFG_M = 1 token
            axil_write(32'h50, 32'd32);          // CFG_N = 32 (one N-tile)
            axil_write(32'h54, expert_cfg_k);    // host/default K; UCODE may override locally
            axil_write(32'h78, 32'd0);           // DEQUANT_CFG: no override (desc drives)
            axil_write(32'h7C, 32'd64);          // DEQUANT_SCALE_LEN: 64 beats
            repeat(5) @(posedge clk);

            $display("[TB][MIG_EXPERT] Starting NPU...");
            if (expert_ntiles > 1) begin
                // Enable per-N-tile acc snapshot monitor (module scope).
                exp_gd_cnt = 0;
                exp_snap_ktiles = expert_gemms_per_proj;
                $display("[TB][MIG_EXPERT] multi-N-tile mode: ntiles=%0d gemms_per_proj=%0d, snapshot per %0d gemm_done",
                         expert_ntiles, expert_gemms_per_proj, expert_gemms_per_proj);
            end
            group_stall_monitor_enable = (expert_group_demo != 0);
            axil_write(32'h00, 32'h0000_0001);   // START

            // START clears the Top-K route cache. Seed the test routes only
            // after that clear pulse has been consumed, but before the first
            // route-group instruction reaches decode.
            if (expert_group_demo) begin
                wait (!u_npu.start_pulse);
                for (tk = 0; tk < 32; tk++) begin
                    u_npu.u_topk_unit.route_valid[tk] = 1'b1;
                    u_npu.u_topk_unit.route_e[tk][0] =
                        (tk < expert_group_rows) ? route0[5:0] : route1[5:0];
                    u_npu.u_topk_unit.route_e[tk][1] =
                        (tk < expert_group_rows) ? route1[5:0] : route2[5:0];
                    u_npu.u_topk_unit.route_e[tk][2] =
                        (tk < expert_group_rows) ? route2[5:0] : route3[5:0];
                    u_npu.u_topk_unit.route_e[tk][3] =
                        (tk < expert_group_rows) ? route3[5:0] : (route0 + 1) % 60;
                    u_npu.u_topk_unit.route_w[tk][0] = 16'h3400;
                    u_npu.u_topk_unit.route_w[tk][1] = 16'h3400;
                    u_npu.u_topk_unit.route_w[tk][2] = 16'h3400;
                    u_npu.u_topk_unit.route_w[tk][3] = 16'h3400;
                end
                $display("[TB][MIG_EXPERT] seeded route cache after START clear");
            end

            cycle_cnt = 0;
            same_pc_stall_count = 0;
            last_pc = -1;
            gldw_cnt = 0; gemm_cnt = 0; dmard_cnt = 0;
            while (!u_npu.npu_done_latch && cycle_cnt < expert_timeout_cycles) begin
                @(posedge clk); cycle_cnt = cycle_cnt + 1;
                if (u_npu.execution_fault)
                    $fatal(1, "[TB][MIG_EXPERT] execution_fault at cycle=%0d pc=%0d route_error=%b topk_busy=%b route_group_count=%0d",
                           cycle_cnt, u_npu.u_fetch.pc, u_npu.route_error,
                           u_npu.topk_busy_r, u_npu.route_group_count);
                if (u_npu.u_decode.gld_w_cmd_valid)  gldw_cnt  = gldw_cnt + 1;
                if (u_npu.u_decode.gemm_cmd_valid)   gemm_cnt  = gemm_cnt + 1;
                if (u_npu.u_decode.dma_rd_cmd_valid) dmard_cnt = dmard_cnt + 1;
                if (u_npu.u_fetch.state_q == 3'd3 &&
                    u_npu.u_fetch.pc == last_pc && !u_npu.fetch_instr_ready) begin
                    same_pc_stall_count = same_pc_stall_count + 1;
                    if (same_pc_stall_count >= 100000)
                        $fatal(1, "[TB][MIG_EXPERT] stuck dispatch pc=%0d instr=%08x can_issue=%b ready=%b wrf_valid=%b arf_valid=%b topk_done=%b route_busy=%b res_busy=%02x",
                               u_npu.u_fetch.pc, u_npu.lc_dec_instr,
                               u_npu.u_decode.can_issue, u_npu.fetch_instr_ready,
                               u_npu.wrf_data_valid, u_npu.arf_data_valid,
                               u_npu.topk_done_latched, u_npu.topk_busy_r,
                               u_npu.res_busy);
                end else begin
                    last_pc = u_npu.u_fetch.pc;
                    same_pc_stall_count = 0;
                end
                if (cycle_cnt % 200000 == 0)
                    $display($sformatf("[TB][MIG_EXPERT] diag cycle=%0d pc=%0d fetch_state=%0d instr=%08x valid=%b ready=%b can_issue=%b wrf_valid=%b arf_valid=%b topk_done=%b route_busy=%b barrier=%b res_busy=%02x dma_rd_busy=%b sc_load_active=%b gemm_busy=%b gldw=%0d gemm=%0d",
                             cycle_cnt, u_npu.u_fetch.pc, u_npu.u_fetch.state_q,
                             u_npu.lc_dec_instr, u_npu.lc_dec_valid,
                             u_npu.fetch_instr_ready, u_npu.u_decode.can_issue,
                             u_npu.wrf_data_valid, u_npu.arf_data_valid,
                             u_npu.topk_done_latched, u_npu.topk_busy_r,
                             u_npu.u_decode.barrier_active, u_npu.res_busy,
                             u_npu.dma_rd_busy, u_npu.sc_load_active,
                             u_npu.u_gemm_ctrl.busy, gldw_cnt, gemm_cnt));
            end
            group_stall_monitor_enable = 1'b0;
            if (!u_npu.npu_done_latch) begin
                $display($sformatf("[TB][MIG_EXPERT] TIMEOUT diag: pc=%0d dma_rd_busy=%b fetch_state=%0d res_busy=%02x sc_load_active=%b gldw=%0d gemm=%0d dmard=%0d",
                         u_npu.u_fetch.pc, u_npu.dma_rd_busy, u_npu.u_fetch.state_q,
                         u_npu.res_busy, u_npu.sc_load_active, gldw_cnt, gemm_cnt, dmard_cnt));
                $fatal(1, "MIG_EXPERT timeout after %0d cycles", cycle_cnt);
            end
            $display("[TB][MIG_EXPERT] NPU DONE at cycle %0d (gldw=%0d gemm=%0d dmard=%0d)",
                     cycle_cnt, gldw_cnt, gemm_cnt, dmard_cnt);
            if (u_npu.cfg_k_active !== 16'(expert_final_cfg_k))
                $fatal(1, "[TB][MIG_EXPERT] CFG_K restore mismatch actual=%0d expected=%0d",
                       u_npu.cfg_k_active, expert_final_cfg_k);
            if (mac_util_peak_ops != 0)
                $display("[TB][MAC_UTIL] GEMMs=%0d useful_MAC=%0d array_capacity_MAC_slots=%0d array_utilization=%0.3f%% enabled_PE_MAC_slots=%0d enabled_PE_utilization=%0.3f%%",
                         mac_util_gemm_count, mac_util_useful_ops, mac_util_peak_ops,
                         100.0 * $itor(mac_util_useful_ops) / $itor(mac_util_peak_ops),
                         mac_util_enabled_pe_slots,
                         100.0 * $itor(mac_util_useful_ops) /
                         $itor(mac_util_enabled_pe_slots));
            $display("[TB][MIG_EXPERT] end-state gld_prec_eff=%0d gemm_prec_eff=%0d deq_nbits=%0d wrf_bs=%b acc_bs=%b last_gemm_acc_bank=%0d",
                     u_npu.gld_prec_eff, u_npu.gemm_prec_eff, u_npu.gemm_dq_nbits_code,
                     u_npu.wrf_bank_sel, u_npu.acc_bank_sel, last_gemm_acc_bank);

            // ---- Chain mode: verify the hardware-autonomous TopK result ----
            if (!force_topk && expert_topk_check) begin
                if (!u_npu.topk_done_latched)
                    $fatal(1, "[TB][MIG_EXPERT] CHAIN FAIL: topk_done_latched never set");
                fd_ref = $fopen("D:/graduation_design/npu-main/sim/modelsim/router_topk_ref.txt", "r");
                if (fd_ref == 0)
                    $fatal(1, "Cannot open router_topk_ref.txt");
                rc = $fscanf(fd_ref, "%s", ref_tag);          // "idx"
                for (tk = 0; tk < 4; tk++)
                    rc = $fscanf(fd_ref, "%d", ref_idx[tk]);
                rc = $fscanf(fd_ref, "%s", ref_tag);          // "val"
                for (tk = 0; tk < 4; tk++)
                    rc = $fscanf(fd_ref, "%s", ref_tag);      // skip 4 fp32 logits
                rc = $fscanf(fd_ref, "%s", ref_tag);          // "sw_fp16"
                for (tk = 0; tk < 4; tk++)
                    rc = $fscanf(fd_ref, "%h", ref_w[tk]);
                $fclose(fd_ref);

                topk_idx_ok = 1'b1;
                w_maxerr = 0.0;
                for (tk = 0; tk < 4; tk++) begin
                    if (u_npu.topk_idx_arr[tk] !== ref_idx[tk][5:0])
                        topk_idx_ok = 1'b0;
                    w_rtl = fp16_to_real(u_npu.topk_w_arr[tk]);
                    w_ref = fp16_to_real(ref_w[tk]);
                    w_err = (w_rtl > w_ref) ? (w_rtl - w_ref) : (w_ref - w_rtl);
                    if (w_err > w_maxerr) w_maxerr = w_err;
                    $display("[TB][MIG_EXPERT] chain topk[%0d] idx rtl=%0d ref=%0d w rtl=%h(%f) ref=%h(%f)",
                             tk, u_npu.topk_idx_arr[tk], ref_idx[tk],
                             u_npu.topk_w_arr[tk], w_rtl, ref_w[tk], w_ref);
                end
                if (topk_idx_ok && w_maxerr <= 2.0e-3)
                    $display("[TB][MIG_EXPERT] CHAIN TOPK PASS idx=%0d %0d %0d %0d w_maxerr=%e",
                             u_npu.topk_idx_arr[0], u_npu.topk_idx_arr[1],
                             u_npu.topk_idx_arr[2], u_npu.topk_idx_arr[3], w_maxerr);
                else
                    $fatal(1, "[TB][MIG_EXPERT] CHAIN TOPK FAIL idx_ok=%0d w_maxerr=%e (tol 2e-3)",
                           topk_idx_ok, w_maxerr);
            end else if (!force_topk) begin
                if (!u_npu.topk_done_latched)
                    $fatal(1, "[TB][MIG_EXPERT] TOPK FAIL: topk_done_latched never set");
                $display("[TB][MIG_EXPERT] TOPK PASS (hardware route): idx=%0d %0d %0d %0d",
                         u_npu.topk_idx_arr[0], u_npu.topk_idx_arr[1],
                         u_npu.topk_idx_arr[2], u_npu.topk_idx_arr[3]);
            end

            if (expert_group_demo) begin
                if (expert_expect_gemm != 4 * GROUP_DEMO_GEMMS_PER_EXPERT)
                    $fatal(1, "[TB][GROUPED_DEMO] expected GEMM argument %0d != projection geometry %0d",
                           expert_expect_gemm,
                           4 * GROUP_DEMO_GEMMS_PER_EXPERT);
                if (!group_prev_dma_valid ||
                    group_dma_chunk_count !=
                        (group_prev_dma_proj == 2'd2
                            ? GROUP_DEMO_DOWN_CHUNK_COUNT
                            : (GROUP_DEMO_DIM / 32) /
                              (GROUP_DEMO_GATE_UP_K / 32)))
                    $fatal(1, "[TB][GROUPED_DEMO] final K-chunk set incomplete: valid=%0d proj=%0d count=%0d",
                           group_prev_dma_valid, group_prev_dma_proj,
                           group_dma_chunk_count);
                if (gemm_cnt != expert_expect_gemm)
                    $fatal(1, "[TB][GROUPED_DEMO] GEMM count %0d != expected %0d",
                           gemm_cnt, expert_expect_gemm);
                if (group_explicit_k_dma_count !=
                    GROUP_DEMO_EXPECTED_K_OVERRIDE_COUNT)
                    $fatal(1, "[TB][GROUPED_DEMO] explicit mixed-K prefetch count %0d != expected %0d",
                           group_explicit_k_dma_count,
                           GROUP_DEMO_EXPECTED_K_OVERRIDE_COUNT);
                if (u_npu.route_group_count != 6'(expert_group_rows))
                    $fatal(1, "[TB][GROUPED_DEMO] route-group rows %0d != expected %0d",
                           u_npu.route_group_count, expert_group_rows);
                if (u_npu.rg_token_stride != 32'd1024 ||
                    u_npu.rg_transfer_bytes != 24'd512 ||
                    u_npu.rg_lines_per_row != 5'd16 ||
                    u_npu.rg_group_rows != 6'd32 ||
                    u_npu.rg_chunk_count != 7'd2 ||
                    u_npu.rg_chunk_index != 7'd1)
                    $fatal(1, "[TB][GROUPED_DEMO] streamed gather geometry mismatch stride=%0d bytes/chunk=%0d lines/row=%0d padded_rows=%0d chunk_count=%0d final_chunk=%0d",
                           u_npu.rg_token_stride, u_npu.rg_transfer_bytes,
                           u_npu.rg_lines_per_row, u_npu.rg_group_rows,
                           u_npu.rg_chunk_count, u_npu.rg_chunk_index);
                if (mac_util_max_m != expert_group_rows)
                    $fatal(1, "[TB][GROUPED_DEMO] active M max %0d != expected %0d",
                           mac_util_max_m, expert_group_rows);
                if (mac_util_peak_ops == 0)
                    $fatal(1, "[TB][GROUPED_DEMO] no enabled MAC cycles recorded");
                if (mac_util_useful_ops * 100 < mac_util_peak_ops * 60)
                    $fatal(1, "[TB][GROUPED_DEMO] full-array MAC utilization %0.3f%% is below 60%%",
                           100.0 * $itor(mac_util_useful_ops) /
                           $itor(mac_util_peak_ops));
                if (expert_group_rows == 32 &&
                    mac_util_useful_ops * 100 < mac_util_peak_ops * 75)
                    $fatal(1, "[TB][GROUPED_DEMO] 32-row feed MAC utilization %0.3f%% is below the 75%% grouped-schedule floor",
                           100.0 * $itor(mac_util_useful_ops) /
                           $itor(mac_util_peak_ops));
                if (mac_util_enabled_pe_slots == 0 ||
                    mac_util_useful_ops > mac_util_enabled_pe_slots)
                    $fatal(1, "[TB][GROUPED_DEMO] useful MAC work exceeds enabled PE slots");
                if (mac_util_wall_capacity_ops == 0 ||
                    mac_util_useful_ops > mac_util_wall_capacity_ops)
                    $fatal(1, "[TB][GROUPED_DEMO] useful MAC work exceeds GEMM wall-cycle capacity");
                if (mac_util_useful_ops * 100 <
                    mac_util_wall_capacity_ops * 50)
                    $fatal(1, "[TB][GROUPED_DEMO] GEMM wall-cycle MAC utilization is below 50%%");
                if (mac_util_dma_overlap_cycles == 0 ||
                    mac_util_gld_overlap_cycles == 0)
                    $fatal(1, "[TB][GROUPED_DEMO] weight ping-pong overlap missing (DMA=%0d GLD_W=%0d)",
                           mac_util_dma_overlap_cycles,
                           mac_util_gld_overlap_cycles);
                if (mac_util_dequant_feed_overlap_cycles !=
                    4 * GROUP_DEMO_DEQUANT_OVERLAP_PER_EXPERT)
                    $fatal(1, "[TB][GROUPED_DEMO] dequant/feed overlap cycles %0d != expected %0d",
                           mac_util_dequant_feed_overlap_cycles,
                           4 * GROUP_DEMO_DEQUANT_OVERLAP_PER_EXPERT);
                if (dequant_precision_seen != 3'b111)
                    $fatal(1, "[TB][GROUPED_DEMO] mixed-precision dequant coverage mask %b != 111",
                           dequant_precision_seen);
                if (mac_util_useful_ops !=
                    4 * longint'(expert_group_rows) * 32 *
                    GROUP_DEMO_MAC_K_SUM_PER_EXPERT)
                    $fatal(1, "[TB][GROUPED_DEMO] useful MAC count %0d disagrees with GEMM/M/K geometry",
                           mac_util_useful_ops);
                if (group_spill_gld_a_count !=
                    4 * (GROUP_DEMO_DIM / 32) *
                        GROUP_DEMO_SPILL_CHUNKS)
                    $fatal(1, "[TB][GROUPED_DEMO] intermediate spill GLD.A count %0d != %0d",
                           group_spill_gld_a_count,
                           4 * (GROUP_DEMO_DIM / 32) *
                               GROUP_DEMO_SPILL_CHUNKS);
                if (group_extension_gld_a_count !=
                    4 * ((GROUP_DEMO_DIM / 32) *
                         GROUP_DEMO_EXTENSION_CHUNKS -
                         (GROUP_DEMO_DIM / 64)))
                    $fatal(1, "[TB][GROUPED_DEMO] extension GLD.A count %0d != %0d",
                           group_extension_gld_a_count,
                           4 * ((GROUP_DEMO_DIM / 32) *
                                GROUP_DEMO_EXTENSION_CHUNKS -
                                (GROUP_DEMO_DIM / 64)));
                if (u_npu.execution_fault)
                    $fatal(1, "[TB][GROUPED_DEMO] execution_fault asserted");
                group_fd = $fopen(expert_group_golden, "rb");
                if (group_fd == 0)
                    $fatal(1, "[TB][GROUPED_DEMO] cannot open golden %0s", expert_group_golden);
                group_golden_size = $fread(group_golden_bytes, group_fd);
                $fclose(group_fd);
                if (group_golden_size != 32*GROUP_DEMO_DIM*2)
                    $fatal(1, "[TB][GROUPED_DEMO] golden size %0d != %0d",
                           group_golden_size, 32*GROUP_DEMO_DIM*2);

                group_mismatches = 0;
                group_max_abs_err = 0.0;
                for (group_row = 0; group_row < expert_group_rows; group_row++) begin
                    for (group_col = 0; group_col < GROUP_DEMO_DIM; group_col++) begin
                        // Accumulator is high-page, token-major FP16 output.
                        group_byte_addr = 32'h10000
                                        + group_row*GROUP_DEMO_DIM*2
                                        + group_col*2;
                        group_line_addr = group_byte_addr >> 5;
                        group_bank = ((group_line_addr & 3) << 3)
                                   + ((group_byte_addr >> 2) & 7);
                        group_bank_row = group_byte_addr >> 7;
                        group_word = u_npu.u_l1_cache.bank_mem[group_bank][group_bank_row];
                        group_rtl_bits = group_byte_addr[1]
                                       ? group_word[31:16] : group_word[15:0];
                        group_golden_index = (group_row*GROUP_DEMO_DIM + group_col)*2;
                        group_ref_bits = {group_golden_bytes[group_golden_index+1],
                                          group_golden_bytes[group_golden_index]};
                        group_rtl_value = fp16_to_real(group_rtl_bits);
                        group_ref_value = fp16_to_real(group_ref_bits);
                        group_abs_err = (group_rtl_value > group_ref_value)
                                      ? group_rtl_value - group_ref_value
                                      : group_ref_value - group_rtl_value;
                        if (group_abs_err > group_max_abs_err)
                            group_max_abs_err = group_abs_err;
                        if (group_abs_err > 0.05) begin
                            if (group_mismatches < 4)
                                $display("[TB][GROUPED_DEMO] mismatch token=%0d col=%0d rtl=%h(%f) ref=%h(%f) err=%f",
                                         group_row, group_col, group_rtl_bits,
                                         group_rtl_value, group_ref_bits,
                                         group_ref_value, group_abs_err);
                            group_mismatches = group_mismatches + 1;
                        end
                    end
                end
                if (group_mismatches != 0)
                    $fatal(1, "[TB][GROUPED_DEMO] %0d/%0d values exceed tolerance; max_abs=%f",
                           group_mismatches, expert_group_rows*GROUP_DEMO_DIM,
                           group_max_abs_err);
                $display("[TB][GROUPED_DEMO] rows=%0d cycles=%0d GEMMs=%0d active_M=%0d",
                         expert_group_rows, cycle_cnt, gemm_cnt, mac_util_max_m);
                $display("[TB][GROUPED_DEMO] numeric PASS max_abs=%f", group_max_abs_err);
                $display("[TB][GROUPED_MAC_UTIL] useful_MAC=%0d array_capacity_MAC_slots=%0d array_utilization=%0.3f%% enabled_PE_MAC_slots=%0d enabled_PE_utilization=%0.3f%% GEMM_wall_capacity_MAC_slots=%0d GEMM_wall_utilization=%0.3f%% GEMM_DMA_overlap_cycles=%0d mixed_K_prefetches=%0d mixed_K_DMA_GEMM_overlap_cycles=%0d GEMM_GLD_W_overlap_cycles=%0d dequant_feed_overlap_cycles=%0d",
                         mac_util_useful_ops, mac_util_peak_ops,
                         100.0 * $itor(mac_util_useful_ops) / $itor(mac_util_peak_ops),
                         mac_util_enabled_pe_slots,
                         100.0 * $itor(mac_util_useful_ops) /
                         $itor(mac_util_enabled_pe_slots),
                         mac_util_wall_capacity_ops,
                         100.0 * $itor(mac_util_useful_ops) /
                         $itor(mac_util_wall_capacity_ops),
                         mac_util_dma_overlap_cycles,
                         group_explicit_k_dma_count,
                         group_explicit_k_dma_overlap_cycles,
                         mac_util_gld_overlap_cycles,
                         mac_util_dequant_feed_overlap_cycles);
                $display("[TB][GROUPED_STALL_CYCLES] dma_busy=%0d gld_busy=%0d gemm_busy=%0d barrier_active=%0d decode_issue_wait=%0d",
                         group_dma_busy_cycles, group_gld_busy_cycles,
                         group_gemm_busy_cycles, group_barrier_busy_cycles,
                         group_fetch_wait_cycles);
                $display("[TB][GROUPED_DECODE_STALLS] NOP=%0d SCALE=%0d other_NOP=%0d VLD=%0d VST=%0d GLD_W=%0d GLD_A=%0d GEMM=%0d DMA_RD=%0d DMA_WR=%0d BARRIER=%0d",
                         group_decode_stall_by_opcode[0],
                         group_scale_stall_cycles,
                         group_other_nop_stall_cycles,
                         group_decode_stall_by_opcode[1],
                         group_decode_stall_by_opcode[2],
                         group_decode_stall_by_opcode[3],
                         group_decode_stall_by_opcode[4],
                         group_decode_stall_by_opcode[5],
                         group_decode_stall_by_opcode[12],
                         group_decode_stall_by_opcode[13],
                         group_decode_stall_by_opcode[14]);
                $display("GROUPED_MAC_UTIL_PASS");
                $finish;
            end

            if (expert_demo && !expert_demo_golden) begin
                if (expert_expect_gemm > 0 && gemm_cnt != expert_expect_gemm)
                    $fatal(1, "[TB][MIG_EXPERT] DEMO FAIL: gemm count %0d != expected %0d",
                           gemm_cnt, expert_expect_gemm);
                $display("[TB][MIG_EXPERT] DEMO PASS: all-resident 60-expert image, 180 descriptors, cycles=%0d routed GEMMs=%0d",
                         cycle_cnt, gemm_cnt);
                $finish;
            end
            if (expert_demo_golden) begin
                if (gemm_cnt != expert_expect_gemm)
                    $fatal(1, "[TB][MIG_EXPERT] DEMO GOLDEN FAIL: gemm count %0d != expected %0d",
                           gemm_cnt, expert_expect_gemm);
                sc_mismatch = 0;
                $display("[TB][MIG_EXPERT] DEMO GOLDEN: comparing mixed-precision projection outputs");
            end

            // ---- Layer-1 check: scale_zero_mem slot/proj tile vs image ----
            // After the LAST NOP_SCALE (N-tile nt_start+ntiles-1) the mem holds
            // that tile's scale block. Block size is per-proj compact:
            // gate/up 64 words (2048 B), down 44 words (1408 B); only the
            // first sc_words entries are loaded/checked (descriptor-driven).
            if (!expert_demo_golden) begin : expert_scale_check
                integer sc_words;
                sc_words = (expert_proj == 2) ? 44 : 64;
                fd_e = $fopen(expert_image, "rb");
                if (fd_e == 0) $fatal(1, "Cannot reopen image %0s", expert_image);
                if ($fseek(fd_e, sc_base
                           + (expert_nt_start + expert_ntiles - 1)*sc_words*32, 0) != 0)
                    $fatal(1, "image seek failed (scale)");
                rdsz = $fread(sc_bytes, fd_e, 0, sc_words*32);
                $fclose(fd_e);
                if (rdsz != sc_words*32) $fatal(1, "scale read short: %0d", rdsz);
                sc_mismatch = 0;
                for (g = 0; g < sc_words; g++) begin
                    for (int b = 0; b < 32; b++)
                        exp_beat[b*8 +: 8] = sc_bytes[g*32 + b];
                    if (u_npu.u_scale_zero_mem.mem[(expert_slot*3 + expert_proj)*
                            (u_npu.wide_scale_layout ? 256 : 64) + g] !== exp_beat) begin
                        if (sc_mismatch < 4)
                            $display("[TB][MIG_EXPERT] SZCHECK beat%0d rtl=%064x exp=%064x",
                                     (expert_slot*3 + expert_proj)*
                                         (u_npu.wide_scale_layout ? 256 : 64) + g,
                                     u_npu.u_scale_zero_mem.mem[(expert_slot*3 + expert_proj)*
                                         (u_npu.wide_scale_layout ? 256 : 64) + g], exp_beat);
                        sc_mismatch = sc_mismatch + 1;
                    end
                end
                $display("[TB][MIG_EXPERT] SZCHECK slot%0d/proj%0d beats %0d..%0d: %0d mismatch(es)",
                         expert_slot, expert_proj, (expert_slot*3 + expert_proj)*
                             (u_npu.wide_scale_layout ? 256 : 64),
                         (expert_slot*3 + expert_proj)*
                             (u_npu.wide_scale_layout ? 256 : 64) + sc_words - 1,
                         sc_mismatch);
            end

            // ---- Layer-2 check: acc_buffer vs golden (col-major, result bank) ----
            max_abs = 0.0;
            fd_gold = $fopen(expert_golden, "rb");
            if (fd_gold == 0)
                $fatal(1, "Cannot open golden file %0s", expert_golden);
            if (expert_ntiles > 1) begin : mt_readback
                // Multi-N-tile: golden holds ntiles*32 fp32; RTL side uses the
                // per-tile snapshots captured at each tile's last gemm_done.
                real tile_max;
                integer t;
                for (t = 0; t < expert_ntiles; t++) begin
                    tile_max = 0.0;
                    for (g = 0; g < 32; g++) begin
                        fr = $fread(gb, fd_gold);
                        if (fr != 4)
                            $fatal(1, "Golden file too short at tile %0d word %0d", t, g);
                        w32 = {gb[3], gb[2], gb[1], gb[0]};
                        gv = $bitstoshortreal(w32);
                        nv = $bitstoshortreal(exp_tile_acc[t][g]);
                        dd = (nv > gv) ? (nv - gv) : (gv - nv);
                        if (dd > tile_max) tile_max = dd;
                        if (dd > max_abs)  max_abs  = dd;
                        if (g < 4)
                            $display("[TB][MIG_EXPERT] nt%0d out[%0d] rtl=%f gold=%f",
                                     expert_nt_start + t, g, nv, gv);
                    end
                    $display("[TB][MIG_EXPERT] nt%0d max_abs=%e",
                             expert_nt_start + t, tile_max);
                end
            end else begin
            for (g = 0; g < 32; g++) begin
                fr = $fread(gb, fd_gold);
                if (fr != 4)
                    $fatal(1, "Golden file too short at word %0d (fr=%0d)", g, fr);
                w32 = {gb[3], gb[2], gb[1], gb[0]};
                gv = $bitstoshortreal(w32);
                nv = last_gemm_acc_bank
                     ? $bitstoshortreal(u_npu.u_acc_buffer.bank1_mem[g][0])
                     : $bitstoshortreal(u_npu.u_acc_buffer.bank0_mem[g][0]);
                dd = (nv > gv) ? (nv - gv) : (gv - nv);
                if (dd > max_abs) max_abs = dd;
                if (g < 8)
                    $display("[TB][MIG_EXPERT] out[%0d] rtl=%f gold=%f", g, nv, gv);
            end
            end
            $fclose(fd_gold);
            $display("[TB][MIG_EXPERT] max_abs(col-major bank%0d)=%e ntiles=%0d gldw=%0d gemm=%0d",
                     last_gemm_acc_bank, max_abs, expert_ntiles, gldw_cnt, gemm_cnt);
            expected_total_gemms = expert_demo_golden
                                 ? expert_expect_gemm
                                 : expert_ntiles * expert_gemms_per_proj
                                   + (force_topk ? 0 : 128);
            if (gemm_cnt != expected_total_gemms)
                $fatal(1, "[TB][MIG_EXPERT] FAIL: gemm count %0d != expected %0d (ntiles*ktiles%0s)",
                       gemm_cnt, expected_total_gemms,
                       force_topk ? "" : " + 128 router");
            if (max_abs <= 1.0e-2 && sc_mismatch == 0)
                begin
                    $display("[TB][MIG_EXPERT] PASS (AC-05) DONE cycle=%0d bank=%0d max_abs=%e ntiles=%0d",
                             cycle_cnt, last_gemm_acc_bank, max_abs, expert_ntiles);
                    if (expert_demo_golden)
                        $display("NUMERIC_REGRESSION_PASS");
                end
            else
                $fatal(1, "[TB][MIG_EXPERT] FAIL: max_abs=%e sc_mismatch=%0d (tol 1e-2)",
                       max_abs, sc_mismatch);
            $finish;
        end

        // ================================================================
        // MIG_LMHEAD: LM head vocab-subset fp16 GEMM numeric validation.
        // Weights: real lm_head.weight rows [0, LMHEAD_ROWS), packed by
        // pack_lmhead_subset.py (K-ascending / lane-minor, K_tile=64).
        // Activation: real_layer_L0.bin byte 0 (input_layernorm output
        // token0) — a layernorm output, NOT the true final hidden state;
        // the goal is proving the fp16 GEMM data path on real weights.
        // Per-window DDR layout (window stride 1 MiB, <= 7 N-tiles each
        // because the DMA imm window is 16-bit x 16 B):
        //   base+0x0000  4 KiB activation copy
        //   base+0x1000  N-tiles x 128 KiB weights (gen_lmhead_mig_ucode.py)
        // Each N-tile's 32-lane fp32 acc result is captured by the
        // exp_snap_ktiles monitor (every 32nd gemm_done) and compared
        // against lmhead_logits_golden.bin (verify_lmhead_golden.py).
        // ================================================================
        if ($test$plusargs("MIG_LMHEAD")) begin : mig_lmhead
            string  lm_image, lm_wgt, lm_golden;
            string  lm_ucode;
            integer lm_rows, lm_ntiles, lm_windows, lm_rem;
            integer lm_base;
            integer lm_start_w;
            integer loaded_bytes, cycle_cnt, wstart;
            integer w, t, g, fr, fd_gold, nt_this;
            real    gv, nv, dd, ma_all, ma_w;
            logic [31:0] w32;
            logic [7:0]  gb [0:3];

            if (!$value$plusargs("REAL_IMAGE=%s", lm_image))
                lm_image = "D:/graduation_design/npu-main/sim/modelsim/real_layer_L0.bin";
            if (!$value$plusargs("LMHEAD_WGT=%s", lm_wgt))
                lm_wgt = "D:/graduation_design/npu-main/sim/modelsim/lmhead_wrf_subset.bin";
            if (!$value$plusargs("LMHEAD_GOLDEN=%s", lm_golden))
                lm_golden = "D:/graduation_design/npu-main/sim/modelsim/lmhead_logits_golden.bin";
            if (!$value$plusargs("LMHEAD_ROWS=%d", lm_rows))
                lm_rows = 224;
            if (!$value$plusargs("LMHEAD_BASE=%d", lm_base))
                lm_base = 32'h0800_0000;   // 128 MiB, beyond the 85 MiB layer image
            // Resume support: skip windows [0, LMHEAD_START_W) entirely (their
            // DDR contents are NOT streamed; only windows >= start_w exist).
            if (!$value$plusargs("LMHEAD_START_W=%d", lm_start_w))
                lm_start_w = 0;

            lm_ntiles  = (lm_rows + 31) / 32;
            lm_windows = (lm_ntiles + 6) / 7;
            $display("[TB][MIG_LMHEAD] rows=%0d ntiles=%0d windows=%0d base=0x%08x start_w=%0d",
                     lm_rows, lm_ntiles, lm_windows, lm_base, lm_start_w);
            if (lm_start_w >= lm_windows)
                $fatal(1, "[TB][MIG_LMHEAD] LMHEAD_START_W=%0d >= windows=%0d",
                       lm_start_w, lm_windows);

            // ---- Common register config (fp16, K_tile=64, M=1) ----
            axil_write(32'h10, 32'h0000_0000);   // DDR_BASE_ACT (unused by ucode DMA)
            axil_write(32'h1C, 32'h0040_0000);   // DDR_BASE_OUT (scratch, unused)
            axil_write(32'h48, 32'h0041_0000);   // DDR_BASE_SCALE (unused for fp16)
            axil_write(32'h4C, 32'd1);           // CFG_M = 1 (last-position logits)
            axil_write(32'h50, 32'd32);          // CFG_N (one N-tile)
            axil_write(32'h54, 32'd64);          // CFG_K = K_tile = 64

            // Enable the per-N-tile acc snapshot monitor (shared with
            // MIG_EXPERT): snapshot at every 32nd gemm_done.
            exp_snap_ktiles = 32;

            ma_all = 0.0;
            // Per-window: stream (act copy + weights) -> run -> compare.
            // Interleaved so a killed run preserves every finished window's
            // PASS line in the log; restart with +LMHEAD_START_W=<next w>.
            for (w = lm_start_w; w < lm_windows; w++) begin
                nt_this = (lm_ntiles - w * 7 < 7) ? (lm_ntiles - w * 7) : 7;
                prefill_mode = 1'b1;
                repeat(10) @(posedge c0_ddr4_ui_clk);
                prefill_ddr4_stream(lm_image, 0,
                                    lm_base + w * 32'h0010_0000, 4096,
                                    1'b1, loaded_bytes);
                prefill_ddr4_stream(lm_wgt, w * 32'h000E_0000,
                                    lm_base + w * 32'h0010_0000 + 32'h1000,
                                    nt_this * 32'h0002_0000, 1'b1, loaded_bytes);
                $display("[TB][MIG_LMHEAD] window %0d streamed (%0d N-tiles)", w, nt_this);
                repeat(10) @(posedge c0_ddr4_ui_clk);
                prefill_mode = 1'b0;
                repeat(20) @(posedge clk);
                lm_ucode = $sformatf(
                    "D:/graduation_design/npu-main/sim/modelsim/mig_lmhead_nt%0d.bin",
                    nt_this);
                $display("[TB][MIG_LMHEAD] window %0d: loading ucode %0s", w, lm_ucode);
                load_ucode_file(lm_ucode);
                axil_write(32'h0C, ucode_size / 4);
                axil_write(32'h14, lm_base + w * 32'h0010_0000);  // DDR_BASE_WGT = window base
                repeat(5) @(posedge clk);

                exp_gd_cnt = 0;   // re-arm the snapshot monitor for this window
                $display("[TB][MIG_LMHEAD] window %0d: START", w);
                axil_write(32'h00, 32'h0000_0001);
                cycle_cnt = 0;
                while (!u_npu.npu_done_latch && cycle_cnt < 20000000) begin
                    @(posedge clk); cycle_cnt = cycle_cnt + 1;
                    if (cycle_cnt % 500000 == 0)
                        $display("[TB][MIG_LMHEAD] window %0d: %0d cyc", w, cycle_cnt);
                end
                if (!u_npu.npu_done_latch)
                    $fatal(1, "[TB][MIG_LMHEAD] window %0d timeout after %0d cycles",
                           w, cycle_cnt);
                $display("[TB][MIG_LMHEAD] window %0d done cycle=%0d", w, cycle_cnt);
                // Drain trailing DMA before the next START.
                wstart = cycle_cnt;
                while ((u_npu.dma_wr_busy || u_npu.dma_rd_busy) &&
                       cycle_cnt - wstart < 2000000) begin
                    @(posedge clk); cycle_cnt = cycle_cnt + 1;
                end

                // Compare this window's snapshots against the golden.
                ma_w = 0.0;
                fd_gold = $fopen(lm_golden, "rb");
                if (fd_gold == 0)
                    $fatal(1, "[TB][MIG_LMHEAD] cannot open golden %0s", lm_golden);
                for (t = 0; t < nt_this; t++) begin
                    for (g = 0; g < 32; g++) begin
                        if ($fseek(fd_gold, ((w * 7 + t) * 32 + g) * 4, 0) != 0)
                            $fatal(1, "[TB][MIG_LMHEAD] golden seek failed");
                        fr = $fread(gb, fd_gold);
                        if (fr != 4) $fatal(1, "[TB][MIG_LMHEAD] golden short read");
                        w32 = {gb[3], gb[2], gb[1], gb[0]};
                        gv = $bitstoshortreal(w32);
                        nv = $bitstoshortreal(exp_tile_acc[t][g]);
                        dd = (nv > gv) ? (nv - gv) : (gv - nv);
                        if (dd > ma_w) ma_w = dd;
                    end
                end
                $fclose(fd_gold);
                if (ma_w > ma_all) ma_all = ma_w;
                $display("[TB][MIG_LMHEAD] window %0d max_abs=%e (rows %0d..%0d)",
                         w, ma_w, w * 224, w * 224 + nt_this * 32 - 1);
            end

            if (ma_all <= 1.0e-2)
                $display("[TB][MIG_LMHEAD] PASS (tol 1e-2, AC-05-aligned) rows %0d..%0d max_abs=%e",
                         lm_start_w * 224, lm_rows - 1, ma_all);
            else
                $fatal(1, "[TB][MIG_LMHEAD] FAIL: max_abs=%e (tol 1e-2)", ma_all);
            $finish;
        end

        // ================================================================
        // MIG_ROUTER: run the real router (gate) fp16 GEMM end-to-end.
        // Loads the real post-attention-layernorm activation token0 (DDR 0x0)
        // and the real router.weight fp16 region (DDR 0x10000) through the MIG
        // front door, runs the router ucode (fp16, N-tile0 rows 0..31 then
        // N-tile1 rows 32..59+pad), and compares the 60 logits against the
        // software reference (verify_router_topk.py / pack_router.py).
        // ================================================================
        if ($test$plusargs("MIG_ROUTER")) begin : mig_router
            string router_image;
            string router_ucode0, router_ucode1;
            string router_golden;
            integer loaded_bytes, cycle_cnt;
            integer gldw_cnt, wrfwr_cnt, glda_cnt, arfwr_cnt, dmard_cnt;
            integer dq_fire_cnt;
            integer router_start_nt;
            real ma_b0, ma_b1, ma0, ma1;
            logic ran_nt0;
            integer fd_gold, g, fr;
            logic [31:0] w32;
            logic [7:0] gb [0:3];
            real gv, nv, dd;
            // TopK end-to-end check state (ROUTER_CHECK_TOPK)
            integer fd_ref, tk, rc;
            integer ref_idx [0:3];
            logic [15:0] ref_w [0:3];
            string ref_tag;
            real w_rtl, w_ref, w_err, w_maxerr;
            logic topk_idx_ok;

            if (!$value$plusargs("REAL_IMAGE=%s", router_image))
                router_image = "sim/modelsim/real_layer_L0.bin";
            if (!$value$plusargs("ROUTER_UCODE0=%s", router_ucode0))
                router_ucode0 = "mig_real_router_nt0.bin";
            if (!$value$plusargs("ROUTER_UCODE1=%s", router_ucode1))
                router_ucode1 = "mig_real_router_nt1.bin";
            if (!$value$plusargs("ROUTER_GOLDEN=%s", router_golden))
                router_golden = "router_logits_golden.bin";
            router_start_nt = 0;
            if (!$value$plusargs("ROUTER_START_NT=%d", router_start_nt))
                router_start_nt = 0;
            ran_nt0 = (router_start_nt == 0);

            prefill_mode = 1'b1;
            repeat(10) @(posedge c0_ddr4_ui_clk);

            $display("[TB][MIG_ROUTER] streaming router activation + weights to DDR...");
            prefill_ddr4_stream("D:/graduation_design/npu-main/sim/modelsim/router_act_token0.bin", 0,
                                32'h0000_0000, 4096, 1'b1, loaded_bytes);
            prefill_ddr4_stream(router_image, 32'd17379328,
                                32'h0001_0000, 262144, 1'b1, loaded_bytes);

            $display("[TB][MIG_ROUTER] DDR pre-fill complete");
            repeat(10) @(posedge c0_ddr4_ui_clk);
            prefill_mode = 1'b0;
            repeat(20) @(posedge clk);

            // Common NPU register config for the fp16 router GEMM.
            axil_write(32'h10, 32'h0000_0000);   // DDR_BASE_ACT
            axil_write(32'h14, 32'h0000_0000);   // DDR_BASE_WGT (imm carries 0x10000)
            axil_write(32'h1C, 32'h0008_0000);   // DDR_BASE_OUT (scratch)
            axil_write(32'h48, 32'h0041_0000);   // DDR_BASE_SCALE (unused for fp16)
            axil_write(32'h4C, 32'd1);           // CFG_M
            axil_write(32'h50, 32'd32);          // CFG_N (one N-tile)
            axil_write(32'h54, 32'd32);          // CFG_K (per K-tile)

            // ---- N-tile 0: rows 0..31 (skipped when ROUTER_START_NT=1) ----
            if (router_start_nt == 0) begin
                $display("[TB][MIG_ROUTER] Loading ucode (N-tile0: %0s)...", router_ucode0);
                load_ucode_file(router_ucode0);
                axil_write(32'h0C, ucode_size / 4);
                repeat(5) @(posedge clk);
                $display("[TB][MIG_ROUTER] Starting N-tile0...");
                axil_write(32'h00, 32'h0000_0001);   // START
                cycle_cnt = 0;
                while (!u_npu.npu_done_latch && cycle_cnt < 2000000) begin
                    @(posedge clk); cycle_cnt = cycle_cnt + 1;
                end
                if (!u_npu.npu_done_latch)
                    $fatal(1, "MIG_ROUTER N-tile0 timeout after %0d cycles", cycle_cnt);
                $display("[TB][MIG_ROUTER] N-tile0 done cycle=%0d", cycle_cnt);
                // The final DMA_WR (acc result -> DDR) runs after END; wait for
                // both DMA engines to drain before the next START, otherwise the
                // leftover DMA_WR occupies scoreboard bit1 and the next program's
                // first BARRIER never sees all_idle.
                cycle_cnt = 0;
                while ((u_npu.dma_wr_busy || u_npu.dma_rd_busy) && cycle_cnt < 2000000) begin
                    @(posedge clk); cycle_cnt = cycle_cnt + 1;
                end
                $display("[TB][MIG_ROUTER] nt0 DMA drained after %0d cycles", cycle_cnt);

                // Read back rows 0..31 from acc_buffer (bank that holds the result).
                ma_b0 = 0.0; ma_b1 = 0.0;
                fd_gold = $fopen(router_golden, "rb");
                if (fd_gold == 0)
                    $fatal(1, "Cannot open router golden %0s", router_golden);
                for (g = 0; g < 32; g++) begin
                    fr = $fread(gb, fd_gold);
                    w32 = {gb[3], gb[2], gb[1], gb[0]};
                    gv = $bitstoshortreal(w32);
                    nv = $bitstoshortreal(u_npu.u_acc_buffer.bank0_mem[g][0]);
                    dd = (nv > gv) ? (nv - gv) : (gv - nv); if (dd > ma_b0) ma_b0 = dd;
                    nv = $bitstoshortreal(u_npu.u_acc_buffer.bank1_mem[g][0]);
                    dd = (nv > gv) ? (nv - gv) : (gv - nv); if (dd > ma_b1) ma_b1 = dd;
                end
                $fclose(fd_gold);
                ma0 = (ma_b0 < ma_b1) ? ma_b0 : ma_b1;
                $display("[TB][MIG_ROUTER] nt0 max_abs(bank0)=%e bank1=%e best=%e",
                         ma_b0, ma_b1, ma0);
            end else begin
                ma0 = 0.0;   // nt0 skipped
                $display("[TB][MIG_ROUTER] N-tile0 skipped (ROUTER_START_NT=1)");
            end

            // ---- N-tile 1: rows 32..59 (+4 zero pad) ----
            $display("[TB][MIG_ROUTER] Loading ucode (N-tile1: %0s)...", router_ucode1);
            load_ucode_file(router_ucode1);
            axil_write(32'h0C, ucode_size / 4);
            repeat(5) @(posedge clk);
            $display("[TB][MIG_ROUTER] Starting N-tile1...");
            axil_write(32'h00, 32'h0000_0001);   // START
            cycle_cnt = 0;
            while (!u_npu.npu_done_latch && cycle_cnt < 2000000) begin
                @(posedge clk); cycle_cnt = cycle_cnt + 1;
                if (cycle_cnt % 200000 == 0)
                    $display($sformatf("[TB][MIG_ROUTER] nt1 diag cycle=%0d fetch_pc=%0d fetch_state=%0d instr_valid=%b instr_ready=%b barrier=%b res_busy=%02x dma_rd_busy=%b lc_state=%0d lc_instr=%08x",
                             cycle_cnt, u_npu.u_fetch.pc,
                             u_npu.u_fetch.state_q,
                             u_npu.u_fetch.instr_valid,
                             u_npu.u_decode.instr_ready,
                             u_npu.u_decode.barrier_active,
                             u_npu.res_busy,
                             u_npu.dma_rd_busy,
                             u_npu.u_loop_ctrl.state,
                             u_npu.u_loop_ctrl.dec_instr));
            end
            if (!u_npu.npu_done_latch)
                $fatal(1, "MIG_ROUTER N-tile1 timeout after %0d cycles", cycle_cnt);
            $display("[TB][MIG_ROUTER] N-tile1 done cycle=%0d", cycle_cnt);

            // Read back rows 32..59 (28 real; golden file holds 60 fp32).
            ma_b0 = 0.0; ma_b1 = 0.0;
            fd_gold = $fopen(router_golden, "rb");
            if (fd_gold == 0)
                $fatal(1, "Cannot open router golden %0s", router_golden);
            for (g = 0; g < 28; g++) begin
                if ($fseek(fd_gold, (32 + g) * 4, 0) != 0)
                    $fatal(1, "golden seek failed at row %0d", 32 + g);
                fr = $fread(gb, fd_gold);
                if (fr != 4) $fatal(1, "golden short at row %0d", 32 + g);
                w32 = {gb[3], gb[2], gb[1], gb[0]};
                gv = $bitstoshortreal(w32);
                nv = $bitstoshortreal(u_npu.u_acc_buffer.bank0_mem[g][0]);
                dd = (nv > gv) ? (nv - gv) : (gv - nv); if (dd > ma_b0) ma_b0 = dd;
                nv = $bitstoshortreal(u_npu.u_acc_buffer.bank1_mem[g][0]);
                dd = (nv > gv) ? (nv - gv) : (gv - nv); if (dd > ma_b1) ma_b1 = dd;
            end
            $fclose(fd_gold);
            ma1 = (ma_b0 < ma_b1) ? ma_b0 : ma_b1;
            $display("[TB][MIG_ROUTER] nt1 max_abs(bank0)=%e bank1=%e best=%e",
                     ma_b0, ma_b1, ma1);

            // ---- Optional TopK end-to-end check (ROUTER_CHECK_TOPK) ----
            // Requires the nt1 ucode to end with NOP_TOPK (row 0).  topk_unit
            // reads logits 0..31 from the N-tile0 acc set and 32..59 from the
            // N-tile1 set, then emits 4 expert indices + 4 fp16 weights.
            if ($test$plusargs("ROUTER_CHECK_TOPK")) begin
                cycle_cnt = 0;
                while (!u_npu.topk_done_latched && cycle_cnt < 100000) begin
                    @(posedge clk); cycle_cnt = cycle_cnt + 1;
                end
                if (!u_npu.topk_done_latched)
                    $fatal(1, "[TB][MIG_ROUTER] TOPK timeout after %0d cycles", cycle_cnt);
                $display("[TB][MIG_ROUTER] TOPK done after %0d cycles", cycle_cnt);

                fd_ref = $fopen("D:/graduation_design/npu-main/sim/modelsim/router_topk_ref.txt", "r");
                if (fd_ref == 0)
                    $fatal(1, "Cannot open router_topk_ref.txt");
                rc = $fscanf(fd_ref, "%s", ref_tag);          // "idx"
                for (tk = 0; tk < 4; tk++)
                    rc = $fscanf(fd_ref, "%d", ref_idx[tk]);
                rc = $fscanf(fd_ref, "%s", ref_tag);          // "val"
                for (tk = 0; tk < 4; tk++)
                    rc = $fscanf(fd_ref, "%s", ref_tag);      // skip 4 fp32 logits
                rc = $fscanf(fd_ref, "%s", ref_tag);          // "sw_fp16"
                for (tk = 0; tk < 4; tk++)
                    rc = $fscanf(fd_ref, "%h", ref_w[tk]);
                $fclose(fd_ref);

                topk_idx_ok = 1'b1;
                w_maxerr = 0.0;
                for (tk = 0; tk < 4; tk++) begin
                    if (u_npu.topk_idx_arr[tk] !== ref_idx[tk][5:0])
                        topk_idx_ok = 1'b0;
                    w_rtl = fp16_to_real(u_npu.topk_w_arr[tk]);
                    w_ref = fp16_to_real(ref_w[tk]);
                    w_err = (w_rtl > w_ref) ? (w_rtl - w_ref) : (w_ref - w_rtl);
                    if (w_err > w_maxerr) w_maxerr = w_err;
                    $display("[TB][MIG_ROUTER] topk[%0d] idx rtl=%0d ref=%0d w rtl=%h(%f) ref=%h(%f)",
                             tk, u_npu.topk_idx_arr[tk], ref_idx[tk],
                             u_npu.topk_w_arr[tk], w_rtl, ref_w[tk], w_ref);
                end
                if (topk_idx_ok && w_maxerr <= 2.0e-3)
                    $display("[TB][MIG_ROUTER] TOPK PASS idx=%0d %0d %0d %0d w_maxerr=%e",
                             u_npu.topk_idx_arr[0], u_npu.topk_idx_arr[1],
                             u_npu.topk_idx_arr[2], u_npu.topk_idx_arr[3], w_maxerr);
                else
                    $fatal(1, "[TB][MIG_ROUTER] TOPK FAIL idx_ok=%0d w_maxerr=%e (tol 2e-3)",
                           topk_idx_ok, w_maxerr);
            end

            if ((!ran_nt0 || ma0 <= 1.0e-2) && (ma1 <= 1.0e-2))
                $display("[TB][MIG_ROUTER] PASS (AC-05) router logits nt0/nt1 = %e / %e%0s",
                         ma0, ma1, ran_nt0 ? "" : " (nt0 skipped)");
            else
                $fatal(1, "[TB][MIG_ROUTER] FAIL: nt0=%e nt1=%e (tolerance 1e-2%0s)",
                       ma0, ma1, ran_nt0 ? "" : ", nt0 skipped");
            $finish;
        end

        // ================================================================
        // MIG_LAYER0: full real-weight Layer-0 end-to-end.
        // Streams the entire real_layer_L0.bin image into DDR (identity
        // mapping: file offset == DDR byte address), loads all 180 expert
        // descriptors, runs ucode_real_layer_L0.bin to END.
        // UCODE_DEPTH must be >= 262144 (compile with -GUCODE_DEPTH=262144).
        // ================================================================
        if ($test$plusargs("MIG_LAYER0")) begin : mig_layer0
            string layer0_image, layer0_ucode, layer0_edesc;
            string layer0_stage_prefix, layer0_stage_ucode;
            integer loaded_bytes, cycle_cnt, pc_done_cycle, watchdog_cycles;
            integer stage_count, stage_index, total_compute_cycles;
            integer fd_e, rdsz, ei;
            integer attention_context, attention_stream_bytes;
            integer attention_q_count, attention_q_bytes;
            integer q_source_offset, k_source_offset, v_source_offset;
            logic [31:0] layer0_base_act;
            logic [7:0]  edesc_bytes [0:180*12-1];
            logic [31:0] ew0, ew1, ew2;
            integer img_fd, img_size;
            real layer0_ui_period_ns;

            if (!$value$plusargs("LAYER0_IMAGE=%s", layer0_image))
                layer0_image = "D:/graduation_design/npu-main/sim/modelsim/real_layer_L0.bin";
            layer0_base_act = 32'h0000_0000;
            if (!$value$plusargs("LAYER0_BASE_ACT=%h", layer0_base_act))
                layer0_base_act = 32'h0000_0000;
            attention_context = 0;
            if (!$value$plusargs("LAYER0_ATTENTION_CONTEXT=%d", attention_context))
                attention_context = 0;
            q_source_offset = 0;
            k_source_offset = 0;
            v_source_offset = 0;
            if (attention_context > 0) begin
                if (!$value$plusargs("LAYER0_Q_OFFSET=%h", q_source_offset) ||
                    !$value$plusargs("LAYER0_K_OFFSET=%h", k_source_offset) ||
                    !$value$plusargs("LAYER0_V_OFFSET=%h", v_source_offset))
                    $fatal(1, "Sparse attention image requires Q/K/V offsets");
                if (attention_context > 1536)
                    $fatal(1, "Sparse attention context exceeds supported 1536 tokens");
                attention_q_count = attention_context;
                if (!$value$plusargs("LAYER0_Q_COUNT=%d", attention_q_count))
                    attention_q_count = attention_context;
                if (attention_q_count < 1 || attention_q_count > attention_context)
                    $fatal(1, "Sparse attention Q count must be in [1,context]");
            end
            if (!$value$plusargs("LAYER0_UCODE=%s", layer0_ucode))
                layer0_ucode = "D:/graduation_design/npu-main/sim/modelsim/ucode_real_layer_L0.bin";
            stage_count = 1;
            if (!$value$plusargs("LAYER0_UCODE_STAGES=%d", stage_count))
                stage_count = 1;
            if (stage_count < 1 || stage_count > 16)
                $fatal(1, "[TB][MIG_LAYER0] LAYER0_UCODE_STAGES must be in [1,16]");
            if (stage_count > 1) begin
                if (!$value$plusargs("LAYER0_UCODE_PREFIX=%s", layer0_stage_prefix))
                    $fatal(1, "[TB][MIG_LAYER0] staged mode requires LAYER0_UCODE_PREFIX");
            end
            if (!$value$plusargs("LAYER0_EDESC=%s", layer0_edesc))
                layer0_edesc = "D:/graduation_design/npu-main/sim/modelsim/expert_desc.bin";
            if (!$value$plusargs("LAYER0_WATCHDOG=%d", watchdog_cycles))
                watchdog_cycles = 1000000000;
            if (watchdog_cycles <= 0)
                $fatal(1, "[TB][MIG_LAYER0] LAYER0_WATCHDOG must be positive");

            if (UCODE_DEPTH < 262144)
                $fatal(1, "[TB][MIG_LAYER0] UCODE_DEPTH=%0d too small; recompile with -GUCODE_DEPTH=262144",
                       UCODE_DEPTH);

            // Get image size
            img_fd = $fopen(layer0_image, "rb");
            if (img_fd == 0) $fatal(1, "[TB][MIG_LAYER0] Cannot open %0s", layer0_image);
            img_size = $fseek(img_fd, 0, 2);   // seek to end
            img_size = $ftell(img_fd);
            $fclose(img_fd);
            $display("[TB][MIG_LAYER0] image=%0s (%0d bytes)", layer0_image, img_size);

            prefill_mode = 1'b1;
            repeat(10) @(posedge c0_ddr4_ui_clk);

            if (attention_context > 0) begin
                // Attention smoke images have sparse address layouts. Stream
                // only scale and Q/K/V rows, preserving their identity-mapped
                // DDR addresses without writing the zero-filled gaps.
                attention_stream_bytes = attention_context * 4096;
                attention_q_bytes = attention_q_count * 4096;
                prefill_ddr4_stream_fast(
                    layer0_image, layer0_base_act + 32'h0000_1000,
                    layer0_base_act + 32'h0000_1000, 256, 1'b1, loaded_bytes);
                if (loaded_bytes != 256)
                    $fatal(1, "Sparse attention scale prefill short");
                prefill_ddr4_stream_fast(
                    layer0_image, layer0_base_act + q_source_offset,
                    layer0_base_act + q_source_offset,
                    attention_q_bytes, 1'b1, loaded_bytes);
                if (loaded_bytes != attention_q_bytes)
                    $fatal(1, "Sparse attention Q prefill short");
                prefill_ddr4_stream_fast(
                    layer0_image, layer0_base_act + k_source_offset,
                    layer0_base_act + k_source_offset,
                    attention_stream_bytes, 1'b1, loaded_bytes);
                if (loaded_bytes != attention_stream_bytes)
                    $fatal(1, "Sparse attention K prefill short");
                prefill_ddr4_stream_fast(
                    layer0_image, layer0_base_act + v_source_offset,
                    layer0_base_act + v_source_offset,
                    attention_stream_bytes, 1'b1, loaded_bytes);
                if (loaded_bytes != attention_stream_bytes)
                    $fatal(1, "Sparse attention V prefill short");
                $display("[TB][MIG_LAYER0] sparse attention pre-fill complete: context=%0d bytes=%0d",
                         attention_context,
                         attention_q_bytes + attention_stream_bytes * 2 + 256);
            end else begin
                // Stream the entire image into DDR with identity mapping.
                // prefill_ddr4_stream_fast writes each 8-byte word through MIG.
                prefill_ddr4_stream_fast(layer0_image, 0, 32'h0000_0000,
                                         img_size, 1'b1, loaded_bytes);
                $display("[TB][MIG_LAYER0] DDR pre-fill complete: %0d bytes", loaded_bytes);
            end
            repeat(10) @(posedge c0_ddr4_ui_clk);
            prefill_mode = 1'b0;
            repeat(20) @(posedge clk);

            // ---- Load all 180 expert descriptors through AXI-Lite ----
            fd_e = $fopen(layer0_edesc, "rb");
            if (fd_e == 0) $fatal(1, "[TB][MIG_LAYER0] Cannot open %0s", layer0_edesc);
            rdsz = $fread(edesc_bytes, fd_e);
            $fclose(fd_e);
            if (rdsz != 180*12) $fatal(1, "[TB][MIG_LAYER0] expert_desc.bin short: %0d bytes", rdsz);
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
            $display("[TB][MIG_LAYER0] loaded 180 expert descriptors");

            // Each stage runs to END and all-idle before the instruction store
            // is reloaded. DDR tensors and expert descriptors remain resident
            // across launches; instruction-load time is excluded from the
            // accumulated compute-cycle total.
            total_compute_cycles = 0;
            for (stage_index = 0; stage_index < stage_count; stage_index++) begin
                if (stage_count == 1)
                    layer0_stage_ucode = layer0_ucode;
                else if (stage_index < 10)
                    layer0_stage_ucode = $sformatf("%0s.stage0%0d.bin",
                                                   layer0_stage_prefix, stage_index);
                else
                    layer0_stage_ucode = $sformatf("%0s.stage%0d.bin",
                                                   layer0_stage_prefix, stage_index);
                $display("[TB][MIG_LAYER0] Loading stage %0d/%0d (%0s)...",
                         stage_index + 1, stage_count, layer0_stage_ucode);
                load_ucode_file(layer0_stage_ucode);
                repeat(5) @(posedge clk);

                $display("[TB][MIG_LAYER0] Configuring stage %0d registers...", stage_index);
                axil_write(32'h0C, ucode_size / 4); // UCODE_LEN
                axil_write(32'h10, layer0_base_act); // DDR_BASE_ACT
                axil_write(32'h14, 32'h0000_0000);   // DDR_BASE_WGT
                axil_write(32'h1C, 32'h0008_0000);   // DDR_BASE_OUT (scratch)
                axil_write(32'h48, 32'h0041_0000);   // DDR_BASE_SCALE
                axil_write(32'h4C, 32'd1);           // CFG_M reset value
                axil_write(32'h50, 32'd32);          // CFG_N (one N-tile)
                axil_write(32'h54, 32'd32);          // CFG_K (per K-tile)
                axil_write(32'h78, 32'd0);           // Descriptor drives precision
                axil_write(32'h7C, 32'd64);          // DEQUANT_SCALE_LEN
                repeat(5) @(posedge clk);

                $display("[TB][MIG_LAYER0] Starting stage %0d/%0d...",
                         stage_index + 1, stage_count);
                axil_write(32'h00, 32'h0000_0001);   // START
                cycle_cnt = 0;
                while (!u_npu.npu_done_latch && cycle_cnt < watchdog_cycles) begin
                    @(posedge clk); cycle_cnt = cycle_cnt + 1;
                    if (cycle_cnt % 1000000 == 0)
                        $display("[TB][MIG_LAYER0] stage=%0d cycle=%0d pc=%0d",
                                 stage_index, cycle_cnt, u_npu.u_fetch.pc);
                end
                if (!u_npu.npu_done_latch)
                    $fatal(1, "[TB][MIG_LAYER0] stage %0d timeout after %0d cycles (pc=%0d)",
                           stage_index, cycle_cnt, u_npu.u_fetch.pc);
                pc_done_cycle = cycle_cnt;
                $display("[MARK] STAGE_PC_DONE layer=0 stage=%0d cycle=%0d",
                         stage_index, pc_done_cycle);

                // END only proves that fetch reached the last instruction.
                while (!u_npu.exec_all_idle && cycle_cnt < watchdog_cycles) begin
                    @(posedge clk); cycle_cnt = cycle_cnt + 1;
                end
                if (!u_npu.exec_all_idle)
                    $fatal(1, "[TB][MIG_LAYER0] stage %0d all-idle timeout at cycle=%0d",
                           stage_index, cycle_cnt);
                total_compute_cycles = total_compute_cycles + cycle_cnt;
                $display("[MARK] STAGE_RESULT_VALID layer=0 stage=%0d cycles=%0d total=%0d",
                         stage_index, cycle_cnt, total_compute_cycles);
            end

            layer0_ui_period_ns = calib_second_edge - calib_first_edge;
            $display("[MARK] FIRST_RESULT_VALID layer=0 cycle=%0d TTFT_compute_cycles=%0d",
                     total_compute_cycles, total_compute_cycles);
            $display("[TB][MIG_LAYER0] TTFT_compute=%0.6f ms (period=%0.3f ns)",
                     total_compute_cycles * layer0_ui_period_ns / 1.0e6, layer0_ui_period_ns);
            $display("[TB][MIG_LAYER0] PASS: %0d stages reached END and all_idle",
                     stage_count);
            $finish;
        end

        // ================================================================
        // Pre-fill DDR4 with real data
        // ================================================================
        prefill_mode = 1'b1;
        repeat(10) @(posedge c0_ddr4_ui_clk);

        // Prefill: only small files needed (data content doesn't affect timing)
        prefill_ddr4("wq.bin", 32'h0000_1000, 2048);
        prefill_ddr4("activation.bin", 32'h0000_0000, 2048);
        prefill_ddr4("input_ln.bin", 32'h0000_8200, 256);
        prefill_ddr4("post_attn_ln.bin", 32'h0000_8400, 256);

        $display("[TB] DDR4 pre-fill complete!");
        repeat(10) @(posedge c0_ddr4_ui_clk);
        prefill_mode = 1'b0;
        repeat(20) @(posedge c0_ddr4_ui_clk);

        // ================================================================
        // Load UCODE
        // ================================================================
        $display("[TB] Loading ucode...");
        load_ucode();
        repeat(5) @(posedge clk);

        // Configure registers
        $display("[TB] Configuring registers...");
        axil_write(32'h0C, 32'd4125);        // UCODE_LEN = 4125
        axil_write(32'h10, 32'h0000_0000);   // DDR_BASE_ACT
        axil_write(32'h14, 32'h0000_1000);   // DDR_BASE_WGT
        axil_write(32'h1C, 32'h0000_2000);   // DDR_BASE_OUT
        axil_write(32'h48, 32'h0000_3000);   // DDR_BASE_SCALE
        axil_write(32'h4C, 32'd32);          // CFG_M
        axil_write(32'h50, 32'd32);          // CFG_N
        axil_write(32'h54, 32'd128);         // CFG_K
        repeat(5) @(posedge clk);

        // Start NPU
        $display("[TB] Starting NPU execution...");
        axil_write(32'h00, 32'h0000_0001);   // CTRL.START

        $display("[TB] Waiting for completion...");
        fork
            begin
                integer cycle_cnt;
                cycle_cnt = 0;
                while (cycle_cnt < 5000000) begin
                    @(posedge clk);
                    cycle_cnt = cycle_cnt + 1;
                    if (u_npu.program_end) begin
                        $display("[TB] Program END at cycle %0d", cycle_cnt);
                    end
                    if (u_npu.npu_done_latch) begin
                        $display("[TB] NPU DONE at cycle %0d (sim_time = %0.3f ns)", cycle_cnt, $realtime);
                        break;
                    end
                end
                if (!u_npu.npu_done_latch) begin
                    $display("[TB] Timeout after %0d cycles", cycle_cnt);
                end
            end
        join

        repeat(50) @(posedge clk);
        $display("[TB] Simulation complete.");
        $finish;
    end

    // =========================================================================
    // Watchdog
    // =========================================================================
    initial begin
        integer wdt_ms;
        if ($test$plusargs("WDT_OFF")) begin
            $display("[TB] WATCHDOG disabled by +WDT_OFF");
        end else begin
            if (!$value$plusargs("WDT_MS=%d", wdt_ms))
                wdt_ms = 2000;
            #(wdt_ms * 1ms);
            $display("[TB] WATCHDOG TIMEOUT — simulation killed after %0d ms", wdt_ms);
            $finish;
        end
    end

endmodule
