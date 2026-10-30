// =============================================================================
// ucode_decode.sv - 32-bit Instruction Decode and Dispatch
// =============================================================================
// Decodes 32-bit instructions and routes commands to engines/RF/DMA.
// Uses scoreboard for hazard detection and stall control.
// =============================================================================
`default_nettype none

module ucode_decode
  import npu_pkg::*;
  import isa_pkg::*;
#(
    parameter int INSTR_W = 32
) (
    input  wire                 clk,
    input  wire                 rst_n,

    // --- From fetch stage ---
    input  wire                 instr_valid,
    input  wire  [INSTR_W-1:0] instr_data,
    output logic                instr_ready,

    // --- TopK completion (expert precision-aware MoE) ---
    input  wire                 topk_done_sig,   // latched: 1 after TopK done
    input  wire                 route_unit_busy, // serialize consumers behind TopK/group scan
    input  wire                 l1_clear_busy,
    input  wire                 scale_loader_busy,
    input  wire                 dma_rd_l1_bank_valid,
    input  wire [3:0]           dma_rd_l1_bank,

    // --- Scoreboard interface (8-bit) ---
    input  wire  [7:0]          res_busy,       // one bit per resource_t
    input  wire                 all_idle,
    input  wire                 wrf_data_valid, // GLD_W finished, WRF data ready
    input  wire                 arf_data_valid, // GLD_A finished, ARF data ready

    // --- Loop interface (from fetch) ---
    input  wire                 loop_active,    // hardware loop is running
    input  wire  [15:0]          loop_iter,       // current iteration (0-based)
    input  wire  [15:0]          loop_remaining,
    input  wire                 static_k128_replay, // cached loop carries direct nbits + scale group
    input  wire  [5:0]           token_loop_iter,
    input  wire  [5:0]           expert_loop_iter,
    input  wire  [15:0]          addr_stride,     // stride per iter in imm units
    input  wire  [15:0]          cfg_M,
    input  wire  [15:0]          cfg_K,

    // --- VLD/VST: PostProc RF data movement ---
    output logic                vld_cmd_valid,
    output logic [3:0]          vld_cmd_vd,
    output logic [15:0]         vld_cmd_addr,
    output logic [1:0]          vld_cmd_page,
    output logic                vld_cmd_token_stride_mode,
    output logic [12:0]         vld_cmd_token_stride_bytes,

    output logic                vst_cmd_valid,
    output logic [3:0]          vst_cmd_vs1,
    output logic [15:0]         vst_cmd_addr,
    output logic [1:0]          vst_cmd_page,
    output logic                vst_cmd_token_stride_mode,
    output logic [12:0]         vst_cmd_token_stride_bytes,

    // --- GLD.W: GEMM Weight RF load ---
    output logic                gld_w_cmd_valid,
    output logic [15:0]         gld_w_cmd_addr,
    output logic [1:0]          gld_w_cmd_prec,
    output logic [5:0]          gld_w_cmd_group,  // scale group idx (up to 63)
    output logic [3:0]          gld_w_cmd_expert_sel, // vs2: 0=direct, 1-4=topk slot
    output logic [1:0]          gld_w_cmd_proj,      // 0=gate, 1=up, 2=down
    output logic [2:0]          gld_w_cmd_direct_nbits,

    // --- GLD.A: GEMM Activation RF load ---
    output logic                gld_a_cmd_valid,
    output logic [15:0]         gld_a_cmd_addr,
    output logic                gld_a_cmd_batch,
    output logic                gld_a_cmd_transpose,
    output logic [1:0]          gld_a_cmd_page,
    output logic [8:0]          gld_a_cmd_stride_lines,

    // --- GEMM engine ---
    output logic                gemm_cmd_valid,
    output logic                gemm_cmd_acc,     // K-tiling accumulate: vd[2]
    output logic [1:0]          gemm_cmd_prec,    // precision: vd[1:0]
    output logic [15:0]         gemm_cmd_k,       // K_tile per GEMM call: imm
    output logic                gemm_cmd_skip_store, // skip store phase: vd[3]
    output logic [3:0]          gemm_cmd_expert_sel, // vs2: 0=direct, 1-4=topk slot
    output logic [1:0]          gemm_cmd_proj,       // 0=gate, 1=up, 2=down
    output logic [2:0]          gemm_cmd_direct_nbits,

    // --- PostProc engine commands ---
    output logic                softmax_cmd_valid,
    output logic [3:0]          softmax_cmd_vd,
    output logic [3:0]          softmax_cmd_vs1,
    output logic [3:0]          softmax_cmd_alpha_vd,
    output logic [6:0]          softmax_cmd_state_slot,
    output logic [5:0]          softmax_cmd_valid_lanes,
    output logic                softmax_cmd_online,
    output logic                softmax_cmd_online_reset,
    output logic                softmax_cmd_raw_output,
    output logic                softmax_cmd_finalize,

    output logic                rmsnorm_cmd_valid,
    output logic [3:0]          rmsnorm_cmd_vd,
    output logic [3:0]          rmsnorm_cmd_vs1,
    output logic [3:0]          rmsnorm_cmd_vs2,
    output logic [1:0]          rmsnorm_cmd_mode,
    output logic                rmsnorm_cmd_reset,
    output logic [15:0]         rmsnorm_cmd_inv_count,

    output logic                rope_cmd_valid,
    output logic [3:0]          rope_cmd_vd,
    output logic [3:0]          rope_cmd_vs1,
    output logic [3:0]          rope_cmd_vs2,

    output logic                silu_cmd_valid,
    output logic [3:0]          silu_cmd_vd,
    output logic [3:0]          silu_cmd_vs1,
    output logic                silu_cmd_mode,

    output logic                add_cmd_valid,
    output logic [3:0]          add_cmd_vd,
    output logic [3:0]          add_cmd_vs1,
    output logic [3:0]          add_cmd_vs2,

    output logic                mul_cmd_valid,
    output logic [3:0]          mul_cmd_vd,
    output logic [3:0]          mul_cmd_vs1,
    output logic [3:0]          mul_cmd_vs2,

    // --- DMA commands ---
    output logic                dma_rd_cmd_valid,
    output logic [15:0]         dma_rd_cmd_addr,
    output logic [3:0]          dma_rd_cmd_vd,    // L1 write bank select (vd field)
    output logic                dma_rd_cmd_token_stride,
    output logic                dma_rd_cmd_l1_token_stride,
    output logic                dma_rd_cmd_out_token_group,
    output logic [1:0]          dma_rd_cmd_out_token_base,
    output logic                dma_rd_cmd_out_token_addr_hi,
    output logic                dma_rd_cmd_head_gather_wide,
    output logic                dma_rd_cmd_static,
    output logic                dma_rd_cmd_group_gather,
    output logic                dma_rd_cmd_group_gather_out,
    output logic                dma_rd_cmd_out_token,
    output logic [3:0]          dma_rd_cmd_len_code,
    output logic                dma_rd_cmd_k_override,
    output logic [8:0]          dma_rd_cmd_transfer_k,

    output logic                dma_wr_cmd_valid,
    output logic [3:0]          dma_wr_cmd_vd,
    output logic [15:0]         dma_wr_cmd_addr,
    output logic                dma_wr_cmd_token_stride,
    output logic                dma_wr_cmd_linear_burst,
    output logic                dma_wr_cmd_page,
    output logic                dma_wr_cmd_head_scatter,
    output logic                dma_wr_cmd_head_source1,
    output logic [3:0]          dma_wr_cmd_len_code,

    // --- TopK + Expert indirect addressing ---
    output logic                topk_cmd_valid,      // trigger TopK unit
    output logic [4:0]          topk_cmd_token_row,   // route-cache token ID
    output logic [4:0]          topk_cmd_acc_row,     // source row in acc_buffer
    output logic                route_cmd_valid,
    output logic [4:0]          route_cmd_token,
    output logic                route_group_cmd_valid,
    output logic [5:0]          route_group_cmd_expert,
    output logic [5:0]          route_group_cmd_tokens,
    output logic [4:0]          route_group_cmd_start,
    output logic                rweight_cmd_valid,
    output logic [3:0]          rweight_cmd_vd,
    output logic [3:0]          rweight_cmd_slot,
    output logic                group_weight_cmd_valid,
    output logic [3:0]          group_weight_cmd_vd,
    output logic [4:0]          group_weight_cmd_row,
    output logic                bcast_cmd_valid,
    output logic [3:0]          bcast_cmd_vd,
    output logic [3:0]          bcast_cmd_vs1,
    output logic [4:0]          bcast_cmd_lane,
    output logic                l1_accum_clear_cmd_valid,
    output logic [11:0]         l1_accum_clear_words,
    output logic                token_loop_begin,
    output logic                token_loop_end,
    output logic [5:0]          token_loop_count,
    output logic                expert_loop_begin,
    output logic                expert_loop_end,
    output logic [5:0]          expert_loop_count,
    output logic [3:0]          dma_rd_cmd_expert_sel,// vs2: 0=direct, 1-4=indirect
    output logic [1:0]          dma_rd_cmd_proj,     // vs1[1:0]: 0=gate,1=up,2=down
    output logic                scale_cmd_valid,     // NOP_SCALE: trigger per-expert scale load
    output logic [7:0]          scale_cmd_ntile,     // NOP_SCALE: N-tile index (imm[7:0])
    output logic                scale_cmd_current_expert_only, // vd[0]: grouped route descriptor mode
    output logic                static_scale_cmd_valid,
    output logic [11:0]         static_scale_cmd_addr,
    output logic [6:0]          static_scale_cmd_beats,
    output logic                static_wpage_cmd_valid,
    output logic [11:0]         static_wpage_cmd_page,
    output logic                tgld_cmd_valid,
    output logic [10:0]         tgld_cmd_base_line,
    output logic [5:0]          tgld_cmd_row_stride,
    output logic                tgld_cmd_transpose,
    output logic                cfg_k_write_valid,
    output logic [11:0]         cfg_k_write_value,
    output logic                cfg_m_write_valid,
    output logic [5:0]          cfg_m_write_value,

    // --- AC2V: acc_buffer -> PostProc RF (NOP_AC2V) ---
    output logic                ac2v_cmd_valid,      // NOP_AC2V: trigger acc drain
    output logic [3:0]          ac2v_cmd_vd,         // PRF 目的寄存器 (vd)
    output logic [4:0]          ac2v_cmd_row,        // acc 行号 (imm[4:0])
    output logic                ac2v_cmd_set,        // set 选择 (imm[5])

    // --- Control ---
    output logic                barrier_active,
    output logic                program_end,

    // --- Loop control (to fetch unit) ---
    output logic                loop_cmd_valid,     // current instruction is LOOP
    output logic [3:0]           loop_body_len,      // body length (1-15)
    output logic [11:0]          loop_count_init,    // iteration count
    output logic [1:0]           loop_stride_code,   // stride code

    // --- Scoreboard acquire (8-bit) ---
    output logic [7:0]          res_acquire     // which resources this instruction uses
);

    // Decode fields
    decoded_instr_t dec;
    assign dec = decode_instr(instr_data);

    // Hazard check: can we issue this instruction?
    logic can_issue;
    logic [7:0] needed_resources;

    always_comb begin
        needed_resources = 8'b0;
        case (dec.opcode)
            OP_VLD:      needed_resources[RES_POSTPROC] = 1'b1;
            OP_VST:      needed_resources[RES_POSTPROC] = 1'b1;
            OP_GLD_W:    needed_resources[RES_WRF_WR] = 1'b1;
            OP_GLD_A:    needed_resources[RES_ARF_WR] = 1'b1;
            OP_GEMM:     begin
                needed_resources[RES_GEMM]   = 1'b1;
                needed_resources[RES_WRF_RD] = 1'b1;  // GEMM reads active WRF
                needed_resources[RES_ARF_RD] = 1'b1;  // GEMM reads active ARF
            end
            OP_SOFTMAX, OP_RMSNORM, OP_ROPE, OP_SILU,
            OP_ADD, OP_MUL:
                         needed_resources[RES_POSTPROC] = 1'b1;
            OP_DMA_RD:   needed_resources[RES_DMA_RD] = 1'b1;
            OP_DMA_WR:   needed_resources[RES_DMA_WR] = 1'b1;
            OP_NOP:      // Extended operations acquire their physical target.
                         if (dec.imm[15:12] == NOP_AC2V ||
                             dec.imm[15:12] == NOP_RWEIGHT ||
                             dec.imm[15:12] == NOP_GROUP_WEIGHT ||
                             dec.imm[15:12] == NOP_BCAST)
                             needed_resources[RES_POSTPROC] = 1'b1;
                         else if (dec.imm[15:12] == NOP_TGLD)
                             needed_resources[RES_WRF_WR] = 1'b1;
            default:     needed_resources = 8'b0;
        endcase

        // Standard resource conflict check
        can_issue = !(|(needed_resources & res_busy));
        // AC2V consumes the accumulator written by GEMM. Wait only for that
        // producer instead of requiring a global BARRIER, so unrelated DMA
        // and RF work can remain in flight while the drain is issued.
        if (dec.opcode == OP_NOP && dec.imm[15:12] == NOP_AC2V &&
            res_busy[RES_GEMM])
            can_issue = 1'b0;
        if (l1_clear_busy)
            can_issue = 1'b0;
        if (route_unit_busy)
            can_issue = 1'b0;
        // GLD.A is serialized behind DMA by default because transposed/batched
        // sources can span multiple L1 banks. A compiler may opt in only after
        // proving its source range is disjoint from the DMA staging banks:
        // simple GLD.A uses vs1=1; batch transpose uses imm[1]. Strided batch
        // transpose reserves imm[2] for waiting on a prior PostProc store.
        // Existing padded/split transpose fields remain unchanged. GLD.W may overlap
        // a tracked weight DMA only when its source bank differs.
        // Unknown/legacy DMA destinations retain the conservative hazard.
        if (dec.opcode == OP_GLD_A && res_busy[RES_DMA_RD] &&
            !((dec.vs1 == 4'h1 ||
               (dec.vs1[3:2] == 2'b11 && dec.imm[1])) &&
              (dma_rd_l1_bank_valid === 1'b1) &&
              (dma_rd_l1_bank[3:2] === 2'b00)))
            can_issue = 1'b0;
        if (dec.opcode == OP_GLD_A &&
            ((dec.vd == 0 &&
              ((!dec.vs1[3] && dec.vs2[0]) ||
               (dec.vs1[3] && dec.vs1[2] && !dec.vs1[0] &&
                dec.vs2 == 4'd1))) ||
             (dec.vs1[3:2] == 2'b11 && !dec.vs1[0] && dec.imm[2])) &&
            (!loop_active || loop_iter == 0) &&
            res_busy[RES_POSTPROC])
            can_issue = 1'b0;
        if (dec.opcode == OP_GLD_W && res_busy[RES_DMA_RD] &&
            ((dma_rd_l1_bank_valid !== 1'b1) ||
             (dec.imm[15:12] ^
              ((loop_active && loop_iter[0]) ? 4'h1 : 4'h0)) ==
                 dma_rd_l1_bank))
            can_issue = 1'b0;
        // Do not start a consumer GEMM until both inactive RF writes finish;
        // top.sv commits their deferred active-bank flips on GEMM completion.
        if (dec.opcode == OP_GEMM &&
            (res_busy[RES_WRF_WR] || res_busy[RES_ARF_WR]))
            can_issue = 1'b0;
        // GEMM overwrites the shared accumulator. Do not start a new output
        // tile while AC2V/vector drain operations from the preceding tile are
        // still consuming it; this targeted dependency replaces a full idle
        // fence and lets the next weight/activation loads overlap the drain.
        if (dec.opcode == OP_GEMM && res_busy[RES_POSTPROC])
            can_issue = 1'b0;
        // GLD.W's dequant parameters come from the SRAM populated by the
        // asynchronous scale loader. This explicit consumer dependency also
        // covers prefetched weights whose DMA completed before NOP_SCALE.
        if (dec.opcode == OP_GLD_W && scale_loader_busy)
            can_issue = 1'b0;
        // Route changes must not race outstanding expert DMA/scale/GEMM.
        // TopK and SCALE have busy state outside the resource scoreboard.
        // TOKEN_BEGIN vd[0] may overlap only an unrelated PostProc drain.
        // vd[1] may additionally overlap a GEMM and its RF reads, but only
        // when compiler-managed ping/pong L1 buffers keep the DMA writes
        // disjoint from the in-flight tile. TOKEN_END remains all-idle gated.
        if ((dec.opcode == OP_NOP &&
             (dec.imm[15:12] == NOP_TOPK || dec.imm[15:12] == NOP_ROUTE ||
              dec.imm[15:12] == NOP_TOKEN_END ||
              dec.imm[15:12] == NOP_SET_CFG_M)) || dec.opcode == OP_END)
            can_issue = can_issue && all_idle;
        if (dec.opcode == OP_NOP &&
            dec.imm[15:12] == NOP_TOKEN_BEGIN) begin
            if (dec.vd[1]) begin
                can_issue = can_issue &&
                            ((res_busy & 8'h33) == 8'b0) &&
                            !route_unit_busy && !l1_clear_busy &&
                            !scale_loader_busy;
            end else if (dec.vd[0]) begin
                can_issue = can_issue &&
                            ((res_busy & 8'hF7) == 8'b0) &&
                            !route_unit_busy && !l1_clear_busy &&
                            !scale_loader_busy;
            end else begin
                can_issue = can_issue && all_idle;
            end
        end
        // CFG_K is sampled when DMA/GLD commands are accepted, while GEMM
        // captures its K from the instruction immediate. Permit a K update
        // to issue during an in-flight GEMM or PostProc drain, but never while
        // a reader/writer that consumes cfg_K can still observe the old value.
        // This lets mixed-K software pipelines set up the next tile's width
        // without serializing behind the current systolic-array computation.
        if (dec.opcode == OP_NOP && dec.imm[15:12] == NOP_SET_CFG_K &&
            (res_busy[RES_DMA_RD] || res_busy[RES_WRF_WR] ||
             res_busy[RES_ARF_WR] || res_busy[RES_POSTPROC] || loop_active))
            can_issue = 1'b0;
        // Scale metadata is loaded into scale_zero_mem; scale_rd_ctrl keeps
        // separate banked snapshots for the active GEMM. It is therefore safe
        // to prefetch the next scale tile during GEMM, while the shared AXI
        // read port and any in-flight GLD.W remain protected. GLD.W itself
        // waits for scale_loader_busy before copying metadata into its target
        // context.
        if (dec.opcode == OP_NOP &&
            (dec.imm[15:12] == NOP_SCALE ||
             dec.imm[15:12] == NOP_STATIC_SCALE) &&
            (scale_loader_busy || res_busy[RES_DMA_RD] ||
             res_busy[RES_WRF_WR]))
            can_issue = 1'b0;
        // The static weight-page register is sampled only when a later
        // static DMA command forms its DDR address. It may be changed while
        // GEMM/PostProc work drains, but never during an active read.
        if (dec.opcode == OP_NOP &&
            dec.imm[15:12] == NOP_SET_WPAGE && res_busy[RES_DMA_RD])
            can_issue = 1'b0;
        if (dec.opcode == OP_DMA_RD && dec.vs1[3] && dec.vs1[2] &&
            dec.vs2 == 4'd0)
            can_issue = can_issue && all_idle;
        if (dec.opcode == OP_NOP && dec.imm[15:12] == 4'd0 &&
            dec.imm[11:0] != 0 &&
            dec.vd == 4'hE && dec.vs1 == 4'hE && dec.vs2 == 4'hF)
            can_issue = can_issue && all_idle;
        if (dec.opcode == OP_NOP && dec.imm[15:12] == NOP_NOP &&
            ((dec.vd == 4'hE && dec.vs1 == 4'hE && dec.vs2 == 4'hD) ||
             (dec.vd == 4'hE && dec.vs1 == 4'hD && dec.vs2 == 4'hE)))
            can_issue = can_issue && all_idle;

        // TopK sync: expert-addressed DMA_RD/GLD_W/GEMM and NOP_SCALE must wait
        // until TopK has completed (topk_idx_arr stable) -- runtime expert
        // precision lookup depends on it.
        // Static DMA uses vs2 as a byte-length code. Direct head-field
        // gathers use vs2[3] as a marker for ACT/WGT (vs1=2/3), while OUT
        // gathers use the reserved vs1=E form. Keep both independent of Top-K.
        if ((dec.opcode == OP_DMA_RD && dec.vs2 != 4'd0 &&
             ((!dec.vs1[3] && !dec.vs1[2] &&
               !((dec.vs1 == 4'h2 || dec.vs1 == 4'h3) && dec.vs2[3])) ||
              (dec.vs1[3] && dec.vs1[2] && dec.vs1 != 4'hE))) ||
            (dec.opcode == OP_GLD_W  && dec.vs2 != 4'd0 && !dec.vs1[3]) ||
            (dec.opcode == OP_GEMM   && dec.vs2 != 4'd0 && !dec.vs1[3]) ||
            (dec.opcode == OP_NOP    && (dec.imm[15:12] == NOP_SCALE ||
                                        dec.imm[15:12] == NOP_RWEIGHT ||
                                        dec.imm[15:12] == NOP_GROUP_WEIGHT))) begin
            if (!topk_done_sig)
                can_issue = 1'b0;
        end

        // GEMM also needs data-valid (GLD must have finished filling RF)
        if (dec.opcode == OP_GEMM && (!wrf_data_valid || !arf_data_valid))
            can_issue = 1'b0;
    end

    // Stall if resources busy or if BARRIER waiting
    assign instr_ready = instr_valid && can_issue && !barrier_active;
    assign res_acquire = (instr_valid && can_issue && !barrier_active) ? needed_resources : 8'b0;

    // Dispatch logic
    always_comb begin
        // Default all outputs inactive
        vld_cmd_valid     = 1'b0;
        route_cmd_valid = 1'b0;
        route_cmd_token = '0;
        route_group_cmd_valid = 1'b0;
        route_group_cmd_expert = '0;
        route_group_cmd_tokens = '0;
        route_group_cmd_start = '0;
        rweight_cmd_valid = 1'b0;
        rweight_cmd_vd = '0;
        rweight_cmd_slot = '0;
        group_weight_cmd_valid = 1'b0;
        group_weight_cmd_vd = '0;
        group_weight_cmd_row = '0;
        bcast_cmd_valid = 1'b0;
        bcast_cmd_vd = '0;
        bcast_cmd_vs1 = '0;
        bcast_cmd_lane = '0;
        l1_accum_clear_cmd_valid = 1'b0;
        l1_accum_clear_words = '0;
        token_loop_begin = 1'b0;
        token_loop_end = 1'b0;
        token_loop_count = '0;
        expert_loop_begin = 1'b0;
        expert_loop_end = 1'b0;
        expert_loop_count = '0;
        vld_cmd_vd        = '0;
        vld_cmd_addr      = '0;
        vld_cmd_page = 2'd0;
        vld_cmd_token_stride_mode = 1'b0;
        vld_cmd_token_stride_bytes = '0;
        vst_cmd_valid     = 1'b0;
        vst_cmd_vs1       = '0;
        vst_cmd_addr      = '0;
        vst_cmd_page = 2'd0;
        vst_cmd_token_stride_mode = 1'b0;
        vst_cmd_token_stride_bytes = '0;
        gld_w_cmd_valid   = 1'b0;
        gld_w_cmd_addr    = '0;
        gld_w_cmd_prec    = '0;
        gld_w_cmd_group   = '0;
        gld_w_cmd_expert_sel = '0;
        gld_w_cmd_proj    = '0;
        gld_w_cmd_direct_nbits = '0;
        gld_a_cmd_valid   = 1'b0;
        gld_a_cmd_addr    = '0;
        gld_a_cmd_batch   = 1'b0;
        gld_a_cmd_transpose = 1'b0;
        gld_a_cmd_page = 2'd0;
        gld_a_cmd_stride_lines = 9'd0;
        gemm_cmd_valid    = 1'b0;
        gemm_cmd_acc      = 1'b0;
        gemm_cmd_prec     = 2'b00;
        gemm_cmd_k        = 16'd0;
        gemm_cmd_skip_store = 1'b0;
        gemm_cmd_expert_sel = '0;
        gemm_cmd_proj     = '0;
        gemm_cmd_direct_nbits = '0;
        softmax_cmd_valid = 1'b0;
        softmax_cmd_vd    = '0;
        softmax_cmd_vs1   = '0;
        softmax_cmd_alpha_vd = '0;
        softmax_cmd_state_slot = '0;
        softmax_cmd_valid_lanes = 6'd32;
        softmax_cmd_online = 1'b0;
        softmax_cmd_online_reset = 1'b0;
        softmax_cmd_raw_output = 1'b0;
        softmax_cmd_finalize = 1'b0;
        rmsnorm_cmd_valid = 1'b0;
        rmsnorm_cmd_vd    = '0;
        rmsnorm_cmd_vs1   = '0;
        rmsnorm_cmd_vs2   = '0;
        rmsnorm_cmd_mode  = '0;
        rmsnorm_cmd_reset = 1'b0;
        rmsnorm_cmd_inv_count = '0;
        rope_cmd_valid    = 1'b0;
        rope_cmd_vd       = '0;
        rope_cmd_vs1      = '0;
        rope_cmd_vs2      = '0;
        silu_cmd_valid    = 1'b0;
        silu_cmd_vd       = '0;
        silu_cmd_vs1      = '0;
        silu_cmd_mode     = 1'b0;
        add_cmd_valid     = 1'b0;
        add_cmd_vd        = '0;
        add_cmd_vs1       = '0;
        add_cmd_vs2       = '0;
        mul_cmd_valid     = 1'b0;
        mul_cmd_vd        = '0;
        mul_cmd_vs1       = '0;
        mul_cmd_vs2       = '0;
        dma_rd_cmd_valid  = 1'b0;
        dma_rd_cmd_addr   = '0;
        dma_rd_cmd_vd     = '0;
        dma_rd_cmd_token_stride = 1'b0;
        dma_rd_cmd_l1_token_stride = 1'b0;
        dma_rd_cmd_out_token_group = 1'b0;
        dma_rd_cmd_out_token_base = 2'd0;
        dma_rd_cmd_out_token_addr_hi = 1'b0;
        dma_rd_cmd_head_gather_wide = 1'b0;
        dma_rd_cmd_static = 1'b0;
        dma_rd_cmd_group_gather = 1'b0;
        dma_rd_cmd_group_gather_out = 1'b0;
        dma_rd_cmd_out_token = 1'b0;
        dma_rd_cmd_len_code = '0;
        dma_rd_cmd_k_override = 1'b0;
        dma_rd_cmd_transfer_k = '0;
        dma_wr_cmd_valid  = 1'b0;
        dma_wr_cmd_vd     = '0;
        dma_wr_cmd_addr   = '0;
        dma_wr_cmd_token_stride = 1'b0;
        dma_wr_cmd_linear_burst = 1'b0;
        dma_wr_cmd_page = 1'b0;
        dma_wr_cmd_head_scatter = 1'b0;
        dma_wr_cmd_head_source1 = 1'b0;
        dma_wr_cmd_len_code = '0;
        topk_cmd_valid    = 1'b0;
        topk_cmd_token_row = '0;
        topk_cmd_acc_row = '0;
        scale_cmd_valid   = 1'b0;
        scale_cmd_ntile   = '0;
        scale_cmd_current_expert_only = 1'b0;
        static_scale_cmd_valid = 1'b0;
        static_scale_cmd_addr = '0;
        static_scale_cmd_beats = '0;
        static_wpage_cmd_valid = 1'b0;
        static_wpage_cmd_page = '0;
        tgld_cmd_valid = 1'b0;
        tgld_cmd_base_line = '0;
        tgld_cmd_row_stride = '0;
        tgld_cmd_transpose = 1'b1;
        cfg_k_write_valid = 1'b0;
        cfg_k_write_value = '0;
        cfg_m_write_valid = 1'b0;
        cfg_m_write_value = '0;
        ac2v_cmd_valid    = 1'b0;
        ac2v_cmd_vd       = '0;
        ac2v_cmd_row      = '0;
        ac2v_cmd_set      = 1'b0;
        dma_rd_cmd_expert_sel = '0;
        dma_rd_cmd_proj   = '0;
        barrier_active    = 1'b0;
        program_end       = 1'b0;
        loop_cmd_valid    = 1'b0;
        loop_body_len     = 4'd0;
        loop_count_init   = 12'd0;
        loop_stride_code  = 2'b00;

        if (instr_valid && can_issue) begin
            case (dec.opcode)
                OP_VLD: begin
                    vld_cmd_valid = 1'b1;
                    vld_cmd_vd    = dec.vd;
                    vld_cmd_token_stride_mode = dec.vs1[3];
                    // Bit 2 selects a flat page only when token-stride mode
                    // is off; token-stride mode stores the page in the
                    // otherwise-unused low five immediate bits.
                    vld_cmd_page = dec.vs1[3]
                                   ? dec.imm[1:0]
                                   : dec.vs1[1]
                                   ? 2'd2
                                   : (dec.vs1[2]
                                      ? 2'd1 : 2'd0);
                    vld_cmd_token_stride_bytes =
                        ({6'd0, dec.vs1[2:0], dec.vs2} + 13'd1) << 5;
                    // The strided form addresses the same vector field in
                    // successive token-major L1 rows.  Legacy encodings keep
                    // vs1[3] clear and therefore preserve the immediate.
                    vld_cmd_addr  = (dec.vs1[3]
                                      ? {dec.imm[15:5], 5'b0} : dec.imm)
                                      + (dec.vs1[3]
                                      ? token_loop_iter *
                                        (({9'd0, dec.vs1[2:0], dec.vs2} + 16'd1) << 5)
                                      : 16'd0);
                end

                OP_VST: begin
                    vst_cmd_valid = 1'b1;
                    vst_cmd_vs1   = dec.vs1;
                    vst_cmd_token_stride_mode = dec.vd[3];
                    vst_cmd_page = dec.vd[3]
                                   ? dec.imm[1:0]
                                   : dec.vs2[0]
                                   ? 2'd2
                                   : (dec.vd[2]
                                      ? 2'd1 : 2'd0);
                    vst_cmd_token_stride_bytes =
                        ({6'd0, dec.vd[2:0], dec.vs2} + 13'd1) << 5;
                    vst_cmd_addr  = (dec.vd[3]
                                      ? {dec.imm[15:5], 5'b0} : dec.imm)
                                      + (dec.vd[3]
                                      ? token_loop_iter *
                                        (({9'd0, dec.vd[2:0], dec.vs2} + 16'd1) << 5)
                                      : 16'd0);
                end

                OP_GLD_W: begin
                    gld_w_cmd_valid = 1'b1;
                    if (static_k128_replay) begin
                        // Cached K=128 replay packs direct nbits in vs2[2:0],
                        // scale-group[3] in vs2[3], and group[2:0] in vs1.
                        // vs1 may equal 4'hF for group 7; do not mistake that
                        // for the extended expert-slot encoding.
                        gld_w_cmd_addr = dec.imm;
                        gld_w_cmd_group = {2'd0, dec.vs2[3], dec.vs1[2:0]};
                        gld_w_cmd_expert_sel = 4'd0;
                        gld_w_cmd_direct_nbits = dec.vs2[2:0];
                    end else if (dec.vs1 == 4'hF && dec.vs2 >= 4'd1 && dec.vs2 <= 4'd4) begin
                        gld_w_cmd_addr = {dec.imm[15:12], 12'd0};
                        gld_w_cmd_group = {2'd0, dec.imm[5:0]};
                    end else begin
                        gld_w_cmd_addr  = dec.imm ^
                                          ((loop_active && loop_iter[0]) ? 16'h1000 : 16'h0000);
                        gld_w_cmd_group = (loop_active && dec.vs2 != 0 && !dec.vs1[3])
                                          ? ((loop_iter * (cfg_K >> 5)) >> 2)
                                          : {2'd0,
                                             (dec.vs1[3] ? dec.vs2[3] : 1'b0),
                                             dec.vs1[2:0]};
                    end
                    gld_w_cmd_prec  = dec.vd[1:0];
                    if (!static_k128_replay) begin
                        gld_w_cmd_expert_sel = (dec.vs1 == 4'hF &&
                                                dec.vs2 >= 4'd1 && dec.vs2 <= 4'd4)
                                               ? dec.vs2
                                               : dec.vs1[3] ? 4'd0 : dec.vs2;
                        gld_w_cmd_direct_nbits = (dec.vs1 == 4'hF &&
                                                  dec.vs2 >= 4'd1 && dec.vs2 <= 4'd4)
                                                 ? 3'd0
                                                 : dec.vs1[3] ? dec.vs2[2:0] : 3'd0;
                    end
                    gld_w_cmd_proj      = dec.vd[3:2];  // 0=gate, 1=up, 2=down
                end

                OP_GLD_A: begin
                    gld_a_cmd_valid = 1'b1;
                    gld_a_cmd_batch = dec.vs1[3];
                    gld_a_cmd_transpose = dec.vs1[3] && dec.vs1[2];
                    // vd==0, vs2==1 remains the legacy PostProc-wait form;
                    // other nonzero vs2 values can encode short row strides.
                    if (dec.vs1[3:2] == 2'b11 && !dec.vs1[0] &&
                        dec.vs2 != 0 &&
                        (dec.vd != 0 || dec.vs2 != 4'd1))
                        gld_a_cmd_stride_lines = {dec.imm[0], dec.vd, dec.vs2} + 9'd1;
                    gld_a_cmd_page = (dec.vd != 0 && dec.vs2 == 0)
                                     ? 2'd2
                                     : {1'b0, dec.vs1[1]};
                    if (dec.vd != 0 && dec.vs1[0] && dec.vs2 != 0 && loop_active &&
                        loop_iter >= dec.vs2) begin
                        // Split route-group intermediate: the first K chunks
                        // reside in the selected page; the spill range starts
                        // at vd*4KiB in the opposite page.
                        gld_a_cmd_page = {1'b0, ~dec.vs1[1]};
                        gld_a_cmd_addr = {dec.vd, 12'b0} +
                            (loop_iter - dec.vs2) *
                            (dec.vs1[3]
                                ? (dec.vs1[0] ? (cfg_K * 16'd64)
                                             : (cfg_K * cfg_M * 16'd2))
                                : (cfg_K * 16'd2));
                    end else begin
                        gld_a_cmd_addr = dec.imm + (loop_active
                            ? loop_iter * (dec.vs1[3]
                                ? ((dec.vd == 0 && dec.vs1[0] &&
                                    dec.vs2 != 0)
                                    ? (cfg_K * {12'd0, dec.vs2} * 16'd2)
                                    : (dec.vd != 0 && !dec.vs1[0] &&
                                       dec.vs2 != 0)
                                    ? (cfg_K * 16'd2)
                                    : dec.vs1[0] ? (cfg_K * 16'd64)
                                                 : (cfg_K * cfg_M * 16'd2))
                                : (cfg_K * 16'd2))
                            : 16'd0);
                    end
                end

                OP_GEMM: begin
                    gemm_cmd_valid = 1'b1;
                    // Loop-managed static quantized groups use vd[2:3] as
                    // cross-group accumulate / final-store controls; loop
                    // iteration still handles the within-group K reduction.
                    gemm_cmd_acc   = (loop_active &&
                                      ((dec.vs2 != 0 && !dec.vs1[3]) || dec.vs1[3:2] == 2'b11))
                                     ? ((loop_iter != 0) ||
                                        (dec.vs1[3:2] == 2'b11 && dec.vd[2]))
                                     : dec.vd[2];
                    gemm_cmd_prec  = dec.vd[1:0];    // precision mode
                    gemm_cmd_k     = dec.imm;         // K_tile (per-instruction)
                    gemm_cmd_skip_store = (loop_active &&
                                           ((dec.vs2 != 0 && !dec.vs1[3]) || dec.vs1[3:2] == 2'b11))
                                          ? ((loop_remaining != 1) ||
                                             (dec.vs1[3:2] == 2'b11 && dec.vd[3]))
                                          : dec.vd[3];
                    gemm_cmd_expert_sel = dec.vs1[3] ? 4'd0 : dec.vs2;
                    gemm_cmd_direct_nbits = dec.vs1[3] ? dec.vs2[2:0] : 3'd0;
                    gemm_cmd_proj      = dec.vs1[1:0]; // 0=gate, 1=up, 2=down
                end

                OP_SOFTMAX: begin
                    softmax_cmd_valid = 1'b1;
                    softmax_cmd_vd    = dec.vd;
                    softmax_cmd_vs1   = dec.vs1[3] && dec.imm[14]
                                         ? {1'b0, dec.vs1[2:0]} : dec.vs1;
                    softmax_cmd_alpha_vd = dec.vs2;
                    softmax_cmd_state_slot = dec.vs1[3] && dec.imm[14]
                                             ? dec.imm[14:8]
                                             : {1'b0, dec.imm[13], dec.imm[12:8]};
                    softmax_cmd_valid_lanes = (dec.imm[5:0] == 0 ||
                                               dec.imm[5:0] > 6'd32)
                                              ? 6'd32 : dec.imm[5:0];
                    softmax_cmd_online = dec.imm[6];
                    softmax_cmd_online_reset = dec.imm[7];
                    softmax_cmd_raw_output = dec.imm[15];
                    softmax_cmd_finalize = dec.vs1[3] && dec.imm[14]
                                           ? !dec.imm[6] : dec.imm[14];
                end

                OP_RMSNORM: begin
                    rmsnorm_cmd_valid = 1'b1;
                    rmsnorm_cmd_vd    = dec.vd;
                    rmsnorm_cmd_vs1   = dec.vs1;
                    rmsnorm_cmd_vs2   = dec.vs2;
                    rmsnorm_cmd_mode  = dec.imm[15:14];
                    rmsnorm_cmd_reset = dec.imm[13];
                    rmsnorm_cmd_inv_count = {dec.vd, dec.vs1, dec.vs2,
                                             dec.imm[3:0]};
                end

                OP_ROPE: begin
                    rope_cmd_valid = 1'b1;
                    rope_cmd_vd    = dec.vd;
                    rope_cmd_vs1   = dec.vs1;
                    rope_cmd_vs2   = dec.vs2;
                end

                OP_SILU: begin
                    silu_cmd_valid = 1'b1;
                    silu_cmd_vd    = dec.vd;
                    silu_cmd_vs1   = dec.vs1;
                    silu_cmd_mode  = dec.imm[0];
                end

                OP_ADD: begin
                    add_cmd_valid = 1'b1;
                    add_cmd_vd    = dec.vd;
                    add_cmd_vs1   = dec.vs1;
                    add_cmd_vs2   = dec.vs2;
                end

                OP_MUL: begin
                    mul_cmd_valid = 1'b1;
                    mul_cmd_vd    = dec.vd;
                    mul_cmd_vs1   = dec.vs1;
                    mul_cmd_vs2   = dec.vs2;
                end

                OP_DMA_RD: begin
                    dma_rd_cmd_valid = 1'b1;
                    // Legacy token gathers retain 16-byte immediate units.
                    // Attention head-gathers use 256-byte units to expose a
                    // 16-MiB window without changing their 4-KiB row stride.
                    dma_rd_cmd_out_token_group =
                        (dec.vs1 == 4'hE ||
                         (dec.vs1 == 4'hB && dec.vs2[3]) ||
                         ((dec.vs1 == 4'h2 || dec.vs1 == 4'h3 ||
                           dec.vs1 == 4'h6 || dec.vs1 == 4'h7) &&
                          dec.vs2[3]) ||
                         (dec.vs1 == 4'hD && dec.vs2[3]));
                    dma_rd_cmd_out_token_base =
                        (dec.vs1 == 4'h2 || dec.vs1 == 4'h6) ? 2'd1 :
                        (dec.vs1 == 4'h3 || dec.vs1 == 4'h7) ? 2'd2 : 2'd0;
                    dma_rd_cmd_out_token_addr_hi =
                        (dec.vs1 == 4'h6 || dec.vs1 == 4'h7 ||
                         (dec.vs1 == 4'hB && dec.vs2[3]));
                    dma_rd_cmd_head_gather_wide =
                        (((dec.vs1 == 4'h6 || dec.vs1 == 4'h7) &&
                          dec.vs2[3]) ||
                         (dec.vs1 == 4'hD && dec.vs2[3]));
                    dma_rd_cmd_k_override = !dma_rd_cmd_out_token_group &&
                        (((dec.vs1 == 4'hF) &&
                          (dec.vs2 >= 4'd2) && (dec.vs2 <= 4'd13)) ||
                         ((dec.vs1 == 4'hC || dec.vs1 == 4'hD) &&
                          (dec.vs2 != 4'd0)));
                    if (dma_rd_cmd_k_override) begin
                        if (dec.vs1 == 4'hF) begin
                            dma_rd_cmd_addr = {2'd0, dec.imm[13:0]};
                            case (dec.imm[15:14])
                                2'd0: dma_rd_cmd_transfer_k = 9'd256;
                                2'd1: dma_rd_cmd_transfer_k = 9'd32;
                                2'd2: dma_rd_cmd_transfer_k = 9'd64;
                                2'd3: dma_rd_cmd_transfer_k = 9'd128;
                            endcase
                        end else begin
                            dma_rd_cmd_addr = {3'd0, dec.imm[12:0]};
                            case (dec.imm[15:13])
                                3'd1: dma_rd_cmd_transfer_k = 9'd32;
                                3'd2: dma_rd_cmd_transfer_k = 9'd64;
                                3'd3: dma_rd_cmd_transfer_k = 9'd128;
                                3'd4: dma_rd_cmd_transfer_k = 9'd256;
                                default: dma_rd_cmd_transfer_k = '0;
                            endcase
                        end
                    end else begin
                        // Auto-increment uses logical K tiles for indirect
                        // expert DMA, compact tile-byte codes for static
                        // quantized DMA, and legacy 1/2-KiB strides otherwise.
                        dma_rd_cmd_addr = dec.imm + (loop_active &&
                                           !dma_rd_cmd_out_token_group &&
                                           !(dec.vs1[3] && dec.vs1[2])
                                           ? ((dec.vs1[2] && !dec.vs1[3] && dec.vs2 != 0)
                                              ? loop_iter * ((dec.vs2 == 4'd1) ? 16'd8 :
                                                             (dec.vs2 == 4'd2) ? 16'd16 :
                                                             (dec.vs2 == 4'd3) ? 16'd32 :
                                                             (dec.vs2 == 4'd4) ? 16'd64 : 16'd32)
                                              : (dec.vs2 != 0 && !dec.vs1[3])
                                                ? loop_iter * ((cfg_K + 16'd31) >> 5)
                                                : (dec.vs1[2] && !dec.vs1[3])
                                                  ? loop_iter * (dec.vs1[0] ? 16'd128 : 16'd64)
                                                  : loop_iter * addr_stride)
                                           : 16'd0);
                    end
                    // L1 banks are 4 KiB apart. Alternate adjacent bank IDs
                    // while a K loop streams into the inactive staging bank.
                    dma_rd_cmd_vd    = dec.vd ^
                                      ((loop_active && !(dec.vs1[3] && dec.vs1[2]) &&
                                        !(dec.vs1[3] && !dec.vs1[2] &&
                                          dec.vs1[1:0] == 2'b10) &&
                                        !(dec.vs1 == 4'hB && dec.vs2[3]) &&
                                        dec.vd != 4'd0 && loop_iter[0])
                                       ? 4'd1 : 4'd0);
                    // vs1[3] selects a token-strided activation transfer. In
                    // that mode vs2 encodes transfer bytes in 256-byte units
                    // minus one; it is not an expert selector.
                    // Token-group encoding vs1[1:0]=2'b10 also strides the
                    // L1 destination by the same transfer size per iteration.
                    dma_rd_cmd_group_gather = !dma_rd_cmd_out_token_group &&
                                              (((dec.vs1 == 4'hC ||
                                                 dec.vs1 == 4'hD) &&
                                                (dec.vs2 == 4'd0)) ||
                                               (dec.vs1 == 4'hF &&
                                                dec.vs2[3:1] == 3'd0));
                    dma_rd_cmd_group_gather_out = dma_rd_cmd_group_gather &&
                                                  (dec.vs1 == 4'hF);
                    dma_rd_cmd_out_token = (dec.vs1 == 4'h9);
                    dma_rd_cmd_token_stride = dec.vs1[3] && !dec.vs1[2];
                    dma_rd_cmd_l1_token_stride =
                        (dec.vs1[3] && !dec.vs1[2] &&
                         (dec.vs1[1:0] == 2'b10)) ||
                        dma_rd_cmd_out_token_group;
                    dma_rd_cmd_static = dec.vs1[2] && !dec.vs1[3];
                    dma_rd_cmd_len_code =
                        ((dec.vs1 == 4'h2 || dec.vs1 == 4'h3 ||
                          dec.vs1 == 4'h6 || dec.vs1 == 4'h7 ||
                          dec.vs1 == 4'hD) && dec.vs2[3]) ||
                        (dec.vs1 == 4'hB && dec.vs2[3])
                        ? {1'b0, dec.vs2[2:0]} : dec.vs2;
                    dma_rd_cmd_expert_sel = (dma_rd_cmd_out_token_group ||
                                             dma_rd_cmd_out_token) ? 4'd0 :
                                           dma_rd_cmd_group_gather ? 4'd1 :
                                           (dec.vs1 == 4'hF &&
                                            dec.vs2 >= 4'd2 &&
                                            dec.vs2 <= 4'd13)
                                           ? (((dec.vs2 - 4'd2) / 4'd3) + 4'd1) :
                                           (dec.vs1[3] && dec.vs1[2])
                                           ? ((dec.vs1 == 4'hC ||
                                               dec.vs1 == 4'hD)
                                              ? dec.vs2 : 4'd0)
                                           : (dec.vs1[3] || dec.vs1[2])
                                             ? 4'd0 : dec.vs2;
                    dma_rd_cmd_proj      = dma_rd_cmd_group_gather_out
                                           ? {1'b0, dec.vs2[0]}
                                           : ((dec.vs1 == 4'hF &&
                                               dec.vs2 >= 4'd2 &&
                                               dec.vs2 <= 4'd13)
                                              ? ((dec.vs2 - 4'd2) % 4'd3)
                                              : dec.vs1[1:0]); // 0=gate, 1=up, 2=down
                end

                OP_DMA_WR: begin
                    dma_wr_cmd_valid = 1'b1;
                    dma_wr_cmd_vd    = dec.vd;
                    dma_wr_cmd_addr  = dec.imm;
                    dma_wr_cmd_token_stride = dec.vs1[3] && !dec.vs1[0];
                    dma_wr_cmd_linear_burst = dec.vs1[3] && dec.vs1[0];
                    dma_wr_cmd_page = dec.vs1[2];
                    dma_wr_cmd_head_scatter = dec.vs1[1];
                    dma_wr_cmd_head_source1 = dec.vs1[0];
                    dma_wr_cmd_len_code = dec.vs2;
                end

                OP_NOP: begin
                    // Extended NOP sub-opcodes (imm[15:12])
                    case (dec.imm[15:12])
                        NOP_NOP: begin
                            if (dec.imm[11:0] != 0 && dec.vd == 4'hE &&
                                dec.vs1 == 4'hE && dec.vs2 == 4'hF)
                                l1_accum_clear_cmd_valid = 1'b1;
                            if (dec.imm[11:0] != 0 && dec.vd == 4'hE &&
                                dec.vs1 == 4'hE && dec.vs2 == 4'hF)
                                l1_accum_clear_words = dec.imm[11:0];
                            if (dec.imm[11:0] != 0 && dec.vd == 4'hE &&
                                dec.vs1 == 4'hE && dec.vs2 == 4'hD) begin
                                expert_loop_begin = 1'b1;
                                expert_loop_count = dec.imm[5:0];
                            end
                            if (dec.imm[11:0] == 0 && dec.vd == 4'hE &&
                                dec.vs1 == 4'hD && dec.vs2 == 4'hE)
                                expert_loop_end = 1'b1;
                        end
                        NOP_TOPK: begin
                            topk_cmd_valid    = 1'b1;
                            topk_cmd_token_row = dec.imm[5]
                                ? (dec.vs1[1]
                                   ? dec.imm[4:0] + token_loop_iter[4:0]
                                   : token_loop_iter[4:0])
                                : dec.imm[4:0];
                            // Legacy TOPK reads the same row it caches. The
                            // extended encoding decouples a local ACC row from
                            // the global token-cache ID for grouped prefill.
                            topk_cmd_acc_row = dec.vs1[1]
                                ? {dec.vs1[2], dec.vd}
                                : (dec.imm[5]
                                   ? token_loop_iter[4:0]
                                   : dec.imm[4:0]);
                        end
                        NOP_ROUTE: begin
                            if (dec.imm[6]) begin
                                route_group_cmd_valid = 1'b1;
                                route_group_cmd_expert = (dec.imm[5:0] == 6'h3F)
                                                       ? expert_loop_iter : dec.imm[5:0];
                                route_group_cmd_tokens = {1'b0, dec.imm[11:7]} + 6'd1;
                                route_group_cmd_start = {dec.vd[0], dec.vs1};
                            end else begin
                                route_cmd_valid = 1'b1;
                                route_cmd_token = dec.imm[5] ? token_loop_iter[4:0] : dec.imm[4:0];
                            end
                        end
                        NOP_RWEIGHT: begin
                            rweight_cmd_valid = 1'b1;
                            rweight_cmd_vd = dec.vd;
                            rweight_cmd_slot = dec.vs2;
                        end
                        NOP_GROUP_WEIGHT: begin
                            group_weight_cmd_valid = 1'b1;
                            group_weight_cmd_vd = dec.vd;
                            group_weight_cmd_row = dec.imm[4:0];
                        end
                        NOP_BCAST: begin
                            bcast_cmd_valid = 1'b1;
                            bcast_cmd_vd = dec.vd;
                            bcast_cmd_vs1 = dec.vs1;
                            bcast_cmd_lane = dec.imm[4:0];
                        end
                        NOP_SET_CFG_K: begin
                            cfg_k_write_valid = 1'b1;
                            cfg_k_write_value = dec.imm[11:0];
                        end
                        NOP_SET_CFG_M: begin
                            cfg_m_write_valid = 1'b1;
                            cfg_m_write_value = dec.imm[5:0];
                        end
                        NOP_TOKEN_BEGIN: begin
                            token_loop_begin = 1'b1;
                            token_loop_count = dec.imm[5:0];
                        end
                        NOP_TOKEN_END: token_loop_end = 1'b1;
                        NOP_SET_WPAGE: begin
                            static_wpage_cmd_valid = 1'b1;
                            static_wpage_cmd_page = dec.imm[11:0];
                        end
                        NOP_STATIC_SCALE: begin
                            static_scale_cmd_valid = 1'b1;
                            static_scale_cmd_addr = dec.imm[11:0];
                            static_scale_cmd_beats =
                                ({3'b0, dec.vd} + 7'd1) << 2;
                        end
                        NOP_TGLD: begin
                            tgld_cmd_valid = 1'b1;
                            tgld_cmd_base_line = dec.imm[10:0];
                            tgld_cmd_row_stride = {dec.vs2[1:0], dec.vd} + 6'd1;
                            tgld_cmd_transpose = !dec.vs1[0];
                        end
                        NOP_LOOP: begin
                            loop_cmd_valid    = 1'b1;
                            loop_body_len     = dec.vs1[3:0];  // body length (1-15)
                            loop_count_init   = dec.imm[11:0];  // iteration count
                            loop_stride_code  = dec.vd[3:2];   // stride code
                        end
                        NOP_SCALE: begin
                            scale_cmd_valid   = 1'b1;   // trigger per-expert scale load
                            scale_cmd_ntile   = dec.imm[7:0]; // N-tile index (scale region offset)
                            scale_cmd_current_expert_only = dec.vd[0];
                        end
                        NOP_AC2V: begin
                            ac2v_cmd_valid    = 1'b1;   // acc_buffer 一行 -> fp16 -> PRF
                            ac2v_cmd_vd       = dec.vd;
                            ac2v_cmd_row      = dec.imm[4:0];
                            ac2v_cmd_set      = dec.imm[5];
                        end
                        default: ; // genuine NOP
                    endcase
                end

                OP_BARRIER: begin
                    barrier_active = !all_idle;
                end

                OP_END: begin
                    program_end = 1'b1;
                end

                default: ; // NOP
            endcase
        end else if (instr_valid && dec.opcode == OP_BARRIER) begin
            barrier_active = !all_idle;
        end
    end

endmodule

`default_nettype wire
