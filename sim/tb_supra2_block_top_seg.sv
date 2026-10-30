// =============================================================================
// tb_supra2_block_top_seg.sv - Segmented one-block datapath testbench
//
// Memory-wall workaround (2026-10-21): the full connected block-7 run leaks
// ~20 KB per simulated cycle inside the relinked XSim kernel (>50 GB for the
// whole block; machine has 15.7 GB RAM). This testbench runs ONE segment of
// the block per kernel launch and chains segments through checkpoint files of
// the on-chip buffer contents, so each launch stays within a few GB.
//
// Segment control (integer plusargs only -- runtime string paths reach
// $readmemh as garbage in this kernel, so every file path below is a
// compile-time literal):
//   START_STAGE (def 0)   FSM stage to jump to (see supra2_block_top)
//   STOP_STAGE  (def 16)  dump+finish when this stage is ENTERED
//   START_HEAD  (def 0)   first attention head (attention stages only)
//   START_NI    (def 0)   GEMM engine column-tile slice origin
//   STOP_NI     (def 0)   GEMM engine column-tile slice end (0 = full width);
//                         [START_NI, STOP_NI) slices are disjoint, so a large
//                         GEMM stage can be partitioned across kernel launches
//   SLICE_HEAD  (def 0)   1 = stop when cur_head leaves START_HEAD
//                         (per-head self/cross-attention segments)
//
// Checkpoint files (fixed names, chained linearly between segments):
//   build/supra2_block7_top/ckpt/{x_buf,ln_buf,q_buf,k_buf,v_buf,cq_buf,
//                                 ck_buf,cv_buf,attn_buf,proj_buf,mlp_buf}.hex
// The driver seeds ckpt/x_buf.hex from the archive fixture before segment 0
// and deletes the rest, so a missing-file warning here always means "buffer
// not yet produced" (its contents are never consumed by that segment).
//
// Fixture and weight loads replicate tb_supra2_block_top, but x/ctx/param
// enter the DUT through time-0 hierarchical $readmemh instead of the
// cycle-by-cycle DMA element port (the DMA port models delivery, which is
// not what this run validates; skipping it saves ~170k cycles of avoidable
// value-change activity per segment). Weights still stream through the
// wload_req/ack DMA handshake per GEMM stage.
// =============================================================================
`default_nettype none

module tb_supra2_block_top_seg;
    localparam int XN = 256*576;
    localparam int HEADS = 9, CKEYS = 32, HD = 64;
    localparam int CTXN = 32*576;
    localparam int PN = 8*576;
    localparam int MAXW = 2304*576;
    localparam int MAXN = 2304;
    localparam int CKN = 9*32*64;
    localparam int MLP_N = 2304;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic start = 1'b0;
    logic busy, done;
    logic ld_en = 1'b0;
    logic [3:0] ld_target = '0;
    logic [20:0] ld_addr = '0;
    logic [15:0] ld_data = '0;
    logic wt_wr_en = 1'b0;
    logic [15:0] wt_wr_beat = '0;
    logic [255:0] wt_wr_data = '0;
    logic sb_wr_en = 1'b0;
    logic sb_wr_sel = 1'b0;
    logic [15:0] sb_wr_addr = '0;
    logic [31:0] sb_wr_data = '0;
    logic wload_req;
    logic [3:0] wload_stage;
    logic wload_ack = 1'b0;
    logic [4:0] stage_id;
    logic [4:0] start_stage_r;
    logic [4:0] stop_stage_r;
    logic [3:0] start_head_r;
    logic [15:0] start_ni_r;
    logic [15:0] stop_ni_r;

    integer start_stage = 0;
    integer stop_stage = 16;
    integer start_head = 0;
    integer start_ni = 0;
    integer stop_ni = 0;
    integer slice_head = 0;

    // part-selects, not size casts: XSim 2021.1 mis-evaluates runtime
    // integer size casts inside wait() expressions (observed 2026-10-21:
    // wait(stage_id == 5'(stop_stage)) never fired and the sim ran past
    // the stop stage)
    assign start_stage_r = start_stage[4:0];
    assign stop_stage_r = stop_stage[4:0];
    assign start_head_r = start_head[3:0];
    assign start_ni_r = start_ni[15:0];
    // STOP_NI must actually reach the DUT: the port and the header contract
    // existed but were never wired, so every ni slice ran [start_ni, cfg_n)
    // (overlapping suffixes) instead of the documented disjoint
    // [START_NI, STOP_NI) windows (watchdog fix, 2026-10-21).
    assign stop_ni_r = stop_ni[15:0];

    supra2_block_top dut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .start_stage(start_stage_r), .start_head(start_head_r),
        .start_ni(start_ni_r), .stop_ni(stop_ni_r),
        .busy(busy), .done(done),
        .ld_en(ld_en), .ld_target(ld_target), .ld_addr(ld_addr),
        .ld_data(ld_data),
        .ld_vec_en(1'b0), .ld_vec_target(4'd0), .ld_vec_beat(16'd0),
        .ld_vec_data(512'd0),
        .wt_wr_en(wt_wr_en), .wt_wr_beat(wt_wr_beat), .wt_wr_data(wt_wr_data),
        .sb_wr_en(sb_wr_en), .sb_wr_sel(sb_wr_sel), .sb_wr_addr(sb_wr_addr),
        .sb_wr_data(sb_wr_data),
        .result_vec_rd_beat(13'd0), .result_vec_rd_data(),
        .param_vec_wr_en(1'b0), .param_vec_wr_beat(8'd0), .param_vec_wr_data(512'd0),
        .wload_req(wload_req), .wload_stage(wload_stage),
        .wload_ack(wload_ack), .stage_id(stage_id)
    );

    always #5 clk = ~clk;

    // ---------------- fixture memories (weights + golden) ----------------
    logic [15:0] golden_mem [0:XN-1];
    logic [7:0]  w_mem [0:6][0:MAXW-1];
    logic [31:0] ws_mem [0:6][0:MAXN-1];
    logic [31:0] b_mem [0:6][0:MAXN-1];
    logic [7:0]  wtmp [0:MAXW-1];
    logic [31:0] stmp [0:MAXN-1];

    localparam int STG_N [0:6] = '{1728, 576, 576, 1152, 576, 2304, 576};
    localparam int STG_K [0:6] = '{576, 576, 576, 576, 576, 576, 2304};

    integer total_cycles;
    integer stage_mark;
    integer stage_cycles [0:16];
    logic [4:0] prev_stage;
    integer i, j, s;
    integer out_fd;

    // cycle counting + stage accounting
    always @(posedge clk) begin
        if (busy) total_cycles <= total_cycles + 1;
        if (busy && total_cycles != 0 && total_cycles % 100000 == 0)
            $display("PROBE cyc=%0d stage=%0d ss=%0d eng_state=%0d row=%0d mi=%0d ni=%0d ob=%0d head=%0d c1=%0d c2=%0d c3=%0d",
                     total_cycles, stage_id, dut.ss, dut.u_eng.state,
                     dut.u_eng.row, dut.u_eng.mi, dut.u_eng.ni,
                     dut.u_eng.obeat, dut.cur_head, dut.c1, dut.c2, dut.c3);
        if (stage_id != prev_stage) begin
            stage_cycles[prev_stage] <= total_cycles - stage_mark;
            stage_mark <= total_cycles;
            prev_stage <= stage_id;
            $display("SUPRA2_BLOCK_STAGE_ENTER id=%0d cycle=%0d",
                     stage_id, total_cycles);
        end
    end

    function automatic real half_to_real(input logic [15:0] value);
        integer exponent;
        real significand, result;
        begin
            exponent = value[14:10];
            if (exponent == 0) significand = value[9:0] / 1024.0;
            else significand = 1.0 + value[9:0] / 1024.0;
            if (exponent == 0) result = significand * (2.0 ** -14);
            else if (exponent == 31) result = 0.0;
            else result = significand * (2.0 ** (exponent - 15));
            half_to_real = value[15] ? -result : result;
        end
    endfunction

    // weight-load responder (identical protocol to the full-run tb)
    initial begin : wload_server
        integer beats, n_sz, k_sz;
        forever begin
            wait (wload_req === 1'b1);
            s = wload_stage;
            n_sz = STG_N[s];
            k_sz = STG_K[s];
            beats = n_sz*k_sz/32;
            for (i = 0; i < beats; i++) begin
                @(negedge clk);
                wt_wr_en = 1'b1;
                wt_wr_beat = 16'(i);
                for (j = 0; j < 32; j++)
                    wt_wr_data[j*8 +: 8] = w_mem[s][i*32 + j];
            end
            @(negedge clk);
            wt_wr_en = 1'b0;
            for (i = 0; i < n_sz; i++) begin
                @(negedge clk);
                sb_wr_en = 1'b1;
                sb_wr_sel = 1'b0;
                sb_wr_addr = 16'(i);
                sb_wr_data = ws_mem[s][i];
            end
            for (i = 0; i < n_sz; i++) begin
                @(negedge clk);
                sb_wr_en = 1'b1;
                sb_wr_sel = 1'b1;
                sb_wr_addr = 16'(i);
                sb_wr_data = b_mem[s][i];
            end
            @(negedge clk);
            sb_wr_en = 1'b0;
            @(negedge clk);
            wload_ack = 1'b1;
            @(negedge clk);
            wload_ack = 1'b0;
            wait (wload_req === 1'b0);
        end
    end

    // checkpoint dump: literal paths only (see header note)
    task automatic dump_checkpoints;
        begin
            $writememh("build/supra2_block7_top/ckpt/x_buf.hex",    dut.x_buf);
            $writememh("build/supra2_block7_top/ckpt/ln_buf.hex",   dut.ln_buf);
            $writememh("build/supra2_block7_top/ckpt/q_buf.hex",    dut.q_probe_buf);
            $writememh("build/supra2_block7_top/ckpt/k_buf.hex",    dut.k_probe_buf);
            $writememh("build/supra2_block7_top/ckpt/v_buf.hex",    dut.v_buf);
            $writememh("build/supra2_block7_top/ckpt/cq_buf.hex",   dut.cq_probe_buf);
            $writememh("build/supra2_block7_top/ckpt/ck_buf.hex",   dut.ck_probe_buf);
            $writememh("build/supra2_block7_top/ckpt/cv_buf.hex",   dut.cv_buf);
            $writememh("build/supra2_block7_top/ckpt/attn_buf.hex", dut.attn_buf);
            $writememh("build/supra2_block7_top/ckpt/proj_buf.hex", dut.proj_buf);
            $writememh("build/supra2_block7_top/ckpt/mlp_buf.hex",  dut.mlp_buf);
        end
    endtask

    // checkpoint + fixture loads: literal paths only (see header note)
    task automatic load_state;
        begin
            // running state (chain from the previous segment; segment 0's
            // ckpt/x_buf.hex is seeded from the archive fixture by the driver)
            $readmemh("build/supra2_block7_top/ckpt/x_buf.hex",    dut.x_buf);
            $readmemh("build/supra2_block7_top/ckpt/ln_buf.hex",   dut.ln_buf);
            $readmemh("build/supra2_block7_top/ckpt/q_buf.hex",    dut.q_probe_buf);
            $readmemh("build/supra2_block7_top/ckpt/k_buf.hex",    dut.k_probe_buf);
            $readmemh("build/supra2_block7_top/ckpt/v_buf.hex",    dut.v_buf);
            $readmemh("build/supra2_block7_top/ckpt/cq_buf.hex",   dut.cq_probe_buf);
            $readmemh("build/supra2_block7_top/ckpt/ck_buf.hex",   dut.ck_probe_buf);
            $readmemh("build/supra2_block7_top/ckpt/cv_buf.hex",   dut.cv_buf);
            $readmemh("build/supra2_block7_top/ckpt/attn_buf.hex", dut.attn_buf);
            $readmemh("build/supra2_block7_top/ckpt/proj_buf.hex", dut.proj_buf);
            $readmemh("build/supra2_block7_top/ckpt/mlp_buf.hex",  dut.mlp_buf);
            for (i = 0; i < XN; i++) begin
                dut.q_buf[i/32][(i%32)*16 +: 16] = dut.q_probe_buf[i];
                dut.k_buf[i/32][(i%32)*16 +: 16] = dut.k_probe_buf[i];
                dut.cq_buf[i/32][(i%32)*16 +: 16] = dut.cq_probe_buf[i];
            end
            for (i = 0; i < HEADS*CKEYS*HD; i++)
                dut.ck_buf[i/32][(i%32)*16 +: 16] = dut.ck_probe_buf[i];
            // prompt context (constant across segments)
            $readmemh("build/supra2_block7_top/ctx_packed.hex", dut.ctx_buf,
                      0, CTXN-1);
            // AdaLN parameters (constant), straight into param_buf slices
            $readmemh("build/supra2_block7_top/ln0_scale.hex", dut.param_buf,
                      0, 575);
            $readmemh("build/supra2_block7_top/ln0_shift.hex", dut.param_buf,
                      576, 1151);
            $readmemh("build/supra2_block7_top/ln1_scale.hex", dut.param_buf,
                      1152, 1727);
            $readmemh("build/supra2_block7_top/ln1_shift.hex", dut.param_buf,
                      1728, 2303);
            $readmemh("build/supra2_block7_top/ln2_scale.hex", dut.param_buf,
                      2304, 2879);
            $readmemh("build/supra2_block7_top/ln2_shift.hex", dut.param_buf,
                      2880, 3455);
            $readmemh("build/supra2_block7_top/gate0.hex", dut.param_buf,
                      3456, 4031);
            $readmemh("build/supra2_block7_top/gate1.hex", dut.param_buf,
                      4032, 4607);
            // Restore the live RAMs after every checkpoint mirror is loaded.
            dut.restore_probe_memories(start_stage[4:0]);
        end
    endtask

`define B7_LOAD_W(idx, wpath, spath, bpath) \
    $readmemh(wpath, wtmp, 0, STG_N[idx]*STG_K[idx]-1); \
    for (i = 0; i < STG_N[idx]*STG_K[idx]; i++) w_mem[idx][i] = wtmp[i]; \
    $readmemh(spath, stmp, 0, STG_N[idx]-1); \
    for (i = 0; i < STG_N[idx]; i++) ws_mem[idx][i] = stmp[i]; \
    $readmemh(bpath, stmp, 0, STG_N[idx]-1); \
    for (i = 0; i < STG_N[idx]; i++) b_mem[idx][i] = stmp[i];

    initial begin
        if (!$value$plusargs("START_STAGE=%d", start_stage)) start_stage = 0;
        if (!$value$plusargs("STOP_STAGE=%d", stop_stage))   stop_stage = 16;
        if (!$value$plusargs("START_HEAD=%d", start_head))   start_head = 0;
        if (!$value$plusargs("START_NI=%d", start_ni))       start_ni = 0;
        if (!$value$plusargs("STOP_NI=%d", stop_ni))         stop_ni = 0;
        if (!$value$plusargs("SLICE_HEAD=%d", slice_head))   slice_head = 0;

        // golden (used by the final segment's compare)
        $readmemh("build/supra2_block7_top/golden_y.hex", golden_mem, 0, XN-1);
        // weights (literal paths, same as the full-run tb)
        `B7_LOAD_W(0, "build/supra2_block7_top/w_qkv_int8.hex",
                      "build/supra2_block7_top/ws_qkv_fp32.hex",
                      "build/supra2_block7_top/b_qkv_fp32.hex")
        `B7_LOAD_W(1, "build/supra2_block7_top/w_sproj_int8.hex",
                      "build/supra2_block7_top/ws_sproj_fp32.hex",
                      "build/supra2_block7_top/b_sproj_fp32.hex")
        `B7_LOAD_W(2, "build/supra2_block7_top/w_cq_int8.hex",
                      "build/supra2_block7_top/ws_cq_fp32.hex",
                      "build/supra2_block7_top/b_cq_fp32.hex")
        `B7_LOAD_W(3, "build/supra2_block7_top/w_ckv_int8.hex",
                      "build/supra2_block7_top/ws_ckv_fp32.hex",
                      "build/supra2_block7_top/b_ckv_fp32.hex")
        `B7_LOAD_W(4, "build/supra2_block7_top/w_cproj_int8.hex",
                      "build/supra2_block7_top/ws_cproj_fp32.hex",
                      "build/supra2_block7_top/b_cproj_fp32.hex")
        `B7_LOAD_W(5, "build/supra2_block7_top/w_mlp0_int8.hex",
                      "build/supra2_block7_top/ws_mlp0_fp32.hex",
                      "build/supra2_block7_top/b_mlp0_fp32.hex")
        `B7_LOAD_W(6, "build/supra2_block7_top/w_mlp2_int8.hex",
                      "build/supra2_block7_top/ws_mlp2_fp32.hex",
                      "build/supra2_block7_top/b_mlp2_fp32.hex")

        if ($isunknown(golden_mem[0]) || $isunknown(golden_mem[XN-1]) ||
            $isunknown(w_mem[0][0]) ||
            $isunknown(w_mem[6][STG_N[6]*STG_K[6]-1]) ||
            $isunknown(ws_mem[6][STG_N[6]-1]) ||
            $isunknown(b_mem[6][STG_N[6]-1]))
            $fatal(1, "BLOCK_TOP_SEG fixture load failed: weight/golden X/Z");

        total_cycles = 0;
        stage_mark = 0;
        prev_stage = 0;
        for (i = 0; i < 17; i++) stage_cycles[i] = 0;

        // DUT state: checkpoint chain + constant fixtures (time-0, no DMA)
        load_state();
        if ($isunknown(dut.x_buf[0]) || $isunknown(dut.x_buf[XN-1]) ||
            $isunknown(dut.ctx_buf[0]) || $isunknown(dut.ctx_buf[CTXN-1]) ||
            $isunknown(dut.param_buf[0]) || $isunknown(dut.param_buf[PN-1]))
            $fatal(1, "BLOCK_TOP_SEG state load failed: x/ctx/param has X/Z");

        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(negedge clk);

        $display("SUPRA2_SEG_START stage=%0d stop=%0d head=%0d ni=%0d ni1=%0d slice=%0d",
                 start_stage, stop_stage, start_head, start_ni, stop_ni, slice_head);

        start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        fork : seg_watch
            begin
                if (slice_head != 0) begin
                    wait (stage_id == start_stage_r);
                    @(negedge clk);
                    wait ((stage_id != start_stage_r) ||
                          (dut.cur_head != start_head_r));
                end else begin
                    wait (stage_id == stop_stage_r);
                end
            end
            begin
                repeat (4_000_000) @(posedge clk);
                $fatal(1, "BLOCK_TOP_SEG segment timeout: stage=%0d", stage_id);
            end
        join_any
        disable seg_watch;
        @(negedge clk);   // let the boundary writes settle

        stage_cycles[prev_stage] = total_cycles - stage_mark;
        dump_checkpoints();
        $display("SUPRA2_SEG_CYCLES %0d", total_cycles);
        for (i = 0; i < 16; i++)
            if (stage_cycles[i] != 0)
                $display("SUPRA2_SEG_STAGE_CYCLES id=%0d cycles=%0d",
                         i, stage_cycles[i]);

        if (stop_stage == 16 && slice_head == 0) begin
            // final segment (not a per-head slice): compare x_buf against
            // archive golden b7.y. Head-sliced segments end mid-pipeline with
            // intermediate data in x_buf, so the golden gate must not fire.
            begin
                real got, exp_v, abs_err, max_abs, peak;
                real sum_sq, ref_sq, rmse, rel_l2, rel_peak;
                integer bit_diff;
                integer x_words;
                x_words = 0;
                for (i = 0; i < XN; i++)
                    if ($isunknown(dut.x_buf[i]) || $isunknown(golden_mem[i]))
                        x_words++;
                $display("SUPRA2_BLOCK_TOP_X_WORDS %0d", x_words);
                if (x_words > 0)
                    $fatal(1, "BLOCK_TOP_SEG output/golden has %0d X/Z words", x_words);
                max_abs = 0.0; peak = 0.0; sum_sq = 0.0; ref_sq = 0.0;
                bit_diff = 0;
                for (i = 0; i < XN; i++) begin
                    got = half_to_real(dut.x_buf[i]);
                    exp_v = half_to_real(golden_mem[i]);
                    abs_err = (got >= exp_v) ? got - exp_v : exp_v - got;
                    if (abs_err > max_abs) max_abs = abs_err;
                    if ((exp_v >= 0 ? exp_v : -exp_v) > peak)
                        peak = (exp_v >= 0) ? exp_v : -exp_v;
                    sum_sq = sum_sq + abs_err*abs_err;
                    ref_sq = ref_sq + exp_v*exp_v;
                    if (dut.x_buf[i] !== golden_mem[i]) bit_diff++;
                end
                rmse = $sqrt(sum_sq / XN);
                rel_l2 = $sqrt(sum_sq / ((ref_sq > 1.0e-20) ? ref_sq : 1.0));
                rel_peak = max_abs / ((peak > 1.0e-20) ? peak : 1.0);
                $display("SUPRA2_BLOCK_TOP_MAX_ABS %.9g", max_abs);
                $display("SUPRA2_BLOCK_TOP_REL_TO_PEAK %.9g", rel_peak);
                $display("SUPRA2_BLOCK_TOP_RMSE %.9g", rmse);
                $display("SUPRA2_BLOCK_TOP_REL_L2 %.9g", rel_l2);
                $display("SUPRA2_BLOCK_TOP_BIT_DIFF %0d", bit_diff);
                if (rel_peak > 0.02)
                    $fatal(1, "BLOCK_TOP_SEG output exceeds 2%% rel-to-peak gate: %e",
                           rel_peak);
                $display("SUPRA2_BLOCK_TOP_PASS block=7 segmented=1 cycles_sum_note=see_seg_logs");
            end
        end
        $display("SUPRA2_SEG_PASS stage=%0d stop=%0d head=%0d ni=%0d ni1=%0d cycles=%0d",
                 start_stage, stop_stage, start_head, start_ni, stop_ni, total_cycles);
        $finish;
    end
endmodule

`default_nettype wire
