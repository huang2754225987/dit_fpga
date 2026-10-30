// =============================================================================
// tb_topk_unit.sv - Standalone unit test for the MoE router TopK + stable
// softmax (topk_unit). Fills 60 FP32 logits into an acc-buffer model, triggers
// the unit, and prints the emitted top-4 indices + FP16 weights for each test
// case. A companion Python script compares against the PyTorch reference
// (softmax over ALL 60 logits, norm_topk_prob=false, ties -> smaller index).
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_topk_unit;

    parameter int N_EXPERTS = 60;
    parameter int TOP_K     = 4;

    logic clk, rst_n, start, done;
    logic cache_clear = 0, route_select = 0, route_error;
    logic route_group_select = 0, route_group_done;
    logic [5:0] route_group_expert = 0, route_group_tokens = 32;
    logic [4:0] route_group_start = 0;
    logic [5:0] route_group_count;
    logic [4:0] route_group_token [0:31];
    logic [15:0] route_group_weight [0:31];
    logic [4:0] route_token = 0;
    logic [9:0]  acc_rd_addr;
    logic        acc_rd_en, acc_rd_valid, acc_rd_set;
    logic [31:0] acc_rd_data;
    logic [4:0]  token_id;
    logic [9:0]  acc_base_addr;
    logic [5:0]  topk_idx  [0:TOP_K-1];
    logic [15:0] topk_weight [0:TOP_K-1];

    // acc buffer model (1-cycle read latency, 2 ping-pong sets x 1024x32).
    // topk_unit reads logits 0..31 from set 0 (acc_rd_set=0) and 32..59 from
    // set 1 (acc_rd_set=1), each at column (i mod 32).
    logic [31:0] acc_mem0 [0:1023];   // tile0 set: experts 0..31 at [0..31]
    logic [31:0] acc_mem1 [0:1023];   // tile1 set: experts 32..59 at [0..27]
    logic [9:0]  req_addr;
    logic        req_set;
    logic        pending;

    // ---------------------------------------------------------------
    // Unit under test
    // ---------------------------------------------------------------
    topk_unit #(
        .N_EXPERTS (N_EXPERTS),
        .TOP_K     (TOP_K),
        .ELEM_W    (16),
        .ACC_DATA_W(32),
        .ACC_ADDR_W(10),
        .N_TOKENS  (32),
        .TOKEN_ID_W(5)
    ) u_dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (start),
        .acc_base_addr (acc_base_addr),
        .token_id      (token_id),
        .cache_clear(cache_clear), .route_select(route_select),
        .route_token(route_token), .route_error(route_error),
        .route_group_select(route_group_select),
        .route_group_expert(route_group_expert),
        .route_group_tokens(route_group_tokens),
        .route_group_start(route_group_start),
        .route_group_done(route_group_done),
        .route_group_count(route_group_count),
        .route_group_token(route_group_token),
        .route_group_weight(route_group_weight),
        .acc_rd_en     (acc_rd_en),
        .acc_rd_addr   (acc_rd_addr),
        .acc_rd_set    (acc_rd_set),
        .acc_rd_data   (acc_rd_data),
        .acc_rd_valid  (acc_rd_valid),
        .topk_idx      (topk_idx),
        .topk_weight   (topk_weight),
        .done          (done)
    );

    // ---------------------------------------------------------------
    // acc buffer read model
    // ---------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            req_addr <= '0;
            pending  <= 1'b0;
            acc_rd_valid <= 1'b0;
            acc_rd_data  <= '0;
        end else begin
            acc_rd_valid <= 1'b0;
            if (acc_rd_en) begin
                req_addr <= acc_rd_addr;
                req_set  <= acc_rd_set;
                pending  <= 1'b1;
            end
            if (pending) begin
                acc_rd_valid <= 1'b1;
                acc_rd_data  <= req_set ? acc_mem1[req_addr]
                                        : acc_mem0[req_addr];
                pending      <= 1'b0;
            end
        end
    end

    // Write logit i (0..59) into the right set at column (i mod 32),
    // mirroring the two-N-tile STORE layout in the real acc_buffer.
    task automatic set_logit(input integer i, input logic [31:0] v);
        begin
            if (i < 32) acc_mem0[i]      = v;
            else        acc_mem1[i - 32] = v;
        end
    endtask

    // 带基地址版本：写入 base + (i mod 32)，用于 token_id/base 锁存测试
    task automatic set_logit_at(input integer base, input integer i,
                                input logic [31:0] v);
        begin
            if (i < 32) acc_mem0[base + i]      = v;
            else        acc_mem1[base + i - 32] = v;
        end
    endtask

    // ---------------------------------------------------------------
    // Helpers (no unpacked-array task ports -> vopt-safe)
    // ---------------------------------------------------------------
    task automatic fill_and_run(input string name);
        begin
            start = 1'b1;
            @(posedge clk);
            start = 1'b0;
            wait (done);
            $display("RESULT %0s %0d %0d %0d %0d %04x %04x %04x %04x",
                     name,
                     topk_idx[0], topk_idx[1], topk_idx[2], topk_idx[3],
                     topk_weight[0], topk_weight[1], topk_weight[2], topk_weight[3]);
            $display("  DBG r_sum=%04x r_recip=%04x best_e=%04x %04x %04x %04x rmax=%04x",
                     u_dut.r_sum, u_dut.r_recip,
                     u_dut.best_e[0], u_dut.best_e[1],
                     u_dut.best_e[2], u_dut.best_e[3],
                     u_dut.r_max);
            @(posedge clk);
        end
    endtask

    // 模拟 top.sv 真实时序：token_id/acc_base_addr 是 ucode_decode 的组合
    // 脉冲，仅与 start 同拍有效，下一拍即归 0。若 DUT 未锁存，LOAD 会读到
    // base=0 的错误数据、WT 会写 route[0] 错误槽位。
    task automatic run_latch(input string name, input logic [4:0] tok,
                             input logic [9:0] base);
        begin
            token_id      = tok;
            acc_base_addr = base;
            start = 1'b1;
            @(posedge clk);
            start = 1'b0;
            token_id      = 5'd0;    // 脉冲结束，输入归 0（同 top.sv）
            acc_base_addr = 10'd0;
            wait (done);
            $display("RESULT %0s %0d %0d %0d %0d %04x %04x %04x %04x",
                     name,
                     topk_idx[0], topk_idx[1], topk_idx[2], topk_idx[3],
                     topk_weight[0], topk_weight[1], topk_weight[2], topk_weight[3]);
            @(posedge clk);
        end
    endtask

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    integer i;
    // 锁存测试用：参考输出与错误计数
    integer errors;
    logic [5:0]  ref_idx [0:TOP_K-1];
    logic [15:0] ref_w   [0:TOP_K-1];

    initial begin
        rst_n = 1'b0;
        start = 1'b0;
        token_id = 5'd0;
        acc_base_addr = 10'd0;
        repeat(4) @(posedge clk);
        rst_n = 1'b1;
        repeat(2) @(posedge clk);

        // Case 1: all-negative distinct (max = least negative)
        for (i = 0; i < 60; i++)
            set_logit(i, $shortrealtobits(-1.0 - i * 0.5));
        fill_and_run("allneg");

        // Case 2: all equal -> top4 = 0,1,2,3, equal weight ~0.25
        for (i = 0; i < 60; i++)
            set_logit(i, $shortrealtobits(0.0));
        fill_and_run("equal");

        // Case 3: extreme spread (one huge max, others very negative)
        for (i = 0; i < 60; i++)
            set_logit(i, $shortrealtobits(-100.0));
        set_logit(7,  $shortrealtobits(100.0));
        set_logit(20, $shortrealtobits(50.0));
        fill_and_run("extreme");

        // Case 4: random normal logits
        for (i = 0; i < 60; i++)
            set_logit(i, $shortrealtobits(($urandom % 2000) / 100.0 - 10.0));
        fill_and_run("random");

        // Case 5: real router logits (token0) from file
        begin
            integer fh;
            logic [31:0] w;
            fh = $fopen("router_logits_golden.bin", "rb");
            if (fh == 0) begin
                $display("WARNING: no router_logits_golden.bin; skipping real case");
            end else begin
                for (i = 0; i < 60; i++) begin
                    logic [7:0] b [0:3];
                    void'($fread(b, fh));
                    w = {b[3], b[2], b[1], b[0]};
                    set_logit(i, w);
                end
                $fclose(fh);
                fill_and_run("real");
            end
        end

        // Case 6: 锁存参考——特征模式 P2 放在 base=0 / token=0，记录参考输出。
        // 模式跨两个 tile（最大值在 tile1 的 40/33 号专家），可同时暴露
        // base 地址错与 token 槽位错。
        for (i = 0; i < 60; i++)
            set_logit(i, $shortrealtobits(-1.0 - i * 0.1));
        set_logit(40, $shortrealtobits(5.0));
        set_logit(33, $shortrealtobits(4.0));
        set_logit(5,  $shortrealtobits(3.0));
        set_logit(59, $shortrealtobits(2.0));
        fill_and_run("latch_ref");
        for (i = 0; i < TOP_K; i++) begin
            ref_idx[i] = topk_idx[i];
            ref_w[i]   = topk_weight[i];
        end

        // Case 7: token_id=5 / acc_base_addr=64 锁存测试（Bug1 回归）。
        // 同一模式 P2 搬到 base 64；base 0 改写为全 -50 的干扰数据，随后
        // start 脉冲后 token_id/acc_base_addr 立即归 0（模拟 top.sv 组合
        // 脉冲）。若 DUT 未锁存：读地址错（拿到 -50 干扰值，输出≠参考），
        // 且 route[0] 被覆写、route[5] 保持复位值 0。
        for (i = 0; i < 60; i++)
            set_logit_at(64, i, $shortrealtobits(-1.0 - i * 0.1));
        set_logit_at(64, 40, $shortrealtobits(5.0));
        set_logit_at(64, 33, $shortrealtobits(4.0));
        set_logit_at(64, 5,  $shortrealtobits(3.0));
        set_logit_at(64, 59, $shortrealtobits(2.0));
        for (i = 0; i < 32; i++) acc_mem0[i] = $shortrealtobits(-50.0 - i);
        for (i = 0; i < 28; i++) acc_mem1[i] = $shortrealtobits(-50.0 - i);
        run_latch("latch_t5", 5'd5, 10'd64);

        errors = 0;
        for (i = 0; i < TOP_K; i++) begin
            if (topk_idx[i] !== ref_idx[i] || topk_weight[i] !== ref_w[i]) begin
                errors = errors + 1;
                $display("FAIL latch_t5 输出 k=%0d: idx=%0d(期望%0d) w=%04x(期望%04x)",
                         i, topk_idx[i], ref_idx[i], topk_weight[i], ref_w[i]);
            end
            if (u_dut.route_e[5][i] !== ref_idx[i] ||
                u_dut.route_w[5][i] !== ref_w[i]) begin
                errors = errors + 1;
                $display("FAIL latch_t5 缓存槽5 k=%0d: e=%0d(期望%0d) w=%04x(期望%04x)",
                         i, u_dut.route_e[5][i], ref_idx[i],
                         u_dut.route_w[5][i], ref_w[i]);
            end
            if (u_dut.route_e[0][i] !== ref_idx[i] ||
                u_dut.route_w[0][i] !== ref_w[i]) begin
                errors = errors + 1;
                $display("FAIL latch_t5 槽0被覆写 k=%0d: e=%0d(期望%0d) w=%04x(期望%04x)",
                         i, u_dut.route_e[0][i], ref_idx[i],
                         u_dut.route_w[0][i], ref_w[i]);
            end
        end
        if (errors == 0)
            $display("PASS latch_t5: token_id=5 base=64 输出与参考一致（idx=%0d,%0d,%0d,%0d），route[5] 正确、route[0] 未被覆写",
                     ref_idx[0], ref_idx[1], ref_idx[2], ref_idx[3]);
        else
            $display("FAIL latch_t5: %0d 项不匹配", errors);

        if (errors) $fatal(1, "latch test failed");
        // Overwrite the current outputs with a distinct token, then restore
        // token 5 through the public command port, not a hierarchical force.
        for (i = 0; i < 60; i++) set_logit(i, $shortrealtobits(0.0));
        fill_and_run("restore_interference");
        @(negedge clk); route_token = 5; route_select = 1;
        @(negedge clk); route_select = 0;
        if (!done || route_error) $fatal(1, "route restore did not complete");
        for (i = 0; i < TOP_K; i++)
            if (topk_idx[i] !== ref_idx[i] || topk_weight[i] !== ref_w[i])
                $fatal(1, "route restore mismatch at slot %0d", i);
        @(negedge clk); route_token = 31; route_select = 1;
        @(negedge clk); route_select = 0;
        if (!route_error || done) $fatal(1, "missing route was accepted");

        // A group scan over a partially populated cache must fail closed and
        // must not publish a usable partial count.
        @(negedge clk); route_group_expert = 0; route_group_tokens = 32;
        route_group_select = 1;
        @(negedge clk); route_group_select = 0;
        wait (route_error || route_group_done);
        if (!route_error || route_group_done || route_group_count != 0)
            $fatal(1, "partial route group was published");

        // Fill all 32 entries with tied logits (stable Top-K => experts 0..3),
        // then verify the expert-0 bucket returns every token exactly once.
        for (i = 0; i < 60; i++) set_logit(i, $shortrealtobits(0.0));
        for (i = 0; i < 32; i++) run_latch("group_seed", 5'(i), 10'd0);
        @(negedge clk); route_group_expert = 0; route_group_tokens = 32;
        route_group_select = 1;
        @(negedge clk); route_group_select = 0;
        wait (route_group_done);
        if (route_error || route_group_count != 32)
            $fatal(1, "expert bucket count mismatch err=%b count=%0d",
                   route_error, route_group_count);
        for (i = 0; i < 32; i++) begin
            if (route_group_token[i] != i ||
                route_group_weight[i] !== u_dut.route_w[i][0])
                $fatal(1, "expert bucket row%0d token=%0d weight=%04x expected=%04x",
                       i, route_group_token[i], route_group_weight[i], u_dut.route_w[i][0]);
        end
        @(negedge clk); route_group_start = 8; route_group_tokens = 8;
        route_group_select = 1;
        @(negedge clk); route_group_select = 0;
        wait (route_group_done);
        if (route_error || route_group_count != 8)
            $fatal(1, "offset route group count mismatch err=%b count=%0d",
                   route_error, route_group_count);
        for (i = 0; i < 8; i++)
            if (route_group_token[i] != i + 8)
                $fatal(1, "offset group row%0d maps to token %0d", i,
                       route_group_token[i]);
        @(negedge clk); route_group_start = 28; route_group_tokens = 8;
        route_group_select = 1;
        @(negedge clk); route_group_select = 0;
        wait (route_error || route_group_done);
        if (!route_error || route_group_done)
            $fatal(1, "out-of-range route-group token span was accepted");
        $display("PASS route group: full and offset token ranges map in stable order");
        @(negedge clk); cache_clear = 1;
        @(negedge clk); cache_clear = 0; route_token = 5; route_select = 1;
        @(negedge clk); route_select = 0;
        if (!route_error || done) $fatal(1, "old-run route survived cache clear");
        $display("PASS route restore: cached IDs/weights, missing token, new-run invalidation");
        $display("=== TB done ===");
        $finish;
    end

endmodule

`default_nettype wire
