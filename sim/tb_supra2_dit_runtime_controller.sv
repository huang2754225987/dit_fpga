`timescale 1ns/1ps
`default_nettype none

module tb_supra2_dit_runtime_controller;
    import supra2_dit_ddr_layout::*;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic start = 1'b0;
    logic busy, done;
    logic context_start, context_done = 1'b0;
    logic step_prep_start, step_prep_done = 1'b0;
    logic block_prepare_start, block_prepare_done = 1'b0;
    logic block_start, block_done = 1'b0;
    logic branch_index;
    logic final_start, final_done = 1'b0;
    logic cfg_combine_start, cfg_combine_done = 1'b0;
    logic euler_start, euler_done = 1'b0;
    logic dicache_enable = 1'b0;
    logic dicache_decision_valid = 1'b0, dicache_reuse = 1'b0;
    logic dicache_reuse_done = 1'b0;
    logic dicache_probe_start;
    logic [5:0] step_index;
    logic [3:0] block_index;
    logic [2:0] dma_stage_index = 3'd0;
    logic block_is_skipped;
    logic [31:0] timestep_address;
    logic [6:0] stage_layer_id;
    logic [31:0] stage_weight_address, stage_scale_address, stage_bias_address;
    logic [31:0] stage_weight_bytes, stage_scale_bytes, stage_bias_bytes;
    logic [31:0] stage_weight_beats, stage_scale_beats, stage_bias_beats;
    logic [15:0] stage_output_features, stage_input_features;
    logic [15:0] stage_n_tiles, stage_k_tiles;
    logic [31:0] stage_n_tile_stride_bytes, stage_k_tile_stride_bytes;
    logic [31:0] block_adaln_weight_address, block_adaln_bias_address;

    integer context_count = 0;
    integer prep_count = 0;
    integer block_prepare_count = 0;
    integer block_count = 0;
    integer final_count = 0;
    integer cfg_count = 0;
    integer euler_count = 0;
    integer cycles = 0;
    integer last_step = -1;
    integer last_block = -1;
    integer last_prep_step = -1;
    integer last_branch = -1;

    always #5 clk = ~clk;

    supra2_dit_runtime_controller dut (.*);

    // One-cycle-later completion models independent engines with a short
    // latency while preserving the start/done handshake contract.
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            context_done <= 1'b0;
            step_prep_done <= 1'b0;
            block_prepare_done <= 1'b0;
            block_done <= 1'b0;
            final_done <= 1'b0;
            cfg_combine_done <= 1'b0;
            euler_done <= 1'b0;
        end else begin
            context_done <= context_start;
            step_prep_done <= step_prep_start;
            block_prepare_done <= block_prepare_start;
            block_done <= block_start;
            final_done <= final_start;
            cfg_combine_done <= cfg_combine_start;
            euler_done <= euler_start;
        end
    end

    // Verify each emitted block and all seven direct DMA descriptors while
    // the controller holds the block index stable in its wait state.
    initial begin : descriptor_check
        forever begin
            @(posedge clk);
            if (rst_n && block_start) begin
                #1;
                if (block_is_skipped !== 1'b0)
                    $fatal(1, "controller started a skipped block step=%0d block=%0d",
                           step_index, block_index);
                if (timestep_address !== DDR_BASE + T_SCHEDULE_OFFSET + step_index * 4)
                    $fatal(1, "bad timestep address at step %0d", step_index);
                if (block_adaln_weight_address !==
                    DDR_BASE + BLOCK_ADALN_WEIGHT_OFFSET[block_index] ||
                    block_adaln_bias_address !==
                    DDR_BASE + BLOCK_ADALN_BIAS_OFFSET[block_index])
                    $fatal(1, "bad AdaLN descriptor block=%0d", block_index);
                for (int stage = 0; stage < BLOCK_STAGE_COUNT; stage++) begin
                    dma_stage_index = stage[2:0];
                    #1;
                    if (stage_layer_id !== BLOCK_STAGE_LAYER_ID[block_index][stage] ||
                        stage_weight_address !== DDR_BASE +
                            BLOCK_STAGE_WEIGHT_OFFSET[block_index][stage] ||
                        stage_scale_address !== DDR_BASE +
                            BLOCK_STAGE_SCALE_OFFSET[block_index][stage] ||
                        stage_bias_address !== DDR_BASE +
                            BLOCK_STAGE_BIAS_OFFSET[block_index][stage] ||
                        stage_weight_bytes !== BLOCK_STAGE_WEIGHT_BYTES[block_index][stage] ||
                        stage_scale_bytes !== BLOCK_STAGE_SCALE_BYTES[block_index][stage] ||
                        stage_bias_bytes !== BLOCK_STAGE_BIAS_BYTES[block_index][stage] ||
                        stage_weight_beats !== BLOCK_STAGE_WEIGHT_BEATS[block_index][stage] ||
                        stage_scale_beats !== BLOCK_STAGE_SCALE_BEATS[block_index][stage] ||
                        stage_bias_beats !== BLOCK_STAGE_BIAS_BEATS[block_index][stage] ||
                        stage_output_features !== BLOCK_STAGE_OUT[block_index][stage] ||
                        stage_input_features !== BLOCK_STAGE_IN[block_index][stage] ||
                        stage_n_tiles !== BLOCK_STAGE_N_TILES[block_index][stage] ||
                        stage_k_tiles !== BLOCK_STAGE_K_TILES[block_index][stage] ||
                        stage_n_tile_stride_bytes !==
                            BLOCK_STAGE_N_TILE_STRIDE_BYTES[block_index][stage] ||
                        stage_k_tile_stride_bytes !==
                            BLOCK_STAGE_K_TILE_STRIDE_BYTES[block_index][stage])
                        $fatal(1, "bad DMA descriptor step=%0d block=%0d stage=%0d",
                               step_index, block_index, stage);
                end
            end
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            context_count = 0;
            prep_count = 0;
            block_prepare_count = 0;
            block_count = 0;
            final_count = 0;
            cfg_count = 0;
            euler_count = 0;
            cycles = 0;
            last_step = -1;
            last_block = -1;
            last_prep_step = -1;
            last_branch = -1;
        end else begin
            cycles = cycles + 1;
            if (cycles > 10000) $fatal(1, "runtime controller watchdog");
            if (context_start) context_count = context_count + 1;
            if (step_prep_start) prep_count = prep_count + 1;
            if (step_prep_start) begin
                if (step_index != (prep_count-1)/2 ||
                    branch_index != ((prep_count-1)%2))
                    $fatal(1, "branch prep sequence mismatch prep=%0d step=%0d branch=%0d",
                           prep_count,step_index,branch_index);
                last_prep_step = step_index;
            end
            if (block_prepare_start) block_prepare_count = block_prepare_count + 1;
            if (final_start) begin
                if (step_index != final_count/2 || branch_index != final_count%2)
                    $fatal(1, "final branch order mismatch index=%0d step=%0d branch=%0d",
                           final_count,step_index,branch_index);
                final_count = final_count + 1;
            end
            if (cfg_combine_start) cfg_count = cfg_count + 1;
            if (euler_start) euler_count = euler_count + 1;
            if (block_start) begin
                block_count = block_count + 1;
                if (step_index != last_step) begin
                    for (int skipped_step = last_step + 1;
                         skipped_step < step_index; skipped_step++) begin
                        if (BLOCK_SKIP_MASK[skipped_step] != 14'h3fff)
                            $fatal(1, "step sequence skipped active step %0d",
                                   skipped_step);
                    end
                    if (branch_index != 0)
                        $fatal(1, "new step must begin on branch 0, step=%0d branch=%0d",
                               step_index,branch_index);
                    last_step = step_index;
                    last_branch = 0;
                    last_block = -1;
                end else if (branch_index != last_branch) begin
                    if (last_branch != 0 || branch_index != 1)
                        $fatal(1, "branch sequence mismatch step=%0d branch=%0d",
                               step_index,branch_index);
                    last_branch = branch_index;
                    last_block = -1;
                end
                if (last_block >= 0 && block_index <= last_block)
                    $fatal(1, "block sequence out of order at step=%0d block=%0d",
                           step_index, block_index);
                last_block = block_index;
            end
            if (done) begin
                if (busy || context_count != 1 || prep_count != 2*FLOW_STEPS ||
                    block_prepare_count != 828 || block_count != 828 ||
                    final_count != 2*FLOW_STEPS ||
                    cfg_count != FLOW_STEPS || euler_count != FLOW_STEPS)
                    $fatal(1, "bad totals ctx=%0d prep=%0d prepare=%0d blocks=%0d final=%0d cfg=%0d euler=%0d",
                           context_count, prep_count, block_prepare_count,
                           block_count, final_count,
                           cfg_count, euler_count);
                $display("SUPRA2_DIT_RUNTIME_CONTROLLER_PASS steps=%0d branches=2 executed_blocks=%0d cycles=%0d",
                         FLOW_STEPS, block_count, cycles);
                $finish;
            end
        end
    end

    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk) rst_n = 1'b1;
        @(negedge clk) start = 1'b1;
        @(negedge clk) start = 1'b0;
    end
endmodule

`default_nettype wire
