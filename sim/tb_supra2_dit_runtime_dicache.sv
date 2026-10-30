`timescale 1ns/1ps
`default_nettype none

module tb_supra2_dit_runtime_dicache;
    import supra2_dit_ddr_layout::*;
    localparam int CACHE_LANES = 4;
    localparam int CACHE_ELEMENTS = 64;
    localparam int CACHE_BEATS = CACHE_ELEMENTS / CACHE_LANES;
    logic clk = 1'b0, rst_n = 1'b0, start = 1'b0;
    logic busy, done;
    logic context_start, context_done = 1'b0;
    logic step_prep_start, step_prep_done = 1'b0;
    logic block_prepare_start, block_prepare_done = 1'b0;
    logic block_start, block_done = 1'b0;
    logic final_start, final_done = 1'b0;
    logic cfg_combine_start, cfg_combine_done = 1'b0;
    logic euler_start, euler_done = 1'b0;
    logic [5:0] step_index;
    logic branch_index;
    logic [3:0] block_index;
    wire block_is_skipped;
    logic dicache_enable = 1'b1;
    logic dicache_probe_start;
    logic [31:0] dicache_probe_error_q16 = 32'd4000;
    logic [31:0] dicache_threshold_q16 = 32'd6554;
    logic [15:0] dicache_gamma_fp16 = 16'h3800;
    wire dicache_cache_refresh_done;
    wire dicache_decision_valid, dicache_reuse, dicache_refresh_residual;
    wire [31:0] dicache_accumulated_error;
    wire dicache_cache_valid;
    wire refresh_ready, cache_out_valid, cache_out_last, cache_reuse_done;
    wire [CACHE_LANES*16-1:0] cache_out_data;
    wire cache_valid, cache_dcta_valid, cache_error;
    logic [CACHE_LANES*16-1:0] refresh_data;
    logic [3:0] refresh_beat = '0;
    integer refresh_count [0:1];
    integer reuse_groups [0:1];
    integer reuse_beat = 0;
    integer blocks = 0, probes = 0, finals = 0, cfgs = 0, eulers = 0;
    integer refreshes = 0, reuse_count = 0, reuse_beats = 0;
    integer cycles = 0;
    integer run_index = 0;

    always #5 clk = ~clk;

    supra2_dicache_online_policy policy (
        .clk(clk), .rst_n(rst_n), .run_start(start), .enable(dicache_enable),
        .branch_index(branch_index), .probe_valid(dicache_probe_start),
        .probe_error_q16(dicache_probe_error_q16),
        .threshold_q16(dicache_threshold_q16),
        .cache_refresh_done(dicache_cache_refresh_done),
        .decision_valid(dicache_decision_valid), .reuse_residual(dicache_reuse),
        .refresh_residual(dicache_refresh_residual),
        .accumulated_error_q16(dicache_accumulated_error),
        .cache_valid(dicache_cache_valid)
    );

    always_comb begin
        refresh_data = {CACHE_LANES{((refresh_count[branch_index] & 1) == 0) ?
                                      16'h3c00 : 16'h4200}};
    end

    supra2_dicache_residual_cache #(
        .ELEMENTS(CACHE_ELEMENTS), .LANES(CACHE_LANES)
    ) residual_cache (
        .clk(clk), .rst_n(rst_n), .run_start(start), .branch_index(branch_index),
        .refresh_start(dicache_refresh_residual),
        .refresh_valid(refresh_ready), .refresh_ready(refresh_ready),
        .refresh_residual_fp16(refresh_data),
        .refresh_last(refresh_beat == CACHE_BEATS-1),
        .refresh_done(dicache_cache_refresh_done),
        .reuse_start(dicache_reuse), .gamma_fp16(dicache_gamma_fp16),
        .reuse_valid(cache_out_valid), .reuse_ready(1'b1),
        .reuse_residual_fp16(cache_out_data), .reuse_last(cache_out_last),
        .reuse_done(cache_reuse_done), .cache_valid(cache_valid),
        .dcta_valid(cache_dcta_valid), .cache_error(cache_error)
    );

    supra2_dit_runtime_controller dut (
        .clk(clk), .rst_n(rst_n), .start(start), .busy(busy), .done(done),
        .context_start(context_start), .context_done(context_done),
        .step_prep_start(step_prep_start), .step_prep_done(step_prep_done),
        .block_prepare_start(block_prepare_start),
        .block_prepare_done(block_prepare_done), .block_start(block_start),
        .block_done(block_done), .final_start(final_start), .final_done(final_done),
        .cfg_combine_start(cfg_combine_start), .cfg_combine_done(cfg_combine_done),
        .euler_start(euler_start), .euler_done(euler_done),
        .dicache_enable(dicache_enable),
        .dicache_decision_valid(dicache_decision_valid),
        .dicache_reuse(dicache_reuse), .dicache_reuse_done(cache_reuse_done),
        .dicache_probe_start(dicache_probe_start),
        .step_index(step_index), .branch_index(branch_index),
        .block_index(block_index), .block_is_skipped(block_is_skipped),
        .dma_stage_index(3'd0), .timestep_address(), .stage_layer_id(),
        .stage_weight_address(), .stage_scale_address(), .stage_bias_address(),
        .stage_weight_bytes(), .stage_scale_bytes(), .stage_bias_bytes(),
        .stage_weight_beats(), .stage_scale_beats(), .stage_bias_beats(),
        .stage_output_features(), .stage_input_features(), .stage_n_tiles(),
        .stage_k_tiles(), .stage_n_tile_stride_bytes(), .stage_k_tile_stride_bytes(),
        .block_adaln_weight_address(), .block_adaln_bias_address()
    );

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            context_done <= 1'b0; step_prep_done <= 1'b0;
            block_prepare_done <= 1'b0; block_done <= 1'b0;
            final_done <= 1'b0; cfg_combine_done <= 1'b0; euler_done <= 1'b0;
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

    always @(posedge clk) begin
        if (!rst_n) begin
            refresh_beat = 0;
            refresh_count[0] = 0; refresh_count[1] = 0;
            reuse_groups[0] = 0; reuse_groups[1] = 0;
            reuse_beat = 0; reuse_count = 0; reuse_beats = 0; refreshes = 0;
        end else begin
            if (refresh_ready) begin
                if (refresh_beat == CACHE_BEATS-1) begin
                    refresh_beat = 0;
                    refresh_count[branch_index] = refresh_count[branch_index] + 1;
                    refreshes = refreshes + 1;
                end else refresh_beat = refresh_beat + 1;
            end
            if (cache_out_valid) begin
                if (cache_out_data !== {CACHE_LANES{
                        (reuse_groups[branch_index] == 0) ? 16'h3c00 : 16'h4000}} ||
                    cache_out_last !== (reuse_beat == CACHE_BEATS-1))
                    $fatal(1, "cached DCTA stream mismatch branch=%0d group=%0d beat=%0d data=%h last=%b",
                           branch_index, reuse_groups[branch_index], reuse_beat,
                           cache_out_data, cache_out_last);
                reuse_beats = reuse_beats + 1;
                if (cache_reuse_done) begin
                    reuse_beat = 0;
                    reuse_groups[branch_index] = reuse_groups[branch_index] + 1;
                    reuse_count = reuse_count + 1;
                end else reuse_beat = reuse_beat + 1;
            end
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            blocks = 0; probes = 0; finals = 0; cfgs = 0; eulers = 0; cycles = 0;
        end else begin
            cycles = cycles + 1;
            if (cycles > 100000) $fatal(1, "DiCache runtime schedule timeout");
            if (block_start) blocks = blocks + 1;
            if (dicache_probe_start) probes = probes + 1;
            if (final_start) finals = finals + 1;
            if (cfg_combine_start) cfgs = cfgs + 1;
            if (euler_start) eulers = eulers + 1;
            if (done) begin
                if (busy) $fatal(1, "DiCache run completed while controller busy");
                if (run_index == 0) begin
                    if (blocks != 750 || probes != 100 || finals != 100 ||
                        cfgs != FLOW_STEPS || eulers != FLOW_STEPS ||
                        refreshes != 50 || reuse_count != 50 || reuse_beats != 800 ||
                        cache_error || reuse_groups[0] != 25 || reuse_groups[1] != 25)
                        $fatal(1, "DiCache enabled totals blocks=%0d probes=%0d final=%0d cfg=%0d euler=%0d refresh=%0d reuse=%0d beats=%0d error=%b",
                               blocks, probes, finals, cfgs, eulers, refreshes,
                               reuse_count, reuse_beats, cache_error);
                    $display("SUPRA2_DIT_RUNTIME_DICACHE_ENABLED_PASS blocks=%0d probes=%0d refresh=%0d reuse=%0d dcta_beats=%0d",
                             blocks, probes, refreshes, reuse_count, reuse_beats);
                    run_index = 1;
                end else begin
                    if (dicache_enable || blocks != 1578 || probes != 100 ||
                        finals != 200 || cfgs != 2*FLOW_STEPS ||
                        eulers != 2*FLOW_STEPS || refreshes != 50 ||
                        reuse_count != 50 || reuse_beats != 800 ||
                        cache_error)
                        $fatal(1, "DiCache disabled totals blocks=%0d probes=%0d final=%0d cfg=%0d euler=%0d refresh=%0d reuse=%0d beats=%0d error=%b enable=%b",
                               blocks, probes, finals, cfgs, eulers, refreshes,
                               reuse_count, reuse_beats, cache_error, dicache_enable);
                    $display("SUPRA2_DIT_RUNTIME_DICACHE_TOGGLE_PASS enabled_blocks=750 disabled_blocks=%0d disabled_probes=%0d",
                             blocks-750, probes-100);
                    $finish;
                end
            end
        end
    end

    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk); rst_n = 1'b1; start = 1'b1;
        @(negedge clk); start = 1'b0;
        @(posedge done);
        @(negedge clk); dicache_enable = 1'b0; start = 1'b1;
        @(negedge clk); start = 1'b0;
    end
endmodule

`default_nettype wire
