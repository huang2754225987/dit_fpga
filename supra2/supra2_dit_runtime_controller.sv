`default_nettype none

// Package-driven outer controller for the 50-step Supra2-IMG CFG schedule.
// Compute and DMA engines attach through start/done handshakes. The stage
// descriptor ports resolve the active block and requested QLinear stage into
// byte addresses and transfer lengths from the generated DDR layout package.
module supra2_dit_runtime_controller
    import supra2_dit_ddr_layout::*;
(
    input  wire logic        clk,
    input  wire logic        rst_n,
    input  wire logic        start,
    output logic             busy,
    output logic             done,

    output logic             context_start,
    input  wire logic        context_done,
    output logic             step_prep_start,
    input  wire logic        step_prep_done,
    output logic             block_prepare_start,
    input  wire logic        block_prepare_done,
    output logic             block_start,
    input  wire logic        block_done,
    output logic             final_start,
    input  wire logic        final_done,
    output logic             cfg_combine_start,
    input  wire logic        cfg_combine_done,
    output logic             euler_start,
    input  wire logic        euler_done,

    input  wire logic        dicache_enable,
    input  wire logic        dicache_decision_valid,
    input  wire logic        dicache_reuse,
    input  wire logic        dicache_reuse_done,
    output logic             dicache_probe_start,

    output logic [5:0]       step_index,
    output logic             branch_index,
    output logic [3:0]       block_index,
    input  wire logic [2:0]  dma_stage_index,
    output wire logic        block_is_skipped,
    output wire [31:0]      timestep_address,
    output wire [6:0]       stage_layer_id,
    output wire [31:0]      stage_weight_address,
    output wire [31:0]      stage_scale_address,
    output wire [31:0]      stage_bias_address,
    output wire [31:0]      stage_weight_bytes,
    output wire [31:0]      stage_scale_bytes,
    output wire [31:0]      stage_bias_bytes,
    output wire [31:0]      stage_weight_beats,
    output wire [31:0]      stage_scale_beats,
    output wire [31:0]      stage_bias_beats,
    output wire [15:0]      stage_output_features,
    output wire [15:0]      stage_input_features,
    output wire [15:0]      stage_n_tiles,
    output wire [15:0]      stage_k_tiles,
    output wire [31:0]      stage_n_tile_stride_bytes,
    output wire [31:0]      stage_k_tile_stride_bytes,
    output wire [31:0]      block_adaln_weight_address,
    output wire [31:0]      block_adaln_bias_address
);
    typedef enum logic [4:0] {
        S_IDLE,
        S_CONTEXT_START, S_CONTEXT_WAIT,
        S_PREP_START, S_PREP_WAIT,
        S_BLOCK_SCAN, S_BLOCK_PREP_START, S_BLOCK_PREP_WAIT,
        S_BLOCK_START, S_BLOCK_WAIT,
        S_DICACHE_WAIT, S_DICACHE_REUSE_WAIT, S_FINAL_START, S_FINAL_WAIT,
        S_CFG_START, S_CFG_WAIT,
        S_EULER_START, S_EULER_WAIT
    } state_t;

    state_t state;
    wire [3:0] descriptor_block_index =
        (block_index < BLOCK_COUNT) ? block_index : 4'd0;
    wire [2:0] descriptor_stage_index =
        (dma_stage_index < BLOCK_STAGE_COUNT) ? dma_stage_index : 3'd0;

    assign block_is_skipped = (block_index < BLOCK_COUNT) ?
        BLOCK_SKIP_MASK[step_index][block_index] : 1'b0;
    assign timestep_address = DDR_BASE + T_SCHEDULE_OFFSET + (step_index * 4);
    assign stage_layer_id = BLOCK_STAGE_LAYER_ID[descriptor_block_index][descriptor_stage_index];
    assign stage_weight_address = DDR_BASE +
        BLOCK_STAGE_WEIGHT_OFFSET[descriptor_block_index][descriptor_stage_index];
    assign stage_scale_address = DDR_BASE +
        BLOCK_STAGE_SCALE_OFFSET[descriptor_block_index][descriptor_stage_index];
    assign stage_bias_address = DDR_BASE +
        BLOCK_STAGE_BIAS_OFFSET[descriptor_block_index][descriptor_stage_index];
    assign stage_weight_bytes = BLOCK_STAGE_WEIGHT_BYTES[descriptor_block_index][descriptor_stage_index];
    assign stage_scale_bytes = BLOCK_STAGE_SCALE_BYTES[descriptor_block_index][descriptor_stage_index];
    assign stage_bias_bytes = BLOCK_STAGE_BIAS_BYTES[descriptor_block_index][descriptor_stage_index];
    assign stage_weight_beats = BLOCK_STAGE_WEIGHT_BEATS[descriptor_block_index][descriptor_stage_index];
    assign stage_scale_beats = BLOCK_STAGE_SCALE_BEATS[descriptor_block_index][descriptor_stage_index];
    assign stage_bias_beats = BLOCK_STAGE_BIAS_BEATS[descriptor_block_index][descriptor_stage_index];
    assign stage_output_features = BLOCK_STAGE_OUT[descriptor_block_index][descriptor_stage_index];
    assign stage_input_features = BLOCK_STAGE_IN[descriptor_block_index][descriptor_stage_index];
    assign stage_n_tiles = BLOCK_STAGE_N_TILES[descriptor_block_index][descriptor_stage_index];
    assign stage_k_tiles = BLOCK_STAGE_K_TILES[descriptor_block_index][descriptor_stage_index];
    assign stage_n_tile_stride_bytes =
        BLOCK_STAGE_N_TILE_STRIDE_BYTES[descriptor_block_index][descriptor_stage_index];
    assign stage_k_tile_stride_bytes =
        BLOCK_STAGE_K_TILE_STRIDE_BYTES[descriptor_block_index][descriptor_stage_index];
    assign block_adaln_weight_address = DDR_BASE +
        BLOCK_ADALN_WEIGHT_OFFSET[descriptor_block_index];
    assign block_adaln_bias_address = DDR_BASE +
        BLOCK_ADALN_BIAS_OFFSET[descriptor_block_index];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            context_start <= 1'b0;
            step_prep_start <= 1'b0;
            block_prepare_start <= 1'b0;
            block_start <= 1'b0;
            final_start <= 1'b0;
            cfg_combine_start <= 1'b0;
            euler_start <= 1'b0;
            dicache_probe_start <= 1'b0;
            step_index <= '0;
            branch_index <= 1'b0;
            block_index <= '0;
        end else begin
            done <= 1'b0;
            context_start <= 1'b0;
            step_prep_start <= 1'b0;
            block_prepare_start <= 1'b0;
            block_start <= 1'b0;
            final_start <= 1'b0;
            cfg_combine_start <= 1'b0;
            euler_start <= 1'b0;
            dicache_probe_start <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;
                        step_index <= '0;
                        branch_index <= 1'b0;
                        block_index <= '0;
                        state <= S_CONTEXT_START;
                    end
                end
                S_CONTEXT_START: begin
                    context_start <= 1'b1;
                    state <= S_CONTEXT_WAIT;
                end
                S_CONTEXT_WAIT: if (context_done) state <= S_PREP_START;
                S_PREP_START: begin
                    step_prep_start <= 1'b1;
                    state <= S_PREP_WAIT;
                end
                S_PREP_WAIT: if (step_prep_done) begin
                    block_index <= '0;
                    state <= S_BLOCK_SCAN;
                end
                S_BLOCK_SCAN: begin
                    if (block_index == BLOCK_COUNT) begin
                        state <= S_FINAL_START;
                    end else if (!dicache_enable && block_is_skipped) begin
                        block_index <= block_index + 1'b1;
                    end else begin
                        state <= S_BLOCK_PREP_START;
                    end
                end
                S_BLOCK_PREP_START: begin
                    block_prepare_start <= 1'b1;
                    state <= S_BLOCK_PREP_WAIT;
                end
                S_BLOCK_PREP_WAIT: if (block_prepare_done) begin
                    state <= S_BLOCK_START;
                end
                S_BLOCK_START: begin
                    block_start <= 1'b1;
                    state <= S_BLOCK_WAIT;
                end
                S_BLOCK_WAIT: if (block_done) begin
                    if (dicache_enable && block_index == 0) begin
                        block_index <= 4'd1;
                        dicache_probe_start <= 1'b1;
                        state <= S_DICACHE_WAIT;
                    end else begin
                        block_index <= block_index + 1'b1;
                        state <= S_BLOCK_SCAN;
                    end
                end
                S_DICACHE_WAIT: if (dicache_decision_valid) begin
                    if (dicache_reuse) begin
                        block_index <= BLOCK_COUNT;
                        state <= S_DICACHE_REUSE_WAIT;
                    end else state <= S_BLOCK_SCAN;
                end
                S_DICACHE_REUSE_WAIT: if (dicache_reuse_done)
                    state <= S_BLOCK_SCAN;
                S_FINAL_START: begin
                    final_start <= 1'b1;
                    state <= S_FINAL_WAIT;
                end
                S_FINAL_WAIT: if (final_done) begin
                    if (!branch_index) begin
                        branch_index <= 1'b1;
                        state <= S_PREP_START;
                    end else state <= S_CFG_START;
                end
                S_CFG_START: begin
                    cfg_combine_start <= 1'b1;
                    state <= S_CFG_WAIT;
                end
                S_CFG_WAIT: if (cfg_combine_done) state <= S_EULER_START;
                S_EULER_START: begin
                    euler_start <= 1'b1;
                    state <= S_EULER_WAIT;
                end
                S_EULER_WAIT: if (euler_done) begin
                    if (step_index == FLOW_STEPS - 1) begin
                        busy <= 1'b0;
                        done <= 1'b1;
                        state <= S_IDLE;
                    end else begin
                        step_index <= step_index + 1'b1;
                        branch_index <= 1'b0;
                        state <= S_PREP_START;
                    end
                end
                default: begin
                    busy <= 1'b0;
                    state <= S_IDLE;
                end
            endcase
        end
    end

    initial begin
        if (BLOCK_COUNT != 14 || BLOCK_STAGE_COUNT != 7 || FLOW_STEPS != 50 ||
            CFG_ROWS != 512 || WEIGHT_TILE_BYTES != 1024)
            $fatal(1, "unexpected Supra2-IMG runtime layout geometry");
    end
endmodule

`default_nettype wire
