`timescale 1ns/1ps
`default_nettype none

module tb_supra2_dit_weight_stage_loader;
    import supra2_dit_ddr_layout::*;

    localparam int BLOCK = 0;
    localparam int STAGE = 1;
    localparam int NTILES = 18;
    localparam int KTILES = 18;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic wload_req = 1'b0;
    logic [3:0] wload_stage = STAGE;
    logic [3:0] block_index = BLOCK;
    wire wload_ack;
    wire stage_request;
    logic stage_request_ready;
    wire [2:0] stage_request_index;
    wire [1:0] stage_component;
    wire tile_select;
    wire [15:0] n_tile_index, k_tile_index;
    logic [511:0] stage_data;
    wire stage_valid;
    wire stage_ready;
    wire stage_last;
    logic stage_dma_done = 1'b0;
    logic dma_error = 1'b0;
    wire wt_wr_en;
    wire [15:0] wt_wr_beat;
    wire [255:0] wt_wr_data;
    wire sb_wr_en, sb_wr_sel;
    wire [15:0] sb_wr_addr;
    wire [31:0] sb_wr_data;
    wire loader_error;

    logic response_active = 1'b0;
    logic [1:0] response_component = '0;
    logic [15:0] response_n_tile = '0, response_k_tile = '0;
    logic [5:0] response_beats_left = '0;
    integer request_total = 0;
    integer weight_write_total = 0;
    integer scalar_write_total = 0;
    integer cycle_count = 0;

    always #5 clk = ~clk;
    assign stage_request_ready = !response_active;
    assign stage_valid = response_active;
    assign stage_last = response_active && (response_beats_left == 1);

    function automatic logic [7:0] pattern_byte(
        input int component, input int n_tile, input int k_tile, input int offset
    );
        pattern_byte = 8'(component*53 + n_tile*11 + k_tile*7 + offset);
    endfunction

    always_comb begin
        stage_data = '0;
        for (int byte_index = 0; byte_index < 64; byte_index++)
            stage_data[byte_index*8 +: 8] = pattern_byte(
                response_component, response_n_tile, response_k_tile,
                (response_beats_left == 0 ? 0 :
                 ((response_component == 0 ? 16 : 2)-response_beats_left))*64 + byte_index);
    end

    supra2_dit_weight_stage_loader dut (.*);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            response_active <= 1'b0;
            response_component <= '0;
            response_n_tile <= '0;
            response_k_tile <= '0;
            response_beats_left <= '0;
            stage_dma_done <= 1'b0;
        end else begin
            stage_dma_done <= 1'b0;
            if (stage_request && stage_request_ready) begin
                response_active <= 1'b1;
                response_component <= stage_component;
                response_n_tile <= n_tile_index;
                response_k_tile <= k_tile_index;
                response_beats_left <= (stage_component == 0) ? 6'd16 : 6'd2;
            end
            if (stage_valid && stage_ready) begin
                if (response_beats_left == 1) begin
                    response_active <= 1'b0;
                    response_beats_left <= '0;
                    stage_dma_done <= 1'b1;
                end else response_beats_left <= response_beats_left - 1'b1;
            end
        end
    end

    always @(posedge clk) begin : scoreboard
        integer local_n;
        integer n_tile;
        integer k_tile;
        integer byte_index;
        integer scalar_index;
        logic [31:0] expected_scalar;
        if (!rst_n) begin
            request_total = 0;
            weight_write_total = 0;
            scalar_write_total = 0;
            cycle_count = 0;
        end else begin
            cycle_count = cycle_count + 1;
            if (cycle_count > 100000) $fatal(1, "stage-loader watchdog");

            if (stage_request && stage_request_ready) begin
                if (!tile_select || stage_request_index != STAGE[2:0])
                    $fatal(1, "invalid stage request metadata");
                if (request_total < NTILES*KTILES) begin
                    if (stage_component != 0 ||
                        n_tile_index != request_total/KTILES ||
                        k_tile_index != request_total%KTILES)
                        $fatal(1, "weight descriptor order mismatch request=%0d", request_total);
                end else if (request_total < NTILES*KTILES+NTILES) begin
                    if (stage_component != 1 ||
                        n_tile_index != request_total-NTILES*KTILES || k_tile_index != 0)
                        $fatal(1, "scale descriptor order mismatch request=%0d", request_total);
                end else begin
                    if (stage_component != 2 ||
                        n_tile_index != request_total-NTILES*KTILES-NTILES || k_tile_index != 0)
                        $fatal(1, "bias descriptor order mismatch request=%0d", request_total);
                end
                request_total = request_total + 1;
            end

            if (wt_wr_en) begin
                local_n = (int'(wt_wr_beat)/KTILES)%32;
                n_tile = (int'(wt_wr_beat)/KTILES)/32;
                k_tile = int'(wt_wr_beat)%KTILES;
                for (int lane = 0; lane < 32; lane++) begin
                    byte_index = lane*32+local_n;
                    if (wt_wr_data[lane*8 +: 8] !==
                        pattern_byte(0,n_tile,k_tile,byte_index))
                        $fatal(1, "weight byte mismatch beat=%0d lane=%0d", wt_wr_beat,lane);
                end
                weight_write_total = weight_write_total + 1;
            end

            if (sb_wr_en) begin
                scalar_index = int'(sb_wr_addr)%32;
                expected_scalar = {
                    pattern_byte(sb_wr_sel ? 2 : 1, int'(sb_wr_addr)/32, 0, scalar_index*4+3),
                    pattern_byte(sb_wr_sel ? 2 : 1, int'(sb_wr_addr)/32, 0, scalar_index*4+2),
                    pattern_byte(sb_wr_sel ? 2 : 1, int'(sb_wr_addr)/32, 0, scalar_index*4+1),
                    pattern_byte(sb_wr_sel ? 2 : 1, int'(sb_wr_addr)/32, 0, scalar_index*4)
                };
                if (sb_wr_data !== expected_scalar)
                    $fatal(1, "scale/bias word mismatch address=%0d", sb_wr_addr);
                scalar_write_total = scalar_write_total + 1;
            end

            if (wload_ack) begin
                if (loader_error || request_total != NTILES*KTILES+2*NTILES ||
                    weight_write_total != NTILES*KTILES*32 ||
                    scalar_write_total != 2*NTILES*32)
                    $fatal(1, "stage-loader totals req=%0d wt=%0d sb=%0d err=%b",
                           request_total,weight_write_total,scalar_write_total,loader_error);
                $display("SUPRA2_DIT_WEIGHT_STAGE_LOADER_PASS block=%0d stage=%0d tiles=%0dx%0d weight_writes=%0d scalar_writes=%0d",
                         BLOCK,STAGE,NTILES,KTILES,weight_write_total,scalar_write_total);
                $finish;
            end
        end
    end

    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk) rst_n = 1'b1;
        @(negedge clk) wload_req = 1'b1;
        wait (wload_ack);
        @(negedge clk) wload_req = 1'b0;
    end
endmodule

`default_nettype wire
