`timescale 1ns/1ps
`default_nettype none

module tb_supra2_dit_qlinear_descriptor;
    import supra2_dit_ddr_layout::*;

    logic [6:0] layer_id;
    logic [1:0] component;
    logic [15:0] n_tile_index, k_tile_index;
    wire descriptor_valid;
    wire [31:0] descriptor_address, descriptor_beats;
    wire [15:0] output_features, input_features, n_tiles, k_tiles;

    supra2_dit_qlinear_descriptor dut (.*);

    task automatic check_request(
        input logic [6:0] test_layer,
        input logic [1:0] test_component,
        input logic [15:0] test_n,
        input logic [15:0] test_k,
        input logic expected_valid,
        input logic [31:0] expected_address,
        input logic [31:0] expected_beats
    );
        begin
            layer_id = test_layer;
            component = test_component;
            n_tile_index = test_n;
            k_tile_index = test_k;
            #1;
            if (descriptor_valid !== expected_valid ||
                descriptor_address !== expected_address ||
                descriptor_beats !== expected_beats) begin
                $fatal(1, "descriptor mismatch layer=%0d component=%0d n=%0d k=%0d valid=%b addr=%08x beats=%0d",
                       test_layer, test_component, test_n, test_k,
                       descriptor_valid, descriptor_address, descriptor_beats);
            end
        end
    endtask

    initial begin
        check_request(7'(X_EMBED_LAYER_ID), 2'd0, 16'd0, 16'd0, 1'b1,
                      32'(DDR_BASE + QLAYER_WEIGHT_OFFSET[X_EMBED_LAYER_ID]), 32'd16);
        if (output_features != 576 || input_features != 16 ||
            n_tiles != 18 || k_tiles != 1)
            $fatal(1, "x_embed dimensions or tiling mismatch");

        check_request(7'(CTX_PROJ_LAYER_ID), 2'd0, 16'd17, 16'd23, 1'b1,
                      32'(DDR_BASE + QLAYER_WEIGHT_OFFSET[CTX_PROJ_LAYER_ID] +
                           17 * QLAYER_N_TILE_STRIDE_BYTES[CTX_PROJ_LAYER_ID] +
                           23 * QLAYER_K_TILE_STRIDE_BYTES[CTX_PROJ_LAYER_ID]),
                      32'd16);
        if (output_features != 576 || input_features != 768 ||
            n_tiles != 18 || k_tiles != 24)
            $fatal(1, "ctx_proj dimensions or tiling mismatch");

        check_request(7'(CTX_PROJ_LAYER_ID), 2'd1, 16'd17, 16'd0, 1'b1,
                      32'(DDR_BASE + QLAYER_SCALE_OFFSET[CTX_PROJ_LAYER_ID] + 17 * 128),
                      32'd2);
        check_request(7'(CTX_PROJ_LAYER_ID), 2'd2, 16'd17, 16'd0, 1'b1,
                      32'(DDR_BASE + QLAYER_BIAS_OFFSET[CTX_PROJ_LAYER_ID] + 17 * 128),
                      32'd2);
        check_request(7'(CTX_PROJ_LAYER_ID), 2'd0, 16'd18, 16'd0, 1'b0, 32'd0, 32'd0);
        check_request(7'(CTX_PROJ_LAYER_ID), 2'd0, 16'd0, 16'd24, 1'b0, 32'd0, 32'd0);
        check_request(7'(QLAYER_COUNT), 2'd0, 16'd0, 16'd0, 1'b0, 32'd0, 32'd0);
        check_request(7'(X_EMBED_LAYER_ID), 2'd3, 16'd0, 16'd0, 1'b0, 32'd0, 32'd0);

        $display("SUPRA2_DIT_QLINEAR_DESCRIPTOR_PASS layers=%0d x_embed=576x16 ctx_proj=128x768->576",
                 QLAYER_COUNT);
        $finish;
    end
endmodule

`default_nettype wire
