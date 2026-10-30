`default_nettype none

// Resolves a tile request for any quantized layer in the generated DDR image.
// Component 0 selects one 32x32 INT8 weight tile; components 1 and 2 select
// the matching 32-element FP32 scale or bias vector tile.
module supra2_dit_qlinear_descriptor
    import supra2_dit_ddr_layout::*;
(
    input  wire logic [6:0]  layer_id,
    input  wire logic [1:0]  component,
    input  wire logic [15:0] n_tile_index,
    input  wire logic [15:0] k_tile_index,
    output logic             descriptor_valid,
    output logic [31:0]      descriptor_address,
    output logic [31:0]      descriptor_beats,
    output logic [15:0]      output_features,
    output logic [15:0]      input_features,
    output logic [15:0]      n_tiles,
    output logic [15:0]      k_tiles
);
    logic [31:0] component_offset;
    logic [31:0] component_bytes;
    logic [31:0] tile_offset;
    logic component_valid;

    always_comb begin
        descriptor_valid = 1'b0;
        descriptor_address = '0;
        descriptor_beats = '0;
        output_features = '0;
        input_features = '0;
        n_tiles = '0;
        k_tiles = '0;
        component_offset = '0;
        component_bytes = '0;
        tile_offset = '0;
        component_valid = 1'b0;

        if (layer_id < QLAYER_COUNT) begin
            output_features = 16'(QLAYER_OUT[layer_id]);
            input_features = 16'(QLAYER_IN[layer_id]);
            n_tiles = 16'(QLAYER_N_TILES[layer_id]);
            k_tiles = 16'(QLAYER_K_TILES[layer_id]);

            case (component)
                2'd0: begin
                    component_offset = 32'(QLAYER_WEIGHT_OFFSET[layer_id]);
                    component_bytes = 32'(QLAYER_WEIGHT_BYTES[layer_id]);
                    component_valid = (n_tile_index < n_tiles) &&
                                      (k_tile_index < k_tiles);
                    tile_offset = n_tile_index *
                                  32'(QLAYER_N_TILE_STRIDE_BYTES[layer_id]) +
                                  k_tile_index *
                                  32'(QLAYER_K_TILE_STRIDE_BYTES[layer_id]);
                    descriptor_beats = WEIGHT_TILE_BYTES / 64;
                end
                2'd1: begin
                    component_offset = 32'(QLAYER_SCALE_OFFSET[layer_id]);
                    component_bytes = 32'(QLAYER_SCALE_BYTES[layer_id]);
                    component_valid = (n_tile_index < n_tiles);
                    tile_offset = n_tile_index * 32'd128;
                    descriptor_beats = 32'd2;
                end
                2'd2: begin
                    component_offset = 32'(QLAYER_BIAS_OFFSET[layer_id]);
                    component_bytes = 32'(QLAYER_BIAS_BYTES[layer_id]);
                    component_valid = (n_tile_index < n_tiles);
                    tile_offset = n_tile_index * 32'd128;
                    descriptor_beats = 32'd2;
                end
                default: begin
                    component_valid = 1'b0;
                end
            endcase

            descriptor_address = 32'(DDR_BASE) + component_offset + tile_offset;
            descriptor_valid = component_valid &&
                ((tile_offset + descriptor_beats * 32'd64) <= component_bytes);
            if (!descriptor_valid) begin
                descriptor_address = '0;
                descriptor_beats = '0;
            end
        end
    end
endmodule

`default_nettype wire
