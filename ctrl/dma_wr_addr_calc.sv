`default_nettype none

module dma_wr_addr_calc (
    input  wire [31:0] base_addr,
    input  wire [3:0]  output_window,
    input  wire        token_stride,
    input  wire        head_scatter,
    input  wire [15:0] instruction_offset,
    input  wire [5:0]  token_index,
    input  wire [31:0] token_bytes,
    output logic [31:0] ddr_addr
);
    logic [31:0] window_offset;
    logic [31:0] command_offset;

    always_comb begin
        window_offset = {12'd0, output_window, 16'd0};
        command_offset = token_stride
                       ? (head_scatter
                          ? token_index * 32'd4096 +
                            {20'd0, instruction_offset[11:0]}
                          : token_index * token_bytes)
                       : {16'd0, instruction_offset};
        ddr_addr = base_addr + window_offset + command_offset;
    end
endmodule

`default_nettype wire
