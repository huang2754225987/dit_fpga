`timescale 1ns/1ps
`default_nettype none

module tb_dma_wr_addr_calc;
    logic [31:0] base_addr;
    logic [3:0] output_window;
    logic token_stride;
    logic head_scatter;
    logic [15:0] instruction_offset;
    logic [5:0] token_index;
    logic [31:0] token_bytes;
    logic [31:0] ddr_addr;

    dma_wr_addr_calc dut (.*);

    task automatic check_addr(input logic [31:0] expected,
                              input string label);
        #1;
        if (ddr_addr !== expected)
            $fatal(1, "%s: expected=%08x actual=%08x",
                   label, expected, ddr_addr);
        $display("[TB][DMA_WR_WINDOW] PASS %s addr=%08x", label, ddr_addr);
    endtask

    initial begin
        base_addr = 32'h0100_0000;
        output_window = 0;
        token_stride = 0;
        head_scatter = 0;
        instruction_offset = 16'h2000;
        token_index = 0;
        token_bytes = 32'd1024;
        check_addr(32'h0100_2000, "legacy-window-zero");

        output_window = 3;
        check_addr(32'h0103_2000, "static-window-three");

        output_window = 2;
        token_stride = 1;
        token_index = 31;
        check_addr(32'h0102_7c00, "token-window-two");

        output_window = 0;
        token_bytes = 32'd256;
        token_index = 5;
        check_addr(32'h0100_0500, "legacy-token-stride");

        output_window = 2;
        head_scatter = 1;
        instruction_offset = 16'h0100;
        token_index = 5;
        check_addr(32'h0102_5100, "head-token-scatter-stride");

        instruction_offset = 16'h6100;
        check_addr(32'h0102_5100, "head-token-scatter-extended-source-offset");

        $display("[TB][DMA_WR_WINDOW] ALL PASS");
        $finish;
    end
endmodule

`default_nettype wire
