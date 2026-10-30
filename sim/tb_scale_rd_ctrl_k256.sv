`timescale 1ns/1ps

module tb_scale_rd_ctrl_k256;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic start = 1'b0;
    logic [15:0] load_k = 16'd0;
    logic [5:0] group_idx = 6'd0;
    logic wide_scale_layout = 1'b0;
    logic group_sel = 1'b0;
    logic [3:0] expert_sel = 4'd0;
    logic [1:0] proj = 2'd0;
    logic wr_bank_sel = 1'b0;
    logic rd_bank_sel = 1'b0;
    logic start_overrun;
    logic rd_en;
    logic [11:0] rd_addr;
    logic [255:0] rd_data = '0;
    logic rd_valid = 1'b0;
    wire [511:0] scale_data;
    wire [511:0] zero_data;
    wire [1:0] ready;

    logic [255:0] mem [0:1023];
    integer read_count = 0;

    always #5 clk = ~clk;

    always_ff @(posedge clk) begin
        rd_valid <= rd_en;
        if (rd_en) begin
            rd_data <= mem[rd_addr];
            read_count <= read_count + 1;
        end
    end

    scale_rd_ctrl dut (
        .clk(clk), .rst_n(rst_n), .start(start), .load_k(load_k),
        .invalidate(1'b0),
        .group_idx(group_idx), .group_sel(group_sel),
        .wide_scale_layout(wide_scale_layout),
        .expert_sel(expert_sel), .proj(proj),
        .wr_bank_sel(wr_bank_sel), .rd_bank_sel(rd_bank_sel),
        .start_overrun(start_overrun), .rd_en(rd_en), .rd_addr(rd_addr),
        .rd_data(rd_data), .rd_valid(rd_valid),
        .scale_data(scale_data), .zero_data(zero_data), .ready(ready)
    );

    task automatic load_group(
        input logic [15:0] k_value,
        input logic [5:0] group_value,
        input logic [3:0] expert_value,
        input logic [1:0] proj_value,
        input logic bank_value,
        input integer expected_reads
    );
        integer reads_before;
        begin
            reads_before = read_count;
            @(negedge clk);
            load_k = k_value;
            group_idx = group_value;
            expert_sel = expert_value;
            proj = proj_value;
            wr_bank_sel = bank_value;
            rd_bank_sel = bank_value;
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            wait (ready[bank_value] === 1'b1);
            if (read_count - reads_before != expected_reads)
                $fatal(1, "scale read count %0d != expected %0d",
                       read_count - reads_before, expected_reads);
            if (start_overrun !== 1'b0)
                $fatal(1, "unexpected scale read overrun");
        end
    endtask

    integer i;
    integer base_addr;
    initial begin
        for (i = 0; i < 1024; i = i + 1)
            mem[i] = i;

        repeat (3) @(negedge clk);
        rst_n = 1'b1;

        // Expert 1 / Gate / group 2: K=256 must read groups 2 and 3.
        load_group(16'd256, 4'd2, 4'd1, 2'd0, 1'b1, 8);
        base_addr = 8;
        group_sel = 1'b0;
        #1;
        if (scale_data !== {mem[base_addr+1], mem[base_addr]} ||
            zero_data !== {mem[base_addr+3], mem[base_addr+2]})
            $fatal(1, "first 128-K scale/zero group mismatch");
        group_sel = 1'b1;
        #1;
        if (scale_data !== {mem[base_addr+5], mem[base_addr+4]} ||
            zero_data !== {mem[base_addr+7], mem[base_addr+6]})
            $fatal(1, "second 128-K scale/zero group mismatch");

        // A 128-K read remains four beats and selects its only group.
        load_group(16'd128, 4'd4, 4'd2, 2'd1, 1'b0, 4);
        base_addr = ((2 - 1) * 3 + 1) * 64 + 4 * 4;
        group_sel = 1'b0;
        #1;
        if (scale_data !== {mem[base_addr+1], mem[base_addr]} ||
            zero_data !== {mem[base_addr+3], mem[base_addr+2]})
            $fatal(1, "128-K scale/zero group mismatch");

        $display("[TB][SCALE_RD_K256] PASS: 8-beat double-group and 4-beat single-group reads");
        $finish;
    end
endmodule
