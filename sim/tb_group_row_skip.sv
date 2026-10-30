`timescale 1ns/1ps
`default_nettype none

module tb_group_row_skip;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic fetch_valid = 1'b0;
    logic [31:0] fetch_instr = '0;
    logic fetch_ready;
    logic dec_valid;
    logic [31:0] dec_instr;
    logic dec_ready = 1'b1;
    logic route_group_empty = 1'b0;
    logic [5:0] route_group_count = '0;
    logic source_skip_valid;
    logic [15:0] source_skip_count;
    logic [15:0] cfg_K = 16'd32;

    always #5 clk = ~clk;

    loop_controller dut (
        .clk(clk),
        .rst_n(rst_n),
        .fetch_valid(fetch_valid),
        .fetch_instr(fetch_instr),
        .fetch_ready(fetch_ready),
        .dec_valid(dec_valid),
        .dec_instr(dec_instr),
        .dec_ready(dec_ready),
        .route_group_empty(route_group_empty),
        .route_group_count(route_group_count),
        .source_skip_valid(source_skip_valid),
        .source_skip_count(source_skip_count),
        .cfg_K(cfg_K)
    );

    task automatic check_row(input logic [4:0] row,
                             input logic [5:0] active_rows,
                             input logic expected_skip);
        logic [15:0] marker_imm;
        begin
            marker_imm = {12'd0, row[4], 1'b0, 1'b0, 1'b0} |
                         {13'd0, 3'd6};
            fetch_instr = {4'h0, 4'hE, 4'hD, row[3:0], marker_imm};
            route_group_count = active_rows;
            route_group_empty = 1'b0;
            #1;
            if (source_skip_valid !== expected_skip ||
                source_skip_count !== 16'd6)
                $fatal(1, "row skip mismatch row=%0d active=%0d got=%b count=%0d",
                       row, active_rows, source_skip_valid, source_skip_count);
        end
    endtask

    initial begin
        repeat (2) @(posedge clk);
        rst_n = 1'b1;
        fetch_valid = 1'b1;
        #1;
        if (fetch_ready !== 1'b1 || dec_valid !== 1'b1)
            $fatal(1, "loop controller did not enter pass-through state");

        check_row(5'd1, 6'd2, 1'b0);
        check_row(5'd2, 6'd2, 1'b1);
        check_row(5'd17, 6'd18, 1'b0);
        check_row(5'd17, 6'd17, 1'b1);

        fetch_instr = {4'h0, 4'hE, 4'hF, 4'h0, 16'd23};
        route_group_empty = 1'b1;
        #1;
        if (source_skip_valid !== 1'b1 || source_skip_count !== 16'd23)
            $fatal(1, "empty-group skip mismatch valid=%b count=%0d",
                   source_skip_valid, source_skip_count);

        $display("PASS: active rows execute; inactive rows and empty groups skip");
        $finish;
    end
endmodule

`default_nettype wire
