`default_nettype none
module tb_supra2_gemm_frontend_overlap;
    logic clk=0, rst_n=0, start=0, wt_en=0, sb_en=0;
    logic [15:0] wt_beat=0, sb_addr=0;
    wire done, out_valid, array_en;
    wire [15:0] out_row, out_ct;
    wire [511:0] out_data;
    integer outputs=0, frontend_array_overlap=0;
    logic [95:0] seen='0;
    always #5 clk=~clk;
    supra2_gemm_engine #(.MAX_M(96),.MAX_N(32),.MAX_K(256),
        .MAX_WEIGHT_ELEMS(32*32)) dut (
        .clk(clk),.rst_n(rst_n),.start(start),.busy(),.done(done),
        .cfg_fp16(1'b0),.cfg_fp16_bias_en(1'b0),
        .cfg_fp16_act_scale(32'h3f800000),.cfg_fp16_wt_scale(32'h3f800000),
        .cfg_m(16'd96),.cfg_n(16'd32),.cfg_k(16'd32),.cfg_h(9'd16),
        .cfg_ni0(16'd0),.cfg_ni1(16'd0),.cfg_scale(32'h3f800000),
        .raw_row(),.raw_beat(),.raw_prefetch_en(),.raw_prefetch_row(),
        .raw_prefetch_beat(),.raw_rd_data(512'd0),
        .f16_wr_en(1'b0),.f16_wr_sel(1'b0),.f16_wr_beat(16'd0),.f16_wr_data(512'd0),
        .wt_wr_en(wt_en),.wt_wr_beat(wt_beat),.wt_wr_data(256'd0),
        .weight_write_slot(1'b0),.weight_read_slot(1'b0),.weight_load_k(16'd32),
        .sb_wr_en(sb_en),.sb_wr_sel(1'b1),.sb_wr_addr(sb_addr),
        .sb_wr_data(32'h3f800000),.out_valid(out_valid),.out_row(out_row),
        .out_ct(out_ct),.out_data(out_data),.shared_array_clear(),
        .shared_array_en(array_en),.shared_array_a(),.shared_array_b(),
        .shared_array_acc(),.shared_array_acc_valid(1'b0));
    always @(posedge clk) if (rst_n) begin
        if (array_en && dut.fe_state != dut.FE_IDLE) frontend_array_overlap <= frontend_array_overlap+1;
        if (out_valid) begin
            if (out_row>=96 || out_ct!=0 || seen[out_row]) $fatal(1,"bad output row=%0d",out_row);
            seen[out_row]=1'b1;
            for (int lane=0;lane<32;lane++)
                if (out_data[lane*16+:16] !== 16'h3c00)
                    $fatal(1,"wrong row=%0d lane=%0d got=%h",out_row,lane,out_data[lane*16+:16]);
            outputs=outputs+1;
        end
        if (done) begin
            if (outputs!=96 || seen!=={96{1'b1}} || frontend_array_overlap==0)
                $fatal(1,"bad completion outputs=%0d overlap=%0d",outputs,frontend_array_overlap);
            $display("SUPRA2_GEMM_FRONTEND_OVERLAP_PASS outputs=%0d overlap_cycles=%0d",outputs,frontend_array_overlap);
            $finish;
        end
    end
    initial begin
        repeat (4) @(negedge clk); rst_n=1;
        for (int i=0;i<32;i++) begin @(negedge clk); wt_en=1; wt_beat=16'(i); end
        @(negedge clk); wt_en=0;
        for (int i=0;i<32;i++) begin @(negedge clk); sb_en=1; sb_addr=16'(i); end
        @(negedge clk); sb_en=0; start=1;
        @(negedge clk); start=0;
        repeat (20000) @(negedge clk);
        $fatal(1,"frontend overlap watchdog state=%0d fe=%0d row=%0d beat=%0d obeat=%0d ready=%h mi=%0d outputs=%0d h_ready=%b h_done=%b q_done=%b",dut.state,dut.fe_state,dut.row,dut.beat,dut.obeat,dut.activation_ready,dut.mi,outputs,dut.fe_h_in_ready,dut.fe_h_done,dut.fe_q_done);
    end
endmodule
`default_nettype wire
