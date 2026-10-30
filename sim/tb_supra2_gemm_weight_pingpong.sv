`default_nettype none
module tb_supra2_gemm_weight_pingpong;
    logic clk=0, rst_n=0, start=0, f16_en=0, f16_sel=0, sb_en=0, wt_en=0;
    logic write_slot=0, read_slot=0;
    logic [15:0] f16_beat=0, sb_addr=0;
    logic [15:0] wt_beat=0, load_k=32;
    logic [255:0] wt_data=0;
    logic [31:0] sb_data=0;
    wire done, out_valid, array_en;
    wire [15:0] out_row, out_ct;
    wire [511:0] out_data;
    integer pass=0, outputs=0, concurrent_writes=0;
    always #5 clk=~clk;
    supra2_gemm_engine #(.MAX_M(32),.MAX_N(32),.MAX_K(256),
        .MAX_WEIGHT_ELEMS(32*64),.MAX_F16_K(32),.MAX_F16_N(32),
        .DUAL_WEIGHT_BUFFERS(1'b1)) dut (
        .clk(clk),.rst_n(rst_n),.start(start),.busy(),.done(done),
        .cfg_fp16(1'b1),.cfg_fp16_bias_en(1'b1),
        .cfg_fp16_act_scale(32'h3f800000),.cfg_fp16_wt_scale(32'h3f800000),
        .cfg_m(16'd32),.cfg_n(16'd32),.cfg_k(16'd32),.cfg_h(9'd16),
        .cfg_ni0(16'd0),.cfg_ni1(16'd0),.cfg_scale(32'h3f800000),
        .raw_row(),.raw_beat(),.raw_prefetch_en(),.raw_prefetch_row(),
        .raw_prefetch_beat(),.raw_rd_data(512'd0),
        .f16_wr_en(f16_en),.f16_wr_sel(f16_sel),.f16_wr_beat(f16_beat),.f16_wr_data(512'd0),
        .wt_wr_en(wt_en),.wt_wr_beat(wt_beat),.wt_wr_data(wt_data),
        .weight_write_slot(write_slot),.weight_read_slot(read_slot),.weight_load_k(load_k),
        .sb_wr_en(sb_en),.sb_wr_sel(1'b1),.sb_wr_addr(sb_addr),.sb_wr_data(sb_data),
        .out_valid(out_valid),.out_row(out_row),.out_ct(out_ct),.out_data(out_data),
        .shared_array_clear(),.shared_array_en(array_en),.shared_array_a(),.shared_array_b(),
        .shared_array_acc(),.shared_array_acc_valid(1'b0));
    always @(posedge clk) if (rst_n) begin
        if (sb_en && write_slot && array_en) concurrent_writes <= concurrent_writes+1;
        if (out_valid) begin
            if (out_ct!=0 || out_row>=32) $fatal(1,"invalid output");
            for (int j=0;j<32;j++)
                if (out_data[j*16+:16] !== (pass==0 ? 16'h3c00 : 16'h4000))
                    $fatal(1,"slot corruption pass=%0d row=%0d lane=%0d got=%h",pass,out_row,j,out_data[j*16+:16]);
            outputs=outputs+1;
        end
    end
    initial begin
        repeat (4) @(negedge clk); rst_n=1;
        // 加载 K 可与计算 K 不同：beat 2 在 K=64 时属于输出行 1。
        @(negedge clk); write_slot=0; load_k=32; wt_en=1; wt_beat=0; wt_data={32{8'h11}};
        @(negedge clk); write_slot=1; load_k=64; wt_beat=2; wt_data={32{8'h22}};
        @(negedge clk); wt_en=0;
        if (dut.gen_operand_ram_banks[0].weight_ram.mem[0] !== {32{8'h11}} ||
            dut.gen_operand_ram_banks[1].g_weight_second_slot.weight_ram1.mem[0] !== {32{8'h22}})
            $fatal(1,"weight slot or independent load K mapping failed");
        write_slot=0; load_k=32;
        for (int i=0;i<32;i++) begin @(negedge clk); f16_en=1; f16_sel=0; f16_beat=16'(i); end
        for (int i=0;i<32;i++) begin @(negedge clk); f16_sel=1; f16_beat=16'(i); end
        @(negedge clk); f16_en=0;
        for (int i=0;i<32;i++) begin @(negedge clk); sb_en=1; sb_addr=16'(i); sb_data=32'h3f800000; end
        @(negedge clk); sb_en=0; start=1;
        @(negedge clk); start=0;
        wait (array_en);
        for (int i=0;i<32;i++) begin
            @(negedge clk); write_slot=1; sb_en=1; sb_addr=16'(i); sb_data=32'h40000000;
        end
        @(negedge clk); sb_en=0;
        wait(done); @(negedge clk);
        if (outputs!=32 || concurrent_writes==0) $fatal(1,"first pass incomplete");
        pass=1; read_slot=1; start=1;
        @(negedge clk); start=0;
        wait(done); @(negedge clk);
        if (outputs!=64) $fatal(1,"second pass incomplete");
        $display("SUPRA2_GEMM_WEIGHT_PINGPONG_PASS outputs=%0d concurrent_writes=%0d",outputs,concurrent_writes);
        $finish;
    end
    initial begin #1000000; $fatal(1,"weight ping-pong watchdog"); end
endmodule
`default_nettype wire
