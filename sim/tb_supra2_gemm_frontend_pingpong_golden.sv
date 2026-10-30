`timescale 1ns/1ps
`default_nettype none
// Three M tiles ensure the first activation slot is reused with real data.
module tb_supra2_gemm_frontend_pingpong_golden #(
    parameter int M=96, N=32, K=576
);
    logic clk=0, rst_n=0, start=0, wt_en=0, sb_en=0, sb_sel=0;
    logic [15:0] wt_beat=0, sb_addr=0;
    logic [255:0] wt_data=0;
    logic [31:0] sb_data=0;
    wire done, out_valid, array_en;
    wire [15:0] raw_row, raw_beat, out_row, out_ct;
    logic [511:0] raw_data;
    wire [511:0] out_data;
    logic [15:0] raw_mem[0:M*K-1], expected_mem[0:M*N-1];
    logic [7:0] weight_mem[0:N*K-1];
    logic [31:0] scale_mem[0:N-1], bias_mem[0:N-1];
    logic [M*(N/32)-1:0] seen='0;
    integer outputs=0, overlap_cycles=0, drain_overlap_cycles=0;
    string raw_file, weight_file, scale_file, bias_file, expected_file;
    always #5 clk=~clk;
    initial begin #30000000; $fatal(1,"golden ping-pong watchdog"); end
    supra2_gemm_engine #(.MAX_M(M),.MAX_N(N),.MAX_K(K),.MAX_WEIGHT_ELEMS(N*K)) dut (
        .clk(clk),.rst_n(rst_n),.start(start),.busy(),.done(done),
        .cfg_fp16(1'b0),.cfg_fp16_bias_en(1'b0),.cfg_fp16_act_scale(32'd0),
        .cfg_fp16_wt_scale(32'd0),.cfg_m(16'(M)),.cfg_n(16'(N)),
        .cfg_k(16'(K)),.cfg_h(9'd64),.cfg_ni0(16'd0),.cfg_ni1(16'd0),.cfg_scale(32'd0),
        .raw_row(raw_row),.raw_beat(raw_beat),.raw_rd_data(raw_data),
        .raw_prefetch_en(),.raw_prefetch_row(),.raw_prefetch_beat(),
        .f16_wr_en(1'b0),.f16_wr_sel(1'b0),.f16_wr_beat(16'd0),.f16_wr_data(512'd0),
        .wt_wr_en(wt_en),.wt_wr_beat(wt_beat),.wt_wr_data(wt_data),
        .weight_write_slot(1'b0),.weight_read_slot(1'b0),.weight_load_k(16'd0),
        .sb_wr_en(sb_en),.sb_wr_sel(sb_sel),.sb_wr_addr(sb_addr),.sb_wr_data(sb_data),
        .out_valid(out_valid),.out_row(out_row),.out_ct(out_ct),.out_data(out_data),
        .shared_array_clear(),.shared_array_en(array_en),.shared_array_a(),.shared_array_b(),
        .shared_array_acc(),.shared_array_acc_valid(1'b0));
    always_comb begin
        raw_data='0;
        for(int lane=0;lane<32;lane++)
            if(raw_row<M && int'(raw_beat)*32+lane<K)
                raw_data[lane*16+:16]=raw_mem[int'(raw_row)*K+int'(raw_beat)*32+lane];
    end
    always @(posedge clk) if(rst_n) begin
        if(array_en && dut.fe_state!=dut.FE_IDLE) overlap_cycles<=overlap_cycles+1;
        if(array_en && dut.drain_active) drain_overlap_cycles<=drain_overlap_cycles+1;
        if(out_valid) begin
            if(out_ct>=N/32 || out_row>=M || seen[int'(out_row)*(N/32)+int'(out_ct)])
                $fatal(1,"invalid/duplicate output row=%0d col=%0d",out_row,out_ct);
            seen[int'(out_row)*(N/32)+int'(out_ct)]=1'b1;
            for(int lane=0;lane<32;lane++)
                if(out_data[lane*16+:16] !== expected_mem[int'(out_row)*N+int'(out_ct)*32+lane])
                    $fatal(1,"golden mismatch row=%0d lane=%0d got=%h expected=%h",
                        out_row,lane,out_data[lane*16+:16],expected_mem[int'(out_row)*N+int'(out_ct)*32+lane]);
            outputs++;
        end
    end
    initial begin
        if(!$value$plusargs("RAW=%s",raw_file) || !$value$plusargs("WEIGHT=%s",weight_file) ||
           !$value$plusargs("SCALE=%s",scale_file) || !$value$plusargs("BIAS=%s",bias_file) ||
           !$value$plusargs("EXPECTED=%s",expected_file)) $fatal(1,"missing fixture args");
        $readmemh(raw_file,raw_mem,0,M*K-1);
        $readmemh(weight_file,weight_mem,0,N*K-1);
        $readmemh(scale_file,scale_mem,0,N-1); $readmemh(bias_file,bias_mem,0,N-1);
        $readmemh(expected_file,expected_mem,0,M*N-1);
        repeat(4) @(negedge clk); rst_n=1;
        for(int wb=0;wb<N*K/32;wb++) begin
            @(negedge clk); wt_en=1; wt_beat=16'(wb);
            for(int lane=0;lane<32;lane++) wt_data[lane*8+:8]=weight_mem[wb*32+lane];
        end
        @(negedge clk); wt_en=0;
        for(int sel=0;sel<2;sel++) for(int col=0;col<N;col++) begin
            sb_en=1; sb_sel=1'(sel); sb_addr=16'(col);
            sb_data=sel ? bias_mem[col] : scale_mem[col]; @(negedge clk);
        end
        sb_en=0; start=1; @(negedge clk); start=0;
        wait(done); @(negedge clk);
        if(seen!=={(M*(N/32)){1'b1}} || outputs!=M*(N/32) ||
           (M>32 && overlap_cycles==0) || (N>32 && drain_overlap_cycles==0))
            $fatal(1,"incomplete output=%0d frontend_overlap=%0d drain_overlap=%0d",outputs,overlap_cycles,drain_overlap_cycles);
        $display("SUPRA2_GEMM_PINGPONG_GOLDEN_PASS M=%0d N=%0d K=%0d frontend_overlap=%0d drain_overlap=%0d",M,N,K,overlap_cycles,drain_overlap_cycles);
        $finish;
    end
endmodule
`default_nettype wire
