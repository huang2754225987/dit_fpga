`timescale 1ns/1ps
`default_nettype none
// Exercises the real engine feeder, including inter-block Hadamard stalls.
module tb_supra2_gemm_frontend;
    localparam int M=32, N=32, MAX_K=2304;
    logic clk=0, rst_n=0, start=0;
    logic [15:0] cfg_k;
    logic [8:0] cfg_h;
    wire busy, done, out_valid;
    wire [15:0] raw_row, raw_beat, out_row, out_ct;
    logic [511:0] raw_data;
    wire [511:0] out_data;
    wire shared_array_clear, shared_array_en;
    wire signed [7:0] shared_array_a [M], shared_array_b [M];
    wire signed [31:0] shared_array_acc [M][M];
    wire shared_array_acc_valid;
    logic wt_en=0, sb_en=0, sb_sel=0;
    logic [15:0] wt_beat=0, sb_addr=0;
    logic [255:0] wt_data=0;
    logic [31:0] sb_data=0;
    logic [15:0] raw_mem[0:M*MAX_K-1], expected_mem[0:M*N-1];
    logic [7:0] weight_mem[0:N*MAX_K-1];
    logic [31:0] scale_mem[0:N-1], bias_mem[0:N-1];
    logic [M-1:0] seen='0;
    integer k, h, outputs=0, stalls=0;
    logic held=0;
    logic [15:0] held_row, held_beat;
    logic [511:0] held_data;
    string raw_file, weight_file, scale_file, bias_file, expected_file;
    always #5 clk=~clk;
    initial begin #5000000; $fatal(1,"GEMM frontend timeout"); end
    supra2_gemm_engine #(.MAX_M(M),.MAX_N(N),.MAX_K(MAX_K),
        .MAX_WEIGHT_ELEMS(N*MAX_K), .EXTERNAL_ARRAY(1'b1)) dut (
        .clk(clk),.rst_n(rst_n),.start(start),.busy(busy),.done(done),
        .cfg_fp16(1'b0),.cfg_fp16_bias_en(1'b0),
        .cfg_fp16_act_scale(32'd0),.cfg_fp16_wt_scale(32'd0),
        .cfg_m(16'(M)),.cfg_n(16'(N)),.cfg_k(cfg_k),.cfg_h(cfg_h),
        .cfg_ni0(16'd0),.cfg_ni1(16'd0),.cfg_scale(32'd0),
        .raw_row(raw_row),.raw_beat(raw_beat),.raw_rd_data(raw_data),
        .f16_wr_en(1'b0),.f16_wr_sel(1'b0),.f16_wr_beat(16'd0),.f16_wr_data(512'd0),
        .wt_wr_en(wt_en),.wt_wr_beat(wt_beat),.wt_wr_data(wt_data),
        .weight_write_slot(1'b0),.weight_read_slot(1'b0),.weight_load_k(16'd0),
        .sb_wr_en(sb_en),.sb_wr_sel(sb_sel),.sb_wr_addr(sb_addr),.sb_wr_data(sb_data),
        .out_valid(out_valid),.out_row(out_row),.out_ct(out_ct),.out_data(out_data),
        .shared_array_clear(shared_array_clear), .shared_array_en(shared_array_en),
        .shared_array_a(shared_array_a), .shared_array_b(shared_array_b),
        .shared_array_acc(shared_array_acc),
        .shared_array_acc_valid(shared_array_acc_valid)
    );
    systolic_array #(
        .M(M), .N(M), .DATA_W(8), .ACC_W(32),
        .RUNTIME_UNSIGNED_A(1'b1), .PIPE_STAGE(32)
    ) shared_array (
        .clk(clk), .rst_n(rst_n), .clear_acc(shared_array_clear),
        .en(shared_array_en), .a_unsigned(1'b0), .active_rows(6'(M)),
        .a_col(shared_array_a), .b_row(shared_array_b),
        .acc_out(shared_array_acc), .acc_valid(shared_array_acc_valid)
    );
    always_comb begin
        raw_data='0;
        for(int lane=0;lane<32;lane++)
            if(raw_row<M && int'(raw_beat)*32+lane<k)
                raw_data[lane*16+:16]=raw_mem[int'(raw_row)*k+int'(raw_beat)*32+lane];
    end
    always @(posedge clk) if(rst_n) begin
        if(held && (raw_row!==held_row || raw_beat!==held_beat || raw_data!==held_data))
            $fatal(1,"Engine changed raw data/address while Hadamard stalled");
        held <= dut.fe_in_valid && !dut.fe_h_in_ready;
        held_row <= raw_row; held_beat <= raw_beat; held_data <= raw_data;
        if(dut.fe_in_valid && !dut.fe_h_in_ready) stalls++;
        if(out_valid) begin
            if(out_ct!=0 || out_row>=M || seen[out_row])
                $fatal(1,"Unexpected/duplicate engine output row=%0d ct=%0d",out_row,out_ct);
            seen[out_row]=1'b1;
            for(int lane=0;lane<N;lane++)
                if(out_data[lane*16+:16] !== expected_mem[int'(out_row)*N+lane])
                    $fatal(1,"Engine golden mismatch row=%0d lane=%0d got=%h expected=%h",
                        out_row,lane,out_data[lane*16+:16],expected_mem[int'(out_row)*N+lane]);
            outputs++;
        end
    end
    initial begin
        if(!$value$plusargs("K=%d",k) || !$value$plusargs("H=%d",h) ||
           !$value$plusargs("RAW=%s",raw_file) || !$value$plusargs("WEIGHT=%s",weight_file) ||
           !$value$plusargs("SCALE=%s",scale_file) || !$value$plusargs("BIAS=%s",bias_file) ||
           !$value$plusargs("EXPECTED=%s",expected_file) || k<=0 || k>MAX_K)
            $fatal(1,"Missing GEMM frontend fixture arguments");
        cfg_k=16'(k); cfg_h=9'(h);
        $readmemh(raw_file,raw_mem,0,M*k-1);
        $readmemh(weight_file,weight_mem,0,N*k-1);
        $readmemh(scale_file,scale_mem); $readmemh(bias_file,bias_mem);
        $readmemh(expected_file,expected_mem);
        repeat(4) @(negedge clk);
        rst_n=1;
        for(int wb=0;wb<N*k/32;wb++) begin
            @(negedge clk); wt_en=1; wt_beat=16'(wb);
            for(int lane=0;lane<32;lane++) wt_data[lane*8+:8]=weight_mem[wb*32+lane];
        end
        @(negedge clk); wt_en=0;
        for(int sel=0;sel<2;sel++) for(int col=0;col<N;col++) begin
            sb_en=1; sb_sel=1'(sel); sb_addr=16'(col);
            sb_data=sel ? bias_mem[col] : scale_mem[col];
            @(negedge clk);
        end
        sb_en=0; start=1;
        @(negedge clk); start=0;
        wait(done); @(negedge clk);
        if(seen!=={M{1'b1}} || outputs!=M || busy || stalls==0)
            $fatal(1,"Incomplete GEMM frontend test outputs=%0d stalls=%0d",outputs,stalls);
        $display("SUPRA2_GEMM_FRONTEND_PASS M=32 N=32 K=%0d H=%0d outputs=%0d stalls=%0d",k,h,outputs,stalls);
        $finish;
    end
endmodule
`default_nettype wire
