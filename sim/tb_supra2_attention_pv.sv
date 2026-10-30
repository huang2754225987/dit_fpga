`timescale 1ns/1ps
`default_nettype none

module tb_supra2_attention_pv;
    localparam int M=32, N=32, K=256, TOKENS=256, HEADS=9, HEAD_DIM=64;
    localparam int PROB_COUNT=TOKENS*HEADS*TOKENS;
    localparam int VALUE_COUNT=HEADS*HEAD_DIM*TOKENS;
    localparam int OUT_COUNT=HEADS*TOKENS*HEAD_DIM;
    logic clk=1'b0, rst_n=1'b0;
    logic clear_acc=1'b0, array_en=1'b1, dtype_fp16=1'b1;
    logic [5:0] active_rows=M;
    logic signed [7:0] a_col[M], b_row[N];
    logic signed [15:0] a_col_fp16[M], b_row_fp16[N];
    logic signed [31:0] acc_out[M][N];
    logic acc_valid;
    logic [15:0] prob_mem[0:PROB_COUNT-1];
    logic [15:0] value_mem[0:VALUE_COUNT-1];
    logic [15:0] expected_mem[0:OUT_COUNT-1];
    logic [15:0] actual_mem[0:OUT_COUNT-1];
    string prob_file, value_file, expected_file, actual_file;
    integer head, query_tile, feature_tile, t, i, j, source_k;
    integer query, feature, flat, cycles, mismatches, output_fd;
    real got, expected, abs_error, max_error, max_ref;

    always #5 clk=~clk;
    systolic_array #(.M(M),.N(N),.DATA_W(8),.ACC_W(32),.PIPE_STAGE(32)) dut (
        .clk,.rst_n,.clear_acc,.en(array_en),.a_unsigned(1'b0),.active_rows,
        .a_col,.b_row,.acc_out,.acc_valid
    );

    function automatic real half_to_real(input logic [15:0] value);
        integer exponent;
        real significand;
        begin
            exponent=value[14:10];
            if (exponent==0) significand=value[9:0]/1024.0;
            else significand=1.0+value[9:0]/1024.0;
            half_to_real=(value[15]?-1.0:1.0)*significand*
                (exponent==0?(2.0**-14):(2.0**(exponent-15)));
        end
    endfunction

    always @(posedge clk) if (array_en) cycles<=cycles+1;

    initial begin
        if (!$value$plusargs("PROB_FILE=%s",prob_file) ||
            !$value$plusargs("VALUE_FILE=%s",value_file) ||
            !$value$plusargs("EXPECTED_FILE=%s",expected_file) ||
            !$value$plusargs("ACTUAL_FILE=%s",actual_file))
            $fatal(1,"Missing attention PV fixture plusargs");
        $readmemh(prob_file,prob_mem);
        $readmemh(value_file,value_mem);
        $readmemh(expected_file,expected_mem);
        output_fd=$fopen(actual_file,"w");
        if (!output_fd) $fatal(1,"Cannot open attention PV RTL output");
        cycles=0; mismatches=0; max_error=0.0; max_ref=0.0;
        repeat(4) @(negedge clk);
        rst_n=1'b1;
        for (head=0;head<HEADS;head++) begin
            for (query_tile=0;query_tile<TOKENS/M;query_tile++) begin
                for (feature_tile=0;feature_tile<HEAD_DIM/N;feature_tile++) begin
                    clear_acc=1'b1;
                    @(negedge clk);
                    clear_acc=1'b0;
                    for (t=0;t<K+M+N+3;t++) begin
                        for (i=0;i<M;i++) begin
                            source_k=t-i;
                            query=query_tile*M+i;
                            a_col[i]=(source_k>=0&&source_k<K)
                                ? fp16_utils_pkg::fp16_to_int8_q(
                                    prob_mem[(query*HEADS+head)*TOKENS+source_k],7) : 8'sd0;
                        end
                        for (j=0;j<N;j++) begin
                            source_k=t-j;
                            feature=feature_tile*N+j;
                            b_row[j]=(source_k>=0&&source_k<K)
                                ? fp16_utils_pkg::fp16_to_int8_q(
                                    value_mem[(head*HEAD_DIM+feature)*TOKENS+source_k],5) : 8'sd0;
                        end
                        @(negedge clk);
                    end
                    repeat(3) @(negedge clk);
                    if (!acc_valid) $fatal(1,"PV tile did not assert acc_valid");
                    for (i=0;i<M;i++) begin
                        query=query_tile*M+i;
                        for (j=0;j<N;j++) begin
                            feature=feature_tile*N+j;
                            flat=(head*TOKENS+query)*HEAD_DIM+feature;
                            actual_mem[flat]=fp16_utils_pkg::fp32_to_fp16(
                                fp16_utils_pkg::fp32_mul_rne(
                                    fp16_utils_pkg::fp32_from_int32_rne(acc_out[i][j]),
                                    32'h39800000));
                            got=half_to_real(actual_mem[flat]);
                            expected=half_to_real(expected_mem[flat]);
                            abs_error=(got>=expected)?got-expected:expected-got;
                            if (abs_error>max_error) max_error=abs_error;
                            if ((expected>=0?expected:-expected)>max_ref)
                                max_ref=(expected>=0?expected:-expected);
                            if (abs_error>0.004) mismatches++;
                        end
                    end
                end
            end
        end
        for (query=0;query<TOKENS;query++)
            for (head=0;head<HEADS;head++)
                for (feature=0;feature<HEAD_DIM;feature++) begin
                    flat=(head*TOKENS+query)*HEAD_DIM+feature;
                    $fdisplay(output_fd,"%04h",actual_mem[flat]);
                end
        $fclose(output_fd);
        $display("SUPRA2_ATTN_PV_PASS queries=%0d heads=%0d keys=%0d cycles=%0d",
                 TOKENS,HEADS,TOKENS,cycles);
        $display("SUPRA2_ATTN_PV_MAX_ABS %.9g",max_error);
        $display("SUPRA2_ATTN_PV_REL_TO_MAX %.9g",max_error/max_ref);
        $display("SUPRA2_ATTN_PV_TOL_MISMATCHES %0d",mismatches);
        if (mismatches!=0) $fatal(1,"PV output exceeded 0.004 absolute error gate");
        $finish;
    end
endmodule

`default_nettype wire
