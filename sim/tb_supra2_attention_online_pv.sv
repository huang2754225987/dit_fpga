`timescale 1ns/1ps
`default_nettype none

module tb_supra2_attention_online_pv;
    localparam int TOKENS=256, HEADS=9, HEAD_DIM=64, TILE=32;
    localparam int V_COUNT=TOKENS*HEAD_DIM;
    logic clk=1'b0,rst_n=1'b0;
    logic v_load_start=1'b0,v_load_ready,v_load_valid=1'b0,v_load_accept;
    logic [15:0] v_load_fp16;
    logic query_start=1'b0,query_ready;
    logic score_valid=1'b0,score_ready;
    logic [TILE*16-1:0] scores_fp16;
    logic out_valid,out_ready=1'b1;
    logic [15:0] out_fp16;
    logic busy,done;
    logic [15:0] score_mem[0:HEADS*TOKENS*TOKENS-1];
    logic [15:0] value_mem[0:HEADS*V_COUNT-1];
    logic [15:0] expected_mem[0:HEADS*TOKENS*HEAD_DIM-1];
    logic [15:0] actual_mem[0:HEADS*TOKENS*HEAD_DIM-1];
    real head_max_error[0:HEADS-1];
    string score_file,value_file,expected_file,actual_file;
    integer head,query,tile,lane,v_idx,out_idx,cycles,mismatches,output_fd;
    real got,expected,abs_error,max_error,max_ref;

    always #5 clk=~clk;
    always_comb begin
        scores_fp16='0;
        for(int l=0;l<TILE;l++)
            scores_fp16[l*16 +:16]=score_mem[
                (head*TOKENS+query)*TOKENS+tile*TILE+l];
    end

    supra2_attention_online_pv dut(
        .clk,.rst_n,.v_load_start,.v_load_ready,.v_load_valid,.v_load_accept,
        .v_load_fp16,.query_start,.query_ready,.score_valid,.score_ready,
        .scores_fp16,.out_valid,.out_ready,.out_fp16,.busy,.done
    );

    function automatic real half_to_real(input logic [15:0] value);
        integer exponent; real significand;
        begin
            exponent=value[14:10];
            if(exponent==0) significand=value[9:0]/1024.0;
            else significand=1.0+value[9:0]/1024.0;
            half_to_real=(value[15]?-1.0:1.0)*significand*
                (exponent==0?(2.0**-14):(2.0**(exponent-15)));
        end
    endfunction

    always @(posedge clk) begin
        if(busy) cycles<=cycles+1;
        if(out_valid&&out_ready) begin
            got=half_to_real(out_fp16);
            expected=half_to_real(expected_mem[
                (head*TOKENS+query)*HEAD_DIM+(out_idx%HEAD_DIM)]);
            if(query==0 && (out_idx%HEAD_DIM)<12)
                $display("ONLINE_PV_SAMPLE head=%0d dim=%0d got=%h/%g exp=%h/%g",
                         head,out_idx%HEAD_DIM,out_fp16,got,
                         expected_mem[(head*TOKENS+query)*HEAD_DIM+
                                      (out_idx%HEAD_DIM)],expected);
            abs_error=(got>=expected)?got-expected:expected-got;
            if(abs_error>max_error) max_error=abs_error;
            if(abs_error>head_max_error[head]) head_max_error[head]=abs_error;
            actual_mem[(head*TOKENS+query)*HEAD_DIM+(out_idx%HEAD_DIM)]=out_fp16;
            if((expected>=0?expected:-expected)>max_ref)
                max_ref=(expected>=0?expected:-expected);
            out_idx<=out_idx+1;
        end
    end

    initial begin
        if(!$value$plusargs("SCORE_FILE=%s",score_file)||
           !$value$plusargs("VALUE_FILE=%s",value_file)||
           !$value$plusargs("EXPECTED_FILE=%s",expected_file)||
           !$value$plusargs("ACTUAL_FILE=%s",actual_file))
            $fatal(1,"Missing online PV fixture arguments");
        $readmemh(score_file,score_mem);
        $readmemh(value_file,value_mem);
        $readmemh(expected_file,expected_mem);
        output_fd=$fopen(actual_file,"w");
        if(!output_fd) $fatal(1,"Cannot open online PV RTL output");
        cycles=0;out_idx=0;mismatches=0;max_error=0.0;max_ref=0.0;
        for(integer h=0;h<HEADS;h++) head_max_error[h]=0.0;
        repeat(4) @(negedge clk);
        rst_n=1'b1;
        for(head=0;head<HEADS;head++) begin
            @(negedge clk); v_load_start=1'b1;
            @(negedge clk); v_load_start=1'b0;
            @(negedge clk);
            while(!v_load_ready) @(negedge clk);
            v_load_valid=1'b1;
            for(v_idx=0;v_idx<V_COUNT;v_idx++) begin
                v_load_fp16=value_mem[head*V_COUNT+v_idx];
                @(negedge clk);
            end
            v_load_valid=1'b0;
            for(query=0;query<TOKENS;query++) begin
                wait(query_ready);
                @(negedge clk); query_start=1'b1;
                @(negedge clk); query_start=1'b0;
                for(tile=0;tile<TOKENS/TILE;tile++) begin
                    wait(score_ready);
                    @(negedge clk); score_valid=1'b1;
                    @(negedge clk); score_valid=1'b0;
                end
                wait(out_idx==((head*TOKENS+query+1)*HEAD_DIM));
            end
            $display("ONLINE_PV_HEAD_MAX_ABS head=%0d error=%.9g",head,head_max_error[head]);
            @(negedge clk);
        end
        $display("SUPRA2_ATTN_ONLINE_PV_PASS queries=%0d heads=%0d keys=%0d cycles=%0d",
                 TOKENS,HEADS,TOKENS,cycles);
        $display("SUPRA2_ATTN_ONLINE_PV_MAX_ABS %.9g",max_error);
        $display("SUPRA2_ATTN_ONLINE_PV_REL_TO_MAX %.9g",max_error/max_ref);
        for(integer q=0;q<TOKENS;q++)
            for(integer h=0;h<HEADS;h++)
                for(integer d=0;d<HEAD_DIM;d++)
                    $fdisplay(output_fd,"%04h",actual_mem[(h*TOKENS+q)*HEAD_DIM+d]);
        $fclose(output_fd);
        if(out_idx!=HEADS*TOKENS*HEAD_DIM||max_error/max_ref>0.01)
            $fatal(1,"Integrated online PV exceeded 1%% relative-to-peak gate");
        $finish;
    end
endmodule

`default_nettype wire
