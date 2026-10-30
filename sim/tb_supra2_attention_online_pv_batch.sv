`default_nettype none

module tb_supra2_attention_online_pv_batch #(
    parameter int KEYS=256,
    parameter int VALID_KEYS=KEYS,
    parameter bit EXTERNAL_ARRAY=1'b0
);
    import fp16_utils_pkg::*;
    localparam int QUERY_TOKENS=256,HEADS=9,HEAD_DIM=64,TILE=32;
    localparam int V_COUNT=KEYS*HEAD_DIM;
    logic clk=1'b0,rst_n=1'b0;
    logic v_load_start=1'b0,v_load_ready,v_load_valid=1'b0,v_load_accept;
    logic [15:0] v_load_fp16;
    logic batch_start=1'b0,batch_ready;
    logic score_valid=1'b0,score_ready;
    logic [TILE*16-1:0] scores_fp16;
    logic out_valid,out_ready=1'b1;
    logic [15:0] out_fp16;
    logic busy,done;
    logic array_clear,array_en;
    logic signed [7:0] array_a_col[32],array_b_row[32];
    wire signed [31:0] external_acc[32][32];
    wire external_acc_valid;
    logic [15:0] score_mem[0:HEADS*QUERY_TOKENS*KEYS-1];
    logic [15:0] value_mem[0:HEADS*V_COUNT-1];
    logic [15:0] expected_mem[0:HEADS*QUERY_TOKENS*HEAD_DIM-1];
    logic [15:0] actual_mem[0:HEADS*QUERY_TOKENS*HEAD_DIM-1];
    real head_max_error[0:HEADS-1];
    string score_file,value_file,expected_file,actual_file;
    integer head,group_idx,key_tile,row,lane,v_idx,out_idx,out_batch_idx;
    integer batch_base,cycles,output_fd;
    real got,expected,abs_error,max_error,max_ref;

    always #5 clk=~clk;
    always_comb begin
        scores_fp16='0;
        for(int l=0;l<TILE;l++)
            scores_fp16[l*16 +:16]=score_mem[
                (head*QUERY_TOKENS+batch_base%QUERY_TOKENS+row)*KEYS+
                key_tile*TILE+l];
    end

    supra2_attention_online_pv_batch #(
        .TOKENS(KEYS),.EXTERNAL_ARRAY(EXTERNAL_ARRAY)) dut(
        .clk,.rst_n,.v_load_start,.v_load_ready,.v_load_valid,.v_load_accept,
        .valid_keys(9'(VALID_KEYS)),
        .v_load_fp16,.batch_start,.batch_ready,.score_valid,.score_ready,
        .scores_fp16,.out_valid,.out_ready,.out_fp16,.busy,.done,
        .array_clear,.array_en,.array_a_col,.array_b_row,
        .external_array_acc(external_acc),
        .external_array_acc_valid(external_acc_valid)
    );

    generate if (EXTERNAL_ARRAY) begin : g_runtime_unsigned_array
        systolic_array #(.M(32),.N(32),.DATA_W(8),.ACC_W(32),
                         .UNSIGNED_A(1'b0),.RUNTIME_UNSIGNED_A(1'b1),
                         .PIPE_STAGE(32)) u_array (
            .clk(clk),.rst_n(rst_n),.clear_acc(array_clear),.en(array_en),
            .a_unsigned(1'b1),.active_rows(6'd32),
            .a_col(array_a_col),.b_row(array_b_row),
            .acc_out(external_acc),.acc_valid(external_acc_valid)
        );
    end else begin : g_no_external_array
        for(genvar ai=0;ai<32;ai++) begin : g_acc_rows
            for(genvar aj=0;aj<32;aj++) begin : g_acc_cols
                assign external_acc[ai][aj]='0;
            end
        end
        assign external_acc_valid=1'b0;
    end endgenerate

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
                (batch_base+out_batch_idx/HEAD_DIM)*HEAD_DIM+
                (out_batch_idx%HEAD_DIM)]);
            abs_error=(got>=expected)?got-expected:expected-got;
            if(abs_error>max_error) max_error=abs_error;
            if(abs_error>head_max_error[head]) head_max_error[head]=abs_error;
            if((expected>=0?expected:-expected)>max_ref)
                max_ref=(expected>=0?expected:-expected);
            actual_mem[(batch_base+out_batch_idx/HEAD_DIM)*HEAD_DIM+
                       (out_batch_idx%HEAD_DIM)]=out_fp16;
            out_idx<=out_idx+1;
            out_batch_idx<=out_batch_idx+1;
        end
    end

    initial begin
        if(!$value$plusargs("SCORE_FILE=%s",score_file)||
           !$value$plusargs("VALUE_FILE=%s",value_file)||
           !$value$plusargs("EXPECTED_FILE=%s",expected_file)||
           !$value$plusargs("ACTUAL_FILE=%s",actual_file))
            $fatal(1,"Missing batched online PV fixture arguments");
        $readmemh(score_file,score_mem);
        $readmemh(value_file,value_mem);
        $readmemh(expected_file,expected_mem);
        output_fd=$fopen(actual_file,"w");
        if(!output_fd) $fatal(1,"Cannot open batched online PV RTL output");
        cycles=0;out_idx=0;out_batch_idx=0;max_error=0.0;max_ref=0.0;
        for(integer h=0;h<HEADS;h++) head_max_error[h]=0.0;
        repeat(4) @(negedge clk);
        rst_n=1'b1;
        for(head=0;head<HEADS;head++) begin
            @(negedge clk);v_load_start=1'b1;
            @(negedge clk);v_load_start=1'b0;
            while(!v_load_ready) @(negedge clk);
            v_load_valid=1'b1;
            for(v_idx=0;v_idx<V_COUNT;v_idx++) begin
                v_load_fp16=value_mem[head*V_COUNT+v_idx];
                @(negedge clk);
            end
            v_load_valid=1'b0;
            if(head==0) begin
                for(int q=0;q<V_COUNT;q++) begin
                    int feature,key,bank,addr;
                    feature=q/KEYS; key=q%KEYS; bank=feature%TILE;
                    addr=(feature/TILE)*KEYS+key;
                    if(dut.v_mem[bank][addr] !==
                       fp16_to_int8_q(value_mem[q],4))
                        $fatal(1,"PV V-bank load mismatch q=%0d feature=%0d key=%0d",
                               q,feature,key);
                end
            end
            for(group_idx=0;group_idx<QUERY_TOKENS/TILE;group_idx++) begin
                batch_base=head*QUERY_TOKENS+group_idx*TILE;
                out_batch_idx=0;
                wait(batch_ready);
                @(negedge clk);batch_start=1'b1;
                @(negedge clk);batch_start=1'b0;
                for(key_tile=0;key_tile<KEYS/TILE;key_tile++)
                    for(row=0;row<TILE;row++) begin
                        wait(score_ready);
                        @(negedge clk);score_valid=1'b1;
                        @(negedge clk);score_valid=1'b0;
                    end
                wait(out_idx==((batch_base+TILE)*HEAD_DIM));
            end
            $display("BATCH_ONLINE_PV_HEAD_MAX_ABS head=%0d error=%.9g",
                     head,head_max_error[head]);
            @(negedge clk);
        end
        $display("SUPRA2_ATTN_ONLINE_PV_BATCH_PASS queries=%0d heads=%0d keys=%0d cycles=%0d",
                 QUERY_TOKENS,HEADS,KEYS,cycles);
        $display("SUPRA2_ATTN_ONLINE_PV_BATCH_MAX_ABS %.9g",max_error);
        $display("SUPRA2_ATTN_ONLINE_PV_BATCH_REL_TO_MAX %.9g",max_error/max_ref);
        for(integer h=0;h<HEADS;h++)
            for(integer q=0;q<QUERY_TOKENS;q++)
                for(integer d=0;d<HEAD_DIM;d++)
                    $fdisplay(output_fd,"%04h",actual_mem[(h*QUERY_TOKENS+q)*HEAD_DIM+d]);
        $fclose(output_fd);
        if(out_idx!=HEADS*QUERY_TOKENS*HEAD_DIM||max_error/max_ref>0.01)
            $fatal(1,"Batched online PV exceeded 1%% relative-to-peak gate");
        $finish;
    end
endmodule

`default_nettype wire
