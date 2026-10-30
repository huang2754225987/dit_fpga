`default_nettype none
module tb_supra2_final_head_projection;
    localparam int TOKENS=256, WIDTH=576, LANES=32, OUT=16;
    logic clk=0, rst_n=0, act_wr_en=0, weight_wr_en=0, bias_wr_en=0;
    logic [15:0] act_wr_beat=0, weight_wr_beat=0;
    logic [511:0] act_wr_data='0, weight_wr_data='0;
    logic [4:0] bias_wr_addr=0;
    logic [15:0] bias_wr_data=0;
    logic start=0, busy, done, out_valid, out_ready=1, out_last;
    logic [511:0] out_data;
    logic [15:0] act_mem[0:TOKENS*WIDTH-1];
    logic [15:0] weight_mem[0:LANES*WIDTH-1];
    logic [15:0] bias_mem[0:OUT-1];
    logic [15:0] expected[0:4095];
    logic [15:0] array_tokens[0:TOKENS*OUT-1];
    integer errors=0, received=0, cycles=0;
    integer dump_fd;
    real max_error=0.0, peak=0.0, got_value, exp_value, delta;
    string root="build/supra2_final_head/";

    always #5 clk=~clk;
    supra2_final_head_projection dut(.*);

    function automatic real half_real(input logic [15:0] h);
        integer e, m;
        real v;
        begin
            e=h[14:10]; m=h[9:0];
            if(e==0) v=m/1024.0*(2.0**-14);
            else if(e==31) v=0.0;
            else v=(1.0+m/1024.0)*(2.0**(e-15));
            return h[15] ? -v : v;
        end
    endfunction

    initial begin
        $readmemh({root,"normalized_modulated_fp16.hex"},act_mem);
        $readmemh({root,"linear_weight_fp16.hex"},weight_mem);
        $readmemh({root,"linear_bias_fp16.hex"},bias_mem);
        $readmemh({root,"reference_latent_fp16.hex"},expected);
        $readmemh({root,"rtl_tokens_fp16.hex"},array_tokens);
        repeat(5) @(negedge clk); rst_n=1;
        for(int b=0;b<TOKENS*WIDTH/LANES;b++) begin
            for(int l=0;l<LANES;l++) act_wr_data[l*16+:16]=act_mem[b*LANES+l];
            act_wr_beat=b; act_wr_en=1; @(negedge clk);
        end
        act_wr_en=0;
        for(int b=0;b<LANES*WIDTH/LANES;b++) begin
            for(int l=0;l<LANES;l++) begin
                int index;
                index=b*LANES+l;
                weight_wr_data[l*16+:16]=(index<OUT*WIDTH)?weight_mem[index]:16'h0000;
            end
            weight_wr_beat=b; weight_wr_en=1; @(negedge clk);
        end
        weight_wr_en=0;
        for(int j=0;j<OUT;j++) begin
            bias_wr_addr=j; bias_wr_data=bias_mem[j]; bias_wr_en=1; @(negedge clk);
        end
        bias_wr_en=0; start=1; @(negedge clk); start=0;
        for(int q=0;q<TOKENS*WIDTH;q++)
            if(dut.u_engine.f16_act[q] !== act_mem[q]) $fatal(1,"activation load mismatch at %0d",q);
        for(int q=0;q<LANES*WIDTH;q++) begin
            logic [15:0] expected_weight;
            expected_weight=(q<OUT*WIDTH)?weight_mem[q]:16'h0000;
            if(dut.u_engine.f16_wt[q] !== expected_weight) $fatal(1,"weight load mismatch at %0d",q);
        end
        while(received<128 && cycles<400000) begin
            @(negedge clk); cycles++;
            if(out_valid && out_ready) begin
                for(int l=0;l<LANES;l++) begin
                    int idx, x, y, c, tok, patch;
                    got_value=half_real(out_data[l*16+:16]);
                    exp_value=half_real(expected[received*LANES+l]);
                    delta=(got_value>=exp_value)?got_value-exp_value:exp_value-got_value;
                    if(delta>max_error) max_error=delta;
                    if((exp_value>=0.0?exp_value:-exp_value)>peak)
                        peak=(exp_value>=0.0?exp_value:-exp_value);
                    idx=received*LANES+l; x=idx%32; y=(idx/32)%32; c=idx/1024;
                    tok=(y/2)*16+x/2; patch=c*4+(y%2)*2+x%2;
                    if(out_data[l*16+:16] !== array_tokens[tok*OUT+patch]) errors++;
                end
                if(out_last !== (received==127)) $fatal(1,"out_last mismatch beat=%0d",received);
                received++;
            end
        end
        dump_fd=$fopen({root,"projection_tokens_fp16.hex"},"w");
        if(dump_fd==0) $fatal(1,"Cannot open projection token dump");
        for(int r=0;r<TOKENS;r++)
            for(int j=0;j<OUT;j++) $fdisplay(dump_fd,"%04h",dut.token_mem[r*LANES+j]);
        $fclose(dump_fd);
        if(received!=128 || cycles>=400000 || peak==0.0 || errors!=0)
            $fatal(1,"projection/base mismatch beats=%0d bit_errors=%0d cycles=%0d",received,errors,cycles);
        wait(done); $display("SUPRA2_FINAL_HEAD_PROJECTION_PASS beats=%0d cycles=%0d array_bit_mismatches=%0d fp16_reference_rel_peak=%e",received,cycles,errors,max_error/peak); $finish;
    end
endmodule
`default_nettype wire
