`default_nettype none
module tb_supra2_final_head_datapath;
    localparam int TOKENS=256, WIDTH=576, LANES=32, OUT=16;
    logic clk=0, rst_n=0, act_wr_en=0, shift_wr_en=0, scale_wr_en=0;
    logic [15:0] act_wr_beat=0;
    logic [511:0] act_wr_data='0, shift_wr_data='0, scale_wr_data='0;
    logic [4:0] shift_wr_beat=0, scale_wr_beat=0, bias_wr_addr=0;
    logic weight_wr_en=0, bias_wr_en=0;
    logic [15:0] weight_wr_beat=0, bias_wr_data=0;
    logic [511:0] weight_wr_data='0;
    logic start=0, busy, done, out_valid, out_ready=1, out_last;
    logic [511:0] out_data;
    logic [15:0] act_mem[0:TOKENS*WIDTH-1];
    logic [15:0] shift_mem[0:WIDTH-1], scale_mem[0:WIDTH-1];
    logic [15:0] weight_mem[0:OUT*WIDTH-1], bias_mem[0:OUT-1];
    logic [15:0] expected[0:4095];
    integer received=0, cycles=0;
    real max_error=0.0, peak=0.0, got_value, exp_value, delta;
    string root="build/supra2_final_head/";

    always #5 clk=~clk;
    supra2_final_head_datapath dut(.*);

    function automatic real half_real(input logic [15:0] h);
        integer e,m; real v;
        begin
            e=h[14:10]; m=h[9:0];
            if(e==0) v=m/1024.0*(2.0**-14);
            else if(e==31) v=0.0;
            else v=(1.0+m/1024.0)*(2.0**(e-15));
            return h[15] ? -v : v;
        end
    endfunction

    initial begin
        $readmemh({root,"final_norm_x_fp16.hex"},act_mem);
        $readmemh({root,"final_shift_fp16.hex"},shift_mem);
        $readmemh({root,"final_scale_fp16.hex"},scale_mem);
        $readmemh({root,"linear_weight_fp16.hex"},weight_mem);
        $readmemh({root,"linear_bias_fp16.hex"},bias_mem);
        $readmemh({root,"reference_latent_fp16.hex"},expected);
        repeat(5) @(negedge clk); rst_n=1;
        for(int b=0;b<TOKENS*WIDTH/LANES;b++) begin
            for(int l=0;l<LANES;l++) act_wr_data[l*16+:16]=act_mem[b*LANES+l];
            act_wr_beat=b; act_wr_en=1; @(negedge clk);
        end
        act_wr_en=0;
        for(int b=0;b<WIDTH/LANES;b++) begin
            for(int l=0;l<LANES;l++) begin
                shift_wr_data[l*16+:16]=shift_mem[b*LANES+l];
                scale_wr_data[l*16+:16]=scale_mem[b*LANES+l];
            end
            shift_wr_beat=b; scale_wr_beat=b;
            shift_wr_en=1; scale_wr_en=1; @(negedge clk);
        end
        shift_wr_en=0; scale_wr_en=0;
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
        while(received<128 && cycles<50000) begin
            out_ready=(cycles%17 != 8);
            @(negedge clk); cycles++;
            if(out_valid && out_ready) begin
                for(int l=0;l<LANES;l++) begin
                    got_value=half_real(out_data[l*16+:16]);
                    exp_value=half_real(expected[received*LANES+l]);
                    delta=(got_value>=exp_value)?got_value-exp_value:exp_value-got_value;
                    if(delta>max_error) max_error=delta;
                    if((exp_value>=0.0?exp_value:-exp_value)>peak)
                        peak=(exp_value>=0.0?exp_value:-exp_value);
                end
                if(out_last !== (received==127)) $fatal(1,"out_last mismatch beat=%0d",received);
                received++;
            end
        end
        if(received!=128 || cycles>=50000 || peak==0.0 || max_error/peak>0.01)
            $fatal(1,"final-head datapath failed beats=%0d rel_peak=%e cycles=%0d",received,max_error/peak,cycles);
        wait(done);
        $display("SUPRA2_FINAL_HEAD_DATAPATH_PASS beats=%0d cycles=%0d rel_peak=%e",received,cycles,max_error/peak);
        $finish;
    end
endmodule
`default_nettype wire
