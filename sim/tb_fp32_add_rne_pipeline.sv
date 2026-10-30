`timescale 1ns/1ps
`default_nettype none

module tb_fp32_add_rne_pipeline;
    import fp16_utils_pkg::*;
    localparam int CASES=20000;
    logic clk=0, rst_n=0, in_valid=0, out_valid;
    logic [31:0] a=0,b=0,result;
    logic [31:0] expected[0:CASES-1];
    int sent=0, received=0;
    logic [31:0] rng=32'hd831a5c7;
    always #1.25 clk=~clk;

    supra2_fp32_add_rne_pipeline dut(.*);

    function automatic logic [31:0] random_word;
        rng={rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};
        return rng;
    endfunction

    initial begin
        repeat(4) @(negedge clk);
        rst_n=1;
        for(int i=0;i<CASES;i++) begin
            @(negedge clk);
            in_valid=1;
            case(i)
                0: begin a=0; b=0; end
                1: begin a=32'h3f800000; b=32'h3f800000; end
                2: begin a=32'h3f800000; b=32'hbf800000; end
                3: begin a=32'h00000001; b=32'h00000001; end
                4: begin a=32'h7f7fffff; b=32'h7f7fffff; end
                5: begin a=32'hff800000; b=32'h7f800000; end
                default: begin a=random_word(); b=random_word(); end
            endcase
            expected[i]=fp32_add_rne(a,b);
            sent++;
        end
        @(negedge clk); in_valid=0;
        wait(received==CASES);
        $display("FP32_ADD_RNE_PIPELINE_PASS cases=%0d latency=8 ii=1",received);
        $finish;
    end

    always @(posedge clk) begin
        if(rst_n && out_valid) begin
            if(received>=sent || result!==expected[received])
                $fatal(1,"FP32 pipeline mismatch index=%0d got=%h expected=%h",
                    received,result,expected[received]);
            received++;
        end
    end

    initial begin
        #1000000;
        $fatal(1,"FP32 pipeline regression timeout sent=%0d received=%0d",sent,received);
    end
endmodule

`default_nettype wire
