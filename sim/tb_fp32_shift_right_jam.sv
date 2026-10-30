`timescale 1ns/1ps
module tb_fp32_shift_right_jam;
    import fp16_utils_pkg::*;
    logic [26:0] value, expected, actual;
    logic [31:0] random_state=32'h9284c105;
    integer checked=0;
    initial begin
        for(int vector=0;vector<96;vector++) begin
            random_state={random_state[30:0],random_state[31]^random_state[21]^random_state[1]^random_state[0]};
            if(vector<27) value=27'd1 << vector;
            else if(vector==27) value='0;
            else if(vector==28) value='1;
            else value=random_state[26:0];
            for(int amount=0;amount<256;amount++) begin
                expected='0;
                // Independent bit-level definition: retain shifted bits and
                // OR every bit discarded by the right shift into bit zero.
                for(int bit_index=0;bit_index<27;bit_index++) begin
                    if(bit_index+amount<27) expected[bit_index]=value[bit_index+amount];
                end
                for(int bit_index=0;bit_index<27;bit_index++)
                    if(bit_index<amount) expected[0]=expected[0] | value[bit_index];
                actual=fp32_shift_right_jam(value,8'(amount));
                if(actual!==expected)
                    $fatal(1,"shift_right_jam value=%h amount=%0d got=%h expected=%h",value,amount,actual,expected);
                checked++;
            end
        end
        $display("FP32_SHIFT_RIGHT_JAM_PASS cases=%0d",checked);
        $finish;
    end
endmodule
