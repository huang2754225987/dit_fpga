`default_nettype none

module tb_supra2_gelu_tanh;
    localparam int COUNT=256*2304;
    logic clk=1'b0,rst_n=1'b0,in_valid=1'b0,in_ready;
    logic [15:0] in_fp16;
    logic out_valid,out_ready=1'b1;
    logic [15:0] out_fp16;
    logic [15:0] input_mem[0:COUNT-1];
    logic [15:0] expected_mem[0:COUNT-1];
    string input_file,expected_file,actual_file;
    integer input_index,output_index,cycles,fd;
    real got,expected,error,max_error,max_ref;

    always #5 clk=~clk;
    supra2_gelu_tanh dut(.clk,.rst_n,.in_valid,.in_ready,.in_fp16,
                         .out_valid,.out_ready,.out_fp16);

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
        if(in_valid&&in_ready) cycles<=cycles+1;
        if(out_valid&&out_ready) begin
            got=half_to_real(out_fp16);
            expected=half_to_real(expected_mem[output_index]);
            error=(got>=expected)?got-expected:expected-got;
            if(error>max_error) max_error=error;
            if((expected>=0?expected:-expected)>max_ref)
                max_ref=(expected>=0?expected:-expected);
            $fdisplay(fd,"%04h",out_fp16);
            output_index<=output_index+1;
        end
    end

    initial begin
        if(!$value$plusargs("INPUT_FILE=%s",input_file)||
           !$value$plusargs("EXPECTED_FILE=%s",expected_file)||
           !$value$plusargs("ACTUAL_FILE=%s",actual_file))
            $fatal(1,"Missing GELU fixture plusargs");
        $readmemh(input_file,input_mem);
        $readmemh(expected_file,expected_mem);
        fd=$fopen(actual_file,"w");
        if(!fd) $fatal(1,"Cannot open GELU RTL output");
        input_index=0;output_index=0;cycles=0;max_error=0.0;max_ref=0.0;
        repeat(4) @(negedge clk);
        rst_n=1'b1;
        while(input_index<COUNT) begin
            @(negedge clk);
            if(in_ready) begin
                in_valid=1'b1;in_fp16=input_mem[input_index];input_index=input_index+1;
            end else in_valid=1'b0;
        end
        @(negedge clk);in_valid=1'b0;
        wait(output_index==COUNT);
        @(negedge clk);
        $fclose(fd);
        $display("SUPRA2_GELU_TANH_PASS elements=%0d cycles=%0d",COUNT,cycles);
        $display("SUPRA2_GELU_TANH_MAX_ABS %.9g",max_error);
        $display("SUPRA2_GELU_TANH_REL_TO_MAX %.9g",max_error/max_ref);
        if(max_error/max_ref>0.003)
            $fatal(1,"GELU(tanh) ROM exceeded 0.3%% relative-to-peak error");
        $finish;
    end
endmodule

`default_nettype wire
