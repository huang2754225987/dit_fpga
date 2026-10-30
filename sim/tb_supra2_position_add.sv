`default_nettype none

module tb_supra2_position_add;
    localparam int ELEMENTS = 256 * 576;
    logic clk=1'b0, rst_n=1'b0;
    logic in_valid=1'b0, in_ready;
    logic [15:0] token_fp16, position_fp16;
    logic out_valid, out_ready=1'b1;
    logic [15:0] out_fp16;
    logic [15:0] token_mem[0:ELEMENTS-1];
    logic [15:0] position_mem[0:ELEMENTS-1];
    logic [15:0] expected_mem[0:ELEMENTS-1];
    string token_file, position_file, expected_file, actual_file;
    integer input_index, output_index, cycles, actual_fd;

    always #5 clk=~clk;
    supra2_position_add dut(.*);

    always @(posedge clk) begin
        if (in_valid && in_ready) cycles <= cycles + 1;
        if (out_valid && out_ready) begin
            if (out_fp16 !== expected_mem[output_index])
                $fatal(1,"Position-add mismatch at %0d: got %h expected %h",
                       output_index,out_fp16,expected_mem[output_index]);
            if (actual_fd) $fdisplay(actual_fd,"%04h",out_fp16);
            output_index <= output_index + 1;
        end
    end

    initial begin
        if (fp16_utils_pkg::fp16_add(16'hB826,16'h37B9) !== 16'hA898)
            $fatal(1,"FP16 subtractive normalization regression failed");
        if (!$value$plusargs("TOKEN_FILE=%s",token_file) ||
            !$value$plusargs("POSITION_FILE=%s",position_file) ||
            !$value$plusargs("EXPECTED_FILE=%s",expected_file))
            $fatal(1,"Missing position-add fixture plusargs");
        $readmemh(token_file,token_mem);
        $readmemh(position_file,position_mem);
        $readmemh(expected_file,expected_mem);
        actual_fd=0;
        if ($value$plusargs("ACTUAL_FILE=%s",actual_file)) begin
            actual_fd=$fopen(actual_file,"w");
            if (!actual_fd) $fatal(1,"Cannot open position-add RTL output");
        end
        input_index=0; output_index=0; cycles=0;
        repeat(4) @(negedge clk);
        rst_n=1'b1;
        while (input_index<ELEMENTS) begin
            @(negedge clk);
            if (in_ready) begin
                in_valid=1'b1;
                token_fp16=token_mem[input_index];
                position_fp16=position_mem[input_index];
                input_index=input_index+1;
            end else in_valid=1'b0;
        end
        @(negedge clk);
        in_valid=1'b0;
        wait(output_index==ELEMENTS);
        @(negedge clk);
        $display("SUPRA2_POSITION_ADD_PASS elements=%0d cycles=%0d",ELEMENTS,cycles);
        if (actual_fd) $fclose(actual_fd);
        $finish;
    end
endmodule

`default_nettype wire
