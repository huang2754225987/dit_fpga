`default_nettype none

module tb_supra2_residual_gate;
    localparam int FEATURES=576, ROWS=256, ELEMENTS=FEATURES*ROWS;
    logic clk=1'b0, rst_n=1'b0;
    logic in_valid=1'b0, in_ready;
    logic [15:0] residual_fp16, update_fp16, gate_fp16;
    logic out_valid, out_ready=1'b1;
    logic [15:0] out_fp16;
    logic [15:0] residual_mem[0:ELEMENTS-1];
    logic [15:0] update_mem[0:ELEMENTS-1];
    logic [15:0] gate_mem[0:FEATURES-1];
    logic [15:0] expected_mem[0:ELEMENTS-1];
    string residual_file, update_file, gate_file, expected_file, actual_file;
    integer input_index, output_index, cycles, actual_fd;
    real got, expected, abs_error, max_error, max_ref;

    always #5 clk=~clk;
    supra2_residual_gate dut(
        .clk,.rst_n,.in_valid,.in_ready,.residual_fp16,.update_fp16,
        .gate_fp16,.out_valid,.out_ready,.out_fp16
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

    always @(posedge clk) begin
        if (in_valid && in_ready) cycles<=cycles+1;
        if (out_valid && out_ready) begin
            got=half_to_real(out_fp16);
            expected=half_to_real(expected_mem[output_index]);
            if (actual_fd) $fdisplay(actual_fd,"%04h",out_fp16);
            abs_error=(got>=expected)?got-expected:expected-got;
            if (abs_error>max_error) max_error=abs_error;
            if ((expected>=0?expected:-expected)>max_ref)
                max_ref=(expected>=0?expected:-expected);
            output_index<=output_index+1;
        end
    end

    initial begin
        if (!$value$plusargs("RESIDUAL_FILE=%s",residual_file) ||
            !$value$plusargs("UPDATE_FILE=%s",update_file) ||
            !$value$plusargs("GATE_FILE=%s",gate_file) ||
            !$value$plusargs("EXPECTED_FILE=%s",expected_file))
            $fatal(1,"Missing residual fixture plusargs");
        $readmemh(residual_file,residual_mem);
        $readmemh(update_file,update_mem);
        $readmemh(gate_file,gate_mem);
        $readmemh(expected_file,expected_mem);
        actual_fd=0;
        if ($value$plusargs("ACTUAL_FILE=%s",actual_file)) begin
            actual_fd=$fopen(actual_file,"w");
            if (!actual_fd) $fatal(1,"Cannot open residual RTL output file");
        end
        input_index=0; output_index=0; cycles=0; max_error=0.0; max_ref=0.0;
        repeat(4) @(negedge clk);
        rst_n=1'b1;
        while (input_index<ELEMENTS) begin
            @(negedge clk);
            if (in_ready) begin
                in_valid=1'b1;
                residual_fp16=residual_mem[input_index];
                update_fp16=update_mem[input_index];
                gate_fp16=gate_mem[input_index%FEATURES];
                input_index=input_index+1;
            end else in_valid=1'b0;
        end
        @(negedge clk);
        in_valid=1'b0;
        wait(output_index==ELEMENTS);
        @(negedge clk);
        $display("SUPRA2_RESIDUAL_GATE_PASS rows=%0d features=%0d cycles=%0d",
                 ROWS,FEATURES,cycles);
        $display("SUPRA2_RESIDUAL_GATE_MAX_ABS %.9g",max_error);
        $display("SUPRA2_RESIDUAL_GATE_REL_TO_MAX %.9g",max_error/max_ref);
        if (actual_fd) $fclose(actual_fd);
        if (max_error/max_ref>0.005)
            $fatal(1,"Residual gate exceeded 0.5%% relative-to-peak gate");
        $finish;
    end
endmodule

`default_nettype wire
