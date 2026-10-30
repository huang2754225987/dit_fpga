`default_nettype none

module tb_supra2_softmax_2pass;
    localparam int ROWS=32;
    localparam int KEYS=256;
    localparam int COUNT=ROWS*KEYS;
    logic clk=1'b0,rst_n=1'b0;
    logic wr_en=1'b0;
    logic [4:0] wr_row;
    logic [2:0] wr_beat;
    logic [511:0] wr_data;
    logic start=1'b0,busy,done;
    logic out_valid;
    logic [$clog2(KEYS)-1:0] out_col;
    logic [ROWS*16-1:0] out_data;

    logic [15:0] score_mem [0:COUNT-1];
    logic [15:0] expected_mem [0:COUNT-1];
    string scores_file,expected_file,actual_file;
    integer beats_in,cols_out,cycles,fd;
    real got,expected,error,max_error,max_ref;

    always #5 clk=~clk;
    supra2_softmax_2pass #(.ROWS(ROWS),.KEYS(KEYS)) dut(
        .clk,.rst_n,.wr_en,.wr_row,.wr_beat,.wr_data,
        .start,.busy,.done,.out_valid,.out_col,.out_data);

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

    // fill: 32 beats per row (256 keys, 32 per beat); score_mem is row-major
    task automatic fill_beat(input integer row, input integer beat);
        begin
            wr_en=1'b1; wr_row=row[4:0]; wr_beat=beat[2:0];
            for(int g=0; g<32; g++)
                wr_data[g*16 +: 16]=score_mem[row*KEYS+beat*32+g];
        end
    endtask

    // capture: out streams one key column per beat, row i at out_data[i*16 +: 16]
    always @(posedge clk) begin
        if(out_valid) begin
            for(int i=0; i<ROWS; i++) begin
                got=half_to_real(out_data[i*16 +: 16]);
                expected=half_to_real(expected_mem[i*KEYS+out_col]);
                error=(got>=expected)?got-expected:expected-got;
                if(error>max_error) max_error=error;
                if((expected>=0?expected:-expected)>max_ref)
                    max_ref=(expected>=0?expected:-expected);
                $fdisplay(fd,"%04h",out_data[i*16 +: 16]);
            end
            cols_out<=cols_out+1;
        end
        if(busy) cycles<=cycles+1;
    end

    initial begin
        if(!$value$plusargs("SCORES_FILE=%s",scores_file)||
           !$value$plusargs("EXPECTED_FILE=%s",expected_file)||
           !$value$plusargs("ACTUAL_FILE=%s",actual_file))
            $fatal(1,"Missing softmax_2pass fixture plusargs");
        $readmemh(scores_file,score_mem);
        $readmemh(expected_file,expected_mem);
        fd=$fopen(actual_file,"w");
        if(!fd) $fatal(1,"Cannot open softmax_2pass RTL output");
        beats_in=0;cols_out=0;cycles=0;max_error=0.0;max_ref=0.0;
        repeat(4) @(negedge clk);
        rst_n=1'b1;
        repeat(2) @(negedge clk);
        while(beats_in<ROWS*(KEYS/32)) begin
            @(negedge clk);
            fill_beat(beats_in/(KEYS/32), beats_in%(KEYS/32));
            beats_in=beats_in+1;
        end
        @(negedge clk); wr_en=1'b0;
        @(negedge clk); start=1'b1;
        @(negedge clk); start=1'b0;
        wait(done);
        @(negedge clk);
        if(cols_out!=KEYS) $fatal(1,"softmax_2pass emitted %0d columns, expected %0d",cols_out,KEYS);
        $fclose(fd);
        $display("SUPRA2_SOFTMAX_2PASS rows=%0d keys=%0d cycles=%0d",ROWS,KEYS,cycles);
        $display("SUPRA2_SOFTMAX_2PASS_MAX_ABS %.9g",max_error);
        $display("SUPRA2_SOFTMAX_2PASS_REL_TO_PEAK %.9g",max_error/max_ref);
        if(max_error/max_ref>0.01)
            $fatal(1,"softmax_2pass exceeded 1%% relative-to-peak error");
        $finish;
    end
endmodule

`default_nettype wire
