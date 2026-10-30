`default_nettype none

module tb_supra2_layernorm_32lane;
    localparam int ROWS=32;
    localparam int FEATURES=576;
    localparam int KGROUP=32;
    localparam int BEATS=FEATURES/KGROUP;   // 18
    localparam int OUT_BEATS=ROWS*BEATS;

    logic clk=1'b0,rst_n=1'b0;
    logic wr_en=1'b0, wr_ss_en=1'b0, wr_ss_sel=1'b0;
    logic [4:0] wr_row;
    logic [4:0] wr_beat, wr_ss_beat;
    logic [511:0] wr_data, wr_ss_data;
    logic start=1'b0,busy,done;
    logic out_valid;
    logic [4:0] out_row, out_beat;
    logic [511:0] out_data;

    logic [15:0] x_mem [0:ROWS*FEATURES-1];
    logic [15:0] shift_mem [0:FEATURES-1];
    logic [15:0] scale_mem [0:FEATURES-1];
    logic [15:0] expected_mem [0:ROWS*FEATURES-1];
    string x_file,shift_file,scale_file,expected_file,actual_file;
    integer fills,out_count,cycles,fd;
    real got,expected,error,max_error,max_ref;

    always #5 clk=~clk;
    supra2_layernorm_modulate_32lane #(.ROWS(ROWS),.FEATURES(FEATURES)) dut(
        .clk,.rst_n,.wr_en,.wr_row,.wr_beat,.wr_data,
        .wr_ss_en,.wr_ss_sel,.wr_ss_beat,.wr_ss_data,
        .start,.busy,.done,
        .out_valid,.out_row,.out_beat,.out_data);

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
        if(out_valid) begin
            for(int g=0; g<KGROUP; g++) begin
                got=half_to_real(out_data[g*16 +: 16]);
                expected=half_to_real(expected_mem[out_row*FEATURES+out_beat*KGROUP+g]);
                error=(got>=expected)?got-expected:expected-got;
                if(error>max_error) max_error=error;
                if((expected>=0?expected:-expected)>max_ref)
                    max_ref=(expected>=0?expected:-expected);
                $fdisplay(fd,"%04h",out_data[g*16 +: 16]);
            end
            out_count<=out_count+1;
        end
        if(busy) cycles<=cycles+1;
    end

    initial begin
        if(!$value$plusargs("X_FILE=%s",x_file)||
           !$value$plusargs("SHIFT_FILE=%s",shift_file)||
           !$value$plusargs("SCALE_FILE=%s",scale_file)||
           !$value$plusargs("EXPECTED_FILE=%s",expected_file)||
           !$value$plusargs("ACTUAL_FILE=%s",actual_file))
            $fatal(1,"Missing layernorm_32lane fixture plusargs");
        $readmemh(x_file,x_mem);
        $readmemh(shift_file,shift_mem);
        $readmemh(scale_file,scale_mem);
        $readmemh(expected_file,expected_mem);
        fd=$fopen(actual_file,"w");
        if(!fd) $fatal(1,"Cannot open layernorm_32lane RTL output");
        fills=0;out_count=0;cycles=0;max_error=0.0;max_ref=0.0;
        repeat(4) @(negedge clk);
        rst_n=1'b1;
        repeat(2) @(negedge clk);
        // fill x: 32 rows x 18 beats
        while(fills<ROWS*BEATS) begin
            @(negedge clk);
            wr_en=1'b1; wr_row=fills/BEATS; wr_beat=fills%BEATS;
            for(int g=0; g<KGROUP; g++)
                wr_data[g*16 +: 16]=x_mem[(fills/BEATS)*FEATURES+(fills%BEATS)*KGROUP+g];
            fills=fills+1;
        end
        @(negedge clk); wr_en=1'b0;
        // fill shift then scale: 18 beats each
        for(int b=0; b<BEATS; b++) begin
            @(negedge clk);
            wr_ss_en=1'b1; wr_ss_sel=1'b0; wr_ss_beat=b;
            for(int g=0; g<KGROUP; g++) wr_ss_data[g*16 +: 16]=shift_mem[b*KGROUP+g];
        end
        for(int b=0; b<BEATS; b++) begin
            @(negedge clk);
            wr_ss_en=1'b1; wr_ss_sel=1'b1; wr_ss_beat=b;
            for(int g=0; g<KGROUP; g++) wr_ss_data[g*16 +: 16]=scale_mem[b*KGROUP+g];
        end
        @(negedge clk); wr_ss_en=1'b0;
        @(negedge clk); start=1'b1;
        @(negedge clk); start=1'b0;
        wait(done);
        @(negedge clk);
        if(out_count!=OUT_BEATS)
            $fatal(1,"layernorm_32lane emitted %0d beats, expected %0d",out_count,OUT_BEATS);
        $fclose(fd);
        $display("SUPRA2_LAYERNORM_32LANE_PASS rows=%0d features=%0d cycles=%0d",
                 ROWS,FEATURES,cycles);
        $display("SUPRA2_LAYERNORM_32LANE_MAX_ABS %.9g",max_error);
        $display("SUPRA2_LAYERNORM_32LANE_REL_TO_PEAK %.9g",max_error/max_ref);
        if(max_error/max_ref>0.01)
            $fatal(1,"layernorm_32lane exceeded 1%% relative-to-peak error");
        $finish;
    end
endmodule

`default_nettype wire
