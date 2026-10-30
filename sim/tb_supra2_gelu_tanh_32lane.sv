`default_nettype none

module tb_supra2_gelu_tanh_32lane;
    localparam int LANES = 32;
    localparam int ROWS = 256;
    localparam int FEATURES = 2304;
    localparam int COUNT = ROWS*FEATURES;
    localparam int BEATS = COUNT/LANES;

    logic clk = 1'b0, rst_n = 1'b0;
    logic in_valid = 1'b0;
    logic [LANES*16-1:0] in_fp16 = '0;
    logic out_valid;
    logic [LANES*16-1:0] out_fp16;

    logic [15:0] input_mem [0:COUNT-1];
    logic [15:0] expected_mem [0:COUNT-1];
    string input_file, expected_file, actual_file;
    integer beats_out, cycles, errors, fd;
    real got, expected, max_abs, max_ref, peak;

    always #5 clk = ~clk;

    supra2_gelu_tanh_32lane #(.LANES(LANES)) dut (
        .clk, .rst_n, .in_valid, .in_fp16, .out_valid, .out_fp16);

    function automatic real half_to_real(input logic [15:0] value);
        integer exponent; real significand;
        begin
            exponent = value[14:10];
            if (exponent == 0) significand = value[9:0]/1024.0;
            else significand = 1.0 + value[9:0]/1024.0;
            half_to_real = (value[15] ? -1.0 : 1.0) * significand *
                (exponent == 0 ? (2.0**-14) : (2.0**(exponent-15)));
        end
    endfunction

    // count cycles from first input beat to last output beat
    always @(posedge clk) begin
        if (rst_n && in_valid) cycles <= cycles + 1;
    end

    // check one 32-wide output beat per cycle
    always @(posedge clk) begin
        if (rst_n && out_valid) begin
            if (beats_out >= BEATS) $fatal(1, "extra output beat");
            for (int g = 0; g < LANES; g++) begin
                logic [15:0] got_h, exp_h;
                got_h = out_fp16[g*16 +: 16];
                exp_h = expected_mem[beats_out*LANES + g];
                if (got_h !== exp_h) begin
                    errors = errors + 1;
                    if (errors <= 10)
                        $display("MISMATCH beat=%0d lane=%0d got=%h exp=%h",
                                 beats_out, g, got_h, exp_h);
                end
                got = half_to_real(got_h);
                expected = half_to_real(exp_h);
                if ((expected - got) > max_abs) max_abs = expected - got;
                if ((got - expected) > max_abs) max_abs = got - expected;
            end
            beats_out <= beats_out + 1;
        end
    end

    initial begin
        if (!$value$plusargs("INPUT_FILE=%s", input_file) ||
            !$value$plusargs("EXPECTED_FILE=%s", expected_file) ||
            !$value$plusargs("ACTUAL_FILE=%s", actual_file))
            $fatal(1, "Missing gelu_32lane fixture plusargs");
        $readmemh(input_file, input_mem);
        $readmemh(expected_file, expected_mem);

        cycles = 0; beats_out = 0; errors = 0; max_abs = 0.0;
        peak = 0.0;
        for (int i = 0; i < COUNT; i++) begin
            real v;
            v = half_to_real(expected_mem[i]);
            if (v > peak) peak = v;
            if (-v > peak) peak = -v;
        end

        repeat (8) @(posedge clk);
        rst_n <= 1'b1;
        @(posedge clk);

        // stream all beats back-to-back; drive on negedge to avoid the
        // posedge sampling race with the DUT
        for (int b = 0; b < BEATS; b++) begin
            @(negedge clk);
            for (int g = 0; g < LANES; g++)
                in_fp16[g*16 +: 16] = input_mem[b*LANES + g];
            in_valid = 1'b1;
        end
        @(negedge clk);
        in_valid = 1'b0;

        wait (beats_out == BEATS);
        @(posedge clk);

        fd = $fopen(actual_file, "w");
        $fdisplay(fd, "# supra2_gelu_tanh_32lane: %0d beats streamed, %0d mismatches",
                  BEATS, errors);
        $fclose(fd);

        $display("SUPRA2_GELU_32LANE beats=%0d cycles=%0d errors=%0d max_abs=%g rel_to_peak=%g",
                 BEATS, cycles, errors, max_abs, max_abs/peak);
        if (errors == 0)
            $display("SUPRA2_GELU_32LANE_PASS rows=%0d features=%0d cycles=%0d",
                     ROWS, FEATURES, cycles);
        else
            $display("SUPRA2_GELU_32LANE_FAIL errors=%0d", errors);
        $finish;
    end

    initial begin
        #50_000_000;
        $fatal(1, "timeout");
    end
endmodule

`default_nettype wire
