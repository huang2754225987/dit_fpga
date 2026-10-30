`default_nettype none

module tb_supra2_residual_gate_32lane;
    localparam int LANES = 32;
    localparam int ROWS = 256;
    localparam int FEATURES = 576;
    localparam int COUNT = ROWS*FEATURES;
    localparam int BEATS = COUNT/LANES;         // 4608
    localparam int GBEATS = FEATURES/LANES;     // 18

    logic clk = 1'b0, rst_n = 1'b0;
    logic in_valid = 1'b0;
    logic [LANES*16-1:0] residual_fp16 = '0;
    logic [LANES*16-1:0] update_fp16 = '0;
    logic [LANES*16-1:0] gate_fp16 = '0;
    logic out_valid;
    logic [LANES*16-1:0] out_fp16;

    logic [15:0] residual_mem [0:COUNT-1];
    logic [15:0] update_mem [0:COUNT-1];
    logic [15:0] gate_mem [0:FEATURES-1];
    logic [15:0] expected_mem [0:COUNT-1];
    string residual_file, update_file, gate_file, expected_file, actual_file;
    integer beats_out, cycles, errors, fd;

    always #5 clk = ~clk;

    supra2_residual_gate_32lane #(.LANES(LANES)) dut (
        .clk, .rst_n, .in_valid,
        .residual_fp16, .update_fp16, .gate_fp16,
        .out_valid, .out_fp16);

    always @(posedge clk) begin
        if (rst_n && in_valid) cycles <= cycles + 1;
    end

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
            end
            beats_out <= beats_out + 1;
        end
    end

    initial begin
        if (!$value$plusargs("RESIDUAL_FILE=%s", residual_file) ||
            !$value$plusargs("UPDATE_FILE=%s", update_file) ||
            !$value$plusargs("GATE_FILE=%s", gate_file) ||
            !$value$plusargs("EXPECTED_FILE=%s", expected_file) ||
            !$value$plusargs("ACTUAL_FILE=%s", actual_file))
            $fatal(1, "Missing residual_32lane fixture plusargs");
        $readmemh(residual_file, residual_mem);
        $readmemh(update_file, update_mem);
        $readmemh(gate_file, gate_mem);
        $readmemh(expected_file, expected_mem);

        cycles = 0; beats_out = 0; errors = 0;

        repeat (8) @(posedge clk);
        rst_n <= 1'b1;

        // stream all beats; drive on negedge to avoid the sampling race
        for (int b = 0; b < BEATS; b++) begin
            @(negedge clk);
            for (int g = 0; g < LANES; g++) begin
                residual_fp16[g*16 +: 16] = residual_mem[b*LANES + g];
                update_fp16[g*16 +: 16]   = update_mem[b*LANES + g];
                gate_fp16[g*16 +: 16]     = gate_mem[(b % GBEATS)*LANES + g];
            end
            in_valid = 1'b1;
        end
        @(negedge clk);
        in_valid = 1'b0;

        wait (beats_out == BEATS);
        @(posedge clk);

        fd = $fopen(actual_file, "w");
        $fdisplay(fd, "# supra2_residual_gate_32lane: %0d beats, %0d mismatches",
                  BEATS, errors);
        $fclose(fd);

        $display("SUPRA2_RESIDUAL_32LANE beats=%0d cycles=%0d errors=%0d",
                 BEATS, cycles, errors);
        if (errors == 0)
            $display("SUPRA2_RESIDUAL_32LANE_PASS rows=%0d features=%0d cycles=%0d",
                     ROWS, FEATURES, cycles);
        else
            $display("SUPRA2_RESIDUAL_32LANE_FAIL errors=%0d", errors);
        $finish;
    end

    initial begin
        #50_000_000;
        $fatal(1, "timeout");
    end
endmodule

`default_nettype wire
