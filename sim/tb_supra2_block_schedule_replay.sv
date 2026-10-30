`default_nettype none

module tb_supra2_block_schedule_replay;
    localparam int NRES = 7;
    logic clk = 1'b0, rst_n = 1'b0;
    logic [NRES-1:0] res_busy, res_error;
    logic replay_done;
    logic [31:0] makespan_cycles;
    logic [31:0] busy_cycles [0:NRES-1];

    integer expected_makespan, expected_array_busy;
    integer errors;

    always #5 clk = ~clk;

    supra2_block_schedule_replay #(.NRES(NRES)) dut (
        .clk, .rst_n,
        .res_busy, .res_error,
        .replay_done, .makespan_cycles, .busy_cycles);

    initial begin
        if (!$value$plusargs("EXPECTED_MAKESPAN=%d", expected_makespan) ||
            !$value$plusargs("EXPECTED_BUSY_ARRAY=%d", expected_array_busy))
            $fatal(1, "Missing EXPECTED_MAKESPAN / EXPECTED_BUSY_ARRAY plusargs");

        repeat (8) @(posedge clk);
        rst_n <= 1'b1;

        wait (replay_done);
        @(posedge clk);

        errors = 0;
        if (makespan_cycles !== expected_makespan) begin
            $display("MAKESEPAN_MISMATCH got=%0d expected=%0d",
                     makespan_cycles, expected_makespan);
            errors = errors + 1;
        end
        if (busy_cycles[0] !== expected_array_busy) begin
            $display("ARRAY_BUSY_MISMATCH got=%0d expected=%0d",
                     busy_cycles[0], expected_array_busy);
            errors = errors + 1;
        end
        if (res_error != 0) begin
            $display("RESOURCE_VIOLATIONS res_error=%b", res_error);
            errors = errors + 1;
        end

        $display("SCHEDULE_REPLAY makespan=%0d array_busy=%0d ln_a=%0d ln_b=%0d ln_c=%0d r1=%0d r2=%0d sft=%0d errors=%0d",
                 makespan_cycles, busy_cycles[0], busy_cycles[1], busy_cycles[2],
                 busy_cycles[3], busy_cycles[4], busy_cycles[5], busy_cycles[6],
                 errors);
        if (errors == 0)
            $display("SUPRA2_SCHEDULE_REPLAY_PASS makespan=%0d array_busy=%0d",
                     makespan_cycles, busy_cycles[0]);
        else
            $display("SUPRA2_SCHEDULE_REPLAY_FAIL errors=%0d", errors);
        $finish;
    end

    initial begin
        #500_000_000;
        $fatal(1, "timeout");
    end
endmodule

`default_nettype wire
