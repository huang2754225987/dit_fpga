`default_nettype none

module tb_supra2_euler_replay;
    localparam int LANES = 32;
    localparam int ELEMENTS = 4096;
    localparam int STEPS = 50;
    localparam int BEATS = ELEMENTS / LANES;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic start = 1'b0;
    logic [12:0] vector_elements = '0;
    logic [15:0] step_size_fp16 = 16'h251f;
    logic in_valid = 1'b0;
    logic in_ready;
    logic [LANES*16-1:0] latent_fp16 = '0;
    logic [LANES*16-1:0] velocity_fp16 = '0;
    logic out_valid;
    logic out_ready = 1'b1;
    logic [LANES*16-1:0] updated_latent_fp16;
    logic busy, done;

    logic [15:0] latent_mem [0:ELEMENTS-1];
    logic [15:0] velocity_mem [0:STEPS*ELEMENTS-1];
    logic [15:0] expected_mem [0:ELEMENTS-1];
    logic [15:0] reference_mem [0:ELEMENTS-1];
    string z0_file, velocity_file, expected_file, reference_file;
    integer step, beat, lane, index, total_cycles, mismatches, reference_mismatches;
    real got_value, expected_value, squared_error, squared_reference, relative_l2;

    always #5 clk = ~clk;

    supra2_euler_update #(.LANES(LANES), .COUNT_W(13)) dut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .vector_elements(vector_elements), .step_size_fp16(step_size_fp16),
        .in_valid(in_valid), .in_ready(in_ready), .latent_fp16(latent_fp16),
        .velocity_fp16(velocity_fp16), .out_valid(out_valid),
        .out_ready(out_ready), .updated_latent_fp16(updated_latent_fp16),
        .busy(busy), .done(done)
    );

    function automatic real half_to_real(input logic [15:0] value);
        integer exponent;
        real significand, magnitude;
        begin
            exponent = value[14:10];
            if (exponent == 0) significand = value[9:0] / 1024.0;
            else significand = 1.0 + value[9:0] / 1024.0;
            if (exponent == 0) magnitude = significand * (2.0 ** -14);
            else if (exponent == 31) magnitude = 0.0;
            else magnitude = significand * (2.0 ** (exponent - 15));
            half_to_real = value[15] ? -magnitude : magnitude;
        end
    endfunction

    initial begin
        if (!$value$plusargs("Z0_FILE=%s", z0_file) ||
            !$value$plusargs("VELOCITY_FILE=%s", velocity_file) ||
            !$value$plusargs("EXPECTED_FILE=%s", expected_file) ||
            !$value$plusargs("REFERENCE_FILE=%s", reference_file))
            $fatal(1, "Provide Z0_FILE, VELOCITY_FILE, EXPECTED_FILE, REFERENCE_FILE");
        $readmemh(z0_file, latent_mem);
        $readmemh(velocity_file, velocity_mem);
        $readmemh(expected_file, expected_mem);
        $readmemh(reference_file, reference_mem);
        total_cycles = 0;
        mismatches = 0;
        reference_mismatches = 0;

        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        vector_elements = ELEMENTS;
        for (step = 0; step < STEPS; step++) begin
            @(negedge clk);
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            in_valid = 1'b1;
            for (beat = 0; beat < BEATS; beat++) begin
                for (lane = 0; lane < LANES; lane++) begin
                    index = beat*LANES + lane;
                    latent_fp16[lane*16 +: 16] = latent_mem[index];
                    velocity_fp16[lane*16 +: 16] =
                        velocity_mem[step*ELEMENTS + index];
                end
                @(posedge clk);
                if (!in_ready || !out_valid)
                    $fatal(1, "Euler stream handshake failed at step %0d beat %0d",
                           step, beat);
                for (lane = 0; lane < LANES; lane++)
                    latent_mem[beat*LANES+lane] =
                        updated_latent_fp16[lane*16 +: 16];
                total_cycles++;
                @(negedge clk);
            end
            in_valid = 1'b0;
            if (busy)
                $fatal(1, "Euler unit did not finish step %0d", step);
        end

        squared_error = 0.0;
        squared_reference = 0.0;
        for (index = 0; index < ELEMENTS; index++) begin
            got_value = half_to_real(latent_mem[index]);
            expected_value = half_to_real(expected_mem[index]);
            squared_error = squared_error +
                (got_value-expected_value)*(got_value-expected_value);
            squared_reference = squared_reference + expected_value*expected_value;
            if (latent_mem[index] !== expected_mem[index]) mismatches++;
            if (latent_mem[index] !== reference_mem[index]) begin
                if (reference_mismatches < 8)
                    $display("EULER_SOFTWARE_MISMATCH index=%0d got=%h expected=%h",
                             index, latent_mem[index], reference_mem[index]);
                reference_mismatches++;
            end
        end
        relative_l2 = $sqrt(squared_error / squared_reference);
        if (relative_l2 >= 0.01)
            $fatal(1, "Euler replay relative L2 exceeded 1%%: %e", relative_l2);
        if (total_cycles != STEPS*BEATS)
            $fatal(1, "Unexpected Euler cycle count: %0d", total_cycles);
        if (reference_mismatches != 0)
            $fatal(1, "Euler RTL differed from FP16 software replay at %0d values",
                   reference_mismatches);
        $display("SUPRA2_EULER_REPLAY_PASS steps=%0d elements=%0d",
                 STEPS, ELEMENTS);
        $display("SUPRA2_EULER_REPLAY_CYCLES %0d", total_cycles);
        $display("SUPRA2_EULER_REPLAY_BIT_MISMATCHES %0d", mismatches);
        $display("SUPRA2_EULER_REPLAY_SOFTWARE_MISMATCHES %0d", reference_mismatches);
        $display("SUPRA2_EULER_REPLAY_REL_L2 %e", relative_l2);
        $finish;
    end
endmodule

`default_nettype wire
