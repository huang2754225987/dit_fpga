`timescale 1ns/1ps
`default_nettype none

module tb_supra2_hadamard;
    localparam int LANES = 32;
    localparam int MAX_K = 2304;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic start = 1'b0;
    logic [11:0] vector_k = '0;
    logic [8:0] block_size = '0;
    logic in_valid = 1'b0;
    logic in_ready;
    logic [LANES*16-1:0] in_fp16 = '0;
    logic out_valid;
    logic out_ready = 1'b1;
    logic [LANES*16-1:0] out_fp16;
    logic busy, done;
    logic done_seen = 1'b0;
    logic [15:0] input_mem [0:MAX_K-1];
    logic [15:0] expected_mem [0:MAX_K-1];
    logic [15:0] package_mem [0:MAX_K-1];
    string input_file, expected_file, package_file;
    integer k_size, h_size, beat, lane, mismatches, package_mismatches;
    real got_value, package_value, error_value, max_package_error;
    real package_absmax, package_relative_error;
    integer ready_cycle = 0;
    integer input_stalls = 0, output_stalls = 0;
    logic held_output = 1'b0;
    logic [LANES*16-1:0] held_data;

    always #5 clk = ~clk;
    initial begin
        #1000000;
        $fatal(1, "Hadamard regression timeout");
    end
    always @(posedge clk)
        if (done) done_seen <= 1'b1;
    always @(negedge clk) begin
        ready_cycle <= ready_cycle + 1;
        out_ready <= (ready_cycle > 20) && (ready_cycle % 5 == 0);
    end
    always @(posedge clk) begin
        if (rst_n) begin
            if (held_output && (!out_valid || out_fp16 !== held_data))
                $fatal(1, "Hadamard changed a stalled output beat");
            held_output <= out_valid && !out_ready;
            held_data <= out_fp16;
            if (in_valid && !in_ready) input_stalls <= input_stalls + 1;
            if (out_valid && !out_ready) output_stalls <= output_stalls + 1;
        end
    end

    supra2_hadamard_transform #(
        .MAX_K(MAX_K), .MAX_H(256), .LANES(LANES), .K_W(12), .H_W(9)
    ) dut (
        .clk(clk), .rst_n(rst_n), .start(start), .vector_k(vector_k),
        .block_size(block_size), .in_valid(in_valid), .in_ready(in_ready),
        .in_fp16(in_fp16), .out_valid(out_valid), .out_ready(out_ready),
        .out_fp16(out_fp16), .busy(busy), .done(done)
    );

    function automatic real half_to_real(input logic [15:0] value);
        integer exponent;
        real significand, result;
        begin
            exponent = value[14:10];
            if (exponent == 0) significand = value[9:0] / 1024.0;
            else significand = 1.0 + value[9:0] / 1024.0;
            if (exponent == 0) result = significand * (2.0 ** -14);
            else if (exponent == 31) result = 0.0;
            else result = significand * (2.0 ** (exponent - 15));
            half_to_real = value[15] ? -result : result;
        end
    endfunction

    initial begin
        if (!$value$plusargs("K=%d", k_size) ||
            !$value$plusargs("H=%d", h_size) ||
            !$value$plusargs("INPUT_FILE=%s", input_file) ||
            !$value$plusargs("EXPECTED_FILE=%s", expected_file) ||
            !$value$plusargs("PACKAGE_FILE=%s", package_file) ||
            k_size <= 0 || k_size > MAX_K ||
            h_size <= 0 || k_size % h_size != 0)
            $fatal(1, "Provide valid K, H, INPUT_FILE, EXPECTED_FILE, PACKAGE_FILE");
        $readmemh(input_file, input_mem, 0, k_size-1);
        $readmemh(expected_file, expected_mem, 0, k_size-1);
        $readmemh(package_file, package_mem, 0, k_size-1);
        package_absmax = 0.0;
        for (int i = 0; i < k_size; i++) begin
            package_value = half_to_real(package_mem[i]);
            if ((package_value >= 0 ? package_value : -package_value) > package_absmax)
                package_absmax = package_value >= 0 ? package_value : -package_value;
        end

        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk);
        vector_k = k_size;
        block_size = h_size;
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;
        mismatches = 0;
        package_mismatches = 0;
        max_package_error = 0.0;
        fork
        begin : producer
            for (int ib = 0; ib < (k_size+LANES-1)/LANES; ib++) begin
                if (ib % 3 == 1) begin
                    in_valid = 1'b0;
                    @(negedge clk);
                end
                in_valid = 1'b1;
                for (int il = 0; il < LANES; il++)
                    in_fp16[il*16 +: 16] = (ib*LANES+il < k_size)
                        ? input_mem[ib*LANES+il] : 16'b0;
                @(posedge clk);
                while (!in_ready) @(posedge clk);
                @(negedge clk);
            end
            in_valid = 1'b0;
            in_fp16 = '0;
        end
        begin : consumer
        beat = 0;
        while (beat < (k_size+LANES-1)/LANES) begin
            @(posedge clk);
            if (out_valid && out_ready) begin
                for (lane = 0; lane < LANES; lane++) begin
                    got_value = half_to_real(out_fp16[lane*16 +: 16]);
                    if (beat*LANES+lane < k_size &&
                        out_fp16[lane*16 +: 16] !== expected_mem[beat*LANES+lane]) begin
                        if (mismatches < 8)
                            $display("HADAMARD_MISMATCH index=%0d got=%h expected=%h",
                                     beat*LANES+lane,
                                     out_fp16[lane*16 +: 16],
                                     expected_mem[beat*LANES+lane]);
                        mismatches++;
                    end
                    if (beat*LANES+lane >= k_size) begin
                        if (out_fp16[lane*16 +: 16] !== 16'b0)
                            mismatches++;
                        continue;
                    end
                    package_value = half_to_real(package_mem[beat*LANES+lane]);
                    error_value = (got_value >= package_value)
                                ? got_value - package_value
                                : package_value - got_value;
                    if (error_value > max_package_error)
                        max_package_error = error_value;
                    if (out_fp16[lane*16 +: 16] !== package_mem[beat*LANES+lane])
                        package_mismatches++;
                end
                beat++;
            end
        end
        end
        join
        @(negedge clk);
        package_relative_error = max_package_error /
            ((package_absmax > 1.0e-20) ? package_absmax : 1.0);
        if (mismatches != 0)
            $fatal(1, "Hadamard RTL differed from FP16 software transform at %0d values",
                   mismatches);
        if (!done_seen || busy)
            $fatal(1, "Hadamard completion handshake missing");
        $display("SUPRA2_HADAMARD_PACKAGE_BIT_MISMATCHES %0d", package_mismatches);
        $display("SUPRA2_HADAMARD_PACKAGE_REL_ERROR %e", package_relative_error);
        if (output_stalls == 0 || (k_size > h_size && k_size > LANES && input_stalls == 0))
            $fatal(1, "Hadamard regression failed to exercise backpressure");
        $display("SUPRA2_HADAMARD_BACKPRESSURE_PASS input_stalls=%0d output_stalls=%0d", input_stalls, output_stalls);
        $display("SUPRA2_HADAMARD_PASS K=%0d H=%0d", k_size, h_size);
        $finish;
    end
endmodule

`default_nettype wire
