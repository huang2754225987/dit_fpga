`default_nettype none

module tb_supra2_mac_fp16_probe;
    localparam int TOKENS = 256;
    localparam int HEADS = 9;
    localparam int HEAD_DIM = 64;
    localparam int KEYS = 256;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic clear_acc = 1'b0;
    logic en = 1'b0;
    logic [15:0] a_in = '0;
    logic [15:0] b_in = '0;
    logic [31:0] acc_out;
    logic [15:0] q_mem [0:TOKENS*HEADS*HEAD_DIM-1];
    logic [15:0] k_mem [0:KEYS*HEADS*HEAD_DIM-1];
    logic [15:0] expected_mem [0:TOKENS*HEADS*KEYS-1];
    string q_file, k_file, expected_file;
    integer query, head, key, feature, q_index, k_index, score_index;
    real got_score, expected_score, abs_error, relative_error;

    always #5 clk = ~clk;

    mac_fp16 #(.DATA_W(16), .ACC_W(32)) dut (
        .clk(clk), .rst_n(rst_n), .clear_acc(clear_acc), .en(en),
        .a_in(a_in), .b_in(b_in), .acc_out(acc_out)
    );

    function automatic real half_to_real(input logic [15:0] value);
        integer exponent;
        real significand;
        begin
            exponent = value[14:10];
            if (exponent == 0) significand = value[9:0] / 1024.0;
            else significand = 1.0 + value[9:0] / 1024.0;
            half_to_real = (value[15] ? -1.0 : 1.0) * significand *
                (exponent == 0 ? (2.0 ** -14) : (2.0 ** (exponent - 15)));
        end
    endfunction

    function automatic real single_to_real(input logic [31:0] value);
        integer exponent;
        real significand;
        begin
            exponent = value[30:23];
            if (exponent == 0) significand = value[22:0] / 8388608.0;
            else significand = 1.0 + value[22:0] / 8388608.0;
            single_to_real = (value[31] ? -1.0 : 1.0) * significand *
                (exponent == 0 ? (2.0 ** -126) : (2.0 ** (exponent - 127)));
        end
    endfunction

    initial begin
        if (!$value$plusargs("Q_FILE=%s", q_file) ||
            !$value$plusargs("K_FILE=%s", k_file) ||
            !$value$plusargs("EXPECTED_FILE=%s", expected_file) ||
            !$value$plusargs("QUERY=%d", query) ||
            !$value$plusargs("HEAD=%d", head) ||
            !$value$plusargs("KEY=%d", key))
            $fatal(1, "Missing FP16 MAC probe plusargs");
        if (query < 0 || query >= TOKENS || head < 0 || head >= HEADS ||
            key < 0 || key >= KEYS)
            $fatal(1, "FP16 MAC probe coordinates are out of range");

        $readmemh(q_file, q_mem);
        $readmemh(k_file, k_mem);
        $readmemh(expected_file, expected_mem);
        q_index = (head * TOKENS + query) * HEAD_DIM;
        k_index = (head * KEYS + key) * HEAD_DIM;
        score_index = (query * HEADS + head) * KEYS + key;
        expected_score = half_to_real(expected_mem[score_index]);

        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk);
        clear_acc = 1'b1;
        @(negedge clk);
        clear_acc = 1'b0;
        for (feature = 0; feature < HEAD_DIM; feature++) begin
            en = 1'b1;
            a_in = q_mem[q_index + feature];
            b_in = k_mem[k_index + feature];
            @(negedge clk);
        end
        en = 1'b0;
        a_in = '0;
        b_in = '0;
        repeat (3) @(negedge clk);

        got_score = single_to_real(acc_out) / 8.0;
        abs_error = (got_score >= expected_score)
                  ? got_score - expected_score : expected_score - got_score;
        relative_error = abs_error / ((expected_score >= 0.0)
                                    ? ((expected_score > 1.0e-20) ? expected_score : 1.0)
                                    : ((-expected_score > 1.0e-20) ? -expected_score : 1.0));
        $display("SUPRA2_FP16_MAC_PROBE query=%0d head=%0d key=%0d acc=%h got=%e expected=%e abs=%e rel=%e",
                 query, head, key, acc_out, got_score, expected_score,
                 abs_error, relative_error);
        if (abs_error > 0.005)
            $fatal(1, "FP16 MAC probe exceeded score error tolerance");
        $display("SUPRA2_FP16_MAC_PROBE_PASS");
        $finish;
    end
endmodule

`default_nettype wire
