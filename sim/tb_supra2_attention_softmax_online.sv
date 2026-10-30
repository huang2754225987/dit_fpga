`timescale 1ns/1ps
`default_nettype none

module tb_supra2_attention_softmax_online;
    localparam int TOKENS = 256;
    localparam int HEADS = 9;
    localparam int TILE = 32;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic cmd_valid = 1'b0;
    logic cmd_ready;
    logic [3:0] cmd_vd = 4'd1;
    logic [3:0] cmd_vs1 = 4'd0;
    logic [3:0] cmd_alpha_vd = 4'd2;
    logic [6:0] cmd_state_slot = '0;
    logic [5:0] cmd_valid_lanes = TILE;
    logic cmd_online = 1'b1;
    logic cmd_online_reset = 1'b0;
    logic cmd_raw_output = 1'b1;
    logic cmd_finalize = 1'b0;
    logic [3:0] rf_rd_addr;
    logic [511:0] rf_rd_data;
    logic rf_wr_en;
    logic [3:0] rf_wr_addr;
    logic [511:0] rf_wr_data;
    logic busy, done;
    logic [15:0] scores [0:TOKENS*HEADS*TOKENS-1];
    logic [15:0] expected [0:TOKENS*HEADS*TOKENS-1];
    logic [15:0] raw_exp [0:TOKENS*HEADS*TOKENS-1];
    logic [15:0] probability [0:TOKENS*HEADS*TOKENS-1];
    logic [15:0] alpha [0:HEADS-1][0:TOKENS/TILE-1];
    logic [15:0] reciprocal [0:HEADS-1];
    string score_file, expected_file, actual_file;
    integer active_query, active_head, active_tile, lane, query, h, tile, index;
    integer command_cycles, total_cycles, output_count;
    integer output_fd;
    logic [15:0] prob_bits;
    real got, ref_value, abs_error, max_error, max_ref, sum_sq;

    always #5 clk = ~clk;
    always_comb begin
        rf_rd_data = '0;
        for (int i = 0; i < TILE; i++)
            rf_rd_data[i*16 +: 16] =
                scores[(active_query*HEADS+active_head)*TOKENS+active_tile*TILE+i];
    end

    softmax_engine #(.LANES(TILE), .ELEM_W(16)) dut (
        .clk, .rst_n, .cmd_valid, .cmd_ready, .cmd_vd, .cmd_vs1,
        .cmd_alpha_vd, .cmd_state_slot, .cmd_valid_lanes, .cmd_online,
        .cmd_online_reset, .cmd_raw_output, .cmd_finalize,
        .rf_rd_addr, .rf_rd_data, .rf_wr_en, .rf_wr_addr, .rf_wr_data,
        .busy, .done
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

    always @(posedge clk) begin
        if (busy) total_cycles <= total_cycles + 1;
        if (rf_wr_en) begin
            if (rf_wr_addr == cmd_vd) begin
                for (int i = 0; i < TILE; i++)
                    raw_exp[(active_query*HEADS+active_head)*TOKENS+
                            active_tile*TILE+i] <=
                        rf_wr_data[i*16 +: 16];
            end else if (rf_wr_addr == cmd_alpha_vd) begin
                if (cmd_finalize)
                    reciprocal[active_head] <= rf_wr_data[15:0];
                else
                    alpha[active_head][active_tile] <= rf_wr_data[15:0];
            end
        end
    end

    initial begin
        if (!$value$plusargs("SCORE_FILE=%s", score_file) ||
            !$value$plusargs("EXPECTED_FILE=%s", expected_file) ||
            !$value$plusargs("ACTUAL_FILE=%s", actual_file))
            $fatal(1, "Missing attention-softmax fixture plusargs");
        $readmemh(score_file, scores);
        $readmemh(expected_file, expected);
        output_fd = $fopen(actual_file, "w");
        if (!output_fd) $fatal(1, "Cannot open online-softmax output file");
        total_cycles = 0;
        output_count = 0;
        max_error = 0.0;
        max_ref = 0.0;
        sum_sq = 0.0;
        repeat (4) @(negedge clk);
        rst_n = 1'b1;

        for (query = 0; query < TOKENS; query++) begin
          active_query = query;
          for (h = 0; h < HEADS; h++) begin
            active_head = h;
            for (tile = 0; tile < TOKENS/TILE; tile++) begin
                active_tile = tile;
                @(negedge clk);
                while (!cmd_ready) @(negedge clk);
                cmd_state_slot = h;
                cmd_online_reset = (tile == 0);
                cmd_finalize = 1'b0;
                cmd_raw_output = 1'b1;
                cmd_valid = 1'b1;
                @(negedge clk);
                cmd_valid = 1'b0;
                wait (done);
                @(negedge clk);
            end

            active_tile = TOKENS/TILE-1;
            while (!cmd_ready) @(negedge clk);
            cmd_state_slot = h;
            cmd_online_reset = 1'b0;
            cmd_finalize = 1'b1;
            cmd_raw_output = 1'b0;
            cmd_valid = 1'b1;
            @(negedge clk);
            cmd_valid = 1'b0;
            wait (done);
            @(negedge clk);
            cmd_finalize = 1'b0;

            for (tile = 0; tile < TOKENS/TILE; tile++) begin
                for (lane = 0; lane < TILE; lane++) begin
                    index = (query*HEADS+h)*TOKENS+tile*TILE+lane;
                    prob_bits = raw_exp[index];
                    for (integer later = tile+1; later < TOKENS/TILE; later++)
                        prob_bits = fp16_utils_pkg::fp16_mul(prob_bits, alpha[h][later]);
                    prob_bits = fp16_utils_pkg::fp16_mul(prob_bits, reciprocal[h]);
                    probability[index] = prob_bits;
                    got = half_to_real(prob_bits);
                    ref_value = half_to_real(expected[index]);
                    abs_error = (got >= ref_value) ? got-ref_value : ref_value-got;
                    if (abs_error > max_error) max_error = abs_error;
                    if ((ref_value >= 0 ? ref_value : -ref_value) > max_ref)
                        max_ref = (ref_value >= 0 ? ref_value : -ref_value);
                    sum_sq = sum_sq + abs_error*abs_error;
                    output_count = output_count + 1;
                end
            end
          end
        end

        $display("SUPRA2_ATTN_SOFTMAX_PASS queries=%0d heads=%0d keys=%0d cycles=%0d",
                 TOKENS, HEADS, TOKENS, total_cycles);
        $display("SUPRA2_ATTN_SOFTMAX_MAX_ABS %.9g", max_error);
        $display("SUPRA2_ATTN_SOFTMAX_REL_TO_MAX %.9g", max_error/max_ref);
        $display("SUPRA2_ATTN_SOFTMAX_RMSE %.9g", $sqrt(sum_sq/output_count));
        for (index = 0; index < TOKENS*HEADS*TOKENS; index++)
            $fdisplay(output_fd, "%04h", probability[index]);
        $fclose(output_fd);
        if (output_count != TOKENS*HEADS*TOKENS || max_error/max_ref > 0.02)
            $fatal(1, "Online attention softmax failed 2%% relative-to-peak gate");
        $finish;
    end
endmodule

`default_nettype wire
