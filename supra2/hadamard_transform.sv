`default_nettype none

// Block-buffered, block-diagonal normalized Sylvester Hadamard transform.
// The current Supra2 export uses block sizes 16, 64, and 256.
module supra2_hadamard_transform #(
    parameter int MAX_K = 2304,
    parameter int MAX_H = 256,
    parameter int LANES = 32,
    parameter int PAIRS_PER_CYCLE = LANES/2,
    parameter int K_W = 12,
    parameter int H_W = 9
) (
    input  wire logic                     clk,
    input  wire logic                     rst_n,
    input  wire logic                     start,
    input  wire logic [K_W-1:0]           vector_k,
    input  wire logic [H_W-1:0]           block_size,
    input  wire logic                     in_valid,
    output logic                          in_ready,
    input  wire logic [LANES*16-1:0]      in_fp16,
    output logic                          out_valid,
    input  wire logic                     out_ready,
    output logic [LANES*16-1:0]           out_fp16,
    output logic                          busy,
    output logic                          done
);
    import fp16_utils_pkg::*;

    typedef enum logic [2:0] {S_IDLE, S_LOAD, S_TRANSFORM, S_DRAIN, S_OUTPUT} state_t;
    state_t state;
    // Bank by the 32-element stream lane. The bounded butterfly issue width
    // limits the number of independent FP32 adders and dynamic memory ports.
    // H=16 shares one stream beat with a second block when K permits it.
    // Storage capacity depends on H, never on the full token-vector length K.
    localparam int SCRATCH_ELEMS = (MAX_H > LANES) ? MAX_H : LANES;
    localparam int BANK_DEPTH = SCRATCH_ELEMS / LANES;
    localparam int COUNT_W = $clog2(SCRATCH_ELEMS + LANES + 1);
    localparam int ROW_W = (BANK_DEPTH > 1) ? $clog2(BANK_DEPTH) : 1;
    localparam int LOG_LANES = $clog2(LANES);
    logic [31:0] activation_mem [0:LANES-1][0:BANK_DEPTH-1];
    logic [ROW_W-1:0] read_row0, read_row1;
    logic [31:0] bank_read0 [0:LANES-1];
    logic [31:0] bank_read1 [0:LANES-1];
    logic [LANES-1:0] activation_we0, activation_we1;
    logic [ROW_W-1:0] activation_wa0 [0:LANES-1];
    logic [ROW_W-1:0] activation_wa1 [0:LANES-1];
    logic [31:0] activation_wd0 [0:LANES-1];
    logic [31:0] activation_wd1 [0:LANES-1];
    logic [K_W-1:0] k_reg;
    logic [H_W-1:0] h_reg;
    logic [K_W-1:0] vector_base;
    logic [COUNT_W-1:0] input_count, output_count, block_base;
    logic [COUNT_W-1:0] group_elements;
    logic [H_W-1:0] pair_base;
    logic [3:0] stage_index;

    logic [2:0] normalization_shift;
    always_comb begin
        group_elements = (h_reg < LANES) ? COUNT_W'(LANES) : COUNT_W'(h_reg);
        if ((k_reg - vector_base) < group_elements)
            group_elements = COUNT_W'(k_reg - vector_base);
    end

    // All pairs in an aligned issue group address the same two memory rows.
    // Read each bank exactly twice, then select banks for the butterflies.
    // Indexing both dimensions independently per pair expands RAM read ports.
    always_comb begin : shared_read_address
        integer stride_value;
        integer first_index;
        stride_value = 1 << stage_index;
        first_index = int'(block_base) +
            ((int'(pair_base) >> stage_index) << (stage_index+1)) +
            (int'(pair_base) & (stride_value-1));
        read_row0 = ROW_W'(first_index / LANES);
        read_row1 = ROW_W'((first_index + stride_value) / LANES);
        if (state == S_OUTPUT)
            read_row0 = ROW_W'(output_count / LANES);
    end

    // Each stage has a compile-time bank permutation. Select among these
    // small fixed permutations rather than index all banks independently for
    // every butterfly and dynamically scatter results back to all banks.
    logic [3:0] route_stage;
    logic [7:0] transform_valid_pipe;
    logic [3:0] route_stage_pipe [0:7];
    logic [ROW_W-1:0] write_row0_pipe [0:7];
    logic [ROW_W-1:0] write_row1_pipe [0:7];
    logic high_half_pipe [0:7];
    logic [31:0] butterfly_sum [0:PAIRS_PER_CYCLE-1];
    logic [31:0] butterfly_diff [0:PAIRS_PER_CYCLE-1];
    assign route_stage = (stage_index < LOG_LANES) ? stage_index : 4'(LOG_LANES);
    generate
        for (genvar pair_lane=0; pair_lane<PAIRS_PER_CYCLE; pair_lane++) begin : gen_butterfly
            wire [31:0] lo_choice [0:LOG_LANES];
            wire [31:0] hi_choice [0:LOG_LANES];
            for (genvar s=0; s<LOG_LANES; s++) begin : gen_stage_input
                localparam int LOW_BANK = ((pair_lane >> s) << (s+1)) |
                                          (pair_lane & ((1 << s)-1));
                assign lo_choice[s] = bank_read0[LOW_BANK];
                assign hi_choice[s] = bank_read0[LOW_BANK + (1 << s)];
            end
            assign lo_choice[LOG_LANES] = pair_base[LOG_LANES-1]
                ? bank_read0[pair_lane + PAIRS_PER_CYCLE] : bank_read0[pair_lane];
            assign hi_choice[LOG_LANES] = pair_base[LOG_LANES-1]
                ? bank_read1[pair_lane + PAIRS_PER_CYCLE] : bank_read1[pair_lane];
            supra2_fp32_add_rne_pipeline sum_adder (
                .clk(clk), .rst_n(rst_n), .in_valid(state == S_TRANSFORM),
                .a(lo_choice[route_stage]), .b(hi_choice[route_stage]),
                .out_valid(), .result(butterfly_sum[pair_lane])
            );
            supra2_fp32_add_rne_pipeline diff_adder (
                .clk(clk), .rst_n(rst_n), .in_valid(state == S_TRANSFORM),
                .a(lo_choice[route_stage]),
                .b(hi_choice[route_stage] ^ 32'h80000000),
                .out_valid(), .result(butterfly_diff[pair_lane])
            );
        end
        for (genvar bank=0; bank<LANES; bank++) begin : gen_activation_bank
            wire [31:0] result_choice [0:LOG_LANES];
            for (genvar s=0; s<LOG_LANES; s++) begin : gen_stage_output
                localparam int PAIR = ((bank >> (s+1)) << s) |
                                      (bank & ((1 << s)-1));
                assign result_choice[s] = ((bank & (1 << s)) != 0)
                    ? butterfly_diff[PAIR] : butterfly_sum[PAIR];
            end
            assign result_choice[LOG_LANES] = butterfly_sum[bank % PAIRS_PER_CYCLE];
            assign bank_read0[bank] = activation_mem[bank][read_row0];
            assign bank_read1[bank] = activation_mem[bank][read_row1];
            always_comb begin
                activation_we0[bank] = 1'b0;
                activation_we1[bank] = 1'b0;
                activation_wa0[bank] = read_row0;
                activation_wa1[bank] = read_row1;
                activation_wd0[bank] = result_choice[route_stage_pipe[7]];
                activation_wd1[bank] = butterfly_diff[bank % PAIRS_PER_CYCLE];
                if (state == S_LOAD && in_valid && input_count + bank < group_elements) begin
                    activation_we0[bank] = 1'b1;
                    activation_wa0[bank] = ROW_W'(input_count / LANES);
                    activation_wd0[bank] = fp16_to_fp32(in_fp16[bank*16+:16]);
                end else if (transform_valid_pipe[7]) begin
                    activation_wa0[bank] = write_row0_pipe[7];
                    activation_wa1[bank] = write_row1_pipe[7];
                    if (route_stage_pipe[7] < LOG_LANES) begin
                        activation_we0[bank] = (h_reg >= LANES) ||
                            ((bank >= block_base) && (bank < block_base + h_reg));
                    end else if ((bank / PAIRS_PER_CYCLE) == high_half_pipe[7]) begin
                        activation_we0[bank] = 1'b1;
                        activation_we1[bank] = 1'b1;
                    end
                end
            end
            always_ff @(posedge clk) begin
                if (activation_we0[bank])
                    activation_mem[bank][activation_wa0[bank]]<=activation_wd0[bank];
                if (activation_we1[bank])
                    activation_mem[bank][activation_wa1[bank]]<=activation_wd1[bank];
            end
        end
    endgenerate

    always_comb begin
        in_ready = (state == S_LOAD);
        out_valid = (state == S_OUTPUT);
        busy = (state != S_IDLE);
        done = (state == S_OUTPUT) && out_ready &&
               (vector_base + output_count + LANES >= k_reg);
        case (h_reg)
            9'd16: normalization_shift = 3'd2;
            9'd64: normalization_shift = 3'd3;
            9'd256: normalization_shift = 3'd4;
            default: normalization_shift = 3'd0;
        endcase
        for (int lane = 0; lane < LANES; lane++) begin
            out_fp16[lane*16 +: 16] = 16'b0;
            if (state == S_OUTPUT && output_count + lane < group_elements)
                out_fp16[lane*16 +: 16] = fp32_to_fp16(
                    scale_fp32_pow2(bank_read0[lane],
                                    normalization_shift));
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin : transform_metadata
        if (!rst_n) begin
            transform_valid_pipe <= '0;
            for (int pipe_index=0; pipe_index<8; pipe_index++) begin
                route_stage_pipe[pipe_index] <= '0;
                write_row0_pipe[pipe_index] <= '0;
                write_row1_pipe[pipe_index] <= '0;
                high_half_pipe[pipe_index] <= 1'b0;
            end
        end else begin
            transform_valid_pipe <= {transform_valid_pipe[6:0],
                                     state == S_TRANSFORM};
            if (state == S_TRANSFORM) begin
                route_stage_pipe[0] <= route_stage;
                write_row0_pipe[0] <= read_row0;
                write_row1_pipe[0] <= read_row1;
                high_half_pipe[0] <= pair_base[LOG_LANES-1];
            end
            for (int pipe_index=1; pipe_index<8; pipe_index++) begin
                route_stage_pipe[pipe_index] <= route_stage_pipe[pipe_index-1];
                write_row0_pipe[pipe_index] <= write_row0_pipe[pipe_index-1];
                write_row1_pipe[pipe_index] <= write_row1_pipe[pipe_index-1];
                high_half_pipe[pipe_index] <= high_half_pipe[pipe_index-1];
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin : state_update
        integer log2_h;
        if (!rst_n) begin
            state <= S_IDLE;
            k_reg <= '0;
            h_reg <= '0;
            input_count <= '0;
            output_count <= '0;
            block_base <= '0;
            vector_base <= '0;
            pair_base <= '0;
            stage_index <= '0;
        end else begin
            case (state)
                S_IDLE: if (start) begin
                    if (vector_k == 0 || vector_k > MAX_K ||
                        (block_size != 16 && block_size != 64 &&
                         block_size != 256) || block_size > MAX_H ||
                        (vector_k & (K_W'(block_size)-1'b1)) != 0) begin
                        state <= S_IDLE;
                    end else begin
                        k_reg <= vector_k;
                        h_reg <= block_size;
                        input_count <= '0;
                        output_count <= '0;
                        block_base <= '0;
                        vector_base <= '0;
                        pair_base <= '0;
                        stage_index <= '0;
                        state <= S_LOAD;
                    end
                end
                S_LOAD: if (in_valid && in_ready) begin
                    input_count <= input_count + LANES;
                    if (input_count + LANES >= group_elements)
                        state <= S_TRANSFORM;
                end
                S_TRANSFORM: begin
                    if (pair_base + PAIRS_PER_CYCLE >= h_reg/2) begin
                        pair_base <= '0;
                        state <= S_DRAIN;
                    end else begin
                        pair_base <= pair_base + PAIRS_PER_CYCLE;
                    end
                end
                S_DRAIN: if (transform_valid_pipe == 0) begin
                    log2_h = 0;
                    for (int bit_index = 0; bit_index < H_W; bit_index++)
                        if ((1 << bit_index) == h_reg)
                            log2_h = bit_index;
                    if (stage_index + 1 >= log2_h) begin
                        stage_index <= '0;
                        if (block_base + h_reg >= group_elements) begin
                            output_count <= '0;
                            state <= S_OUTPUT;
                        end else begin
                            block_base <= block_base + h_reg;
                            state <= S_TRANSFORM;
                        end
                    end else begin
                        stage_index <= stage_index + 1'b1;
                        state <= S_TRANSFORM;
                    end
                end
                S_OUTPUT: if (out_valid && out_ready) begin
                    if (output_count + LANES >= group_elements) begin
                        state <= (vector_base + group_elements >= k_reg)
                            ? S_IDLE : S_LOAD;
                        vector_base <= vector_base + group_elements;
                        input_count <= '0;
                        output_count <= '0;
                        block_base <= '0;
                    end else begin
                        output_count <= output_count + LANES;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    function automatic logic [31:0] scale_fp32_pow2(
        input logic [31:0] value, input logic [2:0] exponent_delta
    );
        logic [7:0] exponent;
        begin
            exponent = value[30:23];
            if (exponent == 0 || exponent <= exponent_delta)
                return {value[31],31'b0};
            return {value[31], exponent-exponent_delta, value[22:0]};
        end
    endfunction

    initial begin
        if (LANES <= 0 || (LANES % 2) != 0 || (LANES & (LANES-1)) != 0 ||
            PAIRS_PER_CYCLE <= 0 ||
            PAIRS_PER_CYCLE != LANES/2 ||
            MAX_K < LANES ||
            (MAX_K % LANES) != 0 || MAX_H > MAX_K)
            $error("Invalid Supra2 Hadamard dimensions");
    end
endmodule

`default_nettype wire
