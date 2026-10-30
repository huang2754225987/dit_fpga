`default_nettype none

// Two-anchor full-residual cache for CFG's two independent branches.
// Stores ELEMENTS FP16 values per anchor in LANES-wide beats. A refresh rotates
// one branch's pair of anchors; reuse streams the DCTA-aligned residual out.
module supra2_dicache_residual_cache #(
    parameter int ELEMENTS = 4096,
    parameter int LANES = 32,
    parameter int BEATS = ELEMENTS / LANES,
    parameter int INDEX_W = (BEATS <= 1) ? 1 : $clog2(BEATS)
) (
    input  wire logic                 clk,
    input  wire logic                 rst_n,
    input  wire logic                 run_start,
    input  wire logic                 branch_index,
    input  wire logic                 refresh_start,
    input  wire logic                 refresh_valid,
    output wire logic                 refresh_ready,
    input  wire logic [LANES*16-1:0]  refresh_residual_fp16,
    input  wire logic                 refresh_last,
    output logic                      refresh_done,
    input  wire logic                 reuse_start,
    input  wire logic [15:0]          gamma_fp16,
    output wire logic                 reuse_valid,
    input  wire logic                 reuse_ready,
    output wire logic [LANES*16-1:0]  reuse_residual_fp16,
    output wire logic                 reuse_last,
    output wire logic                 reuse_done,
    output wire logic                 cache_valid,
    output wire logic                 dcta_valid,
    output logic                      cache_error
);
    logic [LANES*16-1:0] residual_mem [0:1][0:1][0:BEATS-1];
    logic head_slot [0:1];
    logic [1:0] anchor_count [0:1];

    logic refresh_active, refresh_branch, refresh_slot;
    logic [INDEX_W-1:0] refresh_index;
    logic read_active, read_branch, read_alpha_slot, read_beta_slot;
    logic [INDEX_W-1:0] read_index;
    logic [15:0] gamma_latched;
    logic [LANES*16-1:0] read_alpha_q, read_beta_q;
    logic read_valid_q, read_last_q;
    wire dcta_in_ready, dcta_out_valid, dcta_out_last;
    wire [LANES*16-1:0] dcta_out_data;
    wire read_issue = read_active && (!read_valid_q || dcta_in_ready);

    assign refresh_ready = refresh_active;
    assign cache_valid = anchor_count[branch_index] != 0;
    assign dcta_valid = anchor_count[branch_index] == 2;
    assign reuse_valid = dcta_out_valid;
    assign reuse_residual_fp16 = dcta_out_data;
    assign reuse_last = dcta_out_last;
    assign reuse_done = dcta_out_valid && reuse_ready && dcta_out_last;

    supra2_dicache_dcta_32lane #(.LANES(LANES)) u_dcta (
        .clk(clk), .rst_n(rst_n), .in_valid(read_valid_q),
        .in_ready(dcta_in_ready), .in_last(read_last_q),
        .residual_alpha_fp16(read_alpha_q),
        .residual_beta_fp16(read_beta_q), .gamma_fp16(gamma_latched),
        .out_valid(dcta_out_valid), .out_last(dcta_out_last),
        .out_ready(reuse_ready), .residual_out_fp16(dcta_out_data)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            head_slot[0] <= 1'b0;
            head_slot[1] <= 1'b0;
            anchor_count[0] <= '0;
            anchor_count[1] <= '0;
            refresh_active <= 1'b0;
            refresh_branch <= 1'b0;
            refresh_slot <= 1'b0;
            refresh_index <= '0;
            read_active <= 1'b0;
            read_branch <= 1'b0;
            read_alpha_slot <= 1'b0;
            read_beta_slot <= 1'b0;
            read_index <= '0;
            gamma_latched <= '0;
            read_alpha_q <= '0;
            read_beta_q <= '0;
            read_valid_q <= 1'b0;
            read_last_q <= 1'b0;
            refresh_done <= 1'b0;
            cache_error <= 1'b0;
        end else begin
            refresh_done <= 1'b0;

            if (run_start) begin
                head_slot[0] <= 1'b0;
                head_slot[1] <= 1'b0;
                anchor_count[0] <= '0;
                anchor_count[1] <= '0;
                refresh_active <= 1'b0;
                read_active <= 1'b0;
                read_valid_q <= 1'b0;
                cache_error <= 1'b0;
            end

            if (refresh_start) begin
                if (refresh_active || read_active || reuse_start) begin
                    cache_error <= 1'b1;
                end else begin
                    refresh_active <= 1'b1;
                    refresh_branch <= branch_index;
                    refresh_slot <= (anchor_count[branch_index] == 0) ?
                                    1'b0 : ~head_slot[branch_index];
                    refresh_index <= '0;
                end
            end

            if (refresh_valid && refresh_ready) begin
                residual_mem[refresh_branch][refresh_slot][refresh_index] <=
                    refresh_residual_fp16;
                if (refresh_last !== (refresh_index == BEATS-1)) begin
                    cache_error <= 1'b1;
                    refresh_active <= 1'b0;
                end else if (refresh_last) begin
                    refresh_active <= 1'b0;
                    head_slot[refresh_branch] <= refresh_slot;
                    if (anchor_count[refresh_branch] < 2)
                        anchor_count[refresh_branch] <=
                            anchor_count[refresh_branch] + 1'b1;
                    refresh_done <= 1'b1;
                end else begin
                    refresh_index <= refresh_index + 1'b1;
                end
            end

            if (reuse_start) begin
                if (refresh_active || read_active || anchor_count[branch_index] == 0) begin
                    cache_error <= 1'b1;
                end else begin
                    read_active <= 1'b1;
                    read_branch <= branch_index;
                    read_alpha_slot <= head_slot[branch_index];
                    read_beta_slot <= (anchor_count[branch_index] == 2) ?
                                      ~head_slot[branch_index] : head_slot[branch_index];
                    read_index <= '0;
                    gamma_latched <= gamma_fp16;
                end
            end

            if (read_valid_q && dcta_in_ready) read_valid_q <= 1'b0;
            if (read_issue) begin
                read_alpha_q <= residual_mem[read_branch][read_alpha_slot][read_index];
                read_beta_q <= residual_mem[read_branch][read_beta_slot][read_index];
                read_valid_q <= 1'b1;
                read_last_q <= (read_index == BEATS-1);
                if (read_index == BEATS-1) begin
                    read_active <= 1'b0;
                end else begin
                    read_index <= read_index + 1'b1;
                end
            end
        end
    end

    initial begin
        if (ELEMENTS <= 0 || LANES <= 0 || ELEMENTS % LANES != 0 ||
            BEATS != ELEMENTS / LANES)
            $error("DiCache residual cache geometry must be an integral beat count");
    end
endmodule

`default_nettype wire
