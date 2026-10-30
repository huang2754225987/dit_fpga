`default_nettype none

// DiCache online probe profiling policy.
//
// The probe datapath supplies the relative-L1 estimate in unsigned Q16.16:
//   ||probe_t - probe_(t+1)||_1 / ||probe_(t+1)||_1
// Accumulation and cache state are independent for CFG's conditional and
// unconditional branches. Full-residual storage/reconstruction is a separate
// datapath and must acknowledge a refresh through cache_refresh_done.
module supra2_dicache_online_policy (
    input  wire logic        clk,
    input  wire logic        rst_n,
    input  wire logic        run_start,
    input  wire logic        enable,
    input  wire logic        branch_index,
    input  wire logic        probe_valid,
    input  wire logic [31:0] probe_error_q16,
    input  wire logic [31:0] threshold_q16,
    input  wire logic        cache_refresh_done,
    output logic             decision_valid,
    output logic             reuse_residual,
    output logic             refresh_residual,
    output logic [31:0]      accumulated_error_q16,
    output logic             cache_valid
);
    logic [31:0] error_sum [0:1];
    logic [1:0] residual_valid;
    logic [32:0] error_sum_next;

    always_comb begin
        error_sum_next = {1'b0, error_sum[branch_index]} +
                         {1'b0, probe_error_q16};
    end

    assign accumulated_error_q16 = error_sum[branch_index];
    assign cache_valid = residual_valid[branch_index];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            error_sum[0] <= '0;
            error_sum[1] <= '0;
            residual_valid <= '0;
            decision_valid <= 1'b0;
            reuse_residual <= 1'b0;
            refresh_residual <= 1'b0;
        end else begin
            decision_valid <= 1'b0;
            reuse_residual <= 1'b0;
            refresh_residual <= 1'b0;

            if (run_start) begin
                error_sum[0] <= '0;
                error_sum[1] <= '0;
                residual_valid <= '0;
            end

            if (cache_refresh_done) begin
                residual_valid[branch_index] <= 1'b1;
                error_sum[branch_index] <= '0;
            end

            if (probe_valid) begin
                decision_valid <= 1'b1;
                if (!enable || !residual_valid[branch_index]) begin
                    // Disabled mode is exact recomputation; first use always
                    // seeds the residual cache before any reuse is allowed.
                    refresh_residual <= 1'b1;
                    error_sum[branch_index] <= '0;
                end else if (!error_sum_next[32] &&
                             error_sum_next[31:0] <= threshold_q16) begin
                    reuse_residual <= 1'b1;
                    error_sum[branch_index] <= error_sum_next[31:0];
                end else begin
                    refresh_residual <= 1'b1;
                    error_sum[branch_index] <= '0;
                end
            end
        end
    end
endmodule

`default_nettype wire
