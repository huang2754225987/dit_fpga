`default_nettype none

// Clocked RTL replay of the list-scheduled Supra2 block pipeline.
//
// The schedule table (build/pipeline_sim/schedule_table.hex, produced by
// scripts/simulate_block_pipeline.py) holds one task per line as four 32-bit
// words: resource_id, start_cycle, duration, dep_ready_cycle
// (dep_ready = max finish over the task's dependencies).
//
// This module walks each resource's task list in start order and checks, in
// RTL simulation semantics:
//   1. start >= dep_ready              (dependency feasibility)
//   2. start >= previous finish        (no double-booked resource)
// It re-derives the block makespan and per-resource busy cycle counts and
// exposes per-resource busy strobes, so a future datapath can hang real
// operators on the same schedule. Control-plane replay only: no datapath.
module supra2_block_schedule_replay #(
    parameter int MAXTASK   = 1024,
    parameter int NRES      = 7,
    parameter     TABLE_FILE = "build/pipeline_sim/schedule_table.hex"
) (
    input  wire logic     clk,
    input  wire logic     rst_n,
    output logic [NRES-1:0] res_busy,        // per-resource busy strobe
    output logic [NRES-1:0] res_error,       // per-resource check violation
    output logic          replay_done,
    output logic [31:0]   makespan_cycles,
    output logic [31:0]   busy_cycles [0:NRES-1]
);
    localparam int WORDS = 4;

    // flat table: 4 words per task, (start,resource)-sorted input
    logic [31:0] tbl [0:MAXTASK*WORDS-1];
    integer ntask;
    string table_file;

    // per-resource cursors
    integer res_base  [0:NRES-1];   // first task index of this resource
    integer res_count [0:NRES-1];
    integer res_idx   [0:NRES-1];
    logic [31:0] res_end [0:NRES-1];    // finish of last accepted task
    logic [31:0] cycle;
    logic [31:0] max_end;
    integer accepted;

    initial begin
        ntask = 0;
        table_file = TABLE_FILE;
        if ($value$plusargs("NTASK=%d", ntask) == 0 || ntask <= 0 || ntask > MAXTASK)
            $fatal(1, "Missing/invalid NTASK plusarg (1..%0d)", MAXTASK);
        void'($value$plusargs("TABLE_FILE=%s", table_file));
        // table layout: NRES header words with per-resource task counts,
        // then each resource's tasks contiguous, sorted by start cycle
        $readmemh(table_file, tbl);
        res_base[0] = 0;
        res_count[0] = tbl[0];
        for (int r = 1; r < NRES; r++) begin
            res_count[r] = tbl[r];
            res_base[r] = res_base[r-1] + res_count[r-1];
            res_idx[r] = 0;
        end
    end

    // per-resource sequential acceptance and busy generation
    // word index of a task = NRES header words + task_index*WORDS
    for (genvar r = 0; r < NRES; r++) begin : g_res
        wire [31:0] t_start = tbl[NRES + (res_base[r] + res_idx[r])*WORDS + 1];
        wire [31:0] t_dur   = tbl[NRES + (res_base[r] + res_idx[r])*WORDS + 2];
        wire [31:0] t_dep   = tbl[NRES + (res_base[r] + res_idx[r])*WORDS + 3];

        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                res_idx[r] <= 0;
                res_end[r] <= 0;
                busy_cycles[r] <= 0;
                res_busy[r] <= 1'b0;
                res_error[r] <= 1'b0;
            end else if (!replay_done) begin
                if (res_idx[r] < res_count[r]) begin
                    if (cycle == t_start) begin
                        if (t_start < t_dep || t_start < res_end[r])
                            res_error[r] <= 1'b1;
                        res_busy[r] <= 1'b1;
                        busy_cycles[r] <= busy_cycles[r] + t_dur;
                        res_end[r] <= t_start + t_dur;
                        res_idx[r] <= res_idx[r] + 1;
                    end
                end else begin
                    res_busy[r] <= 1'b0;
                end
            end
        end
    end

    // completion tracking: all tasks accepted and every resource idle
    always_comb begin
        accepted = 0;
        max_end = 0;
        for (int r = 0; r < NRES; r++) begin
            accepted = accepted + res_idx[r];
            if (res_end[r] > max_end) max_end = res_end[r];
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cycle <= 0;
            replay_done <= 1'b0;
            makespan_cycles <= 0;
        end else if (!replay_done) begin
            cycle <= cycle + 1;
            if (accepted == ntask && ntask > 0 && cycle >= max_end) begin
                makespan_cycles <= max_end;
                replay_done <= 1'b1;
            end
            if (cycle > 40_000_000) begin
                $display("REPLAY_TIMEOUT cycle=%0d accepted=%0d ntask=%0d",
                         cycle, accepted, ntask);
                replay_done <= 1'b1;
            end
        end
    end

endmodule

`default_nettype wire
