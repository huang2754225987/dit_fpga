// =============================================================================
// gld_ctrl.sv - GLD.W / GLD.A Load Controller (32-bank WRF + ARF version)
// =============================================================================
// GLD.W: reads 256-bit L1 words, writes 8 x 32-bit to 8 WRF banks per cycle.
//   K-aware banking: word (K, w) -> bank w, entry K.
//   FP16 (16 words/K, 8 per L1 word): 2 L1 words per K, bank_base cycles 0,1
//   INT3 (3 words/K, 8 per L1 word):  1 L1 word per K, bank_base=0 (3 of 8 used)
//
// GLD.A legacy mode reads 16 sequential K values per 256-bit L1 word.
// Batched mode transposes compact active route rows into K-major ARF banks;
// unused ARF rows are explicitly zeroed before variable-M GEMM execution.
// =============================================================================
`default_nettype none

module gld_ctrl #(
    parameter int L1_ADDR_W  = 11,
    parameter int L1_DATA_W  = 256,
    parameter int WRF_DEPTH  = 4096,  // 16 banks x 256 entries = 4096
    parameter int ARF_DEPTH  = 8192,  // 32 banks x 256 entries = 8192
    parameter int NUM_BANKS  = 32,
    parameter int BANK_DEPTH = 256
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire  [15:0]           cfg_K,
    input  wire  [15:0]           cfg_M,

    // GLD.W command
    input  wire                    gld_w_cmd_valid,
    input  wire  [15:0]           gld_w_cmd_addr,
    input  wire  [1:0]            gld_w_cmd_prec,
    // 1: multiple quantized K rows are packed into each 256-bit L1 line.
    // 0: legacy one-K-row-per-line image.
    input  wire                    gld_w_cmd_compact,
    output logic                   gld_w_done,

    // GLD.A command
    input  wire                    gld_a_cmd_valid,
    input  wire  [15:0]           gld_a_cmd_addr,
    input  wire                    gld_a_cmd_batch,
    // Token-major activation tile: transpose M x K into K-major ARF layout.
    input  wire                    gld_a_cmd_transpose,
    input  wire  [1:0]             gld_a_cmd_page,
    // Optional source-row stride in 32-byte L1 lines for transpose loads.
    input  wire  [8:0]             gld_a_cmd_stride_lines,
    output logic                   gld_a_done,

    // L1 cache read port
    output logic                   l1_rd_en,
    output logic [L1_ADDR_W-1:0]  l1_rd_addr,
    output logic [1:0]             l1_rd_page,
    input  wire  [L1_DATA_W-1:0]  l1_rd_data,
    input  wire                    l1_rd_valid,

    // Weight RF write port (8 banks per cycle)
    output logic                   wrf_wr_en,
    output logic [7:0]  wrf_wr_addr,       // entry index (K, 0..255)
    output logic [2:0]            wrf_wr_bank_base,  // bank group 0..3
    output logic [31:0]           wrf_wr_data0,
    output logic [31:0]           wrf_wr_data1,
    output logic [31:0]           wrf_wr_data2,
    output logic [31:0]           wrf_wr_data3,
    output logic [31:0]           wrf_wr_data4,
    output logic [31:0]           wrf_wr_data5,
    output logic [31:0]           wrf_wr_data6,
    output logic [31:0]           wrf_wr_data7,

    // Act RF write port (16 banks per cycle, from 256-bit L1 word)
    output logic                   arf_wr_en,
    output logic                   arf_wr_valid,
    output logic [7:0]  arf_wr_addr,       // entry index (K, 0..255)
    output logic [1:0]            arf_wr_bank_base,  // bank group 0..1
    output logic [15:0]           arf_wr_data0,
    output logic [15:0]           arf_wr_data1,
    output logic [15:0]           arf_wr_data2,
    output logic [15:0]           arf_wr_data3,
    output logic [15:0]           arf_wr_data4,
    output logic [15:0]           arf_wr_data5,
    output logic [15:0]           arf_wr_data6,
    output logic [15:0]           arf_wr_data7,
    output logic [15:0]           arf_wr_data8,
    output logic [15:0]           arf_wr_data9,
    output logic [15:0]           arf_wr_data10,
    output logic [15:0]           arf_wr_data11,
    output logic [15:0]           arf_wr_data12,
    output logic [15:0]           arf_wr_data13,
    output logic [15:0]           arf_wr_data14,
    output logic [15:0]           arf_wr_data15,
    output logic [15:0]           arf_wr_data16,
    output logic [15:0]           arf_wr_data17,
    output logic [15:0]           arf_wr_data18,
    output logic [15:0]           arf_wr_data19,
    output logic [15:0]           arf_wr_data20,
    output logic [15:0]           arf_wr_data21,
    output logic [15:0]           arf_wr_data22,
    output logic [15:0]           arf_wr_data23,
    output logic [15:0]           arf_wr_data24,
    output logic [15:0]           arf_wr_data25,
    output logic [15:0]           arf_wr_data26,
    output logic [15:0]           arf_wr_data27,
    output logic [15:0]           arf_wr_data28,
    output logic [15:0]           arf_wr_data29,
    output logic [15:0]           arf_wr_data30,
    output logic [15:0]           arf_wr_data31,
    output logic                   arf_wr_transpose,

    output logic                   busy
);

    typedef enum logic [3:0] {
        S_IDLE, S_W_READ, S_W_APPLY, S_W_WRITE, S_W_PREFETCH_WAIT,
        S_A_READ, S_A_APPLY, S_A_WRITE, S_AT_READ, S_AT_APPLY,
        S_AT_WRITE, S_DONE
    } state_t;

    state_t state, state_nxt;

    logic [L1_ADDR_W-1:0] r_l1_base;
    logic [L1_ADDR_W-1:0] r_l1_cnt;
    logic [1:0]           r_l1_page;
    logic [7:0]  r_wrf_entry;      // K entry 0..255
    logic [2:0]           r_wrf_bank_base;  // bank group 0..3
    logic [L1_DATA_W-1:0] r_l1_buf;
    logic [L1_DATA_W-1:0] r_w_prefetch_buf;
    logic                 r_w_prefetch_valid;
    logic                 r_is_weight;
    logic [1:0]           r_prec;
    logic                 r_compact;
    logic [2:0]           r_compact_k;
    logic [8:0]           r_k_count;
    wire                  w_fast_commit = (state == S_W_APPLY) && l1_rd_valid &&
                                           !(r_compact && r_prec != 2'b00);
    wire [L1_DATA_W-1:0]  w_commit_data = w_fast_commit ? l1_rd_data : r_l1_buf;
    // A non-transposed activation load can commit each synchronous L1
    // response directly to ARF and request the following line in the same
    // cycle. The bank and row selection for both legacy and batched loads is
    // derived from the current transfer counters, so neither needs r_l1_buf.
    wire                  a_fast_commit = (state == S_A_APPLY) && l1_rd_valid;
    wire [L1_DATA_W-1:0]  a_commit_data = a_fast_commit ? l1_rd_data : r_l1_buf;

    // Bidirectional 1-deep pending
    logic         r_a_pending;
    logic [15:0]  r_a_pending_addr;
    logic         r_a_pending_batch;
    logic         r_a_pending_transpose;
    logic [1:0]   r_a_pending_page;
    logic [8:0]   r_a_pending_stride_lines;
    logic         r_a_batch;
    logic [5:0]   r_a_m;
    logic         r_a_transpose;
    logic [8:0]   r_a_stride_lines;
    logic [7:0]   r_at_k;
    logic [8:0]   r_at_req_cnt;
    logic [L1_ADDR_W-1:0] r_at_row_base;
    logic [3:0]   r_at_col;
    logic         r_at_rd_pending;
    logic [15:0]  a_transpose_buf [0:31][0:255];

    function automatic logic [15:0] a_transpose_value(
        input logic [4:0] row,
        input logic [7:0] k
    );
        if (row < r_a_m)
            a_transpose_value = a_transpose_buf[row][k];
        else
            a_transpose_value = 16'd0;
    endfunction
    logic [5:0]   r_a_pending_m;
    logic         r_w_pending;
    logic [15:0]  r_w_pending_addr;
    logic [1:0]   r_w_pending_prec;
    logic         r_w_pending_compact;
    logic [8:0]   r_w_pending_k, r_a_pending_k;

    // Effective request at S_DONE: a queued pending OR a command arriving
    // exactly in the S_DONE cycle (chained directly, never queued).
    wire         a_req       = r_a_pending || gld_a_cmd_valid;
    wire         w_req       = r_w_pending || gld_w_cmd_valid;
    wire [15:0]  a_chain_addr = r_a_pending ? r_a_pending_addr : gld_a_cmd_addr;
    wire         a_chain_batch = r_a_pending ? r_a_pending_batch : gld_a_cmd_batch;
    wire         a_chain_transpose = r_a_pending ? r_a_pending_transpose : gld_a_cmd_transpose;
    wire [1:0]   a_chain_page = r_a_pending ? r_a_pending_page : gld_a_cmd_page;
    wire [8:0]   a_chain_stride_lines = r_a_pending
                                          ? r_a_pending_stride_lines
                                          : gld_a_cmd_stride_lines;
    wire [5:0]   a_chain_m = r_a_pending ? r_a_pending_m : cfg_M[5:0];
    wire [15:0]  w_chain_addr = r_w_pending ? r_w_pending_addr : gld_w_cmd_addr;
    wire [1:0]   w_chain_prec = r_w_pending ? r_w_pending_prec : gld_w_cmd_prec;
    wire         w_chain_compact = r_w_pending ? r_w_pending_compact : gld_w_cmd_compact;
    wire [8:0]   w_chain_k = r_w_pending ? r_w_pending_k : cfg_K[8:0];
    wire [8:0]   a_chain_k = r_a_pending ? r_a_pending_k : cfg_K[8:0];
    wire         a_chain_transpose_valid = !a_chain_transpose ||
                                           (a_chain_m >= 6'd1 && a_chain_m <= 6'd32 &&
                                            a_chain_k >= 9'd16 &&
                                            a_chain_k <= 9'd256 &&
                                            a_chain_k[3:0] == 4'd0);

    // Compact quantized lines hold 8/4/2 K rows for INT1/INT2/INT3.
    // Legacy quantized images occupy one padded line per K; FP16 occupies two.
    logic [8:0] w_l1_reads, a_l1_reads;
    logic [15:0] a_transpose_reads;
    logic [3:0] compact_k_per_line;
    always_comb begin
        compact_k_per_line = (r_prec == 2'b01) ? 4'd8 :
                             (r_prec == 2'b10) ? 4'd4 : 4'd2;
        if (r_prec == 2'b00)
            w_l1_reads = r_k_count << 1;
        else if (r_compact)
            w_l1_reads = (r_k_count + compact_k_per_line - 1) / compact_k_per_line;
        else
            w_l1_reads = r_k_count;
        a_l1_reads = r_a_batch
                   ? ((r_a_m > 16) ? (r_k_count << 1) : r_k_count)
                   : ((r_k_count + 9'd15) >> 4);
        a_transpose_reads = {10'd0, r_a_m} * (r_k_count >> 4);
        if (w_l1_reads == 0) w_l1_reads = 9'd1;
        if (a_l1_reads == 0) a_l1_reads = 9'd1;
    end

    wire at_response_valid =
        (state == S_AT_READ || state == S_AT_APPLY) &&
        r_at_rd_pending && l1_rd_valid &&
        (r_l1_cnt < L1_ADDR_W'(a_transpose_reads));

    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) state <= S_IDLE;
        else        state <= state_nxt;

    always_comb begin
        state_nxt = state;
        case (state)
            S_IDLE: begin
                if (gld_w_cmd_valid)      state_nxt = S_W_READ;
                else if (gld_a_cmd_valid) begin
                    if (gld_a_cmd_transpose && cfg_M >= 16'd1 && cfg_M <= 16'd32 &&
                        cfg_K >= 16'd16 && cfg_K <= 16'd256 && cfg_K[3:0] == 0)
                        state_nxt = S_AT_READ;
                    else
                        state_nxt = gld_a_cmd_transpose ? S_DONE : S_A_READ;
                end
            end
            S_W_READ:  state_nxt = S_W_APPLY;
            S_W_APPLY: begin
                if (l1_rd_valid) begin
                    if (r_compact && r_prec != 2'b00)
                        state_nxt = S_W_WRITE;
                    else if (r_l1_cnt == L1_ADDR_W'(w_l1_reads - 1))
                        state_nxt = S_DONE;
                    else
                        // Non-compact formats request the next synchronous
                        // L1 line while committing this response. Keep the
                        // apply state across the pipelined responses.
                        state_nxt = S_W_APPLY;
                end
            end
            S_W_WRITE: begin
                if (r_compact && r_prec != 2'b00 && {2'b0, r_wrf_entry} == r_k_count - 1)
                    state_nxt = S_DONE;
                else if (r_compact && r_prec != 2'b00 && r_compact_k != compact_k_per_line - 1)
                    state_nxt = S_W_WRITE;
                else if (r_compact && r_prec != 2'b00 &&
                         r_l1_cnt < L1_ADDR_W'(w_l1_reads - 1)) begin
                    if (r_w_prefetch_valid || l1_rd_valid)
                        state_nxt = S_W_WRITE;
                    else
                        state_nxt = S_W_PREFETCH_WAIT;
                end
                else if (r_l1_cnt == L1_ADDR_W'(w_l1_reads - 1))
                    state_nxt = S_DONE;
                else
                    state_nxt = S_W_READ;
            end
            S_W_PREFETCH_WAIT: if (l1_rd_valid) state_nxt = S_W_WRITE;
            S_A_READ:  state_nxt = S_A_APPLY;
            S_A_APPLY: begin
                if (a_fast_commit) begin
                    if (r_l1_cnt == L1_ADDR_W'(a_l1_reads - 1))
                        state_nxt = S_DONE;
                    else
                        state_nxt = S_A_APPLY;
                end else if (l1_rd_valid) begin
                    state_nxt = S_A_WRITE;
                end
            end
            S_A_WRITE: begin
                if (r_l1_cnt == L1_ADDR_W'(a_l1_reads - 1))
                    state_nxt = S_DONE;
                else
                    state_nxt = S_A_READ;
            end
            // L1 is a one-cycle synchronous memory. Keep issuing one line per
            // cycle, then drain the final registered response in S_AT_APPLY.
            S_AT_READ: begin
                if ({7'd0, r_at_req_cnt} == a_transpose_reads - 1'b1)
                    state_nxt = S_AT_APPLY;
            end
            S_AT_APPLY: begin
                if (at_response_valid) begin
                    if ({7'd0, r_l1_cnt} == a_transpose_reads - 1'b1)
                        state_nxt = S_AT_WRITE;
                end
            end
            S_AT_WRITE: begin
                if (r_at_k == r_k_count - 1)
                    state_nxt = S_DONE;
            end
            S_DONE: begin
                if (r_is_weight && a_req)
                    state_nxt = a_chain_transpose_valid
                              ? (a_chain_transpose ? S_AT_READ : S_A_READ) : S_IDLE;
                else if (!r_is_weight && w_req)  state_nxt = S_W_READ;
                else if (a_req)
                    state_nxt = a_chain_transpose_valid
                              ? (a_chain_transpose ? S_AT_READ : S_A_READ) : S_IDLE;
                else if (w_req)                  state_nxt = S_W_READ;
                else                             state_nxt = S_IDLE;
            end
            default: state_nxt = S_IDLE;
        endcase
    end

    // Latch command parameters
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_l1_base <= '0;
            r_l1_cnt  <= '0;
            r_is_weight <= 1'b0;
            r_prec <= 2'b00;
            r_compact <= 1'b0;
            r_compact_k <= '0;
            r_k_count <= '0;
            r_wrf_entry <= '0;
            r_wrf_bank_base <= '0;
            r_a_batch <= 1'b0;
            r_a_m <= 6'd1;
            r_a_transpose <= 1'b0;
            r_a_stride_lines <= '0;
            r_l1_page <= 1'b0;
            r_at_k <= '0;
        end else if (state == S_IDLE) begin
            r_k_count <= cfg_K[8:0];
            if (gld_w_cmd_valid) begin
                r_l1_base   <= gld_w_cmd_addr[L1_ADDR_W+4:5];
                r_l1_page  <= 1'b0;
                r_l1_cnt    <= '0;
                r_is_weight <= 1'b1;
                r_prec      <= gld_w_cmd_prec;
                r_compact   <= gld_w_cmd_compact;
                r_compact_k <= '0;
                r_wrf_entry <= '0;
                r_wrf_bank_base <= '0;
            end else if (gld_a_cmd_valid) begin
                r_l1_base   <= gld_a_cmd_addr[L1_ADDR_W+4:5];
                r_l1_page  <= gld_a_cmd_page;
                r_l1_cnt    <= '0;
                r_is_weight <= 1'b0;
                r_prec      <= 2'b00;
                r_a_batch  <= gld_a_cmd_batch;
                r_a_m      <= cfg_M[5:0];
                r_a_transpose <= gld_a_cmd_transpose;
                r_a_stride_lines <= gld_a_cmd_stride_lines;
                r_at_k <= '0;
            end
        end else if (state == S_DONE) begin
            if (r_is_weight && a_req) begin
                r_k_count <= a_chain_k;
                r_l1_base   <= a_chain_addr[L1_ADDR_W+4:5];
                r_l1_cnt    <= '0;
                r_is_weight <= 1'b0;
                r_a_batch  <= a_chain_batch;
                r_a_m      <= a_chain_m;
                r_a_transpose <= a_chain_transpose;
                r_a_stride_lines <= a_chain_stride_lines;
                r_l1_page <= a_chain_page;
                r_at_k <= '0;
            end else if (!r_is_weight && w_req) begin
                r_k_count <= w_chain_k;
                r_l1_base   <= w_chain_addr[L1_ADDR_W+4:5];
                r_l1_page  <= 1'b0;
                r_l1_cnt    <= '0;
                r_is_weight <= 1'b1;
                r_prec      <= w_chain_prec;
                r_compact   <= w_chain_compact;
                r_compact_k <= '0;
                r_wrf_entry <= '0;
                r_wrf_bank_base <= '0;
            end else if (a_req) begin
                r_k_count <= a_chain_k;
                r_l1_base   <= a_chain_addr[L1_ADDR_W+4:5];
                r_l1_page  <= a_chain_page;
                r_l1_cnt    <= '0;
                r_is_weight <= 1'b0;
                r_a_batch  <= a_chain_batch;
                r_a_m      <= a_chain_m;
                r_a_transpose <= a_chain_transpose;
                r_a_stride_lines <= a_chain_stride_lines;
                r_at_k <= '0;
            end else if (w_req) begin
                r_k_count <= w_chain_k;
                r_l1_base   <= w_chain_addr[L1_ADDR_W+4:5];
                r_l1_page  <= 1'b0;
                r_l1_cnt    <= '0;
                r_is_weight <= 1'b1;
                r_prec      <= w_chain_prec;
                r_compact   <= w_chain_compact;
                r_compact_k <= '0;
                r_wrf_entry <= '0;
                r_wrf_bank_base <= '0;
            end
        end else if (state == S_W_WRITE || w_fast_commit) begin
            if (!(r_compact && r_prec != 2'b00) || r_compact_k == compact_k_per_line - 1)
                r_l1_cnt <= r_l1_cnt + 1;
            // FP16: bank_base cycles 0,1,0,1,...
            //       entry (K) increments every 2 L1 words
            // INT:  bank_base stays 0, entry (K) increments every L1 word
            if (r_prec == 2'b00) begin
                // FP16
                if (r_wrf_bank_base == 3'd1) begin
                    r_wrf_bank_base <= '0;
                    r_wrf_entry <= r_wrf_entry + 8'd1;
                end else begin
                    r_wrf_bank_base <= r_wrf_bank_base + 3'd1;
                end
            end else begin
                // INT: compact mode expands several K rows from one L1 line.
                r_wrf_entry <= r_wrf_entry + 8'd1;
                r_wrf_bank_base <= '0;
                if (r_compact) begin
                    if (r_compact_k == compact_k_per_line - 1)
                        r_compact_k <= '0;
                    else
                        r_compact_k <= r_compact_k + 3'd1;
                end
            end
        end else if (state == S_A_WRITE || a_fast_commit) begin
            r_l1_cnt <= r_l1_cnt + 1;
        end else if (at_response_valid) begin
            for (int lane = 0; lane < 16; lane++)
                a_transpose_buf[r_l1_cnt / (r_k_count >> 4)]
                               [(r_l1_cnt % (r_k_count >> 4)) * 16 + lane]
                    <= l1_rd_data[lane * 16 +: 16];
            r_l1_cnt <= r_l1_cnt + 1;
        end else if (state == S_AT_WRITE) begin
            if (r_at_k < r_k_count - 1'b1)
                r_at_k <= r_at_k + 1'b1;
        end
    end

    // Compact INT lines expand over multiple WRF writes. Fetch the following
    // line during that expansion into a second buffer; swap only after the
    // final K row from the current line has been committed.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_w_prefetch_buf <= '0;
            r_w_prefetch_valid <= 1'b0;
        end else if ((state == S_IDLE && gld_w_cmd_valid) ||
                     (state == S_DONE && !r_is_weight && w_req)) begin
            r_w_prefetch_valid <= 1'b0;
        end else if (state == S_W_WRITE && r_compact && r_prec != 2'b00 &&
                     r_compact_k == compact_k_per_line - 1 &&
                     r_l1_cnt < L1_ADDR_W'(w_l1_reads - 1) &&
                     (r_w_prefetch_valid || l1_rd_valid)) begin
            r_w_prefetch_valid <= 1'b0;
        end else if (state == S_W_WRITE && r_compact && r_prec != 2'b00 &&
                     l1_rd_valid) begin
            r_w_prefetch_buf <= l1_rd_data;
            r_w_prefetch_valid <= 1'b1;
        end
    end

    // Issue one synchronous L1 request per cycle while independently counting
    // returned lines in r_l1_cnt. The last response is drained in S_AT_APPLY.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_at_req_cnt <= '0;
            r_at_row_base <= '0;
            r_at_col <= '0;
        end else if ((state == S_IDLE && gld_a_cmd_valid &&
                      gld_a_cmd_transpose) ||
                     (state == S_DONE && a_req &&
                      (r_is_weight || !w_req) && a_chain_transpose)) begin
            r_at_req_cnt <= '0;
            r_at_row_base <= '0;
            r_at_col <= '0;
        end else if (state == S_AT_READ &&
                     {7'd0, r_at_req_cnt} < a_transpose_reads) begin
            r_at_req_cnt <= r_at_req_cnt + 9'd1;
            if ({5'd0, r_at_col} + 9'd1 >= (r_k_count >> 4)) begin
                r_at_col <= '0;
                r_at_row_base <= r_at_row_base +
                    (r_a_stride_lines != 0 ? r_a_stride_lines :
                                               (r_k_count >> 4));
            end else begin
                r_at_col <= r_at_col + 4'd1;
            end
        end
    end

    // l1_rd_valid is shared with other L1 clients. Qualify it with this
    // controller's one-cycle request history so a response already in flight
    // when GLD.A starts cannot be mistaken for transpose line zero.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            r_at_rd_pending <= 1'b0;
        else
            r_at_rd_pending <= (state == S_AT_READ);
    end

    // Bidirectional pending logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_a_pending      <= 1'b0;
            r_a_pending_addr <= '0;
            r_a_pending_batch <= 1'b0;
            r_a_pending_transpose <= 1'b0;
            r_a_pending_page <= 1'b0;
            r_a_pending_stride_lines <= '0;
            r_a_pending_m <= 6'd1;
            r_w_pending      <= 1'b0;
            r_w_pending_addr <= '0;
            r_w_pending_prec <= '0;
            r_w_pending_compact <= 1'b0;
            r_w_pending_k <= '0;
            r_a_pending_k <= '0;
        end else begin
            if (state == S_DONE) begin
                if (r_is_weight && r_a_pending)
                    r_a_pending <= 1'b0;
                else if (!r_is_weight && r_w_pending)
                    r_w_pending <= 1'b0;
                else if (r_a_pending)
                    r_a_pending <= 1'b0;
                else if (r_w_pending)
                    r_w_pending <= 1'b0;
            end
            // Capture a command of the OTHER direction whenever this module is
            // busy executing (READ/APPLY/WRITE), or when both commands arrive
            // in the same IDLE cycle (the loser queues as pending). A command
            // arriving exactly in the S_DONE cycle is chained directly by the
            // S_DONE branch logic below (treated like an already-queued
            // pending), so it must NOT also be captured here.
            if (gld_a_cmd_valid &&
                (state == S_W_READ || state == S_W_APPLY || state == S_W_WRITE ||
                 state == S_W_PREFETCH_WAIT ||
                 (state == S_AT_READ || state == S_AT_APPLY || state == S_AT_WRITE) ||
                 (state == S_IDLE && gld_w_cmd_valid))) begin
                if (!r_a_pending) begin
                    r_a_pending      <= 1'b1;
                    r_a_pending_addr <= gld_a_cmd_addr;
                    r_a_pending_batch <= gld_a_cmd_batch;
                    r_a_pending_transpose <= gld_a_cmd_transpose;
                    r_a_pending_page <= gld_a_cmd_page;
                    r_a_pending_stride_lines <= gld_a_cmd_stride_lines;
                    r_a_pending_m <= cfg_M[5:0];
                    r_a_pending_k <= cfg_K[8:0];
                end
            end
            if (gld_w_cmd_valid &&
                (state == S_A_READ || state == S_A_APPLY || state == S_A_WRITE ||
                 state == S_AT_READ || state == S_AT_APPLY || state == S_AT_WRITE ||
                 (state == S_IDLE && gld_a_cmd_valid))) begin
                if (!r_w_pending) begin
                    r_w_pending      <= 1'b1;
                    r_w_pending_addr <= gld_w_cmd_addr;
                    r_w_pending_prec <= gld_w_cmd_prec;
                    r_w_pending_compact <= gld_w_cmd_compact;
                    r_w_pending_k <= cfg_K[8:0];
                end
            end
        end
    end

    // Latch L1 data
    always_ff @(posedge clk) begin
        if (l1_rd_valid) begin
            if (!(state == S_W_WRITE && r_compact && r_prec != 2'b00))
                r_l1_buf <= l1_rd_data;
        end
        if (state == S_W_PREFETCH_WAIT && l1_rd_valid)
            r_l1_buf <= l1_rd_data;
        else if (state == S_W_WRITE && r_compact && r_prec != 2'b00 &&
                 r_compact_k == compact_k_per_line - 1 &&
                 r_l1_cnt < L1_ADDR_W'(w_l1_reads - 1)) begin
            if (l1_rd_valid)
                r_l1_buf <= l1_rd_data;
            else if (r_w_prefetch_valid)
                r_l1_buf <= r_w_prefetch_buf;
        end
    end

    // Non-compact weight loads can commit the synchronous L1 response
    // directly in S_W_APPLY. This removes the separate S_W_WRITE cycle while
    // retaining the existing multi-cycle expansion for compact INT lines.
    // L1 read control
    assign l1_rd_en   = (state == S_W_READ || state == S_A_READ ||
                         state == S_AT_READ ||
                         (state == S_W_APPLY && w_fast_commit &&
                          r_l1_cnt < L1_ADDR_W'(w_l1_reads - 1)) ||
                         (a_fast_commit &&
                          r_l1_cnt < L1_ADDR_W'(a_l1_reads - 1)) ||
                         (state == S_W_WRITE && r_compact && r_prec != 2'b00 &&
                          r_compact_k == 0 &&
                          r_l1_cnt < L1_ADDR_W'(w_l1_reads - 1)));
    assign l1_rd_addr = (state == S_AT_READ)
                      ? r_l1_base + L1_ADDR_W'(r_at_row_base) +
                        L1_ADDR_W'(r_at_col)
                      : ((state == S_W_WRITE && r_compact && r_prec != 2'b00) ||
                         (state == S_W_APPLY && w_fast_commit &&
                          !(r_compact && r_prec != 2'b00)) ||
                         a_fast_commit)
                      ? r_l1_base + r_l1_cnt + L1_ADDR_W'(1)
                      : r_l1_base + r_l1_cnt;
    assign l1_rd_page = r_l1_page;

    // Weight RF write: 8 banks in parallel from 256-bit L1 word
    assign wrf_wr_en        = (state == S_W_WRITE) || w_fast_commit;
    assign wrf_wr_bank_base = r_wrf_bank_base;
    assign wrf_wr_addr      = r_wrf_entry;

    // Data from L1 buffer (256-bit = 8 x 32-bit)
    always_comb begin
        wrf_wr_data0 = w_commit_data[31:0];
        wrf_wr_data1 = w_commit_data[63:32];
        wrf_wr_data2 = w_commit_data[95:64];
        wrf_wr_data3 = w_commit_data[127:96];
        wrf_wr_data4 = w_commit_data[159:128];
        wrf_wr_data5 = w_commit_data[191:160];
        wrf_wr_data6 = w_commit_data[223:192];
        wrf_wr_data7 = w_commit_data[255:224];
        if (r_compact && r_prec != 2'b00) begin
            // Each K contributes nbits*32 bits: one word for INT1, two for
            // INT2, three for INT3.  Unused WRF banks are written as zero;
            // GEMM only consumes the precision-selected bank count.
            wrf_wr_data0 = w_commit_data[r_compact_k * r_prec * 32 +: 32];
            wrf_wr_data1 = (r_prec >= 2'b10) ? w_commit_data[r_compact_k * r_prec * 32 + 32 +: 32] : '0;
            wrf_wr_data2 = (r_prec == 2'b11) ? w_commit_data[r_compact_k * r_prec * 32 + 64 +: 32] : '0;
            wrf_wr_data3 = '0;
            wrf_wr_data4 = '0;
            wrf_wr_data5 = '0;
            wrf_wr_data6 = '0;
            wrf_wr_data7 = '0;
        end
    end

    // Act RF write: 16 sequential K values per 256-bit line.
    assign arf_wr_en        = (state == S_A_WRITE || state == S_AT_WRITE ||
                               a_fast_commit);
    assign arf_wr_valid     = (state == S_A_WRITE || state == S_AT_WRITE ||
                               a_fast_commit);
    assign arf_wr_addr      = r_a_transpose ? r_at_k : r_a_batch && r_a_m > 16
                            ? r_l1_cnt[8:1] : r_l1_cnt[7:0];
    assign arf_wr_bank_base = r_a_transpose ? 2'd0 : r_a_batch
                            ? {1'b0, (r_a_m > 16 && r_l1_cnt[0])}
                            : 2'd0;
    assign arf_wr_transpose = r_a_transpose;

    // Data from L1 buffer (256-bit = 16 x 16-bit)
    always_comb begin
        arf_wr_data0  = r_a_transpose ? a_transpose_value(5'd0, r_at_k) : a_commit_data[15:0];
        arf_wr_data1  = r_a_transpose ? a_transpose_value(5'd1, r_at_k) : a_commit_data[31:16];
        arf_wr_data2  = r_a_transpose ? a_transpose_value(5'd2, r_at_k) : a_commit_data[47:32];
        arf_wr_data3  = r_a_transpose ? a_transpose_value(5'd3, r_at_k) : a_commit_data[63:48];
        arf_wr_data4  = r_a_transpose ? a_transpose_value(5'd4, r_at_k) : a_commit_data[79:64];
        arf_wr_data5  = r_a_transpose ? a_transpose_value(5'd5, r_at_k) : a_commit_data[95:80];
        arf_wr_data6  = r_a_transpose ? a_transpose_value(5'd6, r_at_k) : a_commit_data[111:96];
        arf_wr_data7  = r_a_transpose ? a_transpose_value(5'd7, r_at_k) : a_commit_data[127:112];
        arf_wr_data8  = r_a_transpose ? a_transpose_value(5'd8, r_at_k) : a_commit_data[143:128];
        arf_wr_data9  = r_a_transpose ? a_transpose_value(5'd9, r_at_k) : a_commit_data[159:144];
        arf_wr_data10 = r_a_transpose ? a_transpose_value(5'd10, r_at_k) : a_commit_data[175:160];
        arf_wr_data11 = r_a_transpose ? a_transpose_value(5'd11, r_at_k) : a_commit_data[191:176];
        arf_wr_data12 = r_a_transpose ? a_transpose_value(5'd12, r_at_k) : a_commit_data[207:192];
        arf_wr_data13 = r_a_transpose ? a_transpose_value(5'd13, r_at_k) : a_commit_data[223:208];
        arf_wr_data14 = r_a_transpose ? a_transpose_value(5'd14, r_at_k) : a_commit_data[239:224];
        arf_wr_data15 = r_a_transpose ? a_transpose_value(5'd15, r_at_k) : a_commit_data[255:240];
        arf_wr_data16 = r_a_transpose ? a_transpose_value(5'd16, r_at_k) : 16'd0;
        arf_wr_data17 = r_a_transpose ? a_transpose_value(5'd17, r_at_k) : 16'd0;
        arf_wr_data18 = r_a_transpose ? a_transpose_value(5'd18, r_at_k) : 16'd0;
        arf_wr_data19 = r_a_transpose ? a_transpose_value(5'd19, r_at_k) : 16'd0;
        arf_wr_data20 = r_a_transpose ? a_transpose_value(5'd20, r_at_k) : 16'd0;
        arf_wr_data21 = r_a_transpose ? a_transpose_value(5'd21, r_at_k) : 16'd0;
        arf_wr_data22 = r_a_transpose ? a_transpose_value(5'd22, r_at_k) : 16'd0;
        arf_wr_data23 = r_a_transpose ? a_transpose_value(5'd23, r_at_k) : 16'd0;
        arf_wr_data24 = r_a_transpose ? a_transpose_value(5'd24, r_at_k) : 16'd0;
        arf_wr_data25 = r_a_transpose ? a_transpose_value(5'd25, r_at_k) : 16'd0;
        arf_wr_data26 = r_a_transpose ? a_transpose_value(5'd26, r_at_k) : 16'd0;
        arf_wr_data27 = r_a_transpose ? a_transpose_value(5'd27, r_at_k) : 16'd0;
        arf_wr_data28 = r_a_transpose ? a_transpose_value(5'd28, r_at_k) : 16'd0;
        arf_wr_data29 = r_a_transpose ? a_transpose_value(5'd29, r_at_k) : 16'd0;
        arf_wr_data30 = r_a_transpose ? a_transpose_value(5'd30, r_at_k) : 16'd0;
        arf_wr_data31 = r_a_transpose ? a_transpose_value(5'd31, r_at_k) : 16'd0;
    end

    // Done signals
    assign gld_w_done = (state == S_DONE) && r_is_weight;
    assign gld_a_done = (state == S_DONE) && !r_is_weight;
    assign busy       = (state != S_IDLE);

endmodule

`default_nettype wire
