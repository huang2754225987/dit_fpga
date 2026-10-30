// =============================================================================
// ucode_fetch.sv - 32-bit Instruction Fetch Unit
// =============================================================================
// Fetches 32-bit instructions from instruction SRAM and presents them to the
// decode stage with backpressure support.
// =============================================================================

module ucode_fetch
  import npu_pkg::*;
  import isa_pkg::*;
#(
    parameter int INSTR_W     = 32,
    parameter int SRAM_ADDR_W = 14
) (
    input  logic                    clk,
    input  logic                    rst_n,

    // --- Control interface (from AXI-Lite regs) ---
    input  logic                    start,
    input  logic                    stop,
    input  logic [SRAM_ADDR_W-1:0] ucode_base_addr,
    input  logic [SRAM_ADDR_W-1:0] ucode_len,
    input  logic [SRAM_ADDR_W-1:0] resume_pc,   // host-in-the-loop: start from this PC

    // --- Instruction SRAM read interface (32-bit wide) ---
    output logic                    rd_en,
    output logic [SRAM_ADDR_W-1:0] rd_addr,
    input  logic [INSTR_W-1:0]     rd_data,
    input  logic                    rd_valid,

    // --- To decode stage ---
    output logic                    instr_valid,
    input  logic                    instr_ready,
    output logic [INSTR_W-1:0]     instr_data,
    output logic [SRAM_ADDR_W-1:0] pc,

    // --- Loop interface (to decode stage) ---
    input  logic                    loop_cmd_valid,    // decode says: current instr is LOOP
    input  logic [3:0]              loop_body_len,     // body length (1-15)
    input  logic [11:0]             loop_count_init,   // iteration count
    input  logic [1:0]              loop_stride_code,  // stride code
    input  logic                    token_loop_begin,
    input  logic                    token_loop_end,
    input  logic [5:0]              token_loop_count,
    input  logic                    expert_loop_begin,
    input  logic                    expert_loop_end,
    input  logic [5:0]              expert_loop_count,
    input  logic                    source_skip_valid,
    input  logic [15:0]             source_skip_count,

    // --- Loop state (to decode stage, for DMA auto-increment) ---
    output logic                    loop_active,
    output logic [15:0]            loop_iter,
    output logic [15:0]            loop_remaining,
    output logic [15:0]            addr_stride,       // stride value in imm units
    output logic [5:0]             token_iter,
    output logic [5:0]             expert_iter,

    // --- Status ---
    output logic                    done,
    output logic                    busy
);

    typedef enum logic [2:0] {
        S_IDLE      = 3'd0,
        S_FETCH     = 3'd1,
        S_WAIT_DATA = 3'd2,
        S_DISPATCH  = 3'd3,
        S_INCREMENT = 3'd4,
        S_DONE      = 3'd5
    } fetch_state_e;

    fetch_state_e state_q /* verilator public */, state_d;

    logic [SRAM_ADDR_W-1:0] pc_q /* verilator public */,        pc_d;
    logic [SRAM_ADDR_W-1:0] base_q,      base_d;
    logic [SRAM_ADDR_W-1:0] len_q /* verilator public */,       len_d;
    logic [INSTR_W-1:0]     instr_buf_q, instr_buf_d;
    logic                    done_q,      done_d;

    // OP_END detection: opcode is bits [31:28] in 32-bit format
    logic fetched_is_end;
    assign fetched_is_end = (rd_data[31:28] == OP_END);

    logic [SRAM_ADDR_W-1:0] pc_plus1;
    assign pc_plus1 = pc_q + {{(SRAM_ADDR_W-1){1'b0}}, 1'b1};

    logic [SRAM_ADDR_W-1:0] dispatch_target;
    logic dispatch_terminal;
    logic dispatch_hw_loop_boundary;
    logic dispatch_hw_loop_repeat;

    localparam int SKIP_SUM_W = (SRAM_ADDR_W > 16) ? SRAM_ADDR_W + 1 : 17;
    logic [SKIP_SUM_W-1:0] source_skip_target;
    always_comb begin
        source_skip_target = {{(SKIP_SUM_W-SRAM_ADDR_W){1'b0}}, pc_plus1} +
                             {{(SKIP_SUM_W-16){1'b0}}, source_skip_count};
    end

    // =========================================================================
    // Hardware Loop Registers
    // =========================================================================
    logic [SRAM_ADDR_W-1:0] loop_start_q, loop_start_d;
    logic [SRAM_ADDR_W-1:0] loop_end_q,   loop_end_d;
    logic [15:0]            loop_count_q, loop_count_d;
    logic [15:0]            loop_iter_q,  loop_iter_d;
    logic                   loop_active_q, loop_active_d;
    logic [15:0]            addr_stride_q, addr_stride_d;
    logic [SRAM_ADDR_W-1:0] token_start_q, token_start_d;
    logic [5:0] token_count_q, token_count_d, token_iter_q, token_iter_d;
    logic token_active_q, token_active_d;
    logic [SRAM_ADDR_W-1:0] expert_start_q, expert_start_d;
    logic [5:0] expert_count_q, expert_count_d, expert_iter_q, expert_iter_d;
    logic expert_active_q, expert_active_d;

    // Resolve the next instruction address while the current instruction is
    // accepted. This lets the synchronous SRAM request launch on that same
    // edge, removing the standalone increment cycle from the common path.
    // Redirect priority matches the original dispatch/increment sequencing.
    always_comb begin
        dispatch_target = pc_plus1;
        dispatch_terminal = 1'b0;
        dispatch_hw_loop_boundary = 1'b0;
        dispatch_hw_loop_repeat = 1'b0;

        if (expert_loop_end && expert_active_q &&
            expert_iter_q + 1 < expert_count_q) begin
            dispatch_target = expert_start_q;
        end else if (token_loop_end && token_active_q &&
                     token_iter_q + 1 < token_count_q) begin
            dispatch_target = token_start_q;
        end else if (source_skip_valid && source_skip_count != 0) begin
            dispatch_target = source_skip_target[SRAM_ADDR_W-1:0];
            dispatch_terminal = source_skip_target >=
                {{(SKIP_SUM_W-SRAM_ADDR_W){1'b0}}, len_q};
        end else if (loop_active_q && pc_plus1 > loop_end_q) begin
            dispatch_hw_loop_boundary = 1'b1;
            if (loop_count_q > 16'd1) begin
                dispatch_target = loop_start_q;
                dispatch_hw_loop_repeat = 1'b1;
            end else begin
                dispatch_terminal = pc_plus1 >= len_q;
            end
        end else begin
            dispatch_terminal = pc_plus1 >= len_q;
        end
    end

    always_comb begin
        state_d     = state_q;
        pc_d        = pc_q;
        base_d      = base_q;
        len_d       = len_q;
        instr_buf_d = instr_buf_q;
        done_d      = 1'b0;

        // Loop defaults: hold
        loop_start_d  = loop_start_q;
        loop_end_d    = loop_end_q;
        loop_count_d  = loop_count_q;
        loop_iter_d   = loop_iter_q;
        loop_active_d = loop_active_q;
        addr_stride_d = addr_stride_q;
        token_start_d = token_start_q;
        token_count_d = token_count_q;
        token_iter_d = token_iter_q;
        token_active_d = token_active_q;
        expert_start_d = expert_start_q;
        expert_count_d = expert_count_q;
        expert_iter_d = expert_iter_q;
        expert_active_d = expert_active_q;

        rd_en       = 1'b0;
        rd_addr     = '0;
        instr_valid = 1'b0;
        instr_data  = instr_buf_q;

        case (state_q)
            S_IDLE: begin
                if (start) begin
                    pc_d    = resume_pc;  // host-in-the-loop: resume from PC
                    base_d  = ucode_base_addr;
                    len_d   = ucode_len;
                    token_iter_d = '0;
                    token_active_d = 1'b0;
                    expert_iter_d = '0;
                    expert_active_d = 1'b0;
                    state_d = S_FETCH;
                end
            end

            S_FETCH: begin
                rd_en   = 1'b1;
                rd_addr = base_q + pc_q;
                state_d = S_WAIT_DATA;
            end

            S_WAIT_DATA: begin
                if (stop) begin
                    state_d = S_DONE;
                end else if (rd_valid) begin
                    // Bypass the dispatch buffer on the normal path. The
                    // decoder sees the SRAM response in this cycle, allowing
                    // the next SRAM read to launch as soon as it accepts the
                    // instruction. If decode applies backpressure, preserve
                    // the response in instr_buf_q and use S_DISPATCH below.
                    instr_valid = 1'b1;
                    instr_data  = rd_data;
                    if (!instr_ready) begin
                        instr_buf_d = rd_data;
                        state_d = S_DISPATCH;
                    end else if (fetched_is_end) begin
                        // END must reach decode: decode waits for outstanding
                        // work before accepting it. Stopping here reports done
                        // before the last vector operation has written back.
                        state_d = S_DONE;
                    end else begin
                        pc_d = dispatch_target;
                        if (expert_loop_end) begin
                            if (expert_active_q &&
                                expert_iter_q + 1 < expert_count_q) begin
                                expert_iter_d = expert_iter_q + 1'b1;
                            end else begin
                                expert_active_d = 1'b0;
                                expert_iter_d = '0;
                            end
                        end else if (token_loop_end) begin
                            if (token_active_q &&
                                token_iter_q + 1 < token_count_q) begin
                                token_iter_d = token_iter_q + 1'b1;
                            end else begin
                                token_active_d = 1'b0;
                                token_iter_d = '0;
                            end
                        end
                        if (dispatch_hw_loop_boundary) begin
                            if (dispatch_hw_loop_repeat) begin
                                loop_count_d = loop_count_q - 16'd1;
                                loop_iter_d = loop_iter_q + 16'd1;
                            end else begin
                                loop_active_d = 1'b0;
                            end
                        end
                        if (dispatch_terminal) begin
                            state_d = S_DONE;
                        end else begin
                            // Launch the next sequential/loop-target read on
                            // this acceptance edge; no extra S_FETCH cycle.
                            rd_en = 1'b1;
                            rd_addr = base_q + dispatch_target;
                            state_d = S_WAIT_DATA;
                        end
                        if (token_loop_begin) begin
                            token_start_d = pc_plus1;
                            token_count_d = token_loop_count;
                            token_iter_d = '0;
                            token_active_d = token_loop_count != 0;
                        end
                        if (expert_loop_begin) begin
                            expert_start_d = pc_plus1;
                            expert_count_d = expert_loop_count;
                            expert_iter_d = '0;
                            expert_active_d = expert_loop_count != 0;
                        end
                        if (loop_cmd_valid) begin
                            loop_start_d = pc_q +
                                {{(SRAM_ADDR_W-1){1'b0}}, 1'b1};
                            loop_end_d = pc_q +
                                {{(SRAM_ADDR_W-1){1'b0}}, loop_body_len};
                            loop_count_d = {4'b0, loop_count_init};
                            loop_iter_d = 16'd0;
                            loop_active_d = 1'b1;
                            addr_stride_d = stride_code_to_value(loop_stride_code);
                        end
                    end
                end
            end

            S_DISPATCH: begin
                instr_valid = 1'b1;
                instr_data  = instr_buf_q;
                if (stop) begin
                    state_d = S_DONE;
                end else if (instr_ready) begin
                    if (instr_buf_q[31:28] == 4'hF) begin
                        state_d = S_DONE;
                    end else begin
                        pc_d = dispatch_target;
                        if (expert_loop_end) begin
                            if (expert_active_q &&
                                expert_iter_q + 1 < expert_count_q) begin
                                expert_iter_d = expert_iter_q + 1'b1;
                            end else begin
                                expert_active_d = 1'b0;
                                expert_iter_d = '0;
                            end
                        end else if (token_loop_end) begin
                            if (token_active_q &&
                                token_iter_q + 1 < token_count_q) begin
                                token_iter_d = token_iter_q + 1'b1;
                            end else begin
                                token_active_d = 1'b0;
                                token_iter_d = '0;
                            end
                        end
                        if (dispatch_hw_loop_boundary) begin
                            if (dispatch_hw_loop_repeat) begin
                                loop_count_d = loop_count_q - 16'd1;
                                loop_iter_d = loop_iter_q + 16'd1;
                            end else begin
                                loop_active_d = 1'b0;
                            end
                        end
                        if (dispatch_terminal) begin
                            state_d = S_DONE;
                        end else begin
                            // The SRAM samples this address on the same edge
                            // that accepts the current instruction. Keep the
                            // response/dispatch pipeline full at one read per
                            // cycle when decode is ready.
                            rd_en = 1'b1;
                            rd_addr = base_q + dispatch_target;
                            state_d = S_WAIT_DATA;
                        end
                    end
                    if (token_loop_begin) begin
                        token_start_d = pc_plus1;
                        token_count_d = token_loop_count;
                        token_iter_d = '0;
                        token_active_d = token_loop_count != 0;
                    end
                    if (expert_loop_begin) begin
                        expert_start_d = pc_plus1;
                        expert_count_d = expert_loop_count;
                        expert_iter_d = '0;
                        expert_active_d = expert_loop_count != 0;
                    end
                    // LOOP instruction: set up loop registers
                    if (loop_cmd_valid) begin
                        loop_start_d  = pc_q + {{(SRAM_ADDR_W-1){1'b0}}, 1'b1};
                        loop_end_d    = pc_q + {{(SRAM_ADDR_W-1){1'b0}}, loop_body_len};
                        loop_count_d  = {4'b0, loop_count_init};
                        loop_iter_d   = 16'd0;
                        loop_active_d = 1'b1;
                        addr_stride_d = stride_code_to_value(loop_stride_code);
                    end
                end
            end

            S_DONE: begin
                done_d  = 1'b1;
                state_d = S_IDLE;
            end

            default: begin
                state_d = S_IDLE;
            end
        endcase

        if (stop && (state_q != S_IDLE) && (state_q != S_DONE)) begin
            state_d = S_DONE;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_q     <= S_IDLE;
            pc_q        <= '0;
            base_q      <= '0;
            len_q       <= '0;
            instr_buf_q <= '0;
            done_q      <= 1'b0;
            loop_start_q  <= '0;
            loop_end_q    <= '0;
            loop_count_q  <= '0;
            loop_iter_q   <= '0;
            loop_active_q <= 1'b0;
            addr_stride_q <= '0;
            token_start_q <= '0;
            token_count_q <= '0;
            token_iter_q <= '0;
            token_active_q <= 1'b0;
            expert_start_q <= '0;
            expert_count_q <= '0;
            expert_iter_q <= '0;
            expert_active_q <= 1'b0;
        end else begin
            state_q     <= state_d;
            pc_q        <= pc_d;
            base_q      <= base_d;
            len_q       <= len_d;
            instr_buf_q <= instr_buf_d;
            done_q      <= done_d;
            loop_start_q  <= loop_start_d;
            loop_end_q    <= loop_end_d;
            loop_count_q  <= loop_count_d;
            loop_iter_q   <= loop_iter_d;
            loop_active_q <= loop_active_d;
            addr_stride_q <= addr_stride_d;
            token_start_q <= token_start_d;
            token_count_q <= token_count_d;
            token_iter_q <= token_iter_d;
            token_active_q <= token_active_d;
            expert_start_q <= expert_start_d;
            expert_count_q <= expert_count_d;
            expert_iter_q <= expert_iter_d;
            expert_active_q <= expert_active_d;
        end
    end

    assign pc          = pc_q;
    assign done        = done_q;
    assign busy        = (state_q != S_IDLE);
    assign loop_active = loop_active_q;
    assign loop_iter   = loop_iter_q;
    assign loop_remaining = loop_count_q;
    assign addr_stride = addr_stride_q;
    assign token_iter = token_iter_q;
    assign expert_iter = expert_iter_q;

endmodule
