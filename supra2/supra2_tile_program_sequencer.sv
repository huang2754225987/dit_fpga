`default_nettype none

// Dependency-aware issue sequencer for the compiler's 128-bit tile ISA.
// A command retires only after its datapath client asserts command_done.
module supra2_tile_program_sequencer #(
    parameter PROGRAM_FILE = "block13_tile_program.hex",
    parameter DEPENDENCY_FILE = "block13_tile_dependencies.hex",
    parameter bit DDR_LOAD = 1'b0,
    parameter integer PROGRAM_WORDS = 259,
    parameter integer DEPENDENCY_WORDS = 1052,
    parameter integer MAX_COMMANDS = 512,
    parameter integer MAX_DEPENDENCIES = 4096,
    parameter integer MAX_TOKENS = 512
) (
    input  wire logic         clk,
    input  wire logic         rst_n,
    input  wire logic         start,
    output logic              command_valid,
    input  wire logic         command_ready,
    output logic [127:0]      command_word,
    input  wire logic         command_done,
    output logic              program_done,
    output logic              program_error,
    output logic [15:0]       program_counter,
    input wire logic          program_load_we,
    input wire logic [15:0]   program_load_addr,
    input wire logic [127:0]  program_load_data,
    input wire logic          dependency_load_we,
    input wire logic [15:0]   dependency_load_addr,
    input wire logic [15:0]   dependency_load_data
);
    (* rom_style = "block" *) logic [127:0] program_rom [0:MAX_COMMANDS-1];
    (* rom_style = "block" *) logic [15:0] dependency_rom [0:MAX_DEPENDENCIES-1];
    typedef enum logic [2:0] {S_IDLE,S_FETCH,S_DECODE,S_DEP_FETCH,S_DEP_CHECK,S_ISSUE,S_WAIT} state_t;
    state_t state;
    logic [MAX_TOKENS-1:0] token_done;
    logic active, in_flight;
    logic [15:0] pc, dep_index, dep_read_addr, dep_value;
    logic [127:0] command_reg;
    logic [15:0] dep_start, completion_token;
    logic [7:0] dep_count, opcode;
    integer program_words = PROGRAM_WORDS;
    integer dependency_words = DEPENDENCY_WORDS;

    initial begin
`ifndef SYNTHESIS
        if (program_words < 1 || program_words > MAX_COMMANDS ||
            dependency_words < 0 || dependency_words > MAX_DEPENDENCIES)
            $fatal(1, "Invalid tile-program ROM depths");
        if (!DDR_LOAD) begin
            $readmemh(PROGRAM_FILE, program_rom, 0, program_words-1);
            if (dependency_words != 0)
                $readmemh(DEPENDENCY_FILE, dependency_rom, 0, dependency_words-1);
        end
`endif
    end

    always_comb begin
        command_word = command_reg;
        opcode = command_word[127:120];
        dep_start = command_word[39:24];
        dep_count = command_word[7:0];
        completion_token = command_word[23:8];
        dep_read_addr = dep_start + dep_index;
        command_valid = active && (state == S_ISSUE) && !program_error;
    end

    // Synchronous read ports let Vivado infer block RAM for the DDR-loaded
    // command and dependency tables instead of a large asynchronous mux tree.
    always_ff @(posedge clk) begin
        if (DDR_LOAD) begin
            if (program_load_we && program_load_addr < MAX_COMMANDS)
                program_rom[program_load_addr] <= program_load_data;
            if (dependency_load_we && dependency_load_addr < MAX_DEPENDENCIES)
                dependency_rom[dependency_load_addr] <= dependency_load_data;
        end
        if (state == S_FETCH) command_reg <= program_rom[pc];
        if (state == S_DEP_FETCH) dep_value <= dependency_rom[dep_read_addr];
    end

    assign program_counter = pc;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            active <= 1'b0;
            in_flight <= 1'b0;
            pc <= '0;
            dep_index <= '0;
            token_done <= '0;
            program_done <= 1'b0;
            program_error <= 1'b0;
        end else if (start) begin
            state <= S_FETCH;
            active <= 1'b1;
            in_flight <= 1'b0;
            pc <= '0;
            dep_index <= '0;
            token_done <= '0;
            program_done <= 1'b0;
            program_error <= 1'b0;
        end else if (active && !program_error) begin
            case (state)
                S_IDLE: ;
                S_FETCH: state <= S_DECODE;
                S_DECODE: begin
                    if (pc >= program_words) begin
                        program_error <= 1'b1;
                        active <= 1'b0;
                        state <= S_IDLE;
                    end else if (opcode == 8'h0f) begin
                        if (command_word !== 128'h0f000000000000000000000000000000 ||
                            pc != program_words - 1) begin
                            program_error <= 1'b1;
                            active <= 1'b0;
                            state <= S_IDLE;
                        end else begin
                            program_done <= 1'b1;
                            active <= 1'b0;
                            state <= S_IDLE;
                        end
                    end else if (int'(dep_start) + dep_count > dependency_words) begin
                        program_error <= 1'b1;
                        active <= 1'b0;
                        state <= S_IDLE;
                    end else if (dep_count == 0) begin
                        state <= S_ISSUE;
                    end else begin
                        dep_index <= '0;
                        state <= S_DEP_FETCH;
                    end
                end
                S_DEP_FETCH: state <= S_DEP_CHECK;
                S_DEP_CHECK: begin
                    if (dep_value >= MAX_TOKENS || !token_done[dep_value]) begin
                        program_error <= 1'b1;
                        active <= 1'b0;
                        state <= S_IDLE;
                    end else if (dep_index + 1 >= dep_count) begin
                        state <= S_ISSUE;
                    end else begin
                        dep_index <= dep_index + 1'b1;
                        state <= S_DEP_FETCH;
                    end
                end
                S_ISSUE: if (command_valid && command_ready) begin
                    if (completion_token == 0 || completion_token >= MAX_TOKENS) begin
                    program_error <= 1'b1;
                    active <= 1'b0;
                        state <= S_IDLE;
                    end else begin
                        in_flight <= 1'b1;
                        state <= S_WAIT;
                    end
                end
                S_WAIT: if (in_flight && command_done) begin
                    token_done[completion_token] <= 1'b1;
                    in_flight <= 1'b0;
                    pc <= pc + 1'b1;
                    state <= S_FETCH;
                end
                default: begin
                    program_error <= 1'b1;
                    active <= 1'b0;
                    state <= S_IDLE;
                end
            endcase
        end
    end

`ifndef SYNTHESIS
    always @(posedge clk) begin
        if (rst_n && active && state == S_DECODE && opcode != 8'h0f &&
            int'(dep_start) + dep_count > dependency_words)
            $fatal(1, "Tile dependency slice invalid pc=%0d start=%0d count=%0d depth=%0d",
                pc, dep_start, dep_count, dependency_words);
        if (rst_n && in_flight && command_done && completion_token == 0)
            $fatal(1, "Tile command retired without a completion token pc=%0d", pc);
    end
`endif
endmodule

`default_nettype wire
