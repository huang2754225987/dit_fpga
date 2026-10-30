`default_nettype none

module tb_supra2_compiled_tile_schedule;
    localparam int NRES = 28;
    localparam int MAXTASK = 2048;
    logic clk = 1'b0, rst_n = 1'b0;
    logic [NRES-1:0] res_busy, res_error;
    logic replay_done;
    logic [31:0] makespan_cycles;
    logic [31:0] busy_cycles [0:NRES-1];
    integer expected_makespan, expected_tasks;
    integer expected_program_instructions, program_words;
    string program_file;
    string dependency_file;
    logic [127:0] instruction_rom [0:511];
    logic [15:0] dependency_rom [0:4095];
    integer dependency_words;
    integer errors = 0;

    always #5 clk = ~clk;

    supra2_block_schedule_replay #(.MAXTASK(MAXTASK), .NRES(NRES)) dut (
        .clk(clk), .rst_n(rst_n), .res_busy(res_busy), .res_error(res_error),
        .replay_done(replay_done), .makespan_cycles(makespan_cycles),
        .busy_cycles(busy_cycles)
    );

    initial begin
        if (!$value$plusargs("EXPECTED_MAKESPAN=%d", expected_makespan) ||
            !$value$plusargs("NTASK=%d", expected_tasks) ||
            !$value$plusargs("EXPECTED_INSTRUCTIONS=%d", expected_program_instructions) ||
            !$value$plusargs("PROGRAM_FILE=%s", program_file) ||
            !$value$plusargs("DEPENDENCY_FILE=%s", dependency_file))
            $fatal(1, "Missing expected schedule plusargs");
        program_words = expected_program_instructions;
        $readmemh(program_file, instruction_rom, 0, program_words-1);
        dependency_words = 0;
        if (!$value$plusargs("DEPENDENCY_WORDS=%d", dependency_words))
            $fatal(1, "Missing dependency word count");
        $readmemh(dependency_file, dependency_rom, 0, dependency_words-1);
        if (instruction_rom[program_words-1] !== 128'h0f000000000000000000000000000000)
            $fatal(1, "Tile instruction stream has no terminal END");
        for (integer pc=0; pc<program_words-1; pc++) begin
            if (instruction_rom[pc][127:120] < 1 || instruction_rom[pc][127:120] > 8)
                $fatal(1, "Invalid opcode at tile instruction pc=%0d", pc);
            if ((instruction_rom[pc][87:80] != 8'hff && instruction_rom[pc][87:80] >= 21) ||
                (instruction_rom[pc][79:72] != 8'hff && instruction_rom[pc][79:72] >= 21) ||
                (instruction_rom[pc][71:64] != 8'hff && instruction_rom[pc][71:64] >= 21) ||
                (instruction_rom[pc][63:56] != 8'hff && instruction_rom[pc][63:56] >= 21))
                $fatal(1, "Invalid bank ID at tile instruction pc=%0d", pc);
            if (instruction_rom[pc][127:120] == 3 &&
                (instruction_rom[pc][103:96] >= 9 ||
                 (instruction_rom[pc][95:88] != 1 && instruction_rom[pc][95:88] != 8)))
                $fatal(1, "Invalid attention tile fields at pc=%0d", pc);
            if (instruction_rom[pc][39:24] + instruction_rom[pc][7:0] > dependency_words)
                $fatal(1, "Dependency slice out of range at pc=%0d", pc);
            for (integer d=0; d<instruction_rom[pc][7:0]; d++) begin
                logic found_prior_producer;
                found_prior_producer = 1'b0;
                for (integer p=0; p<pc; p++)
                    if (instruction_rom[p][23:8] == dependency_rom[instruction_rom[pc][39:24] + d])
                        found_prior_producer = 1'b1;
                if (!found_prior_producer)
                    $fatal(1, "Dependency token %0d has no earlier producer at pc=%0d",
                        dependency_rom[instruction_rom[pc][39:24] + d], pc);
            end
        end
        repeat (8) @(posedge clk);
        rst_n <= 1'b1;
        wait (replay_done);
        @(posedge clk);
        if (makespan_cycles !== expected_makespan) errors++;
        if (res_error != '0) errors++;
        $display("COMPILED_TILE_SCHEDULE makespan=%0d replay_events=%0d instructions=%0d array_busy=%0d dma_busy=%0d errors=%0d",
            makespan_cycles, expected_tasks, program_words, busy_cycles[0], busy_cycles[5], errors);
        if (errors == 0)
            $display("SUPRA2_COMPILED_TILE_SCHEDULE_PASS makespan=%0d events=%0d",
                makespan_cycles, expected_tasks);
        else
            $display("SUPRA2_COMPILED_TILE_SCHEDULE_FAIL errors=%0d", errors);
        $finish;
    end

    initial begin
        #500_000_000;
        $fatal(1, "compiled tile schedule replay timeout");
    end
endmodule

`default_nettype wire
