`default_nettype none

// Stage ISA v1: RUN_STAGE = 0x1 in bits 31:28, stage ID in bits 27:23;
// END = 0xF. The compiler emits sixteen stage commands followed by END.
// Stage execution itself remains in supra2_block_top and stalls this reader.
module supra2_block_stage_sequencer #(
    parameter PROGRAM_FILE = "block13_stage_program.hex"
) (
    input wire logic clk,
    input wire logic rst_n,
    input wire logic start,
    input wire logic command_ready,
    output wire logic command_valid,
    output wire logic [4:0] command_stage,
    output logic program_error
);
    (* rom_style = "block" *) logic [31:0] instruction_rom [0:16];
    logic active;
    logic [4:0] pc;
    wire [31:0] instruction = instruction_rom[pc];
    wire is_run = instruction[31:28] == 4'h1 &&
        instruction[27:23] == pc && instruction[22:0] == 23'b0 && pc < 16;
    wire is_end = instruction == 32'hf0000000 && pc == 16;

    initial $readmemh(PROGRAM_FILE, instruction_rom);
    assign command_valid = active && !program_error && (is_run || is_end);
    assign command_stage = is_end ? 5'd16 : instruction[27:23];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active <= 1'b0;
            pc <= '0;
            program_error <= 1'b0;
        end else if (start) begin
            active <= 1'b1;
            pc <= '0;
            program_error <= 1'b0;
        end else if (active && command_ready) begin
            if (!(is_run || is_end)) begin
                active <= 1'b0;
                program_error <= 1'b1;
            end else if (is_end) begin
                active <= 1'b0;
            end else begin
                pc <= pc + 1'b1;
            end
        end
    end
`ifndef SYNTHESIS
    always @(posedge clk)
        if (active && command_ready && !(is_run || is_end))
            $fatal(1, "Invalid stage instruction pc=%0d raw=%08h", pc, instruction);
`endif
endmodule

`default_nettype wire
