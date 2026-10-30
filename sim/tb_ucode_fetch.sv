`timescale 1ns/1ps

module tb_ucode_fetch;
    localparam int ADDR_W = 8;

    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic rst_n = 1'b0;
    logic start = 1'b0;
    logic rd_en;
    logic [ADDR_W-1:0] rd_addr;
    logic [ADDR_W-1:0] ucode_len = ADDR_W'(3);
    logic [31:0] rd_data = '0;
    logic rd_valid = 1'b0;
    logic instr_valid;
    logic instr_ready = 1'b1;
    logic [31:0] instr_data;
    logic [ADDR_W-1:0] pc;
    logic loop_cmd_valid;
    logic [3:0] loop_body_len;
    logic [11:0] loop_count_init;
    logic [1:0] loop_stride_code = 2'b00;
    logic token_loop_begin;
    logic token_loop_end;
    logic [5:0] token_loop_count;
    logic expert_loop_begin;
    logic expert_loop_end;
    logic [5:0] expert_loop_count;
    logic source_skip_valid;
    logic [15:0] source_skip_count;
    logic loop_active;
    logic [15:0] loop_iter;
    logic [15:0] loop_remaining;
    logic [15:0] addr_stride;
    logic [5:0] token_iter;
    logic [5:0] expert_iter;
    logic done;
    logic busy;

    logic [31:0] instr_mem [0:255];
    logic [ADDR_W-1:0] req_addr [0:15];
    integer req_cycle [0:15];
    logic [31:0] accepted_instr [0:15];
    integer req_count = 0;
    integer accepted_count = 0;
    integer cycle_count = 0;

    assign loop_cmd_valid = instr_valid &&
                            instr_data[31:28] == 4'h0 &&
                            instr_data[27:24] == 4'hF;
    assign loop_body_len = instr_data[23:20];
    assign loop_count_init = instr_data[11:0];
    always_comb begin
        token_loop_begin = instr_valid && instr_data[31:28] == 4'h0 &&
                           instr_data[15:12] == 4'h8;
        token_loop_end = instr_valid && instr_data[31:28] == 4'h0 &&
                         instr_data[15:12] == 4'h9;
        token_loop_count = instr_data[5:0];
        source_skip_valid = instr_valid && instr_data[31:28] == 4'h0 &&
                            instr_data[27:24] == 4'hE &&
                            instr_data[23:20] == 4'hF &&
                            instr_data[15:12] == 4'h0;
        source_skip_count = {instr_data[19:16], instr_data[11:0]};
        expert_loop_begin = instr_valid && instr_data[31:28] == 4'h0 &&
                            instr_data[27:24] == 4'hE &&
                            instr_data[23:20] == 4'hE &&
                            instr_data[19:16] == 4'hD &&
                            instr_data[11:0] != 0;
        expert_loop_end = instr_valid && instr_data[31:28] == 4'h0 &&
                          instr_data[27:24] == 4'hE &&
                          instr_data[23:20] == 4'hD &&
                          instr_data[19:16] == 4'hE &&
                          instr_data[11:0] == 0;
        expert_loop_count = instr_data[5:0];
    end

    always_ff @(posedge clk) begin
        cycle_count <= cycle_count + 1;
        if (rd_en) begin
            rd_data <= instr_mem[rd_addr];
            req_addr[req_count] <= rd_addr;
            req_cycle[req_count] <= cycle_count;
            req_count <= req_count + 1;
        end
        rd_valid <= rd_en;

        if (!rst_n) begin
            req_count <= 0;
            accepted_count <= 0;
            cycle_count <= 0;
        end else if (instr_valid && instr_ready) begin
            accepted_instr[accepted_count] <= instr_data;
            accepted_count <= accepted_count + 1;
        end
    end

    ucode_fetch #(
        .INSTR_W(32),
        .SRAM_ADDR_W(ADDR_W)
    ) dut (
        .clk(clk), .rst_n(rst_n), .start(start), .stop(1'b0),
        .ucode_base_addr(ADDR_W'(0)), .ucode_len(ucode_len),
        .resume_pc(ADDR_W'(0)), .rd_en(rd_en), .rd_addr(rd_addr),
        .rd_data(rd_data), .rd_valid(rd_valid),
        .instr_valid(instr_valid), .instr_ready(instr_ready),
        .instr_data(instr_data), .pc(pc),
        .loop_cmd_valid(loop_cmd_valid), .loop_body_len(loop_body_len),
        .loop_count_init(loop_count_init), .loop_stride_code(loop_stride_code),
        .token_loop_begin(token_loop_begin), .token_loop_end(token_loop_end),
        .token_loop_count(token_loop_count),
        .expert_loop_begin(expert_loop_begin), .expert_loop_end(expert_loop_end),
        .expert_loop_count(expert_loop_count),
        .source_skip_valid(source_skip_valid), .source_skip_count(source_skip_count),
        .loop_active(loop_active), .loop_iter(loop_iter),
        .loop_remaining(loop_remaining), .addr_stride(addr_stride),
        .token_iter(token_iter), .expert_iter(expert_iter),
        .done(done), .busy(busy)
    );

    task automatic pulse_reset;
        begin
            rst_n = 1'b0;
            start = 1'b0;
            repeat (2) @(posedge clk);
            @(negedge clk);
            rst_n = 1'b1;
        end
    endtask

    task automatic start_program;
        begin
            @(negedge clk);
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
        end
    endtask

    task automatic wait_done;
        integer cycles;
        begin
            cycles = 0;
            while (!done && cycles < 200) begin
                @(posedge clk);
                cycles = cycles + 1;
            end
            if (!done) $fatal(1, "ucode_fetch timeout");
        end
    endtask

    initial begin
        foreach (instr_mem[i]) instr_mem[i] = 32'hF0000000;

        // Sequential stream: every instruction must be accepted exactly once.
        instr_mem[0] = 32'h00000000;
        instr_mem[1] = 32'h00000000;
        instr_mem[2] = 32'hF0000000;
        pulse_reset();
        start_program();
        wait_done();
        if (req_count != 3 || accepted_count != 3)
            $fatal(1, "sequential counts req=%0d accepted=%0d", req_count, accepted_count);
        if (req_addr[0] != 0 || req_addr[1] != 1 || req_addr[2] != 2)
            $fatal(1, "sequential request addresses are incorrect");
        if (req_cycle[1] - req_cycle[0] != 2 ||
            req_cycle[2] - req_cycle[1] != 2)
            $fatal(1, "sequential read issue interval is not 2 cycles");
        if (accepted_instr[0] != 32'h00000000 ||
            accepted_instr[1] != 32'h00000000 ||
            accepted_instr[2] != 32'hF0000000)
            $fatal(1, "sequential instruction stream is incorrect");

        // A stalled decode must hold the current instruction and must not
        // issue a speculative next read before the acceptance handshake.
        instr_mem[0] = 32'h00000000;
        instr_mem[1] = 32'hF0000000;
        pulse_reset();
        instr_ready = 1'b0;
        start_program();
        wait (instr_valid);
        repeat (3) @(negedge clk);
        if (req_count != 1 || accepted_count != 0 || pc != 0 ||
            instr_data != 32'h00000000)
            $fatal(1, "fetch advanced while decode was backpressuring");
        instr_ready = 1'b1;
        @(negedge clk);
        if (req_count != 2 || accepted_count != 1 || req_addr[1] != 1)
            $fatal(1, "next read did not launch with instruction acceptance");
        instr_ready = 1'b1;

        // Hardware loop: marker at 0, one-instruction body repeated twice,
        // followed by END. The backedge must fetch address 1 directly.
        instr_ready = 1'b1;
        instr_mem[0] = {4'h0, 4'hF, 4'd1, 4'b0, 16'd2};
        instr_mem[1] = 32'h00000000;
        instr_mem[2] = 32'hF0000000;
        pulse_reset();
        start_program();
        wait_done();
        if (req_count != 4 || accepted_count != 4)
            $fatal(1, "loop counts req=%0d accepted=%0d", req_count, accepted_count);
        if (req_addr[0] != 0 || req_addr[1] != 1 ||
            req_addr[2] != 1 || req_addr[3] != 2)
            $fatal(1, "hardware-loop request addresses are incorrect");
        if (req_cycle[1] - req_cycle[0] != 2 ||
            req_cycle[2] - req_cycle[1] != 2 ||
            req_cycle[3] - req_cycle[2] != 2)
            $fatal(1, "hardware-loop read issue interval is not 2 cycles");
        if (accepted_instr[0] != instr_mem[0] ||
            accepted_instr[1] != instr_mem[1] ||
            accepted_instr[2] != instr_mem[1] ||
            accepted_instr[3] != instr_mem[2])
            $fatal(1, "hardware-loop instruction stream is incorrect");

        // Token-loop redirects exercise the other dynamic backedge source.
        instr_mem[0] = 32'h00008002;
        instr_mem[1] = 32'h00009000;
        instr_mem[2] = 32'hF0000000;
        pulse_reset();
        start_program();
        wait_done();
        if (req_count != 4 || accepted_count != 4 ||
            req_addr[0] != 0 || req_addr[1] != 1 ||
            req_addr[2] != 1 || req_addr[3] != 2 ||
            token_iter != 0)
            $fatal(1, "token-loop redirect sequence is incorrect");

        // Expert-loop markers use a different decoder sideband but the same
        // synchronous redirect path as token loops.
        instr_mem[0] = 32'h0EED0002;
        instr_mem[1] = 32'h00000000;
        instr_mem[2] = 32'h0EDE0000;
        instr_mem[3] = 32'hF0000000;
        ucode_len = ADDR_W'(4);
        pulse_reset();
        start_program();
        wait_done();
        if (req_count != 6 || accepted_count != 6 ||
            req_addr[0] != 0 || req_addr[1] != 1 || req_addr[2] != 2 ||
            req_addr[3] != 1 || req_addr[4] != 2 || req_addr[5] != 3 ||
            expert_iter != 0) begin
            $display("expert loop req=%0d accepted=%0d addrs=%0d,%0d,%0d,%0d,%0d,%0d iter=%0d",
                     req_count, accepted_count, req_addr[0], req_addr[1],
                     req_addr[2], req_addr[3], req_addr[4], req_addr[5],
                     expert_iter);
            $fatal(1, "expert-loop redirect sequence is incorrect");
        end
        for (int i = 1; i < 6; i++) begin
            if (req_cycle[i] - req_cycle[i-1] != 2)
                $fatal(1, "expert-loop read interval is not 2 cycles");
        end

        // Empty-group source skip bypasses one dead body word without reading
        // it; the marker-to-target address must be issued on marker acceptance.
        instr_mem[0] = 32'h0EF00001;
        instr_mem[1] = 32'h00000000;
        instr_mem[2] = 32'hF0000000;
        ucode_len = ADDR_W'(3);
        pulse_reset();
        start_program();
        wait_done();
        if (req_count != 2 || accepted_count != 2 ||
            req_addr[0] != 0 || req_addr[1] != 2 ||
            accepted_instr[0] != instr_mem[0] ||
            accepted_instr[1] != instr_mem[2])
            $fatal(1, "source-skip redirect sequence is incorrect");

        $display("[TB][UCODE_FETCH] PASS sequential, backpressure, HW/token/expert loops, source skip");
        $finish;
    end
endmodule
