`timescale 1ns/1ps
module tb_gld_compact;
    logic clk = 0;
    always #5 clk = ~clk;
    logic rst_n = 0, cmd = 0, a_cmd = 0, compact_mode = 0, batch_a = 0, transpose_a = 0;
    logic [15:0] cfg_k = 64;
    logic [15:0] cfg_m = 1;
    logic [8:0] a_stride_lines = 0;
    logic [1:0] prec = 0;
    wire done, a_done, rd_en, wr_en, arf_wr_en, busy;
    wire [1:0] rd_page;
    wire arf_transpose;
    wire [10:0] rd_addr;
    logic [255:0] rd_data;
    logic rd_valid = 0;
    wire [7:0] wr_addr;
    wire [2:0] bank_base;
    wire [31:0] data [0:7];
    wire [7:0] arf_wr_addr;
    wire [1:0] arf_wr_bank_base;
    wire [15:0] arf_data [0:31];
    int a_writes, a_reads;
    bit batch_test = 0, transpose_test = 0, strided_transpose_test = 0;
    logic [255:0] mem [0:2047];
    int writes, reads, test_k, test_prec, per_line, delay_count = 0;
    int cycle = 0, issue_cycle, done_cycle;
    int read_delay = 1;
    logic test_compact;
    logic [10:0] pending_addr;
    gld_ctrl dut (
        .clk(clk), .rst_n(rst_n), .cfg_K(cfg_k), .cfg_M(cfg_m),
        .gld_w_cmd_valid(cmd), .gld_w_cmd_addr(16'd0),
        .gld_w_cmd_prec(prec), .gld_w_cmd_compact(compact_mode), .gld_w_done(done),
        .gld_a_cmd_valid(a_cmd), .gld_a_cmd_addr(16'd0), .gld_a_cmd_batch(batch_a),
        .gld_a_cmd_transpose(transpose_a), .gld_a_cmd_page(2'd0),
        .gld_a_cmd_stride_lines(a_stride_lines),
        .gld_a_done(a_done),
        .l1_rd_en(rd_en), .l1_rd_addr(rd_addr), .l1_rd_page(rd_page),
        .l1_rd_data(rd_data), .l1_rd_valid(rd_valid),
        .wrf_wr_en(wr_en), .wrf_wr_addr(wr_addr), .wrf_wr_bank_base(bank_base),
        .wrf_wr_data0(data[0]), .wrf_wr_data1(data[1]), .wrf_wr_data2(data[2]),
        .wrf_wr_data3(data[3]), .wrf_wr_data4(data[4]), .wrf_wr_data5(data[5]),
        .wrf_wr_data6(data[6]), .wrf_wr_data7(data[7]),
        .arf_wr_en(arf_wr_en), .arf_wr_valid(), .arf_wr_transpose(arf_transpose),
        .arf_wr_addr(arf_wr_addr), .arf_wr_bank_base(arf_wr_bank_base),
        .arf_wr_data0(arf_data[0]), .arf_wr_data1(arf_data[1]),
        .arf_wr_data2(arf_data[2]), .arf_wr_data3(arf_data[3]),
        .arf_wr_data4(arf_data[4]), .arf_wr_data5(arf_data[5]),
        .arf_wr_data6(arf_data[6]), .arf_wr_data7(arf_data[7]),
        .arf_wr_data8(arf_data[8]), .arf_wr_data9(arf_data[9]),
        .arf_wr_data10(arf_data[10]), .arf_wr_data11(arf_data[11]),
        .arf_wr_data12(arf_data[12]), .arf_wr_data13(arf_data[13]),
        .arf_wr_data14(arf_data[14]), .arf_wr_data15(arf_data[15]),
        .arf_wr_data16(arf_data[16]), .arf_wr_data17(arf_data[17]),
        .arf_wr_data18(arf_data[18]), .arf_wr_data19(arf_data[19]),
        .arf_wr_data20(arf_data[20]), .arf_wr_data21(arf_data[21]),
        .arf_wr_data22(arf_data[22]), .arf_wr_data23(arf_data[23]),
        .arf_wr_data24(arf_data[24]), .arf_wr_data25(arf_data[25]),
        .arf_wr_data26(arf_data[26]), .arf_wr_data27(arf_data[27]),
        .arf_wr_data28(arf_data[28]), .arf_wr_data29(arf_data[29]),
        .arf_wr_data30(arf_data[30]), .arf_wr_data31(arf_data[31]),
        .busy(busy)
    );
    function automatic logic [31:0] pattern(input int k, b);
        return 32'h9e370000 ^ (k * 32'h10203) ^ (b * 32'h5011);
    endfunction
    always @(posedge clk) begin
        cycle++;
        rd_valid <= 0;
        if (rd_en) begin
            reads++;
            pending_addr = rd_addr;
            delay_count = read_delay;
        end
        if (delay_count > 0) begin
            delay_count--;
            if (delay_count == 0) begin
                rd_data <= mem[pending_addr];
                rd_valid <= 1;
            end
        end
        if (wr_en) begin
            if (wr_addr !== (test_prec == 0 ? writes / 2 : writes))
                $fatal(1, "WRF entry sequence mismatch write=%0d entry=%0d", writes, wr_addr);
            if (bank_base !== (test_prec == 0 ? writes % 2 : 0))
                $fatal(1, "WRF bank group mismatch");
            for (int b = 0; b < 8; b++) begin
                if (test_prec == 0 || b < test_prec) begin
                    if (data[b] !== pattern(wr_addr, bank_base * 8 + b))
                        $fatal(1, "GLD data mismatch prec=%0d compact=%0d K=%0d bank=%0d got=%h",
                               test_prec, test_compact, wr_addr, b, data[b]);
                end else if (test_compact && data[b] !== 0)
                    $fatal(1, "unused compact bank not zero");
            end
            writes++;
        end
        if (arf_wr_en && batch_test) begin
            if (arf_wr_addr !== a_writes / 2 ||
                arf_wr_bank_base !== a_writes % 2)
                $fatal(1, "GLD.A batch address/group mismatch write=%0d addr=%0d group=%0d",
                       a_writes, arf_wr_addr, arf_wr_bank_base);
            for (int lane = 0; lane < 16; lane++) begin
                if (arf_data[lane] !== (16'h4000 | ((a_writes / 2) << 5) |
                                        ((a_writes % 2) * 16 + lane)))
                    $fatal(1, "GLD.A batch payload mismatch K=%0d token=%0d got=%h",
                           a_writes / 2, (a_writes % 2) * 16 + lane, arf_data[lane]);
            end
            a_writes++;
        end
        if (arf_wr_en && transpose_test) begin
            if (arf_wr_addr !== a_writes || arf_wr_bank_base !== 0)
                $fatal(1, "GLD.A transpose address/group mismatch write=%0d addr=%0d group=%0d",
                       a_writes, arf_wr_addr, arf_wr_bank_base);
            for (int token = 0; token < 32; token++) begin
                if (arf_data[token] !== (16'h4000 | (token << 5) | a_writes))
                    $fatal(1, "GLD.A transpose payload mismatch K=%0d token=%0d got=%h",
                           a_writes, token, arf_data[token]);
            end
            a_writes++;
        end
        if (arf_wr_en && strided_transpose_test) begin
            if (arf_wr_addr !== a_writes || arf_wr_bank_base !== 0)
                $fatal(1, "strided GLD.A transpose address mismatch write=%0d addr=%0d",
                       a_writes, arf_wr_addr);
            for (int token = 0; token < 32; token++) begin
                if (token < 8) begin
                    if (arf_data[token] !== (16'h4000 | (token << 5) | a_writes))
                        $fatal(1, "strided transpose payload mismatch K=%0d token=%0d got=%h",
                               a_writes, token, arf_data[token]);
                end else if (arf_data[token] !== 16'd0)
                    $fatal(1, "strided transpose inactive row is nonzero token=%0d", token);
            end
            a_writes++;
        end
        if (rd_en && (batch_test || transpose_test || strided_transpose_test))
            a_reads++;
    end
    task automatic run_case(input int p, k, input bit compact_flag, input int latency);
        @(negedge clk);
        test_prec=p; test_k=k; test_compact=compact_flag;
        per_line=compact_flag ? (p==1 ? 8 : p==2 ? 4 : 2) : 1;
        for(int i=0;i<256;i++) mem[i]='0;
        for(int row=0;row<k;row++) begin
            if (p==0) begin
                for(int b=0;b<16;b++) mem[row*2+b/8][(b%8)*32 +:32]=pattern(row,b);
            end else begin
                for(int b=0;b<p;b++)
                    mem[row/per_line][((row%per_line)*p+b)*32 +:32]=pattern(row,b);
            end
        end
        writes=0; reads=0; cfg_k=k; prec=p; compact_mode=compact_flag; read_delay=latency; issue_cycle=cycle; done_cycle=-1; cmd=1;
        @(negedge clk); cmd=0; cfg_k=1; prec=0; compact_mode=0;
        // Change command inputs after acceptance to verify latching.
        wait(done); done_cycle=cycle;
        @(negedge clk);
        if(writes != (p==0 ? k*2 : k)) $fatal(1,"incorrect write count %0d",writes);
        if(reads != (p==0 ? k*2 : (k+per_line-1)/per_line)) $fatal(1,"incorrect read count %0d",reads);
        $display("TILE_GLD_LATENCY|precision=%0d|K=%0d|compact=%0d|issue=%0d|done=%0d|gld_cycles=%0d|reads=%0d|writes=%0d|memory_delay=%0d",p,k,compact_flag,issue_cycle,done_cycle,done_cycle-issue_cycle,reads,writes,latency);
        repeat(2) @(negedge clk);
    endtask
    task automatic run_transpose_activation();
        @(negedge clk);
        for (int i=0; i<256; i++) mem[i]='0;
        for (int token=0; token<32; token++)
            for (int k=0; k<32; k++)
                mem[token*2 + k/16][(k%16)*16 +: 16] =
                    16'h4000 | (token << 5) | k;
        cfg_k=32; cfg_m=32; batch_a=1; transpose_a=1;
        read_delay=1; // the transpose controller pipelines one request per cycle
        transpose_test=1; a_writes=0; a_reads=0; a_cmd=1;
        @(negedge clk); a_cmd=0;
        wait(a_done);
        @(negedge clk);
        if (a_writes != 32 || a_reads != 64)
            $fatal(1, "GLD.A transpose count mismatch reads=%0d writes=%0d", a_reads, a_writes);
        $display("PASS GLD.A token-major transpose M=32 K=32 reads=%0d writes=%0d", a_reads, a_writes);
        transpose_test=0; transpose_a=0; batch_a=0; cfg_m=1; cfg_k=1;
        repeat(2) @(negedge clk);
    endtask
    task automatic run_batch_activation();
        @(negedge clk);
        for (int i=0; i<256; i++) mem[i]='0;
        for (int k=0; k<8; k++)
            for (int group=0; group<2; group++)
                for (int lane=0; lane<16; lane++)
                    mem[k*2+group][lane*16 +: 16] =
                        16'h4000 | (k << 5) | (group*16 + lane);
        cfg_k=8; cfg_m=32; batch_a=1; batch_test=1;
        a_writes=0; a_reads=0; a_cmd=1;
        @(negedge clk); a_cmd=0;
        wait(a_done);
        @(negedge clk);
        if (a_writes != 16 || a_reads != 16)
            $fatal(1, "GLD.A batch count mismatch reads=%0d writes=%0d", a_reads, a_writes);
        $display("PASS GLD.A batch M=32 K=8 reads=%0d writes=%0d", a_reads, a_writes);
        batch_test=0; batch_a=0; cfg_m=1; cfg_k=1;
        repeat(2) @(negedge clk);
    endtask
    task automatic run_strided_transpose_activation();
        @(negedge clk);
        for (int i = 0; i < 2048; i++) mem[i] = '0;
        for (int token = 0; token < 8; token++)
            for (int k = 0; k < 32; k++)
                mem[token * 128 + k / 16][(k % 16) * 16 +: 16] =
                    16'h4000 | (token << 5) | k;
        cfg_k = 32; cfg_m = 8; batch_a = 1; transpose_a = 1;
        a_stride_lines = 9'd128;
        read_delay = 1; strided_transpose_test = 1;
        a_writes = 0; a_reads = 0; a_cmd = 1;
        @(negedge clk); a_cmd = 0;
        wait(a_done);
        @(negedge clk);
        if (a_writes != 32 || a_reads != 16)
            $fatal(1, "strided GLD.A count mismatch reads=%0d writes=%0d",
                   a_reads, a_writes);
        $display("PASS GLD.A strided transpose M=8 K=32 row_stride_lines=128 reads=%0d writes=%0d",
                 a_reads, a_writes);
        strided_transpose_test = 0; transpose_a = 0; batch_a = 0;
        a_stride_lines = 0; cfg_m = 1; cfg_k = 1;
        repeat (2) @(negedge clk);
    endtask
    initial begin
        repeat(4) @(negedge clk); rst_n=1;
        run_case(0,32,0,1);
        for(int p=1;p<=3;p++) run_case(p,32,1,1);
        run_case(0,64,0,1);
        run_case(0,64,0,3);
        run_case(0,128,0,2);
        for(int p=1;p<=3;p++) begin
            run_case(p,64,0,1);
            run_case(p,64,0,3);
            run_case(p,64,1,1);
            run_case(p,37,1,3);
            run_case(p,128,1,2);
            run_case(p,128,0,2);
            run_case(p,256,1,2);
        end
        run_batch_activation();
        run_strided_transpose_activation();
        run_transpose_activation();
        $display("PASS all GLD compact/legacy cases"); $finish;
    end
    initial begin #1000000; $fatal(1,"GLD timeout"); end
endmodule
