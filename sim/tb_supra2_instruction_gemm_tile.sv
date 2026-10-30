`default_nettype none

// Executes one compiler-generated QKV GEMM_TILE command using the production
// 32x32 GEMM engine and block-13 operator golden fixture.
module tb_supra2_instruction_gemm_tile;
    localparam int M=32, MAX_N=32, MAX_K=576, MAX_N_TILES=MAX_N/32;
    logic clk=0, rst_n=0, start=0;
    logic [15:0] cfg_n, cfg_k;
    logic [8:0] cfg_h;
    wire busy, done, out_valid;
    wire [15:0] raw_row, raw_beat, raw_prefetch_row, raw_prefetch_beat, out_row, out_ct;
    wire raw_prefetch_en;
    logic [511:0] raw_data;
    wire [511:0] out_data;
    wire shared_array_clear, shared_array_en;
    wire signed [7:0] shared_array_a [M], shared_array_b [M];
    wire signed [31:0] shared_array_acc [M][M];
    wire shared_array_acc_valid;
    logic wt_en=0, sb_en=0, sb_sel=0;
    logic [15:0] wt_beat=0, sb_addr=0;
    logic [255:0] wt_data=0;
    logic [31:0] sb_data=0;
    logic [15:0] raw_mem [0:M*MAX_K-1];
    logic [7:0] weight_mem [0:MAX_N*MAX_K-1];
    logic [31:0] scale_mem [0:MAX_N-1], bias_mem [0:MAX_N-1];
    logic [15:0] expected_mem [0:M*MAX_N-1];
    logic [M*MAX_N_TILES-1:0] seen='0;
    logic [127:0] program_rom [0:1];
    integer n_tiles, k_tiles, n, k, h, source_pc;
    integer output_beats=0, input_stall_cycles=0, array_busy_cycles=0;
    integer engine_busy_cycles=0, execution_cycles=0, measured=0;
    integer weight_write_beats=0, scale_bias_writes=0;
    integer elapsed_cycles=0, errors=0;
    string program_file, raw_file, weight_file, scale_file, bias_file;
    string expected_file, stats_file;
    integer stats_fd;

    always #5 clk=~clk;
    initial begin #100_000_000; $fatal(1,"instruction GEMM tile timeout"); end

    supra2_gemm_engine #(.MAX_M(M),.MAX_N(MAX_N),.MAX_K(MAX_K),
        .MAX_WEIGHT_ELEMS(MAX_N*MAX_K),.EXTERNAL_ARRAY(1'b1)) dut (
        .clk(clk),.rst_n(rst_n),.start(start),.busy(busy),.done(done),
        .cfg_fp16(1'b0),.cfg_fp16_bias_en(1'b0),
        .cfg_fp16_act_scale(32'h41800000),.cfg_fp16_wt_scale(32'h41800000),
        .cfg_m(16'(M)),.cfg_n(cfg_n),.cfg_k(cfg_k),.cfg_h(cfg_h),
        .cfg_ni0(16'd0),.cfg_ni1(16'd0),.cfg_scale(32'h3a000000),
        .raw_row(raw_row),.raw_beat(raw_beat),.raw_prefetch_en(raw_prefetch_en),
        .raw_prefetch_row(raw_prefetch_row),.raw_prefetch_beat(raw_prefetch_beat),
        .raw_rd_data(raw_data),
        .f16_wr_en(1'b0),.f16_wr_sel(1'b0),.f16_wr_beat(16'd0),.f16_wr_data(512'd0),
        .wt_wr_en(wt_en),.wt_wr_beat(wt_beat),.wt_wr_data(wt_data),
        .weight_write_slot(1'b0),.weight_read_slot(1'b0),.weight_load_k(16'd0),
        .sb_wr_en(sb_en),.sb_wr_sel(sb_sel),.sb_wr_addr(sb_addr),.sb_wr_data(sb_data),
        .out_valid(out_valid),.out_row(out_row),.out_ct(out_ct),.out_data(out_data),
        .shared_array_clear(shared_array_clear),.shared_array_en(shared_array_en),
        .shared_array_a(shared_array_a),.shared_array_b(shared_array_b),
        .shared_array_acc(shared_array_acc),.shared_array_acc_valid(shared_array_acc_valid)
    );

    systolic_array #(.M(M),.N(M),.DATA_W(8),.ACC_W(32),
        .RUNTIME_UNSIGNED_A(1'b1),.PIPE_STAGE(32)) shared_array (
        .clk(clk),.rst_n(rst_n),.clear_acc(shared_array_clear),.en(shared_array_en),
        .a_unsigned(1'b0),.active_rows(6'(M)),.a_col(shared_array_a),
        .b_row(shared_array_b),.acc_out(shared_array_acc),.acc_valid(shared_array_acc_valid)
    );

    always_comb begin
        raw_data='0;
        for (int lane=0; lane<32; lane++)
            if (raw_row<M && int'(raw_beat)*32+lane<k)
                raw_data[lane*16+:16]=raw_mem[int'(raw_row)*k+int'(raw_beat)*32+lane];
    end

    always @(posedge clk) begin
        if (rst_n) begin
            elapsed_cycles <= elapsed_cycles+1;
            if (measured) begin
                execution_cycles <= execution_cycles+1;
                if (busy) engine_busy_cycles <= engine_busy_cycles+1;
                if (shared_array_en) array_busy_cycles <= array_busy_cycles+1;
                if (dut.fe_in_valid && !dut.fe_h_in_ready)
                    input_stall_cycles <= input_stall_cycles+1;
            end
            if (out_valid) begin
                if (out_row>=M || out_ct>=n_tiles ||
                    seen[int'(out_row)*MAX_N_TILES+int'(out_ct)])
                    $fatal(1,"bad/duplicate output row=%0d N-tile=%0d",out_row,out_ct);
                seen[int'(out_row)*MAX_N_TILES+int'(out_ct)]=1'b1;
                for (int lane=0;lane<32;lane++) begin
                    int idx;
                    idx=int'(out_row)*n+int'(out_ct)*32+lane;
                    if (out_data[lane*16+:16] !== expected_mem[idx]) begin
                        errors++;
                        $fatal(1,"instruction GEMM golden mismatch row=%0d n_tile=%0d lane=%0d got=%h expected=%h",
                            out_row,out_ct,lane,out_data[lane*16+:16],expected_mem[idx]);
                    end
                end
                output_beats++;
            end
        end
    end

    initial begin
        if (!$value$plusargs("PROGRAM_FILE=%s",program_file) ||
            !$value$plusargs("RAW=%s",raw_file) ||
            !$value$plusargs("WEIGHT=%s",weight_file) ||
            !$value$plusargs("SCALE=%s",scale_file) ||
            !$value$plusargs("BIAS=%s",bias_file) ||
            !$value$plusargs("EXPECTED=%s",expected_file) ||
            !$value$plusargs("STATS=%s",stats_file) ||
            !$value$plusargs("SOURCE_PC=%d",source_pc))
            $fatal(1,"missing single-tile instruction/fixture arguments");
        $readmemh(program_file,program_rom,0,1);
        if (program_rom[0][127:120] != 8'd2 || program_rom[0][119:112] != 8'd1 ||
            program_rom[0][111:104] != 0 || program_rom[0][39:24] != 0 ||
            program_rom[0][7:0] != 0 ||
            program_rom[1] !== 128'h0f000000000000000000000000000000)
            $fatal(1,"invalid isolated GEMM_TILE instruction stream");
        n_tiles=program_rom[0][103:96];
        k_tiles=program_rom[0][95:88];
        n=n_tiles*32; k=k_tiles*32; h=64;
        if (n!=MAX_N || k!=MAX_K || n_tiles!=MAX_N_TILES)
            $fatal(1,"instruction dimensions do not match QKV fixture n=%0d k=%0d",n,k);
        cfg_n=16'(n); cfg_k=16'(k); cfg_h=9'(h);
        $readmemh(raw_file,raw_mem,0,M*k-1);
        $readmemh(weight_file,weight_mem,0,n*k-1);
        $readmemh(scale_file,scale_mem,0,n-1);
        $readmemh(bias_file,bias_mem,0,n-1);
        $readmemh(expected_file,expected_mem,0,M*n-1);

        repeat(4) @(negedge clk);
        rst_n=1;
        for (int wb=0;wb<n*k/32;wb++) begin
            @(negedge clk); wt_en=1; wt_beat=16'(wb);
            for (int lane=0;lane<32;lane++) wt_data[lane*8+:8]=weight_mem[wb*32+lane];
            weight_write_beats++;
        end
        @(negedge clk); wt_en=0;
        for (int sel=0;sel<2;sel++) for (int col=0;col<n;col++) begin
            sb_en=1; sb_sel=1'(sel); sb_addr=16'(col);
            sb_data=sel ? bias_mem[col] : scale_mem[col];
            @(negedge clk); scale_bias_writes++;
        end
        sb_en=0;
        stats_fd=$fopen(stats_file,"w");
        if (stats_fd==0) $fatal(1,"cannot create instruction stats file");
        measured=1; start=1;
        @(negedge clk); start=0;
        wait(done); @(negedge clk); measured=0;
        if (seen !== {M*MAX_N_TILES{1'b1}})
            $fatal(1,"incomplete output tile bitmap=%h",seen);
        if (output_beats != M*n_tiles || busy || errors)
            $fatal(1,"incomplete instruction GEMM outputs=%0d expected=%0d",output_beats,M*n_tiles);
        $fdisplay(stats_fd,"source_pc,opcode,stage,row_tile,n_tiles,k_tiles,weight_write_beats,scale_bias_writes,execution_cycles,engine_busy_cycles,array_busy_cycles,hadamard_stall_cycles,output_beats");
        $fdisplay(stats_fd,"%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
            source_pc,program_rom[0][127:120],program_rom[0][119:112],program_rom[0][111:104],
            n_tiles,k_tiles,weight_write_beats,scale_bias_writes,execution_cycles,
            engine_busy_cycles,array_busy_cycles,input_stall_cycles,output_beats);
        $fclose(stats_fd);
        $display("SUPRA2_TILE_GEMM_PASS source_pc=%0d stage=QKV row_tile=0 M=%0d N=%0d K=%0d cycles=%0d busy=%0d array=%0d stalls=%0d output_beats=%0d",
            source_pc,M,n,k,execution_cycles,engine_busy_cycles,array_busy_cycles,input_stall_cycles,output_beats);
        $finish;
    end
endmodule

`default_nettype wire
