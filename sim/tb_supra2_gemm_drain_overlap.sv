`default_nettype none
// Checks two output-column tiles, metadata, per-column bias and overlap.
module tb_supra2_gemm_drain_overlap;
    localparam int L=32;
    logic clk=0, rst_n=0, start=0;
    always #5 clk=~clk;
    logic f16_wr_en=0, f16_wr_sel=0, sb_wr_en=0;
    logic [15:0] f16_wr_beat=0, sb_wr_addr=0;
    logic [31:0] sb_wr_data=0;
    wire busy, done, out_valid, array_en;
    wire [15:0] out_row, out_ct;
    wire [511:0] out_data;
    integer outputs=0, overlap_cycles=0;
    logic [63:0] seen='0;

    supra2_gemm_engine #(.MAX_M(32),.MAX_N(64),.MAX_K(256),
        .MAX_F16_K(32),.MAX_F16_N(64)) dut (
        .clk(clk),.rst_n(rst_n),.start(start),.busy(busy),.done(done),
        .cfg_fp16(1'b1),.cfg_fp16_bias_en(1'b1),
        .cfg_fp16_act_scale(32'h3f800000),.cfg_fp16_wt_scale(32'h3f800000),
        .cfg_m(16'd32),.cfg_n(16'd64),.cfg_k(16'd32),.cfg_h(9'd32),
        .cfg_ni0(16'd0),.cfg_ni1(16'd0),.cfg_scale(32'h3f800000),
        .raw_row(),.raw_beat(),.raw_prefetch_en(),.raw_prefetch_row(),
        .raw_prefetch_beat(),.raw_rd_data(512'd0),
        .f16_wr_en(f16_wr_en),.f16_wr_sel(f16_wr_sel),
        .f16_wr_beat(f16_wr_beat),.f16_wr_data(512'd0),
        .wt_wr_en(1'b0),.wt_wr_beat(16'd0),.wt_wr_data(256'd0),
        .weight_write_slot(1'b0),.weight_read_slot(1'b0),.weight_load_k(16'd32),
        .sb_wr_en(sb_wr_en),.sb_wr_sel(1'b1),.sb_wr_addr(sb_wr_addr),
        .sb_wr_data(sb_wr_data),.out_valid(out_valid),.out_row(out_row),
        .out_ct(out_ct),.out_data(out_data),.shared_array_clear(),
        .shared_array_en(array_en),.shared_array_a(),.shared_array_b(),
        .shared_array_acc(),.shared_array_acc_valid(1'b0));

    always @(posedge clk) if (rst_n) begin
        if (out_valid && array_en) overlap_cycles <= overlap_cycles+1;
        if (out_valid) begin
            if (out_row>=32 || out_ct>=2 || seen[int'(out_ct)*32+int'(out_row)])
                $fatal(1,"duplicate or invalid output row=%0d coltile=%0d",out_row,out_ct);
            seen[int'(out_ct)*32+int'(out_row)] = 1'b1;
            for (int lane=0;lane<L;lane++)
                if (out_data[lane*16+:16] !== (out_ct==0 ? 16'h3c00 : 16'h4000))
                    $fatal(1,"wrong output row=%0d tile=%0d lane=%0d got=%h",out_row,out_ct,lane,out_data[lane*16+:16]);
            outputs = outputs+1;
        end
        if (done) begin
            if (outputs!=64 || seen!==64'hffffffffffffffff || overlap_cycles==0)
                $fatal(1,"bad completion outputs=%0d overlap=%0d",outputs,overlap_cycles);
            $display("SUPRA2_GEMM_DRAIN_OVERLAP_PASS outputs=%0d overlap_cycles=%0d",outputs,overlap_cycles);
            $finish;
        end
    end

    initial begin
        repeat (4) @(negedge clk);
        rst_n=1;
        for (int i=0;i<32;i++) begin
            @(negedge clk); f16_wr_en=1; f16_wr_sel=0; f16_wr_beat=16'(i);
        end
        for (int i=0;i<64;i++) begin
            @(negedge clk); f16_wr_en=1; f16_wr_sel=1; f16_wr_beat=16'(i);
        end
        @(negedge clk); f16_wr_en=0;
        for (int i=0;i<64;i++) begin
            @(negedge clk); sb_wr_en=1; sb_wr_addr=16'(i);
            sb_wr_data=i<32 ? 32'h3f800000 : 32'h40000000;
        end
        @(negedge clk); sb_wr_en=0; start=1;
        @(negedge clk); start=0;
        repeat (2000) @(negedge clk);
        $fatal(1,"GEMM overlap watchdog");
    end
endmodule
`default_nettype wire
