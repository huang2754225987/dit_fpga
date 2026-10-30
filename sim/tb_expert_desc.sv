// =============================================================================
// tb_expert_desc.sv - standalone test of expert_desc_table + host load protocol
// =============================================================================
// Loads the 180-entry expert_desc.bin exactly the way the host would (via the
// AXI-Lite protocol: IDX then W0/W1/W2 words), then verifies the three read
// ports (weight DMA / GLD_W / GEMM) and the topk-driven scale-base mux against
// the values in the .bin/.txt produced by gen_expert_desc.py.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_expert_desc;
    logic clk, rst_n;
    initial begin clk = 0; forever #5 clk = ~clk; end

    // ---- DUT ports ----
    logic       wr_en;
    logic [9:0] wr_addr;
    logic [31:0] wr_data;
    logic [1:0]  wr_sel;

    logic [5:0] rd_expert_a, rd_expert_b, rd_expert_c;
    logic [1:0] rd_proj_a, rd_proj_b, rd_proj_c;
    logic [31:0] da_wb, da_sb, da_attr;
    logic [31:0] db_wb, db_sb, db_attr;
    logic [31:0] dc_wb, dc_sb, dc_attr;
    logic [5:0]  topk_expert [0:3];
    logic [31:0] exp_scale_base_o [0:3][0:2];

    expert_desc_table #(
        .N_EXPERTS (60),
        .N_PROJ    (3)
    ) u_dut (
        .clk(clk), .rst_n(rst_n),
        .wr_en(wr_en), .wr_addr(wr_addr), .wr_data(wr_data), .wr_sel(wr_sel),
        .rd_expert_a(rd_expert_a), .rd_proj_a(rd_proj_a),
        .desc_a_weight_base(da_wb), .desc_a_scale_base(da_sb), .desc_a_attr(da_attr),
        .rd_expert_b(rd_expert_b), .rd_proj_b(rd_proj_b),
        .desc_b_weight_base(db_wb), .desc_b_scale_base(db_sb), .desc_b_attr(db_attr),
        .rd_expert_c(rd_expert_c), .rd_proj_c(rd_proj_c),
        .desc_c_weight_base(dc_wb), .desc_c_scale_base(dc_sb), .desc_c_attr(dc_attr),
        .topk_expert(topk_expert),
        .exp_scale_base_o(exp_scale_base_o)
    );

    // ---- read expert_desc.bin ----
    integer fd, r;
    reg [7:0] bin [0:180*12-1];
    int n_entries = 180;

    int idx, w0, w1, w2;
    int err_cnt = 0;

    // Load file into byte array
    task load_bin;
        begin
            fd = $fopen("D:/graduation_design/npu-main/sim/modelsim/expert_desc.bin", "rb");
            if (fd == 0) begin
                $display("FAIL: cannot open expert_desc.bin");
                $finish;
            end
            r = $fread(bin, fd);
            $fclose(fd);
            $display("[TB] read %0d bytes from expert_desc.bin (expect %0d)", r, 180*12);
        end
    endtask

    // One AXI-Lite-style write: addr reg then W0/W1/W2 (1 cycle pulse each)
    task write_desc(input int e, input int p);
        automatic int i = e*3 + p;
        begin
            @(posedge clk);
            // W0
            wr_en <= 1'b1; wr_addr <= i; wr_sel <= 2'd0;
            wr_data <= {bin[i*12+3], bin[i*12+2], bin[i*12+1], bin[i*12+0]};
            @(posedge clk);
            wr_en <= 1'b0;
            // W1
            @(posedge clk);
            wr_en <= 1'b1; wr_addr <= i; wr_sel <= 2'd1;
            wr_data <= {bin[i*12+7], bin[i*12+6], bin[i*12+5], bin[i*12+4]};
            @(posedge clk);
            wr_en <= 1'b0;
            // W2
            @(posedge clk);
            wr_en <= 1'b1; wr_addr <= i; wr_sel <= 2'd2;
            wr_data <= {bin[i*12+11], bin[i*12+10], bin[i*12+9], bin[i*12+8]};
            @(posedge clk);
            wr_en <= 1'b0;
        end
    endtask

    task check_a(input int e, input int p);
        automatic int i = e*3+p;
        begin
            @(posedge clk);
            rd_expert_a <= e; rd_proj_a <= p;
            @(posedge clk);
            w0 = {bin[i*12+3], bin[i*12+2], bin[i*12+1], bin[i*12+0]};
            w1 = {bin[i*12+7], bin[i*12+6], bin[i*12+5], bin[i*12+4]};
            w2 = {bin[i*12+11], bin[i*12+10], bin[i*12+9], bin[i*12+8]};
            if (da_wb !== w0 || da_sb !== w1 || da_attr !== w2) begin
                $display("FAIL portA exp=%0d proj=%0d: got wb=%08x sb=%08x attr=%08x",
                         e, p, da_wb, da_sb, da_attr);
                $display("                              exp wb=%08x sb=%08x attr=%08x",
                         w0, w1, w2);
                err_cnt = err_cnt + 1;
            end
            @(posedge clk);
        end
    endtask

    // Same sampling check for read port B (GLD_W): db_wb/db_sb/db_attr were
    // wired but never verified - sweep all 180 entries through it.
    task check_b(input int e, input int p);
        automatic int i = e*3+p;
        begin
            @(posedge clk);
            rd_expert_b <= e; rd_proj_b <= p;
            @(posedge clk);
            w0 = {bin[i*12+3], bin[i*12+2], bin[i*12+1], bin[i*12+0]};
            w1 = {bin[i*12+7], bin[i*12+6], bin[i*12+5], bin[i*12+4]};
            w2 = {bin[i*12+11], bin[i*12+10], bin[i*12+9], bin[i*12+8]};
            if (db_wb !== w0 || db_sb !== w1 || db_attr !== w2) begin
                $display("FAIL portB exp=%0d proj=%0d: got wb=%08x sb=%08x attr=%08x",
                         e, p, db_wb, db_sb, db_attr);
                $display("                              exp wb=%08x sb=%08x attr=%08x",
                         w0, w1, w2);
                err_cnt = err_cnt + 1;
            end
            @(posedge clk);
        end
    endtask

    // W1 (scale_base) of entry (e*3+p) straight from expert_desc.bin, so the
    // scale-mux expectations track the .bin instead of hardcoded constants.
    function automatic logic [31:0] bin_scale_base(input int e, input int p);
        automatic int i = e*3 + p;
        return {bin[i*12+7], bin[i*12+6], bin[i*12+5], bin[i*12+4]};
    endfunction

    initial begin
        rst_n = 0; wr_en = 0; wr_addr = 0; wr_data = 0; wr_sel = 0;
        rd_expert_a = 0; rd_proj_a = 0;
        rd_expert_b = 0; rd_proj_b = 0;
        rd_expert_c = 0; rd_proj_c = 0;
        topk_expert[0]=0; topk_expert[1]=0; topk_expert[2]=0; topk_expert[3]=0;
        repeat(4) @(posedge clk);
        rst_n = 1;
        repeat(2) @(posedge clk);

        load_bin;

        // Load ALL 180 entries via the host protocol (loop)
        for (int e = 0; e < 60; e++)
            for (int p = 0; p < 3; p++)
                write_desc(e, p);

        $display("[TB] loaded all %0d descriptors", n_entries);

        // ---- Check a sample of entries through port A (weight DMA) ----
        check_a(33, 0);
        check_a(33, 1);
        check_a(33, 2);
        check_a(52, 0);
        check_a(22, 1);
        check_a(47, 2);
        check_a(0, 0);    // invalid (not present) -> all zeros
        check_a(59, 2);   // invalid

        // ---- Full sweep: ALL 180 entries through read port B (GLD_W) ----
        for (int e = 0; e < 60; e++)
            for (int p = 0; p < 3; p++)
                check_b(e, p);
        $display("[TB] port B full sweep done (%0d entries)", n_entries);
        // GEMM port (C) reads same entries
        begin
            int e = 33; int p = 0; int i = e*3+p;
            @(posedge clk); rd_expert_c <= e; rd_proj_c <= p;
            @(posedge clk);
            w0 = {bin[i*12+3], bin[i*12+2], bin[i*12+1], bin[i*12+0]};
            w1 = {bin[i*12+7], bin[i*12+6], bin[i*12+5], bin[i*12+4]};
            w2 = {bin[i*12+11], bin[i*12+10], bin[i*12+9], bin[i*12+8]};
            if (dc_wb !== w0 || dc_sb !== w1 || dc_attr !== w2) begin
                $display("FAIL portC exp=33 proj=0"); err_cnt = err_cnt + 1;
            end
        end

        // ---- Scale-base mux: topk = [33,52,22,47] -> slot0..3 = those ----
        @(posedge clk);
        topk_expert[0] <= 33; topk_expert[1] <= 52;
        topk_expert[2] <= 22; topk_expert[3] <= 47;
        @(posedge clk);
        begin
            // Expected scale bases are read back from expert_desc.bin (no
            // magic numbers): slot k uses topk_expert[k]; W1 of entry e*3+p.
            logic [31:0] s0, s1, s2, s3;
            s0 = bin_scale_base(33, 0);   // slot0 = exp33 gate
            s1 = bin_scale_base(52, 1);   // slot1 = exp52 up
            s2 = bin_scale_base(22, 2);   // slot2 = exp22 down
            s3 = bin_scale_base(47, 0);   // slot3 = exp47 gate
            $display("[TB] scale mux: slot0(33,g)=%08x slot1(52,u)=%08x slot2(22,d)=%08x slot3(47,g)=%08x",
                     exp_scale_base_o[0][0], exp_scale_base_o[1][1], exp_scale_base_o[2][2], exp_scale_base_o[3][0]);
            $display("          exp:                       %08x              %08x              %08x              %08x",
                     s0, s1, s2, s3);
            if (exp_scale_base_o[0][0] !== s0) begin $display("FAIL scale mux slot0"); err_cnt++; end
            if (exp_scale_base_o[1][1] !== s1) begin $display("FAIL scale mux slot1"); err_cnt++; end
            if (exp_scale_base_o[2][2] !== s2) begin $display("FAIL scale mux slot2"); err_cnt++; end
            if (exp_scale_base_o[3][0] !== s3) begin $display("FAIL scale mux slot3"); err_cnt++; end
        end

        if (err_cnt == 0)
            $display("[TB] PASS expert_desc_table: %0d descriptors loaded, port A samples + port B full sweep + scale mux verified", n_entries);
        else
            $display("[TB] CHECK %0d errors", err_cnt);
        $finish;
    end

endmodule
`default_nettype wire