`timescale 1ns/1ps
module tb_route_combine;
    logic clk=0, rst_n=0;
    always #5 clk=~clk;
    logic preload=0;
    logic [7:0] preload_addr=0;
    logic [31:0] preload_data=0;
    int count=0;
    top #(.P_UCODE_DEPTH(256)) dut (
        .clk(clk), .rst_n(rst_n),
        .s_axil_awaddr(32'd0), .s_axil_awvalid(1'b0),
        .s_axil_wdata(32'd0), .s_axil_wstrb(4'd0), .s_axil_wvalid(1'b0),
        .s_axil_bready(1'b1), .s_axil_araddr(32'd0), .s_axil_arvalid(1'b0), .s_axil_rready(1'b1),
        .m_axi_arready(1'b0), .m_axi_rid(4'd0), .m_axi_rdata(256'd0),
        .m_axi_rresp(2'd0), .m_axi_rlast(1'b0), .m_axi_rvalid(1'b0),
        .m_axi_awready(1'b0), .m_axi_wready(1'b0), .m_axi_bid(4'd0),
        .m_axi_bresp(2'd0), .m_axi_bvalid(1'b0),
        .tb_ucode_wr_en(preload), .tb_ucode_wr_addr(preload_addr), .tb_ucode_wr_data(preload_data)
    );
    task automatic emit(input logic[31:0] inst);
        @(negedge clk); preload=1; preload_addr=count; preload_data=inst; count++;
    endtask
    function automatic logic [31:0] ins(input int op, vd, vs1, vs2, imm);
        return (op<<28)|(vd<<24)|(vs1<<20)|(vs2<<16)|imm;
    endfunction
    task automatic combine(input int token, result_reg);
        emit(ins(0,0,0,0,16'h6000|token));
        for(int slot=1;slot<=4;slot++) begin
            emit(ins(0,2,0,slot,16'h7000));
            emit(ins(11,slot==1 ? 1:0,slot+3,2,0));
            if(slot!=1) emit(ins(10,1,1,0,0));
        end
        emit(ins(10,result_reg,1,15,0));
    endtask
    initial begin
        repeat(5) @(negedge clk); rst_n=1;
        combine(5,8);
        combine(6,9);
        emit(32'hf0000000);
        @(negedge clk); preload=0;
        force dut.reg_ucode_len = count;
        // Unit fixtures represent independently computed expert outputs.
        // This test exercises fetch/decode/scoreboard/ROUTE/RWEIGHT/MUL/ADD,
        // not DDR or GEMM, which have separate tests.
        for(int r=0;r<16;r++) dut.u_postproc_rf.regs[r]='0;
        dut.u_postproc_rf.regs[4]={32{16'h3c00}};
        dut.u_postproc_rf.regs[5]={32{16'h4000}};
        dut.u_postproc_rf.regs[6]={32{16'h4200}};
        dut.u_postproc_rf.regs[7]={32{16'h4400}};
        force dut.start_pulse=1;
        @(negedge clk); force dut.start_pulse=0;
        dut.u_topk_unit.route_valid[5]=1;
        dut.u_topk_unit.route_valid[6]=1;
        for(int s=0;s<4;s++) begin
            dut.u_topk_unit.route_e[5][s]=s+10;
            dut.u_topk_unit.route_e[6][s]=s+40;
        end
        dut.u_topk_unit.route_w[5][0]=16'h3400;
        dut.u_topk_unit.route_w[5][1]=16'h3000;
        dut.u_topk_unit.route_w[5][2]=16'h3800;
        dut.u_topk_unit.route_w[5][3]=16'h2c00;
        dut.u_topk_unit.route_w[6][0]=16'h3800;
        dut.u_topk_unit.route_w[6][1]=16'h3400;
        dut.u_topk_unit.route_w[6][2]=16'h3000;
        dut.u_topk_unit.route_w[6][3]=16'h2c00;
        wait(dut.program_end);
        @(negedge clk);
        if(dut.execution_fault) $fatal(1,"unexpected execution fault");
        for(int lane=0;lane<32;lane++) begin
            if(dut.u_postproc_rf.regs[8][lane*16+:16] !== 16'h4080)
                $fatal(1,"token5 lane%0d expected 2.25 got %h",lane,dut.u_postproc_rf.regs[8][lane*16+:16]);
            if(dut.u_postproc_rf.regs[9][lane*16+:16] !== 16'h3e80)
                $fatal(1,"token6 lane%0d expected 1.625 got %h",lane,dut.u_postproc_rf.regs[9][lane*16+:16]);
        end
        if(dut.topk_idx_arr[0] !== 6'd40) $fatal(1,"token6 expert ID not selected");
        $display("PASS route/combine through instruction fetch: 2 tokens, 4 weighted experts, 32 lanes");
        // Neither missing descriptors nor an address exactly one tile past
        // the projection may issue AXI. Both must expose STATUS.error.
        for(int fault_case=0;fault_case<2;fault_case++) begin
            @(negedge clk); rst_n=0;
            repeat(4) @(negedge clk); rst_n=1; count=0;
            emit(ins(0,0,0,0,16'h6005));
            emit(ins(12,1,0,1,fault_case==0 ? 0 : 2816));
            emit(32'hf0000000);
            @(negedge clk); preload=0;
            force dut.cfg_K=16'd32;
            force dut.start_pulse=1;
            @(negedge clk); force dut.start_pulse=0;
            dut.u_topk_unit.route_valid[5]=1;
            for(int s=0;s<4;s++) dut.u_topk_unit.route_e[5][s]=0;
            if(fault_case==1)
                dut.u_expert_desc_table.mem[0][95:64]=32'h80000000 | (16<<23) | (64<<16) | (44<<9) | (1<<7) | 2;
            wait(dut.execution_fault);
            repeat(5) @(negedge clk);
            if(dut.program_end) $fatal(1,"faulting program incorrectly completed");
        end
        $display("PASS expert DMA faults: invalid descriptor and projection bounds block AXI");
        $finish;
    end
    always @(posedge clk) if(rst_n && dut.m_axi_arvalid)
        $fatal(1,"route/weight vector test unexpectedly accessed DDR");
    always @(posedge clk) if(dut.fetch_instr_ready)
        $display("ISSUE %h busy=%h",dut.lc_dec_instr,dut.res_busy);
    initial begin #1000000;
        $display("DEBUG busy=%h fault=%b topkbusy=%b toplatch=%b ve=%b instr=%h valid=%b idle=%b fetchdone=%b",
                 dut.res_busy,dut.execution_fault,dut.topk_busy_r,dut.topk_done_latched,
                 dut.ve_busy,dut.lc_dec_instr,dut.lc_dec_valid,dut.exec_all_idle,dut.fetch_done);
        $fatal(1,"route combine timeout pc=%d count=%d",dut.u_fetch.pc,count); end
endmodule
