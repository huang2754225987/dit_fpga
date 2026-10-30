`timescale 1ns/1ps
module tb_dma_topk_dependency;
  logic clk=0, rst_n=0, topk=0;
  logic [31:0] instruction=0;
  logic [7:0] busy=0;
  wire instr_ready, dma_rd_cmd_valid, dma_rd_cmd_out_token_group;
  wire [3:0] dma_rd_cmd_expert_sel;
  wire [7:0] res_acquire;
  integer checks=0;
  integer completed=0;
  always #5 clk=~clk;
  ucode_decode dut (
    .clk(clk),.rst_n(rst_n),.instr_valid(1'b1),.instr_data(instruction),
    .topk_done_sig(topk),.route_unit_busy(1'b0),.l1_clear_busy(1'b0),
    .scale_loader_busy(1'b0),.dma_rd_l1_bank_valid(1'b0),.dma_rd_l1_bank(4'd0),
    .res_busy(busy),.all_idle(busy==0),.wrf_data_valid(1'b1),.arf_data_valid(1'b1),
    .loop_active(1'b0),.loop_iter(16'd0),.loop_remaining(16'd1),
    .static_k128_replay(1'b0),
    .token_loop_iter(6'd0),.expert_loop_iter(6'd0),.addr_stride(16'd0),
    .cfg_M(16'd8),.cfg_K(16'd128),
    .instr_ready(instr_ready),
    .vld_cmd_valid(),
    .vld_cmd_vd(),
    .vld_cmd_addr(),
    .vld_cmd_page(),
    .vld_cmd_token_stride_mode(),
    .vld_cmd_token_stride_bytes(),
    .vst_cmd_valid(),
    .vst_cmd_vs1(),
    .vst_cmd_addr(),
    .vst_cmd_page(),
    .vst_cmd_token_stride_mode(),
    .vst_cmd_token_stride_bytes(),
    .gld_w_cmd_valid(),
    .gld_w_cmd_addr(),
    .gld_w_cmd_prec(),
    .gld_w_cmd_group(),
    .gld_w_cmd_expert_sel(),
    .gld_w_cmd_proj(),
    .gld_w_cmd_direct_nbits(),
    .gld_a_cmd_valid(),
    .gld_a_cmd_addr(),
    .gld_a_cmd_batch(),
    .gld_a_cmd_transpose(),
    .gld_a_cmd_page(),
    .gld_a_cmd_stride_lines(),
    .gemm_cmd_valid(),
    .gemm_cmd_acc(),
    .gemm_cmd_prec(),
    .gemm_cmd_k(),
    .gemm_cmd_skip_store(),
    .gemm_cmd_expert_sel(),
    .gemm_cmd_proj(),
    .gemm_cmd_direct_nbits(),
    .softmax_cmd_valid(),
    .softmax_cmd_vd(),
    .softmax_cmd_vs1(),
    .softmax_cmd_valid_lanes(),
    .rmsnorm_cmd_valid(),
    .rmsnorm_cmd_vd(),
    .rmsnorm_cmd_vs1(),
    .rmsnorm_cmd_vs2(),
    .rmsnorm_cmd_mode(),
    .rmsnorm_cmd_reset(),
    .rmsnorm_cmd_inv_count(),
    .rope_cmd_valid(),
    .rope_cmd_vd(),
    .rope_cmd_vs1(),
    .rope_cmd_vs2(),
    .silu_cmd_valid(),
    .silu_cmd_vd(),
    .silu_cmd_vs1(),
    .silu_cmd_mode(),
    .add_cmd_valid(),
    .add_cmd_vd(),
    .add_cmd_vs1(),
    .add_cmd_vs2(),
    .mul_cmd_valid(),
    .mul_cmd_vd(),
    .mul_cmd_vs1(),
    .mul_cmd_vs2(),
    .dma_rd_cmd_valid(dma_rd_cmd_valid),
    .dma_rd_cmd_addr(),
    .dma_rd_cmd_vd(),
    .dma_rd_cmd_token_stride(),
    .dma_rd_cmd_l1_token_stride(),
    .dma_rd_cmd_out_token_group(dma_rd_cmd_out_token_group),
    .dma_rd_cmd_static(),
    .dma_rd_cmd_group_gather(),
    .dma_rd_cmd_group_gather_out(),
    .dma_rd_cmd_out_token(),
    .dma_rd_cmd_len_code(),
    .dma_rd_cmd_k_override(),
    .dma_rd_cmd_transfer_k(),
    .dma_wr_cmd_valid(),
    .dma_wr_cmd_vd(),
    .dma_wr_cmd_addr(),
    .dma_wr_cmd_token_stride(),
    .dma_wr_cmd_page(),
    .dma_wr_cmd_len_code(),
    .topk_cmd_valid(),
    .topk_cmd_token_row(),
    .topk_cmd_acc_row(),
    .route_cmd_valid(),
    .route_cmd_token(),
    .route_group_cmd_valid(),
    .route_group_cmd_expert(),
    .route_group_cmd_tokens(),
    .route_group_cmd_start(),
    .rweight_cmd_valid(),
    .rweight_cmd_vd(),
    .rweight_cmd_slot(),
    .group_weight_cmd_valid(),
    .group_weight_cmd_vd(),
    .group_weight_cmd_row(),
    .bcast_cmd_valid(),
    .bcast_cmd_vd(),
    .bcast_cmd_vs1(),
    .bcast_cmd_lane(),
    .l1_accum_clear_cmd_valid(),
    .l1_accum_clear_words(),
    .token_loop_begin(),
    .token_loop_end(),
    .token_loop_count(),
    .expert_loop_begin(),
    .expert_loop_end(),
    .expert_loop_count(),
    .dma_rd_cmd_expert_sel(dma_rd_cmd_expert_sel),
    .dma_rd_cmd_proj(),
    .scale_cmd_valid(),
    .scale_cmd_ntile(),
    .scale_cmd_current_expert_only(),
    .static_scale_cmd_valid(),
    .static_scale_cmd_addr(),
    .static_scale_cmd_beats(),
    .static_wpage_cmd_valid(),
    .static_wpage_cmd_page(),
    .tgld_cmd_valid(),
    .tgld_cmd_base_line(),
    .tgld_cmd_row_stride(),
    .cfg_k_write_valid(),
    .cfg_k_write_value(),
    .cfg_m_write_valid(),
    .cfg_m_write_value(),
    .ac2v_cmd_valid(),
    .ac2v_cmd_vd(),
    .ac2v_cmd_row(),
    .ac2v_cmd_set(),
    .barrier_active(),
    .program_end(),
    .loop_cmd_valid(),
    .loop_body_len(),
    .loop_count_init(),
    .loop_stride_code(),
    .res_acquire(res_acquire)
  );
  task automatic check(input logic [3:0] selector, input logic [3:0] length_or_expert,
                       input bit expected);
    instruction={4'hc,4'h2,selector,length_or_expert,16'd0};
    #1;
    if (instr_ready !== expected || dma_rd_cmd_valid !== expected)
      $fatal(1,"DMA TopK dependency selector=%h code=%h topk=%b ready=%b expected=%b",
             selector,length_or_expert,topk,instr_ready,expected);
    if (expected && (selector==4 || selector==5 || selector==14) &&
        dma_rd_cmd_expert_sel != 0) $fatal(1,"Direct DMA incorrectly selects expert");
    checks++;
  endtask
  initial begin
    #12; rst_n=1;
    for (int code=0;code<16;code++) begin
      check(4,code,1); check(5,code,1); check(14,code,1);
    end
    check(0,1,0); check(12,1,0); check(13,2,0); check(15,2,0); check(15,13,0);
    topk=1;
    check(0,1,1); check(12,1,1); check(13,2,1); check(15,2,1); check(15,13,1);
    busy=1;
    check(4,7,0); check(14,15,0);
    $display("DMA_TOPK_DEPENDENCY_PASS checks=%0d",checks);
    completed=1; $finish;
  end
endmodule

