// Minimal 32x32-tile mixed-precision latency calibration.
// Measures controller/dequant timing only; weights are pre-packed constants.
// FP16->INTx quantization is compiler/offline work and is intentionally absent.
`timescale 1ns/1ps
`default_nettype none

module tb_tile_precision_latency;
  import isa_pkg::*;
  localparam int DIM = 32;
  logic clk = 1'b0, rst_n = 1'b0, cmd_valid = 1'b0;
  logic cmd_ready, done;
  logic [15:0] cfg_m = 16'd1, cfg_n = 16'd32, cfg_k = 16'd32, cmd_k = 16'd32;
  precision_t prec_mode;
  logic store_skip;
  logic [2:0] dq_nbits_code;
  logic dq_in_valid, dq_in_ready, dq_out_valid, dq_out_ready;
  logic [DIM*8-1:0] dq_in_data;
  logic [DIM*16-1:0] dq_out_data;
  logic wrf_rd_en, wrf_rd_bank_sel, arf_rd_en;
  logic [7:0] wrf_rd_addr, arf_rd_addr;
  logic [31:0] mock_wrf_data [0:15];
  logic [DIM*16-1:0] mock_arf_data;
  logic [31:0] sa_acc [0:DIM-1][0:DIM-1];
  integer cycle = 0;
  integer issue_cycle, first_dq_in, first_dq_out, last_dq_out, done_cycle;
  integer dq_accepts, dq_commits;
  integer fp16_gather_cycles, fp16_read_count;
  logic measuring;

  always #5 clk = ~clk;

  function automatic logic [15:0] weight_pattern(input logic [7:0] k,
                                                  input integer lane);
    weight_pattern = 16'h3c00 + (k * 32) + lane;
  endfunction

  function automatic logic [15:0] activation_pattern(input logic [7:0] k,
                                                      input integer row);
    activation_pattern = 16'h4000 + (k * 32) + row;
  endfunction

  always_comb begin
    for (int bank = 0; bank < 16; bank++) begin
      mock_wrf_data[bank] = {
        weight_pattern(wrf_rd_addr, bank*2 + 1),
        weight_pattern(wrf_rd_addr, bank*2)
      };
    end
    for (int row = 0; row < DIM; row++)
      mock_arf_data[row*16 +: 16] = activation_pattern(arf_rd_addr, row);
  end

  always @(posedge clk) begin
    cycle <= cycle + 1;
    if (measuring) begin
      if (cmd_valid && cmd_ready) issue_cycle = cycle;
      if (dq_in_valid && dq_in_ready) begin
        if (first_dq_in < 0) first_dq_in = cycle;
        dq_accepts = dq_accepts + 1;
      end
      if (dq_out_valid) begin
        if (first_dq_out < 0) first_dq_out = cycle;
        last_dq_out = cycle;
        dq_commits = dq_commits + 1;
      end
      if (done) done_cycle = cycle;
      if (prec_mode == PREC_FP16 && dut.state == 4'd3)
        fp16_gather_cycles = fp16_gather_cycles + 1;
      if (prec_mode == PREC_FP16 && wrf_rd_en) begin
        if (wrf_rd_addr != fp16_read_count)
          $fatal(1, "FP16 WRF read order mismatch: read=%0d addr=%0d",
                 fp16_read_count, wrf_rd_addr);
        fp16_read_count = fp16_read_count + 1;
      end
    end
  end

  // The compute array is irrelevant to controller/dequant latency.  Keep its
  // input at zero while providing every required controller input.
  integer rr, cc;
  initial for (rr = 0; rr < DIM; rr++) for (cc = 0; cc < DIM; cc++) sa_acc[rr][cc] = '0;

  gemm_ctrl #(.ARRAY_DIM(DIM), .FP16_W(16), .ACC_W(32)) dut (
    .clk(clk), .rst_n(rst_n), .cmd_valid(cmd_valid), .cmd_ready(cmd_ready),
    .cfg_M(cfg_m), .cfg_N(cfg_n), .cfg_K(cfg_k), .prec_mode(prec_mode),
    .trim_store_rows(1'b0),
    .dq_nbits_code(dq_nbits_code), .dequant_group_sel(),
    .acc_accumulate(1'b0), .cmd_K(cmd_k),
    .store_skip(store_skip), .nbits_override_valid(1'b0), .nbits_override('0),
    .wrf_rd_en(wrf_rd_en), .wrf_rd_bank_sel(wrf_rd_bank_sel),
    .wrf_rd_addr(wrf_rd_addr),
    .wrf_rd_data0(mock_wrf_data[0]), .wrf_rd_data1(mock_wrf_data[1]),
    .wrf_rd_data2(mock_wrf_data[2]), .wrf_rd_data3(mock_wrf_data[3]),
    .wrf_rd_data4(mock_wrf_data[4]), .wrf_rd_data5(mock_wrf_data[5]),
    .wrf_rd_data6(mock_wrf_data[6]), .wrf_rd_data7(mock_wrf_data[7]),
    .wrf_rd_data8(mock_wrf_data[8]), .wrf_rd_data9(mock_wrf_data[9]),
    .wrf_rd_data10(mock_wrf_data[10]), .wrf_rd_data11(mock_wrf_data[11]),
    .wrf_rd_data12(mock_wrf_data[12]), .wrf_rd_data13(mock_wrf_data[13]),
    .wrf_rd_data14(mock_wrf_data[14]), .wrf_rd_data15(mock_wrf_data[15]),
    .arf_rd_en(arf_rd_en), .arf_rd_addr(arf_rd_addr),
    .arf_rd_data(mock_arf_data), .arf_rd_data_valid(1'b1),
    .arf_batch_mode(1'b0),
    .dequant_in_valid(dq_in_valid), .dequant_in_ready(dq_in_ready),
    .dequant_params_ready(1'b1), .dequant_in_data(dq_in_data),
    .dequant_out_valid(dq_out_valid), .dequant_out_ready(dq_out_ready), .dequant_out_data(dq_out_data),
    .sa_clear(), .sa_en(), .sa_active_rows(), .sa_a_col(), .sa_b_row(),
    .acc_wr_en(), .acc_wr_addr(), .acc_rd_en(), .acc_rd_addr(), .acc_rd_data('0),
    .acc_wr_data0(),
    .acc_wr_data1(),
    .acc_wr_data2(),
    .acc_wr_data3(),
    .acc_wr_data4(),
    .acc_wr_data5(),
    .acc_wr_data6(),
    .acc_wr_data7(),
    .acc_wr_data8(),
    .acc_wr_data9(),
    .acc_wr_data10(),
    .acc_wr_data11(),
    .acc_wr_data12(),
    .acc_wr_data13(),
    .acc_wr_data14(),
    .acc_wr_data15(),
    .acc_wr_data16(),
    .acc_wr_data17(),
    .acc_wr_data18(),
    .acc_wr_data19(),
    .acc_wr_data20(),
    .acc_wr_data21(),
    .acc_wr_data22(),
    .acc_wr_data23(),
    .acc_wr_data24(),
    .acc_wr_data25(),
    .acc_wr_data26(),
    .acc_wr_data27(),
    .acc_wr_data28(),
    .acc_wr_data29(),
    .acc_wr_data30(),
    .acc_wr_data31(),
    .sa_acc(sa_acc), .busy(), .done(done)
  );

  dequant_unit #(.LANES(DIM), .FP16_W(16)) dequant (
    .clk(clk), .rst_n(rst_n), .nbits_code(dq_nbits_code),
    .in_valid(dq_in_valid), .in_ready(dq_in_ready), .in_data(dq_in_data),
    .scale_data({DIM{16'h3c00}}), .zero_data('0),
    .out_valid(dq_out_valid), .out_ready(1'b1), .out_data(dq_out_data)
  );

  task automatic run_case(input precision_t p, input bit skip,
                         input string label, input integer tile_k);
    begin
      issue_cycle = -1; first_dq_in = -1; first_dq_out = -1; last_dq_out = -1; done_cycle = -1;
      dq_accepts = 0; dq_commits = 0;
      fp16_gather_cycles = 0; fp16_read_count = 0;
      prec_mode = p; store_skip = skip;
      cfg_k = tile_k; cmd_k = tile_k; measuring = 1'b1;
      wait (cmd_ready);
      @(negedge clk); cmd_valid = 1'b1;
      @(negedge clk); cmd_valid = 1'b0;
      wait (done);
      // ``done`` is a one-cycle combinational state decode.  Capture it in
      // the waiter as well as the edge monitor to avoid simulator scheduling
      // races between those two processes.
      if (done_cycle < 0) done_cycle = cycle;
      @(negedge clk); measuring = 1'b0;
      if (p != PREC_FP16 &&
          (dq_accepts != tile_k || dq_commits != tile_k))
        $fatal(1, "%s dequant count accepts=%0d commits=%0d", label, dq_accepts, dq_commits);
      if (p == PREC_FP16 && (dq_accepts != 0 || dq_commits != 0))
        $fatal(1, "%s must bypass dequant", label);
      if (p == PREC_FP16) begin
        if (fp16_read_count != tile_k || fp16_gather_cycles != tile_k ||
            dut.r_gather_k != tile_k)
          $fatal(1, "%s FP16 gather reads/cycles/final_k=%0d/%0d/%0d expected %0d",
                 label, fp16_read_count, fp16_gather_cycles,
                 dut.r_gather_k, tile_k);
        for (int kk = 0; kk < tile_k; kk++) begin
          for (int lane = 0; lane < DIM; lane++) begin
            if (dut.wt_buf[kk][lane] !== weight_pattern(8'(kk), lane))
              $fatal(1, "%s FP16 weight gather mismatch k=%0d lane=%0d got=%h expected=%h",
                     label, kk, lane, dut.wt_buf[kk][lane],
                     weight_pattern(8'(kk), lane));
            if (dut.act_buf[kk][lane] !== activation_pattern(8'(kk), lane))
              $fatal(1, "%s FP16 activation gather mismatch k=%0d row=%0d got=%h expected=%h",
                     label, kk, lane, dut.act_buf[kk][lane],
                     activation_pattern(8'(kk), lane));
          end
        end
      end
      $display("TILE_LATENCY|precision=%0d|label=%0s|K=%0d|M=1|N=32|skip_store=%0d|issue=%0d|first_deq_in=%0d|first_deq_out=%0d|last_deq_out=%0d|done=%0d|gemm_cycles=%0d|deq_accepts=%0d|deq_commits=%0d",
               p, label, tile_k, skip, issue_cycle, first_dq_in, first_dq_out, last_dq_out,
               done_cycle, done_cycle - issue_cycle, dq_accepts, dq_commits);
      repeat (2) @(negedge clk);
    end
  endtask

  initial begin
    prec_mode = PREC_FP16; store_skip = 1'b1;
    repeat (4) @(negedge clk); rst_n = 1'b1;
    run_case(PREC_FP16, 1'b1, "FP16_K32_SKIP", 32);
    run_case(PREC_FP16, 1'b0, "FP16_K32_STORE", 32);
    run_case(PREC_INT1, 1'b1, "INT1_K32_SKIP", 32);
    run_case(PREC_INT1, 1'b0, "INT1_K32_STORE", 32);
    run_case(PREC_INT2, 1'b1, "INT2_K32_SKIP", 32);
    run_case(PREC_INT2, 1'b0, "INT2_K32_STORE", 32);
    run_case(PREC_INT3, 1'b1, "INT3_K32_SKIP", 32);
    run_case(PREC_INT3, 1'b0, "INT3_K32_STORE", 32);
    run_case(PREC_FP16, 1'b1, "FP16_K64_SKIP", 64);
    run_case(PREC_FP16, 1'b1, "FP16_K128_SKIP", 128);
    run_case(PREC_INT1, 1'b1, "INT1_K128_SKIP", 128);
    run_case(PREC_INT2, 1'b1, "INT2_K128_SKIP", 128);
    run_case(PREC_INT3, 1'b1, "INT3_K128_SKIP", 128);
    $display("TILE_LATENCY_DONE"); $finish;
  end
  initial begin #100000; $fatal(1, "tile latency timeout"); end
endmodule
`default_nettype wire
