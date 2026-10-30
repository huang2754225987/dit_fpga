// Real array efficiency and numerical checks; preloaded RF, no DDR.
`timescale 1ns/1ps
`default_nettype none

module tb_gemm_array_efficiency;
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
  logic signed [31:0] array_acc [0:DIM-1][0:DIM-1];
  for (genvar row = 0; row < DIM; row++) begin
    for (genvar col = 0; col < DIM; col++) begin
      assign sa_acc[row][col] = array_acc[row][col];
    end
  end
  integer cycle = 0;
  integer issue_cycle, first_dq_in, first_dq_out, last_dq_out, done_cycle;
  integer dq_accepts, dq_commits;
  integer fp16_gather_cycles, fp16_read_count;
  logic measuring;
  logic sa_clear, sa_en;
  logic [5:0] sa_active_rows;
  logic signed [15:0] sa_a_col [0:DIM-1], sa_b_row [0:DIM-1];
  logic signed [7:0] zero_i8 [0:DIM-1];
  logic acc_valid, acc_wr_en;
  logic [4:0] acc_wr_addr;
  logic [31:0] acc_words [0:DIM-1];
  wire params_ready;
  logic inject_stalls = 1'b0;
  logic [DIM*16-1:0] scales;
  logic [255:0] packed_weights;
  integer array_enabled, array_first, array_last, store_cycles, gather_only, overlap_cycles;
  integer fd, test_count = 0;
  string csv_path;
  function automatic logic [31:0] positive_int_fp32(input integer value);
    integer p;
    logic [7:0] exponent;
    logic [22:0] mantissa;
    begin
      p = 0;
      for (int b = 0; b < 31; b++) if (value >> b) p = b;
      exponent = 127 + p;
      mantissa = (value - (1 << p)) << (23 - p);
      return {1'b0, exponent, mantissa};
    end
  endfunction
  assign params_ready = !inject_stalls || (cycle % 7 >= 2);

  always #5 clk = ~clk;

  always_comb begin
    packed_weights = '0;
    for (int lane = 0; lane < DIM; lane++) begin
      case (prec_mode)
        PREC_INT1: packed_weights[lane] = 1'b1;
        PREC_INT2: packed_weights[lane*2 +: 2] = 2'd1;
        PREC_INT3: packed_weights[lane*3 +: 3] = 3'd1;
        default: ;
      endcase
      scales[lane*16 +: 16] = lane % 2 ? 16'h4000 : 16'h3c00;
      zero_i8[lane] = '0;
    end
    for (int bank = 0; bank < 16; bank++)
      mock_wrf_data[bank] = prec_mode == PREC_FP16
        ? {16'h4000, 16'h3c00} : packed_weights[bank*32 +: 32];
    for (int row = 0; row < DIM; row++)
      mock_arf_data[row*16 +: 16] = row % 2
        ? (arf_rd_addr[0] ? 16'h4400 : 16'h4000)
        : (arf_rd_addr[0] ? 16'h4000 : 16'h3c00);
  end

  always @(posedge clk) begin
    cycle <= cycle + 1;
    if (measuring) begin
      if (cmd_valid && cmd_ready) issue_cycle = cycle;
      if (dq_in_valid && dq_in_ready) begin
        if (first_dq_in < 0) first_dq_in = cycle;
        dq_accepts = dq_accepts + 1;
      end
      if (dq_out_valid && dq_out_ready) begin
        if (first_dq_out < 0) first_dq_out = cycle;
        last_dq_out = cycle;
        dq_commits = dq_commits + 1;
      end
      if (done) done_cycle = cycle;
      if (dut.state == 4'd3) gather_only++;
      if (dut.state == 4'd4) overlap_cycles++;
      if (sa_en) begin
        array_enabled++;
        if (array_first < 0) array_first = cycle;
        array_last = cycle;
      end
      if (acc_wr_en) begin
        store_cycles++;
        if (acc_wr_addr >= cfg_m) $fatal(1, "store row outside live M");
        for (int col = 0; col < DIM; col++)
          if (acc_words[col] !== positive_int_fp32(
              (cmd_k + cmd_k/2) * (acc_wr_addr % 2 ? 2 : 1) * (col % 2 ? 2 : 1)))
            $fatal(1, "writeback mismatch M=%0d K=%0d row=%0d col=%0d got=%h",
                   cfg_m, cmd_k, acc_wr_addr, col, acc_words[col]);
      end
    end
  end

  systolic_array #(.M(DIM), .N(DIM), .DATA_W(8), .ACC_W(32)) array (
    .clk(clk), .rst_n(rst_n), .clear_acc(sa_clear), .en(sa_en),
    .a_unsigned(1'b0), .active_rows(sa_active_rows),
    .a_col(zero_i8), .b_row(zero_i8),
    .acc_out(array_acc), .acc_valid(acc_valid)
  );

  gemm_ctrl #(.ARRAY_DIM(DIM), .FP16_W(16), .ACC_W(32)) dut (
    .clk(clk), .rst_n(rst_n), .cmd_valid(cmd_valid), .cmd_ready(cmd_ready),
    .cfg_M(cfg_m), .cfg_N(cfg_n), .cfg_K(cfg_k), .prec_mode(prec_mode),
    .trim_store_rows(1'b1),
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
    .arf_batch_mode(1'b1),
    .dequant_in_valid(dq_in_valid), .dequant_in_ready(dq_in_ready),
    .dequant_params_ready(params_ready), .dequant_in_data(dq_in_data),
    .dequant_out_valid(dq_out_valid), .dequant_out_ready(dq_out_ready), .dequant_out_data(dq_out_data),
    .sa_clear(sa_clear), .sa_en(sa_en), .sa_active_rows(sa_active_rows), .sa_a_col(sa_a_col), .sa_b_row(sa_b_row),
    .acc_wr_en(acc_wr_en), .acc_wr_addr(acc_wr_addr), .acc_rd_en(), .acc_rd_addr(), .acc_rd_data('0),
    .acc_wr_data0(acc_words[0]),
    .acc_wr_data1(acc_words[1]),
    .acc_wr_data2(acc_words[2]),
    .acc_wr_data3(acc_words[3]),
    .acc_wr_data4(acc_words[4]),
    .acc_wr_data5(acc_words[5]),
    .acc_wr_data6(acc_words[6]),
    .acc_wr_data7(acc_words[7]),
    .acc_wr_data8(acc_words[8]),
    .acc_wr_data9(acc_words[9]),
    .acc_wr_data10(acc_words[10]),
    .acc_wr_data11(acc_words[11]),
    .acc_wr_data12(acc_words[12]),
    .acc_wr_data13(acc_words[13]),
    .acc_wr_data14(acc_words[14]),
    .acc_wr_data15(acc_words[15]),
    .acc_wr_data16(acc_words[16]),
    .acc_wr_data17(acc_words[17]),
    .acc_wr_data18(acc_words[18]),
    .acc_wr_data19(acc_words[19]),
    .acc_wr_data20(acc_words[20]),
    .acc_wr_data21(acc_words[21]),
    .acc_wr_data22(acc_words[22]),
    .acc_wr_data23(acc_words[23]),
    .acc_wr_data24(acc_words[24]),
    .acc_wr_data25(acc_words[25]),
    .acc_wr_data26(acc_words[26]),
    .acc_wr_data27(acc_words[27]),
    .acc_wr_data28(acc_words[28]),
    .acc_wr_data29(acc_words[29]),
    .acc_wr_data30(acc_words[30]),
    .acc_wr_data31(acc_words[31]),
    .sa_acc(sa_acc), .busy(), .done(done)
  );

  dequant_unit #(.LANES(DIM), .FP16_W(16)) dequant (
    .clk(clk), .rst_n(rst_n), .nbits_code(dq_nbits_code),
    .in_valid(dq_in_valid), .in_ready(dq_in_ready), .in_data(dq_in_data),
    .scale_data(scales), .zero_data('0),
    .out_valid(dq_out_valid), .out_ready(dq_out_ready), .out_data(dq_out_data)
  );

  task automatic run_case(input precision_t p, input integer tile_m,
                          input integer tile_k, input bit stalls);
    integer span, expected_cycles, expected_value;
    real efficiency;
    begin
      wait(cmd_ready);
      @(negedge clk);
      issue_cycle = -1; first_dq_in = -1; first_dq_out = -1;
      last_dq_out = -1; done_cycle = -1;
      dq_accepts = 0; dq_commits = 0;
      array_enabled = 0; array_first = -1; array_last = -1;
      store_cycles = 0; gather_only = 0; overlap_cycles = 0;
      prec_mode = p; store_skip = 1'b0; inject_stalls = stalls;
      cfg_m = tile_m; cfg_k = tile_k; cmd_k = tile_k;
      measuring = 1'b1; cmd_valid = 1'b1;
      @(negedge clk); cmd_valid = 1'b0;
      wait(done);
      // Capture at DONE entry; its combinational decode precedes the next
      // monitor edge, so the edge-only timestamp could remain unset.
      done_cycle = cycle;
      @(negedge clk); measuring = 1'b0;
      if (done_cycle <= issue_cycle) $fatal(1, "invalid controller timestamps");
      if (p != PREC_FP16 && (dq_accepts != tile_k || dq_commits != tile_k))
        $fatal(1, "dequant count mismatch");
      if (p == PREC_FP16 && (dq_accepts != 0 || dq_commits != 0))
        $fatal(1, "FP16 must bypass dequant");
      if (store_cycles != tile_m) $fatal(1, "store count mismatch");
      expected_cycles = tile_k + tile_m + DIM - 1;
      if (array_enabled != expected_cycles)
        $fatal(1, "array enabled cycles %0d expected %0d", array_enabled, expected_cycles);
      span = array_last - array_first + 1;
      if (!stalls && span != array_enabled) $fatal(1, "unexpected array feed bubble");
      for (int row = 0; row < tile_m; row++)
        for (int col = 0; col < DIM; col++) begin
          expected_value = (tile_k + tile_k/2) * (row % 2 ? 2 : 1) * (col % 2 ? 2 : 1);
          if (sa_acc[row][col] !== positive_int_fp32(expected_value))
            $fatal(1, "array mismatch M=%0d K=%0d row=%0d col=%0d got=%h expected=%h",
                   tile_m, tile_k, row, col, sa_acc[row][col], positive_int_fp32(expected_value));
        end
      efficiency = real'(tile_m * DIM * tile_k) / (1024.0 * span);
      $display("GEMM_EFF|prec=%0d|M=%0d|K=%0d|stalls=%0d|controller=%0d|array_en=%0d|array_span=%0d|gather_only=%0d|overlap=%0d|store=%0d|dq=%0d|eff=%0.6f|PASS",
               p, tile_m, tile_k, stalls, done_cycle - issue_cycle, array_enabled, span,
               gather_only, overlap_cycles, store_cycles, dq_commits, efficiency);
      $fdisplay(fd, "%0d,%0d,32,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0.9f",
                p, tile_m, tile_k, stalls, done_cycle - issue_cycle, array_enabled,
                span, gather_only, overlap_cycles, store_cycles, dq_commits, efficiency);
      test_count++;
      repeat (2) @(negedge clk);
    end
  endtask

  initial begin
    prec_mode = PREC_FP16; store_skip = 1'b0; measuring = 1'b0;
    if (!$value$plusargs("CSV=%s", csv_path)) csv_path = "gemm_array_efficiency.csv";
    fd = $fopen(csv_path, "w");
    if (!fd) $fatal(1, "cannot open CSV");
    $fdisplay(fd, "precision_code,M,N,K,injected_stalls,controller_cycles,array_enabled_cycles,array_span_cycles,gather_only_cycles,overlap_cycles,store_cycles,dequant_commits,pure_array_efficiency");
    repeat (4) @(negedge clk); rst_n = 1'b1;
    for (int mi = 0; mi < 6; mi++)
      for (int ki = 0; ki < 3; ki++)
        for (int pi = 0; pi < 4; pi++)
          run_case(precision_t'(pi), 1 << mi, 64 << ki, 1'b0);
    run_case(PREC_INT3, 8, 128, 1'b1);
    $fclose(fd);
    $display("GEMM_EFF_DONE cases=%0d", test_count);
    $finish;
  end
  initial begin #1000000; $fatal(1, "array efficiency timeout"); end
endmodule
`default_nettype wire

