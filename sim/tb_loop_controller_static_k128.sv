`timescale 1ns/1ps
`default_nettype none

module tb_loop_controller_static_k128;
  logic clk = 1'b0;
  logic rst_n = 1'b0;
  logic fetch_valid = 1'b0;
  logic [31:0] fetch_instr = '0;
  logic fetch_ready;
  logic dec_valid;
  logic [31:0] dec_instr;
  logic dec_ready = 1'b1;
  logic route_group_empty = 1'b0;
  logic [5:0] route_group_count = 6'd0;
  wire source_skip_valid;
  wire [15:0] source_skip_count;
  wire static_k128_replay;
  logic [15:0] cfg_K = 16'd128;
  logic recording = 1'b0;
  logic [31:0] replayed [0:7];
  integer replay_count = 0;
  logic expect_static_replay = 1'b0;

  always #5 clk = ~clk;

  loop_controller dut (
    .clk(clk), .rst_n(rst_n),
    .fetch_valid(fetch_valid), .fetch_instr(fetch_instr),
    .fetch_ready(fetch_ready),
    .dec_valid(dec_valid), .dec_instr(dec_instr), .dec_ready(dec_ready),
    .route_group_empty(route_group_empty),
    .route_group_count(route_group_count),
    .source_skip_valid(source_skip_valid),
    .source_skip_count(source_skip_count), .cfg_K(cfg_K),
    .static_k128_replay(static_k128_replay)
  );

  always @(posedge clk) begin
    if (recording && dec_valid && dec_ready) begin
      if (expect_static_replay && !static_k128_replay)
        $fatal(1, "static K=128 replay qualifier was not asserted");
      if (replay_count >= 8)
        $fatal(1, "unexpected extra cached-loop replay instruction");
      replayed[replay_count] = dec_instr;
      replay_count = replay_count + 1;
    end
  end

  task automatic send_instruction(input logic [31:0] instruction);
    begin
      @(negedge clk);
      fetch_instr = instruction;
      fetch_valid = 1'b1;
      @(posedge clk);
      @(negedge clk);
      fetch_valid = 1'b0;
    end
  endtask

  task automatic run_stride_case(input logic [3:0] length_code,
                                 input integer expected_stride);
    logic [31:0] marker;
    logic [31:0] dma;
    begin
      replay_count = 0;
      recording = 1'b0;
      marker = {4'h0, 4'hF, 4'h4, 4'hF, 16'd3};
      dma = {4'd12, 4'd2, 4'd4, length_code, 16'h0100};
      send_instruction(marker);
      send_instruction(dma);
      send_instruction({4'd4, 4'd0, 4'd0, 4'd0, 16'h4000});
      send_instruction({4'd3, 4'd1, 4'd8, 4'd3, 16'h2000});
      send_instruction({4'd5, 4'd0, 4'hC, 4'd3, 16'd128});
      recording = 1'b1;
      expect_static_replay = 1'b1;
      wait (replay_count == 8);
      @(negedge clk);
      recording = 1'b0;
      expect_static_replay = 1'b0;

      if (replayed[0][15:0] !== 16'h0100 + expected_stride ||
          replayed[4][15:0] !== 16'h0100 + 2 * expected_stride)
        $fatal(1, "DMA stride mismatch code=%0d got=%h/%h expected=%h/%h",
               length_code, replayed[0][15:0], replayed[4][15:0],
               16'h0100 + expected_stride, 16'h0100 + 2 * expected_stride);
      if (replayed[0][27:24] !== 4'd3 || replayed[4][27:24] !== 4'd2)
        $fatal(1, "DMA ping-pong bank mismatch for length code %0d",
               length_code);
      if (replayed[1][15:0] !== 16'h4100 ||
          replayed[5][15:0] !== 16'h4200)
        $fatal(1, "activation K=128 stride mismatch for length code %0d",
               length_code);
      if (replayed[2][15:0] !== 16'h3000 ||
          replayed[6][15:0] !== 16'h2000 ||
          replayed[2][22:20] !== 3'd1 ||
          replayed[6][22:20] !== 3'd2)
        $fatal(1, "weight-bank/scale replay mismatch for length code %0d",
               length_code);
      if (!replayed[3][26] || !replayed[3][27] ||
          !replayed[7][26] || replayed[7][27])
        $fatal(1, "GEMM accumulation/final-store replay mismatch for length code %0d",
               length_code);
      $display("PASS STATIC_K128_REPLAY|dma_code=%0d|stride16=%0d",
               length_code, expected_stride);
    end
  endtask

  task automatic check_tgld_stride512_passthrough;
    logic [31:0] tgld;
    begin
      // transpose_load(0, 512, transpose=0): vd=F is the encoded 512-byte
      // stride, while vs1=1 selects the non-transposed PV layout.
      tgld = {4'h0, 4'hF, 4'h1, 4'h0, 16'hA000};
      replay_count = 0;
      expect_static_replay = 1'b0;
      recording = 1'b1;
      send_instruction(tgld);
      recording = 1'b0;
      if (replay_count != 1 || replayed[0] !== tgld || dut.state !== 2'd0)
        $fatal(1, "512-byte TGLD was not passed through as one instruction");
      $display("PASS TGLD_STRIDE512_LOOP_MARKER_DISAMBIGUATION");
    end
  endtask

  initial begin
    repeat (4) @(negedge clk);
    rst_n = 1'b1;
    run_stride_case(4'd3, 32);
    run_stride_case(4'd4, 64);
    run_stride_case(4'd5, 128);
    run_stride_case(4'd7, 256);
    check_tgld_stride512_passthrough();
    $display("PASS all static K=128 compact/padded loop cases");
    $finish;
  end

  initial begin
    #100000;
    $fatal(1, "static K=128 loop-controller timeout");
  end
endmodule

`default_nettype wire
