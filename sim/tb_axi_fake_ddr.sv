`timescale 1ns/1ps
module tb_axi_fake_ddr;
    logic clk=0, rst_n=0;
    logic [31:0] araddr=0, awaddr=0;
    logic [7:0] arlen=1, awlen=0;
    logic arvalid=0, rready=0, awvalid=0, wvalid=0, bready=0;
    logic [127:0] wdata=0;
    logic [15:0] wstrb=0;
    wire arready, awready, wready, rvalid, rlast, bvalid;
    wire [127:0] rdata;
    wire [3:0] rid,bid;
    wire [1:0] rresp,bresp;
    wire refill_request;
    wire [31:0] refill_address;
    integer refill_bytes;
    byte unsigned refill_data [0:4095];
    logic refill_complete=0;
    integer completed=0;
    bit patch_written=0;
    integer tests=0;
    always #5 clk=~clk;
    axi_fake_ddr #(.READ_LATENCY(2),.BEAT_INTERVAL(2),.MAX_DIRTY_BEATS(4)) dut (
      .clk(clk),.rst_n(rst_n),.arid(4'd6),.araddr(araddr),.arlen(arlen),
      .arsize(3'd4),.arburst(2'd1),.arvalid(arvalid),.arready(arready),
      .rid(rid),.rdata(rdata),.rresp(rresp),.rlast(rlast),.rvalid(rvalid),.rready(rready),
      .awid(4'd9),.awaddr(awaddr),.awlen(awlen),.awsize(3'd4),.awburst(2'd1),
      .awvalid(awvalid),.awready(awready),.wdata(wdata),.wstrb(wstrb),
      .wlast(1'b1),.wvalid(wvalid),.wready(wready),.bid(bid),.bresp(bresp),
      .bvalid(bvalid),.bready(bready),.refill_request(refill_request),
      .refill_address(refill_address),.refill_bytes(refill_bytes),.refill_complete(refill_complete),
      .debug_pc(18'b0),.debug_instr(32'b0),
      .debug_writer_pc(18'b0),.debug_writer_instr(32'b0),
      .debug_stage(0),.debug_cycle(64'b0)
    );
    function automatic byte unsigned backing_byte(input logic [31:0] address);
      return (address ^ (address >> 12) ^ (address >> 20)) & 255;
    endfunction
    always @(negedge clk) begin
      refill_complete=0;
      if (rst_n && refill_request) begin
        if (refill_address == 'h101000 &&
            (!dut.cache_page_valid[0] || dut.cache_page_base[0] != 'h1000 ||
             !dut.cache_written[0][0]))
          $fatal(1,"FakeDDR overwrote the previous cache bank during refill");
        if (refill_address == 'h201000 &&
            (!dut.cache_page_valid[0] || dut.cache_page_base[0] != 'h1000 ||
             !dut.cache_written[0][0]))
          $fatal(1,"FakeDDR evicted the most recently used page during refill");
        for (int b=0;b<refill_bytes;b++)
          refill_data[b]=backing_byte(refill_address+b);
        dut.fill_block(refill_data,refill_bytes);
        refill_complete=1;
      end
    end
    task automatic check_read(input logic [31:0] address);
      logic [127:0] hold_data, expected;
      bit hold_last;
      @(negedge clk); araddr=address; arvalid=1; rready=0;
      @(posedge clk); while (!arready) @(posedge clk);
      @(negedge clk); arvalid=0;
      wait(rvalid); #1; hold_data=rdata; hold_last=rlast;
      repeat(5) begin
        @(posedge clk); #1;
        if (!rvalid || rdata !== hold_data || rlast !== hold_last || arready)
          $fatal(1,"FakeDDR read backpressure stability failure");
      end
      @(negedge clk); rready=1;
      for (int beat=0;beat<2;beat++) begin
        @(posedge clk); while (!rvalid) @(posedge clk);
        for (int b=0;b<16;b++) begin
          expected[b*8 +: 8]=backing_byte(address+beat*16+b);
          if (patch_written && address+beat*16=='h1000 && b<2)
            expected[b*8 +: 8]=(b==0 ? 8'haa : 8'hbb);
        end
        if (rdata !== expected || rid!=6 || rresp!=0 || rlast !== (beat==1))
          $fatal(1,"FakeDDR refill/write overlay mismatch addr=%h beat=%0d got=%h expected=%h",
                 address,beat,rdata,expected);
      end
      @(negedge clk); rready=0; tests++;
    endtask
    task automatic write_patch;
      logic [3:0] saved_id;
      @(negedge clk); awaddr='h1000; awvalid=1;
      @(posedge clk); while (!awready) @(posedge clk);
      @(negedge clk); awvalid=0; wdata=128'hbbaa; wstrb=16'h0003; wvalid=1;
      @(posedge clk); while (!wready) @(posedge clk);
      @(negedge clk); wvalid=0;
      wait(bvalid); saved_id=bid;
      repeat(4) begin
        @(posedge clk); #1;
        if (!bvalid || bid!=saved_id || saved_id!=9 || bresp!=0)
          $fatal(1,"FakeDDR B response stability failure");
      end
      @(negedge clk); bready=1;
      @(posedge clk); @(negedge clk); bready=0; patch_written=1;
    endtask
    initial begin
      repeat(3) @(negedge clk); rst_n=1;
      check_read('h1000); check_read('h101000); check_read('h1000);
      write_patch(); check_read('h1000); check_read('h101000); check_read('h1000);
      check_read('h1080); check_read('h101080); check_read('h1000);
      check_read('h201000); check_read('h1000);
      if (dut.dirty_count!=1 || dut.refill_count!=5 ||
          dut.cache_hit_count!=6 || dut.write_bytes!=2)
        $fatal(1,"FakeDDR accounting failure");
      $display("FAKE_DDR_UNIT_PASS reads=%0d refills=%0d dirty=%0d",tests,dut.refill_count,dut.dirty_count);
      completed=1; $finish;
    end
    initial begin #100000; $fatal(1,"FakeDDR unit timeout"); end
endmodule
