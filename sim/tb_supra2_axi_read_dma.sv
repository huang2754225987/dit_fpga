`timescale 1ns/1ps
`default_nettype none

module tb_supra2_axi_read_dma;
    localparam logic [31:0] BASE_ADDRESS = 32'h0010_0fc0;
    localparam logic [31:0] TILE_ADDRESS = 32'h0010_2000;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic start = 1'b0;
    logic [31:0] descriptor_address = BASE_ADDRESS;
    logic [31:0] descriptor_beats = 4;
    logic busy, done, dma_error;
    logic [511:0] stream_data;
    logic stream_valid, stream_ready = 1'b0, stream_last;
    logic [3:0] m_axi_arid;
    logic [31:0] m_axi_araddr;
    logic [7:0] m_axi_arlen;
    logic [2:0] m_axi_arsize;
    logic [1:0] m_axi_arburst;
    logic m_axi_arvalid, m_axi_arready = 1'b1;
    logic [3:0] m_axi_rid = 4'h0;
    logic [511:0] m_axi_rdata;
    logic [1:0] m_axi_rresp = 2'b00;
    logic m_axi_rlast, m_axi_rvalid, m_axi_rready;

    logic model_active = 1'b0;
    logic [31:0] model_address = '0;
    logic [6:0] model_beats_left = '0;
    integer accepted_bursts = 0;
    integer received_beats = 0;
    integer cycles = 0;
    integer test_phase = 0;
    integer expected_last_index = 3;
    logic [31:0] expected_data_base = BASE_ADDRESS;
    logic was_stalled = 1'b0;
    logic [511:0] stalled_data;
    logic stalled_last;

    always #5 clk = ~clk;

    assign m_axi_rvalid = model_active;
    assign m_axi_rdata = {16{model_address}};
    assign m_axi_rlast = model_active && (model_beats_left == 1);

    supra2_axi_read_dma dut (.*);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            model_active <= 1'b0;
            model_address <= '0;
            model_beats_left <= '0;
            stream_ready <= 1'b0;
        end else begin
            stream_ready <= (cycles[1:0] != 2'b00);
            if (m_axi_arvalid && m_axi_arready) begin
                if (model_active) $fatal(1, "reader issued overlapping bursts");
                model_active <= 1'b1;
                model_address <= m_axi_araddr;
                model_beats_left <= {1'b0, m_axi_arlen} + 1'b1;
            end
            if (m_axi_rvalid && m_axi_rready) begin
                if (model_beats_left == 1) begin
                    model_active <= 1'b0;
                    model_beats_left <= '0;
                end else begin
                    model_address <= model_address + 32'd64;
                    model_beats_left <= model_beats_left - 1'b1;
                end
            end
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            accepted_bursts = 0;
            received_beats = 0;
            cycles = 0;
            was_stalled <= 1'b0;
        end else begin
            cycles = cycles + 1;
            if (cycles > 1000) $fatal(1, "AXI read DMA watchdog");
            if (was_stalled &&
                (!stream_valid || stream_data !== stalled_data ||
                 stream_last !== stalled_last))
                $fatal(1, "stream data/last changed while backpressured");
            was_stalled <= stream_valid && !stream_ready;
            stalled_data <= stream_data;
            stalled_last <= stream_last;
            if (m_axi_arvalid && m_axi_arready) begin
                if (m_axi_arsize != 3'd6 || m_axi_arburst != 2'b01)
                    $fatal(1, "incorrect 512-bit AXI transfer attributes");
                if ((int'(m_axi_araddr[11:0]) +
                     (int'(m_axi_arlen) + 1) * 64) > 4096)
                    $fatal(1, "AXI burst crossed 4-KiB boundary");
                if (test_phase == 0) begin
                    if ((accepted_bursts == 0) &&
                        (m_axi_araddr != BASE_ADDRESS || m_axi_arlen != 0))
                        $fatal(1, "first burst should be one beat at page end");
                    if ((accepted_bursts == 1) &&
                        (m_axi_araddr != BASE_ADDRESS + 32'd64 || m_axi_arlen != 2))
                        $fatal(1, "second burst should be three beats");
                    if (accepted_bursts > 1) $fatal(1, "unexpected extra boundary burst");
                end else if (test_phase == 1) begin
                    if (accepted_bursts != 0 || m_axi_araddr != TILE_ADDRESS ||
                        m_axi_arlen != 8'd15)
                        $fatal(1, "1-KiB tile should issue one 16-beat burst");
                end else if (test_phase == 2) begin
                    if (accepted_bursts != 0 || m_axi_araddr != TILE_ADDRESS ||
                        m_axi_arlen != 8'd63)
                        $fatal(1, "maximum legal AXI burst should contain 64 beats");
                end else if (test_phase == 3) begin
                    if (accepted_bursts != 0 || m_axi_araddr != 32'd0 ||
                        m_axi_arlen != 8'd63)
                        $fatal(1, "full-range descriptor should begin with 64 beats");
                end else begin
                    $fatal(1, "invalid descriptor unexpectedly reached AXI AR");
                end
                accepted_bursts = accepted_bursts + 1;
            end
            if (stream_valid && stream_ready) begin
                if (stream_data !== {16{expected_data_base + received_beats * 64}})
                    $fatal(1, "bad stream data at beat %0d", received_beats);
                if (stream_last !== (received_beats == expected_last_index))
                    $fatal(1, "bad stream_last at beat %0d", received_beats);
                received_beats = received_beats + 1;
            end
        end
    end

    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk) rst_n = 1'b1;
        @(negedge clk) start = 1'b1;
        @(negedge clk) start = 1'b0;

        wait (done);
        #1;
        if (busy || dma_error || accepted_bursts != 2 || received_beats != 4)
            $fatal(1, "valid descriptor failed busy=%b error=%b bursts=%0d beats=%0d",
                   busy, dma_error, accepted_bursts, received_beats);

        @(negedge clk);
        test_phase = 1;
        accepted_bursts = 0;
        received_beats = 0;
        expected_data_base = TILE_ADDRESS;
        expected_last_index = 15;
        descriptor_address = TILE_ADDRESS;
        descriptor_beats = 16;
        start = 1'b1;
        @(negedge clk) start = 1'b0;
        wait (done);
        #1;
        if (busy || dma_error || accepted_bursts != 1 || received_beats != 16)
            $fatal(1, "tile descriptor failed busy=%b error=%b bursts=%0d beats=%0d",
                   busy, dma_error, accepted_bursts, received_beats);

        @(negedge clk);
        test_phase = 2;
        accepted_bursts = 0;
        received_beats = 0;
        expected_data_base = TILE_ADDRESS;
        expected_last_index = 63;
        descriptor_address = TILE_ADDRESS;
        descriptor_beats = 64;
        start = 1'b1;
        @(negedge clk) start = 1'b0;
        wait (done);
        #1;
        if (busy || dma_error || accepted_bursts != 1 || received_beats != 64)
            $fatal(1, "64-beat descriptor failed busy=%b error=%b bursts=%0d beats=%0d",
                   busy, dma_error, accepted_bursts, received_beats);

        // The address space permits exactly 2^26 64-byte beats from address
        // zero. Check that the narrowed internal counter accepts that limit;
        // reset after the first burst instead of simulating the full 4 GiB.
        @(negedge clk);
        test_phase = 3;
        accepted_bursts = 0;
        descriptor_address = 32'd0;
        descriptor_beats = 32'h0400_0000;
        start = 1'b1;
        @(negedge clk) start = 1'b0;
        wait (m_axi_arvalid);
        #1;
        if (dma_error || m_axi_araddr != 32'd0 || m_axi_arlen != 8'd63)
            $fatal(1, "full-address-space descriptor was truncated or rejected");

        @(negedge clk) rst_n = 1'b0;
        repeat (3) @(negedge clk);
        rst_n = 1'b1;

        @(negedge clk);
        test_phase = 4;
        accepted_bursts = 0;
        descriptor_address = BASE_ADDRESS + 1;
        descriptor_beats = 1;
        start = 1'b1;
        @(negedge clk) start = 1'b0;
        wait (done);
        #1;
        if (!dma_error || busy || accepted_bursts != 0)
            $fatal(1, "unaligned descriptor was not rejected");

        $display("SUPRA2_AXI_READ_DMA_PASS boundary_split=1+3 tile_beats=16 max_burst=64 max_descriptor=2^26 backpressure=exercised");
        $finish;
    end
endmodule

`default_nettype wire
