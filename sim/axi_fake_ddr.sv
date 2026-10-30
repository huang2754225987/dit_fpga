// Simulation-only bounded AXI DDR. TB refills one burst from a backing image.
// Absolute-address dirty overlays preserve writes across refills/staged launches.
// No modulo address aliasing, no full-weight-image allocation, no MIG claims.
`timescale 1ns/1ps
module axi_fake_ddr #(
    parameter int READ_LATENCY = 10,
    parameter int BEAT_INTERVAL = 2,
    parameter int MAX_DIRTY_BEATS = 262144
)(
    input logic clk, rst_n,
    input logic [3:0] arid, input logic [31:0] araddr,
    input logic [7:0] arlen, input logic [2:0] arsize,
    input logic [1:0] arburst, input logic arvalid, output logic arready,
    output logic [3:0] rid, output logic [127:0] rdata,
    output logic [1:0] rresp, output logic rlast, rvalid, input logic rready,
    input logic [3:0] awid, input logic [31:0] awaddr,
    input logic [7:0] awlen, input logic [2:0] awsize,
    input logic [1:0] awburst, input logic awvalid, output logic awready,
    input logic [127:0] wdata, input logic [15:0] wstrb,
    input logic wlast, wvalid, output logic wready,
    output logic [3:0] bid, output logic [1:0] bresp,
    output logic bvalid, input logic bready,
    input logic [17:0] debug_pc, input logic [31:0] debug_instr,
    input logic [17:0] debug_writer_pc,
    input logic [31:0] debug_writer_instr,
    input integer debug_stage, input longint unsigned debug_cycle,
    output logic refill_request, output logic [31:0] refill_address,
    output integer refill_bytes, input logic refill_complete
);
    // Two bounded 4 KiB read banks, stored and validated a 128-bit beat at a
    // time. A refill targets one bank while the previous bank remains intact.
    logic [127:0] read_cache [0:1][0:255];
    logic [15:0] cache_written [0:1][0:255];
    logic [31:0] cache_page_base [0:1];
    bit cache_page_valid [0:1];
    integer read_bank, refill_bank, next_refill_bank;
    logic [127:0] dirty_data [int unsigned];
    logic [15:0] dirty_mask [int unsigned];
    integer dirty_count = 0, refill_count = 0, cache_hit_count = 0;
    longint unsigned read_bytes = 0, write_bytes = 0;
    logic reading, writing;
    integer read_index, read_beats, read_delay;
    integer write_index, write_beats;
    logic [31:0] write_base;
    task automatic fill_byte(input integer offset, input byte unsigned value);
        integer byte_address, beat_index, lane_index;
        if (!refill_request || offset < 0 || offset >= refill_bytes)
            $fatal(1, "FakeDDR illegal TB refill offset");
        byte_address = int'(refill_address[11:0]) + offset;
        beat_index = byte_address >> 4;
        lane_index = byte_address & 15;
        read_cache[refill_bank][beat_index][lane_index*8 +: 8] = value;
        cache_written[refill_bank][beat_index][lane_index] = 1'b1;
    endtask
    task automatic fill_block(ref byte unsigned data [0:4095],
                              input integer count);
        integer first_beat;
        logic [127:0] beat_data;
        if (!refill_request || count < 1 || count > refill_bytes ||
            count > 4096 || count % 16 != 0 || refill_address[3:0] != 0)
            $fatal(1, "FakeDDR illegal TB refill block size=%0d", count);
        first_beat = int'(refill_address[11:4]);
        for (int beat = 0; beat < count/16; beat++) begin
            for (int lane = 0; lane < 16; lane++)
                beat_data[lane*8 +: 8] = data[beat*16+lane];
            read_cache[refill_bank][first_beat+beat] = beat_data;
            cache_written[refill_bank][first_beat+beat] = 16'hffff;
        end
    endtask
    task automatic dump_writes(input string filename);
        integer fd;
        fd = $fopen(filename, "w");
        if (!fd) $fatal(1, "FakeDDR cannot open output dump");
        foreach (dirty_data[key])
            $fdisplay(fd, "%08h %032h %04h", key*16, dirty_data[key], dirty_mask[key]);
        $fclose(fd);
    endtask
    task automatic load_writes(input string filename);
        integer fd, scan_rc;
        int unsigned address, key;
        logic [127:0] data;
        logic [15:0] mask;
        fd = $fopen(filename, "r");
        if (!fd) $fatal(1, "FakeDDR cannot open checkpoint %s", filename);
        while (!$feof(fd)) begin
            scan_rc = $fscanf(fd, "%h %h %h\n", address, data, mask);
            if (scan_rc == 3 && mask != 0) begin
                key = address >> 4;
                if (!dirty_mask.exists(key)) begin
                    if (dirty_count >= MAX_DIRTY_BEATS)
                        $fatal(1, "FakeDDR checkpoint exceeds dirty capacity");
                    dirty_count++;
                end
                dirty_data[key] = data;
                dirty_mask[key] = mask;
            end else if (scan_rc != -1 && !$feof(fd)) begin
                $fatal(1, "FakeDDR malformed checkpoint %s", filename);
            end
        end
        $fclose(fd);
        $display("FAKE_DDR_CHECKPOINT_LOAD file=%s dirty_beats=%0d",
                 filename, dirty_count);
    endtask
    function automatic logic [7:0] read_dirty_byte(input logic [31:0] address);
        int unsigned key;
        int lane;
        key = address >> 4;
        lane = address[3:0];
        if (!dirty_mask.exists(key) || !dirty_mask[key][lane])
            return 8'hxx;
        return dirty_data[key][lane*8 +: 8];
    endfunction
    function automatic logic [127:0] read_word(input integer index);
        logic [127:0] value;
        int unsigned key;
        key = (refill_address >> 4) + index;
        value = read_cache[read_bank][int'(refill_address[11:4])+index];
        if (dirty_mask.exists(key))
            for (int b = 0; b < 16; b++)
                if (dirty_mask[key][b]) value[b*8 +: 8] = dirty_data[key][b*8 +: 8];
        return value;
    endfunction
    assign arready = rst_n && !reading;
    assign awready = rst_n && !writing && !bvalid;
    assign wready = rst_n && writing;
    assign rresp = 2'b00;
    assign bresp = 2'b00;
    initial if (READ_LATENCY < 0 || BEAT_INTERVAL < 1 || MAX_DIRTY_BEATS < 1)
        $fatal(1, "FakeDDR invalid timing/capacity");

    always @(posedge clk) begin : service
        int unsigned key;
        if (!rst_n) begin
            reading <= 0; writing <= 0; rvalid <= 0; bvalid <= 0;
            rlast <= 0; rid <= 0; rdata <= 0; bid <= 0;
            refill_request <= 0; refill_address <= 0; refill_bytes <= 0;
            read_index <= 0; read_beats <= 0; read_delay <= 0;
            read_bank <= 0; refill_bank <= 0; next_refill_bank <= 0;
            for (int bank = 0; bank < 2; bank++) begin
                cache_page_base[bank] <= 0;
                cache_page_valid[bank] <= 0;
                for (int beat = 0; beat < 256; beat++) cache_written[bank][beat] = '0;
            end
            write_index <= 0; write_beats <= 0; write_base <= 0;
            dirty_data.delete(); dirty_mask.delete(); dirty_count = 0;
            refill_count = 0; cache_hit_count = 0;
            read_bytes = 0; write_bytes = 0;
        end else begin
            if (arvalid && arready) begin : accept_read
                integer hit_bank, target_bank;
                bit cache_hit, matching_page;
                if (arsize != 4 || arburst != 1 || araddr[3:0] != 0 ||
                    int'(araddr[11:0]) + (int'(arlen)+1)*16 > 4096)
                    $fatal(1, "FakeDDR unsupported/4KiB-crossing read burst addr=%h len=%0d", araddr, arlen);
                hit_bank = 0;
                cache_hit = 0;
                matching_page = 0;
                target_bank = next_refill_bank;
                for (int bank = 0; bank < 2; bank++) begin
                    if (cache_page_valid[bank] &&
                        cache_page_base[bank] == {araddr[31:12],12'b0}) begin
                        matching_page = 1;
                        hit_bank = bank;
                        cache_hit = 1;
                        for (int beat = 0; beat < int'(arlen)+1; beat++)
                            if (cache_written[bank][int'(araddr[11:4])+beat] != 16'hffff)
                                cache_hit = 0;
                    end
                end
                if (cache_hit) begin
                    cache_hit_count++;
                    read_bank <= hit_bank;
                    // Keep the most recently used page resident. A later
                    // miss refills the other bank, so the TB can stage the
                    // incoming burst without evicting the page just read.
                    next_refill_bank <= 1-hit_bank;
                    refill_request <= 0;
                end else begin
                    if (matching_page) target_bank = hit_bank;
                    else begin
                        cache_page_valid[target_bank] <= 0;
                        cache_page_base[target_bank] <= {araddr[31:12],12'b0};
                        for (int beat = 0; beat < 256; beat++)
                            cache_written[target_bank][beat] = '0;
                    end
                    next_refill_bank <= 1-target_bank;
                    refill_bank <= target_bank;
                    read_bank <= target_bank;
                    refill_request <= 1;
                end
                reading <= 1; refill_address <= araddr;
                refill_bytes <= (int'(arlen)+1)*16;
                read_index <= 0; read_beats <= int'(arlen)+1; rid <= arid;
                if (cache_hit) read_delay <= READ_LATENCY;
            end
            if (refill_request && refill_complete) begin
                for (int beat = 0; beat < refill_bytes/16; beat++)
                    if (cache_written[refill_bank][int'(refill_address[11:4])+beat] != 16'hffff)
                        $fatal(1, "FakeDDR missing refill beat %0d", beat);
                cache_page_base[refill_bank] <= {refill_address[31:12],12'b0};
                cache_page_valid[refill_bank] <= 1;
                refill_request <= 0; read_delay <= READ_LATENCY;
                refill_count++;
            end
            if (reading && !refill_request && !rvalid) begin
                if (read_delay > 0) read_delay <= read_delay - 1;
                else begin
                    rdata <= read_word(read_index);
                    rlast <= read_index == read_beats-1; rvalid <= 1;
                end
            end
            if (rvalid && rready) begin
                read_bytes += 16;
                if (rlast) begin reading <= 0; rvalid <= 0; rlast <= 0; end
                else begin
                    read_index <= read_index+1;
                    if (BEAT_INTERVAL == 1) begin
                        rdata <= read_word(read_index+1);
                        rlast <= read_index+1 == read_beats-1;
                    end else begin
                        rvalid <= 0; read_delay <= BEAT_INTERVAL-2;
                    end
                end
            end
            if (bvalid && bready) bvalid <= 0;
            if (awvalid && awready) begin
                if (awsize != 4 || awburst != 1 || awaddr[3:0] != 0 ||
                    int'(awaddr[11:0]) + (int'(awlen)+1)*16 > 4096)
                    $fatal(1, "FakeDDR unsupported write burst");
                writing <= 1; write_base <= awaddr; write_index <= 0;
                write_beats <= int'(awlen)+1; bid <= awid;
            end
            if (wvalid && wready) begin
                if (wlast !== (write_index == write_beats-1)) $fatal(1, "FakeDDR WLAST mismatch");
                key = (write_base >> 4) + write_index;
                if (wstrb != 0 && !dirty_mask.exists(key)) begin
                    if (dirty_count >= MAX_DIRTY_BEATS) $fatal(1, "FakeDDR dirty capacity exceeded");
                    dirty_mask[key] = 0; dirty_data[key] = 0; dirty_count++;
                end
                for (int b = 0; b < 16; b++) if (wstrb[b]) begin
                    if ($isunknown(wdata[b*8 +: 8])) begin
                        $display("FAKE_DDR_UNKNOWN_WRITE stage=%0d cycle=%0d pc=%0d instr=%08h last_dma_wr_pc=%0d last_dma_wr_instr=%08h addr=%08h beat=%0d lane=%0d strb=%04h data=%032h",
                                 debug_stage, debug_cycle, debug_pc, debug_instr,
                                 debug_writer_pc, debug_writer_instr,
                                 write_base+write_index*16, write_index, b,
                                 wstrb, wdata);
                        $fatal(1, "FakeDDR unknown write byte addr=%h lane=%0d", write_base+write_index*16, b);
                    end
                    dirty_data[key][b*8 +: 8] = wdata[b*8 +: 8];
                    dirty_mask[key][b] = 1; write_bytes++;
                end
                if (wlast) begin writing <= 0; bvalid <= 1; end
                else write_index <= write_index+1;
            end
        end
    end
endmodule
