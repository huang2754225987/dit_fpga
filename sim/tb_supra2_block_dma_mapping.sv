`timescale 1ns/1ps
`default_nettype none

// Compile the real generated block13_ddr_layout package, supra2_block_dma.sv,
// and this file; select tb_supra2_block_dma_mapping as the simulation top.
// No fixtures, hierarchical accesses, force, or reduced package lengths.
module tb_supra2_block_dma_mapping;
    logic clk = 1'b0;
    wire passed128, passed512;
    always #5 clk = ~clk;

    supra2_block_dma_mapping_case #(.AXI_DATA_W(128)) case128 (
        .clk(clk), .passed(passed128)
    );
    supra2_block_dma_mapping_case #(.AXI_DATA_W(512)) case512 (
        .clk(clk), .passed(passed512)
    );

    initial begin
        wait (passed128 && passed512);
        $display("SUPRA2_BLOCK_DMA_MAPPING_PASS widths=128,512 tiles_per_width=972 columns_per_width=31104 latency=1");
        $finish;
    end
endmodule

module supra2_block_dma_mapping_case #(
    parameter integer AXI_DATA_W = 128
)(
    input wire clk,
    output logic passed = 1'b0
);
    import block13_ddr_layout::*;

    localparam integer DATA_BYTES = AXI_DATA_W/8;
    localparam integer STAGE_N = 1728;
    localparam integer K_TILES = 576/32;
    localparam integer TILE_COUNT = (STAGE_N/32)*K_TILES;
    localparam integer WEIGHT_COLUMNS = TILE_COUNT*32;
    localparam integer OUTPUT_ELEMENTS = 256*576;
    localparam integer OUTPUT_BYTES = OUTPUT_ELEMENTS*2;
    localparam integer OUTPUT_BEATS = OUTPUT_BYTES/DATA_BYTES;
    localparam integer MAX_CYCLES = 1000000;
    // Both widths are aligned, but start one 512-bit beat before a page end.
    localparam logic [31:0] DDR_BASE = 32'h0010_0fc0;
    localparam logic [31:0] OUTPUT_BASE = 32'h0080_0fc0;
    localparam integer OUTPUT_BURSTS = (int'(OUTPUT_BASE[11:0])+OUTPUT_BYTES+4095)/4096;

    function automatic integer fp_word_count();
        integer total;
        begin
            total = 0;
            for (integer component=0; component<10; component++)
                total = total + COMPONENT_BYTES[component]/2;
            fp_word_count = total;
        end
    endfunction
    localparam integer FP_WORDS = fp_word_count();

    // Fold high address bits into the byte so row, column, tile, and component
    // mistakes do not alias to a simple repeating low-address-byte pattern.
    function automatic logic [7:0] byte_pattern(input logic [31:0] address);
        logic [31:0] mixed;
        begin
            mixed = (address ^ (address >> 7) ^ (address >> 13))*32'h045d_9f3b;
            mixed = mixed ^ (mixed >> 11) ^ (mixed >> 19);
            byte_pattern = mixed[7:0];
        end
    endfunction

    // A finite FP16 bit pattern derived from the complete scalar element index.
    // This is a storage-order oracle; no floating-point arithmetic is needed.
    function automatic logic [15:0] result_element(input integer unsigned index);
        logic [31:0] mixed;
        begin
            mixed = (index ^ (index >> 7) ^ (index >> 13))*32'h9e37_79b9;
            result_element = {mixed[15], (5'd8+{1'b0,mixed[14:11]}), mixed[9:0]};
        end
    endfunction

    function automatic integer min_int(input integer a, input integer b);
        min_int = (a<b) ? a : b;
    endfunction

    logic rst_n = 1'b0, start = 1'b0;
    wire busy, done, dma_error, block_start;
    logic block_done = 1'b0;
    wire ld_en;
    wire [3:0] ld_target;
    wire [20:0] ld_addr;
    wire [15:0] ld_data;
    wire wt_wr_en;
    wire [15:0] wt_wr_beat;
    wire [255:0] wt_wr_data;
    wire sb_wr_en, sb_wr_sel;
    wire [15:0] sb_wr_addr;
    wire [31:0] sb_wr_data;
    logic wload_req = 1'b0;
    logic [3:0] wload_stage = 4'd0;
    wire wload_ack;
    wire [14:0] result_rd_beat;
    logic [127:0] result_rd_data;

    wire [3:0] m_axi_awid, m_axi_arid;
    wire [31:0] m_axi_awaddr, m_axi_araddr;
    wire [7:0] m_axi_awlen, m_axi_arlen;
    wire [2:0] m_axi_awsize, m_axi_arsize;
    wire [1:0] m_axi_awburst, m_axi_arburst;
    wire m_axi_awvalid, m_axi_arvalid, m_axi_awready, m_axi_arready;
    wire [AXI_DATA_W-1:0] m_axi_wdata;
    wire [DATA_BYTES-1:0] m_axi_wstrb;
    wire m_axi_wlast, m_axi_wvalid, m_axi_wready, m_axi_bready;
    wire [3:0] m_axi_bid = 4'h3;
    wire [1:0] m_axi_bresp = 2'b00;
    logic m_axi_bvalid = 1'b0;
    wire [3:0] m_axi_rid = 4'h5;
    wire [1:0] m_axi_rresp = 2'b00;
    logic [AXI_DATA_W-1:0] m_axi_rdata = '0;
    logic m_axi_rlast = 1'b0, m_axi_rvalid = 1'b0;
    wire m_axi_rready;

    supra2_block_dma #(
        .DDR_BASE(DDR_BASE), .OUTPUT_BASE(OUTPUT_BASE), .AXI_DATA_W(AXI_DATA_W)
    ) dut (.*);

    // Every address is sampled at posedge; each 128-bit result is eight scalar
    // FP16 elements. NBA updates deliberately prohibit same-edge new-address
    // capture by the DMA. This models the latency-one public result port.
    always @(posedge clk) begin
        if (!rst_n) result_rd_data <= 'x;
        else begin
            for (integer lane=0; lane<8; lane++)
                result_rd_data[lane*16+:16] <= result_element(int'(result_rd_beat)*8+lane);
        end
    end

    integer cycles = 0;
    logic read_active = 1'b0, write_active = 1'b0, b_pending = 1'b0;
    logic [31:0] read_address = '0, write_address = '0;
    integer read_left = 0, write_left = 0;

    // Deterministic bounded stalls: no simulator-global random seed or races.
    // VALID and its payload remain asserted unchanged until a handshake.
    assign m_axi_arready = rst_n && !read_active && !m_axi_rvalid && ((cycles%5)!=0);
    assign m_axi_awready = rst_n && !write_active && !b_pending && !m_axi_bvalid && ((cycles%7)!=0);
    assign m_axi_wready = rst_n && write_active && ((cycles%7)>=3);

    always @(posedge clk) begin : axi_slave
        if (!rst_n) begin
            read_active <= 1'b0;
            read_address <= '0;
            read_left <= 0;
            m_axi_rvalid <= 1'b0;
            m_axi_rdata <= '0;
            m_axi_rlast <= 1'b0;
            write_active <= 1'b0;
            write_address <= '0;
            write_left <= 0;
            b_pending <= 1'b0;
            m_axi_bvalid <= 1'b0;
        end else begin
            if (m_axi_arvalid && m_axi_arready) begin
                if (read_active || m_axi_rvalid)
                    $fatal(1,"AXI%0d overlapping read burst",AXI_DATA_W);
                read_active <= 1'b1;
                read_address <= m_axi_araddr;
                read_left <= int'(m_axi_arlen)+1;
            end
            if (read_active && !m_axi_rvalid && ((cycles%5)!=1) && ((cycles%7)!=3)) begin
                for (integer lane=0; lane<DATA_BYTES; lane++)
                    m_axi_rdata[lane*8+:8] <= byte_pattern(read_address+32'(lane));
                m_axi_rvalid <= 1'b1;
                m_axi_rlast <= (read_left==1);
            end
            if (m_axi_rvalid && m_axi_rready) begin
                m_axi_rvalid <= 1'b0;
                m_axi_rlast <= 1'b0;
                read_address <= read_address+DATA_BYTES;
                read_left <= read_left-1;
                if (read_left==1) read_active <= 1'b0;
            end

            if (m_axi_awvalid && m_axi_awready) begin
                if (write_active || b_pending || m_axi_bvalid)
                    $fatal(1,"AXI%0d overlapping write burst",AXI_DATA_W);
                write_active <= 1'b1;
                write_address <= m_axi_awaddr;
                write_left <= int'(m_axi_awlen)+1;
            end
            if (m_axi_wvalid && m_axi_wready) begin
                write_address <= write_address+DATA_BYTES;
                write_left <= write_left-1;
                if (write_left==1) begin
                    write_active <= 1'b0;
                    b_pending <= 1'b1;
                end
            end
            if (b_pending && ((cycles%5)==2)) begin
                b_pending <= 1'b0;
                m_axi_bvalid <= 1'b1;
            end
            if (m_axi_bvalid && m_axi_bready) m_axi_bvalid <= 1'b0;
        end
    end

    integer fp_component = 0, fp_element = 0, fp_checked = 0;
    integer weight_columns_checked = 0, vector_words_checked = 0;
    integer block_starts = 0, acknowledgements = 0;
    integer ar_component = 0, ar_component_bytes = 0, read_beats = 0;
    integer ar_bursts = 0, aw_bursts = 0, b_responses = 0;
    integer output_bytes_reserved = 0, output_bytes_checked = 0;
    integer read_bubbles = 0, read_holds = 0, write_stalls = 0;
    integer ar_stalls = 0, aw_stalls = 0, weight_page_splits = 0;
    logic ar_held = 1'b0, aw_held = 1'b0, w_held = 1'b0;
    logic [48:0] held_ar, held_aw;
    logic [AXI_DATA_W+DATA_BYTES:0] held_w;

    always @(posedge clk) begin : scoreboard
        integer tile_index, column, expected_beat, component, element;
        integer transfer_bytes, expected_beats, expected_target, expected_ld_addr;
        logic [31:0] address;
        logic [15:0] expected_half;
        logic [31:0] expected_vector;
        if (!rst_n) begin
            cycles <= 0;
            ar_held <= 1'b0;
            aw_held <= 1'b0;
            w_held <= 1'b0;
        end else begin
            cycles <= cycles+1;
            if (!passed && cycles>=MAX_CYCLES)
                $fatal(1,"AXI%0d watchdog cycle=%0d fp=%0d wt=%0d sb=%0d output_bytes=%0d AR=%0d AW=%0d B=%0d",
                       AXI_DATA_W,cycles,fp_checked,weight_columns_checked,vector_words_checked,
                       output_bytes_checked,ar_bursts,aw_bursts,b_responses);
            if (dma_error !== 1'b0) $fatal(1,"AXI%0d dma_error=%b",AXI_DATA_W,dma_error);

            if (ar_held && ((m_axi_arvalid !== 1'b1) ||
                ({m_axi_arid,m_axi_araddr,m_axi_arlen,m_axi_arsize,m_axi_arburst} !== held_ar)))
                $fatal(1,"AXI%0d AR changed while stalled",AXI_DATA_W);
            if (aw_held && ((m_axi_awvalid !== 1'b1) ||
                ({m_axi_awid,m_axi_awaddr,m_axi_awlen,m_axi_awsize,m_axi_awburst} !== held_aw)))
                $fatal(1,"AXI%0d AW changed while stalled",AXI_DATA_W);
            if (w_held && ((m_axi_wvalid !== 1'b1) ||
                ({m_axi_wdata,m_axi_wstrb,m_axi_wlast} !== held_w)))
                $fatal(1,"AXI%0d W changed while stalled",AXI_DATA_W);
            ar_held <= m_axi_arvalid && !m_axi_arready;
            aw_held <= m_axi_awvalid && !m_axi_awready;
            w_held <= m_axi_wvalid && !m_axi_wready;
            held_ar <= {m_axi_arid,m_axi_araddr,m_axi_arlen,m_axi_arsize,m_axi_arburst};
            held_aw <= {m_axi_awid,m_axi_awaddr,m_axi_awlen,m_axi_awsize,m_axi_awburst};
            held_w <= {m_axi_wdata,m_axi_wstrb,m_axi_wlast};
            if (m_axi_arvalid && !m_axi_arready) ar_stalls <= ar_stalls+1;
            if (m_axi_awvalid && !m_axi_awready) aw_stalls <= aw_stalls+1;
            if (m_axi_wvalid && !m_axi_wready) write_stalls <= write_stalls+1;
            if (read_active && !m_axi_rvalid) read_bubbles <= read_bubbles+1;
            if (m_axi_rvalid && !m_axi_rready) read_holds <= read_holds+1;

            // Independently walk the actual package components 0..12:
            // ten FP inputs, stage-0 weights, scales, and biases.
            if (m_axi_arvalid && m_axi_arready) begin
                if (ar_component>12) $fatal(1,"AXI%0d unexpected extra AR",AXI_DATA_W);
                address = DDR_BASE+32'(COMPONENT_OFFSET[ar_component]+ar_component_bytes);
                expected_beats = min_int((COMPONENT_BYTES[ar_component]-ar_component_bytes)/DATA_BYTES,
                                         4096/DATA_BYTES);
                expected_beats = min_int(expected_beats,(4096-int'(address[11:0]))/DATA_BYTES);
                if (ar_component==10) begin
                    expected_beats = min_int(expected_beats,(1024-(ar_component_bytes%1024))/DATA_BYTES);
                    if ((expected_beats*DATA_BYTES)<(1024-(ar_component_bytes%1024)))
                        weight_page_splits <= weight_page_splits+1;
                end
                if (m_axi_arid !== 4'h5 || m_axi_arsize !== 3'($clog2(DATA_BYTES)) ||
                    m_axi_arburst !== 2'b01 || m_axi_araddr !== address ||
                    m_axi_arlen !== 8'(expected_beats-1))
                    $fatal(1,"AXI%0d bad AR component=%0d offset=%0d addr=%h expected=%h beats=%0d expected=%0d",
                           AXI_DATA_W,ar_component,ar_component_bytes,m_axi_araddr,address,
                           int'(m_axi_arlen)+1,expected_beats);
                transfer_bytes = (int'(m_axi_arlen)+1)*DATA_BYTES;
                if ((int'(m_axi_araddr[11:0])+transfer_bytes)>4096 ||
                    (int'(m_axi_araddr)%DATA_BYTES)!=0)
                    $fatal(1,"AXI%0d AR boundary/alignment violation",AXI_DATA_W);
                ar_bursts <= ar_bursts+1;
                if ((ar_component_bytes+transfer_bytes)==COMPONENT_BYTES[ar_component]) begin
                    ar_component <= ar_component+1;
                    ar_component_bytes <= 0;
                end else ar_component_bytes <= ar_component_bytes+transfer_bytes;
            end
            if (m_axi_rvalid && m_axi_rready) begin
                if (!read_active || read_left<1 || m_axi_rlast !== (read_left==1))
                    $fatal(1,"AXI%0d invalid R handshake/RLAST",AXI_DATA_W);
                read_beats <= read_beats+1;
            end

            if (ld_en) begin
                if (fp_component>=10) $fatal(1,"AXI%0d extra FP load",AXI_DATA_W);
                expected_target = (fp_component<2) ? fp_component : 2;
                expected_ld_addr = (fp_component<2) ? fp_element : (fp_component-2)*576+fp_element;
                address = DDR_BASE+32'(COMPONENT_OFFSET[fp_component]+fp_element*2);
                expected_half = {byte_pattern(address+1),byte_pattern(address)};
                if (ld_target !== 4'(expected_target) || ld_addr !== 21'(expected_ld_addr) ||
                    ld_data !== expected_half)
                    $fatal(1,"AXI%0d FP mismatch component=%0d element=%0d target=%0d addr=%0d data=%h expected=%h",
                           AXI_DATA_W,fp_component,fp_element,ld_target,ld_addr,ld_data,expected_half);
                fp_checked <= fp_checked+1;
                if (fp_element==COMPONENT_BYTES[fp_component]/2-1) begin
                    fp_component <= fp_component+1;
                    fp_element <= 0;
                end else fp_element <= fp_element+1;
            end
            if (block_start) begin
                if (fp_checked!=FP_WORDS || block_starts!=0)
                    $fatal(1,"AXI%0d premature/duplicate block_start FP=%0d",AXI_DATA_W,fp_checked);
                block_starts <= block_starts+1;
            end

            if (wt_wr_en) begin
                if (!wload_req || block_starts!=1 || weight_columns_checked>=WEIGHT_COLUMNS)
                    $fatal(1,"AXI%0d unexpected weight write",AXI_DATA_W);
                tile_index = weight_columns_checked/32;
                column = weight_columns_checked%32;
                expected_beat = ((tile_index/18)*32+column)*18+(tile_index%18);
                if (wt_wr_beat !== 16'(expected_beat))
                    $fatal(1,"AXI%0d weight beat tile=%0d col=%0d got=%0d expected=%0d",
                           AXI_DATA_W,tile_index,column,wt_wr_beat,expected_beat);
                for (integer lane=0; lane<32; lane++) begin
                    address = DDR_BASE+32'(COMPONENT_OFFSET[10]+tile_index*1024+lane*32+column);
                    if (wt_wr_data[lane*8+:8] !== byte_pattern(address))
                        $fatal(1,"AXI%0d weight data tile=%0d col=%0d lane=%0d addr=%h got=%h expected=%h",
                               AXI_DATA_W,tile_index,column,lane,address,
                               wt_wr_data[lane*8+:8],byte_pattern(address));
                end
                weight_columns_checked <= weight_columns_checked+1;
            end
            if (sb_wr_en) begin
                if (!wload_req || weight_columns_checked!=WEIGHT_COLUMNS || vector_words_checked>=2*STAGE_N)
                    $fatal(1,"AXI%0d premature/extra vector write",AXI_DATA_W);
                component = 11+vector_words_checked/STAGE_N;
                element = vector_words_checked%STAGE_N;
                address = DDR_BASE+32'(COMPONENT_OFFSET[component]+element*4);
                for (integer lane=0; lane<4; lane++)
                    expected_vector[lane*8+:8] = byte_pattern(address+32'(lane));
                if (sb_wr_sel !== (vector_words_checked>=STAGE_N) || sb_wr_addr !== 16'(element) ||
                    sb_wr_data !== expected_vector)
                    $fatal(1,"AXI%0d scale/bias mismatch component=%0d element=%0d data=%h expected=%h",
                           AXI_DATA_W,component,element,sb_wr_data,expected_vector);
                vector_words_checked <= vector_words_checked+1;
            end
            if (wload_ack) begin
                if (!wload_req || fp_checked!=FP_WORDS || weight_columns_checked!=WEIGHT_COLUMNS ||
                    vector_words_checked!=2*STAGE_N || acknowledgements!=0)
                    $fatal(1,"AXI%0d premature/duplicate weight ack",AXI_DATA_W);
                acknowledgements <= acknowledgements+1;
            end

            if (m_axi_awvalid && m_axi_awready) begin
                if (!block_done || acknowledgements!=1 || output_bytes_reserved>=OUTPUT_BYTES)
                    $fatal(1,"AXI%0d premature/extra AW",AXI_DATA_W);
                address = OUTPUT_BASE+32'(output_bytes_reserved);
                expected_beats = min_int((OUTPUT_BYTES-output_bytes_reserved)/DATA_BYTES,
                                         (4096-int'(address[11:0]))/DATA_BYTES);
                if (m_axi_awid !== 4'h3 || m_axi_awsize !== 3'($clog2(DATA_BYTES)) ||
                    m_axi_awburst !== 2'b01 || m_axi_awaddr !== address ||
                    m_axi_awlen !== 8'(expected_beats-1))
                    $fatal(1,"AXI%0d bad AW addr=%h expected=%h beats=%0d expected=%0d",
                           AXI_DATA_W,m_axi_awaddr,address,int'(m_axi_awlen)+1,expected_beats);
                transfer_bytes = (int'(m_axi_awlen)+1)*DATA_BYTES;
                if ((int'(m_axi_awaddr[11:0])+transfer_bytes)>4096 ||
                    (int'(m_axi_awaddr)%DATA_BYTES)!=0)
                    $fatal(1,"AXI%0d AW boundary/alignment violation",AXI_DATA_W);
                output_bytes_reserved <= output_bytes_reserved+transfer_bytes;
                aw_bursts <= aw_bursts+1;
            end
            if (m_axi_wvalid && !write_active)
                $fatal(1,"AXI%0d W without accepted AW",AXI_DATA_W);
            if (m_axi_wvalid && m_axi_wready) begin
                if (output_bytes_checked>=OUTPUT_BYTES || write_left<1 ||
                    write_address !== OUTPUT_BASE+32'(output_bytes_checked))
                    $fatal(1,"AXI%0d extra/noncontiguous output W",AXI_DATA_W);
                if (m_axi_wstrb !== {DATA_BYTES{1'b1}} || m_axi_wlast !== (write_left==1))
                    $fatal(1,"AXI%0d WSTRB/WLAST mismatch byte=%0d left=%0d last=%b",
                           AXI_DATA_W,output_bytes_checked,write_left,m_axi_wlast);
                for (integer lane=0; lane<DATA_BYTES; lane++) begin
                    element = (output_bytes_checked+lane)/2;
                    expected_half = result_element(element);
                    if (m_axi_wdata[lane*8+:8] !==
                        (((output_bytes_checked+lane)%2)==0 ? expected_half[7:0] : expected_half[15:8]))
                        $fatal(1,"AXI%0d output mismatch byte=%0d element=%0d got=%h expected_half=%h",
                               AXI_DATA_W,output_bytes_checked+lane,element,m_axi_wdata[lane*8+:8],expected_half);
                end
                output_bytes_checked <= output_bytes_checked+DATA_BYTES;
            end
            if (m_axi_bvalid && m_axi_bready) b_responses <= b_responses+1;
        end
    end

    initial begin : run_case
        if (WEIGHT_COMPONENT[0]!=10 || SCALE_COMPONENT[0]!=11 || BIAS_COMPONENT[0]!=12 ||
            COMPONENT_BYTES[10]!=TILE_COUNT*1024 || COMPONENT_BYTES[11]!=STAGE_N*4 ||
            COMPONENT_BYTES[12]!=STAGE_N*4)
            $fatal(1,"AXI%0d requires real stage-0 block13 layout",AXI_DATA_W);
        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk) start = 1'b1;
        @(negedge clk) start = 1'b0;
        wait (block_start === 1'b1);
        @(negedge clk) wload_req = 1'b1;
        wait (wload_ack === 1'b1);
        // Leave ACK asserted across its sampling edge before dropping REQ.
        @(posedge clk);
        @(negedge clk) wload_req = 1'b0;
        repeat (3) @(negedge clk);
        block_done = 1'b1;
        wait (done === 1'b1);
        @(negedge clk);
        block_done = 1'b0;
        if (busy !== 1'b0 || dma_error !== 1'b0 || fp_checked!=FP_WORDS || fp_component!=10 ||
            weight_columns_checked!=WEIGHT_COLUMNS || vector_words_checked!=2*STAGE_N ||
            block_starts!=1 || acknowledgements!=1 || ar_component!=13 || ar_component_bytes!=0 ||
            read_beats!=(FP_WORDS*2+COMPONENT_BYTES[10]+COMPONENT_BYTES[11]+COMPONENT_BYTES[12])/DATA_BYTES ||
            output_bytes_reserved!=OUTPUT_BYTES || output_bytes_checked!=OUTPUT_BYTES ||
            aw_bursts!=OUTPUT_BURSTS || b_responses!=aw_bursts ||
            read_active || m_axi_rvalid || write_active || b_pending || m_axi_bvalid)
            $fatal(1,"AXI%0d incomplete run FP=%0d wt=%0d sb=%0d output=%0d/%0d AW=%0d B=%0d",
                   AXI_DATA_W,fp_checked,weight_columns_checked,vector_words_checked,
                   output_bytes_checked,OUTPUT_BYTES,aw_bursts,b_responses);
        if (read_bubbles==0 || read_holds==0 || write_stalls==0 || weight_page_splits==0)
            $fatal(1,"AXI%0d missing stalls/page-split coverage R_bubbles=%0d R_holds=%0d W_stalls=%0d tile_splits=%0d",
                   AXI_DATA_W,read_bubbles,read_holds,write_stalls,weight_page_splits);
        $display("SUPRA2_BLOCK_DMA_MAPPING_WIDTH_PASS width=%0d cycles=%0d fp=%0d tiles=%0d columns=%0d output_beats=%0d AW=%0d R_bubbles=%0d R_holds=%0d W_stalls=%0d AR_stalls=%0d AW_stalls=%0d tile_splits=%0d",
                 AXI_DATA_W,cycles,fp_checked,TILE_COUNT,weight_columns_checked,OUTPUT_BEATS,aw_bursts,
                 read_bubbles,read_holds,write_stalls,ar_stalls,aw_stalls,weight_page_splits);
        passed = 1'b1;
    end
endmodule

`default_nettype wire
