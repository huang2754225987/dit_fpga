// Single-clock block DMA for the block-13 DDR image.
// The AXI clock is intended to be MIG ui_clk; the attached block therefore
// runs on that same clock in this first integrated system configuration.
`timescale 1ns/1ps
`default_nettype none

module supra2_block_dma #(
    parameter logic [31:0] DDR_BASE = 32'h0010_0000,
    parameter logic [31:0] OUTPUT_BASE = 32'h0080_0000,
    parameter integer AXI_DATA_W = 128
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         start,
    output logic        busy,
    output logic        done,
    output logic        dma_error,

    // supra2_block_top load/control interface
    output logic        block_start,
    input  wire         block_done,
    output logic        ld_en,
    output logic [3:0]  ld_target,
    output logic [20:0] ld_addr,
    output logic [15:0] ld_data,
    output logic        wt_wr_en,
    output logic [15:0] wt_wr_beat,
    output logic [255:0] wt_wr_data,
    output logic        sb_wr_en,
    output logic        sb_wr_sel,
    output logic [15:0] sb_wr_addr,
    output logic [31:0] sb_wr_data,
    input  wire         wload_req,
    input  wire [3:0]   wload_stage,
    output logic        wload_ack,
    output logic [14:0] result_rd_beat,
    // Latency-one read: address sampled at an edge, data updated after it.
    // S_OUT_READ waits before capture; combinational sources also work.
    input  wire [127:0] result_rd_data,

    // AXI4 master. Supported widths: 128-bit simulation and 512-bit board MIG.
    output wire [3:0]   m_axi_awid,
    output logic [31:0] m_axi_awaddr,
    output logic [7:0]  m_axi_awlen,
    output wire [2:0]   m_axi_awsize,
    output wire [1:0]   m_axi_awburst,
    output logic        m_axi_awvalid,
    input  wire         m_axi_awready,
    output wire [AXI_DATA_W-1:0] m_axi_wdata,
    output wire [AXI_DATA_W/8-1:0] m_axi_wstrb,
    output wire         m_axi_wlast,
    output logic        m_axi_wvalid,
    input  wire         m_axi_wready,
    input  wire [3:0]   m_axi_bid,
    input  wire [1:0]   m_axi_bresp,
    input  wire         m_axi_bvalid,
    output logic        m_axi_bready,
    output wire [3:0]   m_axi_arid,
    output logic [31:0] m_axi_araddr,
    output logic [7:0]  m_axi_arlen,
    output wire [2:0]   m_axi_arsize,
    output wire [1:0]   m_axi_arburst,
    output logic        m_axi_arvalid,
    input  wire         m_axi_arready,
    input  wire [3:0]   m_axi_rid,
    input  wire [AXI_DATA_W-1:0] m_axi_rdata,
    input  wire [1:0]   m_axi_rresp,
    input  wire         m_axi_rlast,
    input  wire         m_axi_rvalid,
    output logic        m_axi_rready
);
    import block13_ddr_layout::*;

    localparam integer ROWS=256, FEAT=576, X_ELEMENTS=ROWS*FEAT;
    localparam integer DATA_BYTES=AXI_DATA_W/8;
    localparam integer FP16_LANES=AXI_DATA_W/16;
    localparam integer FP32_LANES=AXI_DATA_W/32;
    localparam integer OUTPUT_RATIO=AXI_DATA_W/128;
    localparam integer OUTPUT_BEATS=X_ELEMENTS/FP16_LANES;
    localparam integer TILE_BEATS=1024/DATA_BYTES;
    localparam integer MAX_BURST_BEATS=4096/DATA_BYTES;
    localparam integer MAX_K=2304;
    localparam integer STG_N [0:6] = '{1728,576,576,1152,576,2304,576};
    localparam integer STG_K [0:6] = '{576,576,576,576,576,576,2304};

    typedef enum logic [4:0] {
        S_IDLE, S_FP_SETUP, S_FP_AR, S_FP_R, S_FP_EMIT, S_FP_NEXT,
        S_W_WAIT, S_W_AR, S_W_R, S_W_EMIT,
        S_V_SETUP, S_V_AR, S_V_R, S_V_EMIT, S_V_NEXT,
        S_ACK, S_WAIT_DROP, S_BLOCK_START, S_BLOCK_RUN,
        S_OUT_AW, S_OUT_PACK, S_OUT_W, S_OUT_B, S_DONE, S_OUT_READ
    } state_t;
    state_t state;

    logic [31:0] component_base, tile_read_addr;
    logic [31:0] beats_left;
    logic [8:0] burst_beats_left;
    logic [AXI_DATA_W-1:0] beat_data;
    logic [$clog2(FP16_LANES)-1:0] fp_lane;
    logic [$clog2(FP32_LANES)-1:0] vec_lane;
    // Payload has 31 components (indices 0..30); 4 bits silently wrapped
    // scale/bias component indices >=16 and generated incorrect AXI offsets.
    logic [4:0] component_index;
    logic [3:0] fp_target;
    logic [20:0] fp_base_addr;
    logic [3:0] stage_index;
    logic [15:0] n_tiles, k_tiles, n_tile_index, k_tile_index;
    logic [5:0] tile_beats_received;
    logic [10:0] tile_bytes_received;
    logic [5:0] emit_n_lane;
    logic [6:0] emit_k_tile;
    logic [15:0] vector_index;
    logic vector_is_bias;
    logic [14:0] output_beat_index;
    logic [14:0] output_beats_left;
    logic [8:0] output_burst_left;
    logic [1:0] output_pack_lane;
    logic [AXI_DATA_W-1:0] output_pack_data;
    wire [255:0] tile_bank_data;
    wire [255:0] tile_column_data;
    wire tile_store_we = rst_n && (state==S_W_R) && m_axi_rvalid && m_axi_rready;

    // Tile byte (row, col), originally at row*32+col, lives in bank
    // row XOR col. For a fixed output column, all 32 rows use different
    // banks, each with one asynchronous read. Only the packed read outputs
    // are permuted; no memory is selected by a variable bank index.
    // On AXI512, a beat holds rows 2*t and 2*t+1. Each bank receives one
    // byte from each row, packed into one entire 16-bit word at address t.
    // Thus byte (r,c) is mem[r/2][8*(r%2)+:8] in bank r XOR c.
    // On AXI128, a beat holds half a row: 16 banks each receive one byte.
    // Byte (r,c) is mem[r] in bank r XOR c for this width.
    // Storage has no reset; the complete tile is loaded before S_W_EMIT.
    generate
        for(genvar bank=0;bank<32;bank++) begin : g_tile_bank
            localparam logic [4:0] BANK = 5'(bank);
            wire [4:0] wr_col = BANK ^ tile_bytes_received[9:5];
            wire [4:0] rd_row = BANK ^ emit_n_lane[4:0];
            if(AXI_DATA_W==512) begin : g_axi512
                (* ram_style = "distributed" *) logic [15:0] mem [0:15];
                wire [4:0] wr_col_odd = wr_col ^ 5'd1;
                wire [15:0] rd_word = mem[rd_row[4:1]];
                always_ff @(posedge clk) begin
                    if(tile_store_we)
                        mem[tile_bytes_received[9:6]] <= {
                            m_axi_rdata[256+wr_col_odd*8+:8],
                            m_axi_rdata[wr_col*8+:8]
                        };
                end
                assign tile_bank_data[bank*8+:8] = rd_row[0] ? rd_word[15:8] : rd_word[7:0];
            end else begin : g_axi128
                (* ram_style = "distributed" *) logic [7:0] mem [0:31];
                always_ff @(posedge clk) begin
                    if(tile_store_we && (wr_col[4]==tile_bytes_received[4]))
                        mem[tile_bytes_received[9:5]] <= m_axi_rdata[wr_col[3:0]*8+:8];
                end
                assign tile_bank_data[bank*8+:8] = mem[rd_row];
            end
        end
        for(genvar row=0;row<32;row++) begin : g_tile_column
            wire [4:0] rd_bank = 5'(row) ^ emit_n_lane[4:0];
            assign tile_column_data[row*8+:8] = tile_bank_data[rd_bank*8+:8];
        end
    endgenerate

    assign m_axi_awid=4'h3;
    assign m_axi_awsize=3'($clog2(DATA_BYTES));
    assign m_axi_awburst=2'b01;
    assign m_axi_wstrb={DATA_BYTES{1'b1}};
    assign m_axi_wdata=output_pack_data;
    assign m_axi_wlast=(state==S_OUT_W)&&(output_burst_left==1);
    assign m_axi_arid=4'h5;
    assign m_axi_arsize=3'($clog2(DATA_BYTES));
    assign m_axi_arburst=2'b01;
    assign result_rd_beat=15'(output_beat_index*OUTPUT_RATIO+output_pack_lane);
    assign busy=(state!=S_IDLE)&&(state!=S_DONE);

    always_comb begin
        integer ar_page_beats;
        integer aw_page_beats;
        m_axi_arvalid=1'b0;
        m_axi_araddr=32'b0;
        m_axi_arlen=8'b0;
        m_axi_rready=1'b0;
        m_axi_awvalid=(state==S_OUT_AW);
        m_axi_awaddr=OUTPUT_BASE+(32'(OUTPUT_BEATS-output_beats_left)*DATA_BYTES);
        m_axi_awlen=(output_beats_left>MAX_BURST_BEATS)?8'(MAX_BURST_BEATS-1):8'(output_beats_left-1'b1);
        m_axi_wvalid=(state==S_OUT_W);
        m_axi_bready=(state==S_OUT_B);

        ld_en=1'b0; ld_target=4'b0; ld_addr=21'b0; ld_data=16'b0;
        wt_wr_en=1'b0; wt_wr_beat=16'b0; wt_wr_data=256'b0;
        sb_wr_en=1'b0; sb_wr_sel=1'b0; sb_wr_addr=16'b0; sb_wr_data=32'b0;
        wload_ack=(state==S_ACK);
        block_start=(state==S_BLOCK_START);

        if(state==S_FP_AR) begin
            m_axi_arvalid=1'b1;
            m_axi_araddr=component_base+32'((COMPONENT_BYTES[component_index]/DATA_BYTES-beats_left)*DATA_BYTES);
            m_axi_arlen=(beats_left>MAX_BURST_BEATS)?8'(MAX_BURST_BEATS-1):8'(beats_left-1'b1);
        end else if(state==S_FP_R) begin
            m_axi_rready=1'b1;
        end else if(state==S_V_AR) begin
            m_axi_arvalid=1'b1;
            m_axi_araddr=component_base+32'((COMPONENT_BYTES[component_index]/DATA_BYTES-beats_left)*DATA_BYTES);
            m_axi_arlen=(beats_left>MAX_BURST_BEATS)?8'(MAX_BURST_BEATS-1):8'(beats_left-1'b1);
        end else if(state==S_V_R) begin
            m_axi_rready=1'b1;
        end else if(state==S_W_AR) begin
            m_axi_arvalid=1'b1;
            m_axi_araddr=tile_read_addr+32'(tile_bytes_received);
            m_axi_arlen=8'(TILE_BEATS-int'(tile_beats_received)-1);
        end else if(state==S_W_R) begin
            m_axi_rready=1'b1;
        end

        // AXI4 bursts may not cross a 4-KiB boundary. Clamp the selected
        // transfer length using the current byte address, including tile reads.
        ar_page_beats=(4096-int'(m_axi_araddr[11:0]))/DATA_BYTES;
        if(m_axi_arvalid && (int'(m_axi_arlen)+1>ar_page_beats))
            m_axi_arlen=8'(ar_page_beats-1);
        aw_page_beats=(4096-int'(m_axi_awaddr[11:0]))/DATA_BYTES;
        if(m_axi_awvalid && (int'(m_axi_awlen)+1>aw_page_beats))
            m_axi_awlen=8'(aw_page_beats-1);

        if(state==S_FP_EMIT) begin
            ld_en=1'b1; ld_target=fp_target;
            // The absolute element address advances once per AXI beat, then
            // the selected FP16 subword lane is added.
            ld_addr=fp_base_addr+21'(fp_lane);
            ld_data=beat_data[fp_lane*16+:16];
        end
        if(state==S_W_EMIT) begin
            wt_wr_en=1'b1;
            wt_wr_beat=16'(((n_tile_index*32+emit_n_lane)*(STG_K[stage_index]/32))+emit_k_tile);
            wt_wr_data=tile_column_data;
        end
        if(state==S_V_EMIT) begin
            sb_wr_en=1'b1; sb_wr_sel=vector_is_bias;
            sb_wr_addr=16'(vector_index*FP32_LANES+vec_lane);
            sb_wr_data=beat_data[vec_lane*32+:32];
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin : dma_fsm
        integer beats_for_burst;
        if(!rst_n) begin
            state<=S_IDLE; done<=1'b0; dma_error<=1'b0;
            component_base<='0; tile_read_addr<='0; beats_left<='0;
            burst_beats_left<='0; beat_data<='0;
            fp_lane<='0; vec_lane<='0; component_index<='0;
            fp_target<='0; fp_base_addr<='0; stage_index<='0;
            n_tiles<='0; k_tiles<='0; n_tile_index<='0; k_tile_index<='0;
            tile_beats_received<='0; tile_bytes_received<='0;
            emit_n_lane<='0; emit_k_tile<='0; vector_index<='0; vector_is_bias<=1'b0;
            output_beat_index<='0; output_beats_left<='0; output_burst_left<='0;
            output_pack_lane<='0; output_pack_data<='0;
        end else begin
            done<=1'b0;
            if((m_axi_rvalid&&m_axi_rready)&&((m_axi_rresp!=2'b00)||(m_axi_rid!=4'h5)))
                dma_error<=1'b1;
            if((m_axi_bvalid&&m_axi_bready)&&((m_axi_bresp!=2'b00)||(m_axi_bid!=4'h3)))
                dma_error<=1'b1;

            case(state)
                S_IDLE: if(start) begin
                    dma_error<=1'b0; component_index<=0; state<=S_FP_SETUP;
                end
                S_FP_SETUP: begin
                    component_base<=DDR_BASE+32'(COMPONENT_OFFSET[component_index]);
                    beats_left<=32'(COMPONENT_BYTES[component_index]/DATA_BYTES);
                    if(component_index<2) begin
                        fp_target<=4'(component_index); fp_base_addr<=0;
                    end else begin
                        fp_target<=4'd2; fp_base_addr<=21'((component_index-2)*FEAT);
                    end
                    state<=S_FP_AR;
                end
                S_FP_AR: if(m_axi_arvalid&&m_axi_arready) begin
                    beats_for_burst=int'(m_axi_arlen)+1;
                    burst_beats_left<=9'(beats_for_burst);
                    state<=S_FP_R;
                end
                S_FP_R: if(m_axi_rvalid&&m_axi_rready) begin
                    beat_data<=m_axi_rdata; fp_lane<=0; state<=S_FP_EMIT;
                end
                S_FP_EMIT: begin
                    if(fp_lane==FP16_LANES-1) begin
                        fp_lane<=0; fp_base_addr<=fp_base_addr+FP16_LANES;
                        beats_left<=beats_left-1'b1;
                        burst_beats_left<=burst_beats_left-1'b1;
                        if(beats_left==1) state<=S_FP_NEXT;
                        else if(burst_beats_left==1) state<=S_FP_AR;
                        else state<=S_FP_R;
                    end else fp_lane<=fp_lane+1'b1;
                end
                S_FP_NEXT: begin
                    if(component_index==9) state<=S_BLOCK_START;
                    else begin component_index<=component_index+1'b1; state<=S_FP_SETUP; end
                end
                // Keep servicing the block's descriptor-driven weight requests
                // after launch. The block cannot complete until each request
                // is acknowledged, so waiting for block_done here deadlocks.
                S_BLOCK_START: state<=S_W_WAIT;
                S_BLOCK_RUN: state<=S_W_WAIT;

                S_W_WAIT: begin
                    if(block_done) begin
                        output_beat_index<=0;
                        output_beats_left<=15'(OUTPUT_BEATS);
                        state<=S_OUT_AW;
                    end else if(wload_req) begin
                        if(wload_stage>6) begin dma_error<=1'b1; state<=S_DONE; end
                        else begin
                            stage_index<=wload_stage;
                            n_tiles<=16'(STG_N[wload_stage]/32);
                            k_tiles<=16'(STG_K[wload_stage]/32);
                            n_tile_index<=0; k_tile_index<=0;
                            component_base<=DDR_BASE+32'(COMPONENT_OFFSET[WEIGHT_COMPONENT[wload_stage]]);
                            tile_read_addr<=DDR_BASE+32'(COMPONENT_OFFSET[WEIGHT_COMPONENT[wload_stage]]);
                            tile_bytes_received<=0; tile_beats_received<=0;
                            emit_n_lane<=0;
                            state<=S_W_AR;
                        end
                    end
                end
                S_W_AR: if(m_axi_arvalid&&m_axi_arready) begin
                    burst_beats_left<=9'(int'(m_axi_arlen)+1);
                    state<=S_W_R;
                end
                S_W_R: if(m_axi_rvalid&&m_axi_rready) begin
                    tile_bytes_received<=tile_bytes_received+DATA_BYTES;
                    tile_beats_received<=tile_beats_received+1'b1;
                    burst_beats_left<=burst_beats_left-1'b1;
                    if(tile_beats_received==TILE_BEATS-1) begin
                        emit_k_tile<=7'(k_tile_index);
                        state<=S_W_EMIT;
                    end
                    else if(burst_beats_left==1) state<=S_W_AR;
                end
                S_W_EMIT: begin
                    if(emit_n_lane<31) emit_n_lane<=emit_n_lane+1'b1;
                    else if(k_tile_index+1<k_tiles) begin
                        k_tile_index<=k_tile_index+1'b1;
                        tile_read_addr<=component_base+32'(((n_tile_index*k_tiles)+(k_tile_index+1))*1024);
                        tile_bytes_received<=0; tile_beats_received<=0;
                        emit_n_lane<=0;
                        state<=S_W_AR;
                    end else if(n_tile_index+1<n_tiles) begin
                        n_tile_index<=n_tile_index+1'b1; k_tile_index<=0;
                        tile_read_addr<=component_base+32'(((n_tile_index+1)*k_tiles)*1024);
                        tile_bytes_received<=0; tile_beats_received<=0;
                        emit_n_lane<=0;
                        state<=S_W_AR;
                    end else begin vector_is_bias<=1'b0; state<=S_V_SETUP; end
                end

                S_V_SETUP: begin
                    component_index<=vector_is_bias ? BIAS_COMPONENT[stage_index] : SCALE_COMPONENT[stage_index];
                    component_base<=DDR_BASE+32'(COMPONENT_OFFSET[vector_is_bias ? BIAS_COMPONENT[stage_index] : SCALE_COMPONENT[stage_index]]);
                    beats_left<=32'(STG_N[stage_index]/FP32_LANES);
                    vector_index<=0; vec_lane<=0; state<=S_V_AR;
                end
                S_V_AR: if(m_axi_arvalid&&m_axi_arready) begin
                    beats_for_burst=int'(m_axi_arlen)+1;
                    burst_beats_left<=9'(beats_for_burst);
                    state<=S_V_R;
                end
                S_V_R: if(m_axi_rvalid&&m_axi_rready) begin beat_data<=m_axi_rdata; vec_lane<=0; state<=S_V_EMIT; end
                S_V_EMIT: begin
                    if(vec_lane==FP32_LANES-1) begin
                        vec_lane<=0; vector_index<=vector_index+1'b1;
                        beats_left<=beats_left-1'b1; burst_beats_left<=burst_beats_left-1'b1;
                        if(beats_left==1) state<=S_V_NEXT;
                        else if(burst_beats_left==1) state<=S_V_AR;
                        else state<=S_V_R;
                    end else vec_lane<=vec_lane+1'b1;
                end
                S_V_NEXT: if(!vector_is_bias) begin vector_is_bias<=1'b1; state<=S_V_SETUP; end
                         else state<=S_ACK;
                S_ACK: state<=S_WAIT_DROP;
                S_WAIT_DROP: if(!wload_req) state<=S_W_WAIT;

                S_OUT_AW: if(m_axi_awvalid&&m_axi_awready) begin
                    beats_for_burst=int'(m_axi_awlen)+1;
                    output_burst_left<=9'(beats_for_burst);
                    output_pack_lane<=0;
                    state<=S_OUT_READ;
                end
                // The address is stable throughout this cycle. A latency-one
                // result RAM updates after this edge; PACK captures next edge.
                S_OUT_READ: state<=S_OUT_PACK;
                S_OUT_PACK: begin
                    output_pack_data[output_pack_lane*128+:128]<=result_rd_data;
                    if(output_pack_lane==OUTPUT_RATIO-1) begin
                        output_pack_lane<=0;
                        state<=S_OUT_W;
                    end else begin
                        output_pack_lane<=output_pack_lane+1'b1;
                        state<=S_OUT_READ;
                    end
                end
                S_OUT_W: if(m_axi_wvalid&&m_axi_wready) begin
                    output_beats_left<=output_beats_left-1'b1;
                    output_beat_index<=output_beat_index+1'b1;
                    output_burst_left<=output_burst_left-1'b1;
                    if(output_burst_left==1) state<=S_OUT_B;
                    else begin
                        output_pack_lane<=0;
                        state<=S_OUT_READ;
                    end
                end
                S_OUT_B: if(m_axi_bvalid&&m_axi_bready) begin
                    if(output_beats_left==0) state<=S_DONE;
                    else state<=S_OUT_AW;
                end
                S_DONE: begin done<=1'b1; state<=S_IDLE; end
                default: begin dma_error<=1'b1; state<=S_DONE; end
            endcase
        end
    end

`ifndef SYNTHESIS
    initial begin
        if ((AXI_DATA_W!=128)&&(AXI_DATA_W!=512))
            $fatal(1,"AXI_DATA_W must be 128 or 512");
        if ((1024%DATA_BYTES)!=0)
            $fatal(1,"AXI_DATA_W must divide the 1-KiB weight tile");
    end
`endif

    // Avoid an unused-input warning without affecting response checking.
    wire unused_bid = ^m_axi_bid;
endmodule
`default_nettype wire
