// PL-only UART loader for wide DDR writes and a block-start command.
// Packet format: 0x57, byte address (u32 LE), AXI beat count (u32 LE), then
// beat_count consecutive AXI_DATA_W/8-byte payload beats. 0x53 starts the block.
`timescale 1ns/1ps
`default_nettype none
module uart_ddr_loader #(
    parameter integer CLK_HZ = 300_000_000,
    parameter integer BAUD = 115_200,
    parameter integer AXI_DATA_W = 128
)(
    input wire clk,
    input wire rst_n,
    input wire uart_rx,
    output wire uart_tx,
    input wire block_done,
    output logic start_pulse,
    output logic load_error,
    output wire [3:0] host_awid,
    output logic [31:0] host_awaddr,
    output wire [7:0] host_awlen,
    output wire [2:0] host_awsize,
    output wire [1:0] host_awburst,
    output logic host_awvalid,
    input wire host_awready,
    output logic [AXI_DATA_W-1:0] host_wdata,
    output wire [AXI_DATA_W/8-1:0] host_wstrb,
    output wire host_wlast,
    output logic host_wvalid,
    input wire host_wready,
    input wire [3:0] host_bid,
    input wire [1:0] host_bresp,
    input wire host_bvalid,
    output logic host_bready,
    output wire [3:0] host_arid,
    output wire [31:0] host_araddr,
    output wire [7:0] host_arlen,
    output wire [2:0] host_arsize,
    output wire [1:0] host_arburst,
    output wire host_arvalid,
    input wire host_arready,
    input wire [3:0] host_rid,
    input wire [AXI_DATA_W-1:0] host_rdata,
    input wire [1:0] host_rresp,
    input wire host_rlast,
    input wire host_rvalid,
    output wire host_rready
);
    localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;
    localparam integer RX_CW = $clog2(CLKS_PER_BIT + 1);
    localparam integer DATA_BYTES = AXI_DATA_W / 8;
    localparam integer BYTE_INDEX_W = $clog2(DATA_BYTES);

    typedef enum logic [2:0] {RX_IDLE, RX_START, RX_BITS, RX_STOP} rx_state_t;
    rx_state_t rx_state;
    logic [RX_CW-1:0] rx_count;
    logic [2:0] rx_bit;
    logic [7:0] rx_shift, rx_data;
    logic rx_valid;
    logic uart_rx_meta, uart_rx_sync;
    logic tx_busy, tx_done, tx_start;
    logic [7:0] tx_data;
    logic [9:0] tx_shift;
    logic [3:0] tx_bits_left;
    logic [RX_CW-1:0] tx_count;
    logic done_reported;

    typedef enum logic [2:0] {P_CMD, P_ADDR, P_COUNT, P_DATA, P_AW, P_W, P_B, P_ACK} parser_state_t;
    parser_state_t parser_state;
    logic [1:0] field_byte;
    logic [31:0] field_value;
    logic [31:0] beats_left;
    logic [BYTE_INDEX_W-1:0] payload_byte;
    logic [31:0] write_address;
    logic [AXI_DATA_W-1:0] payload_word;

    assign host_awid=4'h1;
    assign host_awlen=8'd0;
    assign host_awsize=3'($clog2(DATA_BYTES));
    assign host_awburst=2'b01;
    assign host_wstrb={DATA_BYTES{1'b1}};
    assign host_wlast=1'b1;
    assign host_arid=4'b0;
    assign host_araddr=32'b0;
    assign host_arlen=8'b0;
    assign host_arsize=3'b0;
    assign host_arburst=2'b01;
    assign host_arvalid=1'b0;
    assign host_rready=1'b1;
    assign uart_tx=tx_busy ? tx_shift[0] : 1'b1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin uart_rx_meta<=1'b1; uart_rx_sync<=1'b1; end
        else begin uart_rx_meta<=uart_rx; uart_rx_sync<=uart_rx_meta; end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state<=RX_IDLE; rx_count<='0; rx_bit<='0;
            rx_shift<='0; rx_data<='0; rx_valid<=1'b0;
        end else begin
            rx_valid<=1'b0;
            case (rx_state)
                RX_IDLE: if (!uart_rx_sync) begin
                    rx_count<=RX_CW'(CLKS_PER_BIT/2);
                    rx_state<=RX_START;
                end
                RX_START: if (rx_count==0) begin
                    if (!uart_rx_sync) begin
                        rx_count<=RX_CW'(CLKS_PER_BIT-1);
                        rx_bit<=0; rx_state<=RX_BITS;
                    end else rx_state<=RX_IDLE;
                end else rx_count<=rx_count-1'b1;
                RX_BITS: if (rx_count==0) begin
                    rx_shift[rx_bit]<=uart_rx_sync;
                    rx_count<=RX_CW'(CLKS_PER_BIT-1);
                    if (rx_bit==7) rx_state<=RX_STOP;
                    else rx_bit<=rx_bit+1'b1;
                end else rx_count<=rx_count-1'b1;
                RX_STOP: if (rx_count==0) begin
                    rx_data<=rx_shift; rx_valid<=uart_rx_sync;
                    rx_state<=RX_IDLE;
                end else rx_count<=rx_count-1'b1;
                default: rx_state<=RX_IDLE;
            endcase
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_busy<=1'b0; tx_done<=1'b0; tx_shift<=10'h3ff;
            tx_bits_left<=0; tx_count<=0;
        end else begin
            tx_done<=1'b0;
            if (tx_start&&!tx_busy) begin
                tx_shift<={1'b1,tx_data,1'b0};
                tx_bits_left<=10; tx_count<=RX_CW'(CLKS_PER_BIT-1);
                tx_busy<=1'b1;
            end else if (tx_busy) begin
                if (tx_count==0) begin
                    tx_shift<={1'b1,tx_shift[9:1]};
                    tx_count<=RX_CW'(CLKS_PER_BIT-1);
                    if (tx_bits_left==1) begin
                        tx_bits_left<=0; tx_busy<=1'b0; tx_done<=1'b1;
                    end else tx_bits_left<=tx_bits_left-1'b1;
                end else tx_count<=tx_count-1'b1;
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            parser_state<=P_CMD; field_byte<=0; field_value<=0;
            beats_left<=0; payload_byte<=0; write_address<=0;
            payload_word<=0; host_awaddr<=0; host_awvalid<=0;
            host_wdata<=0; host_wvalid<=0; host_bready<=0;
            start_pulse<=0; load_error<=0;
            tx_start<=0; tx_data<=0;
            done_reported<=0;
        end else begin
            start_pulse<=1'b0;
            tx_start<=1'b0;
            if (block_done&&!done_reported) begin
                tx_data<=8'h04; tx_start<=1'b1; done_reported<=1'b1;
            end
            case (parser_state)
                P_CMD: if (rx_valid) begin
                    if (rx_data==8'h57) begin
                        field_byte<=0; field_value<=0;
                        parser_state<=P_ADDR;
                    end else if (rx_data==8'h53) begin
                        start_pulse<=1'b1; tx_data<=8'h06; tx_start<=1'b1;
                        done_reported<=1'b0;
                    end
                end
                P_ADDR, P_COUNT: if (rx_valid) begin
                    field_value[field_byte*8+:8]<=rx_data;
                    if (field_byte==3) begin
                        field_byte<=0;
                        if (parser_state==P_ADDR) begin
                            write_address<={rx_data,field_value[23:0]};
                            field_value<=0; parser_state<=P_COUNT;
                        end else begin
                            beats_left<={rx_data,field_value[23:0]};
                            field_value<=0;
                            if ({rx_data,field_value[23:0]}==0) begin
                                load_error<=1'b1; parser_state<=P_CMD;
                            end else begin
                                payload_byte<=0; payload_word<=0;
                                parser_state<=P_DATA;
                            end
                        end
                    end else field_byte<=field_byte+1'b1;
                end
                P_DATA: if (rx_valid) begin
                    payload_word[payload_byte*8+:8]<=rx_data;
                    if (payload_byte==DATA_BYTES-1) begin
                        host_wdata<={rx_data,payload_word[AXI_DATA_W-1:8]};
                        host_awaddr<=write_address;
                        host_awvalid<=1'b1; host_wvalid<=1'b0;
                        parser_state<=P_AW; payload_byte<=0;
                    end else payload_byte<=payload_byte+1'b1;
                end
                P_AW: begin
                    if (host_awvalid&&host_awready) begin
                        host_awvalid<=1'b0; host_wvalid<=1'b1;
                        parser_state<=P_W;
                    end
                end
                P_W: begin
                    if (host_wvalid&&host_wready) begin
                        host_wvalid<=1'b0; host_bready<=1'b1;
                        parser_state<=P_B;
                    end
                end
                P_B: begin
                    if (host_bvalid&&host_bready) begin
                        host_bready<=1'b0;
                        if ((host_bresp!=2'b00)||(host_bid!=4'h1))
                            load_error<=1'b1;
                        tx_data<=((host_bresp==2'b00)&&(host_bid==4'h1))?8'h06:8'h15;
                        tx_start<=1'b1; parser_state<=P_ACK;
                    end
                end
                P_ACK: begin
                    if (tx_done) begin
                        if (beats_left>1) begin
                            beats_left<=beats_left-1'b1;
                            write_address<=write_address+32'(DATA_BYTES);
                            payload_byte<=0; payload_word<=0;
                            parser_state<=P_DATA;
                        end else begin beats_left<=0; parser_state<=P_CMD; end
                    end
                end
                default: parser_state<=P_CMD;
            endcase
        end
    end

`ifndef SYNTHESIS
    initial if (CLKS_PER_BIT<16) $fatal(1,"UART baud rate too high for clock");
`endif
endmodule
`default_nettype wire
