`timescale 1ns/1ps

module tb_axi_to_mig_bridge;
    localparam int AXI_DATA_W = 128;
    localparam int MIG_DATA_W = 64;
    localparam int MIG_ADDR_W = 29;
    localparam int READ_BEATS = 64;
    localparam int READ_LATENCY = 20;
    localparam int READ_WORD_CREDITS = 8 * 2;

    logic clk = 1'b0;
    logic rst_n = 1'b0;

    logic [3:0] axi_awid = '0;
    logic [31:0] axi_awaddr = '0;
    logic [7:0] axi_awlen = '0;
    logic [2:0] axi_awsize = '0;
    logic [1:0] axi_awburst = '0;
    logic axi_awvalid = 1'b0;
    wire axi_awready;
    logic [AXI_DATA_W-1:0] axi_wdata = '0;
    logic [AXI_DATA_W/8-1:0] axi_wstrb = '0;
    logic axi_wlast = 1'b0;
    logic axi_wvalid = 1'b0;
    wire axi_wready;
    wire [3:0] axi_bid;
    wire [1:0] axi_bresp;
    wire axi_bvalid;
    logic axi_bready = 1'b1;

    logic [3:0] axi_arid = '0;
    logic [31:0] axi_araddr = '0;
    logic [7:0] axi_arlen = '0;
    logic [2:0] axi_arsize = '0;
    logic [1:0] axi_arburst = '0;
    logic axi_arvalid = 1'b0;
    wire axi_arready;
    wire [3:0] axi_rid;
    wire [AXI_DATA_W-1:0] axi_rdata;
    wire [1:0] axi_rresp;
    wire axi_rlast;
    wire axi_rvalid;
    logic axi_rready = 1'b0;

    wire [MIG_ADDR_W-1:0] app_addr;
    wire [2:0] app_cmd;
    wire app_en;
    wire app_hi_pri;
    wire [MIG_DATA_W-1:0] app_wdf_data;
    wire app_wdf_end;
    wire [MIG_DATA_W/8-1:0] app_wdf_mask;
    wire app_wdf_wren;
    wire [MIG_DATA_W-1:0] app_rd_data;
    wire app_rd_data_end;
    wire app_rd_data_valid;
    logic app_rdy = 1'b1;
    logic app_wdf_rdy = 1'b1;

    logic [MIG_DATA_W-1:0] memory [0:1023];
    logic [READ_LATENCY-1:0] read_valid_pipe;
    logic [MIG_ADDR_W-1:0] read_addr_pipe [0:READ_LATENCY-1];
    integer read_issue_count = 0;
    integer read_return_count = 0;
    integer read_outstanding = 0;
    integer max_read_outstanding = 0;

    integer i;
    integer beat;
    integer expected_word;
    logic [AXI_DATA_W-1:0] expected_data;

    always #5 clk = ~clk;

    axi_to_mig_bridge #(
        .AXI_DATA_W(AXI_DATA_W),
        .AXI_ADDR_W(32),
        .AXI_ID_W(4),
        .MIG_DATA_W(MIG_DATA_W),
        .MIG_ADDR_W(MIG_ADDR_W),
        .MIG_MASK_W(MIG_DATA_W / 8),
        .RATIO(2),
        .READ_FIFO_BEATS(8)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .s_axi_awid(axi_awid),
        .s_axi_awaddr(axi_awaddr),
        .s_axi_awlen(axi_awlen),
        .s_axi_awsize(axi_awsize),
        .s_axi_awburst(axi_awburst),
        .s_axi_awvalid(axi_awvalid),
        .s_axi_awready(axi_awready),
        .s_axi_wdata(axi_wdata),
        .s_axi_wstrb(axi_wstrb),
        .s_axi_wlast(axi_wlast),
        .s_axi_wvalid(axi_wvalid),
        .s_axi_wready(axi_wready),
        .s_axi_bid(axi_bid),
        .s_axi_bresp(axi_bresp),
        .s_axi_bvalid(axi_bvalid),
        .s_axi_bready(axi_bready),
        .s_axi_arid(axi_arid),
        .s_axi_araddr(axi_araddr),
        .s_axi_arlen(axi_arlen),
        .s_axi_arsize(axi_arsize),
        .s_axi_arburst(axi_arburst),
        .s_axi_arvalid(axi_arvalid),
        .s_axi_arready(axi_arready),
        .s_axi_rid(axi_rid),
        .s_axi_rdata(axi_rdata),
        .s_axi_rresp(axi_rresp),
        .s_axi_rlast(axi_rlast),
        .s_axi_rvalid(axi_rvalid),
        .s_axi_rready(axi_rready),
        .app_addr(app_addr),
        .app_cmd(app_cmd),
        .app_en(app_en),
        .app_hi_pri(app_hi_pri),
        .app_wdf_data(app_wdf_data),
        .app_wdf_end(app_wdf_end),
        .app_wdf_mask(app_wdf_mask),
        .app_wdf_wren(app_wdf_wren),
        .app_rd_data(app_rd_data),
        .app_rd_data_end(app_rd_data_end),
        .app_rd_data_valid(app_rd_data_valid),
        .app_rdy(app_rdy),
        .app_wdf_rdy(app_wdf_rdy)
    );

    assign app_rd_data_valid = read_valid_pipe[READ_LATENCY-1];
    assign app_rd_data_end = app_rd_data_valid;
    assign app_rd_data = memory[read_addr_pipe[READ_LATENCY-1][12:3]];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            read_valid_pipe <= '0;
            for (integer stage = 0; stage < READ_LATENCY; stage = stage + 1)
                read_addr_pipe[stage] <= '0;
            read_issue_count <= 0;
            read_return_count <= 0;
            read_outstanding <= 0;
            max_read_outstanding <= 0;
        end else begin
            read_valid_pipe <= {read_valid_pipe[READ_LATENCY-2:0],
                                app_en && app_cmd == 3'b001 && app_rdy};
            for (integer stage = READ_LATENCY - 1; stage > 0; stage = stage - 1)
                read_addr_pipe[stage] <= read_addr_pipe[stage-1];
            if (app_en && app_cmd == 3'b001 && app_rdy) begin
                read_addr_pipe[0] <= app_addr;
                read_issue_count <= read_issue_count + 1;
            end
            if (app_rd_data_valid)
                read_return_count <= read_return_count + 1;
            case ({app_en && app_cmd == 3'b001 && app_rdy,
                   app_rd_data_valid})
                2'b10: begin
                    read_outstanding <= read_outstanding + 1;
                    if (read_outstanding + 1 > max_read_outstanding)
                        max_read_outstanding <= read_outstanding + 1;
                end
                2'b01: read_outstanding <= read_outstanding - 1;
                default: ;
            endcase
        end
    end

    initial begin
        for (i = 0; i < 1024; i = i + 1)
            memory[i] = 64'hA5A5_0000_0000_0000 | 64'(i);

        repeat (5) @(negedge clk);
        rst_n = 1'b1;

        @(negedge clk);
        axi_arid = 4'h9;
        axi_araddr = 32'h0000_0200;
        axi_arlen = READ_BEATS - 1;
        axi_arsize = 3'd4;
        axi_arburst = 2'b01;
        axi_arvalid = 1'b1;
        do @(posedge clk); while (!axi_arready);
        @(negedge clk);
        axi_arvalid = 1'b0;

        for (beat = 0; beat < READ_BEATS; beat = beat + 1) begin
            do @(negedge clk); while (!axi_rvalid);
            expected_word = (32'h0000_0200 >> 3) + beat * 2;
            expected_data = {memory[expected_word + 1], memory[expected_word]};
            if (axi_rid !== 4'h9 || axi_rresp !== 2'b00 ||
                axi_rdata !== expected_data ||
                axi_rlast !== (beat == READ_BEATS - 1))
                $fatal(1, "AXI R mismatch beat=%0d id=%h data=%h expected=%h last=%b",
                       beat, axi_rid, axi_rdata, expected_data, axi_rlast);

            if ((beat % 8) == 0) begin
                repeat (4) begin
                    @(negedge clk);
                    if (!axi_rvalid || axi_rdata !== expected_data ||
                        axi_rlast !== (beat == READ_BEATS - 1))
                        $fatal(1, "AXI R response changed under backpressure at beat %0d",
                               beat);
                end
            end

            axi_rready = 1'b1;
            @(posedge clk);
            @(negedge clk);
            axi_rready = 1'b0;
            repeat (2) @(negedge clk);
        end

        repeat (4) @(negedge clk);
        if (axi_rvalid || read_issue_count != READ_BEATS * 2 ||
            read_return_count != READ_BEATS * 2 || read_outstanding != 0)
            $fatal(1, "Read accounting mismatch issued=%0d returned=%0d outstanding=%0d rvalid=%b",
                   read_issue_count, read_return_count, read_outstanding,
                   axi_rvalid);
        if (max_read_outstanding != READ_WORD_CREDITS)
            $fatal(1, "MIG read credit limit mismatch: max outstanding=%0d expected=%0d",
                   max_read_outstanding, READ_WORD_CREDITS);

        $display("AXI_TO_MIG_BRIDGE_PIPELINE_PASS beats=%0d max_mig_outstanding=%0d",
                 READ_BEATS, max_read_outstanding);
        $finish;
    end
endmodule
