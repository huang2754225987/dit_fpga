// =============================================================================
// acc_flush_ctrl.sv - Host-triggered sequential flush of acc_buffer (sim)
// =============================================================================
// Triggered by host writing REG_HOST_CMD[HOST_CMD_FLUSH_ACC] with
// REG_ACC_FLUSH_ADDR (DDR target, informational) and REG_ACC_FLUSH_LEN
// (number of 32-bit words).
//
// This is a SIM-FRIENDLY flush path: it does not drive a real AXI write master.
// Instead it sequentially reads acc_buffer (using its 1-cycle-latency read port)
// and exposes each word on the top-level outputs s_acc_flush_valid /
// s_acc_flush_addr / s_acc_flush_data, which the testbench samples to capture
// router logits into the shared DDR array.
//
// acc_buffer read port has 1-cycle latency: on cycle N, assert rd_en+rd_addr;
// on cycle N+1, rd_valid asserts and rd_data carries the word for that addr.
// We present the word on s_acc_flush_* in the same cycle that rd_valid is high,
// and the address on s_acc_flush_addr is the *requested* address.
// =============================================================================
`default_nettype none

module acc_flush_ctrl #(
    parameter int ACC_ADDR_W = 10,
    parameter int DATA_W     = 32,
    parameter int AXI_ADDR_W = 32
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // Host trigger and configuration
    input  wire                    trigger,         // 1-cycle pulse
    input  wire [AXI_ADDR_W-1:0]  acc_flush_addr,  // DDR target (informational)
    input  wire [31:0]            acc_flush_len,   // number of 32-bit words

    // acc_buffer read port (1-cycle latency)
    output logic                   acc_rd_en,
    output logic [ACC_ADDR_W-1:0]  acc_rd_addr,
    input  wire  [DATA_W-1:0]     acc_rd_data,
    input  wire                    acc_rd_valid,

    // Sim-friendly flush output (exposed at top level)
    output logic                   s_acc_flush_valid,
    output logic [31:0]            s_acc_flush_addr,
    output logic [31:0]            s_acc_flush_data,

    // Status
    output logic                   flush_active,
    output logic                   done
);
    typedef enum logic [1:0] { S_IDLE, S_READ, S_DONE } state_t;
    state_t state;

    logic [ACC_ADDR_W-1:0] req_addr;    // next address to request
    logic [ACC_ADDR_W-1:0] pend_addr;   // address of the in-flight read
    logic                  pending;     // a read is in flight (waiting for rd_valid)
    logic [31:0]           words_left;
    logic [31:0]           out_word_idx; // running word index on the output

    assign flush_active = (state != S_IDLE);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= S_IDLE;
            acc_rd_en         <= 1'b0;
            acc_rd_addr       <= '0;
            s_acc_flush_valid <= 1'b0;
            s_acc_flush_addr  <= '0;
            s_acc_flush_data  <= '0;
            req_addr          <= '0;
            pend_addr         <= '0;
            pending           <= 1'b0;
            words_left        <= '0;
            out_word_idx      <= '0;
            done              <= 1'b0;
        end else begin
            // defaults
            acc_rd_en         <= 1'b0;
            s_acc_flush_valid <= 1'b0;
            done              <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (trigger && acc_flush_len != 32'd0) begin
                        req_addr     <= '0;
                        pend_addr    <= '0;
                        pending     <= 1'b0;
                        words_left  <= acc_flush_len;
                        out_word_idx <= '0;
                        state        <= S_READ;
                    end
                end

                S_READ: begin
                    // Issue a read whenever no read is in flight and words remain
                    if (!pending && words_left != 32'd0) begin
                        acc_rd_en   <= 1'b1;
                        acc_rd_addr <= req_addr;
                        pend_addr   <= req_addr;
                        pending     <= 1'b1;
                        req_addr    <= req_addr + 1'b1;
                        words_left  <= words_left - 1'b1;
                    end

                    // Collect the in-flight read result
                    if (pending && acc_rd_valid) begin
                        s_acc_flush_valid <= 1'b1;
                        s_acc_flush_addr   <= {22'b0, pend_addr};  // word index in buffer
                        s_acc_flush_data   <= acc_rd_data;
                        out_word_idx       <= out_word_idx + 1'b1;
                        pending            <= 1'b0;
                    end

                    // Done issuing and the last in-flight beat has been drained
                    if (words_left == 32'd0 && !pending) begin
                        state <= S_DONE;
                    end
                end

                S_DONE: begin
                    state <= S_IDLE;
                    done  <= 1'b1;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
