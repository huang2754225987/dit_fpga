// =============================================================================
// vld_vst_ctrl.sv - VLD/VST Controller (PostProc RF <-> L1 Cache)
// =============================================================================
// VLD: reads 2 consecutive 256-bit words from L1 → packs into 512-bit → writes
//      to PostProc RF register vd
// VST: reads 512-bit PostProc RF register vs1 → writes 2 consecutive 256-bit
//      words to L1
// =============================================================================
`default_nettype none

module vld_vst_ctrl #(
    parameter int L1_ADDR_W = 11,
    parameter int L1_DATA_W = 256,
    parameter int VEC_W     = 512
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // VLD command
    input  wire                   vld_cmd_valid,
    input  wire  [3:0]           vld_cmd_vd,
    input  wire  [15:0]          vld_cmd_addr,
    input  wire  [1:0]           vld_cmd_page,
    output logic                  vld_done,

    // VST command
    input  wire                   vst_cmd_valid,
    input  wire  [3:0]           vst_cmd_vs1,
    input  wire  [15:0]          vst_cmd_addr,
    input  wire  [1:0]           vst_cmd_page,
    output logic                  vst_done,

    // L1 cache read port (shared with gld_ctrl via top-level mux)
    output logic                  l1_rd_en,
    output logic [L1_ADDR_W-1:0] l1_rd_addr,
    output logic [1:0]           l1_rd_page,
    input  wire  [L1_DATA_W-1:0] l1_rd_data,
    input  wire                   l1_rd_valid,

    // L1 cache write port
    output logic                  l1_wr_en,
    output logic [L1_ADDR_W-1:0] l1_wr_addr,
    output logic [1:0]           l1_wr_page,
    output logic [L1_DATA_W-1:0] l1_wr_data,

    // PostProc RF read port (for VST)
    output logic [3:0]           prf_rd_addr,
    input  wire  [VEC_W-1:0]    prf_rd_data,

    // PostProc RF write port (for VLD)
    output logic                  prf_wr_en,
    output logic [3:0]           prf_wr_addr,
    output logic [VEC_W-1:0]    prf_wr_data,

    output logic                  busy
);

    typedef enum logic [2:0] {
        S_IDLE,
        S_VLD_READ,
        S_VLD_COLLECT,
        S_VLD_WRITE,
        S_VST_READ,
        S_VST_WRITE,
        S_DONE
    } state_t;

    state_t state, state_nxt;

    logic [L1_ADDR_W-1:0] r_l1_base;
    logic [1:0]           r_l1_page;
    logic [1:0]           r_beat_cnt;   // 0-1 beats for 512/256 = 2
    logic [3:0]           r_vd, r_vs1;
    logic                 r_is_vld;
    logic [VEC_W-1:0]    r_vec_buf;     // collect 2 × 256 bits

    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) state <= S_IDLE;
        else        state <= state_nxt;

    always_comb begin
        state_nxt = state;
        case (state)
            S_IDLE: begin
                if (vld_cmd_valid)      state_nxt = S_VLD_READ;
                else if (vst_cmd_valid) state_nxt = S_VST_READ;
            end
            S_VLD_READ:    state_nxt = S_VLD_COLLECT;
            S_VLD_COLLECT: begin
                if (l1_rd_valid) begin
                    if (r_beat_cnt == 2'd1)
                        state_nxt = S_VLD_WRITE;
                    else
                        state_nxt = S_VLD_READ;
                end
            end
            S_VLD_WRITE:   state_nxt = S_DONE;
            S_VST_READ:    state_nxt = S_VST_WRITE;
            S_VST_WRITE: begin
                if (r_beat_cnt == 2'd1)
                    state_nxt = S_DONE;
            end
            S_DONE:        state_nxt = S_IDLE;
            default:       state_nxt = S_IDLE;
        endcase
    end

    // Command latch
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_l1_base <= '0;
            r_l1_page <= 1'b0;
            r_vd      <= '0;
            r_vs1     <= '0;
            r_is_vld  <= 1'b0;
        end else if (state == S_IDLE) begin
            if (vld_cmd_valid) begin
                r_l1_base <= vld_cmd_addr[L1_ADDR_W+4:5];
                r_l1_page <= vld_cmd_page;
                r_vd      <= vld_cmd_vd;
                r_is_vld  <= 1'b1;
            end else if (vst_cmd_valid) begin
                r_l1_base <= vst_cmd_addr[L1_ADDR_W+4:5];
                r_l1_page <= vst_cmd_page;
                r_vs1     <= vst_cmd_vs1;
                r_is_vld  <= 1'b0;
            end
        end
    end

    // Beat counter
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            r_beat_cnt <= '0;
        else if (state == S_IDLE)
            r_beat_cnt <= '0;
        else if (state == S_VLD_COLLECT && l1_rd_valid)
            r_beat_cnt <= r_beat_cnt + 2'd1;
        else if (state == S_VST_WRITE)
            r_beat_cnt <= r_beat_cnt + 2'd1;
    end

    // VLD: collect 2 × 256-bit into vector buffer
    always_ff @(posedge clk) begin
        if (state == S_VLD_COLLECT && l1_rd_valid) begin
            case (r_beat_cnt)
                2'd0: r_vec_buf[255:0]   <= l1_rd_data;
                2'd1: r_vec_buf[511:256] <= l1_rd_data;
            endcase
        end else if (state == S_VST_READ) begin
            r_vec_buf <= prf_rd_data;
        end
    end

    // L1 read control (for VLD)
    assign l1_rd_en   = (state == S_VLD_READ);
    assign l1_rd_addr = r_l1_base + {9'b0, r_beat_cnt};
    assign l1_rd_page = r_l1_page;

    // L1 write control (for VST)
    assign l1_wr_en   = (state == S_VST_WRITE);
    assign l1_wr_addr = r_l1_base + {9'b0, r_beat_cnt};
    assign l1_wr_page = r_l1_page;
    always_comb begin
        case (r_beat_cnt)
            2'd0: l1_wr_data = r_vec_buf[255:0];
            2'd1: l1_wr_data = r_vec_buf[511:256];
        endcase
    end

    // PostProc RF read (for VST: read register vs1)
    assign prf_rd_addr = r_vs1;

    // PostProc RF write (for VLD: write register vd)
    assign prf_wr_en   = (state == S_VLD_WRITE);
    assign prf_wr_addr = r_vd;
    assign prf_wr_data = r_vec_buf;

    // Done signals
    assign vld_done = (state == S_DONE) && r_is_vld;
    assign vst_done = (state == S_DONE) && !r_is_vld;
    assign busy     = (state != S_IDLE);

endmodule

`default_nettype wire
