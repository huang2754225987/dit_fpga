// =============================================================================
// silu_engine.sv - SiLU activation over 32-element FP16 vector (PostProc RF)
// =============================================================================
// SiLU(x) = x * sigmoid(x)
// Uses sigmoid LUT (8-bit index from FP16 upper bits) -> FP16 sigmoid approx.
// Then multiplies x * sigmoid(x) per element.
// =============================================================================
`default_nettype none

module silu_engine
  import isa_pkg::*;
#(
    parameter int LANES  = 32,
    parameter int ELEM_W = 16
) (
    input  wire                          clk,
    input  wire                          rst_n,

    // Command from ucode_decode
    input  wire                          cmd_valid,
    output logic                         cmd_ready,
    input  wire  [3:0]                   cmd_vd,
    input  wire  [3:0]                   cmd_vs1,
    // mode=0: SiLU  x*sigmoid(x); mode=1: bare sigmoid(x)
    input  wire                          cmd_mode,

    // PostProc RF read port
    output logic [3:0]                   rf_rd_addr,
    input  wire  [LANES*ELEM_W-1:0]     rf_rd_data,

    // PostProc RF write port
    output logic                         rf_wr_en,
    output logic [3:0]                   rf_wr_addr,
    output logic [LANES*ELEM_W-1:0]     rf_wr_data,

    output logic                         busy,
    output logic                         done
);

    import fp16_utils_pkg::*;

    typedef enum logic [2:0] {
        S_IDLE,
        S_LOAD,       // latch vector from RF
        S_LUT,        // drive LUT address for current element
        S_COMPUTE,    // latch LUT output, compute x * sigmoid(x)
        S_NEXT,       // advance index or finish
        S_WRITE,      // write result to RF
        S_DONE
    } state_t;

    state_t state, state_nxt;

    logic [3:0] r_vd, r_vs1;
    logic       r_mode;
    logic [4:0] r_idx;

    // Internal storage
    logic [ELEM_W-1:0] vec_in  [0:LANES-1];
    logic [ELEM_W-1:0] vec_out [0:LANES-1];

    // SiLU LUT: indexed by upper 8 bits of FP16, outputs FP16 sigmoid approximation
    logic [7:0]  lut_addr;
    logic [15:0] lut_data;

    silu_lut_fp16 u_silu_lut (
        .clk      (clk),
        .addr     (lut_addr),
        .data_out (lut_data)
    );

    // ----------------------------------------------------------------
    // FSM
    // ----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) state <= S_IDLE;
        else        state <= state_nxt;

    always_comb begin
        state_nxt = state;
        case (state)
            S_IDLE:    if (cmd_valid) state_nxt = S_LOAD;
            S_LOAD:    state_nxt = S_LUT;
            S_LUT:     state_nxt = S_COMPUTE;
            S_COMPUTE: state_nxt = S_NEXT;
            S_NEXT: begin
                if (r_idx == LANES[4:0] - 5'd1)
                    state_nxt = S_WRITE;
                else
                    state_nxt = S_LUT;
            end
            S_WRITE:   state_nxt = S_DONE;
            S_DONE:    state_nxt = S_IDLE;
            default:   state_nxt = S_IDLE;
        endcase
    end

    // ----------------------------------------------------------------
    // Command latch
    // ----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_vd   <= '0;
            r_vs1  <= '0;
            r_mode <= 1'b0;
        end else if (state == S_IDLE && cmd_valid) begin
            r_vd   <= cmd_vd;
            r_vs1  <= cmd_vs1;
            r_mode <= cmd_mode;
        end
    end

    // RF read (async)
    assign rf_rd_addr = (state == S_IDLE) ? cmd_vs1 : r_vs1;

    // ----------------------------------------------------------------
    // Latch vector
    // ----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (state == S_LOAD) begin
            for (int i = 0; i < LANES; i++)
                vec_in[i] <= rf_rd_data[i*ELEM_W +: ELEM_W];
        end
    end

    // ----------------------------------------------------------------
    // Index counter
    // ----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            r_idx <= '0;
        else if (state == S_LOAD)
            r_idx <= '0;
        else if (state == S_NEXT && state_nxt == S_LUT)
            r_idx <= r_idx + 5'd1;
    end

    // ----------------------------------------------------------------
    // LUT: upper 8 bits of FP16 -> sigmoid approximation (FP16)
    // ----------------------------------------------------------------
    assign lut_addr = (state == S_LUT) ? vec_in[r_idx][15:8] : 8'd0;

    // ----------------------------------------------------------------
    // Compute: out[i] = mode ? sigmoid_lut(x[i]) : x[i] * sigmoid_lut(x[i])
    // ----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (state == S_COMPUTE)
            vec_out[r_idx] <= r_mode ? lut_data
                                     : fp16_mul(vec_in[r_idx], lut_data);
    end

    // ----------------------------------------------------------------
    // Write result
    // ----------------------------------------------------------------
    always_comb begin
        rf_wr_en   = (state == S_WRITE);
        rf_wr_addr = r_vd;
        for (int i = 0; i < LANES; i++)
            rf_wr_data[i*ELEM_W +: ELEM_W] = vec_out[i];
    end

    // ----------------------------------------------------------------
    // Status
    // ----------------------------------------------------------------
    assign cmd_ready = (state == S_IDLE);
    assign busy      = (state != S_IDLE);
    assign done      = (state == S_DONE);

endmodule

`default_nettype wire
