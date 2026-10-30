// =============================================================================
// loop_controller.sv - Hardware Loop Buffer (K-tile sequencer)
// =============================================================================
// Sits between ucode_fetch and ucode_decode. Detects LOOP instruction,
// buffers the loop body in a small cache, and replays it N times.
//
// LOOP instruction encoding (reuse OP_NOP, vd=0xF):
//   [31:28] = 0 (OP_NOP)
//   [27:24] = 0xF (LOOP marker)
//   [23:20] = body_size (1-15 instructions)
//   [19:16] = prec (0=FP16, 1=INT1, 2=INT2, 3=INT3)
//   [15:0]  = repeat_count (1-65535)
//
// Per-iteration auto-modification:
//   DMA_RD (opcode=12): imm += stride * iter, vd toggles pair (vd[1] ^= 1)
//   GLD_W  (opcode=3):  imm toggles L1 pair (XOR 0x2000 if iter odd)
//   GLD_A  (opcode=4):  legacy imm toggles L1 pair; explicitly marked
//                      contiguous-address loads keep their compiler address
//   GEMM   (opcode=5):  acc=1 (iter>0), skip_store=(iter < count-1 ? 1 : 0)
//
// Stride: imm is in 16-byte (128-bit) word units.
// From ucode generator: w_off = (nt*AD*K + kt*K_tile) * 2 bytes
// Per K-tile increment = K_tile * 2 / 16 = K_tile / 8 (16B units)
// Same formula for all precisions (ucode uses *2 for all).
// =============================================================================
`default_nettype none

module loop_controller #(
    parameter int BUF_DEPTH = 16  // instruction buffer depth
)(
    input  wire         clk,
    input  wire         rst_n,

    // From ucode_fetch
    input  wire         fetch_valid,
    input  wire [31:0]  fetch_instr,
    output logic        fetch_ready,   // 0 when replaying (pause fetch)

    // To ucode_decode
    output logic        dec_valid,
    output logic [31:0] dec_instr,
    input  wire         dec_ready,
    output wire         static_k128_replay,

    // Runtime route-group predicates used by empty-group and inactive-row skips.
    input  wire         route_group_empty,
    input  wire [5:0]   route_group_count,
    // Fast-forward the source PC past an empty group's encoded instruction body.
    output wire         source_skip_valid,
    output wire [15:0]  source_skip_count,

    // Config
    input  wire [15:0]  cfg_K          // K_tile size (e.g., 64)
);

    // Cached-loop marker: NOP/vd=F/body length plus vs2=F. TGLD uses
    // vd=F for a 512-byte row stride and vs1=1 for non-transposed loads, so
    // vd/body-length alone aliases a legal transpose-load instruction.
    wire is_loop = (fetch_instr[31:28] == 4'b0000) &&
                   (fetch_instr[27:24] == 4'hF) &&
                   (fetch_instr[23:20] != 4'h0) && // body_size > 0
                   (fetch_instr[19:16] == 4'hF);  // reserved loop marker

    // Loop body buffer
    logic [31:0] loop_buf [0:BUF_DEPTH-1];

    // Latched loop parameters
    logic [3:0]  r_body_size;
    logic [15:0] r_repeat_count;
    logic [1:0]  r_prec;
    logic        r_static_k128_loop;

    // State
    logic [4:0]  r_fill_cnt;    // 0..15, fill counter
    logic [4:0]  r_replay_idx;  // 0..14, index within loop body
    logic [15:0] r_iter;        // current iteration (0..repeat_count-1)

    // Declare the state before combinational outputs reference it.
    typedef enum logic [1:0] {
        S_PASS,    // pass-through (normal fetch)
        S_FILL,    // buffering loop body (1st iteration passes through)
        S_REPLAY   // replay from buffer (iterations 1..N-1)
    } state_t;

    state_t state, state_nxt;

    assign static_k128_replay = (state == S_REPLAY) && r_static_k128_loop;

    // NOP with vd=E, vs1=F and imm[15:12]=0 is a conditional source-word
    // skip marker. vs2 carries count[15:12], imm[11:0] carries count[11:0].
    wire is_group_skip = (fetch_instr[31:28] == 4'h0) &&
                         (fetch_instr[27:24] == 4'hE) &&
                         (fetch_instr[23:20] == 4'hF) &&
                         (fetch_instr[15:12] == 4'h0);
    // A row marker uses vd=E, vs1=D, non-zero imm[3:0]; imm[3] is
    // row[4], vs2 is row[3:0], and imm[2:0] is the skip length.
    wire is_group_row_skip = (fetch_instr[31:28] == 4'h0) &&
                             (fetch_instr[27:24] == 4'hE) &&
                             (fetch_instr[23:20] == 4'hD) &&
                             (fetch_instr[15:4] == 12'd0) &&
                             (fetch_instr[3:0] != 4'd0);
    wire group_row_inactive =
        route_group_count <= {fetch_instr[3], fetch_instr[19:16]};

    // The marker itself still passes through decode and waits for all prior
    // work. Once accepted, ucode_fetch can skip the dead source words in one
    // PC update instead of issuing one SRAM read per skipped instruction.
    assign source_skip_valid = (state == S_PASS) && fetch_valid && dec_ready &&
                               ((is_group_skip && route_group_empty) ||
                                (is_group_row_skip && group_row_inactive));
    assign source_skip_count = is_group_row_skip
                             ? {13'd0, fetch_instr[2:0]}
                             : {fetch_instr[19:16], fetch_instr[11:0]};

    // Stride: each tile = AD × K_tile × 2B in 16B word units
    // K_tile=64: 32 × 64 × 2 = 4096B = 256 in 16B units
    // Stride = K_tile * AD * 2 / 16 = K_tile * 4 (for AD=32)
    logic [15:0] stride;
    assign stride = cfg_K * 4;  // K_tile * 4 (16B word units)

    // Scale group advance during replay: group_size=128 elements, one loop
    // iteration consumes cfg_K elements, so group = base + (iter * cfg_K)/128
    // = base + (iter >> group_shift) with group_shift = 7 - log2(cfg_K).
    logic [3:0] group_shift;
    always_comb begin
        case (cfg_K)
            16'd16:  group_shift = 4'd3;
            16'd32:  group_shift = 4'd2;
            16'd64:  group_shift = 4'd1;
            default: group_shift = 4'd0; // cfg_K >= 128: group advances every iter
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_PASS;
            r_fill_cnt   <= '0;
            r_replay_idx <= '0;
            r_iter       <= '0;
            r_body_size  <= '0;
            r_repeat_count <= '0;
            r_prec       <= '0;
            r_static_k128_loop <= 1'b0;
        end else begin
            case (state)
                S_PASS: begin
                    if (fetch_valid && is_loop && dec_ready) begin
                        // Latch loop parameters, start filling
                        r_body_size    <= fetch_instr[23:20];
                        r_repeat_count <= fetch_instr[15:0];
                        r_prec         <= fetch_instr[17:16];
                        r_static_k128_loop <= (fetch_instr[19:16] == 4'hF);
                        r_fill_cnt     <= '0;
                        state          <= S_FILL;
                    end
                end

                S_FILL: begin
                    // Buffer each instruction and pass it through (1st iteration)
                    if (fetch_valid && dec_ready) begin
                        loop_buf[r_fill_cnt] <= fetch_instr;
                        r_fill_cnt <= r_fill_cnt + 1;
                        if (r_fill_cnt == r_body_size - 1) begin
                            // Buffer full, start replay from iteration 1
                            r_iter       <= 16'd1;
                            r_replay_idx <= '0;
                            state        <= S_REPLAY;
                        end
                    end
                end

                S_REPLAY: begin
                    if (dec_ready) begin
                        if (r_replay_idx == r_body_size - 1) begin
                            // End of this iteration's body
                            r_replay_idx <= '0;
                            if (r_iter == r_repeat_count - 1) begin
                                // All iterations done
                                state <= S_PASS;
                            end else begin
                                r_iter <= r_iter + 16'd1;
                            end
                        end else begin
                            r_replay_idx <= r_replay_idx + 1;
                        end
                    end
                end

            endcase
        end
    end

    // =========================================================================
    // Instruction modification during replay
    // =========================================================================
    logic [31:0] replay_instr;
    always_comb begin
        logic [31:0] base;
        logic        toggle;
        logic [4:0]  wide_scale_group;
        logic [15:0] static_dma_stride;

        base   = loop_buf[r_replay_idx[3:0]];
        toggle = r_iter[0]; // 1 for odd iterations
        wide_scale_group = {1'b0, base[19], base[22:20]} + r_iter[4:0];
        // DMA length codes for static K=128 groups encode 512/1024/2048/
        // 4096 bytes as 32/64/128/256 sixteen-byte address units.
        case (base[19:16])
            4'd3: static_dma_stride = 16'd32;
            4'd4: static_dma_stride = 16'd64;
            4'd5: static_dma_stride = 16'd128;
            4'd6: static_dma_stride = 16'd192;
            default: static_dma_stride = 16'd256;
        endcase

        replay_instr = base;

        case (base[31:28])
            4'd12: begin // DMA_RD
                // Toggle the bank paired with this loop family. Static K=128
                // weights use banks 2/3 (vd[0]); legacy loops use pairs such
                // as 0/2 (vd[1]). This must match GLD.W's 0x2000/0x3000 flip.
                if (toggle) begin
                    if (r_static_k128_loop)
                        replay_instr[24] = ~base[24]; // bank 2 <-> 3
                    else
                        replay_instr[25] = ~base[25]; // legacy pair toggle
                end
                // Auto-increment DDR offset
                if (r_static_k128_loop && base[23:20] == 4'h4 &&
                    base[19:16] >= 4'd3 && base[19:16] <= 4'd7)
                    replay_instr[15:0] = base[15:0] +
                                         (static_dma_stride * r_iter);
                else
                    replay_instr[15:0] = base[15:0] + (stride * r_iter);
            end

            4'd3: begin // GLD_W
                if (r_static_k128_loop) begin
                    // Static K=128 uses L1 banks 2/3; scale group is encoded
                    // across vs1[2:0] and vs2[3].
                    if (toggle)
                        replay_instr[12] = ~base[12]; // 0x2000 <-> 0x3000
                    replay_instr[22:20] = wide_scale_group[2:0];
                    replay_instr[19] = wide_scale_group[3];
                end else begin
                    // Match DMA_RD's vd[1] ping-pong (bank xor 2).
                    if (toggle)
                        replay_instr[13] = ~base[13]; // flip bit 13 (0x2000)
                    // Advance scale group (vs1) with the iteration index; the
                    // buffered body carries the group of iteration 0.
                    replay_instr[23:20] = base[23:20] +
                                          ((r_iter >> group_shift) & 16'hF);
                end
            end

            4'd4: begin // GLD_A
                if (r_static_k128_loop)
                    replay_instr[15:0] = base[15:0] + (16'd256 * r_iter);
                else begin
                    // Legacy GLD.A loops alternate the L1 source pair. Compiler-
                    // marked linear activation loads instead carry an explicit
                    // contiguous K/row-stride address and must not be redirected.
                    if (toggle &&
                        !(base[23:20] == 4'h1 ||
                          (base[23:22] == 2'b11 && base[1])))
                        replay_instr[12] = ~base[12]; // flip bit 12 (0x1000)
                end
            end

            4'd5: begin // GEMM
                // acc = 1 (always, since replay starts at iter=1)
                replay_instr[26] = 1'b1;
                // Keep every segment non-final until its final replay.
                replay_instr[27] = r_static_k128_loop
                    ? (base[27] || (r_iter != r_repeat_count - 1))
                    : ((r_iter == r_repeat_count - 1) ? 1'b0 : 1'b1);
            end

            default: ; // pass through unchanged
        endcase
    end

    // =========================================================================
    // Output mux
    // =========================================================================
    always_comb begin
        case (state)
            S_PASS:  begin dec_valid = fetch_valid; dec_instr = fetch_instr; end
            S_FILL:  begin dec_valid = fetch_valid; dec_instr = fetch_instr; end
            S_REPLAY: begin dec_valid = 1'b1;       dec_instr = replay_instr; end
            default: begin dec_valid = 1'b0;       dec_instr = '0;           end
        endcase
    end

    // Fetch ready: pause fetch during replay, pass-through dec_ready otherwise
    assign fetch_ready = (state == S_PASS)  ? dec_ready :
                         (state == S_FILL) ? dec_ready : 1'b0;

endmodule

`default_nettype wire
