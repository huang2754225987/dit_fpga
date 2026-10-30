// =============================================================================
// scoreboard.sv - 8-bit Resource Scoreboard with Data-Valid Handshake
// =============================================================================
// Tracks busy/idle state of all hardware resources. 8-bit scoreboard:
//   bit0: DMA_RD     bit1: DMA_WR     bit2: GEMM      bit3: POSTPROC
//   bit4: WRF_WR (GLD_W)  bit5: ARF_WR (GLD_A)
//   bit6: WRF_RD (GEMM)   bit7: ARF_RD (GEMM)
//
// Key change from 6-bit: WRF/ARF split into WR/RD. GEMM acquires RD bits,
// GLD acquires WR bits → GLD can run during GEMM (different banks, no hazard).
//
// Data-valid state: WRF/ARF contents remain valid across GEMM reads because
// GEMM does not modify either RF. A GLD completion publishes the newly filled
// inactive bank; GEMM acquire must not consume the validity of resident data.
// =============================================================================
`default_nettype none

module scoreboard
  import isa_pkg::*;
(
    input  wire        clk,
    input  wire        rst_n,

    // Acquire: decoder sets bits when issuing instruction (8-bit)
    input  wire [7:0]  acquire,

    // Release: each engine/unit clears its bit when done
    input  wire        dma_rd_done,
    input  wire        dma_wr_done,
    input  wire        gemm_done,
    input  wire        postproc_done,
    input  wire        weight_rf_done,   // WRF + scale ready, bank safe to publish/reuse
    input  wire        act_rf_done,      // ARF bank safe to publish/reuse

    // Status output (8-bit)
    output logic [7:0] res_busy,
    output logic       all_idle,

    // Data-valid handshake (GEMM waits for these before reading RF)
    output logic       wrf_data_valid,  // set when a completed WRF bank is published
    output logic       arf_data_valid   // set when a completed ARF bank is published
);

    // === 8-bit resource busy tracking ===
    // Release map (MSB→LSB = bit7→bit0):
    //   bit7 (ARF_RD): gemm_done
    //   bit6 (WRF_RD): gemm_done
    //   bit5 (ARF_WR): act_rf_done  (NOT gemm_done)
    //   bit4 (WRF_WR): weight_rf_done (NOT gemm_done)
    //   bit3 (POSTPROC): postproc_done
    //   bit2 (GEMM): gemm_done
    //   bit1 (DMA_WR): dma_wr_done
    //   bit0 (DMA_RD): dma_rd_done
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            res_busy <= 8'b0;
        end else begin
            res_busy <= (res_busy | acquire) &
                        ~{gemm_done, gemm_done,       // bit7, bit6
                           act_rf_done, weight_rf_done, // bit5, bit4
                           postproc_done, gemm_done,    // bit3, bit2
                           dma_wr_done, dma_rd_done};   // bit1, bit0
        end
    end

    // === Data-valid handshake ===
    // Set when the inactive RF bank is complete and safe to publish. If a GEMM
    // was still reading the other bank at producer completion, top.sv delays
    // this handshake until GEMM retirement and the deferred bank flip.
    // GEMM reads are non-destructive, so validity is sticky after the first
    // completed load. This permits a resident activation/weight tile to feed
    // consecutive GEMMs (for example Gate/Up or adjacent N tiles) without an
    // artificial reload. A producer's *_rf_done pulse publishes its newly
    // filled bank; decode independently blocks GEMM while a producer is busy.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wrf_data_valid <= 1'b0;
            arf_data_valid <= 1'b0;
        end else begin
            if (weight_rf_done)
                wrf_data_valid <= 1'b1;

            if (act_rf_done)
                arf_data_valid <= 1'b1;
        end
    end

    assign all_idle = (res_busy == 8'b0);

endmodule

`default_nettype wire
