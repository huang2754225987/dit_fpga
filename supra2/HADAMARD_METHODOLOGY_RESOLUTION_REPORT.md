# Hadamard synthesis and timing investigation

Target: xczu7ev-ffvc1156-2-e, Vivado 2021.1, internal OOC clock 2.500 ns.
These results do not include the board clock tree, MIG, external I/O delays,
the complete compute bridge, or full inference.

## Measured changes

| Design revision | LUT | FF | DSP / BRAM | Outcome |
|---|---:|---:|---:|---|
| H-block scratch, dynamic bank gather/scatter | 117,104 | 8,261 | 0 / 0 | Synthesis completed; memory guard stopped timing reporting |
| H-block scratch, static bank permutations, pre-jam fix | 53,904 | 8,266 | 0 / 0 | Synthesis completed; 21 Synth 8-295 critical warnings; guarded stop after DCP save |
| Scalar registered FP32 adder, barrel jam helper | 620 | 96 | 0 / 0 | Synthesis and text methodology checks completed; internal setup timing fails |

The two Hadamard runs preserve sixteen butterfly pairs per issue cycle.
The static bank layout supports full-width issue only (`PAIRS_PER_CYCLE =
LANES/2`). The scalar probe is a diagnostic and does not replace the full
frontend as the implementation target.

The shared jam helper now uses distinct 1/2/4/8/16-bit barrel-stage variables.
The original inferred-loop warnings point to the former helper implementation;
this localization is not a proof that every full-Hadamard loop is resolved.

## Validation and artifacts

- Full static-bank, pre-jam x_embed/QKV/MLP.2 256-row Questa regressions pass
  with zero Hadamard, quantizer and software-chain mismatches. Projection
  counts remain 15,264 / 322,784 / 545,696 cycles. These are component counts.
- Post-jam right-shift-jam tests pass 24,576 cases: every 8-bit shift amount
  for 96 basis/edge/LFSR vectors. Log directory:
  `build/fp32_shift_right_jam/questa_runs/20261023_233536/`.
- Post-jam five-shape Hadamard golden/backpressure checks pass in
  `build/supra2_hadamard/questa_runs/20261023_233853/`.
- Post-jam actual GEMM M=N=32 tests check all 1,024 FP16 outputs of each
  x_embed/QKV/MLP.2 tile and raw-data hold during stalls. All pass in
  `build/supra2_gemm_frontend/questa_runs/20261023_233902/`.
- Pre-jam static-bank DCP and area report:
  `build/supra2_hadamard_ooc/vivado_ooc_2p500ns_20261023_232548/`.
- Current scalar probe reports:
  `build/supra2_fp32_add_probe/20261023_234302/`.
  The original synthesis DCP is in the sibling `20261023_233823/` directory.

## Current methodology findings: scalar probe

Vivado 2021.1 text `report_methodology`, `report_drc` and `check_timing -file`
were actually run. The installed skill describes a newer JSON workflow;
unsupported release-specific syntax was not assumed to work.

| Check | Count | Evidence / status |
|---|---:|---|
| Synthesis critical warnings | 0 | Scalar probe synthesis log |
| LUTLP-1 | 0 | `combinational_loops.rpt` |
| check_timing combinational / latch loops | 0 / 0 | `check_timing.rpt` |
| TIMING-16 large setup violation | 60 | Open: FP32 combinational arithmetic requires pipelining |
| TIMING-18 missing I/O delays | 96 | Open OOC boundary: no board timing contract is asserted |

The worst scalar path is `b_reg_reg[23]/C` to `result_reg[0]/R`, WNS
**-5.604 ns**, TNS **-310.201 ns**, 32 logic levels, and estimated unplaced
data-path delay **7.995 ns**. Both registers use the same `probe_clk` at
2.500 ns; clock uncertainty is 0.035 ns. This is a data-dependent synchronous
reset pin inferred by synthesis, not an asynchronous reset path to exclude.
No false paths, multicycle paths or timing-disable waivers were introduced.
Unplaced delays do not establish physical Fmax or FPGA latency.

## Remaining implementation work

1. Pipeline the exact FP32 RNE addition contract; maintain II=1 and sixteen
   butterfly pairs per issue cycle. Check arbitrary finite bit patterns and
   corner cases against the existing combinational function.
2. Delay bank row/half and final-group metadata with results. Drain a complete
   transform stage before reading its updated values for the next stage.
3. Repeat scalar and full-Hadamard synthesis/methodology/timing checks, then
   place and route. Preserve a synthesized DCP before timing reporting.
4. Continue full compute bridge, MIG, embedding/context/latent feedback,
   automatic DiCache estimation, complete inference and bitstream integration.

The former 3.5-GB guard also stopped normal timing analysis: the small probe
itself reports approximately 3.7 GB peak. Future protected runs need a budget
compatible with timing initialization and an explicit global commit reserve;
raising the threshold requires observing available memory, not changing the
system pagefile or terminating unrelated applications.

Command syntax reference: AMD
[UG835 check_timing](https://docs.amd.com/api/khub/documents/0UEc0Pr7HVthl2nugR8iNA/content).
The RTL repair is a local candidate validated at the scopes above; no matching
Answer Record workaround or full-system timing closure is claimed.
