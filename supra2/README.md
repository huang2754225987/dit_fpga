# Supra2-IMG Hardware Slice (256 Tokens, CFG Batch 512)

This directory defines the model-specific hardware boundary. It is a design
contract, not yet a synthesizable top or a source list for a bitstream. The
generic Qwen/MoE top and model-inapplicable Top-K/expert/low-bit operators
have been removed from this isolated copy. DMA, memory, and controller
primitives remain as integration candidates; no Supra2 system top exists yet.

## Workload contract (from `supra2-img-hw-export.tar.gz`)

- 32x32 systolic array; 100 quantized QLinear layers use INT8 activations and
  INT8 weights, while 37 passthrough tensors are stored as FP16.
- Latent input: 256 tokens x 576 features (4x32x32 latent, 2x2 patches).
- 14 DiT blocks, width 576, 9 heads x 64.
- Text context: 128 x 768, projected to width 576.
- MLP expansion: 576 -> 2304 -> 576; GELU(tanh).
- Sampling: 50 Euler-flow steps, CFG=3.0. Batch cond/uncond together as M=512
  so each projection streams weights once; cache cross-attention K/V per prompt.
- Quantization: `x_rot=x@H`, per-token symmetric INT8 activation quantization,
  per-output-channel symmetric INT8 weights, INT32 accumulation, then FP32
  `acc*s_t*weight_scale+bias`. `H` is a block-diagonal Sylvester Hadamard with
  `h_block` in the manifest. The exported INT8 weights are `W@H`.
- Skip trace covers 50x14 block decisions and skips 40.857% of block calls.
  The package supplies precomputed inputs and golden outputs; no Transformer
  forward is required to run the FPGA verification path.
- T5 and VAE are outside the DiT FPGA boundary in the supplied package flow:
  host supplies `ctx_all`, and host decodes the final latent.

## Keep / replace / remove boundary

| Area | Model-specific decision |
| --- | --- |
| 32x32 systolic array, INT8 MAC, INT32 accumulator | Keep; golden K=576 and K=2304 array tiles pass exactly |
| FP16 MAC/GEMM path | Keep for timestep, AdaLN, position embedding, and final passthrough tensors |
| AXI DMA, L1/tile storage, fetch, scoreboard, loops | Keep and specialize for 512-row CFG batches and cached K/V |
| Online softmax, vector ADD/MUL, SiLU | Keep for self/cross-attention, residual/AdaLN, timestep MLP |
| Block Hadamard transform | Keep new `hadamard_transform.sv`; H=64 QKV and H=256 MLP.2 full-batch frontend tests pass |
| Dynamic absmax INT8 quantizer | Keep `supra2_activation_quantizer.sv`; verified standalone |
| INT8-to-FP32 scale/bias epilogue | Keep `qlinear_epilogue.sv`; verified in model-specific projection tests |
| FP16 LayerNorm, AdaLN and GELU(tanh) | Keep the model-specific operators and golden regressions |
| Top-K, expert descriptor/precision tables, route-group logic | Removed from this isolated copy |
| INT1/2/3 low-bit GEMM, dequantization and scale/zero loader | Removed; keep the signed INT8 MAC path |
| RoPE and RMSNorm engines | Not model operators; removed from this copy |
| Generic `rtl/top.sv` | Removed from this copy; a dedicated Supra2 top and decode/schedule path are still required |
| Flan-T5 and SD-VAE | Host-side in the supplied verification flow |

The removed generic top coupled model-inapplicable operators to one hierarchy.
The current model operator filelist is the operator keep-set, while DMA,
SRAM/L1, fetch, scoreboard, loop, and address-generation RTL remains as
infrastructure for the future top. Unused generic operators and MoE/low-bit
datapaths are not part of the model-only source closure.

### Model-only pruning decision

`rtl/supra2/supra2_model_operators.f` is the consolidated allowlist for the
currently implemented model operators. It excludes testbenches, the generic
`rtl/top.sv`, MoE/Top-K routing, low-bit dequantization, RoPE, RMSNorm, and
generic controllers. Timestep SiLU remains in the operator list; GELU(tanh)
is separately implemented for the DiT MLP. The archive-driven `x_embed` path now has a dedicated
FP16 position-add operator and fixture boundary so its RTL output can feed
block 0 instead of substituting archive `b0.x`. Per-operator `.f` files remain the regression entry
points. This list is not yet a complete FPGA top-level list: no Supra2
command/DMA top exists, so generic DMA, SRAM/L1, fetch, scoreboard, loop, and
address-generation files remain integration candidates until transitive use
can be resolved. Close the dedicated top's own `.f` list before synthesis.

For this model, prioritize implementation work in this order:

1. Extend the projection/scheduler path from the validated single-branch
   M=256 fixtures to logical CFG M=512, while keeping physical array tiles at
   32 rows. Fuse the scale/bias epilogue on accumulator drain so the L1 output
   footprint uses FP16 rather than spilled INT32.
2. Reuse each N/K weight tile over as many M tiles as on-chip capacity permits;
   if residency does not fit, ping-pong the next 1-KiB K tile with compute.
   Measure actual DDR bytes and array stalls before claiming a speedup.
3. Preserve the already-tested fused online-softmax/PV path and packed valid
   cross-attention prefix; add the model's 50x14 skip schedule and cross-K/V
   cache control rather than materializing score/probability matrices.
4. Only after the command/DMA path is connected, profile the compute roofline
   and synthesis resource/timing reports to decide whether array replication
   or a larger array is needed. The package's 1.54-TMAC estimate implies about
   1.504 billion ideal issue cycles for one 32x32 array (about 7.52 s at
   200 MHz), so CFG batching and traffic reduction alone cannot support the
   package's 308-ms estimate on a single such array.

These are architectural priorities, not verified hardware performance. The
current cycle results remain independent ModelSim operator runs; no complete
Supra2 top, target-part synthesis, or board timing has been verified.

For the dedicated Supra2 synthesis closure, use this candidate source boundary
(resolve transitive dependencies from the new top before creating the final
`.f` file):

- **Retain/reuse:** `systolic_array.sv`, `pe.sv`, `mac_int8.sv` only for the
  production multiply-accumulate array; `mac_fp16.sv` is excluded from the
  production source lists (only its standalone diagnostic remains). Other
  reusable candidates: `gemm_act_rf.sv`, `gemm_weight_rf.sv`, `acc_buffer.sv`,
  `postproc_rf.sv`, `l1_cache.sv`, `tile_buffer.sv`; AXI read/write DMA;
  address generation, fetch, scoreboard, barriers, and loop control; online
  `softmax_engine.sv` and the necessary vector primitives.
- **Add/replace for this model:** a Supra2 command/top module; direct signed
  INT8 activation/weight load and INT32 accumulation control; block Hadamard
  rotation and per-token absmax quantization; FP32 scale/bias epilogue; FP16
  LayerNorm/AdaLN/GELU(tanh) operators; 512-row CFG schedule and K/V cache
  control. The existing generic `gemm_ctrl.sv` is not evidence of the direct
  quantized path because its integer mode dequantizes weights to FP16.
- **Excluded from this copy:**
  `topk_unit.sv`, `expert_desc_table.sv`, `expert_prec_table.sv`,
  expert route/group logic, low-bit scale/zero loaders and `dequant_unit.sv`,
  `rope_engine.sv`, and `rmsnorm_engine.sv`. Retain `silu_engine.sv` and its
  FP16 LUT for the timestep MLP; it does not replace the DiT's GELU(tanh).
  Generic DMA/memory/controller source remains because it may be reused by
  the dedicated top; resolve exact dependencies before making its final list.

The pass-through package tensors still require an FP16 linear path unless the
host partition explicitly takes over those operations. LayerNorm is not
RMSNorm, and GELU(tanh) is not SiLU; substituting the existing engines would
change model semantics.

## Keep set for the current model-specific RTL slice

Use `supra2_model_operators.f` as the consolidated model-operator RTL
allowlist, or the smaller per-operator `.f` files for focused regressions. It
intentionally contains no testbench and does not claim to elaborate a complete
model top. Do not include generic `rtl/top.sv` in a Supra2 build: it selects
the old MoE/low-bit design and does not instantiate this INT8 QLinear path.

A complete denoiser top must additionally close over AXI DMA/L1 storage, FP16
projection support, online softmax, vector arithmetic, LayerNorm/AdaLN/GELU,
and schedule/control logic. The current unit lists do not claim that this full
top exists.

## Verified slices

The earlier FP16 `x_embed` smoke is a bring-up baseline only; it uses the
separate local checkpoint, not the archive's quantized `x_embed.qw`, and is
not representative of the target QLinear path. It emits a 256-token,
M=32-batched `x_embed` program using the checkpoint's real FP16 weight and
bias. The token activations are deterministic synthetic values (the complete
denoiser is not run). The generic NPU top and FakeDDR execute 144 GEMMs and
write all 147,456 output elements. Numerical comparison against the matching
single-layer reference passes with max absolute error 0.001953125 and RMSE
0.0001022. This is RTL simulation only: FakeDDR does not model MIG timing and
there is no bitstream or board result.

The first targeted bandwidth optimization enables a compact 2-KiB static
FP16 DMA for a single 2-KiB K tile. The writeback path also emits aligned
4-KiB linear bursts and places one completion fence after each batch, instead
of 288 individual 1-KiB commands/fences. The pre-optimization biased run
transferred 659,456 read bytes in 289,389 modeled cycles. The optimized run
transferred 364,544 read bytes in 241,805 modeled cycles (44.7% fewer read
bytes, 16.4% fewer modeled cycles), with write commands reduced from 288 to 72
and ucode from 19,668 to 19,172 instructions. The 4-KiB command is internally
split by the 128-bit FakeDDR AXI configuration. These are FakeDDR profile
deltas, not predicted board speedups. The key next optimization is weight residency:
the same x_embed weights are still fetched once per each of eight token
batches. A dedicated top/DMA destination map should retain the 36-KiB packed
weight matrix across all batches before more aggressive array redesign.

The archive-derived INT8 test exports one complete K vector and a 32x32
output-channel slice from each supplied unit golden. `tb_supra2_int8_golden.sv`
passes exact INT32 comparison for `blocks.0.self_attn.qkv` (K=576) and
`blocks.0.mlp.2` (K=2304). This isolates the signed INT8 systolic datapath; it
does not validate Hadamard/quantization or DMA/RF loading.

The standalone `supra2_qlinear_epilogue` implements INT32-to-FP32 conversion,
FP32 scale multiplication/addition, and FP16 output conversion. Its 32-lane
golden test passes for QKV and MLP.2 tiles. The measured maximum absolute /
full-tensor-relative errors are 0.002441406 / 0.000172843 and 0.125 / 0.002232143,
respectively, both below the 0.0045 relative-error gate. These values compare
only selected 32x32 tiles to package `y_out`; they are not full-layer hardware
results and do not cover DMA/RF loading or the activation front end.
The current direct-feed test schedule takes 643 array input/drain cycles for
K=576 and 2,371 for K=2,304. These exclude DMA, Hadamard/quantization, output
stores, and the full token/channel tile schedule; they are array-feed counts,
not end-to-end image latency or FPGA timing.

The new streaming `supra2_activation_quantizer` accepts one FP16 rotated token,
buffers K values for the absmax pass, computes a shared Q40 reciprocal, then
emits 32 signed INT8 values per beat. The QKV K=576 and MLP.2 K=2304 tests
match every quantized value from their stored FP16 `x_rot` inputs, including
the MLP ties-to-even boundary case. The FP32 scale output is widened from an
FP16 multiply by 1/127; measured relative scale errors are 0.0276% and 0.0166%
for those tokens. The RTL test verifies the scale against `max(abs(x_rot))/127`
within 0.1%. The new `supra2_hadamard_transform` applies the manifest's
block-diagonal Sylvester butterfly for H=16/64/256, processes 16 pairs per
cycle, and normalizes by exact powers of two for the shipped block sizes. It
widens FP16 inputs to FP32, rounds each butterfly stage in FP32, then rounds
the normalized result to FP16 including IEEE subnormals. RTL output matches
the FP32-stage software fixture bit-for-bit for QKV K=576/H=64 and MLP.2
K=2304/H=256. Against stored package `x_rot`, the maximum full-vector
relative errors remain 0.3397% and 0.3655%; the package tensors were computed
before source tensors were rounded to FP16.

The combined token fixture connects raw FP16 input -> Hadamard -> dynamic
INT8 quantizer -> signed-INT8 array -> FP32 scale/bias epilogue. Four real
archive token/channel tiles pass, with exact equality to the generated
FP32-stage/FP32-epilogue reference: QKV K=576/H=64 takes 811 testbench cycles;
MLP.2 K=2304/H=256 takes 3,169 cycles. Against package `y_out`, the relative
maximum errors are 0.0560% (QKV row 0), 0.0235% (QKV row 64), 0.1674% (MLP.2
row 24), and 0.0593% (MLP.2 row 128). They bypass DMA/DDR and cover one token
and 32 output channels, not a full projection, block, image latency, or board.

The full QKV array/epilogue run covers M=256, N=1728, K=576 in 280,800
testbench cycles across 432 array tiles. Every INT32 output matches the archive
exactly. The RTL epilogue differs from the reconstructed FP32 software chain
at 142/442,368 FP16 encodings, with only 4.29e-6 maximum full-output-relative
error; the archive `y_out` differs from that FP32 formula at 418,563 encodings
and 0.3319% maximum relative error. This full-projection fixture uses package
`x_int8`/`s_t` directly, so it validates all array tiles and epilogue outputs,
not the new Hadamard/quantizer across all 256 rows. It also has no DDR/DMA.

The full QKV projection test now traverses all 256 archive rows and all 1,728
output channels using 32x32 array tiles (8 row tiles x 54 channel tiles).
`tb_supra2_qlinear_projection.sv` checks every INT32 accumulator and runs the
FP32 scale/bias epilogue for every output element. Besides the direct
package-`x_int8` test, frontend mode starts from all 256 stored `x_raw` rows,
runs RTL Hadamard and dynamic quantization, then executes the complete QKV
projection. QKV (M=256,N=1728,K=576,H=64) has 0 Hadamard mismatches, 0 INT8
mismatches, and 0 software-chain mismatches in 322,784 testbench cycles;
maximum relative error against archive `y_out` is 0.3872% under the 0.45%
gate. The same full-row test for MLP.2 (M=256,N=576,K=2304,H=256) passes in
545,696 cycles at 0.3906% maximum relative error, also with zero front-end or
software-chain mismatches. QKV and MLP.2 have 429,815 and 142,197 FP16
bit-pattern differences against archive outputs because archive goldens were
generated before storage rounding. Both runs still bypass DMA, DDR, model
control, and the rest of the DiT; they are not image latency.

Reproduce the x_embed, QKV, and MLP.2 projection runs (fixture generation,
RTL compile, ModelSim checks, and per-case logs) with:

```powershell
.\scripts\run_qlinear_frontend_regression.ps1
```

The runner requires the configured ModelSim installation/license and writes
fixtures/logs under `build/`; it does not run a Transformer or require a GPU.

The 50-step Euler update is also implemented as `euler_update.sv`. It replays
the archive's precomputed `v_per_step` on RTL: 50 x 4,096 elements in 6,400
stream cycles, exact to the FP16 software replay and relL2=0.1137% against
archive `z_final`. This is a sampler datapath check with golden velocities,
not end-to-end denoiser inference. Reproduce it with:

```powershell
.\scripts\run_euler_replay.ps1
```

```powershell
python -m compiler.supra2.prepare_qlinear_projection
vlog -sv -work build/modelsim/work -f rtl/supra2/qlinear_projection.f
$env:LM_LICENSE_FILE = 'D:\soft\modelsim\LICENSE.TXT'
$env:MGLS_LICENSE_FILE = 'D:\soft\modelsim\LICENSE.TXT'
vsim -c -work build/modelsim/work tb_supra2_qlinear_projection `
  +M=256 +N=1728 +K=576 `
  +ACTIVATION_FILE=build/supra2_qkv_projection/activation_int8.hex `
  +WEIGHT_FILE=build/supra2_qkv_projection/weight_int8.hex `
  +ACC_FILE=build/supra2_qkv_projection/accumulator_int32.hex `
  +OUTPUT_FILE=build/supra2_qkv_projection/output_fp16.hex `
  +TOKEN_SCALE_FILE=build/supra2_qkv_projection/token_scale_fp32.hex `
  +WEIGHT_SCALE_FILE=build/supra2_qkv_projection/weight_scale_fp32.hex `
  +BIAS_FILE=build/supra2_qkv_projection/bias_fp32.hex `
  +REFERENCE_OUTPUT_FILE=build/supra2_qkv_projection/reference_output_fp16.hex `
  -do "run -all; quit -f"
```

The Hadamard module has its own archive-row unit test. Generate the maximum
block-size MLP fixture (the QKV H=64 fixture is the default), compile
`hadamard.f`, and run:

```powershell
python -m compiler.supra2.prepare_hadamard --layer blocks.0.mlp.2 `
  --row 24 --output-dir build/supra2_hadamard_mlp24
vlog -sv -work build/modelsim/work -f rtl/supra2/hadamard.f
vsim -c -work build/modelsim/work tb_supra2_hadamard `
  +K=2304 +H=256 `
  +INPUT_FILE=build/supra2_hadamard_mlp24/input_raw_fp16.hex `
  +EXPECTED_FILE=build/supra2_hadamard_mlp24/expected_rotated_fp16.hex `
  +PACKAGE_FILE=build/supra2_hadamard_mlp24/package_rotated_fp16.hex `
  -do "run -all; quit -f"
```

```powershell
python -m compiler.supra2.prepare_activation_quantizer `
  --output-dir build/supra2_quantizer_qkv_row0
python -m compiler.supra2.prepare_activation_quantizer `
  --layer blocks.0.mlp.2 --row 24 `
  --output-dir build/supra2_quantizer_mlp_row24
vlog -sv -work build/modelsim/work -f rtl/supra2/activation_quantizer.f
vsim -c -work build/modelsim/work tb_supra2_activation_quantizer `
  +K=576 `
  +INPUT_FILE=build/supra2_quantizer_qkv_row0/input_fp16.hex `
  +EXPECTED_FILE=build/supra2_quantizer_qkv_row0/expected_int8.hex `
  +SCALE_FILE=build/supra2_quantizer_qkv_row0/expected_scale_fp32.hex `
  -do "run -all; quit -f"
```

Generate and run the epilogue fixtures after compiling its file list:

```powershell
python -m compiler.supra2.prepare_qlinear_epilogue
python -m compiler.supra2.prepare_qlinear_epilogue --layer blocks.0.mlp.2 `
  --output-dir build/supra2_epilogue_mlp
vlog -sv -work build/modelsim/work -f rtl/supra2/qlinear_epilogue.f
vsim -c -work build/modelsim/work tb_supra2_qlinear_epilogue `
  +ACC_FILE=build/supra2_epilogue_golden/acc_int32.hex `
  +TOKEN_SCALE_FILE=build/supra2_epilogue_golden/token_scale_fp32.hex `
  +WEIGHT_SCALE_FILE=build/supra2_epilogue_golden/weight_scale_fp32.hex `
  +BIAS_FILE=build/supra2_epilogue_golden/bias_fp32.hex `
  +EXPECTED_FILE=build/supra2_epilogue_golden/expected_fp16.hex `
  +REFERENCE_ABSMAX_FILE=build/supra2_epilogue_golden/reference_absmax_fp32.hex `
  -do "run -all; quit -f"
```

The minimal unit source closure is listed in `int8_array_golden.f`. From this
directory, compile it into a ModelSim work library, generate fixtures, then
run the two K shapes:

```powershell
python -m compiler.supra2.prepare_int8_golden_tile
vlog -sv -work build/modelsim/work -f rtl/supra2/int8_array_golden.f
vsim -c -work build/modelsim/work tb_supra2_int8_golden `
  +K=576 `
  +X_FILE=build/supra2_int8_golden_tile/activation_int8.hex `
  +W_FILE=build/supra2_int8_golden_tile/weight_int8.hex `
  +GOLD_FILE=build/supra2_int8_golden_tile/accumulator_int32.hex `
  +TOKEN_SCALE_FILE=build/supra2_int8_golden_tile/token_scale_fp32.hex `
  +WEIGHT_SCALE_FILE=build/supra2_int8_golden_tile/weight_scale_fp32.hex `
  +BIAS_FILE=build/supra2_int8_golden_tile/bias_fp32.hex `
  +OUTPUT_FILE=build/supra2_int8_golden_tile/output_fp16.hex `
  +OUTPUT_ABSMAX_FILE=build/supra2_int8_golden_tile/output_absmax_fp32.hex `
  -do "run -all; quit -f"
python -m compiler.supra2.prepare_int8_golden_tile `
  --layer blocks.0.mlp.2 --output-dir build/supra2_int8_golden_mlp
vsim -c -work build/modelsim/work tb_supra2_int8_golden `
  +K=2304 `
  +X_FILE=build/supra2_int8_golden_mlp/activation_int8.hex `
  +W_FILE=build/supra2_int8_golden_mlp/weight_int8.hex `
  +GOLD_FILE=build/supra2_int8_golden_mlp/accumulator_int32.hex `
  +TOKEN_SCALE_FILE=build/supra2_int8_golden_mlp/token_scale_fp32.hex `
  +WEIGHT_SCALE_FILE=build/supra2_int8_golden_mlp/weight_scale_fp32.hex `
  +BIAS_FILE=build/supra2_int8_golden_mlp/bias_fp32.hex `
  +OUTPUT_FILE=build/supra2_int8_golden_mlp/output_fp16.hex `
  +OUTPUT_ABSMAX_FILE=build/supra2_int8_golden_mlp/output_absmax_fp32.hex `
  -do "run -all; quit -f"
```

## 256-token projection tiling

The denoiser has 256 latent tokens, but conditional and unconditional rows
should be concatenated into one logical M=512 batch. The array consumes at
most 32 rows per launch. Assuming a 64-KiB output page stores INT32 partial
sums, each row costs `align_up(N,32) * 4` bytes:

| Projection output width | Padded INT32 row bytes | Rows per batch | Batches for M=512 |
| ---: | ---: | ---: | ---: |
| 576 | 2,304 | 16 | 32 |
| 1,728 (self-attn QKV) | 6,912 | 8 | 64 |
| 1,536 (cross-attention KV) | 6,144 | 8 | 64 |
| 2,304 (MLP expansion) | 9,216 | 4 | 128 |

These are conservative single-page limits; double buffering and other live
tensors can reduce them. If the INT32 accumulator remains inside the array and
the scale/bias epilogue is fused before writing FP16 output, the output page
can hold twice as many rows (32/16/16/8), or 16/32/32/64 batches respectively.
The compiler must track accumulator lifetime separately from final output
storage; treating both as one L1 allocation either wastes capacity or loses
exact accumulation across K tiles.

A 32-row array cannot consume all 512 CFG rows at once. Unless the full layer
matrix fits on chip, a streamed weight tile must be fetched again for later
row batches. Prioritize weight-tile residency across M batches where SRAM
allows, then double-buffer the next 1-KiB tile against compute, and reuse
activation tiles across N tiles. Do not assume CFG batching alone guarantees
one physical weight read per projection.

The export README estimates 1.54 TMAC per image including the measured
0.58x block-cache compute ratio, with 3.0 GB INT8 weight traffic after CFG
batching. It estimates 308 ms at 10 TOPS and a 159 ms DDR4 lower bound at
19 GB/s. These package estimates are not measurements on this FPGA design;
this copy has no synthesis/board timing result yet.

### Recommended first specialization for 256 tokens

For a latent-token projection, use an M=32 activation batch and stream output
one N=32 tile at a time. A full FP16 output batch for QKV is
32 x 1,728 x 2 = 108 KiB, larger than the current 64-KiB L1 page; one output
tile is only 32 x 32 x 2 = 2 KiB. This avoids shrinking the row batch merely
to retain the entire projection output in L1. It gives eight row batches for
256 tokens, or sixteen for the combined CFG=512 rows, while retaining the
existing 32x32 array shape. Keep the INT32 accumulator tile local (4 KiB) and
apply the FP32 scale/bias epilogue as each N tile drains, writing final FP16
values directly to their DDR destination.

The next optimization is weight reuse across those row batches: for one
N=32 tile, QKV has K=576, or 18 packed 1-KiB INT8 weight tiles (18 KiB total).
If an explicit BRAM/L1 bank can retain this 18-KiB working set, loop over the
eight M batches before evicting that N tile. This can reduce that tile's weight
reads by up to 8x; do not assume it is possible until synthesis confirms the
storage and a bank/lifetime map proves it does not alias activation or output
buffers. If it does not fit, use ping-pong 1-KiB K-tile buffers and overlap
AXI fetch of the next K tile with the array, then measure whether DMA stalls
actually fall.

For M=256, each projection has eight 32-row batches; with CFG M=512 it has
sixteen. A 32-output-channel INT8 weight tile occupies `K*32` bytes: 18 KiB
for QKV/MLP.0 (K=576), 24 KiB for cross-KV (K=768), and 72 KiB for MLP.2
(K=2304). The first two are plausible whole-tile residency candidates in a
64-KiB bank, subject to activation/output buffers and physical BRAM mapping.
MLP.2 cannot fit as one whole tile in that bank; use two 1-KiB K-tile buffers
to hide fetch latency, and do not claim cross-row weight reuse unless a larger
bank or a measured cache is implemented. Keep the 32x32 array unchanged until
the target FPGA part and post-route resource/timing reports are known.

The package-activation QKV direct-feed test is 280,800 testbench cycles for
432 32x32 output/row tiles and has exact INT32 accumulators. The full raw-input
QKV test, including serial Hadamard/quantization, is 322,784 cycles; the
frontend adds 41,984 cycles (164 per token) in this testbench schedule. The
full MLP.2 path is 545,696 cycles. These tests have no weight DMA or DDR
writeback, and do not represent deployment latency. The measured frontend
overhead makes token-level front-end/array overlap a candidate optimization,
but it should follow construction of the DMA-connected top so its benefit is
measured in the real schedule.

### Compute roofline before architectural tuning

The package's 1.54 TMAC/image estimate and a 32x32 array imply at least
`1.54e12 / 1024 = 1.504e9` array issue cycles at perfect utilization. At
200 MHz this is about 7.52 seconds of MAC time, before quantization,
normalization, attention, DMA stalls, or the 50-step schedule. The package's
308 ms estimate assumes 10 TOPS, roughly 25 times the ideal MAC rate of one
32x32 array at 200 MHz (counting one MAC as two ops). CFG batching and traffic
reduction alone therefore cannot deliver that estimate on a single array.
Check target clock and DSP budget first, then evaluate parallel arrays or a
larger array against timing and memory bandwidth.

Recommended specialization sequence:

1. Freeze the numerical contract: FP16 input -> block Hadamard -> per-row
   INT8 quantization -> INT8 GEMM/INT32 accumulation -> FP32 scale/bias -> FP16.
2. Keep CFG rows together (M=512), but pipeline 32-row array tiles. Reuse
   each weight tile across M tiles as far as on-chip storage permits; only
   then add double buffering with an explicit bank/lifetime map.
3. Fuse QLinear epilogue into accumulator drain to avoid spilling INT32 tiles.
4. Add Hadamard and model-correct LayerNorm/AdaLN/GELU, validating each
   operator against unit/block goldens before attention scheduling work.
5. Measure utilization, AXI bytes/stalls, DSP/BRAM/LUT, and post-route Fmax.
   Choose array replication/tiling from these reports, not unit-test cycles.

## Pipelined block schedule and utilization plan (no-skip baseline)

Baseline: one CFG=3.0 step without skipping executes all 14 blocks at M=512
(2 branches x 256 tokens). Summing the measured operator busy cycles gives
9,767,212 cycles per block, split as follows:

| Stage group | Cycles/block (M=512) | Share | Hardware |
| --- | ---: | ---: | --- |
| Self-attn online softmax->PV | 1,845,648 | 18.9% | array, ~8% util |
| 3x LayerNorm/AdaLN | 1,771,008 | 18.1% | none (2 cyc/elem stream) |
| GELU(tanh) | 1,179,648 | 12.1% | none (1 cyc/elem LUT stream) |
| MLP.2 (K=2304) | 1,091,392 | 11.2% | array |
| 3x residual/gate add | 884,736 | 9.1% | none (1 cyc/elem stream) |
| MLP.0 (N=2304) | 832,768 | 8.5% | array |
| QKV (N=1728) | 645,568 | 6.6% | array, ~77% util |
| 3x output/Q projections (N=576,K=576) | 813,504 | 8.3% | array |
| Cross-attn online PV | 509,040 | 5.2% | array, ~4% util |
| Self QK | 155,528 | 1.6% | array, ~47% util |
| Cross KV + QK (cached ctx) | 38,372 | 0.4% | array |

Only ~36% of block time is dense INT8 GEMM at reasonable utilization; the
rest is either array-idle streaming (39%) or attention passes whose online
rescaling serializes the array (24%). The 32x32 array stays unchanged; the
plan below adds small streaming units and restructures attention so the
block becomes array-bound.

### Row-tile pipeline schedule

Everything is scheduled at 32-row tile granularity (16 tiles per CFG batch;
branch 0 and branch 1 are independent residual streams until the final head,
so their tiles interleave freely and fill each other's bubbles):

```text
per row tile t (branches interleaved):
  LN1+AdaLN[t]        (stream unit A)  -> QKV[t]           (array)
attention (per branch, after its K/V complete):
  for query tile q: QK pass A over 8 key tiles (array)
                    softmax over 256 keys     (softmax engine)
                    PV pass B as dense GEMM   (array)
  concat -> self-proj[t] (array) -> gated residual[t] (stream unit R1)
  LN2+AdaLN[t] (unit B) -> cross-Q[t] (array)
  cross QK + softmax over 32 cached keys (engine) -> cross PV (array)
  -> cross-proj[t] (array) -> residual[t] (unit R2)
  LN3+AdaLN[t] (unit C) -> MLP0[t] (array) -> GELU (fused epilogue)
  -> MLP2[t] (array) -> gated residual[t] (unit R1)
```

Producer-consumer rates per 32-row tile (M=512): QKV GEMM produces a
32x576 activation tile every ~40.3k cycles; the LN feeding it needs 36.9k
cycles at the current 2 cyc/elem, so even the unmodified LN unit can hide
under the QKV GEMM if it is not shared serially. MLP0 needs ~52k cycles/tile;
GELU at 1 cyc/elem needs 73.7k cycles/tile -- 32% slower than its consumer,
so GELU must either go 32-wide or fuse into the MLP0 epilogue. The three
residuals (18.4k each at 1 cyc/elem) fit under any following GEMM.

### Rate mismatches and added units (array untouched)

1. **Self-attention PV (biggest win, 1.85M -> ~0.35M cycles/block).** The
   online-softmax wrapper rescales (m, l, acc) after every 32-key tile, so
   the array stalls between key tiles (~4% util). Since K=256 per branch is
   small, buffer the scores per 32-query tile (32x256 FP16 = 16 KiB), then:
   pass A = QK for all 8 key tiles back-to-back (array stays busy, QK-style
   scheduling); softmax on a 32-lane vector unit (max-tree, exp LUT,
   sum-tree, reciprocal-multiply); pass B = PV as a dense 32-query GEMM.
   Adds: 2x16 KiB ping-pong score BRAM, one 32-lane softmax engine. The
   softmax engine is implemented and verified: `supra2_softmax_2pass`
   (rtl/supra2/supra2_softmax_2pass.sv) passes its XSim unit test on real
   block-7 QK scores at 2,401 cycles per 32x256 tile with 0.78%
   relative-to-peak error (1% gate), reusing the graph exp LUT plus 1/16-step
   interpolation and the recip LUT, so numerics match the online path. The
   same engine handles cross-attention (15 valid keys in one 32-key tile),
   taking cross PV from 509k to ~90k cycles. With the measured 2,401-cycle
   latency fed back into `scripts/simulate_block_pipeline.py`, the block
   makespan is unchanged (3,663,668 cycles) and SFT is 1.4% busy.
2. **LayerNorm/AdaLN (1.77M -> hidden).** Current unit runs 2 cyc/elem and is
   effectively shared across all three sites. Give each site its own unit,
   single-pass sum+sumsq (mean/variance) with a 32-lane FP16 tree, rsqrt,
   then scale/shift/gate lanes: 294,912 elements per site at M=512 finish in
   ~9.2k cycles, fully hidden under the preceding/succeeding GEMM. The 32-lane
   unit is implemented and verified: `layernorm_modulate_32lane`
   (rtl/supra2/layernorm_modulate_32lane.sv) passes its XSim unit test on a
   block-7 self-norm slice at 596 cycles per 32x576 tile with
   2.6e-05 relative-to-peak error (1% gate), using the identical FP32
   sum/sumsq, clamped variance, 3-round rsqrt and FP16 modulation chain as the
   row-sequential unit, so numerics match bit-for-bit per feature. With the
   measured 596-cycle latency fed back into `scripts/simulate_block_pipeline.py`
   the block makespan is unchanged (3,663,668 cycles) and LN_A/B/C stay under
   1% busy.
3. **GELU (1.18M -> hidden).** Fuse the tanh LUT into the MLP0 accumulator
   drain (evaluate on the FP32 pre-output, before FP16 store), or replicate
   the ROM 32-wide; either way it disappears under the MLP2 GEMM. The 32-lane
   unit is implemented and verified: `gelu_tanh_32lane`
   (rtl/supra2/gelu_tanh_32lane.sv) streams the whole 256x2304 block-7 MLP0
   output in 18,432 cycles (32 elem/cycle vs 589,824 for the scalar unit)
   with zero mismatches against the scalar unit's own output, since every
   lane replicates the verified address math and 4K-entry ROM bit-for-bit.
4. **Residuals (0.88M -> hidden).** Two 32-lane FP16 multiply-add units
   (one per gated-residual site; the ungated cross residual reuses either).
   The 32-lane unit is implemented and verified: `residual_gate_32lane`
   (rtl/supra2/residual_gate_32lane.sv) streams the 256x576 block-7
   gated residual in 4,608 cycles (vs 147,456 scalar) with zero mismatches
   against the verified scalar unit, taking the per-feature AdaLN gate beat
   alongside residual/update. With RES_TILE backfilled to the measured 576
   cycles in `scripts/simulate_block_pipeline.py`, R1/R2 stay under 1% busy
   and the block makespan holds at ~3.66M cycles.
5. **Weight streaming.** QKV/MLP.0 need an 18-KiB working set per N-tile,
   MLP.2 72 KiB; with 8 B/cycle DDR and ping-pong K-tile buffers the fetch
   (~124k cycles for QKV weights) hides under the 645k-cycle compute. Keep
   per-N-tile weight residency across the 16 row batches where a 64-KiB bank
   allows (QKV, MLP.0, projections yes; MLP.2 uses ping-pong only).

### Projected result (component estimates, not measurements)

| Metric | Current (serial sum) | Pipelined plan |
| --- | ---: | ---: |
| Cycles/block (M=512) | 9,767,212 | ~3.66M (schedule-simulated) |
| Array busy fraction | 60.7% | 99.97% |
| Array utilization (MAC/peak) | ~25.7% | ~68.5% |
| Step, no skip (200 MHz) | 688 ms | ~261 ms |
| Step, 40.857% skip trace | 409 ms | ~156 ms |
| Image (50 steps, with skip) | ~20.5 s | ~7.8 s |

The pipelined column comes from `scripts/simulate_block_pipeline.py`, a
list-scheduling event simulation at 32-row tile granularity whose stage
durations are anchored to the measured XSim counters (GEMM stages use the
measured per-tile busy cycles including the Hadamard/quantizer frontend, so
the model is conservative on that part). The same schedule is replayed in
RTL: `scripts/simulate_block_pipeline.py` exports `build/pipeline_sim/schedule_table.hex` (272 tasks, grouped per resource with start/duration/dependency-ready cycles) and `supra2_block_schedule_replay` (rtl/supra2/block_schedule_replay.sv) re-runs it in XSim as a clocked control-plane replay, checking dependency feasibility (start >= dep_ready), no double-booked resource, and re-deriving makespan and per-resource busy counts. The RTL replay reproduces the Python numbers exactly: makespan 3,663,668 cycles, ARRAY busy 3,662,496, LN_A/B/C 9,536 each, R1 18,432, R2 9,216, SFT 52,816, zero violations (SUPRA2_SCHEDULE_REPLAY_PASS; driver `build/run_sched_replay_driver.ps1`). In the simulated schedule the ARRAY
is busy 3,662,496 of 3,663,668 block cycles: every added streaming unit
(LN_A/B/C, R1/R2, SFT) sits under 1% busy, i.e. all producer-consumer rates
fit and the systolic array is the only wall. Per-block speedup 2.67x.

Current-hardware baseline statistics (serial sum of measured counters,
M=512, no skip): array busy 5,931,820 cycles = 60.7% of the 9,767,212-cycle
block; streaming units 39.3%; attention PV alone is 24.1% of the block and
runs at 4.0% (self) / 1.8% (cross) MAC utilization due to online-rescale
serialization. Overall array MAC utilization is ~25.7%, versus 2,569 GMAC
requiring 2,509,056 ideal issue cycles per block.

These remain sums of operator counters plus explicit overlap assumptions; no
connected top, synthesis, or board timing exists yet. The schedule assumes
the added streaming units and the score buffer land in the target part's
budget -- verify with synthesis before quoting utilization as achieved.

## Optimization order

1. **Build the quantized QLinear path first.** The removed generic top
   hardwired the systolic array to FP16 and routed integer formats through
   low-bit FP16 dequantization; the model needs signed INT8 x INT8 -> INT32. Verify the
   Hadamard, quantizer, array, and FP32 epilogue against the unit golden.
2. **Prune only through a dedicated top/source list.** Remove expert,
   low-bit quantization, RoPE, and RMSNorm logic from the new synthesis
   closure. Verify the resulting BRAM/DSP/LUT and timing report before
   deciding the array size.
3. **Batch CFG conditionals together.** Compile cond and uncond as one M=512
   logical batch. This avoids separate schedules but does not itself guarantee
   one physical weight read; row batching still applies.
4. **Keep exact INT32 partial sums and tile M/N/K independently.** Start with
   the conservative INT32-page limits above. Fuse the FP32 scale/bias epilogue
   before writing final FP16 output; only then can the output buffer use
   two-byte elements.
5. **Reduce repeated weight traffic.** The x_embed FakeDDR run confirms that
   the packed 36-KiB weight matrix is fetched for each of eight token batches.
   First evaluate L1 weight residency and a DMA destination map that can retain
   weights without aliasing activations or output. This should remove up to
   seven of eight weight reads for this layer. Changing loop order alone is
   unsafe because accumulator lifetime and L1 aliasing must be explicit.
6. **Fuse model-specific epilogues.** Add LayerNorm(mean/variance/rsqrt),
   AdaLN scale/shift/gate, and GELU(tanh) vector datapaths. This cuts DDR/L1
   round trips, but requires PyTorch-vs-RTL error tests before fusion is enabled.
7. **Tile both attention axes.** Use 32-query x 32-key online-softmax tiles;
   self-attention spans 256 keys and text cross-attention spans 128. Keep K/V
   tiles resident/reused across heads and query blocks where the SRAM budget
   allows; do not materialize a full 256x256 score matrix.

## Block-input LayerNorm/AdaLN boundary

`layernorm_modulate.sv` implements token-wise FP16 LayerNorm (epsilon `1e-6`)
and AdaLN scale/shift modulation. The 576-to-3456 timestep SiLU/linear is a
separate producer: the archive-backed regression precomputes its FP16 output
on CPU, while deployment should run that projection once per timestep and
conditioning group, then broadcast its six 576-wide parameter vectors across
the 256 tokens. This avoids repeating the timestep MLP for every token; its
FPGA scheduling/storage interface is not connected yet.

The LayerNorm accumulator updates sum and sum-of-squares while each x feature
is loaded, preserving the serial FP32 accumulation order and eliminating a
second 576-cycle read/reduction pass. On the archive's 256-row block-0 fixture,
the standalone RTL replay is 295,168 cycles versus 442,624 before this
optimization (33.3% fewer module cycles), with 0.00458% maximum relative error
against the CPU FP16 reference. Output differs from archived QKV input by
0.2189% maximum relative error, primarily due to archived tensor storage
rounding.

Feeding these RTL-produced rows into the complete QKV regression retains exact
Hadamard, INT8, and software-chain checks and passes the 0.45% package-output
gate at 0.4425% maximum relative error; the projection takes 322,784
testbench cycles. These are ModelSim results, not synthesized timing/resource
or board results.

Reproduce both stages with:

```powershell
.\scripts\run_layernorm_qkv_regression.ps1
```

The fixture uses archived `b0.x`, `b0.c`, block-0 AdaLN weights, and package
QKV goldens. It computes only the timestep MLP boundary and does not run a
Transformer/DiT forward or require a GPU.

### 256-key online-softmax boundary

The existing 32-lane `softmax_engine` is exercised as the online-softmax stage
for all 256 queries and 9 heads in block 0. It consumes eight 32-key score
tiles per query/head, updates online max/sum state, emits the tile scale factors,
and finalizes the reciprocal. The test reconstructs all 589,824 probabilities
and compares them with an FP32 softmax reference: 470,015 RTL busy cycles,
5.67e-5 maximum absolute error, and 6.86e-6 RMSE. In the chained regression,
Q/K inputs come from the RTL-produced QKV tensor, and QK dot products are
computed by the FP16 systolic array. The same chain runs PV GEMM, quantized
`self_attn.proj`, and the gated residual. QK covers all 256x9x256 scores in
77,764 cycles with 0.000977 maximum absolute score error; PV takes 47,092
cycles with 0.1934% maximum relative-to-peak error; the output projection
takes 135,584 cycles with exact INT32/software-chain agreement; the residual
takes 147,456 cycles at one feature per cycle with 3.76e-7 relative-to-peak
error.

`supra2_attention_online_pv.sv` joins online softmax and PV without
materializing the 256×256 probability matrix. It loads one head's 256×64 V
matrix once, accepts eight 32-key score tiles per query, and keeps the
64-element FP16 PV accumulator on chip. The full 256-query × 9-head RTL run
passes at 4,603,401 busy cycles, 0.0048828 maximum absolute error, and 0.2418%
relative-to-peak error. Its token-major RTL output feeds
`blocks.0.self_attn.proj` directly; that INT8 projection passes in 135,584
cycles with zero frontend and software-chain mismatches.

V storage is organized as 32 feature banks of 512×16 bits (32 KiB total).
Each systolic B lane reads its own bank; a one-cycle synchronous prefetch is
issued during clear/feed so the array still consumes one K element per cycle.
The ModelSim regression after this banking change retained the same numerical
result. This validates functional alignment, not BRAM inference or area
reduction; those require synthesis for the actual FPGA part.

The optimized `supra2_attention_online_pv_batch.sv` processes 32 queries as
one microbatch. It buffers one 32×32 raw-exp tile (2 KiB), uses the softmax
engine's 32 independent state slots, retains 32×64 FP16 output accumulators
(4 KiB), and runs PV with all 32 systolic rows active. It preserves per-query
online normalization and avoids a full probability matrix. Full 256-query ×
9-head simulation passes in 922,824 busy cycles with 0.0048828 maximum
absolute error and 0.2418% relative-to-peak error. This is 79.95% fewer cycles
than the single-row fused baseline at 4,603,401 cycles, and the two output
files are bit-identical. Reproduce with
`scripts/run_attention_online_pv_batch.ps1`; the one-click QKV/QK/batched-PV/
projection/residual sequence uses this variant.

These are ModelSim cycles, not board latency. Banking has functional evidence,
but BRAM inference, resource use, and timing still require synthesis for the
actual FPGA part. The next throughput target is the serialized per-row
softmax command loop; parallel softmax lanes or a pipelined row front-end need
an area/latency comparison before replication is selected.

### Cross-attention context projection boundary

The conditional text embedding from `golden_steps.ctx_all[0]` has shape
128×768. It passes through `ctx_proj` as an M=128, K=768, N=576 quantized
projection in 94,672 ModelSim cycles with zero Hadamard/INT8/INT32/software
chain mismatches. The RTL FP16 output differs from archived
`golden_block.b0.ctx` by 0.4941% relative to the reference peak (0.6% gate).
Reproduce this host-context-to-FPGA-projection boundary with
`scripts/run_context_projection.ps1`. Archive `mask_all[0]` has 15 valid tokens
in a 128-token padded context. The cross-attention runner preserves this mask,
packs the valid prefix into a 32-key tile, and projects only M=32 KV rows
(28,648 cycles). Q is M=256,N=576 (135,584 cycles); QK is 9,724 cycles and
online PV is 254,520 cycles with 0.487% relative-to-peak error. Compared with
the earlier unmasked 128-key run, QK decreases 75.0% and PV 52.9%. A runtime
9-bit `valid_keys` input (able to represent the full 256-key case) controls the number of softmax lanes included in each
online update; padded values do not enter the denominator or PV result.

The same regression continues through cross output projection/residual, MLP
LayerNorm/AdaLN, 576→2304 projection, a 4K-entry GELU(tanh) ROM, 2304→576
projection, and the gated residual. The final RTL tensor is compared elementwise
to archive `b0.y` (147,456 elements): max_abs=1.125, relative-to-peak=0.521%,
RMSE=0.02744, relative-L2=0.294%. These remain separate ModelSim module runs
connected by RTL output files; there is no command/RF/DMA block top or synthesis
result yet.

Self-attention, cross-attention, and MLP stages are separate ModelSim runs
bridged by actual RTL output files. Block-0 output now passes the archive
golden comparison at the stated tolerance, but there is not yet a DMA/RF
controller, a connected single-top block execution, or a synthesized design.

The same block chain has now been exercised with block 7's archived input,
block-specific weights/AdaLN parameters, and `b7.y` output golden. The final
147,456 elements pass at 0.3685% relative-to-peak error (0.3448% relative L2;
2% gate). Self QKV/QK/online-PV and cross Q/KV/QK/online-PV are all RTL-driven;
the MLP.0, GELU(tanh), MLP.2, and both residual boundaries also pass. The
QK testbench uses a 0.1% relative-to-peak gate because score magnitudes differ
by block; block-7 self/cross QK measured 0.0390%/0.0546%. Reproduce with
`scripts/run_layernorm_qkv_regression.ps1 -Block 7` and then
`scripts/run_cross_attention_regression.ps1 -Block 7`. These are still
separate ModelSim runs joined by output files, with timestep AdaLN computed
on the host; they do not prove execution of sequential blocks or CFG=512.
Since then, a separate connected single-block top-level simulation has also
passed on the block-13 fixture; see the INT8-only array update below.

## Remaining system integration gate

This copy intentionally contains model-relevant operators plus reusable
DMA/memory/control infrastructure. It has a connected block-compute
simulation top, but not a complete DMA/RF system top. Before bitstream work,
add the system register map, connect the compiler schedule and DDR traffic,
and run full system regressions. The systolic array and runtime-DMA/controller
block have target-part OOC synthesis/timing; the complete top still needs
synthesis and timing closure.

2026-10-23 update: `supra2_dit_compute_bridge.sv` now elaborates the generated
50-step controller, 512-bit descriptor DMA, tile-to-GEMM loader, and the
existing block compute top as one module graph. The controller time-multiplexes
the M=256 engine over conditional/unconditional branches; its self-checking
Questa test passes 50 steps, both branches, and 828 non-skipped block launches
at the control level. A separate smoke test only compiles/elaborates the
compute bridge; it does not execute a block with real DDR data. Context,
step/AdaLN preparation, final head, and MIG/board top-level integration remain
open. The bridge includes 32-lane CFG=3 arithmetic, an 8-KiB buffer for 128
velocity beats, and the 32-lane Euler updater for 4096 latent elements. The
controller waits for CFG to fill the buffer, starts Euler, and waits for the
final accepted output beat. Both branch velocity streams and the latent stream
remain external inputs, while updated latent is exposed for the next-step
loader. Questa covers the arithmetic and a reduced 64-element CFG-buffer-Euler
path with backpressure; full-size integration and the actual final-head
producer remain unverified.

The shared 512-bit AXI reader keeps its 64-byte data beat and now uses a
27-bit remaining-beat counter plus 7-bit burst/page selection, sized for the
full 2^26-beat 32-bit address space while avoiding a 32-bit burst-length
selection path. Questa covers 4-KiB splitting, 16- and 64-beat bursts,
maximum descriptor acceptance, invalid alignment, and downstream
backpressure. The runtime DMA/controller, including the generic QLinear tile
request path, was synthesized and placed/routed at a 2.500 ns clock (400 MHz):
the latest route has WNS +0.079 ns and TNS zero, with 1,024 CLB LUTs, 351
registers, 6 DSPs, and no BRAM. This is the user-selected DMA OOC target, not
a claim that 400 MHz is the design maximum. The OOC module excludes the
compute bridge and MIG. Its 727 input ports and 148 output ports have no
external delay constraints, and it has no board clock-tree constraint, so this
result establishes internal OOC timing only. The latest run is recorded in
`build/supra2_dit_runtime_dma/vivado_ooc_2p500ns_20261023_214352/`; repeat with
`scripts/run_supra2_dit_runtime_dma_ooc.ps1` (default 2.500 ns), which writes a
run-tagged report directory. The prior +0.143 ns result predates the generic
QLinear arbitration and is superseded.

Before the 400 MHz route check, the request path was shortened with a
registered descriptor-launch stage and continuously precomputed step/block
AdaLN offsets. The 50-step runtime-DMA regression, final-parameter DMA bridge
regression, and final-head CFG/DiCache regression all pass after these changes.
The MIG pin-model vector-DMA regression remains separately verified at
333.33 MHz; it does not combine the bridge with this 400 MHz OOC result.

The 512-bit `supra2_axi_read_dma` has been exercised through the
512-to-64-bit `axi_to_mig_bridge` and official ZU7EV MIG/DDR4 pin model. A
standalone 4-KiB bridge/DMA transaction read back 64 beats exactly in 541 UI
cycles at 333.33 MHz (2,523.72 MB/s). The package-driven runtime controller
and descriptor DMA regression reads a 16-beat weight tile, then a 16-beat
vector load, checking all data and destination tags. The latest pin-model run
passes in 7,871 UI cycles at 333.33 MHz, with 0 simulator errors and 3
warnings. The remaining compute handshakes are testbench models, so this is
not an inference latency or throughput figure. Full-volume runtime DMA, the
compute hierarchy, and a full 50-step RTL inference remain unverified.

2026-10-23 update: the generated DDR layout exports per-layer N/K tile counts,
byte strides, and component sizes/beats for all 100 QLinear layers.
`supra2_dit_qlinear_descriptor.sv` resolves aligned weight, scale, and bias
tile requests against those tables, including bounds checks. The resolver is
now integrated into `supra2_dit_compute_bridge`, and its generic request/data
stream is arbitrated by `supra2_dit_runtime_dma`. Questa passes the 100-layer
descriptor test, the 50-step/828-block runtime-DMA test with a 16-beat
`x_embed` tile under backpressure, and the bridge regression checking its
layer metadata, DDR address, beat count, and all 32 FP16 lanes across 16 beats.
The earlier Windows simulator launch failure is resolved. The model-specific
input-preparation scheduler is still external.

An OOC implementation runner has been added for the complete
`supra2_dit_compute_bridge` hierarchy at the selected 2.500 ns target. The
first Vivado 2021.1 attempt terminated during synthesis while processing
`supra2_gemm_engine` with `EXCEPTION_ACCESS_VIOLATION`; no synthesis or route
reports were produced. The engine currently declares simulation-sized local
activation and weight arrays (up to 256x2304 and 2304x2304), and the run log
shows synthesis entering this module, but Vivado supplied no stack trace, so
the crash cause is not established. The runtime-DMA 400 MHz OOC result must
not be generalized to the compute bridge. The new runner is
`scripts/run_supra2_dit_compute_bridge_ooc.ps1`; a synthesis-ready GEMM storage
architecture and successful bridge-level route remain open.

Follow-up synthesis isolation reproduced the failure in the standalone
2304-K Hadamard frontend, independently of DMA and full-matrix GEMM storage.
The block GEMM weight capacity now follows the largest actual QLinear product
(1,327,104 INT8 elements instead of 5,308,416). Hadamard retains sixteen
butterflies per cycle; its address generation uses shifts/masks and its bank
reads use two shared rows, with direct write-port routing. The three
representative first-row vectors (K/H 16/16, 576/64, 2304/256) pass bit-exact
Questa comparisons via `scripts/run_supra2_hadamard_questa.ps1`. These checks
do not establish full inference or synthesis completion. The latest isolated
synthesis of that version was deliberately stopped at 3.56 GB private memory
before reports were produced.

The current frontend now stores only `max(MAX_H, LANES)` FP32 elements: 256
instead of 2,304 at the largest model configuration. It processes and emits
each H block before loading the next, with two H=16 blocks packed into one
32-lane beat where available. The GEMM engine advances its asynchronous raw
read address only on a Hadamard input handshake. External streams remain
512 bits, the butterfly arithmetic remains FP32, and sixteen pairs can issue
per transform cycle. These changes introduce legal input stalls between
blocks; callers must honor `in_ready`.

Questa 2024.1 passes five Hadamard golden/backpressure shapes (K/H=16/16,
32/16, 48/16, 576/64, 2304/256), including distinct H=16 blocks, partial-lane
padding, output hold stability, and simultaneous input/output activity. A new
real-engine regression (`scripts/run_supra2_gemm_frontend_questa.ps1`) checks
all 1,024 outputs of each 32x32 x_embed/QKV/MLP.2 projection tile and the held
raw read boundary. All are bit-exact against the existing software-chain
fixtures. Full 256-row frontend/projection regressions also pass for the
three model shapes, with zero Hadamard, quantizer, and software-chain bit
differences. These are standalone RTL checks, not complete inference.

The H-block OOC attempt (`20261023_230342`) completed synthesis in 15m33s at
117,104 LUT and 8,261 FF, then hit its memory guard during timing reporting.
Static, generate-time bank permutations subsequently reduced synthesis area
to 53,904 LUT and 8,266 FF (`20261023_232548`), preserving sixteen pairs per
issue cycle. The static layout requires `PAIRS_PER_CYCLE == LANES/2`.
Both runs have zero DSP/BRAM use; these are synthesis-only resource results.

Clock constraints loaded before synthesis exposed timing loops in the shared
FP32 right-shift-jam helper. Its explicit barrel-stage rewrite passes 24,576
focused Questa checks, the five Hadamard cases and the three actual GEMM
tiles. A scalar registered FP32-adder synthesis probe now has zero detected
combinational loops, but fails 2.500 ns with WNS -5.604 ns and 32 logic levels.
This scalar result does not establish full-Hadamard timing or loop removal.
The next required change is a bit-exact FP32 adder pipeline with metadata and
write-drain scheduling in Hadamard. There is no current accepted 400-MHz
Hadamard route result. See `HADAMARD_METHODOLOGY_RESOLUTION_REPORT.md` for
the measured scope, remaining methodology findings and report paths.

## DiCache adaptive residual caching (runtime hook)

The repository's `papers/related_work_papers.md` refers to DiCache (ICLR 2026), not
the unrelated simulation-only `axi_fake_ddr` read-page cache. DiCache uses a
shallow-layer relative-L1 probe to accumulate estimated output-change error,
refreshing the full model residual when the threshold is crossed. Its second
component, Dynamic Cache Trajectory Alignment (DCTA), estimates a trajectory
coefficient from shallow residuals and combines cached full residuals. See the
[DiCache paper](https://arxiv.org/abs/2508.17356).

The new `supra2_dicache_online_policy.sv` implements the online decision with
per-branch Q16.16 error accumulators, first-use cache seeding, refresh/reuse
strobes, and a runtime `enable` input. The policy is connected through
`supra2_dit_runtime_dma` and `supra2_dit_compute_bridge`. When disabled, the
original package skip-trace scheduler remains active. When enabled,
`supra2_dit_runtime_controller` always runs block 0 as the shallow probe; after
the policy decision it either skips the remaining blocks for that branch or
computes them all to refresh the cache. `supra2_dicache_residual_cache.sv`
stores two full FP16 residual anchors for each CFG branch (32 KiB total for
4096 elements/branch) and streams them through the three-stage,
32-lane `supra2_dicache_dcta_32lane.sv` interpolation datapath. With
`final_head_enable`, the bridge now captures each branch's 128-beat final-head
velocity, stores the current latent across both Euler banks, and after Euler
completion refreshes the cache with `velocity - x_t` for each requested branch.
The runtime controller does not receive Euler completion until all queued
branch refreshes have been acknowledged. On reuse, DCTA residual beats have the
current latent added back before entering the matching conditional or
unconditional CFG velocity buffer, then the repeated final projection is
bypassed. A branch tag accompanies delayed refresh completion so the online
policy marks the correct branch cache valid. With final-head integration
disabled, the existing external residual-stream interface remains active.
`dicache_enable` selects the runtime policy path, while
`dicache_gamma_fp16`, probe-error reduction, and shallow-probe scheduling remain
external inputs. Questa covers policy/scheduler behavior, two-anchor
refresh/reuse, DCTA interpolation, branch isolation, and bridge elaboration.
The runtime-policy Questa test now runs both settings consecutively: enabled
mode launches 750 blocks with 100 probes, 50 refreshes, 50 reuses, and 800
DCTA beats; disabled mode adds the exact 828-launch package schedule with no
additional probe or cache traffic. This toggle regression uses a fixed
Q16.16 probe error, fixed gamma 0.5, and a reduced 64-element residual cache,
so it verifies the policy switch and scheduler behavior rather than an
autonomous probe estimator or full-size cache implementation.
The final-head integration regression now exercises both 128-beat branch
refreshes after Euler completion, checks both policy-valid bits, reconstructs
velocity from the stored residual plus latent within a 1% relative-to-peak
gate, and verifies that CFG consumes the reconstructed stream and the repeated
projection is bypassed. Full 50-step DiCache accuracy, cache memory mapping,
and performance remain unverified because the full inference and board
implementation have not run. The optional `final_param_dma_enable` path also
loads final projection weight (288 512-bit beats), aligned bias (one 512-bit
beat expanded to 16 FP16 writes), and per-step final AdaLN (36 beats). The
compiler table is `[step, 2, 576]`, with shift first and scale second; the
bridge routes the first 18 beats to shift and the next 18 to scale. Final-head
execution waits until both activation and parameter transfers complete.
Dedicated runtime-DMA and bridge Questa tests check addresses, bursts, data
ordering, parameter-write counts, and the output stream. These tests cover
only this loading slice, not a full inference or board implementation. The
runtime DMA retains the invariant final projection weight and bias after their
first transfer in a `start` run; subsequent CFG branches/steps fetch only the
36-beat AdaLN table. Its two-launch regression observes 288 weight beats, one
bias beat, 72 AdaLN beats, and eight bursts total.

For final-head activation intake, `supra2_block_top` now has a 512-bit read
port over its 32-lane FP16 x buffer. The bridge consumes one vector beat per
cycle for 4,608 beats per 256x576 activation, replacing four 128-bit reads per
beat (18,432 read cycles). The old 128-bit port remains for the external
debug/read interface. The final-head/CFG Questa regression checks all 4,608
accepted vector beats for each branch and still matches the latent golden at
0.96875% relative-to-peak error. This is an internal read-path cycle reduction;
it does not establish routed timing or board bandwidth.

The runtime DMA now accepts a generic aligned vector-read descriptor
(DDR address, beat count, destination buffer, and destination beat base) and
routes its existing shared AXI-reader stream directly through the bridge into
the block's 512-bit vector-write port. This avoids an intermediate 512-bit
external data loop for x/context/parameter buffer fills. The runtime reader
arbitrates vector requests against timestep, block AdaLN, final-head parameter,
and weight-tile reads. Questa integration reads four beats into x-buffer beats
10-13, two beats into context beats 3-4, and one beat into parameter beat 4;
lane-varying DDR fixture values verify exact ordering across all 224 written
FP16 entries before final parameter DMA and the final-head fixture. Runtime
controller/DMA, CFG/DiCache, and bridge
elaboration regressions all pass. The producer still supplies the vector
descriptor, and the model-specific x-embedding/context projection schedule is
not yet autonomous.

The block input path now also exposes a 512-bit vector-write port through
`supra2_dit_compute_bridge` into `supra2_block_top`. Each accepted beat writes
32 consecutive FP16 elements to the selected activation, context, or parameter
buffer; the older 16-bit debug/load port remains available. Questa checks the
first and last lane at nonzero beat addresses for both activation and context
buffers, and the bridge elaboration smoke passes. This provides a wide ingress
boundary for the next DMA-to-activation/context connection; the reader stream
is not yet internally routed to this port.

## INT8-only 32x32 array and block-13 closure (2026-10-21)

The production systolic datapath is INT8 multiply with INT32 accumulation;
`pe.sv` has no FP16 MAC instance, and the production `.f` lists omit
`mac_fp16.sv` (its independent diagnostic test remains). The 32x32 array uses
signed inputs for QLinear/QK and unsigned-A mode for attention PV's nonnegative
Q8 softmax weights. FP16 Q/K and V tensors are quantized at their array
interfaces; this is not an FP16 systolic array.

The connected block-13 Questa regression passes against the staged serial RTL
chain reference: 3,307,544 cycles, zero X/Z words, 0.3341% relative-to-peak
error, 0.3191% relative-L2, max absolute error 3. The important fix was to
convert the INT32 QK accumulator to FP32 before dequantization; previously its
raw integer bit pattern was interpreted as a float, producing zero and
infinite scores. Reproduce with `build/questa_full_run.ps1 -BlockId 13` using
the block-13 fixture directory.

Vivado 2021.1 OOC synthesis of the 32x32 unsigned-A array configuration for
xczu7ev-ffvc1156-2-e reports 54,050 LUTs (23.46%), 68,104 registers (14.78%),
1,024 DSP48E2 (59.26%), 4,096 CARRY8 (14.22%), and no BRAM. Under a 5 ns XDC,
WNS is +2.780 ns, with zero setup or hold failures (WHS +0.079 ns). This is
array-only OOC evidence, not synthesis/timing closure of the whole block or a
board measurement.

At an assumed 200 MHz, the measured block count is 16.54 ms per 256-row
branch. Conditional CFG=3 extrapolation with two branches and 8.28 active
blocks/step from the 40.857% skip trace is ~54.77M cycles or 274 ms/step,
before unmeasured embedding, final-head, control, and DDR costs. Without
skips, 14 blocks are ~92.61M cycles or 463 ms before overhead. The skipped
step estimate is below 500 ms only under those assumptions; no connected
full-step RTL simulation or complete top-level implementation has been run.

### OOC placed-and-routed INT8 array (2026-10-21)

The 32x32 `systolic_array` was freshly synthesized and taken through Vivado
`opt_design`, `place_design`, `phys_opt_design`, and `route_design` for
xczu7ev-ffvc1156-2-e using `build/impl_module.ps1 -top systolic_array`.
Post-route utilization is 51,474 LUTs (22.34%), 68,104 registers (14.78%),
4,096 CARRY8 (14.22%), and 1,024 DSP48E2 (59.26%). At 200 MHz, final WNS is
+1.617 ns and WHS is +0.057 ns, with zero failing setup/hold endpoints; all
user-specified timing constraints are met. All 90,647 routable nets completed
with zero routing errors. The routed DCP and utilization/timing/route/DRC
reports are under `build/zcu106_impl_*_systolic_array*`.

The OOC DRC has zero errors, with 4,143 warning/advisory findings, mostly
unregistered DSP48 input/output pipeline ports. This is a routed compute
module result only: it does not include the block controller, DDR interface,
board pin/clock-tree placement, or a system-level XDC. The OOC timing report
also has unconstrained external I/O delays and no top-level `HD.CLK_SRC`.

### MIG + official DDR4 model bridge check (2026-10-21)

Vivado generated a simulation-only x8 MT40A512M8RH-075E MIG configuration
(750 ps DDR CK, 64-bit native app interface, 13.501 ns differential input
clock). The Questa 2024.1 run connects `axi_to_mig_bridge` to this MIG and the
Vivado-installed Xilinx DDR4 pin-level device model. It writes and then reads
back 64 128-bit AXI beats with zero data mismatches. Read transaction result:
189 MIG UI cycles at 333.33 MHz, 1,806 MB/s effective for this single 1-KiB
burst; simulation exits with zero errors and three vendor-model startup
warnings. Reproduce with `build/run_mig_ddr4_sim.ps1`.

This is a bridge/MIG/device-model microbenchmark, not a transformer block
through-DDR measurement. A block-13 DMA/DDR system top has since been added;
its actual-size RTL-DMA performance run is tracked separately in `HANDOFF.md`.
The earlier block/step extrapolations still exclude measured DDR traffic and
must not be treated as end-to-end latency.

### Resource-aware array pipeline exploration (2026-10-22)

The verified one-array tile schedule already keeps the 32x32 array busy for
3,662,496 of 3,663,668 cycles (99.97%). Reordering those same serial array
tasks cannot materially improve array busy fraction; the remaining opportunity
is to improve MAC density inside each task or add parallel MAC capacity. Its
current schedule estimate is 68.48% MAC/peak, not 99.97%: the latter is only
the fraction of cycles in which the array stage is occupied.

Added `scripts/simulate_block_pipeline_dual_array.py` to explore retaining the
32x32 array and adding a 16x32 INT8 sidecar (512 additional DSP48E2, or a
linear estimate of 1,536/1,728 = 88.89% of the ZCU106 DSP budget). At 16-row
sub-tile granularity, the list schedule estimates 2,462,151 cycles/block
versus 3,663,668 for the existing one-array pipeline (1.488x faster). The
simulated main/sidecar busy fractions are 99.98%/97.55%; combined MAC/peak is
67.94%. With the existing fixed step-work estimate this corresponds to
177.0 ms no-skip and 106.6 ms for the recorded skip trace at 200 MHz, excluding
DDR/DMA and system overhead.

This is an optimistic architecture study, not an RTL implementation or
synthesis result. The schedule assumes row-linear stage duration (a 16-row
task takes half as long on the 32x32 array and the same duration on a 16x32
array), and omits dual-engine memory ports, buffering, and dispatch overhead.
The unchanged MAC workload has a 2,509,056-cycle single-array peak-throughput
lower bound; the sidecar estimate is only 1.9% faster than that ideal. This
makes front-end/array-feed overlap the first performance target; retain the
extra array only if the measured end-to-end gain justifies its cost.

The 16x32 **array primitive** has since been functionally tested and taken
through standalone Vivado OOC route. `rtl/sim/tb_systolic_array_16x32.sv`
checks all 512 INT32 outputs for K=576 and K=2304; Questa passes both. The
core feed length is K+16+32+2 cycles, giving 92.01% and 97.88% useful-MAC
utilization respectively for these two full-width array tiles (this excludes
the GEMM frontend, epilogue, and block scheduler). The
existing generic `systolic_array` was synthesized with M=16,N=32,
UNSIGNED_A=1 via `build/impl_module.ps1 -top systolic_array_16x32`.
Post-route results: 25,689 LUT (11.15%), 33,927 FF (7.36%), 512 DSP48E2
(29.63%), 2,048 CARRY8 (7.11%), 0 BRAM; WNS +1.648 ns, WHS +0.057 ns at 5 ns,
with zero setup/hold failures and 45,208/45,208 routable nets. The DRC has
2,048 warnings (DPIP-2/DPOP-3/DPOP-4: DSP input/output pipeline registers
disabled), not errors. This is an array-only OOC result; its XDC has no
top-level clock insertion or external I/O constraints. The 32x32 and 16x32
OOC resource counts sum to 77,163 LUT, 102,031 FF, and 1,536 DSP (88.89% of
the ZCU106 DSP total), but this sum is not a combined-system implementation.

Remaining validation: parameterize/connect the 16x32 GEMM engine and row-slice
dispatcher, measure actual M=16 end-to-end tile latency, then synthesize and
route the combined block. No integrated dual-array speedup is verified yet.
