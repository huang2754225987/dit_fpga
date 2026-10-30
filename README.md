# RTL Kept for Supra2-IMG Bring-Up

This isolated copy no longer contains the generic Qwen/MoE `top.sv` or
model-inapplicable Top-K/expert, low-bit dequantization, RoPE, and RMSNorm
datapaths. It retains the verified Supra2 operators, timestep SiLU, shared 32x32
FP16/INT8 array primitives, and generic DMA/memory/control infrastructure as
integration candidates. A dedicated Supra2 system top has not been implemented.

The supplied archive targets 100 signed-INT8 QLinear operators and 37 FP16
passthrough tensors. The removed generic top hardwired its array to FP16 and
used the INT1/2/3 dequant-to-FP16 path for integer formats; the archive's signed
INT8-to-INT32-to-FP32 epilogue is exercised in standalone RTL slices but is
not yet integrated with DMA/DDR. The independent 32x32 INT8 array test is
`rtl/sim/tb_supra2_int8_golden.sv`.

Supra2-specific hardware priorities:

1. Add online Hadamard + per-token INT8 quantization and the signed INT8
   QLinear path to a dedicated top.
2. Batch CFG rows as M=512, tile to local-memory capacity, cache cross-attention
   K/V, and follow the package's block-skip trace.
3. Use online softmax for self/cross attention; add LayerNorm/AdaLN/GELU(tanh).
4. Keep FP16 GEMM for passthrough tensors; omit experts/MoE/low-bit quantization,
   RoPE, and RMSNorm from the new source closure.

The copied `tb_gemm_array_efficiency.sv` now compiles and passes 73 directed
FP16/INT1/INT2/INT3 GEMM controller/array checks in ModelSim. This validates
the isolated GEMM core with preloaded register-file fixtures; it does not
validate model checkpoint loading, DMA/DDR traffic, the generic NPU top, or a
full Supra2-IMG denoiser. No synthesis or FPGA timing/resource report
has been produced for this copy.

The earlier 256-token x_embed projection (real checkpoint weight+bias,
synthetic inputs) passed through the generic top and FakeDDR before the top
was removed from this isolated copy. Compact 2-KiB FP16 weight reads plus
4-KiB linear writeback reduced FakeDDR reads from 659,456 to 364,544 bytes and
modeled cycles from 289,389 to 241,805. This is not MIG, synthesis, or board
validation. The remaining DMA/memory/control primitives are candidates, not a
verified Supra2 system closure.
