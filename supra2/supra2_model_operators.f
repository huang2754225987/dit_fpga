# Supra2-IMG operator RTL allowlist; excludes testbenches and legacy NPU RTL.
# Candidate source set only: this is not yet a complete FPGA top-level list.
rtl/pkg/isa_pkg.sv
rtl/pkg/fp16_utils_pkg.sv
rtl/ops/graph_exp_lut_fp164k.sv
rtl/ops/recip_lut4k.sv
rtl/ops/softmax_engine.sv
rtl/ops/silu_engine.sv
rtl/ops/silu_lut_fp16.sv
rtl/gemm/mac_int8.sv
rtl/gemm/pe.sv
rtl/gemm/systolic_array.sv
rtl/supra2/fp32_add_rne_pipeline.sv
rtl/supra2/hadamard_transform.sv
rtl/supra2/activation_quantizer.sv
rtl/supra2/qlinear_epilogue.sv
rtl/supra2/position_add.sv
rtl/supra2/layernorm_modulate.sv
rtl/supra2/gelu_tanh.sv
rtl/supra2/residual_gate.sv
rtl/supra2/attention_online_pv.sv
rtl/supra2/attention_online_pv_batch.sv
rtl/supra2/euler_update.sv
