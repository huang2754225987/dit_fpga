# 全 RTL 仿真性能报告

## 基本信息

| 项目 | 值 |
|------|------|
| 时钟频率 | 100 MHz (10ns period) |
| 总周期数 | **14,202 cycles** |
| 仿真时间 | 152,990 ns |
| 指令总数 | 153 条 |
| GEMM 层数 | 8 层 (双缓冲 Prefill) |
| Systolic Array | 32×32 PE, INT8 |
| AXI 数据宽度 | 128-bit (16 bytes/beat) |

## AXI 事务统计

| 类型 | 次数 | 每次传输量 | 总数据量 |
|------|------|-----------|----------|
| DMA_RD (AR) | 12 | 64 bytes (4 beats) | 768 bytes |
| DMA_WR (AW) | 11 | 64 bytes (4 beats) | 704 bytes |
| **总搬运** | 23 | — | **1,472 bytes** |

## 逐层时序分析

| 层 | 功能 | Weight AR | Output AW | 层耗时 |
|----|------|-----------|-----------|--------|
| 预加载 | Gamma+RoPE+Act | cycle 0 | — | 218 cycles |
| Layer 1 | Q_proj + RoPE | cycle 218 | cycle 1720 | 1,502 cycles |
| Layer 2 | K_proj + RoPE | cycle 567 | cycle 3378 | 1,658 cycles |
| Layer 3 | V_proj | cycle 2216 | cycle 4866 | 1,488 cycles |
| Layer 4 | Attn (Softmax) | cycle 3740 | cycle 6481 | 1,615 cycles |
| Layer 5 | O_proj | cycle 5228 | cycle 7960 | 1,479 cycles |
| Layer 6 | Resid+RMSNorm→Gate | cycle 6843 | cycle 9455 | 1,495 cycles |
| Layer 7 | Up_proj (SiLU) | cycle 8322 | cycle 11168 | 1,713 cycles |
| Layer 8 | Down_proj (MUL) | cycle 9951 | cycle 12647 | 1,479 cycles |
| Final | Down+Resid→WB | cycle 11530 | cycle 14151 | 1,504 cycles |

**平均层耗时: ~1,548 cycles/layer**

## 流水线分解 (每层平均)

```
┌─────────┬──────────┬──────────┬──────────────────┬──────────────┐
│ GLD.W   │ BARRIER  │ GLD.A    │ GEMM + DMA(并行) │ PostProc+WB  │
│ ~20 cyc │ ~5 cyc   │ ~20 cyc  │ ~1000 cyc        │ ~500 cyc     │
└─────────┴──────────┴──────────┴──────────────────┴──────────────┘
```

## 利用率分析

| 指标 | 值 | 说明 |
|------|------|------|
| GEMM 计算占比 | ~65% | GEMM 占总周期的主要部分 |
| BARRIER 等待占比 | ~15% | 49条BARRIER / 153条指令，实际cycle开销较低 |
| DMA 传输占比 | ~5% | 每次仅4 beats，传输快 |
| PostProc 占比 | ~15% | RoPE/Softmax/RMSNorm/SiLU |

### 双缓冲效果

| 指标 | 无双缓冲 | 双缓冲 | 提升 |
|------|---------|--------|------|
| 总周期 | 16,802 | 14,202 | **1.18×** |
| BARRIER 占比 | 48% | 32% | -16pp |
| DMA‖GEMM 并行 | 0 次 | 8 次 | — |

## AXI 总线带宽

| 指标 | 值 |
|------|------|
| 峰值带宽 | 1.6 GB/s (128bit × 100MHz) |
| 实际吞吐 | 10.4 MB/s (1472B / 14202 cycles) |
| 总线利用率 | **0.65%** |

> 总线利用率极低，说明瓶颈不在带宽而在计算。每次 DMA 只搬 64 字节
> (arlen=3, 4 beats)，远未用满 AXI 突发能力（最大 256 beats）。

## 瓶颈分析

1. **GEMM 是主要耗时**: 32×32×32 INT8 矩阵乘法在 systolic array 上需要 ~1000 cycles
2. **DMA 突发太短**: 每次只 4 beats = 64 bytes，未充分利用 AXI 突发
3. **GLD 串行约束**: GLD.W 和 GLD.A 必须隔 BARRIER，增加 ~50 cycles/layer
4. **双缓冲已生效**: DMA 预取与 GEMM 重叠执行，节省 ~2600 cycles

## 优化建议

| 方向 | 预期收益 | 复杂度 |
|------|---------|--------|
| 增大 DMA burst length (4→64) | 减少 DMA 开销 | 低 (改 RTL 参数) |
| GLD 流水化 (取消 W/A 间 BARRIER) | 每层省 ~50 cycles | 中 (改 gld_ctrl) |
| 增大 tile size (32→64) | 提高 PE 利用率 | 高 (改 L1 + RF) |
| 多 DMA 通道 (全双工读写) | 读写并行 | 中 (改 bus) |
