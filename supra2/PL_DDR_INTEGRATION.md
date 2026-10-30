# ZCU106 PL DDR4 integration status

## Board-matched MIG settings

The installed ZCU106 2.6 board preset selects `MT40A256M16GE-075E`, a 64-bit
DDR4 interface, 833 ps DDR4 clock period, and 3332 ps differential reference
clock period. The generated native-app MIG wrapper confirms a 512-bit app data
bus, 64-bit byte mask, 28-bit app address, `ui_clk`, `ui_clk_sync_rst`, and
`init_calib_complete`. The user design must run in `ui_clk` and remain reset
until calibration completes. MIG owns the DDR4 PHY pins and board timing XDC.

Recreate the IP with Vivado 2021.1:

```powershell
$env:PROCESSOR_ARCHITECTURE = 'AMD64'
& 'D:\soft\vitis\Vivado\2021.1\bin\vivado.bat' -mode batch `
  -source build/create_zcu106_pl_mig.tcl
```

Generated output is isolated under `build/zcu106_pl/`; it does not replace the
64-bit simulation MIG under `build/mig_ddr4_sim/`.

## Current 512-bit integration

The board top now configures both the UART loader and block DMA for 512-bit
AXI and connects that bus directly to the board MIG, with no width converter.
The reusable simulation system retains a 128-bit default and supports a
512-bit configuration through its AXI-to-64-bit native-app bridge. The block
DMA packs four 128-bit block-result reads into each 512-bit output write.

DDR payload components and their final extent are aligned to 64 bytes. DMA
bursts are capped at 4 KiB (64 beats at 512 bits), and the UART write command's
beat count now means 64-byte beats on the board top. The loader receives the
serialized payload in that format before the start command. Sustained UART
load time is separate from the compute/DMA timing measurement.

The board still requires a host to deliver the payload using the documented
UART packet format.

## Verification state

Vivado 2021.1 successfully generated the board-parameterized MIG IP. Its AXI
wrapper is 512 bits wide, and its native app interface reports
`c0_ddr4_app_wdf_data[511:0]`, `c0_ddr4_app_wdf_mask[63:0]`, and
`c0_ddr4_app_addr[27:0]`. On 2026-10-23, the project `supra2_axi_read_dma`
was connected through the 512-bit AXI-to-64-bit MIG app bridge to the official
DDR4 pin model. Questa wrote and read back a 4-KiB burst (64 AXI beats), checked
every 512-bit word, and verified one aligned INCR burst with ARSIZE=6. Result:
541 UI cycles at 333.33 MHz and 2,523.72 MB/s, with zero simulator errors and
three DDR4 vendor-model startup warnings. Reproduce with
`build/run_mig_ddr4_sim.ps1`; the log is
`build/questa_mig_ddr4_dma_512_run.log`. This validates the DMA/bridge/MIG
transfer path, not the full model compute top. A further 2026-10-23 Questa run
connected `supra2_dit_runtime_controller` through `supra2_dit_runtime_dma`, the
512-bit bridge, and the official DDR4 pin model. It wrote and read back the
first package-addressed 16-beat (1-KiB) weight tile, checked all 512-bit words,
and completed the 50-step controller schedule with one real tile fetch; the
remaining compute handshakes were supplied by the testbench. The run completed
in 7,522 UI cycles with zero errors and the same three vendor-model startup
warnings. This cycle count includes the synthetic schedule handshakes and is
not an inference latency or throughput result. Full-volume payload traffic,
actual compute-engine integration, full-system synthesis/implementation,
bitstream, and system-level frequency remain unverified. The prior 3.10 ns
result is an out-of-context array result, not a whole-system clock result.

The 2026-10-23 follow-up extends that pin-model test to read the same written
16-beat DDR region through the generic vector-load client after the controller
weight-tile transaction. All 16 vector beats are checked against the written
512-bit pattern, including target and destination beat numbering. The combined
controller-tile plus vector DMA run passes in 7,868 UI cycles at 333.33 MHz
with zero simulator errors and three vendor-model startup warnings. This
validates the real MIG path for the vector client; the bridge-to-buffer write
mapping is separately covered by Questa with a mocked AXI source.
