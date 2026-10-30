// =============================================================================
// acc_drain_ctrl.sv - ACC -> PostProc RF 搬运控制器 (NOP_AC2V)
// =============================================================================
// 一条命令搬运 acc_buffer 的一行（32 bank × 32-bit fp32）：
//   1. 发 acc 行读（行并行读口，组合输出 32 bank）
//   2. 锁存 32 个 fp32
//   3. 32 路组合 fp32_to_fp16（RNE），打一拍寄存
//   4. 打包 512-bit 驱动 PRF 写口 1 拍
// 每条命令搬 1 行；ucode 循环 32 次搬一个 32x32 tile。
// busy/done 握手风格与 vec_engine 一致（done 为 1 拍脉冲）。
// =============================================================================
`default_nettype none

module acc_drain_ctrl
  import fp16_utils_pkg::*;
#(
    parameter int LANES  = 32,
    parameter int ACC_W  = 32,   // acc bank 数据宽度 (fp32)
    parameter int ELEM_W = 16    // PRF lane 宽度 (fp16)
) (
    input  wire                       clk,
    input  wire                       rst_n,

    // AC2V 命令（来自 ucode_decode，1 拍脉冲）
    input  wire                       cmd_valid,
    input  wire [3:0]                 cmd_vd,   // PRF 目的寄存器
    input  wire [4:0]                 cmd_row,  // acc 行号 (imm[4:0])
    input  wire                       cmd_set,  // set 选择 (imm[5]): 0=非活动集, 1=活动集
    input  wire                       cmd_zero, // write an all-zero vector without reading ACC

    // acc_buffer 读口（行并行）；rd_grant=0 表示口被 topk/flush 占用，等待
    input  wire                       rd_grant,
    output logic                      acc_rd_en,
    output logic [9:0]                acc_rd_addr,  // flat: {row, 5'd0}
    input  wire [ACC_W-1:0]           acc_rd_data [0:LANES-1],

    // 读口占用标志（top 用于读 mux / set mux 选择）
    output logic                      rd_active,
    output logic                      rd_set,       // 锁存后的 set 选择位

    // PostProc RF 写口
    output logic                      prf_wr_en,
    output logic [3:0]                prf_wr_addr,
    output logic [LANES*ELEM_W-1:0]   prf_wr_data,

    output logic                      busy,
    output logic                      done
);

    localparam int VEC_W = LANES * ELEM_W;

    typedef enum logic [2:0] {
        S_IDLE,
        S_READ,     // 驱动 acc 行读，grant 当拍锁存 32×fp32
        S_CVT,      // 32 路组合 fp32->fp16，打一拍（拆流水保时序）
        S_WRITE,    // 驱动 PRF 写口 1 拍
        S_DONE
    } state_t;

    state_t state, state_nxt;

    logic [3:0] r_vd;
    logic [4:0] r_row;
    logic       r_set;
    logic       r_zero;

    logic [ACC_W-1:0]  row_fp32 [0:LANES-1];
    logic [ELEM_W-1:0] row_fp16 [0:LANES-1];

    // ----------------------------------------------------------------
    // FSM
    // ----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) state <= S_IDLE;
        else        state <= state_nxt;

    always_comb begin
        state_nxt = state;
        case (state)
            S_IDLE:  if (cmd_valid) state_nxt = cmd_zero ? S_WRITE : S_READ;
            // 读口被高优先级消费者（topk/flush）占用时停在 S_READ 等待
            S_READ:  if (rd_grant)  state_nxt = S_CVT;
            S_CVT:                  state_nxt = S_WRITE;
            S_WRITE:                state_nxt = S_DONE;
            S_DONE:                 state_nxt = S_IDLE;
            default:                state_nxt = S_IDLE;
        endcase
    end

    // ----------------------------------------------------------------
    // 命令锁存
    // ----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_vd  <= '0;
            r_row <= '0;
            r_set <= 1'b0;
            r_zero <= 1'b0;
        end else if (state == S_IDLE && cmd_valid) begin
            r_vd  <= cmd_vd;
            r_row <= cmd_row;
            r_set <= cmd_set;
            r_zero <= cmd_zero;
        end
    end

    // ----------------------------------------------------------------
    // acc 行读：组合读口，grant 当拍数据有效，拍尾锁存
    // ----------------------------------------------------------------
    always_comb begin
        acc_rd_en   = (state == S_READ) && rd_grant && !r_zero;
        acc_rd_addr = {r_row, 5'd0};   // 行地址；列字段不用（行并行全出）
    end

    assign rd_active = (state == S_READ);
    assign rd_set    = r_set;

    always_ff @(posedge clk) begin
        if (state == S_READ && rd_grant && r_zero) begin
            for (int i = 0; i < LANES; i++)
                row_fp32[i] <= '0;
        end else if (state == S_READ && rd_grant) begin
            for (int i = 0; i < LANES; i++)
                row_fp32[i] <= acc_rd_data[i];
        end
    end

    // ----------------------------------------------------------------
    // 32 路组合 fp32 -> fp16（RNE），S_CVT 拍尾锁存
    // ----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (state == S_CVT) begin
            for (int i = 0; i < LANES; i++)
                row_fp16[i] <= fp32_to_fp16(row_fp32[i]);
        end
    end

    // ----------------------------------------------------------------
    // 打包 512-bit 写 PRF（1 拍）
    // ----------------------------------------------------------------
    always_comb begin
        prf_wr_en   = (state == S_WRITE);
        prf_wr_addr = r_vd;
        for (int i = 0; i < LANES; i++)
            prf_wr_data[i*ELEM_W +: ELEM_W] = r_zero ? '0 : row_fp16[i];
    end

    // ----------------------------------------------------------------
    // 状态
    // ----------------------------------------------------------------
    assign busy = (state != S_IDLE);
    assign done = (state == S_DONE);

endmodule

`default_nettype wire
