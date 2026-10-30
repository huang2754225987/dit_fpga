// ===========================================================================
//  AXI4-Lite Slave Register Bank for NPU Control/Status
//
//  Implements full AXI4-Lite write and read protocol with:
//    - CTRL register: START bit auto-clears after 1 cycle (pulse),
//      SOFT_RESET bit holds asserted for 4 cycles then auto-clears.
//    - STATUS register: read-only, driven by external done/busy/error inputs.
//    - All other registers: standard read-write with reset defaults of 0.
//
//  Fully synthesizable -- no delays.
// ===========================================================================
module axi_lite_regs
    import npu_pkg::*;
    import axi_types_pkg::*;
(
    input  logic                      clk,
    input  logic                      rst_n,

    // -----------------------------------------------------------------------
    //  AXI4-Lite slave interface
    // -----------------------------------------------------------------------
    // Write-address channel
    input  logic [LITE_ADDR_W-1:0]    s_axil_awaddr,
    input  logic                      s_axil_awvalid,
    output logic                      s_axil_awready,

    // Write-data channel
    input  logic [LITE_DATA_W-1:0]    s_axil_wdata,
    input  logic [LITE_STRB_W-1:0]    s_axil_wstrb,
    input  logic                      s_axil_wvalid,
    output logic                      s_axil_wready,

    // Write-response channel
    output logic [1:0]                s_axil_bresp,
    output logic                      s_axil_bvalid,
    input  logic                      s_axil_bready,

    // Read-address channel
    input  logic [LITE_ADDR_W-1:0]    s_axil_araddr,
    input  logic                      s_axil_arvalid,
    output logic                      s_axil_arready,

    // Read-data channel
    output logic [LITE_DATA_W-1:0]    s_axil_rdata,
    output logic [1:0]                s_axil_rresp,
    output logic                      s_axil_rvalid,
    input  logic                      s_axil_rready,

    // -----------------------------------------------------------------------
    //  Status inputs (directly drive the read-only STATUS register)
    // -----------------------------------------------------------------------
    input  logic                      done_i,
    input  logic                      busy_i,
    input  logic                      error_i,

    // -----------------------------------------------------------------------
    //  Register value outputs
    // -----------------------------------------------------------------------
    output logic [31:0]               ctrl_o,
    output logic [31:0]               status_o,
    output logic [31:0]               ucode_base_o,
    output logic [31:0]               ucode_len_o,
    output logic [31:0]               ddr_base_act_o,
    output logic [31:0]               ddr_base_wgt_o,
    output logic [31:0]               ddr_base_kv_o,
    output logic [31:0]               ddr_base_out_o,
    output logic [31:0]               model_hidden_o,
    output logic [31:0]               model_heads_o,
    output logic [31:0]               model_head_dim_o,
    output logic [31:0]               seq_len_o,
    output logic [31:0]               token_idx_o,
    output logic [31:0]               debug_ctrl_o,
    output logic [31:0]               exec_mode_o,

    // -----------------------------------------------------------------------
    //  Graph Mode status inputs (read-only)
    // -----------------------------------------------------------------------
    input  logic [31:0]               graph_status_i,
    input  logic [31:0]               graph_pc_i,
    input  logic [31:0]               graph_last_op_i,

    // -----------------------------------------------------------------------
    //  Derived control outputs
    // -----------------------------------------------------------------------
    output logic                      start_pulse_o,
    output logic                      soft_reset_o,

    // -----------------------------------------------------------------------
    //  New config outputs (GEMM dimensions + scale pointer)
    // -----------------------------------------------------------------------
    output logic [31:0]               ddr_base_scale_o,
    output logic [15:0]               cfg_M_o,
    output logic [15:0]               cfg_N_o,
    output logic [15:0]               cfg_K_o,

    // ---- Host-in-the-loop MoE outputs ----
    output logic [31:0]               host_cmd_o,
    output logic [31:0]               trap_status_o,
    output logic [31:0]               ucode_load_addr_o,
    output logic [31:0]               ucode_load_len_o,
    output logic [31:0]               resume_pc_o,
    output logic [31:0]               acc_flush_addr_o,
    output logic [31:0]               acc_flush_len_o,

    // K-tiling accumulate flag (sticky read/write). 1 = skip sa_clear and
    // accumulate across GEMM calls; 0 = clear systolic acc on next GEMM.
    output logic                      acc_accum_o,

    // Dequant group-wise asymmetric config (qwen1.5quant)
    output logic [2:0]                deq_nbits_code_o,  // 1/2/3/4/8-bit per expert proj
    output logic [31:0]               deq_scale_len_o,  // 256-bit beats to DMA into scale SRAM

    // Expert precision table load (one entry per write, REG_EXPERT_PREC_CTRL)
    output logic                      expert_prec_wr_en_o,   // 1-cycle pulse
    output logic [9:0]                expert_prec_wr_addr_o, // exp_idx*3 + proj
    output logic [1:0]                expert_prec_wr_data_o, // precision_t

    // Expert descriptor table load (REG_EDESC_IDX/W0/W1/W2)
    output logic                      edesc_wr_en_o,        // 1-cycle pulse
    output logic [9:0]                edesc_wr_addr_o,      // 0..179 = exp_idx*3 + proj
    output logic [31:0]               edesc_wr_data_o,      // word being written
    output logic [1:0]                edesc_wr_sel_o,       // 0=weight,1=scale,2=attr

    // ---- Host-in-the-loop MoE status inputs ----
    input  logic                      trap_set_i      // set trap_status[0] (sticky, segment done)
);

    // =======================================================================
    //  Internal register storage
    // =======================================================================
    logic [31:0] reg_ctrl;
    logic [31:0] reg_ucode_base;
    logic [31:0] reg_ucode_len;
    logic [31:0] reg_ddr_base_act;
    logic [31:0] reg_ddr_base_wgt;
    logic [31:0] reg_ddr_base_kv;
    logic [31:0] reg_ddr_base_out;
    logic [31:0] reg_model_hidden;
    logic [31:0] reg_model_heads;
    logic [31:0] reg_model_head_dim;
    logic [31:0] reg_seq_len;
    logic [31:0] reg_token_idx;
    logic [31:0] reg_debug_ctrl;
    logic [31:0] reg_exec_mode;
    logic [31:0] reg_ddr_base_scale;
    logic [31:0] reg_cfg_M;
    logic [31:0] reg_cfg_N;
    logic [31:0] reg_cfg_K;

    // Host-in-the-loop MoE registers
    logic [31:0] reg_host_cmd;          // W1S-style strobe (self-clearing bits)
    logic [31:0] reg_trap_status;       // RO
    logic [31:0] reg_ucode_load_addr;
    logic [31:0] reg_ucode_load_len;
    logic [31:0] reg_resume_pc;
    logic [31:0] reg_acc_flush_addr;
    logic [31:0] reg_acc_flush_len;
    logic [31:0] reg_acc_accum;          // sticky R/W: bit0 drives gemm_ctrl.acc_accumulate
    logic [31:0] reg_deq_nbits_code;     // sticky R/W: [2:0] nbits_code for dequant
    logic [31:0] reg_deq_scale_len;      // sticky R/W: 256-bit beats to DMA into scale SRAM
    logic [31:0] reg_expert_prec_entry;  // cached write data for expert prec
    logic        reg_expert_prec_wr_stb; // 1-cycle pulse on write to REG_EXPERT_PREC_CTRL
    logic [9:0]  reg_edesc_idx;          // latched descriptor entry index
    logic [31:0] reg_edesc_wdata;        // latched word to write
    logic [1:0]  reg_edesc_sel;          // 0=weight_base,1=scale_base,2=attr
    logic        reg_edesc_wr_stb;       // 1-cycle pulse on W0/W1/W2 write

    // Soft-reset hold counter (holds reset for 4 cycles)
    logic [2:0]  soft_reset_cnt;

    // =======================================================================
    //  AXI4-Lite Write FSM
    // =======================================================================
    //  We accept AW and W simultaneously (both ready asserted together).
    //  When both handshakes complete, we perform the register write and
    //  present the B response.
    // =======================================================================
    logic        aw_latched;
    logic [31:0] aw_addr_r;
    logic        w_latched;
    logic [31:0] w_data_r;
    logic [3:0]  w_strb_r;

    // AW ready: accept when we have no pending write and B channel is clear
    assign s_axil_awready = !aw_latched && !s_axil_bvalid;
    // W ready: accept when we have no pending write data and B channel is clear
    assign s_axil_wready  = !w_latched  && !s_axil_bvalid;

    // Write response is always OKAY (no decode errors for valid addresses)
    assign s_axil_bresp = AXI_RESP_OKAY;

    // Latch AW and W channels; perform write when both are captured
    logic wr_en;
    logic [31:0] wr_addr;
    logic [31:0] wr_data;
    logic [3:0]  wr_strb;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            aw_latched <= 1'b0;
            aw_addr_r  <= 32'd0;
            w_latched  <= 1'b0;
            w_data_r   <= 32'd0;
            w_strb_r   <= 4'd0;
        end else begin
            // wr_en clearing has priority over new latch-set to prevent
            // latches getting stuck when AW+W arrive simultaneously
            if (wr_en) begin
                aw_latched <= 1'b0;
                w_latched  <= 1'b0;
            end else begin
                if (s_axil_awvalid && s_axil_awready)
                    aw_latched <= 1'b1;
                if (s_axil_wvalid && s_axil_wready)
                    w_latched <= 1'b1;
            end

            if (s_axil_awvalid && s_axil_awready)
                aw_addr_r <= s_axil_awaddr;

            if (s_axil_wvalid && s_axil_wready) begin
                w_data_r <= s_axil_wdata;
                w_strb_r <= s_axil_wstrb;
            end
        end
    end

    // Determine when both AW and W data are available (possibly in same cycle)
    logic aw_fire, w_fire;
    assign aw_fire = (s_axil_awvalid && s_axil_awready) || aw_latched;
    assign w_fire  = (s_axil_wvalid  && s_axil_wready)  || w_latched;
    assign wr_en   = aw_fire && w_fire && !s_axil_bvalid;

    // Select between newly arriving data and latched data
    assign wr_addr = aw_latched ? aw_addr_r : s_axil_awaddr;
    assign wr_data = w_latched  ? w_data_r  : s_axil_wdata;
    assign wr_strb = w_latched  ? w_strb_r  : s_axil_wstrb;

    // BVALID management
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            s_axil_bvalid <= 1'b0;
        else if (wr_en)
            s_axil_bvalid <= 1'b1;
        else if (s_axil_bvalid && s_axil_bready)
            s_axil_bvalid <= 1'b0;
    end

    // =======================================================================
    //  Byte-lane write helper: apply strobes to produce new register value
    // =======================================================================
    function automatic logic [31:0] strb_write(
        input logic [31:0] old_val,
        input logic [31:0] new_val,
        input logic [3:0]  strb
    );
        logic [31:0] result;
        result[ 7: 0] = strb[0] ? new_val[ 7: 0] : old_val[ 7: 0];
        result[15: 8] = strb[1] ? new_val[15: 8] : old_val[15: 8];
        result[23:16] = strb[2] ? new_val[23:16] : old_val[23:16];
        result[31:24] = strb[3] ? new_val[31:24] : old_val[31:24];
        return result;
    endfunction

    // =======================================================================
    //  Register write logic
    // =======================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_ctrl           <= 32'd0;
            reg_ucode_base     <= 32'd0;
            reg_ucode_len      <= 32'd0;
            reg_ddr_base_act   <= 32'd0;
            reg_ddr_base_wgt   <= 32'd0;
            reg_ddr_base_kv    <= 32'd0;
            reg_ddr_base_out   <= 32'd0;
            reg_model_hidden   <= 32'd0;
            reg_model_heads    <= 32'd0;
            reg_model_head_dim <= 32'd0;
            reg_seq_len        <= 32'd0;
            reg_token_idx      <= 32'd0;
            reg_debug_ctrl     <= 32'd0;
            reg_exec_mode      <= 32'd0;
            reg_ddr_base_scale <= 32'd0;
            reg_cfg_M          <= 32'd0;
            reg_cfg_N          <= 32'd0;
            reg_cfg_K          <= 32'd0;
            reg_host_cmd       <= 32'd0;
            reg_trap_status    <= 32'd0;
            reg_ucode_load_addr<= 32'd0;
            reg_ucode_load_len <= 32'd0;
            reg_resume_pc      <= 32'd0;
            reg_acc_flush_addr <= 32'd0;
            reg_acc_flush_len  <= 32'd0;
            reg_acc_accum      <= 32'd0;
            reg_deq_nbits_code <= 32'd0;
            reg_deq_scale_len  <= 32'd0;
            reg_expert_prec_entry <= 32'd0;
            reg_expert_prec_wr_stb <= 1'b0;
            reg_edesc_idx     <= 10'd0;
            reg_edesc_wdata   <= 32'd0;
            reg_edesc_sel     <= 2'd0;
            reg_edesc_wr_stb  <= 1'b0;
            soft_reset_cnt     <= 3'd0;
        end else begin
            // Expert prec strobe self-clears after 1 cycle
            reg_expert_prec_wr_stb <= 1'b0;
            // Expert descriptor write strobe self-clears after 1 cycle
            reg_edesc_wr_stb <= 1'b0;
            // ----------------------------------------------------------
            //  START bit auto-clear: pulse for exactly 1 cycle
            // ----------------------------------------------------------
            if (reg_ctrl[CTRL_START])
                reg_ctrl[CTRL_START] <= 1'b0;

            // ----------------------------------------------------------
            //  SOFT_RESET auto-clear after 4 cycles
            // ----------------------------------------------------------
            if (reg_ctrl[CTRL_SOFT_RESET]) begin
                if (soft_reset_cnt == 3'd3) begin
                    reg_ctrl[CTRL_SOFT_RESET] <= 1'b0;
                    soft_reset_cnt            <= 3'd0;
                end else begin
                    soft_reset_cnt <= soft_reset_cnt + 3'd1;
                end
            end

            // ----------------------------------------------------------
            //  HOST_CMD auto-clear: strobe for 1 cycle (like START)
            // ----------------------------------------------------------
            if (reg_host_cmd != 32'd0)
                reg_host_cmd <= 32'd0;

            // ----------------------------------------------------------
            //  TRAP_STATUS sticky set: HW sets bit0 when trap_set_i pulses
            //  (e.g. npu segment done). Read-only by host; cleared only by reset.
            // ----------------------------------------------------------
            if (trap_set_i)
                reg_trap_status[0] <= 1'b1;

            // ----------------------------------------------------------
            //  Register write decode
            // ----------------------------------------------------------
            if (wr_en) begin
                case (wr_addr[7:0])   // decode lower 8 bits of address
                    REG_CTRL[7:0]: begin
                        reg_ctrl <= strb_write(reg_ctrl, wr_data, wr_strb);
                        // Reset the soft-reset counter whenever CTRL is written
                        if (wr_strb[0] && wr_data[CTRL_SOFT_RESET])
                            soft_reset_cnt <= 3'd0;
                    end
                    // REG_STATUS is read-only -- writes are silently ignored
                    REG_UCODE_BASE[7:0]:     reg_ucode_base     <= strb_write(reg_ucode_base,     wr_data, wr_strb);
                    REG_UCODE_LEN[7:0]:      reg_ucode_len      <= strb_write(reg_ucode_len,      wr_data, wr_strb);
                    REG_DDR_BASE_ACT[7:0]:   reg_ddr_base_act   <= strb_write(reg_ddr_base_act,   wr_data, wr_strb);
                    REG_DDR_BASE_WGT[7:0]:   reg_ddr_base_wgt   <= strb_write(reg_ddr_base_wgt,   wr_data, wr_strb);
                    REG_DDR_BASE_KV[7:0]:    reg_ddr_base_kv    <= strb_write(reg_ddr_base_kv,    wr_data, wr_strb);
                    REG_DDR_BASE_OUT[7:0]:   reg_ddr_base_out   <= strb_write(reg_ddr_base_out,   wr_data, wr_strb);
                    REG_MODEL_HIDDEN[7:0]:   reg_model_hidden   <= strb_write(reg_model_hidden,   wr_data, wr_strb);
                    REG_MODEL_HEADS[7:0]:    reg_model_heads     <= strb_write(reg_model_heads,    wr_data, wr_strb);
                    REG_MODEL_HEAD_DIM[7:0]: reg_model_head_dim <= strb_write(reg_model_head_dim, wr_data, wr_strb);
                    REG_SEQ_LEN[7:0]:        reg_seq_len        <= strb_write(reg_seq_len,        wr_data, wr_strb);
                    REG_TOKEN_IDX[7:0]:      reg_token_idx      <= strb_write(reg_token_idx,      wr_data, wr_strb);
                    REG_DEBUG_CTRL[7:0]:     reg_debug_ctrl     <= strb_write(reg_debug_ctrl,     wr_data, wr_strb);
                    REG_EXEC_MODE[7:0]:      reg_exec_mode      <= strb_write(reg_exec_mode,      wr_data, wr_strb);
                    REG_DDR_BASE_SCALE[7:0]: reg_ddr_base_scale <= strb_write(reg_ddr_base_scale, wr_data, wr_strb);
                    REG_CFG_M[7:0]:          reg_cfg_M          <= strb_write(reg_cfg_M,          wr_data, wr_strb);
                    REG_CFG_N[7:0]:          reg_cfg_N          <= strb_write(reg_cfg_N,          wr_data, wr_strb);
                    REG_CFG_K[7:0]:          reg_cfg_K          <= strb_write(reg_cfg_K,          wr_data, wr_strb);
                    REG_HOST_CMD[7:0]:       reg_host_cmd        <= wr_data;  // strobe, self-clears next cycle
                    REG_UCODE_LOAD_ADDR[7:0]:reg_ucode_load_addr<= strb_write(reg_ucode_load_addr, wr_data, wr_strb);
                    REG_UCODE_LOAD_LEN[7:0]: reg_ucode_load_len <= strb_write(reg_ucode_load_len,  wr_data, wr_strb);
                    REG_RESUME_PC[7:0]:      reg_resume_pc      <= strb_write(reg_resume_pc,       wr_data, wr_strb);
                    REG_ACC_FLUSH_ADDR[7:0]: reg_acc_flush_addr<= strb_write(reg_acc_flush_addr,  wr_data, wr_strb);
                    REG_ACC_FLUSH_LEN[7:0]:  reg_acc_flush_len <= strb_write(reg_acc_flush_len,   wr_data, wr_strb);
                    REG_ACC_ACCUM[7:0]:      reg_acc_accum     <= strb_write(reg_acc_accum,       wr_data, wr_strb);
                    REG_DEQUANT_CFG[7:0]:    reg_deq_nbits_code<= strb_write(reg_deq_nbits_code, wr_data, wr_strb);
                    REG_DEQUANT_SCALE_LEN[7:0]: reg_deq_scale_len<= strb_write(reg_deq_scale_len, wr_data, wr_strb);
                    // Expert precision table: one entry per write.
                    //   [1:0]=prec, [3:2]=proj, [9:4]=expert idx.
                    REG_EXPERT_PREC_CTRL[7:0]: begin
                        reg_expert_prec_entry  <= wr_data;
                        reg_expert_prec_wr_stb <= 1'b1;
                    end
                    // Expert descriptor table: IDX latches target entry,
                    // W0/W1/W2 each strobe one 32-bit word into it.
                    REG_EDESC_IDX[7:0]: begin
                        reg_edesc_idx   <= wr_data[9:0];
                    end
                    REG_EDESC_W0[7:0]: begin
                        reg_edesc_wdata  <= wr_data;
                        reg_edesc_sel    <= 2'd0;
                        reg_edesc_wr_stb <= 1'b1;
                    end
                    REG_EDESC_W1[7:0]: begin
                        reg_edesc_wdata  <= wr_data;
                        reg_edesc_sel    <= 2'd1;
                        reg_edesc_wr_stb <= 1'b1;
                    end
                    REG_EDESC_W2[7:0]: begin
                        reg_edesc_wdata  <= wr_data;
                        reg_edesc_sel    <= 2'd2;
                        reg_edesc_wr_stb <= 1'b1;
                    end
                    // REG_TRAP_STATUS is read-only
                    // REG_GRAPH_STATUS, REG_GRAPH_PC, REG_GRAPH_LAST_OP are read-only
                    default: ;  // unmapped -- silently ignore
                endcase
            end
        end
    end

    // =======================================================================
    //  AXI4-Lite Read FSM
    // =======================================================================
    //  Accept AR, decode address, drive R data.
    // =======================================================================
    logic        rd_pending;
    logic [31:0] rd_addr_r;

    assign s_axil_arready = !rd_pending && !s_axil_rvalid;
    assign s_axil_rresp   = AXI_RESP_OKAY;

    // Assemble the read-only STATUS register from external inputs
    logic [31:0] status_reg;
    assign status_reg = {29'd0, error_i, busy_i, done_i};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axil_rvalid <= 1'b0;
            s_axil_rdata  <= 32'd0;
            rd_pending    <= 1'b0;
            rd_addr_r     <= 32'd0;
        end else begin
            // Latch read address
            if (s_axil_arvalid && s_axil_arready) begin
                rd_pending <= 1'b1;
                rd_addr_r  <= s_axil_araddr;
            end

            // Drive read data one cycle after accepting AR
            if (rd_pending && !s_axil_rvalid) begin
                s_axil_rvalid <= 1'b1;
                rd_pending    <= 1'b0;

                case (rd_addr_r[7:0])
                    REG_CTRL[7:0]:           s_axil_rdata <= reg_ctrl;
                    REG_STATUS[7:0]:         s_axil_rdata <= status_reg;
                    REG_UCODE_BASE[7:0]:     s_axil_rdata <= reg_ucode_base;
                    REG_UCODE_LEN[7:0]:      s_axil_rdata <= reg_ucode_len;
                    REG_DDR_BASE_ACT[7:0]:   s_axil_rdata <= reg_ddr_base_act;
                    REG_DDR_BASE_WGT[7:0]:   s_axil_rdata <= reg_ddr_base_wgt;
                    REG_DDR_BASE_KV[7:0]:    s_axil_rdata <= reg_ddr_base_kv;
                    REG_DDR_BASE_OUT[7:0]:   s_axil_rdata <= reg_ddr_base_out;
                    REG_MODEL_HIDDEN[7:0]:   s_axil_rdata <= reg_model_hidden;
                    REG_MODEL_HEADS[7:0]:    s_axil_rdata <= reg_model_heads;
                    REG_MODEL_HEAD_DIM[7:0]: s_axil_rdata <= reg_model_head_dim;
                    REG_SEQ_LEN[7:0]:        s_axil_rdata <= reg_seq_len;
                    REG_TOKEN_IDX[7:0]:      s_axil_rdata <= reg_token_idx;
                    REG_DEBUG_CTRL[7:0]:     s_axil_rdata <= reg_debug_ctrl;
                    REG_EXEC_MODE[7:0]:      s_axil_rdata <= reg_exec_mode;
                    REG_GRAPH_STATUS[7:0]:   s_axil_rdata <= graph_status_i;
                    REG_GRAPH_PC[7:0]:       s_axil_rdata <= graph_pc_i;
                    REG_GRAPH_LAST_OP[7:0]:  s_axil_rdata <= graph_last_op_i;
                    REG_DDR_BASE_SCALE[7:0]: s_axil_rdata <= reg_ddr_base_scale;
                    REG_CFG_M[7:0]:          s_axil_rdata <= reg_cfg_M;
                    REG_CFG_N[7:0]:          s_axil_rdata <= reg_cfg_N;
                    REG_CFG_K[7:0]:          s_axil_rdata <= reg_cfg_K;
                    REG_HOST_CMD[7:0]:       s_axil_rdata <= reg_host_cmd;
                    REG_TRAP_STATUS[7:0]:    s_axil_rdata <= reg_trap_status;
                    REG_UCODE_LOAD_ADDR[7:0]:s_axil_rdata <= reg_ucode_load_addr;
                    REG_UCODE_LOAD_LEN[7:0]: s_axil_rdata <= reg_ucode_load_len;
                    REG_RESUME_PC[7:0]:      s_axil_rdata <= reg_resume_pc;
                    REG_ACC_FLUSH_ADDR[7:0]: s_axil_rdata <= reg_acc_flush_addr;
                    REG_ACC_FLUSH_LEN[7:0]:  s_axil_rdata <= reg_acc_flush_len;
                    REG_ACC_ACCUM[7:0]:      s_axil_rdata <= reg_acc_accum;
                    REG_DEQUANT_CFG[7:0]:    s_axil_rdata <= reg_deq_nbits_code;
                    REG_DEQUANT_SCALE_LEN[7:0]: s_axil_rdata <= reg_deq_scale_len;
                    REG_EDESC_IDX[7:0]:      s_axil_rdata <= {22'd0, reg_edesc_idx};
                    REG_EDESC_W0[7:0]:       s_axil_rdata <= reg_edesc_wdata;
                    REG_EDESC_W1[7:0]:       s_axil_rdata <= reg_edesc_wdata;
                    REG_EDESC_W2[7:0]:       s_axil_rdata <= reg_edesc_wdata;
                    default:                 s_axil_rdata <= 32'hDEAD_BEEF;
                endcase
            end

            // De-assert RVALID when master accepts the data
            if (s_axil_rvalid && s_axil_rready)
                s_axil_rvalid <= 1'b0;
        end
    end

    // =======================================================================
    //  Output assignments
    // =======================================================================
    assign ctrl_o           = reg_ctrl;
    assign status_o         = status_reg;
    assign ucode_base_o     = reg_ucode_base;
    assign ucode_len_o      = reg_ucode_len;
    assign ddr_base_act_o   = reg_ddr_base_act;
    assign ddr_base_wgt_o   = reg_ddr_base_wgt;
    assign ddr_base_kv_o    = reg_ddr_base_kv;
    assign ddr_base_out_o   = reg_ddr_base_out;
    assign model_hidden_o   = reg_model_hidden;
    assign model_heads_o    = reg_model_heads;
    assign model_head_dim_o = reg_model_head_dim;
    assign seq_len_o        = reg_seq_len;
    assign token_idx_o      = reg_token_idx;
    assign debug_ctrl_o     = reg_debug_ctrl;
    assign exec_mode_o      = reg_exec_mode;
    assign ddr_base_scale_o = reg_ddr_base_scale;
    assign cfg_M_o          = reg_cfg_M[15:0];
    assign cfg_N_o          = reg_cfg_N[15:0];
    assign cfg_K_o          = reg_cfg_K[15:0];

    // Host-in-the-loop MoE register outputs
    assign host_cmd_o         = reg_host_cmd;
    assign trap_status_o      = reg_trap_status;
    assign ucode_load_addr_o  = reg_ucode_load_addr;
    assign ucode_load_len_o   = reg_ucode_load_len;
    assign resume_pc_o        = reg_resume_pc;
    assign acc_flush_addr_o  = reg_acc_flush_addr;
    assign acc_flush_len_o   = reg_acc_flush_len;
    assign acc_accum_o       = reg_acc_accum[0];
    assign deq_nbits_code_o  = reg_deq_nbits_code[2:0];
    assign deq_scale_len_o   = reg_deq_scale_len;

    // Expert precision table write port (address = exp_idx*3 + proj)
    assign expert_prec_wr_en_o   = reg_expert_prec_wr_stb;
    assign expert_prec_wr_addr_o = {reg_expert_prec_entry[9:4]} * 3'd3
                                   + reg_expert_prec_entry[3:2];
    assign expert_prec_wr_data_o = reg_expert_prec_entry[1:0];

    // Expert descriptor table write port (address = exp_idx*3 + proj)
    assign edesc_wr_en_o   = reg_edesc_wr_stb;
    assign edesc_wr_addr_o = reg_edesc_idx;
    assign edesc_wr_data_o = reg_edesc_wdata;
    assign edesc_wr_sel_o  = reg_edesc_sel;

    // START is a single-cycle pulse derived from the register bit
    assign start_pulse_o = reg_ctrl[CTRL_START];

    // SOFT_RESET stays high while the counter is active
    assign soft_reset_o  = reg_ctrl[CTRL_SOFT_RESET];

endmodule : axi_lite_regs
