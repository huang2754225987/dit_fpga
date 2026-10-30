// =============================================================================
// tb_supra2_block_top.sv - Connected one-block datapath testbench
//
// Loads staged one-block fixtures (input tensor, packed context, AdaLN
// parameters, seven INT8 weight sets) into supra2_block_top through the
// DMA-model load ports, runs the full block (self-attn, masked cross-attn,
// gated MLP) as one connected on-chip datapath, dumps the final residual
// tensor, compares it against the supplied fixture reference, and reports
// total and per-stage
// busy cycles.
//
// Plusargs: DIR=<fixture dir> ACTUAL_FILE=<dump path>
// =============================================================================
`default_nettype none

module tb_supra2_block_top #(parameter bit USE_TILE_PROGRAM=1'b0);
    localparam int XN = 256*576;
    localparam int CTXN = 32*576;
    localparam int PN = 8*576;
    localparam int MAXW = 2304*576;   // max weight elements (mlp0/mlp2)
    localparam int MAXN = 2304;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic start = 1'b0;
    logic busy, done;
    logic ld_en = 1'b0;
    logic [3:0] ld_target = '0;
    logic [20:0] ld_addr = '0;
    logic [15:0] ld_data = '0;
    logic wt_wr_en = 1'b0;
    logic [15:0] wt_wr_beat = '0;
    logic [255:0] wt_wr_data = '0;
    logic sb_wr_en = 1'b0;
    logic sb_wr_sel = 1'b0;
    logic [15:0] sb_wr_addr = '0;
    logic [31:0] sb_wr_data = '0;
    logic wload_req;
    logic [3:0] wload_stage;
    logic wload_ack = 1'b0;
    logic [4:0] stage_id;
    wire tile_cmd_valid, tile_cmd_ready, tile_cmd_done, tile_program_done, tile_program_error;
    wire [127:0] tile_cmd_word;
    wire [15:0] tile_program_counter;

    generate if (USE_TILE_PROGRAM) begin : g_tile_program
        supra2_tile_program_sequencer #(
            .PROGRAM_FILE("build/supra2_tile_program/block13_tile_program.hex"),
            .DEPENDENCY_FILE("build/supra2_tile_program/block13_tile_dependencies.hex")
        ) u_tile_program (
            .clk(clk),.rst_n(rst_n),.start(start),
            .command_valid(tile_cmd_valid),.command_ready(tile_cmd_ready),
            .command_word(tile_cmd_word),.command_done(tile_cmd_done),
            .program_done(tile_program_done),.program_error(tile_program_error),
            .program_counter(tile_program_counter));
    end else begin : g_no_tile_program
        assign tile_cmd_valid=1'b0;
        assign tile_cmd_word='0;
        assign tile_program_done=1'b0;
        assign tile_program_error=1'b0;
        assign tile_program_counter='0;
    end endgenerate

    supra2_block_top #(.PROGRAMMED_TILES(USE_TILE_PROGRAM)) dut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .start_stage(5'd0), .start_head(4'd0), .start_ni(16'd0),
        .stop_ni(16'd0),
        .busy(busy), .done(done),
        .stage_cmd_valid(1'b0),.stage_cmd_id(5'd0),.stage_cmd_ready(),
        .tile_cmd_valid(tile_cmd_valid),.tile_cmd_word(tile_cmd_word),
        .tile_cmd_ready(tile_cmd_ready),.tile_cmd_done(tile_cmd_done),
        .ld_en(ld_en), .ld_target(ld_target), .ld_addr(ld_addr),
        .ld_data(ld_data),
        .ld_vec_en(1'b0), .ld_vec_target(4'd0), .ld_vec_beat(16'd0),
        .ld_vec_data(512'd0),
        .wt_wr_en(wt_wr_en), .wt_wr_beat(wt_wr_beat), .wt_wr_data(wt_wr_data),
        .sb_wr_en(sb_wr_en), .sb_wr_sel(sb_wr_sel), .sb_wr_addr(sb_wr_addr),
        .sb_wr_data(sb_wr_data),
        .result_vec_rd_beat(13'd0), .result_vec_rd_data(),
        .result_rd_beat(15'd0), .result_rd_data(),
        .param_vec_wr_en(1'b0), .param_vec_wr_beat(8'd0), .param_vec_wr_data(512'd0),
        .wload_req(wload_req), .wload_stage(wload_stage),
        .wload_ack(wload_ack), .stage_id(stage_id)
    );

    always #5 clk = ~clk;

    // ---------------- fixture memories ----------------
    logic [15:0] x_mem [0:XN-1];
    logic [15:0] ctx_mem [0:CTXN-1];
    logic [15:0] param_mem [0:PN-1];
    logic [15:0] golden_mem [0:XN-1];
    // Reuse one stage-sized weight/scale/bias fixture buffer. Keeping all seven
    // matrices resident multiplies XSim's unpacked-array overhead and caused
    // the full-block run to exhaust memory before the final comparison.
    logic [7:0]  wtmp [0:MAXW-1];
    logic [31:0] stmp [0:MAXN-1];

    // per-stage GEMM sizes: {M, N, K}
    localparam int STG_N [0:6] = '{1728, 576, 576, 1152, 576, 2304, 576};
    localparam int STG_K [0:6] = '{576, 576, 576, 576, 576, 576, 2304};
    string stage_name [0:6] = '{"qkv", "sproj", "cq", "ckv", "cproj",
                                "mlp0", "mlp2"};

    string dir, actual_file, probe_dir;
    integer total_cycles;
    integer dump_probes;
    integer dump_scores;
    integer stop_after_stage;
    integer load_only;
    integer block_id;
    integer stage_mark;
    integer stage_cycles [0:20];
    logic [4:0] prev_stage;
    integer i, j, s, fd;
    integer out_fd;
    integer tile_stats_fd, tile_cmd_start_cycle, tile_cmd_active;
    integer tile_cmd_opcode, tile_cmd_stage, tile_cmd_row, tile_cmd_head;
    integer tile_cmd_count;
    integer tile_heartbeat_fd;
    integer tile_engine_busy_cycles, tile_array_enable_cycles;
    integer tile_hadamard_backpressure_cycles, tile_gemm_output_beats;
    integer tile_ln_output_beats, tile_residual_output_beats, tile_gelu_output_beats;
    integer tile_probe_index;
    logic [15:0] tile_probe_value;
    integer qkv_first_out_logged;
    integer qkv_first_array_logged;

    // Watchdog diagnostic (2026-10-21): dump all tensor buffers at every
    // stage boundary to localize the 14.85% rel-to-peak block failure.
    // Read-only probes; no effect on DUT behaviour. File tag = ENTERED stage
    // id, so enter_NN_* captures the buffer state left by stage NN-1.
    task automatic dump_probe(input integer sid);
        string p;
        begin
            p = $sformatf("%s/enter_%02d_", probe_dir, sid);
            $writememh({p, "x_buf.hex"},    dut.x_buf);
            $writememh({p, "ln_buf.hex"},   dut.ln_buf);
            $writememh({p, "q_buf.hex"},    dut.q_probe_buf);
            $writememh({p, "k_buf.hex"},    dut.k_probe_buf);
            $writememh({p, "v_buf.hex"},    dut.v_buf);
            $writememh({p, "cq_buf.hex"},   dut.cq_probe_buf);
            $writememh({p, "ck_buf.hex"},   dut.ck_probe_buf);
            $writememh({p, "cv_buf.hex"},   dut.cv_buf);
            $writememh({p, "attn_buf.hex"}, dut.attn_buf);
            $writememh({p, "proj_buf.hex"}, dut.proj_buf);
            $writememh({p, "mlp_buf.hex"},  dut.mlp_buf);
        end
    endtask

    // cycle counting + stage accounting
    always @(posedge clk) begin
        if (busy) total_cycles <= total_cycles + 1;
        if (USE_TILE_PROGRAM && stage_id == 1 && dut.eng_out_valid &&
            !qkv_first_out_logged) begin
            qkv_first_out_logged <= 1;
            $display("SUPRA2_QKV_FIRST_OUT row=%0d ct=%0d data=%04h raw=%04h acc=%08h token_scale=%08h scale=%08h bias=%08h",
                dut.eng_out_row, dut.eng_out_ct, dut.eng_out_data[15:0],
                dut.eng_raw_data[15:0], dut.u_eng.epi_acc_bus[31:0],
                dut.u_eng.epi_token_scale, dut.u_eng.scale_bank_rdata[0],
                dut.u_eng.bias_bank_rdata[0]);
        end
        if (USE_TILE_PROGRAM && stage_id == 1 && dut.eng_array_en &&
            dut.u_eng.state == 5 && !qkv_first_array_logged) begin
            qkv_first_array_logged <= 1;
            $display("SUPRA2_QKV_FIRST_ARRAY raw=%04h h=%04h q=%02h actbank=%08h wtbank=%08h a=%02h b=%02h wtmem=%08h ws_mirror=%08h bias_mirror=%08h ws_mem=%08h",
                dut.eng_raw_data[15:0], dut.u_eng.fe_h_out_data[15:0],
                dut.u_eng.fe_q_out_data[7:0], dut.u_eng.act_bank_rdata[0][7:0],
                dut.u_eng.wt_bank_rdata[0][7:0], dut.eng_array_a[0],
                dut.eng_array_b[0],
                dut.u_eng.gen_operand_ram_banks[0].weight_ram.mem[0][31:0],
                dut.u_eng.ws_fp32[0], dut.u_eng.bias_fp32[0],
                dut.u_eng.gen_operand_ram_banks[0].scale_ram.mem[0]);
        end
        if (USE_TILE_PROGRAM && busy && total_cycles != 0 &&
            total_cycles % 100000 == 0 && tile_heartbeat_fd != 0) begin
            $fdisplay(tile_heartbeat_fd,
                "cycle=%0d pc=%0d stage=%0d ss=%0d eng_state=%0d row=%0d beat=%0d",
                total_cycles, tile_program_counter, stage_id, dut.ss,
                dut.u_eng.state, dut.u_eng.row, dut.u_eng.beat);
            $fflush(tile_heartbeat_fd);
        end
        if (USE_TILE_PROGRAM && tile_cmd_valid && tile_cmd_ready) begin
            tile_cmd_start_cycle <= total_cycles;
            tile_cmd_active <= 1;
            tile_engine_busy_cycles = 0;
            tile_array_enable_cycles = 0;
            tile_hadamard_backpressure_cycles = 0;
            tile_gemm_output_beats = 0;
            tile_ln_output_beats = 0;
            tile_residual_output_beats = 0;
            tile_gelu_output_beats = 0;
            tile_cmd_opcode <= tile_cmd_word[127:120];
            tile_cmd_stage <= tile_cmd_word[119:112];
            tile_cmd_row <= tile_cmd_word[111:104];
            tile_cmd_head <= tile_cmd_word[99:96];
        end
        if (USE_TILE_PROGRAM && tile_cmd_active) begin
            if (dut.eng_busy) tile_engine_busy_cycles = tile_engine_busy_cycles + 1;
            if (dut.eng_array_en) tile_array_enable_cycles = tile_array_enable_cycles + 1;
            if (dut.u_eng.fe_h_busy && !dut.u_eng.fe_h_in_ready)
                tile_hadamard_backpressure_cycles = tile_hadamard_backpressure_cycles + 1;
            if (dut.eng_out_valid) tile_gemm_output_beats = tile_gemm_output_beats + 1;
            if (dut.ln_out_valid) tile_ln_output_beats = tile_ln_output_beats + 1;
            if (dut.res_out_valid) tile_residual_output_beats = tile_residual_output_beats + 1;
            if (dut.gelu_out_valid) tile_gelu_output_beats = tile_gelu_output_beats + 1;
        end
        if (USE_TILE_PROGRAM && tile_cmd_active && tile_cmd_done) begin
            tile_probe_index = 0;
            tile_probe_value = '0;
            case (tile_cmd_opcode)
                8'd1: begin
                    $display("SUPRA2_WEIGHT_PROBE pc=%0d stage=%0d wt=%02h scale=%08h bias=%08h wt_ram=%08h scale_ram=%08h bias_ram=%08h",
                        tile_program_counter, tile_cmd_stage,
                        dut.u_eng.wt_int8[0], dut.u_eng.ws_fp32[0],
                        dut.u_eng.bias_fp32[0],
                        dut.u_eng.gen_operand_ram_banks[0].weight_ram.mem[0][31:0],
                        dut.u_eng.gen_operand_ram_banks[0].scale_ram.mem[0],
                        dut.u_eng.gen_operand_ram_banks[0].bias_ram.mem[0]);
                    if ($isunknown(dut.u_eng.wt_int8[0]) ||
                        $isunknown(dut.u_eng.ws_fp32[0]) ||
                        $isunknown(dut.u_eng.bias_fp32[0]) ||
                        $isunknown(dut.u_eng.gen_operand_ram_banks[0].weight_ram.mem[0]) ||
                        $isunknown(dut.u_eng.gen_operand_ram_banks[0].scale_ram.mem[0]) ||
                        $isunknown(dut.u_eng.gen_operand_ram_banks[0].bias_ram.mem[0]))
                        $fatal(1, "DMA_WEIGHT did not initialize the engine banks pc=%0d stage=%0d",
                            tile_program_counter, tile_cmd_stage);
                end
                8'd2: begin
                    case (tile_cmd_stage)
                        1: begin
                            tile_probe_index = tile_cmd_row*32*64;
                            tile_probe_value = dut.q_probe_buf[tile_probe_index];
                        end
                        3, 9, 14: begin
                            tile_probe_index = tile_cmd_row*32*576;
                            tile_probe_value = dut.proj_buf[tile_probe_index];
                        end
                        6: begin
                            tile_probe_index = tile_cmd_row*32*64;
                            tile_probe_value = dut.cq_probe_buf[tile_probe_index];
                        end
                        7: tile_probe_value = dut.ck_probe_buf[0];
                        12: begin
                            tile_probe_index = tile_cmd_row*32*2304;
                            tile_probe_value = dut.mlp_buf[tile_probe_index];
                        end
                        default: tile_probe_value = '0;
                    endcase
                end
                8'd3: begin
                    tile_probe_index = (tile_cmd_head*256 + tile_cmd_row*32)*64;
                    tile_probe_value = dut.attn_buf[tile_probe_index];
                end
                8'd4: begin
                    tile_probe_index = tile_cmd_row*32*576;
                    tile_probe_value = dut.ln_buf[tile_probe_index];
                end
                8'd5: begin
                    tile_probe_index = tile_cmd_row*32*576;
                    tile_probe_value = dut.x_buf[tile_probe_index];
                end
                8'd6: begin
                    tile_probe_index = tile_cmd_row*32*2304;
                    tile_probe_value = dut.mlp_buf[tile_probe_index];
                end
                default: tile_probe_value = '0;
            endcase
            if (tile_cmd_opcode >= 2 && tile_cmd_opcode <= 6) begin
                $display("SUPRA2_TILE_PROBE pc=%0d op=%0d stage=%0d row=%0d head_n=%0d index=%0d value=%04h",
                    tile_program_counter, tile_cmd_opcode, tile_cmd_stage,
                    tile_cmd_row, tile_cmd_head, tile_probe_index, tile_probe_value);
                if ($isunknown(tile_probe_value))
                    $fatal(1, "Tile produced X/Z sample pc=%0d op=%0d stage=%0d row=%0d head_n=%0d index=%0d",
                        tile_program_counter, tile_cmd_opcode, tile_cmd_stage,
                        tile_cmd_row, tile_cmd_head, tile_probe_index);
            end
            $fdisplay(tile_stats_fd, "%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
                tile_program_counter, tile_cmd_opcode, tile_cmd_stage,
                tile_cmd_row, tile_cmd_head,
                total_cycles - tile_cmd_start_cycle, total_cycles,
                tile_engine_busy_cycles, tile_array_enable_cycles,
                tile_hadamard_backpressure_cycles, tile_gemm_output_beats,
                tile_ln_output_beats, tile_residual_output_beats,
                tile_gelu_output_beats);
            $fflush(tile_stats_fd);
            tile_cmd_active <= 0;
            tile_cmd_count <= tile_cmd_count + 1;
        end
        if (busy && total_cycles != 0 && total_cycles % 20000 == 0)
            $display("PROBE cyc=%0d stage=%0d ss=%0d eng_state=%0d row=%0d beat=%0d mi=%0d ni=%0d ob=%0d head=%0d c1=%0d c2=%0d c3=%0d h_state=%0d h_in=%0d h_out=%0d h_base=%0d h_ready=%0d h_valid=%0d q_state=%0d q_in=%0d q_out=%0d q_ready=%0d q_valid=%0d",
                     total_cycles, stage_id, dut.ss, dut.u_eng.state,
                     dut.u_eng.row, dut.u_eng.beat, dut.u_eng.mi, dut.u_eng.ni,
                     dut.u_eng.obeat, dut.cur_head, dut.c1, dut.c2, dut.c3,
                     dut.u_eng.u_hadamard.state,
                     dut.u_eng.u_hadamard.input_count,
                     dut.u_eng.u_hadamard.output_count,
                     dut.u_eng.u_hadamard.vector_base,
                     dut.u_eng.fe_h_in_ready, dut.u_eng.fe_h_out_valid,
                     dut.u_eng.u_quantizer.state,
                     dut.u_eng.u_quantizer.input_count,
                     dut.u_eng.u_quantizer.output_count,
                     dut.u_eng.fe_q_in_ready, dut.u_eng.fe_q_out_valid);
        if (stage_id != prev_stage) begin
            stage_cycles[prev_stage] <= total_cycles - stage_mark;
            stage_mark <= total_cycles;
            prev_stage <= stage_id;
            $display("SUPRA2_BLOCK_STAGE_ENTER id=%0d cycle=%0d",
                     stage_id, total_cycles);
            if (dump_probes) dump_probe(stage_id);
            if (dump_scores && stage_id == 3)
                $writememh($sformatf("%s/score_self_int8.hex", probe_dir), dut.score_probe_buf);
            if (dump_scores && stage_id == 9)
                $writememh($sformatf("%s/score_cross_int8.hex", probe_dir), dut.cscore_probe_buf);
        end
    end

    function automatic real half_to_real(input logic [15:0] value);
        integer exponent;
        real significand, result;
        begin
            exponent = value[14:10];
            if (exponent == 0) significand = value[9:0] / 1024.0;
            else significand = 1.0 + value[9:0] / 1024.0;
            if (exponent == 0) result = significand * (2.0 ** -14);
            else if (exponent == 31) result = 0.0;
            else result = significand * (2.0 ** (exponent - 15));
            half_to_real = value[15] ? -result : result;
        end
    endfunction

    task automatic load_fp16(input string fname, input logic [3:0] target,
                             input int count, input int param_offset);
        logic [15:0] tmp [0:XN-1];
        begin
            $readmemh(fname, tmp, 0, count-1);
            for (int e = 0; e < count; e++) begin
                @(negedge clk);
                ld_en = 1'b1;
                ld_target = target;
                ld_addr = 21'(param_offset + e);
                ld_data = tmp[e];
            end
            @(negedge clk);
            ld_en = 1'b0;
        end
    endtask

`define LOAD_STAGE_WEIGHT_SCALE(wname, sname) \
    begin \
        $readmemh($sformatf("%s/%s", dir, wname), wtmp, 0, n_sz*k_sz-1); \
        $readmemh($sformatf("%s/%s", dir, sname), stmp, 0, n_sz-1); \
    end
`define LOAD_STAGE_BIAS(bname) \
    $readmemh($sformatf("%s/%s", dir, bname), stmp, 0, n_sz-1)

    // Weight-load responder (runs concurrently with the main flow). Fixture
    // files are read only when their stage is requested, then streamed through
    // the modeled DMA port before the shared buffers are reused.
    initial begin : wload_server
        integer beats, n_sz, k_sz;
        forever begin
            wait (wload_req === 1'b1);
            s = wload_stage;
            n_sz = STG_N[s];
            k_sz = STG_K[s];
            beats = n_sz*k_sz/32;
            case (s)
                0: `LOAD_STAGE_WEIGHT_SCALE("w_qkv_int8.hex", "ws_qkv_fp32.hex")
                1: `LOAD_STAGE_WEIGHT_SCALE("w_sproj_int8.hex", "ws_sproj_fp32.hex")
                2: `LOAD_STAGE_WEIGHT_SCALE("w_cq_int8.hex", "ws_cq_fp32.hex")
                3: `LOAD_STAGE_WEIGHT_SCALE("w_ckv_int8.hex", "ws_ckv_fp32.hex")
                4: `LOAD_STAGE_WEIGHT_SCALE("w_cproj_int8.hex", "ws_cproj_fp32.hex")
                5: `LOAD_STAGE_WEIGHT_SCALE("w_mlp0_int8.hex", "ws_mlp0_fp32.hex")
                6: `LOAD_STAGE_WEIGHT_SCALE("w_mlp2_int8.hex", "ws_mlp2_fp32.hex")
                default: $fatal(1, "Invalid block weight stage %0d", s);
            endcase
            if ($isunknown(wtmp[0]) || $isunknown(wtmp[n_sz*k_sz-1]) ||
                $isunknown(stmp[0]) || $isunknown(stmp[n_sz-1]))
                $fatal(1, "Block weight/scale fixture load failed at stage %0d", s);
            for (i = 0; i < beats; i++) begin
                @(negedge clk);
                wt_wr_en = 1'b1;
                wt_wr_beat = 16'(i);
                for (j = 0; j < 32; j++)
                    wt_wr_data[j*8 +: 8] = wtmp[i*32 + j];
            end
            @(negedge clk);
            wt_wr_en = 1'b0;
            for (i = 0; i < n_sz; i++) begin
                @(negedge clk);
                sb_wr_en = 1'b1;
                sb_wr_sel = 1'b0;
                sb_wr_addr = 16'(i);
                sb_wr_data = stmp[i];
            end
            case (s)
                0: `LOAD_STAGE_BIAS("b_qkv_fp32.hex");
                1: `LOAD_STAGE_BIAS("b_sproj_fp32.hex");
                2: `LOAD_STAGE_BIAS("b_cq_fp32.hex");
                3: `LOAD_STAGE_BIAS("b_ckv_fp32.hex");
                4: `LOAD_STAGE_BIAS("b_cproj_fp32.hex");
                5: `LOAD_STAGE_BIAS("b_mlp0_fp32.hex");
                6: `LOAD_STAGE_BIAS("b_mlp2_fp32.hex");
                default: $fatal(1, "Invalid block bias stage %0d", s);
            endcase
            if ($isunknown(stmp[0]) || $isunknown(stmp[n_sz-1]))
                $fatal(1, "Block bias fixture load failed at stage %0d", s);
            for (i = 0; i < n_sz; i++) begin
                @(negedge clk);
                sb_wr_en = 1'b1;
                sb_wr_sel = 1'b1;
                sb_wr_addr = 16'(i);
                sb_wr_data = stmp[i];
            end
            @(negedge clk);
            sb_wr_en = 1'b0;
            @(negedge clk);
            wload_ack = 1'b1;
            @(negedge clk);
            wload_ack = 1'b0;
            wait (wload_req === 1'b0);
        end
    end

    // ------------------------------------------------------------------
    // Questa supports runtime fixture paths; keep each block's fixtures
    // isolated under its own build directory.
`define BLOCK_LOAD_PARAM(idx, name) \
    $readmemh($sformatf("%s/%s", dir, name), ptmp, 0, 575); \
    for (i = 0; i < 576; i++) param_mem[idx*576 + i] = ptmp[i];
    initial begin
        dump_probes = 0;
        void'($value$plusargs("DUMP_PROBES=%d", dump_probes));
        dump_scores = 0;
        void'($value$plusargs("DUMP_SCORES=%d", dump_scores));
        stop_after_stage = -1;
        void'($value$plusargs("STOP_AFTER_STAGE=%d", stop_after_stage));
        load_only = 0;
        void'($value$plusargs("LOAD_ONLY=%d", load_only));
        block_id = 0;
        void'($value$plusargs("BLOCK_ID=%d", block_id));
        if (!$value$plusargs("DIR=%s", dir))
            dir = "build/supra2_block7_top";
        probe_dir = $sformatf("%s/probe", dir);
        void'($value$plusargs("PROBE_DIR=%s", probe_dir));
        if (!$value$plusargs("ACTUAL_FILE=%s", actual_file))
            actual_file = $sformatf("%s/rtl_actual_fp16.hex", dir);

        $readmemh($sformatf("%s/x_input.hex", dir), x_mem, 0, XN-1);
        $readmemh($sformatf("%s/ctx_packed.hex", dir), ctx_mem, 0, CTXN-1);
        $readmemh($sformatf("%s/golden_y.hex", dir), golden_mem, 0, XN-1);
        // AdaLN params into param_buf order
        begin
            logic [15:0] ptmp [0:575];
            `BLOCK_LOAD_PARAM(0, "ln0_scale.hex")
            `BLOCK_LOAD_PARAM(1, "ln0_shift.hex")
            `BLOCK_LOAD_PARAM(2, "ln1_scale.hex")
            `BLOCK_LOAD_PARAM(3, "ln1_shift.hex")
            `BLOCK_LOAD_PARAM(4, "ln2_scale.hex")
            `BLOCK_LOAD_PARAM(5, "ln2_shift.hex")
            `BLOCK_LOAD_PARAM(6, "gate0.hex")
            `BLOCK_LOAD_PARAM(7, "gate1.hex")
        end
        // fail fast at time 0 if any fixture failed to load
        if ($isunknown(x_mem[0]) || $isunknown(x_mem[XN-1]) ||
            $isunknown(ctx_mem[0]) || $isunknown(ctx_mem[CTXN-1]) ||
            $isunknown(golden_mem[0]) || $isunknown(golden_mem[XN-1]) ||
            $isunknown(param_mem[0]) || $isunknown(param_mem[PN-1]))
            $fatal(1, "BLOCK_TOP fixture load failed: input memory has X/Z");

        total_cycles = 0;
        stage_mark = 0;
        prev_stage = 0;
        for (i = 0; i < 21; i++) stage_cycles[i] = 0;
        tile_cmd_start_cycle = 0;
        tile_cmd_active = 0;
        tile_cmd_count = 0;
        qkv_first_out_logged = 0;
        qkv_first_array_logged = 0;
        tile_stats_fd = 0;
        tile_heartbeat_fd = 0;
        if (USE_TILE_PROGRAM) begin
            string tile_stats_path, tile_heartbeat_path;
            if (!$value$plusargs("TILE_STATS=%s", tile_stats_path))
                tile_stats_path = $sformatf("%s/tile_instruction_stats.csv", dir);
            tile_stats_fd = $fopen(tile_stats_path, "w");
            if (!tile_stats_fd) $fatal(1, "Cannot open TILE_STATS output");
            $fdisplay(tile_stats_fd,
                "pc,opcode,stage,row_tile,head,execution_cycles,retire_cycle,engine_busy_cycles,array_enable_cycles,hadamard_backpressure_cycles,gemm_output_beats,ln_output_beats,residual_output_beats,gelu_output_beats");
            if (!$value$plusargs("TILE_HEARTBEAT=%s", tile_heartbeat_path))
                tile_heartbeat_path = $sformatf("%s/tile_heartbeat.log", dir);
            tile_heartbeat_fd = $fopen(tile_heartbeat_path, "w");
            if (!tile_heartbeat_fd) $fatal(1, "Cannot open TILE_HEARTBEAT output");
            $fdisplay(tile_heartbeat_fd, "cycle,pc,stage,ss,eng_state,row,beat");
            $fflush(tile_heartbeat_fd);
        end

        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(negedge clk);

        // DMA-model loads: x input, packed context, AdaLN params
        for (i = 0; i < XN; i++) begin
            @(negedge clk);
            ld_en = 1'b1; ld_target = 4'd0;
            ld_addr = 21'(i); ld_data = x_mem[i];
        end
        for (i = 0; i < CTXN; i++) begin
            @(negedge clk);
            ld_en = 1'b1; ld_target = 4'd1;
            ld_addr = 21'(i); ld_data = ctx_mem[i];
        end
        for (i = 0; i < PN; i++) begin
            @(negedge clk);
            ld_en = 1'b1; ld_target = 4'd2;
            ld_addr = 21'(i); ld_data = param_mem[i];
        end
        @(negedge clk);
        ld_en = 1'b0;
        repeat (2) @(negedge clk);

        if (load_only != 0) begin
            $display("SUPRA2_FIXTURE_LOAD x0=%04h xlast=%04h ram_x0=%04h ctx0=%04h param0=%04h",
                dut.x_buf[0], dut.x_buf[XN-1], dut.u_x_ram.mem[0][15:0],
                dut.ctx_buf[0], dut.param_buf[0]);
            if (dut.x_buf[0] !== x_mem[0] ||
                dut.x_buf[XN-1] !== x_mem[XN-1] ||
                dut.u_x_ram.mem[0][15:0] !== x_mem[0] ||
                dut.ctx_buf[0] !== ctx_mem[0] ||
                dut.param_buf[0] !== param_mem[0])
                $fatal(1, "Fixture load did not reach RTL mirrors and physical RAM");
            $display("SUPRA2_FIXTURE_LOAD_PASS");
            $finish;
        end

        // run the block
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        fork : watchdog
            begin
                wait ((USE_TILE_PROGRAM && tile_program_done) ||
                      (!USE_TILE_PROGRAM && done) ||
                      (stop_after_stage >= 0 && stage_id > stop_after_stage));
            end
            begin
                repeat (12_000_000) @(posedge clk);
                $fatal(1, "BLOCK_TOP timeout: stage=%0d ss stuck", stage_id);
            end
        join_any
        disable watchdog;

        if (!done && stop_after_stage >= 0) begin
            $display("SUPRA2_BLOCK_TOP_STOP_AFTER_STAGE stage=%0d cycles=%0d",
                     stop_after_stage, total_cycles);
            $finish;
        end

        if (USE_TILE_PROGRAM) begin
            if (tile_program_error || !tile_program_done)
                $fatal(1, "Tile instruction program failed: done=%0b error=%0b pc=%0d",
                    tile_program_done, tile_program_error, tile_program_counter);
            if (tile_cmd_active) $fatal(1, "Tile command was not retired");
            $fclose(tile_stats_fd);
            $display("SUPRA2_TILE_PROGRAM_PASS commands=%0d cycles=%0d",
                tile_cmd_count, total_cycles);
        end

        stage_cycles[prev_stage] = total_cycles - stage_mark;

        // dump final residual tensor
        out_fd = $fopen(actual_file, "w");
        if (!out_fd) $fatal(1, "Cannot open actual output file");
        for (i = 0; i < XN; i++)
            $fdisplay(out_fd, "%04h", dut.x_buf[i]);
        $fclose(out_fd);

        // compare against archive golden b7.y
        begin
            real got, exp_v, abs_err, max_abs, peak;
            real sum_sq, ref_sq, rmse, rel_l2, rel_peak;
            integer bit_diff;
            integer x_words;
            // An X-poisoned tensor must never pass: half_to_real collapses
            // unknown bits to a fixed value, which made an all-X output
            // compare "equal" to an all-X golden (vacuous PASS, 2026-10-21).
            x_words = 0;
            for (i = 0; i < XN; i++)
                if ($isunknown(dut.x_buf[i]) || $isunknown(golden_mem[i]))
                    x_words++;
            $display("SUPRA2_BLOCK_TOP_X_WORDS %0d", x_words);
            if (x_words > 0)
                $fatal(1, "BLOCK_TOP output/golden contains %0d X/Z words; result invalid", x_words);
            max_abs = 0.0; peak = 0.0; sum_sq = 0.0; ref_sq = 0.0;
            bit_diff = 0;
            for (i = 0; i < XN; i++) begin
                got = half_to_real(dut.x_buf[i]);
                exp_v = half_to_real(golden_mem[i]);
                abs_err = (got >= exp_v) ? got - exp_v : exp_v - got;
                if (abs_err > max_abs) max_abs = abs_err;
                if ((exp_v >= 0 ? exp_v : -exp_v) > peak)
                    peak = (exp_v >= 0) ? exp_v : -exp_v;
                sum_sq = sum_sq + abs_err*abs_err;
                ref_sq = ref_sq + exp_v*exp_v;
                if (dut.x_buf[i] !== golden_mem[i]) bit_diff++;
            end
            rmse = $sqrt(sum_sq / XN);
            rel_l2 = $sqrt(sum_sq / ((ref_sq > 1.0e-20) ? ref_sq : 1.0));
            rel_peak = max_abs / ((peak > 1.0e-20) ? peak : 1.0);
            $display("SUPRA2_BLOCK_TOP_CYCLES %0d", total_cycles);
            for (i = 0; i < 16; i++)
                $display("SUPRA2_BLOCK_STAGE_CYCLES id=%0d cycles=%0d",
                         i, stage_cycles[i]);
            $display("SUPRA2_BLOCK_TOP_MAX_ABS %.9g", max_abs);
            $display("SUPRA2_BLOCK_TOP_REL_TO_PEAK %.9g", rel_peak);
            $display("SUPRA2_BLOCK_TOP_RMSE %.9g", rmse);
            $display("SUPRA2_BLOCK_TOP_REL_L2 %.9g", rel_l2);
            $display("SUPRA2_BLOCK_TOP_BIT_DIFF %0d", bit_diff);
            if (rel_peak > 0.02)
                $fatal(1, "BLOCK_TOP output exceeds 2%% relative-to-peak gate: %e",
                       rel_peak);
            $display("SUPRA2_BLOCK_TOP_PASS block=%0d cycles=%0d", block_id, total_cycles);
        end
        $finish;
    end
endmodule

`default_nettype wire
