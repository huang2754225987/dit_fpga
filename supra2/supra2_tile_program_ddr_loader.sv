`default_nettype none

// Fetches the compiler program and dependency token list from DDR through AXI.
// The 512-bit board interface returns four 128-bit commands or 32 16-bit
// dependency tokens per beat. A command becomes available only after preload.
module supra2_tile_program_ddr_loader #(
    parameter integer AXI_DATA_W = 512,
    parameter logic [31:0] PROGRAM_BASE = 32'h0090_0000,
    parameter logic [31:0] DEPENDENCY_BASE = 32'h0091_0000,
    parameter integer MAX_COMMANDS = 512,
    parameter integer MAX_DEPENDENCIES = 4096
) (
    input wire clk, input wire rst_n, input wire start,
    output logic busy, output logic done, output logic error,
    input wire [15:0] program_words, input wire [15:0] dependency_words,
    output logic load_we, output logic [15:0] load_addr,
    output logic [127:0] load_data,
    output logic dependency_we, output logic [15:0] dependency_addr,
    output logic [15:0] dependency_data,
    output logic [3:0] m_axi_arid, output logic [31:0] m_axi_araddr,
    output logic [7:0] m_axi_arlen, output logic [2:0] m_axi_arsize,
    output logic [1:0] m_axi_arburst, output logic m_axi_arvalid,
    input wire m_axi_arready,
    input wire [3:0] m_axi_rid, input wire [AXI_DATA_W-1:0] m_axi_rdata,
    input wire [1:0] m_axi_rresp, input wire m_axi_rlast,
    input wire m_axi_rvalid, output logic m_axi_rready
);
    typedef enum logic [3:0] {S_IDLE,S_PROG_AR,S_PROG_R,S_PROG_EMIT,S_DEP_AR,S_DEP_R,S_DEP_EMIT,S_DONE,S_ERROR} state_t;
    state_t state;
    logic [AXI_DATA_W-1:0] read_buffer;
    logic [5:0] lane_index;
    logic [15:0] p_words, d_words;
    logic [15:0] p_beat, d_beat;
    logic [15:0] p_beats, d_beats;
    logic [31:0] expected_addr;

    always_comb begin
        m_axi_arid=4'h9;
        m_axi_araddr=0;
        m_axi_arlen=0;
        m_axi_arsize=3'd6;
        m_axi_arburst=2'b01;
        m_axi_arvalid=1'b0;
        m_axi_rready=1'b0;
        case(state)
            S_PROG_AR: begin
                m_axi_araddr=PROGRAM_BASE+32'(p_beat)*64;
                m_axi_arvalid=1'b1;
            end
            S_PROG_R: m_axi_rready=1'b1;
            S_DEP_AR: begin
                m_axi_araddr=DEPENDENCY_BASE+32'(d_beat)*64;
                m_axi_arvalid=1'b1;
            end
            S_DEP_R: m_axi_rready=1'b1;
            default: ;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin : loader_fsm
        integer lane, idx;
        if(!rst_n) begin
            state<=S_IDLE; busy<=0; done<=0; error<=0;
            p_words<=0; d_words<=0; p_beat<=0; d_beat<=0;
            p_beats<=0; d_beats<=0; expected_addr<=0;
            read_buffer<='0; lane_index<=0;
            load_we<=0; load_addr<=0; load_data<=0;
            dependency_we<=0; dependency_addr<=0; dependency_data<=0;
        end else begin
            load_we<=0; dependency_we<=0;
            case(state)
                S_IDLE: if(start) begin
                    if(program_words<1 || program_words>MAX_COMMANDS ||
                       dependency_words>MAX_DEPENDENCIES) begin
                        error<=1; state<=S_ERROR;
                    end else begin
                        p_words<=program_words; d_words<=dependency_words;
                        p_beats<=(program_words+3)/4; d_beats<=(dependency_words+31)/32;
                        p_beat<=0; d_beat<=0; busy<=1; done<=0; error<=0;
                        state<=S_PROG_AR;
                    end
                end
                S_PROG_AR: if(m_axi_arvalid && m_axi_arready) begin
                    expected_addr<=PROGRAM_BASE+32'(p_beat)*64;
                    state<=S_PROG_R;
                end
                S_PROG_R: if(m_axi_rvalid && m_axi_rready) begin
                    if(m_axi_rid!=4'h9 || m_axi_rresp!=0 || !m_axi_rlast) begin
                        error<=1; state<=S_ERROR;
                    end else begin
                        read_buffer<=m_axi_rdata; lane_index<=0; state<=S_PROG_EMIT;
                    end
                end
                S_PROG_EMIT: begin
                    idx=int'(p_beat)*4+int'(lane_index);
                    if(idx<p_words) begin
                        load_we<=1; load_addr<=16'(idx);
                        load_data<=read_buffer[lane_index*128+:128];
                    end
                    if(lane_index==3) begin
                        if(p_beat+1>=p_beats) begin
                            if(d_words==0) state<=S_DONE;
                            else state<=S_DEP_AR;
                        end else begin p_beat<=p_beat+1'b1; state<=S_PROG_AR; end
                    end else lane_index<=lane_index+1'b1;
                end
                S_DEP_AR: if(m_axi_arvalid && m_axi_arready) begin
                    expected_addr<=DEPENDENCY_BASE+32'(d_beat)*64;
                    state<=S_DEP_R;
                end
                S_DEP_R: if(m_axi_rvalid && m_axi_rready) begin
                    if(m_axi_rid!=4'h9 || m_axi_rresp!=0 || !m_axi_rlast) begin
                        error<=1; state<=S_ERROR;
                    end else begin
                        read_buffer<=m_axi_rdata; lane_index<=0; state<=S_DEP_EMIT;
                    end
                end
                S_DEP_EMIT: begin
                    idx=int'(d_beat)*32+int'(lane_index);
                    if(idx<d_words) begin
                        dependency_we<=1; dependency_addr<=16'(idx);
                        dependency_data<=read_buffer[lane_index*16+:16];
                    end
                    if(lane_index==31) begin
                        if(d_beat+1>=d_beats) state<=S_DONE;
                        else begin d_beat<=d_beat+1'b1; state<=S_DEP_AR; end
                    end else lane_index<=lane_index+1'b1;
                end
                S_DONE: begin busy<=0; done<=1; state<=S_DONE; end
                S_ERROR: begin busy<=0; error<=1; state<=S_ERROR; end
                default: begin error<=1; state<=S_ERROR; end
            endcase
        end
    end

`ifndef SYNTHESIS
    always @(posedge clk) begin
        if(rst_n && m_axi_rvalid && m_axi_rready &&
           (m_axi_rid!=4'h9 || m_axi_rresp!=0 || !m_axi_rlast))
            $error("Tile-program DDR read response error expected_addr=%h",expected_addr);
    end
`endif
endmodule

`default_nettype wire
