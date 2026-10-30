`default_nettype none

// Complete final-head datapath: block-13 tokens -> final LayerNorm/AdaLN ->
// calibrated FP16 576x16 projection -> latent unpatchify.
module supra2_final_head_datapath #(
    parameter int TOKENS=256,
    parameter int WIDTH=576,
    parameter int LANES=32,
    parameter int PATCH_VALUES=16
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 act_wr_en,
    input  wire [15:0]          act_wr_beat,
    input  wire [LANES*16-1:0]  act_wr_data,
    input  wire                 shift_wr_en,
    input  wire [4:0]           shift_wr_beat,
    input  wire [LANES*16-1:0]  shift_wr_data,
    input  wire                 scale_wr_en,
    input  wire [4:0]           scale_wr_beat,
    input  wire [LANES*16-1:0]  scale_wr_data,
    input  wire                 weight_wr_en,
    input  wire [15:0]          weight_wr_beat,
    input  wire [LANES*16-1:0]  weight_wr_data,
    input  wire                 bias_wr_en,
    input  wire [4:0]           bias_wr_addr,
    input  wire [15:0]          bias_wr_data,
    input  wire                 start,
    output logic                busy,
    output logic                done,
    output wire                 out_valid,
    input  wire                 out_ready,
    output wire                 out_last,
    output wire [LANES*16-1:0]  out_data
);
    localparam int BEATS_PER_ROW=WIDTH/LANES;
    localparam int TILE_ROWS=32;
    localparam int LOAD_CYCLES=TILE_ROWS*BEATS_PER_ROW;
    logic [15:0] activation_mem[0:TOKENS*WIDTH-1];
    logic [15:0] shift_mem[0:WIDTH-1];
    logic [15:0] scale_mem[0:WIDTH-1];
    logic [9:0] load_index;
    logic [5:0] parameter_index;
    logic [2:0] tile_index;
    logic ln_wr_en, ln_wr_ss_en, ln_wr_ss_sel, ln_start;
    logic [4:0] ln_wr_row, ln_wr_beat, ln_wr_ss_beat;
    logic [LANES*16-1:0] ln_wr_data, ln_wr_ss_data;
    logic ln_busy, ln_done, ln_out_valid;
    logic [4:0] ln_out_row, ln_out_beat;
    logic [LANES*16-1:0] ln_out_data;
    logic proj_start, proj_busy, proj_done;
    typedef enum logic [3:0] {
        S_IDLE, S_PARAMETERS, S_TILE_LOAD, S_LN_START, S_LN_WAIT,
        S_PROJ_START, S_PROJ_WAIT, S_DONE
    } state_t;
    state_t state;

    always_ff @(posedge clk) begin
        if (act_wr_en)
            for (int l=0;l<LANES;l++)
                activation_mem[int'(act_wr_beat)*LANES+l] <= act_wr_data[l*16+:16];
        if (shift_wr_en)
            for (int l=0;l<LANES;l++)
                shift_mem[int'(shift_wr_beat)*LANES+l] <= shift_wr_data[l*16+:16];
        if (scale_wr_en)
            for (int l=0;l<LANES;l++)
                scale_mem[int'(scale_wr_beat)*LANES+l] <= scale_wr_data[l*16+:16];
    end

    always_comb begin
        ln_wr_en = (state == S_TILE_LOAD);
        ln_wr_row = 5'(load_index / BEATS_PER_ROW);
        ln_wr_beat = 5'(load_index % BEATS_PER_ROW);
        ln_wr_data = '0;
        for (int l=0;l<LANES;l++)
            ln_wr_data[l*16+:16] = activation_mem[
                (int'(tile_index)*TILE_ROWS + int'(ln_wr_row))*WIDTH +
                int'(ln_wr_beat)*LANES + l];
        ln_wr_ss_en = (state == S_PARAMETERS);
        ln_wr_ss_sel = (parameter_index >= WIDTH/LANES);
        ln_wr_ss_beat = 5'(parameter_index % (WIDTH/LANES));
        ln_wr_ss_data = '0;
        for (int l=0;l<LANES;l++)
            ln_wr_ss_data[l*16+:16] = (parameter_index < WIDTH/LANES)
                ? shift_mem[int'(ln_wr_ss_beat)*LANES+l]
                : scale_mem[int'(ln_wr_ss_beat)*LANES+l];
        ln_start = (state == S_LN_START);
        proj_start = (state == S_PROJ_START);
        busy = (state != S_IDLE) && (state != S_DONE);
    end

    supra2_layernorm_modulate_32lane #(
        .ROWS(TILE_ROWS), .FEATURES(WIDTH), .KGROUP(LANES),
        .ROW_W(5), .BEAT_W(5)
    ) u_final_norm (
        .clk(clk), .rst_n(rst_n), .wr_en(ln_wr_en), .wr_row(ln_wr_row),
        .wr_beat(ln_wr_beat), .wr_data(ln_wr_data),
        .wr_ss_en(ln_wr_ss_en), .wr_ss_sel(ln_wr_ss_sel),
        .wr_ss_beat(ln_wr_ss_beat), .wr_ss_data(ln_wr_ss_data),
        .start(ln_start), .busy(ln_busy), .done(ln_done),
        .out_valid(ln_out_valid), .out_row(ln_out_row),
        .out_beat(ln_out_beat), .out_data(ln_out_data)
    );

    supra2_final_head_projection #(
        .TOKENS(TOKENS), .WIDTH(WIDTH), .PATCH_VALUES(PATCH_VALUES), .LANES(LANES)
    ) u_projection (
        .clk(clk), .rst_n(rst_n),
        .act_wr_en(ln_out_valid),
        .act_wr_beat(16'((int'(tile_index)*TILE_ROWS + int'(ln_out_row))*BEATS_PER_ROW + ln_out_beat)),
        .act_wr_data(ln_out_data),
        .weight_wr_en(weight_wr_en), .weight_wr_beat(weight_wr_beat),
        .weight_wr_data(weight_wr_data), .bias_wr_en(bias_wr_en),
        .bias_wr_addr(bias_wr_addr), .bias_wr_data(bias_wr_data),
        .start(proj_start), .busy(proj_busy), .done(proj_done),
        .out_valid(out_valid), .out_ready(out_ready), .out_last(out_last),
        .out_data(out_data)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            load_index <= '0;
            parameter_index <= '0;
            tile_index <= '0;
            done <= 1'b0;
        end else begin
            done <= 1'b0;
            case (state)
                S_IDLE: if (start) begin
                    parameter_index <= '0;
                    state <= S_PARAMETERS;
                end
                S_PARAMETERS: begin
                    if (parameter_index == (2*WIDTH/LANES)-1) begin
                        tile_index <= '0;
                        load_index <= '0;
                        state <= S_TILE_LOAD;
                    end else parameter_index <= parameter_index + 1'b1;
                end
                S_TILE_LOAD: begin
                    if (load_index == LOAD_CYCLES-1) state <= S_LN_START;
                    else load_index <= load_index + 1'b1;
                end
                S_LN_START: state <= S_LN_WAIT;
                S_LN_WAIT: if (ln_done) begin
                    if (tile_index == (TOKENS/TILE_ROWS)-1) state <= S_PROJ_START;
                    else begin
                        tile_index <= tile_index + 1'b1;
                        load_index <= '0;
                        state <= S_TILE_LOAD;
                    end
                end
                S_PROJ_START: state <= S_PROJ_WAIT;
                S_PROJ_WAIT: if (proj_done) state <= S_DONE;
                S_DONE: begin
                    done <= 1'b1;
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
