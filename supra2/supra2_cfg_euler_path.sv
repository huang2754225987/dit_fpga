`default_nettype none

// Full CFG-to-Euler stream boundary. CFG outputs are buffered so the runtime
// controller can complete CFG before starting the Euler datapath.
module supra2_cfg_euler_path #(
    parameter int LANES = 32,
    parameter int ELEMENTS = 4096,
    parameter int COUNT_W = 13
) (
    input  wire logic                  clk,
    input  wire logic                  rst_n,
    input  wire logic                  run_start,
    input  wire logic                  cfg_start,
    input  wire logic [15:0]           cfg_guidance_fp16,
    input  wire logic                  cfg_in_valid,
    output wire logic                  cfg_in_ready,
    input  wire logic [LANES*16-1:0]   velocity_uncond_fp16,
    input  wire logic [LANES*16-1:0]   velocity_cond_fp16,
    output wire logic                  cfg_done,
    input  wire logic                  euler_start,
    input  wire logic [15:0]           euler_step_size_fp16,
    input  wire logic                  latent_valid,
    output wire logic                  latent_ready,
    input  wire logic [LANES*16-1:0]   latent_fp16,
    output wire logic                  updated_valid,
    input  wire logic                  updated_ready,
    output wire logic [LANES*16-1:0]   updated_latent_fp16,
    output wire logic                  euler_done
);
    localparam int BEATS = ELEMENTS / LANES;
    localparam int PTR_W = $clog2(BEATS + 1);
    localparam logic [COUNT_W-1:0] VECTOR_COUNT = COUNT_W'(ELEMENTS);

    logic cfg_out_valid, cfg_out_ready;
    logic [LANES*16-1:0] cfg_out_data;
    logic cfg_module_in_ready;
    logic [PTR_W-1:0] write_ptr;
    logic [PTR_W-1:0] read_ptr;
    logic [LANES*16-1:0] velocity_mem [0:BEATS-1];
    logic velocity_valid;
    logic [LANES*16-1:0] velocity_data;
    logic euler_in_valid, euler_in_ready;
    logic euler_busy;

    assign cfg_out_ready = (write_ptr < PTR_W'(BEATS));
    assign cfg_in_ready = cfg_module_in_ready;
    assign euler_in_valid = velocity_valid && latent_valid;
    assign latent_ready = euler_in_ready && velocity_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            write_ptr <= '0;
            read_ptr <= '0;
            velocity_valid <= 1'b0;
            velocity_data <= '0;
        end else begin
            if (run_start || cfg_start) write_ptr <= '0;
            else if (cfg_out_valid && cfg_out_ready) begin
                velocity_mem[write_ptr] <= cfg_out_data;
                write_ptr <= write_ptr + 1'b1;
            end

            if (euler_start) begin
                read_ptr <= '0;
                velocity_data <= velocity_mem[0];
                velocity_valid <= 1'b1;
            end else if (euler_in_valid && euler_in_ready) begin
                if (read_ptr == PTR_W'(BEATS - 1)) begin
                    velocity_valid <= 1'b0;
                end else begin
                    read_ptr <= read_ptr + 1'b1;
                    velocity_data <= velocity_mem[read_ptr + 1'b1];
                end
            end
        end
    end

    supra2_cfg_combine_32lane #(.LANES(LANES), .COUNT_W(COUNT_W)) u_cfg (
        .clk(clk), .rst_n(rst_n), .start(cfg_start),
        .vector_elements(VECTOR_COUNT), .guidance_fp16(cfg_guidance_fp16),
        .in_valid(cfg_in_valid), .in_ready(cfg_module_in_ready),
        .velocity_uncond_fp16(velocity_uncond_fp16),
        .velocity_cond_fp16(velocity_cond_fp16),
        .out_valid(cfg_out_valid), .out_ready(cfg_out_ready),
        .velocity_cfg_fp16(cfg_out_data), .busy(), .done(cfg_done)
    );

    supra2_euler_update #(.LANES(LANES), .COUNT_W(COUNT_W)) u_euler (
        .clk(clk), .rst_n(rst_n), .start(euler_start),
        .vector_elements(VECTOR_COUNT), .step_size_fp16(euler_step_size_fp16),
        .in_valid(euler_in_valid), .in_ready(euler_in_ready),
        .latent_fp16(latent_fp16), .velocity_fp16(velocity_data),
        .out_valid(updated_valid), .out_ready(updated_ready),
        .updated_latent_fp16(updated_latent_fp16), .busy(euler_busy),
        .done(euler_done)
    );

    initial begin
        if (LANES <= 0 || ELEMENTS <= 0 || ELEMENTS % LANES != 0)
            $error("CFG/Euler path requires positive lane-aligned element count");
    end
endmodule

`default_nettype wire
