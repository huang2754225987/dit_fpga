`timescale 1ns/1ps
`default_nettype none

module tb_supra2_cfg_euler_path;
    localparam int LANES = 32;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic run_start = 1'b0;
    logic cfg_start = 1'b0;
    logic [15:0] guidance = 16'h4200;
    logic cfg_valid = 1'b0;
    wire cfg_ready;
    logic [LANES*16-1:0] uncond_data = '0;
    logic [LANES*16-1:0] cond_data = '0;
    wire cfg_done;
    logic euler_start = 1'b0;
    logic [15:0] step_size = 16'h3800;
    logic latent_valid = 1'b0;
    wire latent_ready;
    logic [LANES*16-1:0] latent_data = '0;
    wire updated_valid;
    logic updated_ready = 1'b0;
    wire [LANES*16-1:0] updated_data;
    wire euler_done;
    integer accepted = 0;

    always #5 clk = ~clk;

    supra2_cfg_euler_path #(.LANES(LANES), .ELEMENTS(64), .COUNT_W(7)) dut (
        .clk(clk), .rst_n(rst_n), .run_start(run_start),
        .cfg_start(cfg_start), .cfg_guidance_fp16(guidance),
        .cfg_in_valid(cfg_valid), .cfg_in_ready(cfg_ready),
        .velocity_uncond_fp16(uncond_data), .velocity_cond_fp16(cond_data),
        .cfg_done(cfg_done), .euler_start(euler_start),
        .euler_step_size_fp16(step_size), .latent_valid(latent_valid),
        .latent_ready(latent_ready), .latent_fp16(latent_data),
        .updated_valid(updated_valid), .updated_ready(updated_ready),
        .updated_latent_fp16(updated_data), .euler_done(euler_done)
    );

    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        run_start = 1'b1;
        @(negedge clk); run_start = 1'b0;

        for (int lane = 0; lane < LANES; lane++) begin
            uncond_data[lane*16 +: 16] = 16'h3c00;
            cond_data[lane*16 +: 16] = 16'h4000;
            latent_data[lane*16 +: 16] = 16'h3c00;
        end
        cfg_start = 1'b1;
        @(negedge clk);
        cfg_start = 1'b0;
        cfg_valid = 1'b1;
        @(negedge clk);
        // First 32 values (1 + 3*(2-1)) are committed; present second beat.
        @(posedge clk);
        if (!cfg_done) $fatal(1, "CFG completion missing on final beat");
        @(negedge clk);
        cfg_valid = 1'b0;
        latent_valid = 1'b1;
        euler_start = 1'b1;
        @(negedge clk);
        euler_start = 1'b0;

        repeat (2) begin
            @(negedge clk);
            if (!updated_valid || latent_ready || euler_done)
                $fatal(1, "Euler output backpressure contract failed");
            for (int lane = 0; lane < LANES; lane++)
                if (updated_data[lane*16 +: 16] !== 16'h4200)
                    $fatal(1, "CFG-to-Euler result lane %0d got %h", lane,
                        updated_data[lane*16 +: 16]);
        end
        updated_ready = 1'b1;
        #1;
        if (!updated_valid || !latent_ready || euler_done)
            $fatal(1, "First Euler beat handshake invalid");
        @(negedge clk);
        for (int lane = 0; lane < LANES; lane++)
            if (updated_data[lane*16 +: 16] !== 16'h4200)
                $fatal(1, "Second CFG-to-Euler result lane %0d got %h", lane,
                    updated_data[lane*16 +: 16]);
        if (!euler_done) $fatal(1, "Euler completion missing on final beat");
        @(negedge clk);
        latent_valid = 1'b0;
        updated_ready = 1'b0;
        if (accepted != 64) $fatal(1, "Expected 64 accepted values, got %0d", accepted);
        $display("SUPRA2_CFG_EULER_PATH_PASS elements=64 guidance=3 backpressure=1");
        $finish;
    end

    always @(posedge clk) begin
        if (updated_valid && updated_ready) accepted <= accepted + LANES;
    end
endmodule

`default_nettype wire
