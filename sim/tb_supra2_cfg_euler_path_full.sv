`timescale 1ns/1ps
`default_nettype none

module tb_supra2_cfg_euler_path_full;
    localparam int LANES = 32;
    localparam int ELEMENTS = 4096;
    localparam int BEATS = ELEMENTS / LANES;

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
    integer accepted;
    integer cycles;

    always #5 clk = ~clk;

    supra2_cfg_euler_path #(
        .LANES(LANES), .ELEMENTS(ELEMENTS), .COUNT_W(13)
    ) dut (
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
        for (int lane = 0; lane < LANES; lane++) begin
            uncond_data[lane*16 +: 16] = 16'h3c00; // 1.0
            cond_data[lane*16 +: 16] = 16'h4000;   // 2.0
            latent_data[lane*16 +: 16] = 16'h3c00; // 1.0
        end

        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        run_start = 1'b1;
        @(negedge clk); run_start = 1'b0;
        cfg_start = 1'b1;
        @(negedge clk); cfg_start = 1'b0; cfg_valid = 1'b1;

        for (int beat = 0; beat < BEATS; beat++) begin
            @(posedge clk);
            if (!cfg_ready) $fatal(1, "CFG input stalled at beat %0d", beat);
            if ((beat == BEATS-1) !== cfg_done)
                $fatal(1, "CFG done mismatch at beat %0d", beat);
            @(negedge clk);
        end
        cfg_valid = 1'b0;

        euler_start = 1'b1;
        latent_valid = 1'b1;
        updated_ready = 1'b1;
        @(negedge clk); euler_start = 1'b0;

        accepted = 0;
        for (cycles = 0; cycles < BEATS + 4 && accepted < BEATS; cycles++) begin
            @(posedge clk);
            if (updated_valid && updated_ready) begin
                for (int lane = 0; lane < LANES; lane++)
                    if (updated_data[lane*16 +: 16] !== 16'h4200)
                        $fatal(1, "full-size Euler mismatch beat=%0d lane=%0d got=%h",
                               accepted, lane, updated_data[lane*16 +: 16]);
                if ((accepted == BEATS-1) !== euler_done)
                    $fatal(1, "Euler done mismatch at beat %0d", accepted);
                accepted = accepted + 1;
            end
        end
        if (accepted != BEATS)
            $fatal(1, "Expected %0d Euler beats, got %0d", BEATS, accepted);

        @(negedge clk);
        latent_valid = 1'b0;
        updated_ready = 1'b0;
        $display("SUPRA2_CFG_EULER_PATH_FULL_PASS elements=%0d beats=%0d guidance=3",
                 ELEMENTS, BEATS);
        $finish;
    end

    initial begin
        repeat (BEATS*4 + 100) @(posedge clk);
        $fatal(1, "full-size CFG/Euler test timed out");
    end
endmodule

`default_nettype wire
