// Compile this file once, before the bench, in a fresh model library/unit.
// Include directories: THIS directory first, then the vendor ddr4_model root.
// Do not include build/mig_ddr4_sim or compile its x8 wrapper. Vendor
// proj_package.sv/interface.sv include this basename recursively; this guard
// makes those includes empty without selecting the old x8 configuration.
`ifndef SUPRA2_BOARD_DDR4_X16_MODEL_WRAPPER
`define SUPRA2_BOARD_DDR4_X16_MODEL_WRAPPER
`define DDR4_4G_X16
`include "arch_package.sv"
`include "proj_package.sv"
`include "interface.sv"
`include "ddr4_model.sv"

`timescale 1ps/1ps
module ddr4_x16_model_wrapper (
    input wire model_enable,
    input wire reset_n, act_n,
    input wire [16:0] adr,
    input wire [1:0] ba,
    input wire bg, cke, odt, cs_n, ck_t, ck_c,
    inout wire [15:0] dq,
    inout wire [1:0] dqs_t, dqs_c, dm_dbi_n
);
    DDR4_if #(.CONFIGURED_DQ_BITS(16)) device_if();
    ddr4_model #(.CONFIGURED_DQ_BITS(16), .CONFIGURED_DENSITY(arch_package::_4G))
        device_model (.model_enable(model_enable), .iDDR4(device_if));

    for (genvar bit_index=0; bit_index<16; bit_index++) begin : g_dq
        tran dq_link(device_if.DQ[bit_index],dq[bit_index]);
    end
    for (genvar byte_index=0; byte_index<2; byte_index++) begin : g_strobe
        tran dqs_t_link(device_if.DQS_t[byte_index],dqs_t[byte_index]);
        tran dqs_c_link(device_if.DQS_c[byte_index],dqs_c[byte_index]);
        tran dm_link(device_if.DM_n[byte_index],dm_dbi_n[byte_index]);
    end
    // One rank of discrete 4-Gb x16 devices: BG0 exists, BG1/C are unused.
    // Pass command/address pins unchanged, including A10/AP and A12/BC_n.
    assign device_if.BG = {1'b0,bg};
    assign device_if.BA = ba;
    assign device_if.C = '0;
    assign device_if.ADDR_17 = 1'b0;
    assign device_if.ADDR = adr[13:0];
    assign device_if.RAS_n_A16 = adr[16];
    assign device_if.CAS_n_A15 = adr[15];
    assign device_if.WE_n_A14 = adr[14];
    assign device_if.CK = {ck_t,ck_c};
    assign device_if.CS_n = cs_n;
    assign device_if.ACT_n = act_n;
    assign device_if.CKE = cke;
    assign device_if.ODT = odt;
    assign device_if.RESET_n = reset_n;
    assign device_if.PARITY = 1'b0;
    assign device_if.TEN = 1'b0;
    assign device_if.ZQ = 1'b1;
    assign device_if.PWR = 1'b1;
    assign device_if.VREF_CA = 1'b1;
    assign device_if.VREF_DQ = 1'b1;
`ifdef DDR4_X8
    initial $fatal(1,"Board model compiled with DDR4_X8; use a fresh library and only the x16 wrapper");
`endif
`ifndef DDR4_X16
    initial $fatal(1,"DDR4_4G_X16 must select DDR4_X16 in vendor arch_defines.v");
`endif
endmodule
`endif
