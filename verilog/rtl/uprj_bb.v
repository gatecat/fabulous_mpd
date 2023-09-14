(* blackbox *)
module RISCV_core
  (input  clk,
   input  reset,
   input  [31:0] o_rftop_rd1,
   input  [31:0] o_rfbot_rd2,
   input  [31:0] i_ROM_instruction,
   input  [31:0] i_dmem_read_data,
   output [7:0] o_rftop_rs1,
   output [7:0] o_rfbot_rs2,
   output o_rf_we,
   output [7:0] o_rf_wa,
   output [31:0] o_rf_wd,
   output [7:0] o_ROM_addr,
   output [9:0] o_dmem_addr,
   output [31:0] o_dmem_write_data,
   output [3:0] o_dmem_write_enable);
endmodule

(* blackbox *)
module SLICE
    #(
        parameter MaxFramesPerCol=21,
        parameter FrameBitsPerRow=32,
        parameter NoConfigBits=642
    )
    (
        input APPLY_INIT,
        input H_I,
        input H6,
        input H5,
        input H4,
        input H3,
        input H2,
        input H1,
        input CKEN_B4,
        input CKEN_B3,
        input G_I,
        input G6,
        input G5,
        input G4,
        input G3,
        input G2,
        input G1,
        input SRST_B2,
        input F_I,
        input F6,
        input F5,
        input F4,
        input F3,
        input F2,
        input F1,
        input CLK_B2,
        input E_I,
        input E6,
        input E5,
        input E4,
        input E3,
        input E2,
        input E1,
        input HX,
        input GX,
        input FX,
        input EX,
        input CIN,
        input AX,
        input BX,
        input CX,
        input DX,
        input D_I,
        input D6,
        input D5,
        input D4,
        input D3,
        input D2,
        input D1,
        input SRST_B1,
        input CKEN_B1,
        input CKEN_B2,
        input C_I,
        input C6,
        input C5,
        input C4,
        input C3,
        input C2,
        input C1,
        input CLK_B1,
        input B_I,
        input B6,
        input B5,
        input B4,
        input B3,
        input B2,
        input B1,
        input A_I,
        input A6,
        input A5,
        input A4,
        input A3,
        input A2,
        input A1,
        output H_O,
        output COUT,
        output HQ2,
        output HQ,
        output HMUX,
        output G_O,
        output GQ2,
        output GQ,
        output GMUX,
        output F_O,
        output FQ2,
        output FQ,
        output FMUX,
        output E_O,
        output EQ2,
        output EQ,
        output EMUX,
        output D_O,
        output DQ2,
        output DQ,
        output DMUX,
        output C_O,
        output CQ2,
        output CQ,
        output CMUX,
        output B_O,
        output BQ2,
        output BQ,
        output BMUX,
        output A_O,
        output AQ2,
        output AQ,
        output AMUX,
    //Tile IO ports from BELs
        //input UserCLK,
        //output UserCLKo,
        input [FrameBitsPerRow -1:0] FrameData, //CONFIG_PORT
        //output [FrameBitsPerRow -1:0] FrameData_O,
        input [MaxFramesPerCol -1:0] FrameStrobe //CONFIG_PORT
        //output [MaxFramesPerCol -1:0] FrameStrobe_O
    //global
);
endmodule

(* blackbox *)
module uart_clock(
  input i_clk,
  input i_reset,
  input [31:0] i_sampling_delay,
  output o_clk
);
endmodule

(* blackbox *)
module usb_cdc(
  input app_clk_i,
  input clk_i,
  input rstn_i,

  output configured_o,

  input dn_rx_i,
  output dn_tx_o,
  output dp_pu_o,
  input dp_rx_i,
  output dp_tx_o,
  output tx_en_o,

  output [10:0] frame_o,
  input [7:0] in_data_i,
  output in_ready_o,
  input in_valid_i,
  output [7:0] out_data_o,
  input out_ready_i,
  output out_valid_o
);
endmodule

(* blackbox *)
module Stump(
  input clk, rst,
  output [15:0] address,
  output [3:0] cc,
  input [15:0] data_in,
  output [15:0] data_out,
  output fetch,
  output mem_ren,
  output mem_wen,
  output [15:0] regC,
  input [2:0] srcC,
);
endmodule

(* blackbox *)
module dac_top(
  input clk, rst,
  input [15:0] in,
  output out
);
endmodule

(* blackbox *)
module quad_wrapper(
  input clock,
  input [19:0] i_vec_20,
  output [19:0] o_vec_20
);
endmodule
