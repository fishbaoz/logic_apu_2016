//////////////////////////////////////////////////////////////////////
////                                                              ////
////  uart_rfifo.v (Modified from uart_fifo.v)                    ////
////                                                              ////
////                                                              ////
////  This file is part of the "UART 16550 compatible" project    ////
////  http://www.opencores.org/cores/uart16550/                   ////
////                                                              ////
////  Documentation related to this project:                      ////
////  - http://www.opencores.org/cores/uart16550/                 ////
////                                                              ////
////  Projects compatibility:                                     ////
////  - WISHBONE                                                  ////
////  RS232 Protocol                                              ////
////  16550D uart (mostly supported)                              ////
////                                                              ////
////  Overview (main Features):                                   ////
////  UART core receiver FIFO                                     ////
////                                                              ////
////  To Do:                                                      ////
////  Nothing.                                                    ////
////                                                              ////
////  Author(s):                                                  ////
////      - gorban@opencores.org                                  ////
////      - Jacob Gorban                                          ////
////      - Igor Mohor (igorm@opencores.org)                      ////
////                                                              ////
////  Created:        2001/05/12                                  ////
////  Last Updated:   2002/07/22                                  ////
////                  (See log for the revision history)          ////
////                                                              ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2000, 2001 Authors                             ////
////                                                              ////
//// This source file may be used and distributed without         ////
//// restriction provided that this copyright statement is not    ////
//// removed from the file and that any derivative work contains  ////
//// the original copyright notice and the associated disclaimer. ////
////                                                              ////
//// This source file is free software; you can redistribute it   ////
//// and/or modify it under the terms of the GNU Lesser General   ////
//// Public License as published by the Free Software Foundation; ////
//// either version 2.1 of the License, or (at your option) any   ////
//// later version.                                               ////
////                                                              ////
//// This source is distributed in the hope that it will be       ////
//// useful, but WITHOUT ANY WARRANTY; without even the implied   ////
//// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ////
//// PURPOSE.  See the GNU Lesser General Public License for more ////
//// details.                                                     ////
////                                                              ////
//// You should have received a copy of the GNU Lesser General    ////
//// Public License along with this source; if not, download it   ////
//// from http://www.opencores.org/lgpl.shtml                     ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
//
// CVS Revision History
//
// $Log: uart_rfifo.v,v $
// Revision 1.4  2003/07/11 18:20:26  gorban
// added clearing the receiver fifo statuses on resets
//
// Revision 1.3  2003/06/11 16:37:47  gorban
// This fixes errors in some cases when data is being read and put to the FIFO at the same time. Patch is submitted by Scott Furman. Update is very recommended.
//
// Revision 1.2  2002/07/29 21:16:18  gorban
// The uart_defines.v file is included again in sources.
//
// Revision 1.1  2002/07/22 23:02:23  gorban
// Bug Fixes:
//  * Possible loss of sync and bad reception of stop bit on slow baud rates fixed.
//   Problem reported by Kenny.Tung.
//  * Bad (or lack of ) loopback handling fixed. Reported by Cherry Withers.
//
// Improvements:
//  * Made FIFO's as general inferrable memory where possible.
//  So on FPGA they should be inferred as RAM (Distributed RAM on Xilinx).
//  This saves about 1/3 of the Slice count and reduces P&R and synthesis times.
//
//  * Added optional baudrate output (baud_o).
//  This is identical to BAUDOUT* signal on 16550 chip.
//  It outputs 16xbit_clock_rate - the divided clock.
//  It's disabled by default. Define UART_HAS_BAUDRATE_OUTPUT to use.
//
// Revision 1.16  2001/12/20 13:25:46  mohor
// rx push changed to be only one cycle wide.
//
// Revision 1.15  2001/12/18 09:01:07  mohor
// Bug that was entered in the last update fixed (rx state machine).
//
// Revision 1.14  2001/12/17 14:46:48  mohor
// overrun signal was moved to separate block because many sequential lsr
// reads were preventing data from being written to rx fifo.
// underrun signal was not used and was removed from the project.
//
// Revision 1.13  2001/11/26 21:38:54  gorban
// Lots of fixes:
// Break condition wasn't handled correctly at all.
// LSR bits could lose their values.
// LSR value after reset was wrong.
// Timing of THRE interrupt signal corrected.
// LSR bit 0 timing corrected.
//
// Revision 1.12  2001/11/08 14:54:23  mohor
// Comments in Slovene language deleted, few small fixes for better work of
// old tools. IRQs need to be fix.
//
// Revision 1.11  2001/11/07 17:51:52  gorban
// Heavily rewritten interrupt and LSR subsystems.
// Many bugs hopefully squashed.
//
// Revision 1.10  2001/10/20 09:58:40  gorban
// Small synopsis fixes
//
// Revision 1.9  2001/08/24 21:01:12  mohor
// Things connected to parity changed.
// Clock devider changed.
//
// Revision 1.8  2001/08/24 08:48:10  mohor
// FIFO was not cleared after the data was read bug fixed.
//
// Revision 1.7  2001/08/23 16:05:05  mohor
// Stop bit bug fixed.
// Parity bug fixed.
// WISHBONE read cycle bug fixed,
// OE indicator (Overrun Error) bug fixed.
// PE indicator (Parity Error) bug fixed.
// Register read bug fixed.
//
// Revision 1.3  2001/05/31 20:08:01  gorban
// FIFO changes and other corrections.
//
// Revision 1.3  2001/05/27 17:37:48  gorban
// Fixed many bugs. Updated spec. Changed FIFO files structure. See CHANGES.txt file.
//
// Revision 1.2  2001/05/17 18:34:18  gorban
// First 'stable' release. Should be sythesizable now. Also added new header.
//
// Revision 1.0  2001-05-17 21:27:12+02  jacob
// Initial revision
//
//

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on

`include "uart_defines.v"

module uart_rfifo (clk, 
	wb_rst_i, data_in, data_out,
// Control signals
	push, // push strobe, active high
	pop,   // pop strobe, active high
// status signals
	overrun,//fifo溢出
	count,
	error_bit,
	fifo_reset,
	reset_status
	);


// FIFO parameters
parameter fifo_width = `UART_FIFO_REC_WIDTH;
parameter fifo_depth = `UART_FIFO_DEPTH;
parameter fifo_pointer_w = `UART_FIFO_POINTER_W;
parameter fifo_counter_w = `UART_FIFO_COUNTER_W;

input				clk;
input				wb_rst_i;
input				push;
input				pop;
input	[fifo_width-1:0]	data_in;
input				fifo_reset;
input       reset_status;

output	[fifo_width-1:0]	data_out;
output				overrun;
output	[fifo_counter_w-1:0]	count;
output				error_bit;

wire	[fifo_width-1:0]	data_out;
wire [7:0] data8_out;
// flags FIFO
reg	[2:0]	fifo[fifo_depth-1:0];//每个fifo单元的标志位

// FIFO pointers
reg	[fifo_pointer_w-1:0]	top;//输入数据到fifo的地址
reg	[fifo_pointer_w-1:0]	bottom;//从FIFO输出数据的地址

reg	[fifo_counter_w-1:0]	count;
reg				overrun;

wire [fifo_pointer_w-1:0] top_plus_1 = top + 1'b1;

raminfr #(fifo_pointer_w,8,fifo_depth) rfifo  
        (.clk(clk), 
			.we(push), 
			.a(top), 
			.dpra(bottom), 
			.di(data_in[fifo_width-1:fifo_width-8]), 
			.dpo(data8_out)
		); 

always @(posedge clk or posedge wb_rst_i) // 同步 FIFO
begin
	if (wb_rst_i)
	begin
		top		<= #1 0;
		bottom		<= #1 1'b0;
		count		<= #1 0;
		fifo[0] <= #1 0;
		fifo[1] <= #1 0;
		fifo[2] <= #1 0;
		fifo[3] <= #1 0;
		fifo[4] <= #1 0;
		fifo[5] <= #1 0;
		fifo[6] <= #1 0;
		fifo[7] <= #1 0;
		fifo[8] <= #1 0;
		fifo[9] <= #1 0;
		fifo[10] <= #1 0;
		fifo[11] <= #1 0;
		fifo[12] <= #1 0;
		fifo[13] <= #1 0;
		fifo[14] <= #1 0;
		fifo[15] <= #1 0;
		`ifdef FIFO_DEPTH_BIG
			fifo[16] <= #1 0;
			fifo[17] <= #1 0;
			fifo[18] <= #1 0;
			fifo[19] <= #1 0;
			fifo[20] <= #1 0;
			fifo[21] <= #1 0;
			fifo[22] <= #1 0;
			fifo[23] <= #1 0;
			fifo[24] <= #1 0;
			fifo[25] <= #1 0;
			fifo[26] <= #1 0;
			fifo[27] <= #1 0;
			fifo[28] <= #1 0;
			fifo[29] <= #1 0;
			fifo[30] <= #1 0;
			fifo[31] <= #1 0;
			fifo[32] <= #1 0;
			fifo[33] <= #1 0;
			fifo[34] <= #1 0;
			fifo[35] <= #1 0;
			fifo[36] <= #1 0;
			fifo[37] <= #1 0;
			fifo[38] <= #1 0;
			fifo[39] <= #1 0;
			fifo[40] <= #1 0;
			fifo[41] <= #1 0;
			fifo[42] <= #1 0;
			fifo[43] <= #1 0;
			fifo[44] <= #1 0;
			fifo[45] <= #1 0;
			fifo[46] <= #1 0;
			fifo[47] <= #1 0;
			fifo[48] <= #1 0;
			fifo[49] <= #1 0;
			fifo[50] <= #1 0;
			fifo[51] <= #1 0;
			fifo[52] <= #1 0;
			fifo[53] <= #1 0;
			fifo[54] <= #1 0;
			fifo[55] <= #1 0;
			fifo[56] <= #1 0;
			fifo[57] <= #1 0;
			fifo[58] <= #1 0;
			fifo[59] <= #1 0;
			fifo[60] <= #1 0;
			fifo[61] <= #1 0;
			fifo[62] <= #1 0;
			fifo[63] <= #1 0;
		`endif
	end
	else
	if (fifo_reset) begin
		top		<= #1 0;
		bottom		<= #1 1'b0;
		count		<= #1 0;
		fifo[0] <= #1 0;
		fifo[1] <= #1 0;
		fifo[2] <= #1 0;
		fifo[3] <= #1 0;
		fifo[4] <= #1 0;
		fifo[5] <= #1 0;
		fifo[6] <= #1 0;
		fifo[7] <= #1 0;
		fifo[8] <= #1 0;
		fifo[9] <= #1 0;
		fifo[10] <= #1 0;
		fifo[11] <= #1 0;
		fifo[12] <= #1 0;
		fifo[13] <= #1 0;
		fifo[14] <= #1 0;
		fifo[15] <= #1 0;
		`ifdef FIFO_DEPTH_BIG
			fifo[16] <= #1 0;
			fifo[17] <= #1 0;
			fifo[18] <= #1 0;
			fifo[19] <= #1 0;
			fifo[20] <= #1 0;
			fifo[21] <= #1 0;
			fifo[22] <= #1 0;
			fifo[23] <= #1 0;
			fifo[24] <= #1 0;
			fifo[25] <= #1 0;
			fifo[26] <= #1 0;
			fifo[27] <= #1 0;
			fifo[28] <= #1 0;
			fifo[29] <= #1 0;
			fifo[30] <= #1 0;
			fifo[31] <= #1 0;
			fifo[32] <= #1 0;
			fifo[33] <= #1 0;
			fifo[34] <= #1 0;
			fifo[35] <= #1 0;
			fifo[36] <= #1 0;
			fifo[37] <= #1 0;
			fifo[38] <= #1 0;
			fifo[39] <= #1 0;
			fifo[40] <= #1 0;
			fifo[41] <= #1 0;
			fifo[42] <= #1 0;
			fifo[43] <= #1 0;
			fifo[44] <= #1 0;
			fifo[45] <= #1 0;
			fifo[46] <= #1 0;
			fifo[47] <= #1 0;
			fifo[48] <= #1 0;
			fifo[49] <= #1 0;
			fifo[50] <= #1 0;
			fifo[51] <= #1 0;
			fifo[52] <= #1 0;
			fifo[53] <= #1 0;
			fifo[54] <= #1 0;
			fifo[55] <= #1 0;
			fifo[56] <= #1 0;
			fifo[57] <= #1 0;
			fifo[58] <= #1 0;
			fifo[59] <= #1 0;
			fifo[60] <= #1 0;
			fifo[61] <= #1 0;
			fifo[62] <= #1 0;
			fifo[63] <= #1 0;
		`endif
	end
  else
	begin
		case ({push, pop})
		2'b10 : if (count<fifo_depth)  // overrun condition
			begin
				top       <= #1 top_plus_1;
				fifo[top] <= #1 data_in[2:0];//存储标志位
				count     <= #1 count + 1'b1;
			end
		2'b01 : if(count>0)
			begin
                fifo[bottom] <= #1 0;
				bottom   <= #1 bottom + 1'b1;
				count	 <= #1 count - 1'b1;
			end
		2'b11 : begin//同时读写到来 count不变,所以不会溢出
				bottom   <= #1 bottom + 1'b1;
				top       <= #1 top_plus_1;
				fifo[top] <= #1 data_in[2:0];
		        end
    default: ;
		endcase
	end
end   // always

//溢出标志
always @(posedge clk or posedge wb_rst_i) // synchronous FIFO
begin
  if (wb_rst_i)
    overrun   <= #1 1'b0;
  else
  if(fifo_reset | reset_status) 
    overrun   <= #1 1'b0;
  else
  if(push & ~pop & (count==fifo_depth))
    overrun   <= #1 1'b1;
end   // always


// please note though that data_out is only valid one clock after pop signal
//注意data_out只有一个clk周期的有效期 在pop信号后第一个周期
assign data_out = {data8_out,fifo[bottom]};

// Additional logic for detection of error conditions (parity and framing) inside the FIFO
// for the Line Status Register bit 7

wire	[2:0]	word0 = fifo[0];
wire	[2:0]	word1 = fifo[1];
wire	[2:0]	word2 = fifo[2];
wire	[2:0]	word3 = fifo[3];
wire	[2:0]	word4 = fifo[4];
wire	[2:0]	word5 = fifo[5];
wire	[2:0]	word6 = fifo[6];
wire	[2:0]	word7 = fifo[7];

wire	[2:0]	word8 = fifo[8];
wire	[2:0]	word9 = fifo[9];
wire	[2:0]	word10 = fifo[10];
wire	[2:0]	word11 = fifo[11];
wire	[2:0]	word12 = fifo[12];
wire	[2:0]	word13 = fifo[13];
wire	[2:0]	word14 = fifo[14];
wire	[2:0]	word15 = fifo[15];
`ifdef FIFO_DEPTH_BIG
 wire	[2:0]	word16 = fifo[16];
 wire	[2:0]	word17 = fifo[17];
 wire	[2:0]	word18 = fifo[18];
 wire	[2:0]	word19 = fifo[19];
 wire	[2:0]	word20 = fifo[20];
 wire	[2:0]	word21 = fifo[21];
 wire	[2:0]	word22 = fifo[22];
 wire	[2:0]	word23 = fifo[23];
 wire	[2:0]	word24 = fifo[24];
 wire	[2:0]	word25 = fifo[25];
 wire	[2:0]	word26 = fifo[26];
 wire	[2:0]	word27 = fifo[27];
 wire	[2:0]	word28 = fifo[28];
 wire	[2:0]	word29 = fifo[29];
 wire	[2:0]	word30 = fifo[30];
 wire	[2:0]	word31 = fifo[31];
 wire	[2:0]	word32 = fifo[32];
 wire	[2:0]	word33 = fifo[33];
 wire	[2:0]	word34 = fifo[34];
 wire	[2:0]	word35 = fifo[35];
 wire	[2:0]	word36 = fifo[36];
 wire	[2:0]	word37 = fifo[37];
 wire	[2:0]	word38 = fifo[38];
 wire	[2:0]	word39 = fifo[39];
 wire	[2:0]	word40 = fifo[40];
 wire	[2:0]	word41 = fifo[41];
 wire	[2:0]	word42 = fifo[42];
 wire	[2:0]	word43 = fifo[43];
 wire	[2:0]	word44 = fifo[44];
 wire	[2:0]	word45 = fifo[45];
 wire	[2:0]	word46 = fifo[46];
 wire	[2:0]	word47 = fifo[47];
 wire	[2:0]	word48 = fifo[48];
 wire	[2:0]	word49 = fifo[49];
 wire	[2:0]	word50 = fifo[50];
 wire	[2:0]	word51 = fifo[51];
 wire	[2:0]	word52 = fifo[52];
 wire	[2:0]	word53 = fifo[53];
 wire	[2:0]	word54 = fifo[54];
 wire	[2:0]	word55 = fifo[55];
 wire	[2:0]	word56 = fifo[56];
 wire	[2:0]	word57 = fifo[57];
 wire	[2:0]	word58 = fifo[58];
 wire	[2:0]	word59 = fifo[59];
 wire	[2:0]	word60 = fifo[60];
 wire	[2:0]	word61 = fifo[61];
 wire	[2:0]	word62 = fifo[62];
 wire	[2:0]	word63 = fifo[63];
`endif

// a 1 is returned if any of the error bits in the fifo is 1
assign	error_bit = |(word0[2:0]  | word1[2:0]  | word2[2:0]  | word3[2:0]  |
            		  word4[2:0]  | word5[2:0]  | word6[2:0]  | word7[2:0]  |
            		  word8[2:0]  | word9[2:0]  | word10[2:0] | word11[2:0] |
            		  word12[2:0] | word13[2:0] | word14[2:0] | word15[2:0] 
                  `ifdef FIFO_DEPTH_BIG
                       | word16[2:0] | word17[2:0] | word18[2:0] | word19[2:0] | word20[2:0] 
                       | word21[2:0] | word22[2:0] | word23[2:0] | word24[2:0] | word25[2:0] 
                       | word26[2:0] | word27[2:0] | word28[2:0] | word29[2:0] | word30[2:0]
                       | word31[2:0] | word32[2:0] | word33[2:0] | word34[2:0] | word35[2:0] 
                       | word36[2:0] | word37[2:0] | word38[2:0] | word39[2:0] | word40[2:0] 
                       | word41[2:0] | word42[2:0] | word43[2:0] | word44[2:0] | word45[2:0]
                       | word46[2:0] | word47[2:0] | word48[2:0] | word49[2:0] | word50[2:0]
                       | word51[2:0] | word52[2:0] | word53[2:0] | word54[2:0] | word55[2:0] 
                       | word56[2:0] | word57[2:0] | word58[2:0] | word59[2:0] | word60[2:0]
                       | word61[2:0] | word62[2:0] | word63[2:0]   
                  `endif
                  );
endmodule
