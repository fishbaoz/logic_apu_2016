// UART core AVALON interface 
//
// Author: Roger.yu   (yfz_431@hotmail.com)
// Company: Kontron China
//

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "uart_defines.v"
 
module uart_al (clk, al_reset_i, 
    al_chipselect_i, al_read_i, al_write_i, al_readdata_o, al_writedata_i, al_address_i, 
    wb_adr_int, wb_dat8_i, wb_dat8_o, wb_dat32_o, al_byteenable_i, 
	we_o, re_o // Write and read enable output for the core
);

input 		  clk;

// AVALON interface	
input 		  al_reset_i;
input 		  al_chipselect_i;
input 		  al_read_i;
input 		  al_write_i;
input [3:0]   al_byteenable_i;
input [`UART_ADDR_WIDTH-1:0] 	al_address_i; //AVALON address line

`ifdef DATA_BUS_WIDTH_8
input [7:0]  wb_dat_i; //input AVALON bus 
output [7:0] wb_dat_o;
reg [7:0] 	 wb_dat_o;
wire [7:0] 	 wb_dat_i;
reg [7:0] 	 wb_dat_is;
`else // for 32 data bus mode
input [31:0]  al_writedata_i; //input AVALON bus 
output [31:0] al_readdata_o;
reg [31:0] 	  al_readdata_o;
wire [31:0]   al_writedata_i;
reg [31:0] 	  wb_dat_is;//±£¡Ù
`endif // !`ifdef DATA_BUS_WIDTH_8

output [`UART_ADDR_WIDTH-1:0]	wb_adr_int; // internal signal for address bus±£¡Ù
input [7:0]   wb_dat8_o; // internal 8 bit output to be put into wb_dat_o
output [7:0]  wb_dat8_i;
input [31:0]  wb_dat32_o; // 32 bit data output (for debug interface)
output 		  we_o;
output 		  re_o;

wire 			  we_o;
wire 			  re_o;
reg [7:0] 	  wb_dat8_i;
wire [7:0] 	  wb_dat8_o;
wire [`UART_ADDR_WIDTH-1:0]	wb_adr_int; // internal signal for address bus
reg [`UART_ADDR_WIDTH-1:0]	wb_adr_is;
reg 								wb_re_is;
reg 								wb_we_is;
reg [3:0] 						wb_sel_is;
wire [3:0]   al_byteenable_i;

assign we_o = wb_we_is & al_chipselect_i; //WE for registers	
assign re_o = wb_re_is & al_chipselect_i; //RE for registers	

// Sample input signals
always  @(posedge clk or posedge al_reset_i)
	if (al_reset_i) begin
		wb_re_is <= #1 0;
		wb_we_is <= #1 0;
		wb_adr_is <= #1 0;
		wb_dat_is <= #1 0;
		wb_sel_is <= #1 0;
	end else begin
		wb_re_is <= #1 al_read_i;
		wb_we_is <= #1 al_write_i;
		wb_adr_is <= #1 al_address_i;
		wb_dat_is <= #1 al_writedata_i;
		wb_sel_is <= #1 al_byteenable_i;
	end

`ifdef DATA_BUS_WIDTH_8 // 8-bit data bus
always @(posedge clk or posedge al_reset_i)
	if (al_reset_i)
		al_readdata_o <= #1 0;
	else
		al_readdata_o <= #1 wb_dat8_o;

always @(wb_dat_is)
	wb_dat8_i = wb_dat_is;

assign wb_adr_int = wb_adr_is;

`else // 32-bit bus
// put output to the correct byte in 32 bits using select line
always @(posedge clk or posedge al_reset_i)
	if (al_reset_i)
		al_readdata_o <= #1 0;
	else if (re_o)

		case (wb_sel_is)
			4'b0001: al_readdata_o <= #1 {24'b0, wb_dat8_o};
			4'b0010: al_readdata_o <= #1 {16'b0, wb_dat8_o,8'b0};
			4'b0100: al_readdata_o <= #1 {8'b0, wb_dat8_o,16'b0};
			4'b1000: al_readdata_o <= #1 {wb_dat8_o, 24'b0};
			4'b1111: al_readdata_o <= #1 wb_dat32_o; // debug interface output
 			default: al_readdata_o <= #1 0;
		endcase // case(wb_sel_i)

reg [1:0] wb_adr_int_lsb;

always @(wb_sel_is or wb_dat_is)
begin
	case (wb_sel_is)
		4'b0001 : wb_dat8_i = wb_dat_is[7:0];
		4'b0010 : wb_dat8_i = wb_dat_is[15:8];
		4'b0100 : wb_dat8_i = wb_dat_is[23:16];
		4'b1000 : wb_dat8_i = wb_dat_is[31:24];
		default : wb_dat8_i = wb_dat_is[7:0];
	endcase // case(wb_sel_i)

  `ifdef LITLE_ENDIAN
	case (wb_sel_is)
		4'b0001 : wb_adr_int_lsb = 2'h0;
		4'b0010 : wb_adr_int_lsb = 2'h1;
		4'b0100 : wb_adr_int_lsb = 2'h2;
		4'b1000 : wb_adr_int_lsb = 2'h3;
		default : wb_adr_int_lsb = 2'h0;
	endcase // case(wb_sel_i)
  `else
	case (wb_sel_is)
		4'b0001 : wb_adr_int_lsb = 2'h3;
		4'b0010 : wb_adr_int_lsb = 2'h2;
		4'b0100 : wb_adr_int_lsb = 2'h1;
		4'b1000 : wb_adr_int_lsb = 2'h0;
		default : wb_adr_int_lsb = 2'h0;
	endcase // case(wb_sel_i)
  `endif
end

assign wb_adr_int = {wb_adr_is[`UART_ADDR_WIDTH-3:0], wb_adr_int_lsb};

`endif // !`ifdef DATA_BUS_WIDTH_8

endmodule










