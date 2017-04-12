
// synopsys translate_off
`include "timescale.v"
// synopsys translate_on

`include "uart_defines.v"

module uart_top	(
	clk, 
	
	// AVALON signals
	reset, al_chipselect_i, al_read_i, al_write_i, al_readdata_o, al_writedata_i, 
	al_address_i, al_byteenable_i, 
	int_o, // interrupt request

	// UART	signals
	// serial input/output
	stx_pad_o, srx_pad_i,

	// modem signals
	rts_pad_o, cts_pad_i, dtr_pad_o, dsr_pad_i, ri_pad_i, dcd_pad_i
`ifdef UART_HAS_BAUDRATE_OUTPUT
	, baud_o
`endif
	);

parameter	uart_data_width = `UART_DATA_WIDTH;
parameter	uart_addr_width = `UART_ADDR_WIDTH;

input 								 clk;

// AVALON interface
input 								 reset;
input [4:0] 	     al_address_i;
input [31:0] 	     al_writedata_i;
output [31:0] 	     al_readdata_o;
input 								 al_read_i;
input 								 al_write_i;
input 								 al_chipselect_i;
input [3:0]							 al_byteenable_i;
output 								 int_o;

// UART	signals
input 								 srx_pad_i;
output 								 stx_pad_o;
output 								 rts_pad_o;
input 								 cts_pad_i;
output 								 dtr_pad_o;
input 								 dsr_pad_i;
input 								 ri_pad_i;
input 								 dcd_pad_i;

// optional baudrate output
`ifdef UART_HAS_BAUDRATE_OUTPUT
output	baud_o;
`endif


wire 								 stx_pad_o;
wire 								 rts_pad_o;
wire 								 dtr_pad_o;

wire [4:0] 	         al_address_i;
wire [31:0] 	         al_writedata_i;
wire [31:0] 	         al_readdata_o;

wire [7:0] 							 wb_dat8_i; // 8-bit internal data input
wire [7:0] 							 wb_dat8_o; // 8-bit internal data output
wire [31:0] 						 wb_dat32_o; // debug interface 32-bit output
wire [3:0] 							 al_byteenable_i;  // AVALON select signal
wire [uart_addr_width-1:0] 	         wb_adr_int;
wire 								 we_o;	// Write enable for registers
wire		          	             re_o;	// Read enable for registers
//
// MODULE INSTANCES
//

`ifdef DATA_BUS_WIDTH_8
`else
// debug interface wires
wire	[3:0] ier;
wire	[3:0] iir;
wire	[1:0] fcr;
wire	[4:0] mcr;
wire	[7:0] lcr;
wire	[7:0] msr;
wire	[7:0] lsr;
wire	[`UART_FIFO_COUNTER_W-1:0] rf_count;
wire	[`UART_FIFO_COUNTER_W-1:0] tf_count;
wire	[2:0] tstate;
wire	[3:0] rstate; 
`endif

`ifdef DATA_BUS_WIDTH_8
////  AVALON interface module
uart_al		al_interface(
		.clk(		clk		),
		.al_reset_i(	reset	),
	    .al_writedata_i(al_writedata_i),
	    .al_readdata_o(al_readdata_o),
	    .wb_dat8_i(wb_dat8_i),
	    .wb_dat8_o(wb_dat8_o),
	    .wb_dat32_o(32'b0),								 
	    .al_byteenable_i(4'b0),
		.al_read_i(al_read_i),
		.al_write_i(al_write_i),
		.al_chipselect_i(al_chipselect_i),
	    .al_address_i(al_address_i),
	    .wb_adr_int(wb_adr_int),
		.we_o(we_o),
		.re_o(re_o)
		);
`else
uart_al		al_interface(
		.clk(clk),
		.al_reset_i(reset),
	    .al_writedata_i(al_writedata_i),
	    .al_readdata_o(al_readdata_o),
	    .wb_dat8_i(wb_dat8_i),
	    .wb_dat8_o(wb_dat8_o),
	    .wb_dat32_o(wb_dat32_o),
	    .al_byteenable_i(al_byteenable_i),
		.al_read_i(al_read_i),
		.al_write_i(al_write_i),		
		.al_chipselect_i(al_chipselect_i),
	    .al_address_i(al_address_i),
	    .wb_adr_int(wb_adr_int),
		.we_o(we_o),
		.re_o(re_o)
		);
`endif

// Registers
uart_regs	regs(
	.clk(clk),
	.wb_rst_i(reset),
	.wb_addr_i(wb_adr_int),
	.wb_dat_i(wb_dat8_i),
	.wb_dat_o(wb_dat8_o),
	.wb_we_i(we_o),
	.wb_re_i(re_o),
	.modem_inputs({cts_pad_i, dsr_pad_i, ri_pad_i, dcd_pad_i}),
	.stx_pad_o(stx_pad_o),
	.srx_pad_i(srx_pad_i),
	`ifdef DATA_BUS_WIDTH_8
	`else
		// debug interface signals	enabled
		.ier(ier), 
		.iir(iir), 
		.fcr(fcr), 
		.mcr(mcr), 
		.lcr(lcr), 
		.msr(msr), 
		.lsr(lsr), 
		.rf_count(rf_count),
		.tf_count(tf_count),
		.tstate(tstate),
		.rstate(rstate),
	`endif					  
	.rts_pad_o(		rts_pad_o		),
	.dtr_pad_o(		dtr_pad_o		),
	.int_o(		int_o		)
	`ifdef UART_HAS_BAUDRATE_OUTPUT
		, .baud_o(baud_o)
	`endif
);

`ifdef DATA_BUS_WIDTH_8
`else
uart_debug_if dbg(/*AUTOINST*/
	// Outputs
	.wb_dat32_o				 (wb_dat32_o[31:0]),
	// Inputs
	.wb_adr_i				 (wb_adr_int[`UART_ADDR_WIDTH-1:0]),
	.ier						 (ier[3:0]),
	.iir						 (iir[3:0]),
	.fcr						 (fcr[1:0]),
	.mcr						 (mcr[4:0]),
	.lcr						 (lcr[7:0]),
	.msr						 (msr[7:0]),
	.lsr						 (lsr[7:0]),
	.rf_count				 (rf_count[`UART_FIFO_COUNTER_W-1:0]),
	.tf_count				 (tf_count[`UART_FIFO_COUNTER_W-1:0]),
	.tstate					 (tstate[2:0]),
	.rstate					 (rstate[3:0]));
`endif 

initial
begin
	`ifdef DATA_BUS_WIDTH_8
		$display("(%m) UART INFO: Data bus width is 8. No Debug interface.\n");
	`else
		$display("(%m) UART INFO: Data bus width is 32. Debug Interface present.\n");
	`endif
	`ifdef UART_HAS_BAUDRATE_OUTPUT
		$display("(%m) UART INFO: Has baudrate output\n");
	`else
		$display("(%m) UART INFO: Doesn't have baudrate output\n");
	`endif
end

endmodule


