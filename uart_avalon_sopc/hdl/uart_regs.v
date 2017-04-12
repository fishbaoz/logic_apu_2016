//////////////////////////////////////////////////////////////////////
////                                                              ////
////  uart_regs.v                                                 ////
//
// CVS Revision History
//
// $Log: uart_regs.v,v $
// Revision 1.42  2004/11/22 09:21:59  igorm
// Timeout interrupt should be generated only when there is at least ony
// character in the fifo.
//

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on

`include "uart_defines.v"

`define UART_DL1 7:0
`define UART_DL2 15:8

module uart_regs (clk,
	wb_rst_i, wb_addr_i, wb_dat_i, wb_dat_o, wb_we_i, wb_re_i, 

	// additional signals
	modem_inputs,
	stx_pad_o, srx_pad_i,

	`ifdef DATA_BUS_WIDTH_8
	`else
		// debug interface signals	enabled
		ier, iir, fcr, mcr, lcr, msr, lsr, rf_count, tf_count, tstate, rstate,
	`endif				
	rts_pad_o, dtr_pad_o, int_o
	`ifdef UART_HAS_BAUDRATE_OUTPUT
		, baud_o
	`endif
);

input clk;
input wb_rst_i;
input [`UART_ADDR_WIDTH-1:0] wb_addr_i;
input [7:0] wb_dat_i;
output [7:0] wb_dat_o;
input wb_we_i;
input wb_re_i;

output stx_pad_o;
input srx_pad_i;

input [3:0] modem_inputs;
output rts_pad_o;
output dtr_pad_o;
output int_o;
`ifdef UART_HAS_BAUDRATE_OUTPUT
	output baud_o;
`endif

`ifdef DATA_BUS_WIDTH_8
`else
	// if 32-bit databus and debug interface are enabled
	output [3:0]							ier;
	output [3:0]							iir;
	output [1:0]							fcr;  /// bits 7 and 6 of fcr. Other bits are ignored
	output [4:0]							mcr;
	output [7:0]							lcr;
	output [7:0]							msr;
	output [7:0] 							lsr;
	output [`UART_FIFO_COUNTER_W-1:0] rf_count;
	output [`UART_FIFO_COUNTER_W-1:0] tf_count;
	output [2:0] 							tstate;
	output [3:0] 							rstate;
`endif

wire [3:0] modem_inputs;
reg enable;
`ifdef UART_HAS_BAUDRATE_OUTPUT
	assign baud_o = enable; // baud_o is actually the enable signal
`endif


wire stx_pad_o;		// received from transmitter module
wire srx_pad_i;
wire srx_pad;

reg [7:0] wb_dat_o;

wire [`UART_ADDR_WIDTH-1:0] wb_addr_i;
wire [7:0] wb_dat_i;


reg [3:0] ier;
reg [3:0] iir;
reg [1:0] fcr;  // bits 7 and 6 of fcr. Other bits are ignored
reg [4:0] mcr;
reg [7:0] lcr;
reg [7:0] msr;
reg [15:0] dl;  // 16-bit divisor latch
reg [7:0] scratch; // UART scratch register
reg [7:0] aut485; // auto rs485 register
reg start_dlc; // activate dlc on writing to UART_DL1
reg lsr_mask_d; // delay for lsr_mask condition
reg msi_reset; // reset MSR 4 lower bits indicator
//reg threi_clear; // THRE interrupt clear flag
reg [15:0] dlc;  // 32-bit divisor latch counter
reg int_o;
reg [`UART_FIFO_COUNTER_W-2:0] trigger_level;
reg rx_reset;
reg tx_reset;

wire dlab;			   // divisor latch access bit
wire cts_pad_i, dsr_pad_i, ri_pad_i, dcd_pad_i; // modem status bits
wire loopback;		   // loopback bit (MCR bit 4)
wire cts, dsr, ri, dcd;	   // effective signals
wire cts_c, dsr_c, ri_c, dcd_c; // Complement effective signals (considering loopback)
wire rts_pad_o, dtr_pad_o;		   // modem control outputs

// LSR bits wires and regs
wire [7:0] lsr;
wire lsr0, lsr1, lsr2, lsr3, lsr4, lsr5, lsr6, lsr7;
reg lsr0r, lsr1r, lsr2r, lsr3r, lsr4r, lsr5r, lsr6r, lsr7r;
wire lsr_mask; // lsr_mask

//dodo add
wire rf_overrun;
wire rf_push_pulse;
wire	fifo_write;

//
// ASSINGS
//

assign lsr[7:0] = { lsr7r, lsr6r, lsr5r, lsr4r, lsr3r, lsr2r, lsr1r, lsr0r };

assign {cts_pad_i, dsr_pad_i, ri_pad_i, dcd_pad_i} = modem_inputs;
assign {cts, dsr, ri, dcd} = ~{cts_pad_i,dsr_pad_i,ri_pad_i,dcd_pad_i};

assign {cts_c, dsr_c, ri_c, dcd_c} = loopback ? {mcr[`UART_MC_RTS],mcr[`UART_MC_DTR],mcr[`UART_MC_OUT1],mcr[`UART_MC_OUT2]}
				: {cts_pad_i,dsr_pad_i,ri_pad_i,dcd_pad_i};

assign dlab = lcr[`UART_LC_DL];
assign loopback = mcr[4];

// auto rst rs485
reg rts_485_o;
always @(posedge clk or posedge wb_rst_i)
begin
	if (wb_rst_i)
		rts_485_o <= #1 0; // rs485 receive
	else
	if (fifo_write || (~lsr6r))	// restore the signal to 0 after one clock cycle
		rts_485_o <= #1 1; // rs485 send
	else
		rts_485_o <= #1 0; // rs485 receive
end

assign rts_pad_o = aut485[0]?rts_485_o:(~mcr[`UART_MC_RTS]);
assign dtr_pad_o = ~mcr[`UART_MC_DTR];

// Interrupt signals
wire rls_int;  // receiver line status interrupt
wire rda_int;  // receiver data available interrupt
wire ti_int;   // timeout indicator interrupt
wire thre_int; // transmitter holding register empty interrupt
wire ms_int;   // modem status interrupt

// FIFO signals
reg tf_push;
reg rf_pop;
wire [`UART_FIFO_REC_WIDTH-1:0] 	rf_data_out;
wire rf_error_bit; // an error (parity or framing) is inside the fifo
wire [`UART_FIFO_COUNTER_W-1:0] 	rf_count;
wire [`UART_FIFO_COUNTER_W-1:0] 	tf_count;
wire [2:0] tstate;
wire [3:0] rstate;
wire [9:0] counter_t;

wire thre_set_en; // THRE status is delayed one character time when a character is written to fifo.
reg [7:0] block_cnt;   // While counter counts, THRE status is blocked (delayed one character cycle)
reg [7:0] block_value; // One character length minus stop bit

// Transmitter Instance
wire serial_out;

uart_transmitter transmitter(clk, wb_rst_i, lcr, tf_push, wb_dat_i, enable, serial_out, tstate, tf_count, tx_reset, lsr_mask);

  // Synchronizing and sampling serial RX input
  uart_sync_flops    i_uart_sync_flops
  (
    .rst_i           (wb_rst_i),
    .clk_i           (clk),
    .stage1_rst_i    (1'b0),
    .stage1_clk_en_i (1'b1),
    .async_dat_i     (srx_pad_i),
    .sync_dat_o      (srx_pad)
  );
  defparam i_uart_sync_flops.width      = 1;
  defparam i_uart_sync_flops.init_value = 1'b1;

// handle loopback
wire serial_in = loopback ? serial_out : srx_pad;
assign stx_pad_o = loopback ? 1'b1 : serial_out;

// Receiver Instance
uart_receiver receiver(clk, wb_rst_i, lcr, rf_pop, serial_in, enable, 
	counter_t, rf_count, rf_data_out, rf_error_bit, rf_overrun, rx_reset, lsr_mask, rstate, rf_push_pulse);


// Asynchronous reading here because the outputs are sampled in uart_wb.v file 
always @(dl or dlab or ier or iir or scratch
			or lcr or lsr or msr or rf_data_out or wb_addr_i or wb_re_i)   // asynchrounous reading
begin
	case (wb_addr_i)
		`UART_REG_RB   : wb_dat_o = dlab ? dl[`UART_DL1] : rf_data_out[10:3];
		`UART_REG_IE	: wb_dat_o = dlab ? dl[`UART_DL2] : ier;
		`UART_REG_II	: wb_dat_o = {4'b1100,iir};
		`UART_REG_LC	: wb_dat_o = lcr;
		`UART_REG_LS	: wb_dat_o = lsr;
		`UART_REG_MS	: wb_dat_o = msr;
		`UART_REG_SR	: wb_dat_o = scratch;
		default:  wb_dat_o = 8'b0; // ??
	endcase // case(wb_addr_i)
end // always @ (dl or dlab or ier or iir or scratch...


// rf_pop signal handling
always @(posedge clk or posedge wb_rst_i)
begin
	if (wb_rst_i)
		rf_pop <= #1 0; 
	else
	if (rf_pop)	// restore the signal to 0 after one clock cycle
		rf_pop <= #1 0;
	else
	if (wb_re_i && wb_addr_i == `UART_REG_RB && !dlab)
		rf_pop <= #1 1; // advance read pointer
end

wire 	lsr_mask_condition;
wire 	iir_read;
wire  msr_read;
wire	fifo_read;

assign lsr_mask_condition = (wb_re_i && wb_addr_i == `UART_REG_LS && !dlab);
assign iir_read = (wb_re_i && wb_addr_i == `UART_REG_II && !dlab);
assign msr_read = (wb_re_i && wb_addr_i == `UART_REG_MS && !dlab);
assign fifo_read = (wb_re_i && wb_addr_i == `UART_REG_RB && !dlab);
assign fifo_write = (wb_we_i && wb_addr_i == `UART_REG_TR && !dlab);

// lsr_mask_d delayed signal handling
always @(posedge clk or posedge wb_rst_i)
begin
	if (wb_rst_i)
		lsr_mask_d <= #1 0;
	else // reset bits in the Line Status Register
		lsr_mask_d <= #1 lsr_mask_condition;
end

// lsr_mask is rise detected
assign lsr_mask = lsr_mask_condition && ~lsr_mask_d;

// msi_reset signal handling
always @(posedge clk or posedge wb_rst_i)
begin
	if (wb_rst_i)
		msi_reset <= #1 1;
	else
	if (msi_reset)
		msi_reset <= #1 0;
	else
	if (msr_read)
		msi_reset <= #1 1; // reset bits in Modem Status Register
end


//
//   WRITES AND RESETS   //
//
// Auto RS485 Register
always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i)
		aut485 <= #1 8'b00000000;
	else
	if (wb_we_i && wb_addr_i==`UART_REG_AUT485)
		aut485 <= #1 wb_dat_i;
		
// Line Control Register
always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i)
		lcr <= #1 8'b00000011; // 8n1 setting
	else
	if (wb_we_i && wb_addr_i==`UART_REG_LC)
		lcr <= #1 wb_dat_i;

// Interrupt Enable Register or UART_DL2
always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i)
	begin
		ier <= #1 4'b0000; // no interrupts after reset
		dl[`UART_DL2] <= #1 8'b0;
	end
	else
	if (wb_we_i && wb_addr_i==`UART_REG_IE)
		if (dlab)
		begin
			dl[`UART_DL2] <= #1 wb_dat_i;
		end
		else
			ier <= #1 wb_dat_i[3:0]; // ier uses only 4 lsb


// FIFO Control Register and rx_reset, tx_reset signals
always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) begin
		fcr <= #1 2'b11; 
		rx_reset <= #1 0;
		tx_reset <= #1 0;
	end else
	if (wb_we_i && wb_addr_i==`UART_REG_FC) begin
		fcr <= #1 wb_dat_i[7:6];
		rx_reset <= #1 wb_dat_i[1];
		tx_reset <= #1 wb_dat_i[2];
	end else begin
		rx_reset <= #1 0;
		tx_reset <= #1 0;
	end

// Modem Control Register
always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i)
		mcr <= #1 5'b0; 
	else
	if (wb_we_i && wb_addr_i==`UART_REG_MC)
			mcr <= #1 wb_dat_i[4:0];

// Scratch register
// Line Control Register
always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i)
		scratch <= #1 0; // 8n1 setting
	else
	if (wb_we_i && wb_addr_i==`UART_REG_SR)
		scratch <= #1 wb_dat_i;

// TX_FIFO or UART_DL1
always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i)
	begin
		//dl[`UART_DL1]  <= #1 8'b0;
		dl[`UART_DL1]  <= #1 8'h12;
		tf_push   <= #1 1'b0;
		start_dlc <= #1 1'b0;
	end
	else
	if (wb_we_i && wb_addr_i==`UART_REG_TR)
		if (dlab)
		begin
			dl[`UART_DL1] <= #1 wb_dat_i;
			start_dlc <= #1 1'b1; // enable DL counter
			tf_push <= #1 1'b0;
		end
		else
		begin
			tf_push   <= #1 1'b1;
			start_dlc <= #1 1'b0;
		end // else: !if(dlab)
	else
	begin
		start_dlc <= #1 1'b0;
		tf_push   <= #1 1'b0;
	end // else: !if(dlab)

// Receiver FIFO trigger level selection logic (asynchronous mux)
always @(fcr)
	case (fcr[`UART_FC_TL])
			2'b00 : trigger_level = 1;
			2'b01 : trigger_level = `UART_FIFO_DEPTH/4;
			2'b10 : trigger_level = `UART_FIFO_DEPTH/2;
			2'b11 : trigger_level = `UART_FIFO_DEPTH-2;	
	endcase // case(fcr[`UART_FC_TL])
	
//
//  STATUS REGISTERS  //
//

// Modem Status Register
reg [3:0] delayed_modem_signals;
always @(posedge clk or posedge wb_rst_i)
begin
	if (wb_rst_i)
	  begin
  		msr <= #1 0;
	  	delayed_modem_signals[3:0] <= #1 0;
	  end
	else begin
		msr[`UART_MS_DDCD:`UART_MS_DCTS] <= #1 msi_reset ? 4'b0 :
			msr[`UART_MS_DDCD:`UART_MS_DCTS] | ({dcd, ri, dsr, cts} ^ delayed_modem_signals[3:0]);
		msr[`UART_MS_CDCD:`UART_MS_CCTS] <= #1 {dcd_c, ri_c, dsr_c, cts_c};
		delayed_modem_signals[3:0] <= #1 {dcd, ri, dsr, cts};
	end
end


// Line Status Register

// activation conditions
assign lsr0 = (rf_count==0 && rf_push_pulse);  // data in receiver fifo available set condition
assign lsr1 = rf_overrun;     // Receiver overrun error
assign lsr2 = rf_data_out[1]; // parity error bit
assign lsr3 = rf_data_out[0]; // framing error bit
assign lsr4 = rf_data_out[2]; // break error in the character
assign lsr5 = (tf_count==0 && thre_set_en);  // transmitter fifo is empty
assign lsr6 = (tf_count==0 && thre_set_en && (tstate == /*`S_IDLE */ 0)); // transmitter empty
assign lsr7 = rf_error_bit | rf_overrun;

// lsr bit0 (receiver data available)
reg 	 lsr0_d;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr0_d <= #1 0;
	else lsr0_d <= #1 lsr0;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr0r <= #1 0;
	else lsr0r <= #1 (rf_count==1 && rf_pop && !rf_push_pulse || rx_reset) ? 0 : // deassert condition
					  lsr0r || (lsr0 && ~lsr0_d); // set on rise of lsr0 and keep asserted until deasserted 

// lsr bit 1 (receiver overrun)
reg lsr1_d; // delayed

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr1_d <= #1 0;
	else lsr1_d <= #1 lsr1;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr1r <= #1 0;
	else	lsr1r <= #1	lsr_mask ? 0 : lsr1r || (lsr1 && ~lsr1_d); // set on rise

// lsr bit 2 (parity error)
reg lsr2_d; // delayed

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr2_d <= #1 0;
	else lsr2_d <= #1 lsr2;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr2r <= #1 0;
	else lsr2r <= #1 lsr_mask ? 0 : lsr2r || (lsr2 && ~lsr2_d); // set on rise

// lsr bit 3 (framing error)
reg lsr3_d; // delayed

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr3_d <= #1 0;
	else lsr3_d <= #1 lsr3;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr3r <= #1 0;
	else lsr3r <= #1 lsr_mask ? 0 : lsr3r || (lsr3 && ~lsr3_d); // set on rise

// lsr bit 4 (break indicator)
reg lsr4_d; // delayed

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr4_d <= #1 0;
	else lsr4_d <= #1 lsr4;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr4r <= #1 0;
	else lsr4r <= #1 lsr_mask ? 0 : lsr4r || (lsr4 && ~lsr4_d);

// lsr bit 5 (transmitter fifo is empty)
reg lsr5_d;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr5_d <= #1 1;
	else lsr5_d <= #1 lsr5;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr5r <= #1 1;
	else lsr5r <= #1 (fifo_write) ? 0 :  lsr5r || (lsr5 && ~lsr5_d);

// lsr bit 6 (transmitter empty indicator)
reg lsr6_d;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr6_d <= #1 1;
	else lsr6_d <= #1 lsr6;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr6r <= #1 1;
	else lsr6r <= #1 (fifo_write) ? 0 : lsr6r || (lsr6 && ~lsr6_d);

// lsr bit 7 (error in fifo)
reg lsr7_d;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr7_d <= #1 0;
	else lsr7_d <= #1 lsr7;

always @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) lsr7r <= #1 0;
	else lsr7r <= #1 lsr_mask ? 0 : lsr7r || (lsr7 && ~lsr7_d);

// Frequency divider
always @(posedge clk or posedge wb_rst_i) 
begin
	if (wb_rst_i)
		dlc <= #1 0;
	else
		if (start_dlc | ~ (|dlc))
  			dlc <= #1 dl - 1;               // preset counter
		else
			dlc <= #1 dlc - 1;              // decrement counter
end

// Enable signal generation logic
always @(posedge clk or posedge wb_rst_i)
begin
	if (wb_rst_i)
		enable <= #1 1'b0;
	else
		if (|dl & ~(|dlc))     // dl>0 & dlc==0
			enable <= #1 1'b1;
		else
			enable <= #1 1'b0;
end

// Delaying THRE status for one character cycle after a character is written to an empty fifo.
always @(lcr)
  case (lcr[3:0])
    4'b0000                             : block_value =  95; // 6 bits
    4'b0100                             : block_value = 103; // 6.5 bits
    4'b0001, 4'b1000                    : block_value = 111; // 7 bits
    4'b1100                             : block_value = 119; // 7.5 bits
    4'b0010, 4'b0101, 4'b1001           : block_value = 127; // 8 bits
    4'b0011, 4'b0110, 4'b1010, 4'b1101  : block_value = 143; // 9 bits
    4'b0111, 4'b1011, 4'b1110           : block_value = 159; // 10 bits
    4'b1111                             : block_value = 175; // 11 bits
  endcase // case(lcr[3:0])

// Counting time of one character minus stop bit
always @(posedge clk or posedge wb_rst_i)
begin
  if (wb_rst_i)
    block_cnt <= #1 8'd0;
  else
  if(lsr5r & fifo_write)  // THRE bit set & write to fifo occured
    block_cnt <= #1 block_value;
  else
  if (enable & block_cnt != 8'b0)  // only work on enable times
    block_cnt <= #1 block_cnt - 1;  // decrement break counter
end // always of break condition detection

// Generating THRE status enable signal
assign thre_set_en = ~(|block_cnt);


//
//	INTERRUPT LOGIC
//

assign rls_int  = ier[`UART_IE_RLS] && (lsr[`UART_LS_OE] || lsr[`UART_LS_PE] || lsr[`UART_LS_FE] || lsr[`UART_LS_BI]);
assign rda_int  = ier[`UART_IE_RDA] && (rf_count >= {1'b0,trigger_level});
assign thre_int = ier[`UART_IE_THRE] && lsr[`UART_LS_TFE];
assign ms_int   = ier[`UART_IE_MS] && (| msr[3:0]);
assign ti_int   = ier[`UART_IE_RDA] && (counter_t == 10'b0) && (|rf_count);

reg 	 rls_int_d;
reg 	 thre_int_d;
reg 	 ms_int_d;
reg 	 ti_int_d;
reg 	 rda_int_d;

// delay lines
always  @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) rls_int_d <= #1 0;
	else rls_int_d <= #1 rls_int;

always  @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) rda_int_d <= #1 0;
	else rda_int_d <= #1 rda_int;

always  @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) thre_int_d <= #1 0;
	else thre_int_d <= #1 thre_int;

always  @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) ms_int_d <= #1 0;
	else ms_int_d <= #1 ms_int;

always  @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) ti_int_d <= #1 0;
	else ti_int_d <= #1 ti_int;

// rise detection signals

wire 	 rls_int_rise;
wire 	 thre_int_rise;
wire 	 ms_int_rise;
wire 	 ti_int_rise;
wire 	 rda_int_rise;

assign rda_int_rise    = rda_int & ~rda_int_d;
assign rls_int_rise 	  = rls_int & ~rls_int_d;
assign thre_int_rise   = thre_int & ~thre_int_d;
assign ms_int_rise 	  = ms_int & ~ms_int_d;
assign ti_int_rise 	  = ti_int & ~ti_int_d;

// interrupt pending flags
reg 	rls_int_pnd;
reg	rda_int_pnd;
reg 	thre_int_pnd;
reg 	ms_int_pnd;
reg 	ti_int_pnd;

// interrupt pending flags assignments
always  @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) rls_int_pnd <= #1 0; 
	else 
		rls_int_pnd <= #1 lsr_mask ? 0 :  						// reset condition
							rls_int_rise ? 1 :						// latch condition
							rls_int_pnd && ier[`UART_IE_RLS];	// default operation: remove if masked

always  @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) rda_int_pnd <= #1 0; 
	else 
		rda_int_pnd <= #1 ((rf_count == {1'b0,trigger_level}) && fifo_read) ? 0 :  	// reset condition
							rda_int_rise ? 1 :						// latch condition
							rda_int_pnd && ier[`UART_IE_RDA];	// default operation: remove if masked

always  @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) thre_int_pnd <= #1 0; 
	else 
		thre_int_pnd <= #1 fifo_write || (iir_read & ~iir[`UART_II_IP] & iir[`UART_II_II] == `UART_II_THRE)? 0 : 
							thre_int_rise ? 1 :
							thre_int_pnd && ier[`UART_IE_THRE];

always  @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) ms_int_pnd <= #1 0; 
	else 
		ms_int_pnd <= #1 msr_read ? 0 : 
							ms_int_rise ? 1 :
							ms_int_pnd && ier[`UART_IE_MS];

always  @(posedge clk or posedge wb_rst_i)
	if (wb_rst_i) ti_int_pnd <= #1 0; 
	else 
		ti_int_pnd <= #1 fifo_read ? 0 : 
							ti_int_rise ? 1 :
							ti_int_pnd && ier[`UART_IE_RDA];
// end of pending flags

// INT_O logic
always @(posedge clk or posedge wb_rst_i)
begin
	if (wb_rst_i)	
		int_o <= #1 1'b0;
	else
		int_o <= #1 
					rls_int_pnd		?	~lsr_mask					:
					rda_int_pnd		? 1								:
					ti_int_pnd		? ~fifo_read					:
					thre_int_pnd	? !(fifo_write & iir_read) :
					ms_int_pnd		? ~msr_read						:
					0;	// if no interrupt are pending
end


// Interrupt Identification register
always @(posedge clk or posedge wb_rst_i)
begin
	if (wb_rst_i)
		iir <= #1 1;
	else
	if (rls_int_pnd)  // interrupt is pending
	begin
		iir[`UART_II_II] <= #1 `UART_II_RLS;	// set identification register to correct value
		iir[`UART_II_IP] <= #1 1'b0;		// and clear the IIR bit 0 (interrupt pending)
	end else // the sequence of conditions determines priority of interrupt identification
	if (rda_int)
	begin
		iir[`UART_II_II] <= #1 `UART_II_RDA;
		iir[`UART_II_IP] <= #1 1'b0;
	end
	else if (ti_int_pnd)
	begin
		iir[`UART_II_II] <= #1 `UART_II_TI;
		iir[`UART_II_IP] <= #1 1'b0;
	end
	else if (thre_int_pnd)
	begin
		iir[`UART_II_II] <= #1 `UART_II_THRE;
		iir[`UART_II_IP] <= #1 1'b0;
	end
	else if (ms_int_pnd)
	begin
		iir[`UART_II_II] <= #1 `UART_II_MS;
		iir[`UART_II_IP] <= #1 1'b0;
	end else	// no interrupt is pending
	begin
		iir[`UART_II_II] <= #1 0;
		iir[`UART_II_IP] <= #1 1'b1;
	end
end

endmodule
