`define LITLE_ENDIAN

// remove comments to restore to use the new version with 8 data bit interface
// in 32bit-bus mode, the wb_sel_i signal is used to put data in correct place
// also, in 8-bit version there'll be no debugging features included
// CAUTION: doesn't work with current version of OR1200
//`define DATA_BUS_WIDTH_8

`ifdef DATA_BUS_WIDTH_8
 `define UART_ADDR_WIDTH 3
 `define UART_DATA_WIDTH 8
`else
 `define UART_ADDR_WIDTH 5
 `define UART_DATA_WIDTH 32
`endif

// Uncomment this if you want your UART to have
// 16xBaudrate output port.
// If defined, the enable signal will be used to drive baudrate_o signal
// It's frequency is 16xbaudrate

// `define UART_HAS_BAUDRATE_OUTPUT
// Register addresses
`define UART_REG_RB	`UART_ADDR_WIDTH'd0	// receiver buffer R
`define UART_REG_TR  `UART_ADDR_WIDTH'd0	// transmitter
`define UART_REG_IE	`UART_ADDR_WIDTH'd1	// Interrupt enable
`define UART_REG_II  `UART_ADDR_WIDTH'd2	// Interrupt identification
`define UART_REG_FC  `UART_ADDR_WIDTH'd2	// FIFO control
`define UART_REG_LC	`UART_ADDR_WIDTH'd3	// Line Control
`define UART_REG_MC	`UART_ADDR_WIDTH'd4	// Modem control
`define UART_REG_LS  `UART_ADDR_WIDTH'd5	// Line status
`define UART_REG_MS  `UART_ADDR_WIDTH'd6	// Modem status
`define UART_REG_SR  `UART_ADDR_WIDTH'd7	// Scratch register
`define UART_REG_DL1	`UART_ADDR_WIDTH'd0	// Divisor latch bytes (1-2)
`define UART_REG_DL2	`UART_ADDR_WIDTH'd1
`define UART_REG_AUT485  `UART_ADDR_WIDTH'd6	// auto RS485 register

// Interrupt Enable register bits
`define UART_IE_RDA		0	// Received Data available interrupt
`define UART_IE_THRE	1	// Transmitter Holding Register empty interrupt
`define UART_IE_RLS		2	// Receiver Line Status Interrupt
`define UART_IE_MS		3	// Modem Status Interrupt

// Interrupt Identification register bits
`define UART_II_IP		0	// Interrupt pending when 0
`define UART_II_II		3:1	// Interrupt identification

// Interrupt identification values for bits 3:1
`define UART_II_RLS		3'b011	// Receiver Line Status
`define UART_II_RDA		3'b010	// Receiver Data available
`define UART_II_TI		3'b110	// Timeout Indication
`define UART_II_THRE	3'b001	// Transmitter Holding Register empty
`define UART_II_MS		3'b000	// Modem Status

// FIFO Control Register bits
`define UART_FC_TL		1:0	// Trigger level

// FIFO trigger level values
`define UART_FC_1		2'b00
`define UART_FC_4		2'b01
`define UART_FC_8		2'b10
`define UART_FC_14		2'b11

// Line Control register bits
`define UART_LC_BITS	1:0	// bits in character
`define UART_LC_SB		2	// stop bits
`define UART_LC_PE		3	// parity enable
`define UART_LC_EP		4	// even parity
`define UART_LC_SP		5	// stick parity
`define UART_LC_BC		6	// Break control
`define UART_LC_DL		7	// Divisor Latch access bit

// Modem Control register bits
`define UART_MC_DTR		0
`define UART_MC_RTS		1
`define UART_MC_OUT1	2
`define UART_MC_OUT2	3
`define UART_MC_LB		4	// Loopback mode

// Line Status Register bits
`define UART_LS_DR		0	// Data ready
`define UART_LS_OE		1	// Overrun Error
`define UART_LS_PE		2	// Parity Error
`define UART_LS_FE		3	// Framing Error
`define UART_LS_BI		4	// Break interrupt
`define UART_LS_TFE		5	// Transmit FIFO is empty
`define UART_LS_TE		6	// Transmitter Empty indicator
`define UART_LS_EI		7	// Error indicator

// Modem Status Register bits
`define UART_MS_DCTS	0	// Delta signals
`define UART_MS_DDSR	1
`define UART_MS_TERI	2
`define UART_MS_DDCD	3
`define UART_MS_CCTS	4	// Complement signals
`define UART_MS_CDSR	5
`define UART_MS_CRI		6
`define UART_MS_CDCD	7

// FIFO parameter defines

`define UART_FIFO_WIDTH	8

//如果需要大FIFO就定义FIFO_DEPTH_BIG,否则不需要定义默认16字节fifo
//`define FIFO_DEPTH_BIG

`ifdef FIFO_DEPTH_BIG
	`define UART_FIFO_DEPTH	    64
	`define UART_FIFO_POINTER_W	6  //fifo地址宽度,为UART_FIFO_DEPTH-1的位宽
	`define UART_FIFO_COUNTER_W	7  //fifo计数器宽度,为UART_FIFO_DEPTH的位宽
`else
	`define UART_FIFO_DEPTH	16
	`define UART_FIFO_POINTER_W	4  //fifo地址宽度,为UART_FIFO_DEPTH-1的位宽
	`define UART_FIFO_COUNTER_W	5  //fifo计数器宽度,为UART_FIFO_DEPTH的位宽
`endif

// receiver fifo has width 11 because it has break, parity and framing error bits
`define UART_FIFO_REC_WIDTH  11

`define VERBOSE_WB  0           // All activity on the WISHBONE is recorded
`define VERBOSE_LINE_STATUS 0   // Details about the lsr (line status register)
`define FAST_TEST   1           // 64/1024 packets are sent
