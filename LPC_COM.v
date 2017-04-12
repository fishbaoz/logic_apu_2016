// --------------------------------------------------------------------
// FileName:  "LPC_Device2.v"
// Author  :  Roger.Yu
// Company :  www.icpc.cn
// uart 0x3f8
// --------------------------------------------------------------------

module LPC_COM 
(
	input lclk,					// Clock 33MHz
	input lreset_n,				// Reset - Active Low (Same as PCI Reset)
	input lpc_en,				// 后端总线使能信号,高电平时总线有效
	input device_cs,
	input [15:0] addr,			// 地址
	input [7:0] din,
	output reg [7:0] dout,
	input io_rden,
	input io_wren,
	output wire com_irq,		// 串口中断线 高电平有效

	input clk_24mhz,			// 24MHz时钟输入
	output reg tx,
	input rx,
	output reg baud_clk			// 波特率时钟输出

);

reg send_busy; // 串口发送中
reg tx_ena; // 发送使能信号

// 地址 读写	寄存器名
// 0x00 R		rbr: 接收buffer
// 0x00 W		tbr: 发送buffer
// 0x01 R/W		icr: 中断使能寄存器
// 0x02 R		isr: 中断状态寄存器
// 0x02 W		fcr: fifo 控制寄存器
// 0x03 R/W		lcr: 线控寄存器
// 0x04 R/W		mcr: modem控制寄存器
// 0x05 R		lsr: 线控状态寄存器
// 0x06 R		msr: modem状态寄存器
reg [7:0] tbr,rbr,icr,isr,fcr,lcr,mcr,msr;
reg [15:0] baud_r; // 波特率寄存器
//reg [15:0] baud_cnt; // 波特率计数器
wire [7:0] lsr;
reg rx_have; // 接收到1个字节置高
reg rx_syn; // 同步外部rx信号
assign lsr = {1'b0,!tx_ena,!tx_ena,4'b0000,rx_have};
assign com_irq = mcr[3] & (~isr[0]);

always @ (posedge lclk or negedge lreset_n) begin
	if(!lreset_n) begin
		tbr <= 8'h00;
		icr <= 8'h00;
		fcr <= 8'hc0;
		lcr <= 8'h03;
		mcr <= 8'h00;
		msr <= 8'h00;
		dout <= 8'hzz;
		baud_r <= 1;
	end
	else if(device_cs & io_rden & lpc_en) begin // 寄存器读
		if(addr==0) begin
			if(lcr[7])
				dout <= baud_r[7:0];
			else
				dout <= rbr;
		end
		else if(addr==1) begin
			if(lcr[7])
				dout <= baud_r[15:8];
			else
				dout <= icr;
		end
		else if(addr==2)
			dout <= isr;
		else if(addr==3)
			dout <= lcr;
		else if(addr==4)
			dout <= mcr;
		else if(addr==5)
			dout <= lsr;
		else if(addr==6)
			dout <= msr;
		else
			dout <= 8'hff;
	end
	else if(device_cs & io_wren & lpc_en) begin // 寄存器写
		dout <= 8'hzz;
		if(addr==0) begin
			if(lcr[7])
				baud_r[7:0] <= din;
			else if(!tx_ena)
				tbr <= din;
		end
		else if(addr==1) begin
			if(lcr[7])
				baud_r[15:8] <= din;
			else
				icr <= {4'b0000,din[3:0]};
		end
		else if(addr==2)
			fcr <= din;
		else if(addr==3)
			lcr <= din;
		else if(addr==4)
			mcr[3] <= din[3];
	end
	else begin
		tbr <= tbr;
		icr <= icr;
		fcr <= fcr;
		lcr <= lcr;
		mcr <= mcr;
		msr <= msr;
		dout <= 8'hzz;
		baud_r <= baud_r;
	end
end

// clk soure 24MHz/13=1.8462MHz
// 用于产生串口模块的时钟源
reg [3:0] PRE_DIV;
always @ (posedge clk_24mhz or negedge lreset_n) begin
	if(!lreset_n) begin
		PRE_DIV <= 0;
		baud_clk <= 0;
	end
	else if(PRE_DIV == 6) begin
		PRE_DIV <= PRE_DIV + 1;
		baud_clk <= ~baud_clk;
	end
	else if(PRE_DIV == 12) begin
		PRE_DIV <= 0;
		baud_clk <= ~baud_clk;
	end
	else begin
		PRE_DIV <= PRE_DIV + 1;
		baud_clk <= baud_clk;
	end
end

// 16x 波特率计数器
reg [3:0] cnt_16x;
always @ (posedge baud_clk or negedge lreset_n) begin
	if(!lreset_n)
		cnt_16x <= 0;
	else if(send_busy)
		cnt_16x <= cnt_16x + 1;
	else
		cnt_16x <= 0;
end

// 串口发送模块
reg [2:0] tx_state;
`define TXIDLE				3'b000
`define TXSTART				3'b001
`define TXDATA				3'b010
`define TXSTOP				3'b011
`define WAITEND				3'b100
reg [3:0] send_bit_cnt;

always @ (posedge baud_clk or negedge lreset_n) begin
	if(!lreset_n) begin
		tx_state <= `TXIDLE;
		tx <= 1;
		send_busy <= 0;
		send_bit_cnt <= 0;
	end
	else begin
		case (tx_state)
			`TXIDLE: begin
				tx <= 1;
				send_busy <= 0;
				send_bit_cnt <= 0;
				if(tx_ena)
					tx_state <= `TXSTART;
				else
					tx_state <= tx_state;
			end
			`TXSTART: begin
				send_busy <= 1;
				if(cnt_16x == 4'hf) begin
					tx_state <= `TXDATA;
					tx <= tbr[send_bit_cnt];
					send_bit_cnt <= send_bit_cnt + 1;
				end
				else begin
					tx_state <= tx_state;
					tx <= 0; // 发送起始位
					send_bit_cnt <= 0;
				end
			end
			`TXDATA: begin
				send_busy <= 1;
				if((send_bit_cnt == 8) && (cnt_16x == 4'hf)) begin
					tx_state <= `TXSTOP;
					tx <= 1;
					send_bit_cnt <= 0;
				end
				else if((send_bit_cnt <= 7) && (cnt_16x == 4'hf)) begin
					tx_state <= tx_state;
					tx <= tbr[send_bit_cnt]; // 发送数据位
					send_bit_cnt <= send_bit_cnt + 1;
				end
				else begin
					tx_state <= tx_state;
					tx <= tx;
					send_bit_cnt <= send_bit_cnt;
				end
			end
			`TXSTOP: begin
				tx <= 1; // 发送停止位
				send_busy <= 1;
				if((send_bit_cnt == 4) && (cnt_16x == 4'hf)) begin
					tx_state <= `WAITEND;
					send_bit_cnt <= 0;
				end
				else if((send_bit_cnt <= 3) && (cnt_16x == 4'hf)) begin
					tx_state <= tx_state;
					send_bit_cnt <= send_bit_cnt + 1;
				end
				else begin
					tx_state <= tx_state;
					send_bit_cnt <= send_bit_cnt;
				end
			end
			`WAITEND: begin // 等待发送结束
				tx <= 1;
				send_bit_cnt <= 0;
				send_busy <= 0;
				if(tx_ena)
					tx_state <= tx_state;
				else
					tx_state <= `TXIDLE;
			end
			default: begin
				send_busy <= 0;
				tx_state <= `TXIDLE;
				tx <= 1;
				send_bit_cnt <= 0;
			end
		endcase
	end
end

// 判断tx_ena和send_busy逻辑
reg send_busy_a,send_busy_b,send_busy_c;
//reg tx_reg_empty;
always @ (posedge lclk or negedge lreset_n) begin
	if(!lreset_n) begin
		send_busy_a <= 0;
		send_busy_b <= 0;
		send_busy_c <= 0;
	end
	else begin
		send_busy_a <= send_busy;
		send_busy_b <= send_busy_a;
		send_busy_c <= send_busy_b;
	end
/*
	if(!lreset_n)
		tx_reg_empty <= 0;
	else if({send_busy_c,send_busy_b} == 2'b10)
		tx_reg_empty <= 1;
	else if(device_cs & io_rden & lpc_en) begin
		if((addr==2) & (isr[3:0] == 4'b0010)) // 读isr寄存器 清空中断
			tx_reg_empty <= 0;
		else
			tx_reg_empty <= tx_reg_empty;
	end
	else
		tx_reg_empty <= tx_reg_empty;
*/
	if(!lreset_n)
		tx_ena <= 0;
	else if({send_busy_c,send_busy_b} == 2'b10)
		tx_ena <= 0;
	else if(device_cs & io_wren & lpc_en) begin
		if((addr==0)&(!lcr[7]))
			tx_ena <= 1;
		else
			tx_ena <= tx_ena;
	end
	else
		tx_ena <= tx_ena;
end

// 串口接收模块
reg [15:0] rx_16x_bit;
always @ (posedge baud_clk or negedge lreset_n) begin
	if(!lreset_n)
		rx_16x_bit <= 16'hffff;
	else begin
		rx_16x_bit[0] <= rx_syn;
		rx_16x_bit[15:1] <= rx_16x_bit[14:0];
	end
end

reg rx_busy,rx_ok;
reg [1:0] rx_state;
reg [3:0] rx_bit_cnt;
reg [3:0] rx_16x_cnt;
reg [7:0] rx_byte_shift;
`define RXIDLE				2'b00
`define RXDATA				2'b01
`define RXSTOP				2'b10
`define RXWAITEND			2'b11

always @ (posedge baud_clk or negedge lreset_n) begin
	if(!lreset_n)
		rx_16x_cnt <= 0;
	else if(rx_busy)
		rx_16x_cnt <= rx_16x_cnt + 1;
	else
		rx_16x_cnt <= 0;
end

always @ (posedge baud_clk or negedge lreset_n) begin
	if(!lreset_n) begin
		rx_busy <= 0;
		rx_state <= `RXIDLE;
		rx_bit_cnt <= 0;
		rx_byte_shift <= 8'hff;
		rx_ok <= 0;
		rbr <= 8'hff;
		rx_syn <= 1;
	end
	else begin
		rx_syn <= rx;
		case (rx_state)
			`RXIDLE: begin
				rx_byte_shift <= 8'hff;
				rbr <= rbr;
				rx_bit_cnt <= 0;
				rx_ok <= 0;
				if(rx_16x_bit==16'h0000) begin // 收到起始位
					rx_state <= `RXDATA;
					rx_busy <= 1;
				end
				else begin
					rx_state <= `RXIDLE;
					rx_busy <= 0;
				end
			end
			`RXDATA: begin
				rx_busy <= 1;
				rx_ok <= 0;
				rbr <= rbr;
				if((rx_bit_cnt == 8) && (rx_16x_cnt == 4'hf)) begin
					rx_state <= `RXSTOP;
					rx_byte_shift <= rx_byte_shift;
					rx_bit_cnt <= 0;
				end
				else if((rx_bit_cnt <= 7) && (rx_16x_cnt == 4'h8)) begin
					rx_state <= rx_state;
					rx_byte_shift[7] <= rx_syn; // 接收数据位
					rx_byte_shift[6:0] <= rx_byte_shift[7:1];
					rx_bit_cnt <= rx_bit_cnt + 1;
				end
				else begin
					rx_state <= rx_state;
					rx_byte_shift <= rx_byte_shift;
					rx_bit_cnt <= rx_bit_cnt;
				end
			end
			`RXSTOP: begin
				rx_busy <= 1;
				rx_ok <= 0;
				rbr <= rbr;
				rx_byte_shift <= rx_byte_shift;
				rx_bit_cnt <= 0;
				if(rx_16x_cnt == 4'h8) begin
					if(rx_syn) // 接收停止位
						rx_state <= `RXWAITEND;
					else // 停止位出错 抛弃当前数据
						rx_state <= `RXIDLE;
				end
				else
					rx_state <= rx_state;
			end
			`RXWAITEND: begin // 等待接收fifo空闲
				rx_byte_shift <= rx_byte_shift;
				rx_bit_cnt <= 0;
				rx_busy <= 0;
				if(rx_have) begin // fifo 满
					rx_ok <= 0;
					rbr <= rbr;
					rx_state <= rx_state;
				end
				else begin
					rx_ok <= 1;
					rbr <= rx_byte_shift;
					rx_state <= `RXIDLE;
				end
			end
		endcase
	end
end

// 判断rx_have和rx_busy逻辑
reg rx_busy_a,rx_busy_b,rx_busy_c;
always @ (posedge lclk or negedge lreset_n) begin
	if(!lreset_n) begin
		rx_busy_a <= 0;
		rx_busy_b <= 0;
		rx_busy_c <= 0;
	end
	else begin
		rx_busy_a <= rx_busy;
		rx_busy_b <= rx_busy_a;
		rx_busy_c <= rx_busy_b;
	end

	if(!lreset_n)
		rx_have <= 0;
	else if({rx_busy_c,rx_busy_b} == 2'b10)
		rx_have <= 1;
	else if(device_cs & io_rden & lpc_en) begin
		if(addr==0)
			rx_have <= 0;
		else
			rx_have <= rx_have;
	end
	else
		rx_have <= rx_have;
end

// isr中断寄存器
always @ (posedge lclk or negedge lreset_n) begin
	if(!lreset_n) begin
		isr <= 8'hc1;
	end
	else if(rx_have & icr[0]) begin // 接收数据中断
		isr[3:0] <= 4'b0100;
	end
	/*
	else if(tx_reg_empty & icr[1]) begin // 传输寄存器空中断
		isr[3:0] <= 4'b0010;
	end
	*/
	else begin
		isr <= 8'hc1;
	end
end

endmodule
