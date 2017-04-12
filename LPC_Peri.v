// --------------------------------------------------------------------
// FileName:  "LPC_Peri.v"
// Author  :  Roger.Yu
// Company :  www.icpc.cn
// --------------------------------------------------------------------
//
// Project:     Low Pin Count PCI
// File:        LPC_Peri.v
// Title:       Low Pin Count State Machine
// Description: Top level of Low Pin Count PCI Memory Interface
//
// --------------------------------------------------------------------
// Revision History :
// --------------------------------------------------------------------
// Revision 1.0  2014-8-22 Roger Yu: Initial Creation, Modified
// --------------------------------------------------------------------

module LPC_Peri (
  // LPC Interface
  input  wire        lclk         , // Clock
  input  wire        lreset_n     , // Reset - Active Low (Same as PCI Reset)
  input  wire        lframe_n     , // Frame - Active Low
  inout  wire [ 3:0] lad_in       , // Address/Data Bus

  output reg [ 3:0]  lpc_data_out , // 用于测试

  output reg         lpc_en       , // 后端总线使能信号,高电平时总线有效
  input  wire        addr_hit     , // 地址匹配置1,不匹配置0
  output reg [15:0]  lpc_addr     , // LPC地址
  input  wire [ 7:0] din          , // LPC读的时候后端输入的数据
  output reg  [ 7:0] lpc_data_in  , // LPC写的时候给后端的输出数据
  output reg         io_rden_sm   ,
  output reg         io_wren_sm   ,
  inout int_serirq,					// 串行中断数据线
  input [7:0] serirq                // 中断输入
);

reg [ 4:0] current_state;
reg lad_in_dir;

`define IDLE             5'h00
`define START            5'h01
`define IO_RD            5'h02
`define IO_RD_ADDR_LCLK1 5'h03
`define IO_RD_ADDR_LCLK2 5'h04
`define IO_RD_ADDR_LCLK3 5'h05
`define IO_RD_ADDR_LCLK4 5'h06
`define IO_RD_TAR_LCLK1  5'h07
`define IO_RD_TAR_LCLK2  5'h08
`define IO_RD_SYNC       5'h09
`define IO_RD_DATA_LCLK1 5'h0B
`define IO_RD_DATA_LCLK2 5'h0C
`define IO_WR            5'h0D
`define IO_WR_ADDR_LCLK1 5'h0E
`define IO_WR_ADDR_LCLK2 5'h0F
`define IO_WR_ADDR_LCLK3 5'h10
`define IO_WR_ADDR_LCLK4 5'h11
`define IO_WR_DATA_LCLK1 5'h12
`define IO_WR_DATA_LCLK2 5'h13
`define IO_WR_TAR_LCLK1  5'h14
`define IO_WR_TAR_LCLK2  5'h15
`define IO_WR_SYNC       5'h16

assign lad_in = lad_in_dir ? lpc_data_out : 4'bzzzz;

// --------------------------------------------------------------------------
// FSM -- state machine supporting LPC I/O read & I/O write only
// --------------------------------------------------------------------------
always @ (posedge lclk or negedge lreset_n) begin
  if (~lreset_n) begin
    current_state <= `IDLE;
    lpc_addr <= 0;
    io_rden_sm <= 0;
	io_wren_sm <= 0;
    lad_in_dir <= 0;
    lpc_data_in <= 0;
    lpc_en <= 0;
  end
  else begin
    case (current_state)
      `IDLE: begin
        lpc_addr <= 0;
        io_rden_sm <= 0;
		io_wren_sm <= 0;
        lad_in_dir <= 0;
        lpc_data_in <= 0;
        lpc_en <= 0;
        if ((lframe_n == 1'b0) && (lad_in == 4'h0))
          current_state <= `START;
        else
          current_state <= `IDLE;
      end
      `START: begin
        if((lframe_n == 1'b1) && (lad_in == 4'h0))
          current_state <= `IO_RD;
        else if((lframe_n == 1'b1) && (lad_in == 4'h2))
          current_state <= `IO_WR;
        else if((lframe_n == 1'b0) && (lad_in == 4'h0))
          current_state <= `START;
        else
          current_state <= `IDLE;
      end

      // LPC读状态机
      `IO_RD: begin
        if(lframe_n == 1'b1) begin
          lpc_addr[15:12] <= lad_in;
          current_state <= `IO_RD_ADDR_LCLK1;
        end
        else
          current_state <= `IDLE;
      end
      `IO_RD_ADDR_LCLK1: begin
        if(lframe_n == 1'b1) begin
          lpc_addr[11:8] <= lad_in;
          current_state <= `IO_RD_ADDR_LCLK2;
        end
        else
          current_state <= `IDLE;
      end
      `IO_RD_ADDR_LCLK2: begin
        if(lframe_n == 1'b1) begin
          lpc_addr[7:4] <= lad_in;
          current_state <= `IO_RD_ADDR_LCLK3;
        end
        else
          current_state <= `IDLE;
      end
      `IO_RD_ADDR_LCLK3: begin
        if(lframe_n == 1'b1) begin
          lpc_addr[3:0] <= lad_in;
          current_state <= `IO_RD_ADDR_LCLK4;
        end
        else
          current_state <= `IDLE;
      end
      `IO_RD_ADDR_LCLK4: begin
        if((lframe_n == 1'b1)&&(lad_in == 4'hf)) begin
          current_state <= `IO_RD_TAR_LCLK1;
          io_rden_sm <= 1;
          lpc_en <= 1;
        end
        else begin
          current_state <= `IDLE;
          io_rden_sm <= 0;
          lpc_en <= 0;
        end
      end
      `IO_RD_TAR_LCLK1: begin
        if((lframe_n == 1'b1)&&(addr_hit == 1'b1)) begin
          current_state <= `IO_RD_TAR_LCLK2;
          lad_in_dir <= 1;
          lpc_data_out <= 4'b0000;
        end
        else begin
          current_state <= `IDLE;
          lad_in_dir <= 0;
          lpc_data_out <= 0;
        end
      end
      `IO_RD_TAR_LCLK2: begin
        if(lframe_n == 1'b1) begin
          current_state <= `IO_RD_SYNC;
          lad_in_dir <= 1;
          lpc_data_out <= din[3:0];
        end
        else begin
          current_state <= `IDLE;
          lad_in_dir <= 0;
          lpc_data_out <= 0;
        end
      end
      `IO_RD_SYNC: begin
        if(lframe_n == 1'b1) begin
          current_state <= `IO_RD_DATA_LCLK1;
          lad_in_dir <= 1;
          lpc_data_out <= din[7:4];
        end
        else begin
          current_state <= `IDLE;
          lad_in_dir <= 0;
          lpc_data_out <= 0;
        end
      end
      `IO_RD_DATA_LCLK1: begin
        if(lframe_n == 1'b1) begin
          current_state <= `IO_RD_DATA_LCLK2;
          lad_in_dir <= 1;
          lpc_data_out <= 4'b1111;
        end
        else begin
          current_state <= `IDLE;
          lad_in_dir <= 0;
          lpc_data_out <= 0;
        end
      end
      `IO_RD_DATA_LCLK2: begin
          current_state <= `IDLE;
          lad_in_dir <= 0;
          lpc_data_out <= 0;
          lpc_en <= 0;
      end

      // LPC写状态机
      `IO_WR: begin
        if(lframe_n == 1'b1) begin
          lpc_addr[15:12] <= lad_in;
          current_state <= `IO_WR_ADDR_LCLK1;
        end
        else
          current_state <= `IDLE;
      end
      `IO_WR_ADDR_LCLK1: begin
        if(lframe_n == 1'b1) begin
          lpc_addr[11:8] <= lad_in;
          current_state <= `IO_WR_ADDR_LCLK2;
        end
        else
          current_state <= `IDLE;
      end
      `IO_WR_ADDR_LCLK2: begin
        if(lframe_n == 1'b1) begin
          lpc_addr[7:4] <= lad_in;
          current_state <= `IO_WR_ADDR_LCLK3;
        end
        else
          current_state <= `IDLE;
      end
      `IO_WR_ADDR_LCLK3: begin
        if(lframe_n == 1'b1) begin
          lpc_addr[3:0] <= lad_in;
          current_state <= `IO_WR_ADDR_LCLK4;
        end
        else
          current_state <= `IDLE;
      end
      `IO_WR_ADDR_LCLK4: begin
        if(lframe_n == 1'b1) begin
          current_state <= `IO_WR_DATA_LCLK1;
          lpc_data_in[3:0] <= lad_in;
        end
        else begin
          current_state <= `IDLE;
          lpc_data_in <= 0;
        end
      end
      `IO_WR_DATA_LCLK1: begin
        if(lframe_n == 1'b1) begin
          current_state <= `IO_WR_DATA_LCLK2;
          lpc_data_in[7:4] <= lad_in;
        end
        else begin
          current_state <= `IDLE;
          lpc_data_in <= 0;
        end
      end
      `IO_WR_DATA_LCLK2: begin
        if((lframe_n == 1'b1)&&(lad_in == 4'hf)) begin
          current_state <= `IO_WR_TAR_LCLK1;
          io_wren_sm <= 1;
          lpc_en <= 1;
        end
        else begin
          current_state <= `IDLE;
          io_wren_sm <= 0;
          lpc_en <= 0;
        end
      end
      `IO_WR_TAR_LCLK1: begin
        if((lframe_n == 1'b1)&&(addr_hit == 1'b1)) begin
          current_state <= `IO_WR_TAR_LCLK2;
          lad_in_dir <= 1;
          lpc_data_out <= 4'b0000;
        end
        else begin
          current_state <= `IDLE;
          lad_in_dir <= 0;
          lpc_data_out <= 0;
        end
      end
      `IO_WR_TAR_LCLK2: begin
        if(lframe_n == 1'b1) begin
          current_state <= `IO_WR_SYNC;
          lad_in_dir <= 1;
          lpc_data_out <= 4'b1111;
        end
        else begin
          current_state <= `IDLE;
          lad_in_dir <= 0;
          lpc_data_out <= 0;
        end
      end
      `IO_WR_SYNC: begin
        lad_in_dir <= 0;
        lpc_data_out <= 0;
        lpc_en <= 0;
        current_state <= `IDLE;
      end
      default: begin
        lpc_addr <= 0;
        io_rden_sm <= 0;
		io_wren_sm <= 0;
        lad_in_dir <= 0;
        lpc_data_in <= 0;
        lpc_en <= 0;
        current_state <= `IDLE;
      end
	endcase
  end
end

// 串行中断模块
reg [5:0] serirq_cnt;
reg [3:0] serirq_shift; // 用于判断起始帧00001
reg [1:0] irq_state;
wire serirq_dir;
wire serirq_data;

`define IRQIDLE          2'b00
`define IRQSTART         2'b01
`define IRQDOING         2'b10

assign int_serirq = serirq_dir ? ~serirq_data : 1'bz;
assign serirq_data = ((serirq_cnt == 6'h02) && serirq[0])				// IRQ0 
					|| ((serirq_cnt == 6'h05) && serirq[1])				// IRQ1 
					|| ((serirq_cnt == 6'h08) && serirq[2])				// IRQ2 
					|| ((serirq_cnt == 6'h0b) && serirq[3])				// IRQ3 
					|| ((serirq_cnt == 6'h0e) && serirq[4])				// IRQ4 
					//|| ((serirq_cnt == 6'h35) && serirq[4])				// IRQ4 
					|| ((serirq_cnt == 6'h11) && serirq[5])				// IRQ5 
					|| ((serirq_cnt == 6'h14) && serirq[6])				// IRQ6 
					|| ((serirq_cnt == 6'h17) && serirq[7]);			// IRQ7

assign serirq_dir = (((serirq_cnt == 6'h02) || (serirq_cnt == 6'h03)) && serirq[0])			// IRQ0
					|| (((serirq_cnt == 6'h05) || (serirq_cnt == 6'h06)) && serirq[1])		// IRQ1
					|| (((serirq_cnt == 6'h08) || (serirq_cnt == 6'h09)) && serirq[2])		// IRQ2
					|| (((serirq_cnt == 6'h0b) || (serirq_cnt == 6'h0c)) && serirq[3])		// IRQ3
					|| (((serirq_cnt == 6'h0e) || (serirq_cnt == 6'h0f)) && serirq[4])		// IRQ4
					//|| (((serirq_cnt == 6'h35) || (serirq_cnt == 6'h36)) && serirq[4])		// IRQ4
					|| (((serirq_cnt == 6'h11) || (serirq_cnt == 6'h12)) && serirq[5])		// IRQ5
					|| (((serirq_cnt == 6'h14) || (serirq_cnt == 6'h15)) && serirq[6])		// IRQ6
					|| (((serirq_cnt == 6'h17) || (serirq_cnt == 6'h18)) && serirq[7]);		// IRQ7

always @ (posedge lclk or negedge lreset_n) begin
	if (~lreset_n) begin
		irq_state <= `IRQIDLE;
		serirq_cnt <= 0;
		serirq_shift <= 4'b1111;
	end
	else begin
		case (irq_state)
			`IRQIDLE: begin
				serirq_cnt <= 0;
				serirq_shift <= 4'b1111;
				irq_state <= `IRQSTART;
			end
			`IRQSTART: begin
				serirq_shift[0] <= int_serirq;
				serirq_shift[3:1] <= serirq_shift[2:0];
				if({serirq_shift, int_serirq}== 5'b0_0001) begin
					irq_state <= `IRQDOING;
					serirq_cnt <= 2; // 这里调整计数器的位置0~3
				end
				else begin
					irq_state <= irq_state;
					serirq_cnt <= 0;
				end
			end
			`IRQDOING: begin
				serirq_cnt <= serirq_cnt + 1;
				if(serirq_cnt <= 62)
					irq_state <= irq_state;
				else
					irq_state <= `IRQIDLE;
			end
			default: begin
				serirq_cnt <= 0;
				serirq_shift <= 4'b1111;
				irq_state <= `IRQSTART;
			end
		endcase
	end
end

endmodule
