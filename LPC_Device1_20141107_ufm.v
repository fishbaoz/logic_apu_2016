// --------------------------------------------------------------------
// FileName:  "LPC_Device1.v"
// Author  :  Roger.Yu
// Company :  www.icpc.cn
// use max ii ufm, can read and write.
// in begin auto read address 0x01 data send out.
// --------------------------------------------------------------------

module LPC_Device1 
(
  input lclk,				// Clock
  input lreset_n,			// Reset - Active Low (Same as PCI Reset)
  input lpc_en,				// 后端总线使能信号,高电平时总线有效
  input device_cs,
  input [15:0] addr,		// 地址
  input [7:0] din,
  output reg [7:0] dout,
  input io_rden,
  input io_wren,

  output wire osc,
  output reg ctrl_signal,	// =0,上电自读取;=1,外部控制
  output reg [15:0] reg_out // 寄存器输出
);

reg [15:0] firsttime_cnt;
wire MYSELF_RSTn;

reg [8:0] addr_ufm,addr_ufm_init;
reg [15:0] datain;
reg nerase;
reg nread,nread_init;
reg nwrite;
wire data_valid;
wire [15:0] dataout;
wire nbusy;

assign MYSELF_RSTn = (firsttime_cnt == 16'h35f);

always @(posedge osc) begin
	if(firsttime_cnt < 16'h35f)
		firsttime_cnt <= firsttime_cnt + 16'h1;
	else
		firsttime_cnt <= firsttime_cnt;
end

reg [3:0] state;
always @ (posedge osc or negedge MYSELF_RSTn) begin
	if(!MYSELF_RSTn) begin
		state <= 4'b0000;
		reg_out <= 0;
		ctrl_signal <= 0;
		addr_ufm_init <= 0;
		nread_init <= 1;
	end
	else begin
		case (state)
			4'b0000: begin
				state <= 4'b0001;
				reg_out <= 0;
				ctrl_signal <= 0;
				addr_ufm_init <= 0;
				nread_init <= 1;
			end
			4'b0001: begin
				if(data_valid & nbusy) begin
					state <= 4'b0010;
					reg_out <= dataout;
					ctrl_signal <= 1;
					nread_init <= 1;
					addr_ufm_init <= 0;
				end
				else begin
					state <= state;
					reg_out <= reg_out;
					ctrl_signal <= 0;
					addr_ufm_init <= 9'h1;
					nread_init <= 0;
				end
			end
			4'b0010: begin
				state <= state;
				reg_out <= reg_out;
				ctrl_signal <= 1;
				addr_ufm_init <= 0;
				nread_init <= 1;
			end
			default: begin
				state <= 4'b0000;
				reg_out <= 0;
				ctrl_signal <= 0;
				addr_ufm_init <= 0;
				nread_init <= 1;
			end
		endcase
	end
end

always @ (posedge lclk or negedge lreset_n) begin
	if(!lreset_n) begin
		addr_ufm <= 0;
		datain <= 0;
		nerase <= 1;
		nread <= 1;
		nwrite <= 1;
		dout <= 8'hzz;
	end
	else if(device_cs & io_rden & lpc_en) begin
		if(addr==0)
			dout <= {1'b0,ctrl_signal,data_valid,nbusy,nerase,nwrite,nread,addr_ufm[8]};
		else if(addr==1)
			dout <= addr_ufm[7:0];
		else if(addr==2)
			dout <= dataout[15:8];
		else if(addr==3)
			dout <= dataout[7:0];
		else
			dout <= 8'hzz;
	end
	else if(device_cs & io_wren & lpc_en) begin
		dout <= 8'hzz;
		if(addr==0) begin
			addr_ufm[8] <= din[0];
			nread <= din[1];
			nwrite <= din[2];
			nerase <= din[3];
		end
		else if(addr==1)
			addr_ufm[7:0] <= din;
		else if(addr==2)
			datain[15:8] <= din;
		else if(addr==3)
			datain[7:0] <= din;
	end
	else begin
		addr_ufm <= addr_ufm;
		datain <= datain;
		nerase <= nerase;
		nread <= nread;
		nwrite <= nwrite;
		dout <= 8'hzz;
	end
end

para_ufm para_ufm(
	.addr ( ctrl_signal ? addr_ufm : addr_ufm_init ),
	.datain ( datain ),
	.nerase ( 1 ),
	.nread ( ctrl_signal ? nread : nread_init ),
	.nwrite ( ctrl_signal ? nwrite : 1 ),
	.data_valid ( data_valid ),
	.dataout ( dataout ),
	.nbusy ( nbusy ),
	.osc ( osc )
);
endmodule
