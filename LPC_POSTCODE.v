// --------------------------------------------------------------------
// FileName:  "LPC_Device0.v"
// Author  :  Roger.Yu
// Company :  www.icpc.cn
// receive 80 port postcode to led
// --------------------------------------------------------------------

module LPC_POSTCODE 
(
  input lclk				, // Clock
  input lreset_n			, // Reset - Active Low (Same as PCI Reset)
  input lpc_en				, // 后端总线使能信号,高电平时总线有效
  input device_cs			,
  input [15:0] addr			, // 地址
  input [7:0] din           ,
  output wire [7:0] dout     ,
  input io_rden				,
  input io_wren				,

  output reg [7:0] postcode

);

assign dout = 8'hzz;

// 接收LPC总线80端口发出的POSTCODE代码
always @ (posedge lclk or negedge lreset_n) begin
	if(!lreset_n)
		postcode <= 0;
	else if(device_cs & io_wren & lpc_en)
		postcode <= din;
	else
		postcode <= postcode;
end

endmodule
