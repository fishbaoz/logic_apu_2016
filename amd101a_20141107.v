////////////////////////////////////////////////////////////////////
// FileName:  "amd-101a.v"
// Author  :  Roger.Yu
// Company :  www.icpc.cn
//////////////////////////////////////////////////////////////////// 

`include "uart_avalon_sopc/hdl/uart_defines.v"



// CPLD分配地址 index(0xa80)/data(0xa81)
`define LPC_INDEX_ADD   16'ha80
`define LPC_DATA_ADD    16'ha81
`define LPC_POST_ADD    16'h080
`define LPC_COM0_ADD    16'h3F8
`define LPC_COM1_ADD    16'h2F8

module amd101a 
(

	/**************************************************
	//PCI-E的状态输出信号(MGM_STS):
	//由驱动程序控制这个信号的输出:高电平表示模块上电初始化后工作正常
	//4个外部复位信号中任一个输入到模块后，模块将PCI-E的状态输出信号
	//(MGM_STS)置为无效500ms后，模块产生硬复位。
	***************************************************/
	output wire MGM_STS,		// PCI-E状态指示信号
	input INHMDG,				// 禁止看门狗信号
	input DPM_RSTn,				// DPM模块输出给模块的复位信号
	input HPM_RSTn,				// 主控模块输出给模块的复位信号
	input MGM_RSTn,				// 外部按钮手动复位信号
	input PSMRSTn,				// 电源模块输出给模块的复位信号
	
	input SYS_MAIN,				// 意义待定 系统监控
	input GSE,					// 意义待定 地面/空中
	input SYS,					// 意义待定 系统调试

	input [1:0] SYS_ID,			// 槽位识别

	output wire [7:0] CPLD_LED,	// 主板前面板LED控制 低电平灯亮
	output HD_DSTRY,			// 硬盘销毁信号 持续低电平销毁硬盘 平时高阻态输出

	inout wire VCC_ALW_PWRGD,	// 为高电平表示0.95v_alw电OK
	output wire PWR_BTNn,		// 开机信号输出给CPU
	input SLP_S3n,				// CPU发送出来
	input SLP_S5n,				// CPU发送出来

	inout wire [2:0] CPLD_IO,	// 至fpc调试接口的IO
	output wire CPLD_SPKR,		// 控制蜂鸣器 高电平响
	output wire DDR_SLP_S3n,	// 用于控制VDDIO_SUS,VTT_SUS
	output wire DDR_SLP_S5n,	// 用于控制VDDIO_SUS,VTT_SUS
	input DDR_PWROK,			// 高电平表示VDDIO_SUS,VTT_SUS电压正常
	output wire VRS_ON,			// 高电平时打开5v,3,3v,0.95v电压
	output wire V1V8_EN,		// 高电平使能+1.8v电压
	input V1V8_PWRGD,			// 高电平表示+1.8v电压正常
	output wire DISCHARGE_S3n,	// 低电平时放电
	output wire DISCHARGE_S5n,	// 低电平时放电
	output wire V1_EN,			// 高电平使能pcie桥1V0_PCIE
	input V1_PWRGD,				// 高电平表示pcie桥1V0_PCIE电压正常
	input APU_VDD_PWRGD,		// 高电平表示APU_VDD_RUN,APU_VDDNB_RUN电压正常
	output wire APU_VRM_EN,		// 高电平使能电压输出
	output wire SYS_PWRGD,      // 所有电OK后 发送到CPU
	output wire ECC_CHECK,		// ECC_CHECK=1:enable ecc;=0 no ecc;
	output wire SYS_RSTn,		// 系统复位信号

	// cpu gpio
	output wire [4:0] PCIE_CLKREQn,
	output wire S_TALERTn,		// 温度报警
	output wire THERMTRIPn,
	input BLINKn,				// 用于指示灯
	output DP0_CAB_DP_HDMIn,	// 高电平为DP端口，低电平为DVI端口 DP0--->LVDS
	output DP1_CAB_DP_HDMIn,	// 高电平为DP端口，低电平为DVI端口 DP1配置为DVI

	// pex8609
	output wire [3:0] PCIE_STRAP,	// upstream端口选择 默认port 0,nt端口和upstream端口不能一样
	output wire NT_ENABLEn,		// 高电平为普通模式,低电平为NT模式
	output wire [3:0] NT_STRAP,	// NT模式时端口选择 应该是port 1, nt端口和upstream端口不能一样
	output wire SSC_ISO_ENAn,	// 高电平(高阻态)为统一时钟模式,低电平为隔离时钟模式
	input PEX_NT_RESETn,		// 未使用
	input [7:0] PEX_LANE_STATEn,		// PEX LANE状态 高电平为关闭，低电平pcie 2.0，发出脉冲为pcie 1.0 未使用
	input PEX_INTAn,			// PEX桥发出的中断 未使用
	input FATAL_ERRn,			// PEX桥发出的致命错误信号 未使用
	output wire SHPC_INTn,		// 热拔插信号 普通保持高电平 未使用
	output wire NT_P2P_ENn,		// 配置NT模式的PCI-TO-PCI桥是否使用，这里使用Legacy NT MODE,置高阻(外部上拉到1)
	output wire UPCFG_TIMER_ENn,	// 当置高阻(外部上拉到1)时，速率可以设置为PCIE 2.0，并且支持自适应速率
									// 当置低电平时，如果第一次握手失败，桥片设置为PCIE 1.0，并且不支持自适应速率

	// lpc接口
	input wire LPC_CLK,			// LPC Clock 33MHz
	input LPC_RSTn,				// rst信号
	input wire LPC_FRAMEn,		// Frame - Active Low
	inout wire [ 3:0] LPC_AD,   // Address/Data Bus
	inout wire int_serirq,      // LPC串行中断 未使用
	output wire LPC_CLKRUNn,	// 低电平请求CPU发送LPCCLK信号
	output wire LDRQn,			// LPC DMA请求 未使用

	// 串口
	input PLD_RX1,				// 至XP3接口
	output wire PLD_TX1,		// 至XP3接口
	input PLD_RX2,				// 至FFC调试接口
	output wire PLD_TX2,		// 至FFC调试接口

	// CH7511 DP--->LVDS
	output wire LCDSEL,			// LCDSEL=0:单通道,双屏复制功能; LCDSEL=1:双通道模式;
	output wire RESET_CH7511Bn, // 复位CH7511芯片
	output wire [3:0] CH7511_GPIO,	// 通过不同的GPIO配置,可以读取boot rom里面16种液晶屏的配置参数.

	input SPI_SCK,
	input clk_24mhz,			// 24MHz晶振
	input RESET_INn				// 前面板按钮

);

wire ctrl_signal;
assign VCC_ALW_PWRGD = ctrl_signal ? 1'bz : 0; // RSMRST#
assign PWR_BTNn = ctrl_signal ? RESET_INn : 1;
assign DDR_SLP_S3n = ctrl_signal & SLP_S3n;
assign DDR_SLP_S5n = ctrl_signal & SLP_S5n;



reg [15:0] firsttime_cnt;		// 自复位计时寄存器
wire MYSELF_RSTn;				// 内部使用的复位信号
wire [7:0] CPLD_IO_DIR;
wire [7:0] CPID_IO_OUT;
wire addr_hit;
wire [15:0] lpc_addr;
wire [7:0] lpc_dout;
wire [7:0] postcode;

wire lpc_io_wren;
wire lpc_en;
wire [7:0] lpc_din;
wire lpc_io_rden;
wire [15:0] reg_out;			// 上电初始化配置寄存器
wire [7:0] LED_REG;				// LED指示灯寄存器
wire osc;
wire com0_irq;

assign MGM_STS = 1;
assign LED_REG = 8'h5a;

assign int_serirq = 1'bz;
assign LPC_CLKRUNn = 1'bz;
assign LDRQn = 1'bz;
assign CPLD_SPKR = 1'bz;
assign SYS_RSTn = MGM_RSTn;

assign VRS_ON = MYSELF_RSTn & DDR_PWROK & SLP_S3n;
assign V1V8_EN = VRS_ON;
assign DISCHARGE_S3n = V1V8_EN;
assign DISCHARGE_S5n = MYSELF_RSTn & SLP_S5n;
assign V1_EN = VRS_ON;
assign APU_VRM_EN = (V1V8_PWRGD & V1_PWRGD) ? 1'bz : 1'b0;
assign SYS_PWRGD = (V1V8_PWRGD & V1_PWRGD) & APU_VDD_PWRGD;
assign PCIE_CLKREQn = 5'b0_0001;
assign S_TALERTn = 1;
assign THERMTRIPn = 1;
//assign RESET_CH7511Bn = ctrl_signal ? (reg_out[13] ? (PWR_BTNn & SYS_RSTn) : 0) : (PWR_BTNn & SYS_RSTn);
assign RESET_CH7511Bn = PWR_BTNn & SYS_RSTn;


// 未使用
assign SHPC_INTn = 1'bz;
assign DP0_CAB_DP_HDMIn = 1; // dp0 dp
assign DP1_CAB_DP_HDMIn = 0; // dp1 dvi

assign CPLD_LED = ctrl_signal ? 
					((reg_out[1:0]==2'b11) ? {1'b1,!SYS_PWRGD,!(V1V8_PWRGD & V1_PWRGD),!DDR_PWROK,!SLP_S5n,!SLP_S3n,!VCC_ALW_PWRGD,!RESET_INn} :
					((reg_out[1:0]==2'b10) ? ~postcode :
					((reg_out[1:0]==2'b01) ? PEX_LANE_STATEn :
					((reg_out[1:0]==2'b00) ? LED_REG : 8'h00)))) : 8'h00;
assign PCIE_STRAP = 4'b0000; // upstream端口选择 默认port 0,nt端口和upstream端口不能一样
assign NT_STRAP = 4'b0001;	// NT模式端口选择 port 1
assign ECC_CHECK = ctrl_signal ? ~reg_out[8] : 1'b0; // =1 enable ecc; =0 no ecc;
assign NT_ENABLEn = ctrl_signal ? reg_out[9] : 1'b1; // 高电平为普通模式,低电平为NT模式
assign SSC_ISO_ENAn = ctrl_signal ? (reg_out[10] ? 1'bz : 1'b0) : 1'bz; // 高电平(高阻态)为统一时钟模式,低电平为隔离时钟模式
assign NT_P2P_ENn = ctrl_signal ? (reg_out[11] ? 1'bz : 1'b0) : 1'bz; // 配置NT模式的PCI-TO-PCI桥是否使用，这里使用Legacy NT MODE,置高阻(外部上拉到1)
assign UPCFG_TIMER_ENn = ctrl_signal ? (reg_out[12] ? 1'bz : 1'b0) : 1'bz;	// 当置高阻(外部上拉到1)时，速率可以设置为PCIE 2.0，并且支持自适应速率
																			// 当置低电平时，如果第一次握手失败，桥片设置为PCIE 1.0，并且不支持自适应速率
assign LCDSEL = ctrl_signal ? reg_out[3] : 1'b1; // LCDSEL=0:单通道,双屏复制功能; LCDSEL=1:双通道模式;
//assign CH7511_GPIO = ctrl_signal ? reg_out[7:4] : 4'b0000; // 通过不同的GPIO配置,可以读取boot rom里面16种液晶屏的配置参数.
assign CH7511_GPIO = 4'b0001; // 通过不同的GPIO配置,可以读取boot rom里面16种液晶屏的配置参数.

assign CPLD_IO_DIR = 3'b000;
assign CPID_IO_OUT = 3'b111;
assign CPLD_IO[2] = CPLD_IO_DIR[2] ? CPID_IO_OUT[2] : 1'bz;
assign CPLD_IO[1] = CPLD_IO_DIR[1] ? CPID_IO_OUT[1] : 1'bz;
assign CPLD_IO[0] = CPLD_IO_DIR[0] ? CPID_IO_OUT[0] : 1'bz;

assign PLD_TX2 = PLD_TX1;

// 5.56MHz clk 计数0x15b8大约延时1s
assign MYSELF_RSTn = (firsttime_cnt == 16'h15b8);

always @(posedge osc) begin
	if(firsttime_cnt < 16'h15b8)
		firsttime_cnt <= firsttime_cnt + 16'h1;
	else
		firsttime_cnt <= firsttime_cnt;
end

// CPLD 地址分配index/data 地址匹配
assign addr_hit =	(lpc_addr == `LPC_INDEX_ADD) || (lpc_addr == `LPC_DATA_ADD) || 
					(lpc_addr == `LPC_POST_ADD) || 
					((lpc_addr >= `LPC_COM0_ADD) && (lpc_addr <= (`LPC_COM0_ADD+7))) ||
					((lpc_addr >= `LPC_COM1_ADD) && (lpc_addr <= (`LPC_COM1_ADD+7)));

// index data reg
reg [7:0] index_add;
always @(posedge LPC_CLK or negedge LPC_RSTn) begin
	if(!LPC_RSTn)
		index_add <= 0;
	else if(lpc_en & lpc_io_wren & (lpc_addr == `LPC_INDEX_ADD))
		index_add <= lpc_dout;
	else
		index_add <= index_add;
end

LPC_Peri LPC_Peri(
	// LPC Interface
	.lclk(LPC_CLK),					// Clock
	.lreset_n(LPC_RSTn),			// Reset - Active Low (Same as PCI Reset)
	.lframe_n(LPC_FRAMEn),			// Frame - Active Low
	.lad_in(LPC_AD),				// Address/Data Bus

	.lpc_data_out(),				// 用于测试

	.lpc_en(lpc_en),				// 后端总线使能信号,高电平时总线有效
	.addr_hit(addr_hit),			// 地址匹配置1,不匹配置0
	.lpc_addr(lpc_addr),			// LPC地址
	.din(lpc_din),					// LPC读的时候后端输入的数据
	.lpc_data_in(lpc_dout),         // LPC写的时候给后端的输出数据
	.io_rden_sm(lpc_io_rden),       // 读使能信号
	.io_wren_sm(lpc_io_wren),       // 写使能信号
	.int_serirq(int_serirq),					// 串行中断
	.serirq({3'b000,com0_irq,4'b0000})							// 中断输入 高电平有效
);

// 寄存器模块 UFM模块 占用0xa80/0xa81 index/data 模式操作
LPC_Device1 LPC_Device1(
	.lclk(LPC_CLK), // Clock
	.lreset_n(LPC_RSTn), // Reset - Active Low (Same as PCI Reset)
	.lpc_en(lpc_en), // 后端总线使能信号,高电平时总线有效
	.device_cs(addr_hit && (lpc_addr == `LPC_DATA_ADD)),
	.addr(index_add), // 地址
	.din(lpc_dout),
	.dout(lpc_din),
	.io_rden(lpc_io_rden),
	.io_wren(lpc_io_wren),

	.osc(osc),
	.ctrl_signal(ctrl_signal),	// =0,上电自读取;=1,外部控制
	.reg_out(reg_out) // 寄存器输出
);

/*
// 20141107 占用0x2f8地址的ufm
// 寄存器模块 UFM模块
LPC_Device1 LPC_Device1(
  .lclk(LPC_CLK), // Clock
  .lreset_n(LPC_RSTn), // Reset - Active Low (Same as PCI Reset)
  .lpc_en(lpc_en), // 后端总线使能信号,高电平时总线有效
  .device_cs(addr_hit && (lpc_addr >= `LPC_COM1_ADD) && (lpc_addr <= `LPC_COM1_ADD+7)),
  .addr(lpc_addr - `LPC_COM1_ADD), // 地址
  .din(lpc_dout),
  .dout(lpc_din),
  .io_rden(lpc_io_rden),
  .io_wren(lpc_io_wren),

  .osc(osc),
  .ctrl_signal(ctrl_signal), // =0,上电自读取;=1,外部控制
  .reg_out(reg_out)
);
*/













// postcode
LPC_Device0 LPC_Device0(
  .lclk(LPC_CLK), // Clock
  .lreset_n(LPC_RSTn), // Reset - Active Low (Same as PCI Reset)
  .lpc_en(lpc_en), // 后端总线使能信号,高电平时总线有效
  .device_cs(addr_hit & (lpc_addr == `LPC_POST_ADD)),
  .addr(index_add), // 地址
  .din(lpc_dout),
  .dout(),
  .io_rden(),
  .io_wren(lpc_io_wren),

  .postcode(postcode)
);

// 串口内核 COM0
LPC_Device2 LPC_Device2(
	.lclk(LPC_CLK),					// Clock 33MHz
	.lreset_n(LPC_RSTn),				// Reset - Active Low (Same as PCI Reset)
	.lpc_en(lpc_en),				// 后端总线使能信号,高电平时总线有效
	.device_cs(addr_hit && (lpc_addr >= `LPC_COM0_ADD) && (lpc_addr <= `LPC_COM0_ADD+7)),
	.addr(lpc_addr - `LPC_COM0_ADD),			// 地址
	.din(lpc_dout),
	.dout(lpc_din),
	.io_rden(lpc_io_rden),
	.io_wren(lpc_io_wren),
	.com_irq(com0_irq),

	.clk_24mhz(clk_24mhz),			// 24MHz时钟输入
	.tx(PLD_TX1),
	.rx(PLD_RX1),
	.baud_clk()			// 波特率时钟输出
);


endmodule

