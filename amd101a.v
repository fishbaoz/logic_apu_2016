////////////////////////////////////////////////////////////////////
// FileName:  "amd-101a.v"
// Author  :  Roger.Yu
// Company :  www.icpc.cn
//////////////////////////////////////////////////////////////////// 
//`include "uart_avalon_sopc/hdl/uart_defines.v"

`define POST_CODE
`define COM0_UART



// CPLD分配地址 index(0xa80)/data(0xa81)
`define LPC_INDEX_ADD   16'h4700
`define LPC_DATA_ADD    16'h4701
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

	output HD_DSTRY,			// 硬盘销毁信号 持续低电平销毁硬盘 平时高阻态输出

	// 上电逻辑信号
	inout wire VCC_ALW_PWRGD,	// 为高电平表示0.95v_alw电OK
	output wire PWR_BTNn,		// 开机信号输出给CPU
	input SLP_S3n,				// CPU发送出来
	input SLP_S5n,				// CPU发送出来
	output wire DDR_SLP_S3n,	// 用于控制VDDIO_SUS,VTT_SUS
	output wire DDR_SLP_S5n,	// 用于控制VDDIO_SUS,VTT_SUS
	input DDR_PWROK,			// 高电平表示VDDIO_SUS,VTT_SUS电压正常
	output wire VRS_ON,			// 高电平时打开5v,3.3v,0.95v电压
	output wire V1V8_EN,		// 高电平时打开1.8v电压
	output wire V1_EN,			// 高电平使能pcie桥1V0_PCIE
	input V1V8_PWRGD,			// 高电平表示+1.8v电压正常
	input V1_PWRGD,				// 高电平表示pcie桥1V0_PCIE电压正常
	output wire APU_VRM_EN,		// 高电平使能电压输出
	input APU_VDD_PWRGD,		// 高电平表示APU_VDD_RUN,APU_VDDNB_RUN电压正常
	output wire SYS_PWRGD,      // 所有电OK后 发送到CPU
	output wire DISCHARGE_S3n,	// 低电平时放电
	output wire DISCHARGE_S5n,	// 低电平时放电

	output wire SYS_RSTn,		// 系统复位信号

	// lpc接口
	input wire LPC_CLK,			// LPC Clock 33MHz
	input LPC_RSTn,				// rst信号
	input wire LPC_FRAMEn,		// Frame - Active Low
	inout wire [3:0] LPC_AD,    // Address/Data Bus
	inout wire int_serirq,      // LPC串行中断
	output wire LPC_CLKRUNn,	// 低电平请求CPU发送LPCCLK信号
	output wire LDRQn,			// LPC DMA请求 未使用

	// CH7511 DP--->LVDS
	output wire LCDSEL,			// LCDSEL=0:单通道,双屏复制功能; LCDSEL=1:双通道模式;
	output wire RESET_CH7511Bn, // 复位CH7511芯片
	output wire [3:0] CH7511_GPIO,	// 通过不同的GPIO配置,可以读取boot rom里面16种液晶屏的配置参数.

	output wire [7:0] CPLD_LED,	// 主板前面板LED控制 低电平灯亮
	input SATA_ACT,				// 硬盘读写指示灯
	output wire [2:0] CPLD_IO,	// 至fpc调试接口的IO
	output wire CPLD_SPKR,		// 控制蜂鸣器 高电平响
	output wire ECC_CHECK,		// ECC_CHECK=1:enable ecc;=0 no ecc;

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

	// 串口
	input PLD_RX1,				// 至XP3接口
	output wire PLD_TX1,		// 至XP3接口
	input PLD_RX2,				// 至FFC调试接口
	output wire PLD_TX2,		// 至FFC调试接口

	input SPI_SCK,
	input SCLK,
	input SDATA,
	input clk_24mhz,			// 24MHz晶振
	input RESET_INn				// 前面板按钮

);

wire [15:0] reg_out;			// 上电初始化配置寄存器
wire ctrl_signal; // =1 CPLD内部初始化完成,=0 CPLD正在内部初始化
assign VCC_ALW_PWRGD = ctrl_signal ? 1'bz : 0; // RSMRST#

// 5.56MHz clk 计数
wire osc;
reg [3:0] PWR_BTN_cnt;
always @(posedge osc or negedge ctrl_signal) begin
	if(!ctrl_signal)
		PWR_BTN_cnt <= 0;
	else if(PWR_BTN_cnt < 4'hf) begin
		if(SLP_S3n & SLP_S5n)
			PWR_BTN_cnt <= PWR_BTN_cnt + 1;
		else
			PWR_BTN_cnt <= 0;
	end
	else
		PWR_BTN_cnt <= PWR_BTN_cnt;
end

assign PWR_BTNn = ctrl_signal ? (reg_out[2] ? ((PWR_BTN_cnt == 4'hf) & RESET_INn) : RESET_INn) : 1; // 开机按钮 需要按一下开机

// GROUP C
assign DDR_SLP_S3n = ctrl_signal & SLP_S3n;
assign DDR_SLP_S5n = ctrl_signal & SLP_S5n;

// GROUP D
assign VRS_ON = ctrl_signal & DDR_PWROK & SLP_S3n;
assign V1V8_EN = ctrl_signal & DDR_PWROK & SLP_S3n;
assign V1_EN = ctrl_signal & DDR_PWROK & SLP_S3n;

// GROUP E
assign APU_VRM_EN = (V1V8_PWRGD & V1_PWRGD) ? 1 : 0;

// 由于APU_VDD_PWRGD引脚没做上拉 所以此处做一个延时 默认APU电OK
reg [3:0] apu_cnt;
assign SYS_PWRGD = (V1V8_PWRGD & V1_PWRGD) & (apu_cnt==4'hf);
always @(posedge osc or negedge APU_VRM_EN) begin
	if(!APU_VRM_EN)
		apu_cnt <= 0;
	else if(apu_cnt < 4'hf)
		apu_cnt <= apu_cnt + 1;
	else
		apu_cnt <= apu_cnt;
end

// 放电逻辑
assign DISCHARGE_S3n = V1V8_EN;
assign DISCHARGE_S5n = DDR_SLP_S5n;

wire USER_RSTn;
assign SYS_RSTn = USER_RSTn;

wire lpc_en;
wire addr_hit;
wire [15:0] lpc_addr;
wire [7:0] lpc_din;
wire [7:0] lpc_dout;
wire lpc_io_wren;
wire lpc_io_rden;

wire [7:0] postcode;
wire com0_irq;

//assign RESET_CH7511Bn = ctrl_signal ? (reg_out[13] ? (SYS_PWRGD & PWR_BTNn & SYS_RSTn) : 0) : 0;
assign RESET_CH7511Bn = LPC_RSTn;
assign LCDSEL = ctrl_signal ? reg_out[3] : 1'b1; // LCDSEL=0:单通道,双屏复制功能; LCDSEL=1:双通道模式;
assign CH7511_GPIO = ctrl_signal ? reg_out[7:4] : 4'b0000; // 通过不同的GPIO配置,可以读取boot rom里面16种液晶屏的配置参数.

wire [7:0] LED_REG;				// LED指示灯寄存器
assign CPLD_LED = ctrl_signal ? 
					((reg_out[1:0]==2'b11) ? {SYS_RSTn,SATA_ACT,!SYS_PWRGD,!(V1V8_PWRGD & V1_PWRGD),!DDR_PWROK,!SLP_S5n,!SLP_S3n,!PWR_BTNn} :
					((reg_out[1:0]==2'b10) ? ~postcode :
					((reg_out[1:0]==2'b01) ? PEX_LANE_STATEn :
					((reg_out[1:0]==2'b00) ? ~LED_REG : 8'h00)))) : 8'h00;

assign CPLD_IO[2] = 1'bz;
assign CPLD_IO[1] = LPC_CLK;
assign CPLD_IO[0] = int_serirq;

assign ECC_CHECK = 1'bz;
assign PCIE_STRAP = 4'b0000; // upstream端口选择 默认port 0,nt端口和upstream端口不能一样
assign NT_STRAP = 4'b0001;	// NT模式端口选择 port 1
assign NT_ENABLEn = ctrl_signal ? reg_out[9] : 1'b1; // 高电平为普通模式,低电平为NT模式
assign SSC_ISO_ENAn = ctrl_signal ? (reg_out[10] ? 1'bz : 1'b0) : 1'bz; // 高电平(高阻态)为统一时钟模式,低电平为隔离时钟模式
assign NT_P2P_ENn = ctrl_signal ? (reg_out[11] ? 1'bz : 1'b0) : 1'bz; // 配置NT模式的PCI-TO-PCI桥是否使用，这里使用Legacy NT MODE,置高阻(外部上拉到1)
assign UPCFG_TIMER_ENn = ctrl_signal ? (reg_out[12] ? 1'bz : 1'b0) : 1'bz;	// 当置高阻(外部上拉到1)时，速率可以设置为PCIE 2.0，并且支持自适应速率
																			// 当置低电平时，如果第一次握手失败，桥片设置为PCIE 1.0，并且不支持自适应速率
assign PLD_TX2 = PLD_TX1;
assign LPC_CLKRUNn = 1'bz;
assign LDRQn = 1'bz;
assign CPLD_SPKR = 1'bz;
assign PCIE_CLKREQn = 5'b0_0001;
assign S_TALERTn = 1;
assign THERMTRIPn = 1;
assign SHPC_INTn = 1'bz; // 未使用
assign DP0_CAB_DP_HDMIn = 1; // dp0 dp
assign DP1_CAB_DP_HDMIn = 0; // dp1 dvi

wire [7:0] lpc_reg;
// CPLD 地址分配index/data 地址匹配
assign addr_hit =	`ifdef POST_CODE
						(lpc_addr == `LPC_POST_ADD) || 
					`endif
					`ifdef COM0_UART
						//((lpc_addr >= `LPC_COM0_ADD) && (lpc_addr <= (`LPC_COM0_ADD+7))) ||
						(lpc_reg[0] && (lpc_addr >= `LPC_COM0_ADD) && (lpc_addr <= (`LPC_COM0_ADD+7))) ||
					`endif
					`ifdef COM1_UART
						(lpc_reg[1] && (lpc_addr >= `LPC_COM1_ADD) && (lpc_addr <= (`LPC_COM1_ADD+7))) ||
					`endif
					(lpc_addr == `LPC_INDEX_ADD) || (lpc_addr == `LPC_DATA_ADD);

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
	.int_serirq(int_serirq),		// 串行中断输入
	//.serirq({3'b000,com0_irq,4'b0000})							// 中断输入 高电平有效
	.serirq({3'b000,com0_irq,4'b000})
);

// 寄存器模块 UFM模块 index/data 模式操作
LPC_Device LPC_Device(
	.SLP_S5n(SLP_S5n),
	.lclk(LPC_CLK), // Clock
	.lreset_n(LPC_RSTn), // Reset - Active Low (Same as PCI Reset)
	.lpc_en(lpc_en), // 后端总线使能信号,高电平时总线有效
	.device_cs(addr_hit && (lpc_addr == `LPC_DATA_ADD)),
	.addr(index_add), // 地址
	.din(lpc_dout),		
	.dout(lpc_din),		
	.io_rden(lpc_io_rden),
	.io_wren(lpc_io_wren),

	.INHMDG(INHMDG),					// =1看门狗禁止,=0看门狗启用
	.DPM_RSTn(DPM_RSTn),				// DPM模块输出给模块的复位信号
	.HPM_RSTn(HPM_RSTn),				// 主控模块输出给模块的复位信号
	.MGM_RSTn(MGM_RSTn),				// 外部按钮手动复位信号
	.PSMRSTn(PSMRSTn),				// 电源模块输出给模块的复位信号
	.MGM_STS(MGM_STS),			// PCI-E状态指示信号
	.USER_RSTn(USER_RSTn),

	.SYS_MAIN(SYS_MAIN),				// 意义待定 系统监控
	.GSE(GSE),					// 意义待定 地面/空中
	.SYS(SYS),					// 意义待定 系统调试
	.SYS_ID(SYS_ID),			// 槽位识别

	.LED_REG(LED_REG),
	.osc(osc),
	.ctrl_signal(ctrl_signal),	// =0,上电自读取;=1,外部控制
	.lpc_reg(lpc_reg),
	.reg_out(reg_out) // 寄存器输出
);

`ifdef POST_CODE
	// postcode
	LPC_POSTCODE LPC_POSTCODE(
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
`else
	assign postcode = 8'hffff;
`endif

`ifdef COM0_UART
	// 串口内核 COM0
	LPC_COM LPC_COM0(
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
`else
	assign PLD_TX1 = 1;
	assign com0_irq = 0;
`endif


endmodule

