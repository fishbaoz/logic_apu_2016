////////////////////////////////////////////////////////////////////
// FileName:  "apu.v"
// Author  :  lisg
// Company :  careri
//////////////////////////////////////////////////////////////////// 
//`include "uart_avalon_sopc/hdl/uart_defines.v"

`define POST_CODE
//`define COM0_UART



// CPLD分配地址 index(0xa80)/data(0xa81)
`define LPC_INDEX_ADD   16'h4700
`define LPC_DATA_ADD    16'h4701
`define LPC_POST_ADD    16'h080
`define LPC_COM0_ADD    16'h3F8
`define LPC_COM1_ADD    16'h2F8

module apu 
(
	/**************************************************
	//PCI-E的状态输出信号(MGM_STS):
	//由驱动程序控制这个信号的输出:高电平表示模块上电初始化后工作正常
	//4个外部复位信号中任一个输入到模块后，模块将PCI-E的状态输出信号
	//(MGM_STS)置为无效500ms后，模块产生硬复位。
	***************************************************/
	output wire [8:0] DIS_OUT,		// 离散量输出，其中DIS_OUT[8] = MGM_STS，PCI-E状态指示信号
	input wire [13:2] DIS_IN,		// 离散量输入
	
	// XMC
	input wire XMC_MRSTIn,			// XMC输入的复位信号
	output wire XMC_MRSTOn,			// XMC输出的复位信号
	output wire XMC_PRESENTn,		// 卡在位信号，启动后为0
	input wire XMC_MSDA,			// 外部I2C
	input wire XMC_MSCL,			// 外部I2C
	
	
	input wire INHMDG,			// 禁止看门狗信号
	input wire SYS_MAIN,			// 意义待定 系统监控
	input wire GSE,				// 意义待定 地面/空中
	input wire [1:0] SYS_ID,		// 槽位识别
	input wire [1:0] SS,			// 工作状态信号
	input wire L_R,				// 位置左右识别
	
	
	input wire PCIE_RSTn,			// CPU输出的PCIE复位信号
	input wire PSMRSTn,			// 电源模块输出给模块的复位信号
	input wire RESET_INn,			// 调试复位按钮
	output wire SYS_RSTn,			// 系统复位信号
//	input wire ISP_IO3n,			// BIOS 烧结口复位
	output wire [2:0] RST_On,		// 复位输出信号
	output wire ETH_RSTn,			// eth reset
	
	
	output wire PWR_BTNn,			// 开机信号输出给CPU
	
	input wire clk_24mhz,			// 24MHz晶振
	
	output wire APU_VRM_EN,			// 高电平使能电压输出
	input wire SYS_PWRGD,      		// 所有电OK后 发送到CPU
	input wire SLP_S3n,			// CPU发送出来
	input wire SLP_S5n,			// CPU发送出来
	output wire VRS_ON,			// 高电平时打开5v,3.3v,0.95v电压
//	input wire V0V95_ALW_PWRGD,		// 0.95V alw POWER GOOD
	output wire DISCHARGE_S3n,		// 低电平时放电
	output wire DISCHARGE_S5n,		// 低电平时放电
	input wire V1V8_PWRGD,			// 高电平表示+1.8v电压正常
	input wire V3V3_ALW_PWRGD,		// 3.3V POWER GOOD
	output wire V1V8_EN,			// 高电平时打开1.8v电压
	input wire APU_VDD_PWRGD,		// 高电平表示APU_VDD_RUN,APU_VDDNB_RUN电压正常
//	input wire DDR_PWROK,			// 高电平表示VDDIO_SUS,VTT_SUS电压正常
//	output wire APU_VDDNB_PWRGD,		// 必须置为1
//	input wire V0V95_PWRGD,			// 0.95V POWER GOOD
	input wire V1V_PWRGD,			// 1V POWER GOOD (CPU 0.95V, ETH 1V)
	input wire V1V5_PWRGD,		// 1.5V alw POWER GOOD
//	input wire V1V8_ALW_PWRGD,		// 1.8V alw POWER GOOD
//	input wire LAN_1V9_PWRGD,		// lan 1.9V POWER GOOD
	input wire NAND_1V2_PWRGD,		// nand flash 1.2V POWER GOOD
	output wire EN_1V5,				// ddr power enable, slp_5#
	input wire V1V2_PWRGD,		// 1.2V POWER GOOD
	output wire SYS_PWR_GOOD_ALL,	// POWER ON FINISH
	
	
	input wire APU_PROCHOTn,		// 
	
	
	
	// lpc接口
	input wire LPC_CLK,			// LPC Clock 33MHz 忘记定义了
	input wire LPC_RSTn,			// rst信号
	input wire LPC_FRAMEn,			// Frame - Active Low
	inout wire [3:0] LPC_AD,    		// Address/Data Bus
	inout wire int_serirq,      		// LPC串行中断
	output wire LPC_CLKRUNn,		// 低电平请求CPU发送LPCCLK信号
	output wire LDRQn,			// LPC DMA请求 未使用
	
	input wire SD_LED,			// SD 卡指示灯，没有使用
	
	input wire [2:0] LAN1_LED,		// 以太网指示灯
	input wire [2:0] LAN2_LED,		// 以太网指示灯
	
	output wire PCIE_WAKE_UPn,		// PCIE WAKE UP
	input wire ETH1_WAKE_UPn,		// ETH1 PCIE WAKE UP
	input wire ETH2_WAKE_UPn,		// ETH2 PCIE WAKE UP
	
	input wire SATA_ACT,				// 硬盘读写指示灯
	
	output wire [7:0] CPLD_LED,		// 主板前面板LED控制 低电平灯亮
	output wire ECC_CHECK,			// ECC_CHECK=1:enable ecc;=0 no ecc;
	output wire CPLD_SPKR,			// 控制蜂鸣器 高电平响
	output wire [2:0] CPLD_IO,	// 至fpc调试接口的IO
	input wire SPKR,			// CPU speaker
	input wire APU_ALERT,		// CPU alert ?
	input wire APU_FANTACH0,	// CPU FAN CONTROL
	input wire APU_FANOUT0,		// CPU FAN CONTROL
	
	// 串口
	input wire PLD_RX1,			// 串口1
	output wire PLD_TX1,			// 串口1
	input wire PLD_RX2,			// 串口2
	output wire PLD_TX2,			// 串口2
	
	input SCLK,
	input SDATA,
	
	output wire SA0,			// spd eeprom ad0

	// cpu gpio
	output wire [4:0] PCIE_CLKREQn,
	output wire S_TALERTn,			// 温度报警
	output wire DP0_CAB_DP_HDMIn,		// 高电平为DP端口，低电平为DVI端口 DP0--->LVDS
	output wire DP1_CAB_DP_HDMIn,		// 高电平为DP端口，低电平为DVI端口 DP1配置为DVI
	output wire THERMTRIPn,
	input wire BLINKn,			// 用于指示灯
	
	
	input wire FAN0_V,
	input wire APU_SIC,
	input wire APU_SID,
	
	output wire AMP_CLRn,
	input wire AMP_ALERTn,
//	input wire TEMP_ALERTn,
//	output wire VLT_RSTn
	input wire [10:3] CPLD1_IO
);

wire MGM_STS;
wire DDR_SLP_S5n;
wire [7:0] PG0;
wire [7:0] PG1;
wire RST_STS;
reg [31:0] counter;
wire USER_RSTn;

//assign APU_VDDNB_PWRGD = 1'b1;
//assign V1V8_PWRGD = DIS_IN[13];
assign SA0 = DIS_IN[13];

assign PCIE_WAKE_UPn = 1'b1;	// ETH1_WAKE_UPn & ETH2_WAKE_UPn;

assign AMP_CLRn = 1'b1;
//assign VLT_RSTn = 1'b1;
//assign DIS_OUT[7:0] = CPLD_LED;//8'hzz;
//assign DIS_OUT[2] = CPLD_LED[0];
//assign DIS_OUT[3] = CPLD_LED[1];
//assign DIS_OUT[1] = CPLD_LED[2];
//assign RST_On[0] = CPLD_LED[3];
//assign DIS_OUT[0] = CPLD_LED[4];
//assign RST_On[2] = CPLD_LED[5];
//assign DIS_OUT[6] = CPLD_LED[6];
//assign DIS_OUT[5] = CPLD_LED[7];
//assign DIS_OUT[7] = CPLD_SPKR;
//assign DIS_OUT[4] = 1'b0;

assign VLT_RSTn = 1'b1;

assign DIS_OUT[2] = APU_VRM_EN;
assign DIS_OUT[3] = SLP_S3n;
assign DIS_OUT[1] = SLP_S5n;
assign RST_On[0] = VRS_ON;
assign DIS_OUT[0] = V1V8_PWRGD;
assign RST_On[2] = V3V3_ALW_PWRGD;
assign DIS_OUT[6] = SYS_PWRGD;
assign DIS_OUT[5] = V1V_PWRGD;
assign DIS_OUT[7] = SPKR;
assign DIS_OUT[4] = 1'b1;

assign RST_On[1] = USER_RSTn;//{USER_RSTn, USER_RSTn, USER_RSTn};

assign SYS_RSTn = USER_RSTn;//RST_STS ? 1'bz : 1'b0;
//assign SYS_RSTn = (RST_STS & (counter == 6672000)) ? 1'bz : 1'b0;
//assign SYS_RSTn = DIS_IN[12] ? 1'bz : 1'b0;

assign XMC_MRSTOn = USER_RSTn;
assign XMC_PRESENTn = USER_RSTn ? 1'b0 : 1'bz;
assign ETH_RSTn = USER_RSTn;

assign PG0 = {APU_VRM_EN, SYS_PWRGD, SLP_S3n, SLP_S5n, VRS_ON, 1'b0, V1V8_PWRGD, V3V3_ALW_PWRGD};
assign PG1 = {V1V8_EN, APU_VDD_PWRGD, 1'b0, V1V_PWRGD, V1V5_PWRGD, 1'b0, 1'b0, NAND_1V2_PWRGD};

wire [15:0] reg_out;			// 上电初始化配置寄存器
wire ctrl_signal; // =1 CPLD内部初始化完成,=0 CPLD正在内部初始化

// 5.56MHz clk 计数
wire osc;

// for reset delay
always @(posedge osc or negedge SYS_PWRGD)	     // 500000 * 40ns=50Hz(20ms*2=40ms)	
begin
	if ( !SYS_PWRGD )
	begin
		counter	<= 0;
	end
	else if(counter < 6672000)
	begin
		counter	<= counter+1;
	end
	else
	begin
		counter	<= 6672000;
	end
end

reg [3:0] PWR_BTN_cnt;
always @(posedge osc or negedge ctrl_signal) begin
	if(!ctrl_signal)
		PWR_BTN_cnt <= 0;
	else if(PWR_BTN_cnt < 4'hf) begin
		if(SLP_S3n & SLP_S5n)
			PWR_BTN_cnt <= PWR_BTN_cnt + 4'h1;
		else
			PWR_BTN_cnt <= 0;
	end
	else
		PWR_BTN_cnt <= PWR_BTN_cnt;
end

//assign PWR_BTNn = ctrl_signal ? (reg_out[2] ? ((PWR_BTN_cnt == 4'hf) & RESET_INn) : RESET_INn) : 1'b1; // 开机按钮 需要按一下开机
//assign PWR_BTNn = 1'bz;
assign PWR_BTNn = RESET_INn;
// GROUP C
//assign DDR_SLP_S3n = ctrl_signal & SLP_S3n;
assign DDR_SLP_S5n = ctrl_signal & SLP_S5n;
assign EN_1V5 = DDR_SLP_S5n;
// GROUP D
assign VRS_ON = ctrl_signal & V1V5_PWRGD & SLP_S3n;
assign V1V8_EN = ctrl_signal & V1V5_PWRGD & SLP_S3n;
//assign V1_EN = ctrl_signal & DDR_PWROK & SLP_S3n;

// GROUP E
assign APU_VRM_EN = (V1V8_PWRGD & V1V_PWRGD & DIS_IN[12]) ? 1 : 0;
assign SYS_PWR_GOOD_ALL = ~APU_VDD_PWRGD;
// 由于APU_VDD_PWRGD引脚没做上拉 所以此处做一个延时 默认APU电OK
reg [3:0] apu_cnt;
//assign SYS_PWRGD = (V1V8_PWRGD & V1_PWRGD) & (apu_cnt==4'hf);
always @(posedge osc or negedge APU_VRM_EN) begin
	if(!APU_VRM_EN)
		apu_cnt <= 0;
	else if(apu_cnt < 4'hf)
		apu_cnt <= apu_cnt + 4'h1;
	else
		apu_cnt <= apu_cnt;
end

// 放电逻辑
assign DISCHARGE_S3n = V1V8_EN;
assign DISCHARGE_S5n = DDR_SLP_S5n;



wire lpc_en;
wire addr_hit;
wire [15:0] lpc_addr;
wire [7:0] lpc_din;
wire [7:0] lpc_dout;
wire lpc_io_wren;
wire lpc_io_rden;

wire [7:0] postcode;
wire com0_irq;

wire [7:0] LED_REG;				// LED指示灯寄存器
assign CPLD_LED = ctrl_signal ? 
					((reg_out[1:0]==2'b11) ? {SYS_RSTn,SATA_ACT,!SYS_PWRGD,!(V1V8_PWRGD),!V1V5_PWRGD,!SLP_S5n,!SLP_S3n,!PWR_BTNn} :
					((reg_out[1:0]==2'b10) ? ~postcode :
					((reg_out[1:0]==2'b01) ? 8'h55 :
					((reg_out[1:0]==2'b00) ? ~LED_REG : 8'h00)))) : 8'h00;

assign CPLD_IO[2] = osc; // 1'bz;
assign CPLD_IO[1] = LPC_CLK;
assign CPLD_IO[0] = int_serirq;

assign ECC_CHECK = 1'bz;


assign PLD_TX2 = PLD_TX1;
assign LPC_CLKRUNn = 1'bz;
assign LDRQn = 1'bz;
assign CPLD_SPKR = 1'bz;
assign PCIE_CLKREQn = 5'b0_0001;
assign S_TALERTn = 1;
assign THERMTRIPn = 1;

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
	.DPM_RSTn(1'b1),				// DPM模块输出给模块的复位信号
	.HPM_RSTn(1'b1),				// 主控模块输出给模块的复位信号
	.MGM_RSTn(DIS_IN[13]),				// 外部按钮手动复位信号
	.PSMRSTn(PSMRSTn),				// 电源模块输出给模块的复位信号
	.MGM_STS(DIS_OUT[8]),			// PCI-E状态指示信号
	.USER_RSTn(USER_RSTn),
	.RST_STS(RST_STS),

	.SYS_MAIN(SYS_MAIN),				// 意义待定 系统监控
	.GSE(GSE),					// 意义待定 地面/空中
	.SYS(SS),					// 意义待定 系统调试
	.SYS_ID(SYS_ID),			// 槽位识别
	.DIS_IN(DIS_IN),			// 离散量输入
	.L_R(L_R),				// 位置识别
	
	.PG0(PG0),				// Power Good Group 0
	.PG1(PG1),				// Power Good Group 1
	
	.ETH_STS({1'b0, LAN2_LED, 1'b0, LAN1_LED}),

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
