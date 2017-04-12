//-----------------------------------------------------------------------
// FileName:  "LPC_Device1.v"
// Author  :  Roger.Yu
// Company :  www.icpc.cn
// use max ii ufm, can read and write.
// in begin auto read address 0x01 data send out.
// --------------------------------------------------------------------
`define CPLD_VERSION    8'h02		// CPLD程序版本号
//`define USE_UFM						// 使用UFM功能

// LPC index/data 寄存器
// 0x0c,0x0d 板卡配置寄存器
// bit		name				说明
// 15:14	NC					NC
// 13		NC					NC
// 12		UPCFG_TIMER_ENn		PEX8609桥PCIE速率配置
// 11		NT_P2P_ENn			PEX8609桥内部桥启用否
// 10		SCCISO_ENAn			1:桥使用内部时钟;0:桥使用外部时钟
// 9		NT_ENA				1:桥为普通模式;0:NT模式
// 8		ECC_ENA				NC
// 7:4		CH7511_GPIO			液晶屏参数选择
// 3		LCDSEL				1:双通道模式;0:单通道,双屏复制功能
// 2		AT_ON				1:AT模式,上电就开始启动;
//								0:ATX模式,上电后需要按power button按钮开机
// 1:0		LEC_CTL				控制LED指示灯显示模式：
//								11:LED指示灯显示上电过程
//								10:LED指示灯显示POSTCODE
//								01:LED指示灯显示PEX8609 PCIE速率状态
//								00:LED指示灯显示LED寄存器
`define REG_OUT_DEFAULT 16'hfd26  // LED显示POSTCODE,选择配置2,单通道屏
//`define REG_OUT_DEFAULT 16'hfd27	// LED显示上电过程,选择配置2,单通道屏
//`define REG_OUT_DEFAULT 16'hfdfe  // LED显示POSTCODE,选择配置16,双通道屏
//`define REG_OUT_DEFAULT 16'hfdff  // LED显示上电过程,选择配置16,双通道屏

module LPC_Device
(
	input SLP_S5n,
	input lclk,				// Clock
	input lreset_n,			// Reset - Active Low (Same as PCI Reset)
	input lpc_en,			// 后端总线使能信号,高电平时总线有效
	input device_cs,
	input [7:0] addr,		// 地址
	input [7:0] din,
	output reg [7:0] dout,
	input io_rden,
	input io_wren,

	input INHMDG,			// 看门狗禁止
	input DPM_RSTn,			// DPM模块输出给模块的复位信号
	input HPM_RSTn,			// 主控模块输出给模块的复位信号
	input MGM_RSTn,			// 外部按钮手动复位信号
	input PSMRSTn,			// 电源模块输出给模块的复位信号
	output wire MGM_STS,	// PCI-E状态指示信号
	output reg USER_RSTn,	// 用户复位信号
	output wire RST_STS,	// 复位信号
	input SYS_MAIN,				// 意义待定 系统监控
	input GSE,					// 意义待定 地面/空中
	input [1:0] SYS,					// 意义待定 系统调试
	input [1:0] SYS_ID,			// 槽位识别
	input [13:2] DIS_IN,			// 离散量输入
	input L_R,			// 位置识别
	
	input [7:0] PG0,		// Power Good Group 0
	input [7:0] PG1,		// Power Good Group 1
	
	input [7:0] ETH_STS,		// ethernet lan status

	output reg [7:0] LED_REG,	// 测试寄存器用于点亮前面板LED灯
	output wire osc,
	output reg [15:0] reg_out, // 寄存器输出
	output reg [7:0] lpc_reg,	// LPC 配置寄存器
	`ifdef USE_UFM
		output reg ctrl_signal	// =0,上电自读取;=1,外部控制
	`else
		output wire ctrl_signal	// =0,上电自读取;=1,外部控制
	`endif
);

// 全局使用的计时器，用于各种延时
// 一个计时满周期大约184us
reg [9:0] time_kick;
always @(posedge osc) begin
	time_kick <= time_kick + 10'h1;
end

// 为了保证上电稳定 整个系统上电后延时1s才开始工作
reg [12:0] firsttime_cnt;
wire MYSELF_RSTn;
assign MYSELF_RSTn = (firsttime_cnt == 13'h153b);
//assign MYSELF_RSTn = 1; // 为了节省逻辑资源取消此计数器
always @(posedge osc) begin
	if(time_kick == 10'h3ff) begin
		if(firsttime_cnt < 13'h153b)
			firsttime_cnt <= firsttime_cnt + 1;
		else
			firsttime_cnt <= firsttime_cnt;
	end
	else
		firsttime_cnt <= firsttime_cnt;
end


	// 调用MAX2器件内部的5.56MHz晶振
	maxII_osc maxII_osc(
		.oscena(1'b1),
		.osc(osc)
	);

	assign ctrl_signal = MYSELF_RSTn;


// 同步外部信号
reg DPM_RSTn_syn,HPM_RSTn_syn,MGM_RSTn_syn,PSMRSTn_syn;
always @ (posedge osc or negedge MYSELF_RSTn) begin
	if(!MYSELF_RSTn) begin
		DPM_RSTn_syn <= 1;
		HPM_RSTn_syn <= 1;
		MGM_RSTn_syn <= 1;
		PSMRSTn_syn <= 1;
	end
	else begin
		DPM_RSTn_syn <= DPM_RSTn;
		HPM_RSTn_syn <= HPM_RSTn;
		MGM_RSTn_syn <= MGM_RSTn;
		PSMRSTn_syn <= PSMRSTn;
	end
end

// 用于延迟复位
reg [11:0] ms_cnt;
reg ena_ms;
always @(posedge osc or negedge MYSELF_RSTn) begin
	if(!MYSELF_RSTn)
		ms_cnt <= 0;
	else if(ena_ms) begin
		if(time_kick == 10'h3ff) begin
			if(ms_cnt < 12'hfff)
				ms_cnt <= ms_cnt + 1;
			else
				ms_cnt <= ms_cnt;
		end
		else
			ms_cnt <= ms_cnt;
	end
	else
		ms_cnt <= 0;
end

// 复位逻辑状态机
reg [3:0] rst_state;
wire watchdog_over;
always @ (posedge osc or negedge MYSELF_RSTn) begin
	if(!MYSELF_RSTn) begin
		rst_state <= 0;
		USER_RSTn <= 1;
		ena_ms <= 0;
	end
	else begin
		case (rst_state)
			0: begin
				USER_RSTn <= 1;
				ena_ms <= 0;
				if(!(DPM_RSTn_syn & HPM_RSTn_syn & MGM_RSTn_syn & PSMRSTn_syn & watchdog_over)) // 任意一个信号为低
					rst_state <= 1;
				else
					rst_state <= rst_state;
			end
			1: begin // 延时0.5s发出复位信号
				USER_RSTn <= 1;
				if( ms_cnt >= 12'ha9d ) begin
					rst_state <= 2;
					ena_ms <= 0;
				end
				else begin
					rst_state <= rst_state;
					ena_ms <= 1;
				end
			end
			2: begin // 等待计数器清零
				rst_state <= 3;
			end
			3: begin // 复位信号保持0.1s
				if( ms_cnt == 12'h221 ) begin
					rst_state <= 0;
					USER_RSTn <= 1;
					ena_ms <= 0;
				end
				else begin
					rst_state <= rst_state;
					USER_RSTn <= 0;
					ena_ms <= 1;
				end
			end
			default: begin
				rst_state <= 0;
				USER_RSTn <= 1;
				ena_ms <= 0;
			end
		endcase
	end
end

// PCI-E的状态输出信号(MGM_STS):
// 由驱动程序控制这个信号的输出:
// 高电平表示模块上电初始化后工作正常
reg MGM_STS_reg;
assign MGM_STS = DPM_RSTn_syn & HPM_RSTn_syn & MGM_RSTn_syn & PSMRSTn_syn & MGM_STS_reg & watchdog_over;
assign RST_STS = DPM_RSTn_syn & HPM_RSTn_syn & MGM_RSTn_syn & PSMRSTn_syn & watchdog_over;
`ifdef USE_UFM
	// 用33MHz计时,UFM的读写信号脉宽在30~100计数之间
	// 读写信号有效后,至少15个周期后才能去判断nbusy和data_valid的状态
	reg [7:0] rw_pluse_cnt;
	reg read_reg;
	reg write_reg;
	wire nbusy_reg;
	assign nread = !(read_reg & (rw_pluse_cnt<70));
	assign nwrite = !(write_reg & (rw_pluse_cnt<70));
	assign nbusy_reg = nbusy & nread & nwrite;

	always @ (posedge lclk or negedge lreset_n) begin
		if(!lreset_n)
			rw_pluse_cnt <= 0;
		else if(read_reg || write_reg) begin
			if(rw_pluse_cnt < 8'hff)
				rw_pluse_cnt <= rw_pluse_cnt + 1;
			else
				rw_pluse_cnt <= rw_pluse_cnt;
		end
		else
			rw_pluse_cnt <= 0;
	end
`else
	reg read_reg;
	reg write_reg;
	wire nbusy_reg;
	assign nbusy_reg = 1;
`endif

// LPC index/data 寄存器
// 地址 读写	寄存器名
// 0x00 R		CPLD程序版本号
// 0x01 R/W		led寄存器
// 0x02 R/W		UFM控制寄存器
// 0x03 R/W		UFM地址寄存器
// 0x04 R/W		UFM数据寄存器-高8位
// 0x05 R/W		UFM数据寄存器-低8位
// 0x06 R/W		用户寄存器
// 0x07 R/W     看门狗使能和清除寄存器
// 0x08 R/W     看门狗定时器[15:8]
// 0x09 R/W     看门狗定时器[7:0]
// 0x0a R	    看门狗计数寄存器[15:8]
// 0x0b R		看门狗计数寄存器[7:0]
// 0x0c R/W     板卡配置寄存器[15:8]
// 0x0d R/W     板卡配置寄存器[7:0]
// 0x0e R/W     LPC寄存器[7:0]
// 0x10 R	离散量输入[0，0，DIS_IN[7:2] ]
// 0x11 R	离散量输入[DIS_IN[13:8]，0，0]
// 0x12 R	位置识别
// 0x13 R	power good 0
// 0x14 R	power good 1
// 0x15 R	ETH_STS

// 0x06 用户寄存器
// bit		读写	name				说明
// 7		R		NC					始终为0
// 6		R		INHMDG				看门狗禁止引脚
// 5:4		R		SYS_ID				槽位识别ID号
// 3		R		SYS					系统调试引脚状态
// 2		R		GSE					地面/空中 高电平不能进入BIOS SETUP菜单
// 1		R		SYS_MAIN			系统监控引脚状态
// 0		R/W		MGM_STS				PCI-E的状态输出信号 高电平表示模块上电初始化后工作正常

wire [7:0] reg_version;
reg watchdog_ena;
reg watchdog_clean;
reg [9:0] watchdog_cnt_kick;
reg [15:0] watchdog_cnt,watchdog_reg,watchdog_reg_syn;
assign watchdog_over = !(watchdog_cnt==watchdog_reg_syn);
assign reg_version = `CPLD_VERSION;

// 看门狗计数器
always @ (posedge osc or posedge watchdog_clean) begin
	watchdog_reg_syn <= watchdog_reg;
	if(watchdog_clean)
		watchdog_cnt <= 0;
	else if(watchdog_ena & (!INHMDG)) begin
		if(watchdog_cnt < watchdog_reg_syn) begin
			if(time_kick == 10'h3ff)
				watchdog_cnt <= watchdog_cnt + 1;
			else
				watchdog_cnt <= watchdog_cnt;
		end
		else
			watchdog_cnt <= watchdog_cnt;
	end
	else
		watchdog_cnt <= 0;
end

// lpc 寄存器操作
always @ (posedge lclk or negedge SLP_S5n) begin
	if(!SLP_S5n) begin
		dout <= 8'hzz;
		LED_REG <= 8'h5a;  // s5才会复位
		lpc_reg <= 8'h00;  // s5才会复位
		MGM_STS_reg <= 1;
		watchdog_ena <= 0;
		watchdog_clean <= 0;
		watchdog_reg <= 16'hffff;
		`ifdef USE_UFM
			read_reg <= 0;
			write_reg <= 0;
			datain <= 0;
		`else
			reg_out <= `REG_OUT_DEFAULT;  // s5才会复位
		`endif
	end
	else if(!lreset_n) begin   // s3复位
		dout <= 8'hzz;
		MGM_STS_reg <= 1;
		watchdog_ena <= 0;
		watchdog_clean <= 0;
		watchdog_reg <= 16'hffff;

		LED_REG <= LED_REG;
		lpc_reg <= lpc_reg;
		`ifdef USE_UFM
			read_reg <= 0;
			write_reg <= 0;
			datain <= 0;
		`else
			reg_out <= reg_out;
		`endif
	end
	else if(device_cs & io_rden & lpc_en) begin // LPC 寄存器读
		if(addr==0)
			dout <= reg_version;
		else if(addr==1)
			dout <= LED_REG;
		`ifdef USE_UFM
			else if(addr==2)
				dout <= {5'b0_0000,ctrl_signal,(data_valid & nbusy_reg),nbusy_reg};
			else if(addr==3)
				dout <= addr_ufm;
			else if(addr==4)
				dout <= dataout[15:8];
			else if(addr==5)
				dout <= dataout[7:0];
		`else
			else if(addr==12)
				dout <= reg_out[15:8];
			else if(addr==13)
				dout <= reg_out[7:0];
		`endif
		else if(addr==6)
			dout <= {INHMDG,SYS_ID,SYS,GSE,SYS_MAIN,MGM_STS_reg};
		else if(addr==7)
			dout <= {7'h0,watchdog_ena};
		else if(addr==8)
			dout <= watchdog_reg[15:8];
		else if(addr==9)
			dout <= watchdog_reg[7:0];
		else if(addr==10)
			dout <= watchdog_cnt[15:8];
		else if(addr==11)
			dout <= watchdog_cnt[7:0];
		else if(addr==14)
			dout <= {7'h0,lpc_reg[0]};
		else if(addr==15)
			dout <= {7'h0,lpc_reg[1]};
		else if(addr==8'h10)
			dout <= {DIS_IN[7:2], 2'b0};
		else if(addr==8'h11)
			dout <= {2'b0, DIS_IN[13:8]};
		else if(addr==8'h12)
			dout <= {7'b0, L_R};
		else if(addr==8'h13)
			dout <= PG0;
		else if(addr==8'h14)
			dout <= PG1;
		else if(addr==8'h15)
			dout <= ETH_STS;
		else
			dout <= 8'hff;
	end
	else if(device_cs & io_wren & lpc_en) begin // LPC寄存器写
		if(addr==1)
			LED_REG <= din;
		`ifdef USE_UFM
			else if(addr==2) begin // UFM模块操作
				if(din == 8'h55) begin // 读操作
					read_reg <= 1;
					write_reg <= 0;
				end
				else if(din == 8'haa) begin // 写操作
					read_reg <= 0;
					write_reg <= 1;
				end
				else if(din == 8'h00) begin // 撤销读写操作
					read_reg <= 0;
					write_reg <= 0;
				end
			end
			else if(addr==3)
				addr_ufm <= din;
			else if(addr==4)
				datain[15:8] <= din;
			else if(addr==5)
				datain[7:0] <= din;
		`else			
			else if(addr==12)
				reg_out[15:8] <= din;
			else if(addr==13)
				reg_out[7:0] <= din;
		`endif
		else if(addr==6)
			MGM_STS_reg <= din[0];
		else if(addr==7) begin
			watchdog_clean <= din[7];
			watchdog_ena <= din[0];
		end
		else if(addr==8)
			watchdog_reg[15:8] <= din;
		else if(addr==9)
			watchdog_reg[7:0] <= {din[7:1],1'b1};
		else if(addr==14)
			lpc_reg[0] <= din[0];
		else if(addr==15)
			lpc_reg[1] <= din[0];
	end
	else begin
		dout <= 8'hzz;
		MGM_STS_reg <= MGM_STS_reg;
		watchdog_clean <= 0;
		watchdog_ena <= watchdog_ena;
		watchdog_reg <= watchdog_reg;
		LED_REG <= LED_REG;
		lpc_reg <= lpc_reg;
		`ifdef USE_UFM
			read_reg <= read_reg;
			write_reg <= write_reg;
			datain <= datain;
		`else
			reg_out <= reg_out;
		`endif
	end
end

endmodule
