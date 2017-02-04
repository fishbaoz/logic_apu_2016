////////////////////////////////////////////////////////////////////
// FileName:  "apu.v"
// Author  :  lisg
// Company :  careri
//////////////////////////////////////////////////////////////////// 
//`include "uart_avalon_sopc/hdl/uart_defines.v"

`define POST_CODE
//`define COM0_UART



// CPLD�����ַ index(0xa80)/data(0xa81)
`define LPC_INDEX_ADD   16'h4700
`define LPC_DATA_ADD    16'h4701
`define LPC_POST_ADD    16'h080
`define LPC_COM0_ADD    16'h3F8
`define LPC_COM1_ADD    16'h2F8

module apu 
(
	/**************************************************
	//PCI-E��״̬����ź�(MGM_STS):
	//�����������������źŵ����:�ߵ�ƽ��ʾģ���ϵ��ʼ����������
	//4���ⲿ��λ�ź�����һ�����뵽ģ���ģ�齫PCI-E��״̬����ź�
	//(MGM_STS)��Ϊ��Ч500ms��ģ�����Ӳ��λ��
	***************************************************/
	output wire [8:0] DIS_OUT,		// ��ɢ�����������DIS_OUT[8] = MGM_STS��PCI-E״ָ̬ʾ�ź�
	input wire [13:2] DIS_IN,		// ��ɢ������
	
	// XMC
	input wire XMC_MRSTIn,			// XMC����ĸ�λ�ź�
	output wire XMC_MRSTOn,			// XMC����ĸ�λ�ź�
	output wire XMC_PRESENTn,		// ����λ�źţ�������Ϊ0
	input wire XMC_MSDA,			// �ⲿI2C
	input wire XMC_MSCL,			// �ⲿI2C
	
	
	input wire INHMDG,			// ��ֹ���Ź��ź�
	input wire SYS_MAIN,			// ������� ϵͳ���
	input wire GSE,				// ������� ����/����
	input wire [1:0] SYS_ID,		// ��λʶ��
	input wire [1:0] SS,			// ����״̬�ź�
	input wire L_R,				// λ������ʶ��
	
	
	input wire PCIE_RSTn,			// CPU�����PCIE��λ�ź�
	input wire PSMRSTn,			// ��Դģ�������ģ��ĸ�λ�ź�
	input wire RESET_INn,			// ���Ը�λ��ť
	output wire SYS_RSTn,			// ϵͳ��λ�ź�
//	input wire ISP_IO3n,			// BIOS �ս�ڸ�λ
	output wire [2:0] RST_On,		// ��λ����ź�
	output wire ETH_RSTn,			// eth reset
	
	
	output wire PWR_BTNn,			// �����ź������CPU
	
	input wire clk_24mhz,			// 24MHz����
	
	output wire APU_VRM_EN,			// �ߵ�ƽʹ�ܵ�ѹ���
	input wire SYS_PWRGD,      		// ���е�OK�� ���͵�CPU
	input wire SLP_S3n,			// CPU���ͳ���
	input wire SLP_S5n,			// CPU���ͳ���
	output wire VRS_ON,			// �ߵ�ƽʱ��5v,3.3v,0.95v��ѹ
//	input wire V0V95_ALW_PWRGD,		// 0.95V alw POWER GOOD
	output wire DISCHARGE_S3n,		// �͵�ƽʱ�ŵ�
	output wire DISCHARGE_S5n,		// �͵�ƽʱ�ŵ�
	input wire V1V8_PWRGD,			// �ߵ�ƽ��ʾ+1.8v��ѹ����
	input wire V3V3_ALW_PWRGD,		// 3.3V POWER GOOD
	output wire V1V8_EN,			// �ߵ�ƽʱ��1.8v��ѹ
	input wire APU_VDD_PWRGD,		// �ߵ�ƽ��ʾAPU_VDD_RUN,APU_VDDNB_RUN��ѹ����
//	input wire DDR_PWROK,			// �ߵ�ƽ��ʾVDDIO_SUS,VTT_SUS��ѹ����
//	output wire APU_VDDNB_PWRGD,		// ������Ϊ1
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
	
	
	
	// lpc�ӿ�
	input wire LPC_CLK,			// LPC Clock 33MHz ���Ƕ�����
	input wire LPC_RSTn,			// rst�ź�
	input wire LPC_FRAMEn,			// Frame - Active Low
	inout wire [3:0] LPC_AD,    		// Address/Data Bus
	inout wire int_serirq,      		// LPC�����ж�
	output wire LPC_CLKRUNn,		// �͵�ƽ����CPU����LPCCLK�ź�
	output wire LDRQn,			// LPC DMA���� δʹ��
	
	input wire SD_LED,			// SD ��ָʾ�ƣ�û��ʹ��
	
	input wire [2:0] LAN1_LED,		// ��̫��ָʾ��
	input wire [2:0] LAN2_LED,		// ��̫��ָʾ��
	
	output wire PCIE_WAKE_UPn,		// PCIE WAKE UP
	input wire ETH1_WAKE_UPn,		// ETH1 PCIE WAKE UP
	input wire ETH2_WAKE_UPn,		// ETH2 PCIE WAKE UP
	
	input wire SATA_ACT,				// Ӳ�̶�дָʾ��
	
	output wire [7:0] CPLD_LED,		// ����ǰ���LED���� �͵�ƽ����
	output wire ECC_CHECK,			// ECC_CHECK=1:enable ecc;=0 no ecc;
	output wire CPLD_SPKR,			// ���Ʒ����� �ߵ�ƽ��
	output wire [2:0] CPLD_IO,	// ��fpc���Խӿڵ�IO
	input wire SPKR,			// CPU speaker
	input wire APU_ALERT,		// CPU alert ?
	input wire APU_FANTACH0,	// CPU FAN CONTROL
	input wire APU_FANOUT0,		// CPU FAN CONTROL
	
	// ����
	input wire PLD_RX1,			// ����1
	output wire PLD_TX1,			// ����1
	input wire PLD_RX2,			// ����2
	output wire PLD_TX2,			// ����2
	
	input SCLK,
	input SDATA,
	
	output wire SA0,			// spd eeprom ad0

	// cpu gpio
	output wire [4:0] PCIE_CLKREQn,
	output wire S_TALERTn,			// �¶ȱ���
	output wire DP0_CAB_DP_HDMIn,		// �ߵ�ƽΪDP�˿ڣ��͵�ƽΪDVI�˿� DP0--->LVDS
	output wire DP1_CAB_DP_HDMIn,		// �ߵ�ƽΪDP�˿ڣ��͵�ƽΪDVI�˿� DP1����ΪDVI
	output wire THERMTRIPn,
	input wire BLINKn,			// ����ָʾ��
	
	
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

wire [15:0] reg_out;			// �ϵ��ʼ�����üĴ���
wire ctrl_signal; // =1 CPLD�ڲ���ʼ�����,=0 CPLD�����ڲ���ʼ��

// 5.56MHz clk ����
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

//assign PWR_BTNn = ctrl_signal ? (reg_out[2] ? ((PWR_BTN_cnt == 4'hf) & RESET_INn) : RESET_INn) : 1'b1; // ������ť ��Ҫ��һ�¿���
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
// ����APU_VDD_PWRGD����û������ ���Դ˴���һ����ʱ Ĭ��APU��OK
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

// �ŵ��߼�
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

wire [7:0] LED_REG;				// LEDָʾ�ƼĴ���
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
// CPLD ��ַ����index/data ��ַƥ��
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

	.lpc_data_out(),				// ���ڲ���

	.lpc_en(lpc_en),				// �������ʹ���ź�,�ߵ�ƽʱ������Ч
	.addr_hit(addr_hit),			// ��ַƥ����1,��ƥ����0
	.lpc_addr(lpc_addr),			// LPC��ַ
	.din(lpc_din),					// LPC����ʱ�������������
	.lpc_data_in(lpc_dout),         // LPCд��ʱ�����˵��������
	.io_rden_sm(lpc_io_rden),       // ��ʹ���ź�
	.io_wren_sm(lpc_io_wren),       // дʹ���ź�
	.int_serirq(int_serirq),		// �����ж�����
	//.serirq({3'b000,com0_irq,4'b0000})							// �ж����� �ߵ�ƽ��Ч
	.serirq({3'b000,com0_irq,4'b000})
);

// �Ĵ���ģ�� UFMģ�� index/data ģʽ����
LPC_Device LPC_Device(
	.SLP_S5n(SLP_S5n),
	.lclk(LPC_CLK), // Clock
	.lreset_n(LPC_RSTn), // Reset - Active Low (Same as PCI Reset)
	.lpc_en(lpc_en), // �������ʹ���ź�,�ߵ�ƽʱ������Ч
	.device_cs(addr_hit && (lpc_addr == `LPC_DATA_ADD)),
	.addr(index_add), // ��ַ
	.din(lpc_dout),		
	.dout(lpc_din),		
	.io_rden(lpc_io_rden),
	.io_wren(lpc_io_wren),

	.INHMDG(INHMDG),					// =1���Ź���ֹ,=0���Ź�����
	.DPM_RSTn(1'b1),				// DPMģ�������ģ��ĸ�λ�ź�
	.HPM_RSTn(1'b1),				// ����ģ�������ģ��ĸ�λ�ź�
	.MGM_RSTn(DIS_IN[13]),				// �ⲿ��ť�ֶ���λ�ź�
	.PSMRSTn(PSMRSTn),				// ��Դģ�������ģ��ĸ�λ�ź�
	.MGM_STS(DIS_OUT[8]),			// PCI-E״ָ̬ʾ�ź�
	.USER_RSTn(USER_RSTn),
	.RST_STS(RST_STS),

	.SYS_MAIN(SYS_MAIN),				// ������� ϵͳ���
	.GSE(GSE),					// ������� ����/����
	.SYS(SS),					// ������� ϵͳ����
	.SYS_ID(SYS_ID),			// ��λʶ��
	.DIS_IN(DIS_IN),			// ��ɢ������
	.L_R(L_R),				// λ��ʶ��
	
	.PG0(PG0),				// Power Good Group 0
	.PG1(PG1),				// Power Good Group 1
	
	.ETH_STS({1'b0, LAN2_LED, 1'b0, LAN1_LED}),

	.LED_REG(LED_REG),
	.osc(osc),
	.ctrl_signal(ctrl_signal),	// =0,�ϵ��Զ�ȡ;=1,�ⲿ����
	.lpc_reg(lpc_reg),
	.reg_out(reg_out) // �Ĵ������
);

`ifdef POST_CODE
	// postcode
	LPC_POSTCODE LPC_POSTCODE(
	  .lclk(LPC_CLK), // Clock
	  .lreset_n(LPC_RSTn), // Reset - Active Low (Same as PCI Reset)
	  .lpc_en(lpc_en), // �������ʹ���ź�,�ߵ�ƽʱ������Ч
	  .device_cs(addr_hit & (lpc_addr == `LPC_POST_ADD)),
	  .addr(index_add), // ��ַ
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
	// �����ں� COM0
	LPC_COM LPC_COM0(
		.lclk(LPC_CLK),					// Clock 33MHz
		.lreset_n(LPC_RSTn),				// Reset - Active Low (Same as PCI Reset)
		.lpc_en(lpc_en),				// �������ʹ���ź�,�ߵ�ƽʱ������Ч
		.device_cs(addr_hit && (lpc_addr >= `LPC_COM0_ADD) && (lpc_addr <= `LPC_COM0_ADD+7)),
		.addr(lpc_addr - `LPC_COM0_ADD),			// ��ַ
		.din(lpc_dout),
		.dout(lpc_din),
		.io_rden(lpc_io_rden),
		.io_wren(lpc_io_wren),
		.com_irq(com0_irq),

		.clk_24mhz(clk_24mhz),			// 24MHzʱ������
		.tx(PLD_TX1),
		.rx(PLD_RX1),
		.baud_clk()			// ������ʱ�����
	);
`else
	assign PLD_TX1 = 1;
	assign com0_irq = 0;
`endif

endmodule
