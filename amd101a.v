////////////////////////////////////////////////////////////////////
// FileName:  "amd-101a.v"
// Author  :  Roger.Yu
// Company :  www.icpc.cn
//////////////////////////////////////////////////////////////////// 
//`include "uart_avalon_sopc/hdl/uart_defines.v"

`define POST_CODE
`define COM0_UART



// CPLD�����ַ index(0xa80)/data(0xa81)
`define LPC_INDEX_ADD   16'h4700
`define LPC_DATA_ADD    16'h4701
`define LPC_POST_ADD    16'h080
`define LPC_COM0_ADD    16'h3F8
`define LPC_COM1_ADD    16'h2F8

module amd101a 
(

	/**************************************************
	//PCI-E��״̬����ź�(MGM_STS):
	//�����������������źŵ����:�ߵ�ƽ��ʾģ���ϵ��ʼ����������
	//4���ⲿ��λ�ź�����һ�����뵽ģ���ģ�齫PCI-E��״̬����ź�
	//(MGM_STS)��Ϊ��Ч500ms��ģ�����Ӳ��λ��
	***************************************************/
	output wire MGM_STS,		// PCI-E״ָ̬ʾ�ź�
	input INHMDG,				// ��ֹ���Ź��ź�
	input DPM_RSTn,				// DPMģ�������ģ��ĸ�λ�ź�
	input HPM_RSTn,				// ����ģ�������ģ��ĸ�λ�ź�
	input MGM_RSTn,				// �ⲿ��ť�ֶ���λ�ź�
	input PSMRSTn,				// ��Դģ�������ģ��ĸ�λ�ź�
	
	input SYS_MAIN,				// ������� ϵͳ���
	input GSE,					// ������� ����/����
	input SYS,					// ������� ϵͳ����
	input [1:0] SYS_ID,			// ��λʶ��

	output HD_DSTRY,			// Ӳ�������ź� �����͵�ƽ����Ӳ�� ƽʱ����̬���

	// �ϵ��߼��ź�
	inout wire VCC_ALW_PWRGD,	// Ϊ�ߵ�ƽ��ʾ0.95v_alw��OK
	output wire PWR_BTNn,		// �����ź������CPU
	input SLP_S3n,				// CPU���ͳ���
	input SLP_S5n,				// CPU���ͳ���
	output wire DDR_SLP_S3n,	// ���ڿ���VDDIO_SUS,VTT_SUS
	output wire DDR_SLP_S5n,	// ���ڿ���VDDIO_SUS,VTT_SUS
	input DDR_PWROK,			// �ߵ�ƽ��ʾVDDIO_SUS,VTT_SUS��ѹ����
	output wire VRS_ON,			// �ߵ�ƽʱ��5v,3.3v,0.95v��ѹ
	output wire V1V8_EN,		// �ߵ�ƽʱ��1.8v��ѹ
	output wire V1_EN,			// �ߵ�ƽʹ��pcie��1V0_PCIE
	input V1V8_PWRGD,			// �ߵ�ƽ��ʾ+1.8v��ѹ����
	input V1_PWRGD,				// �ߵ�ƽ��ʾpcie��1V0_PCIE��ѹ����
	output wire APU_VRM_EN,		// �ߵ�ƽʹ�ܵ�ѹ���
	input APU_VDD_PWRGD,		// �ߵ�ƽ��ʾAPU_VDD_RUN,APU_VDDNB_RUN��ѹ����
	output wire SYS_PWRGD,      // ���е�OK�� ���͵�CPU
	output wire DISCHARGE_S3n,	// �͵�ƽʱ�ŵ�
	output wire DISCHARGE_S5n,	// �͵�ƽʱ�ŵ�

	output wire SYS_RSTn,		// ϵͳ��λ�ź�

	// lpc�ӿ�
	input wire LPC_CLK,			// LPC Clock 33MHz
	input LPC_RSTn,				// rst�ź�
	input wire LPC_FRAMEn,		// Frame - Active Low
	inout wire [3:0] LPC_AD,    // Address/Data Bus
	inout wire int_serirq,      // LPC�����ж�
	output wire LPC_CLKRUNn,	// �͵�ƽ����CPU����LPCCLK�ź�
	output wire LDRQn,			// LPC DMA���� δʹ��

	// CH7511 DP--->LVDS
	output wire LCDSEL,			// LCDSEL=0:��ͨ��,˫�����ƹ���; LCDSEL=1:˫ͨ��ģʽ;
	output wire RESET_CH7511Bn, // ��λCH7511оƬ
	output wire [3:0] CH7511_GPIO,	// ͨ����ͬ��GPIO����,���Զ�ȡboot rom����16��Һ���������ò���.

	output wire [7:0] CPLD_LED,	// ����ǰ���LED���� �͵�ƽ����
	input SATA_ACT,				// Ӳ�̶�дָʾ��
	output wire [2:0] CPLD_IO,	// ��fpc���Խӿڵ�IO
	output wire CPLD_SPKR,		// ���Ʒ����� �ߵ�ƽ��
	output wire ECC_CHECK,		// ECC_CHECK=1:enable ecc;=0 no ecc;

	// cpu gpio
	output wire [4:0] PCIE_CLKREQn,
	output wire S_TALERTn,		// �¶ȱ���
	output wire THERMTRIPn,
	input BLINKn,				// ����ָʾ��
	output DP0_CAB_DP_HDMIn,	// �ߵ�ƽΪDP�˿ڣ��͵�ƽΪDVI�˿� DP0--->LVDS
	output DP1_CAB_DP_HDMIn,	// �ߵ�ƽΪDP�˿ڣ��͵�ƽΪDVI�˿� DP1����ΪDVI

	// pex8609
	output wire [3:0] PCIE_STRAP,	// upstream�˿�ѡ�� Ĭ��port 0,nt�˿ں�upstream�˿ڲ���һ��
	output wire NT_ENABLEn,		// �ߵ�ƽΪ��ͨģʽ,�͵�ƽΪNTģʽ
	output wire [3:0] NT_STRAP,	// NTģʽʱ�˿�ѡ�� Ӧ����port 1, nt�˿ں�upstream�˿ڲ���һ��
	output wire SSC_ISO_ENAn,	// �ߵ�ƽ(����̬)Ϊͳһʱ��ģʽ,�͵�ƽΪ����ʱ��ģʽ
	input PEX_NT_RESETn,		// δʹ��
	input [7:0] PEX_LANE_STATEn,		// PEX LANE״̬ �ߵ�ƽΪ�رգ��͵�ƽpcie 2.0����������Ϊpcie 1.0 δʹ��
	input PEX_INTAn,			// PEX�ŷ������ж� δʹ��
	input FATAL_ERRn,			// PEX�ŷ��������������ź� δʹ��
	output wire SHPC_INTn,		// �Ȱβ��ź� ��ͨ���ָߵ�ƽ δʹ��
	output wire NT_P2P_ENn,		// ����NTģʽ��PCI-TO-PCI���Ƿ�ʹ�ã�����ʹ��Legacy NT MODE,�ø���(�ⲿ������1)
	output wire UPCFG_TIMER_ENn,	// ���ø���(�ⲿ������1)ʱ�����ʿ�������ΪPCIE 2.0������֧������Ӧ����
									// ���õ͵�ƽʱ�������һ������ʧ�ܣ���Ƭ����ΪPCIE 1.0�����Ҳ�֧������Ӧ����

	// ����
	input PLD_RX1,				// ��XP3�ӿ�
	output wire PLD_TX1,		// ��XP3�ӿ�
	input PLD_RX2,				// ��FFC���Խӿ�
	output wire PLD_TX2,		// ��FFC���Խӿ�

	input SPI_SCK,
	input SCLK,
	input SDATA,
	input clk_24mhz,			// 24MHz����
	input RESET_INn				// ǰ��尴ť

);

wire [15:0] reg_out;			// �ϵ��ʼ�����üĴ���
wire ctrl_signal; // =1 CPLD�ڲ���ʼ�����,=0 CPLD�����ڲ���ʼ��
assign VCC_ALW_PWRGD = ctrl_signal ? 1'bz : 0; // RSMRST#

// 5.56MHz clk ����
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

assign PWR_BTNn = ctrl_signal ? (reg_out[2] ? ((PWR_BTN_cnt == 4'hf) & RESET_INn) : RESET_INn) : 1; // ������ť ��Ҫ��һ�¿���

// GROUP C
assign DDR_SLP_S3n = ctrl_signal & SLP_S3n;
assign DDR_SLP_S5n = ctrl_signal & SLP_S5n;

// GROUP D
assign VRS_ON = ctrl_signal & DDR_PWROK & SLP_S3n;
assign V1V8_EN = ctrl_signal & DDR_PWROK & SLP_S3n;
assign V1_EN = ctrl_signal & DDR_PWROK & SLP_S3n;

// GROUP E
assign APU_VRM_EN = (V1V8_PWRGD & V1_PWRGD) ? 1 : 0;

// ����APU_VDD_PWRGD����û������ ���Դ˴���һ����ʱ Ĭ��APU��OK
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

// �ŵ��߼�
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
assign LCDSEL = ctrl_signal ? reg_out[3] : 1'b1; // LCDSEL=0:��ͨ��,˫�����ƹ���; LCDSEL=1:˫ͨ��ģʽ;
assign CH7511_GPIO = ctrl_signal ? reg_out[7:4] : 4'b0000; // ͨ����ͬ��GPIO����,���Զ�ȡboot rom����16��Һ���������ò���.

wire [7:0] LED_REG;				// LEDָʾ�ƼĴ���
assign CPLD_LED = ctrl_signal ? 
					((reg_out[1:0]==2'b11) ? {SYS_RSTn,SATA_ACT,!SYS_PWRGD,!(V1V8_PWRGD & V1_PWRGD),!DDR_PWROK,!SLP_S5n,!SLP_S3n,!PWR_BTNn} :
					((reg_out[1:0]==2'b10) ? ~postcode :
					((reg_out[1:0]==2'b01) ? PEX_LANE_STATEn :
					((reg_out[1:0]==2'b00) ? ~LED_REG : 8'h00)))) : 8'h00;

assign CPLD_IO[2] = 1'bz;
assign CPLD_IO[1] = LPC_CLK;
assign CPLD_IO[0] = int_serirq;

assign ECC_CHECK = 1'bz;
assign PCIE_STRAP = 4'b0000; // upstream�˿�ѡ�� Ĭ��port 0,nt�˿ں�upstream�˿ڲ���һ��
assign NT_STRAP = 4'b0001;	// NTģʽ�˿�ѡ�� port 1
assign NT_ENABLEn = ctrl_signal ? reg_out[9] : 1'b1; // �ߵ�ƽΪ��ͨģʽ,�͵�ƽΪNTģʽ
assign SSC_ISO_ENAn = ctrl_signal ? (reg_out[10] ? 1'bz : 1'b0) : 1'bz; // �ߵ�ƽ(����̬)Ϊͳһʱ��ģʽ,�͵�ƽΪ����ʱ��ģʽ
assign NT_P2P_ENn = ctrl_signal ? (reg_out[11] ? 1'bz : 1'b0) : 1'bz; // ����NTģʽ��PCI-TO-PCI���Ƿ�ʹ�ã�����ʹ��Legacy NT MODE,�ø���(�ⲿ������1)
assign UPCFG_TIMER_ENn = ctrl_signal ? (reg_out[12] ? 1'bz : 1'b0) : 1'bz;	// ���ø���(�ⲿ������1)ʱ�����ʿ�������ΪPCIE 2.0������֧������Ӧ����
																			// ���õ͵�ƽʱ�������һ������ʧ�ܣ���Ƭ����ΪPCIE 1.0�����Ҳ�֧������Ӧ����
assign PLD_TX2 = PLD_TX1;
assign LPC_CLKRUNn = 1'bz;
assign LDRQn = 1'bz;
assign CPLD_SPKR = 1'bz;
assign PCIE_CLKREQn = 5'b0_0001;
assign S_TALERTn = 1;
assign THERMTRIPn = 1;
assign SHPC_INTn = 1'bz; // δʹ��
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
	.DPM_RSTn(DPM_RSTn),				// DPMģ�������ģ��ĸ�λ�ź�
	.HPM_RSTn(HPM_RSTn),				// ����ģ�������ģ��ĸ�λ�ź�
	.MGM_RSTn(MGM_RSTn),				// �ⲿ��ť�ֶ���λ�ź�
	.PSMRSTn(PSMRSTn),				// ��Դģ�������ģ��ĸ�λ�ź�
	.MGM_STS(MGM_STS),			// PCI-E״ָ̬ʾ�ź�
	.USER_RSTn(USER_RSTn),

	.SYS_MAIN(SYS_MAIN),				// ������� ϵͳ���
	.GSE(GSE),					// ������� ����/����
	.SYS(SYS),					// ������� ϵͳ����
	.SYS_ID(SYS_ID),			// ��λʶ��

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

