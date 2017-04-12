////////////////////////////////////////////////////////////////////
// FileName:  "amd-101a.v"
// Author  :  Roger.Yu
// Company :  www.icpc.cn
//////////////////////////////////////////////////////////////////// 

`include "uart_avalon_sopc/hdl/uart_defines.v"



// CPLD�����ַ index(0xa80)/data(0xa81)
`define LPC_INDEX_ADD   16'ha80
`define LPC_DATA_ADD    16'ha81
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

	output wire [7:0] CPLD_LED,	// ����ǰ���LED���� �͵�ƽ����
	output HD_DSTRY,			// Ӳ�������ź� �����͵�ƽ����Ӳ�� ƽʱ����̬���

	inout wire VCC_ALW_PWRGD,	// Ϊ�ߵ�ƽ��ʾ0.95v_alw��OK
	output wire PWR_BTNn,		// �����ź������CPU
	input SLP_S3n,				// CPU���ͳ���
	input SLP_S5n,				// CPU���ͳ���

	inout wire [2:0] CPLD_IO,	// ��fpc���Խӿڵ�IO
	output wire CPLD_SPKR,		// ���Ʒ����� �ߵ�ƽ��
	output wire DDR_SLP_S3n,	// ���ڿ���VDDIO_SUS,VTT_SUS
	output wire DDR_SLP_S5n,	// ���ڿ���VDDIO_SUS,VTT_SUS
	input DDR_PWROK,			// �ߵ�ƽ��ʾVDDIO_SUS,VTT_SUS��ѹ����
	output wire VRS_ON,			// �ߵ�ƽʱ��5v,3,3v,0.95v��ѹ
	output wire V1V8_EN,		// �ߵ�ƽʹ��+1.8v��ѹ
	input V1V8_PWRGD,			// �ߵ�ƽ��ʾ+1.8v��ѹ����
	output wire DISCHARGE_S3n,	// �͵�ƽʱ�ŵ�
	output wire DISCHARGE_S5n,	// �͵�ƽʱ�ŵ�
	output wire V1_EN,			// �ߵ�ƽʹ��pcie��1V0_PCIE
	input V1_PWRGD,				// �ߵ�ƽ��ʾpcie��1V0_PCIE��ѹ����
	input APU_VDD_PWRGD,		// �ߵ�ƽ��ʾAPU_VDD_RUN,APU_VDDNB_RUN��ѹ����
	output wire APU_VRM_EN,		// �ߵ�ƽʹ�ܵ�ѹ���
	output wire SYS_PWRGD,      // ���е�OK�� ���͵�CPU
	output wire ECC_CHECK,		// ECC_CHECK=1:enable ecc;=0 no ecc;
	output wire SYS_RSTn,		// ϵͳ��λ�ź�

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

	// lpc�ӿ�
	input wire LPC_CLK,			// LPC Clock 33MHz
	input LPC_RSTn,				// rst�ź�
	input wire LPC_FRAMEn,		// Frame - Active Low
	inout wire [ 3:0] LPC_AD,   // Address/Data Bus
	inout wire int_serirq,      // LPC�����ж� δʹ��
	output wire LPC_CLKRUNn,	// �͵�ƽ����CPU����LPCCLK�ź�
	output wire LDRQn,			// LPC DMA���� δʹ��

	// ����
	input PLD_RX1,				// ��XP3�ӿ�
	output wire PLD_TX1,		// ��XP3�ӿ�
	input PLD_RX2,				// ��FFC���Խӿ�
	output wire PLD_TX2,		// ��FFC���Խӿ�

	// CH7511 DP--->LVDS
	output wire LCDSEL,			// LCDSEL=0:��ͨ��,˫�����ƹ���; LCDSEL=1:˫ͨ��ģʽ;
	output wire RESET_CH7511Bn, // ��λCH7511оƬ
	output wire [3:0] CH7511_GPIO,	// ͨ����ͬ��GPIO����,���Զ�ȡboot rom����16��Һ���������ò���.

	input SPI_SCK,
	input clk_24mhz,			// 24MHz����
	input RESET_INn				// ǰ��尴ť

);

wire ctrl_signal;
assign VCC_ALW_PWRGD = ctrl_signal ? 1'bz : 0; // RSMRST#
assign PWR_BTNn = ctrl_signal ? RESET_INn : 1;
assign DDR_SLP_S3n = ctrl_signal & SLP_S3n;
assign DDR_SLP_S5n = ctrl_signal & SLP_S5n;



reg [15:0] firsttime_cnt;		// �Ը�λ��ʱ�Ĵ���
wire MYSELF_RSTn;				// �ڲ�ʹ�õĸ�λ�ź�
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
wire [15:0] reg_out;			// �ϵ��ʼ�����üĴ���
wire [7:0] LED_REG;				// LEDָʾ�ƼĴ���
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


// δʹ��
assign SHPC_INTn = 1'bz;
assign DP0_CAB_DP_HDMIn = 1; // dp0 dp
assign DP1_CAB_DP_HDMIn = 0; // dp1 dvi

assign CPLD_LED = ctrl_signal ? 
					((reg_out[1:0]==2'b11) ? {1'b1,!SYS_PWRGD,!(V1V8_PWRGD & V1_PWRGD),!DDR_PWROK,!SLP_S5n,!SLP_S3n,!VCC_ALW_PWRGD,!RESET_INn} :
					((reg_out[1:0]==2'b10) ? ~postcode :
					((reg_out[1:0]==2'b01) ? PEX_LANE_STATEn :
					((reg_out[1:0]==2'b00) ? LED_REG : 8'h00)))) : 8'h00;
assign PCIE_STRAP = 4'b0000; // upstream�˿�ѡ�� Ĭ��port 0,nt�˿ں�upstream�˿ڲ���һ��
assign NT_STRAP = 4'b0001;	// NTģʽ�˿�ѡ�� port 1
assign ECC_CHECK = ctrl_signal ? ~reg_out[8] : 1'b0; // =1 enable ecc; =0 no ecc;
assign NT_ENABLEn = ctrl_signal ? reg_out[9] : 1'b1; // �ߵ�ƽΪ��ͨģʽ,�͵�ƽΪNTģʽ
assign SSC_ISO_ENAn = ctrl_signal ? (reg_out[10] ? 1'bz : 1'b0) : 1'bz; // �ߵ�ƽ(����̬)Ϊͳһʱ��ģʽ,�͵�ƽΪ����ʱ��ģʽ
assign NT_P2P_ENn = ctrl_signal ? (reg_out[11] ? 1'bz : 1'b0) : 1'bz; // ����NTģʽ��PCI-TO-PCI���Ƿ�ʹ�ã�����ʹ��Legacy NT MODE,�ø���(�ⲿ������1)
assign UPCFG_TIMER_ENn = ctrl_signal ? (reg_out[12] ? 1'bz : 1'b0) : 1'bz;	// ���ø���(�ⲿ������1)ʱ�����ʿ�������ΪPCIE 2.0������֧������Ӧ����
																			// ���õ͵�ƽʱ�������һ������ʧ�ܣ���Ƭ����ΪPCIE 1.0�����Ҳ�֧������Ӧ����
assign LCDSEL = ctrl_signal ? reg_out[3] : 1'b1; // LCDSEL=0:��ͨ��,˫�����ƹ���; LCDSEL=1:˫ͨ��ģʽ;
//assign CH7511_GPIO = ctrl_signal ? reg_out[7:4] : 4'b0000; // ͨ����ͬ��GPIO����,���Զ�ȡboot rom����16��Һ���������ò���.
assign CH7511_GPIO = 4'b0001; // ͨ����ͬ��GPIO����,���Զ�ȡboot rom����16��Һ���������ò���.

assign CPLD_IO_DIR = 3'b000;
assign CPID_IO_OUT = 3'b111;
assign CPLD_IO[2] = CPLD_IO_DIR[2] ? CPID_IO_OUT[2] : 1'bz;
assign CPLD_IO[1] = CPLD_IO_DIR[1] ? CPID_IO_OUT[1] : 1'bz;
assign CPLD_IO[0] = CPLD_IO_DIR[0] ? CPID_IO_OUT[0] : 1'bz;

assign PLD_TX2 = PLD_TX1;

// 5.56MHz clk ����0x15b8��Լ��ʱ1s
assign MYSELF_RSTn = (firsttime_cnt == 16'h15b8);

always @(posedge osc) begin
	if(firsttime_cnt < 16'h15b8)
		firsttime_cnt <= firsttime_cnt + 16'h1;
	else
		firsttime_cnt <= firsttime_cnt;
end

// CPLD ��ַ����index/data ��ַƥ��
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

	.lpc_data_out(),				// ���ڲ���

	.lpc_en(lpc_en),				// �������ʹ���ź�,�ߵ�ƽʱ������Ч
	.addr_hit(addr_hit),			// ��ַƥ����1,��ƥ����0
	.lpc_addr(lpc_addr),			// LPC��ַ
	.din(lpc_din),					// LPC����ʱ�������������
	.lpc_data_in(lpc_dout),         // LPCд��ʱ�����˵��������
	.io_rden_sm(lpc_io_rden),       // ��ʹ���ź�
	.io_wren_sm(lpc_io_wren),       // дʹ���ź�
	.int_serirq(int_serirq),					// �����ж�
	.serirq({3'b000,com0_irq,4'b0000})							// �ж����� �ߵ�ƽ��Ч
);

// �Ĵ���ģ�� UFMģ�� ռ��0xa80/0xa81 index/data ģʽ����
LPC_Device1 LPC_Device1(
	.lclk(LPC_CLK), // Clock
	.lreset_n(LPC_RSTn), // Reset - Active Low (Same as PCI Reset)
	.lpc_en(lpc_en), // �������ʹ���ź�,�ߵ�ƽʱ������Ч
	.device_cs(addr_hit && (lpc_addr == `LPC_DATA_ADD)),
	.addr(index_add), // ��ַ
	.din(lpc_dout),
	.dout(lpc_din),
	.io_rden(lpc_io_rden),
	.io_wren(lpc_io_wren),

	.osc(osc),
	.ctrl_signal(ctrl_signal),	// =0,�ϵ��Զ�ȡ;=1,�ⲿ����
	.reg_out(reg_out) // �Ĵ������
);

/*
// 20141107 ռ��0x2f8��ַ��ufm
// �Ĵ���ģ�� UFMģ��
LPC_Device1 LPC_Device1(
  .lclk(LPC_CLK), // Clock
  .lreset_n(LPC_RSTn), // Reset - Active Low (Same as PCI Reset)
  .lpc_en(lpc_en), // �������ʹ���ź�,�ߵ�ƽʱ������Ч
  .device_cs(addr_hit && (lpc_addr >= `LPC_COM1_ADD) && (lpc_addr <= `LPC_COM1_ADD+7)),
  .addr(lpc_addr - `LPC_COM1_ADD), // ��ַ
  .din(lpc_dout),
  .dout(lpc_din),
  .io_rden(lpc_io_rden),
  .io_wren(lpc_io_wren),

  .osc(osc),
  .ctrl_signal(ctrl_signal), // =0,�ϵ��Զ�ȡ;=1,�ⲿ����
  .reg_out(reg_out)
);
*/













// postcode
LPC_Device0 LPC_Device0(
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

// �����ں� COM0
LPC_Device2 LPC_Device2(
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


endmodule

