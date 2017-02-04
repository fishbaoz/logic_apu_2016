// --------------------------------------------------------------------
// FileName:  "LPC_Device1.v"
// Author  :  Roger.Yu
// Company :  www.icpc.cn
// use max ii ufm, can read and write.
// in begin auto read address 0x01 data send out.
// --------------------------------------------------------------------
`define CPLD_VERSION    8'h02		// CPLD����汾��
//`define USE_UFM						// ʹ��UFM����

// LPC index/data �Ĵ���
// 0x0c,0x0d �忨���üĴ���
// bit		name				˵��
// 15:14	NC					NC
// 13		NC					NC
// 12		UPCFG_TIMER_ENn		PEX8609��PCIE��������
// 11		NT_P2P_ENn			PEX8609���ڲ������÷�
// 10		SCCISO_ENAn			1:��ʹ���ڲ�ʱ��;0:��ʹ���ⲿʱ��
// 9		NT_ENA				1:��Ϊ��ͨģʽ;0:NTģʽ
// 8		ECC_ENA				NC
// 7:4		CH7511_GPIO			Һ��������ѡ��
// 3		LCDSEL				1:˫ͨ��ģʽ;0:��ͨ��,˫�����ƹ���
// 2		AT_ON				1:ATģʽ,�ϵ�Ϳ�ʼ����;
//								0:ATXģʽ,�ϵ����Ҫ��power button��ť����
// 1:0		LEC_CTL				����LEDָʾ����ʾģʽ��
//								11:LEDָʾ����ʾ�ϵ����
//								10:LEDָʾ����ʾPOSTCODE
//								01:LEDָʾ����ʾPEX8609 PCIE����״̬
//								00:LEDָʾ����ʾLED�Ĵ���
`define REG_OUT_DEFAULT 16'hfd26  // LED��ʾPOSTCODE,ѡ������2,��ͨ����
//`define REG_OUT_DEFAULT 16'hfd27	// LED��ʾ�ϵ����,ѡ������2,��ͨ����
//`define REG_OUT_DEFAULT 16'hfdfe  // LED��ʾPOSTCODE,ѡ������16,˫ͨ����
//`define REG_OUT_DEFAULT 16'hfdff  // LED��ʾ�ϵ����,ѡ������16,˫ͨ����

module LPC_Device
(
	input SLP_S5n,
	input lclk,				// Clock
	input lreset_n,			// Reset - Active Low (Same as PCI Reset)
	input lpc_en,			// �������ʹ���ź�,�ߵ�ƽʱ������Ч
	input device_cs,
	input [7:0] addr,		// ��ַ
	input [7:0] din,
	output reg [7:0] dout,
	input io_rden,
	input io_wren,

	input INHMDG,			// ���Ź���ֹ
	input DPM_RSTn,			// DPMģ�������ģ��ĸ�λ�ź�
	input HPM_RSTn,			// ����ģ�������ģ��ĸ�λ�ź�
	input MGM_RSTn,			// �ⲿ��ť�ֶ���λ�ź�
	input PSMRSTn,			// ��Դģ�������ģ��ĸ�λ�ź�
	output wire MGM_STS,	// PCI-E״ָ̬ʾ�ź�
	output reg USER_RSTn,	// �û���λ�ź�
	output wire RST_STS,	// ��λ�ź�
	input SYS_MAIN,				// ������� ϵͳ���
	input GSE,					// ������� ����/����
	input [1:0] SYS,					// ������� ϵͳ����
	input [1:0] SYS_ID,			// ��λʶ��
	input [13:2] DIS_IN,			// ��ɢ������
	input L_R,			// λ��ʶ��
	
	input [7:0] PG0,		// Power Good Group 0
	input [7:0] PG1,		// Power Good Group 1
	
	input [7:0] ETH_STS,		// ethernet lan status

	output reg [7:0] LED_REG,	// ���ԼĴ������ڵ���ǰ���LED��
	output wire osc,
	output reg [15:0] reg_out, // �Ĵ������
	output reg [7:0] lpc_reg,	// LPC ���üĴ���
	`ifdef USE_UFM
		output reg ctrl_signal	// =0,�ϵ��Զ�ȡ;=1,�ⲿ����
	`else
		output wire ctrl_signal	// =0,�ϵ��Զ�ȡ;=1,�ⲿ����
	`endif
);

// ȫ��ʹ�õļ�ʱ�������ڸ�����ʱ
// һ����ʱ�����ڴ�Լ184us
reg [9:0] time_kick;
always @(posedge osc) begin
	time_kick <= time_kick + 10'h1;
end

// Ϊ�˱�֤�ϵ��ȶ� ����ϵͳ�ϵ����ʱ1s�ſ�ʼ����
reg [12:0] firsttime_cnt;
wire MYSELF_RSTn;
assign MYSELF_RSTn = (firsttime_cnt == 13'h153b);
//assign MYSELF_RSTn = 1; // Ϊ�˽�ʡ�߼���Դȡ���˼�����
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


	// ����MAX2�����ڲ���5.56MHz����
	maxII_osc maxII_osc(
		.oscena(1'b1),
		.osc(osc)
	);

	assign ctrl_signal = MYSELF_RSTn;


// ͬ���ⲿ�ź�
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

// �����ӳٸ�λ
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

// ��λ�߼�״̬��
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
				if(!(DPM_RSTn_syn & HPM_RSTn_syn & MGM_RSTn_syn & PSMRSTn_syn & watchdog_over)) // ����һ���ź�Ϊ��
					rst_state <= 1;
				else
					rst_state <= rst_state;
			end
			1: begin // ��ʱ0.5s������λ�ź�
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
			2: begin // �ȴ�����������
				rst_state <= 3;
			end
			3: begin // ��λ�źű���0.1s
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

// PCI-E��״̬����ź�(MGM_STS):
// �����������������źŵ����:
// �ߵ�ƽ��ʾģ���ϵ��ʼ����������
reg MGM_STS_reg;
assign MGM_STS = DPM_RSTn_syn & HPM_RSTn_syn & MGM_RSTn_syn & PSMRSTn_syn & MGM_STS_reg & watchdog_over;
assign RST_STS = DPM_RSTn_syn & HPM_RSTn_syn & MGM_RSTn_syn & PSMRSTn_syn & watchdog_over;
`ifdef USE_UFM
	// ��33MHz��ʱ,UFM�Ķ�д�ź�������30~100����֮��
	// ��д�ź���Ч��,����15�����ں����ȥ�ж�nbusy��data_valid��״̬
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

// LPC index/data �Ĵ���
// ��ַ ��д	�Ĵ�����
// 0x00 R		CPLD����汾��
// 0x01 R/W		led�Ĵ���
// 0x02 R/W		UFM���ƼĴ���
// 0x03 R/W		UFM��ַ�Ĵ���
// 0x04 R/W		UFM���ݼĴ���-��8λ
// 0x05 R/W		UFM���ݼĴ���-��8λ
// 0x06 R/W		�û��Ĵ���
// 0x07 R/W     ���Ź�ʹ�ܺ�����Ĵ���
// 0x08 R/W     ���Ź���ʱ��[15:8]
// 0x09 R/W     ���Ź���ʱ��[7:0]
// 0x0a R	    ���Ź������Ĵ���[15:8]
// 0x0b R		���Ź������Ĵ���[7:0]
// 0x0c R/W     �忨���üĴ���[15:8]
// 0x0d R/W     �忨���üĴ���[7:0]
// 0x0e R/W     LPC�Ĵ���[7:0]
// 0x10 R	��ɢ������[0��0��DIS_IN[7:2] ]
// 0x11 R	��ɢ������[DIS_IN[13:8]��0��0]
// 0x12 R	λ��ʶ��
// 0x13 R	power good 0
// 0x14 R	power good 1
// 0x15 R	ETH_STS

// 0x06 �û��Ĵ���
// bit		��д	name				˵��
// 7		R		NC					ʼ��Ϊ0
// 6		R		INHMDG				���Ź���ֹ����
// 5:4		R		SYS_ID				��λʶ��ID��
// 3		R		SYS					ϵͳ��������״̬
// 2		R		GSE					����/���� �ߵ�ƽ���ܽ���BIOS SETUP�˵�
// 1		R		SYS_MAIN			ϵͳ�������״̬
// 0		R/W		MGM_STS				PCI-E��״̬����ź� �ߵ�ƽ��ʾģ���ϵ��ʼ����������

wire [7:0] reg_version;
reg watchdog_ena;
reg watchdog_clean;
reg [9:0] watchdog_cnt_kick;
reg [15:0] watchdog_cnt,watchdog_reg,watchdog_reg_syn;
assign watchdog_over = !(watchdog_cnt==watchdog_reg_syn);
assign reg_version = `CPLD_VERSION;

// ���Ź�������
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

// lpc �Ĵ�������
always @ (posedge lclk or negedge SLP_S5n) begin
	if(!SLP_S5n) begin
		dout <= 8'hzz;
		LED_REG <= 8'h5a;  // s5�ŻḴλ
		lpc_reg <= 8'h00;  // s5�ŻḴλ
		MGM_STS_reg <= 1;
		watchdog_ena <= 0;
		watchdog_clean <= 0;
		watchdog_reg <= 16'hffff;
		`ifdef USE_UFM
			read_reg <= 0;
			write_reg <= 0;
			datain <= 0;
		`else
			reg_out <= `REG_OUT_DEFAULT;  // s5�ŻḴλ
		`endif
	end
	else if(!lreset_n) begin   // s3��λ
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
	else if(device_cs & io_rden & lpc_en) begin // LPC �Ĵ�����
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
	else if(device_cs & io_wren & lpc_en) begin // LPC�Ĵ���д
		if(addr==1)
			LED_REG <= din;
		`ifdef USE_UFM
			else if(addr==2) begin // UFMģ�����
				if(din == 8'h55) begin // ������
					read_reg <= 1;
					write_reg <= 0;
				end
				else if(din == 8'haa) begin // д����
					read_reg <= 0;
					write_reg <= 1;
				end
				else if(din == 8'h00) begin // ������д����
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
