-- AXM_A30.vhd
--
-- Version 3.0
--
-- Description:
-- This is a VHDL component that implements the Acromag AXM-A30 module
-- on the XMC-SLX150 (Spartan 6) board.
--

--WARNING: DUE TO LIMITATIONS IN S6 CLOCK ROUTING A GLOBAL CLOCK CANNOT BE ROUTED OUT
--A STANDARD I/O PIN WITHOUT GENERATING A COMPILER ERRORS.  THIS ISSUE CAN BE BYPASSED
--WITH A CLOCK CONSTRAINT.  THIS CONSTAINT IS REQUIRED FOR THE FPGA CLOCK TO THE LMK03000C.
--NOTE THAT THIS CLOCK SHOULD ONLY BE USED FOR DEMOS AND NOT IN A FINAL SYSTEM DUE TO NOISE
--ISSUE.  The constraint is provided at the end of ucf file.

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;

LIBRARY UNISIM;
USE UNISIM.vcomponents.all;

 -- ENTITY declaration for internal FIFO
entity AXM_A30 is
   port (
-- PORT modes IN, OUT, INOUT, BUFFER (an output feedback)
   CLK: in STD_LOGIC; --Local clock
	RESET: in STD_LOGIC; --Global Reset signal (active high)
	
	ADS_n: in STD_LOGIC;  -- Address Strobe from PCI9056
	LW_R_n: in STD_LOGIC;  -- Local Bus Write / Read driven by PCI9056
   LBE0_n: in STD_LOGIC; --Local Data Bus Byte 0 Enables active low
	LBE1_n: in STD_LOGIC; --Local Data Bus Byte 1 Enables active low
	LBE2_n: in STD_LOGIC; --Local Data Bus Byte 2 Enables active low
	LBE3_n: in STD_LOGIC; --Local Data Bus Byte 3 Enables active low	 
	LD : in  STD_LOGIC_VECTOR (31 downto 0); --Local Data Bus
	READ_DATA: out STD_LOGIC_VECTOR(31 downto 0); --Readback data bus of AXM_A30 Registers
	Int_Status: out STD_LOGIC_VECTOR(1 downto 0); -- Interrupt status of 2 ADC channels

	ADCControl_Adr: in STD_LOGIC; -- AXM-A30 Board Control Reg Strobe
	ADCCh1_Adr: in STD_LOGIC;  -- ADC Channel 1 & 2 Control Reg Strobe
	ADCCh2_Adr: in STD_LOGIC;   -- ADC Channel 1 & 2 Threshold Reg Strobe
	ADCFifo_Adr: in STD_LOGIC;    -- ADC Channel 1 & 2 FIFO Reg Strobe
	PLLControl_Adr: in STD_LOGIC;     -- PLL Control/Status Reg Strobe
	PLLR0_Adr: in STD_LOGIC;    -- PLL R0 Reg Strobe
	PLLR1_Adr: in STD_LOGIC;    -- PLL R1 Reg Strobe
	PLLR3_Adr: in STD_LOGIC;    -- PLL R3 Reg Strobe
	PLLR4_Adr: in STD_LOGIC;    -- PLL R4 Reg Strobe
	PLLR11_Adr: in STD_LOGIC;    -- PLL R11 Reg Strobe
	PLLR13_Adr: in STD_LOGIC;    -- PLL R13 Reg Strobe
	PLLR14_Adr: in STD_LOGIC;    -- PLL R14 Reg Strobe
	PLLR15_Adr: in STD_LOGIC;    -- PLL R15 Reg Strobe
	ADC_SRAM_Adr: in STD_LOGIC;   --AXM-A30 SRAM Control Stobe
	
	AXM_ID: out STD_LOGIC_VECTOR(2 downto 0); -- AXM Module ID
	
--ADC pin connections -------------------------------------------------------

	AD_DCS: out STD_LOGIC_VECTOR(1 downto 0);	-- AD9446 Duty Cycle Stabilizer Active High  (both channels)
	AD_DFS: out STD_LOGIC_VECTOR(1 downto 0);	-- AD9446 Data Format Select (both channels)  
						-- 1 corresp to 2's compl. 0 corresp offset binary.

   AD_CLK1_P: in STD_LOGIC;  --Channel 1 diff input data clock
   AD_CLK1_N: in STD_LOGIC;  --Channel 1 diff input data clock
   AD_CLK2_P: in STD_LOGIC;  --Channel 2 diff input data clock
   AD_CLK2_N: in STD_LOGIC;  --Channel 2 diff input data clock
	
	AD_D1_P: in STD_LOGIC_VECTOR (15 downto 0); -- AD9446 Ch 1 Data Bus Differential Input
	AD_D1_N: in STD_LOGIC_VECTOR (15 downto 0); 
   AD_D2_P: in STD_LOGIC_VECTOR (15 downto 0); -- AD9446 Ch 2 Data Bus Differential Input
	AD_D2_N: in STD_LOGIC_VECTOR (15 downto 0);
	
	Reg_AD_OTR: in STD_LOGIC_VECTOR(1 downto 0);	 -- AD9446 Out of Range Active High  (both channels)

-- General Purpose I/O
	GPIO: inout STD_LOGIC;    --External General Purpose Input/Output
	
-- Note that only one of the following three signals can be enabled at any given time.
-- If more then one signal is active, there will be a bus conflict that may
-- damage the board.
	ExtInput_En: out STD_LOGIC;   --Set low to enable the GPIO as an Input
	ExtOutput_En: out STD_LOGIC;  --Set high to enable the GPIO as an Output
	ExtClkOut_En: out STD_LOGIC;  --Set low to feed LMK Clk1 out to the GPIO

-- LMK03000 Clock Input Control  
-- Note that only one clock enable singal of the following three can be enable at any time.
-- If more then one is enable, there will be a bus conflict that may damage the board.

   Crystal_En: out STD_LOGIC;    --Set high to enable opscillator to LMK03000C input
	ExtCLKIn_En: out STD_LOGIC;   --Set high to enable External Clock to LMK03000C input 
	FPGACLKIn_En: out STD_LOGIC;  --Set high to enable FPGA Clock to LMK03000C input
	
--LMK03000 PLL Connections --------------------------------------------------

	SerDataOut: out STD_LOGIC;  --LMK03000C Serial Interface Data line
	SerCLKOut: out STD_LOGIC;	--LMK03000C Serial Interface CLK (<20MHz)
	SerLatch_n: out STD_LOGIC;  --LMK03000C Serial Interface Enable(Latch)Active Low line  
	
	LMK_GOE: out STD_LOGIC;     --LMK03000C Global Output Enable Pin
	LMK_SYNCH: out STD_LOGIC;	--LMK03000C Synch Pin
	LOCK: in STD_LOGIC;        --LKM03000C Clock Lock Indicator Pin
  
  --Other Connections
   FPGACLKIN: in STD_LOGIC;		--Output from LMK03000C device to FPGA  Note that it is
											--converted from LVDS to single ended by a transceiver.
											--This input is not utilized in this example design.
	FPGACLK_IN_EN: out STD_LOGIC; --Enable signal for output from LMK03000C device to FPGA  
											--Note that this enables an LVDS to single ended by a transceiver.
											 
   FPGACLK_OUT: out STD_LOGIC;	--16.6667MHz for V4, 13.3MHz for V5
											--	clock that can be used as input for the LMK03000.
											--Use only for demo purposes.
	
	CH1_PDN: out STD_LOGIC;		--ADA4937 active low power-down pin Channel 1
	CH2_PDN: out STD_LOGIC;		--ADA4937 active low power-down pin Channel 2
	
	UnusedFrontIO: out STD_LOGIC_VECTOR (4 downto 0); --Set unused Front I/O lines to GND.

	--Connections for writing ADC data to SRAM
	SRAM_ENABLED: in STD_LOGIC;  --Enable FIFO read to send data to SRAM read (from DPSRAM component).
	SRAM_FIFO_DATA: out STD_LOGIC_VECTOR(15 downto 0);  --Data to write to SRAM.
	FIFO_EMPTY_FLAG: out STD_LOGIC;  --FIFO empty flag, specific to channel that is being sent to SRAM.
	ADC_OVERRIDE_EN: out STD_LOGIC;   --Enable ADC data stream to SRAM.
	AXM_SERCLK: in STD_LOGIC;  --15MHz clk derived from PLL.
	
	FIFOtoSRAMPause: in STD_LOGIC; --Hold FIFO Reads while SRAM write operation is in progress.
	FIFODataValidFlag: out STD_LOGIC
	
	);		

end AXM_A30;

-- ARCHITECTURE body describes the content of the design.
architecture AXM_A30_arch of AXM_A30 is

-- Signals are internal wires or nets.
-- ADC Clock and data bus signals
signal AD_D1, AD_D2: STD_LOGIC_VECTOR(15 downto 0);
signal AD_D1_Reg, AD_D2_Reg: STD_LOGIC_VECTOR(15 downto 0);
signal AD_CLK1, AD_CLK2: STD_LOGIC;
signal AD_CLK1_BUF, AD_CLK2_BUF: STD_LOGIC;

-- Write Strobe signals --------------------------------------
signal ADCControl_Stb0 : STD_LOGIC;
signal ADCControl_Stb1 : STD_LOGIC;
signal ADCControl_Stb2: STD_LOGIC;
signal ADCConCh1_Stb0 : STD_LOGIC;
signal ADCConCh1_Stb1 : STD_LOGIC;
signal ADCThrCh1_Stb0 : STD_LOGIC;
signal ADCThrCh1_Stb1 : STD_LOGIC;
signal ADCConCh2_Stb2 : STD_LOGIC;
signal ADCConCh2_Stb3 : STD_LOGIC;
signal ADCThrCh2_Stb2 : STD_LOGIC;    
signal ADCThrCh2_Stb3 : STD_LOGIC;

signal PLLControl_Stb0 : STD_LOGIC;
signal PLLR0_StbAll : STD_LOGIC;
signal PLLR1_StbAll : STD_LOGIC;
signal PLLR3_StbAll : STD_LOGIC;
signal PLLR4_StbAll : STD_LOGIC;
signal PLLR11_StbAll : STD_LOGIC;
signal PLLR13_StbAll : STD_LOGIC;
signal PLLR14_StbAll : STD_LOGIC;
signal PLLR15_StbAll : STD_LOGIC;
signal ADC_SRAM_Stb0 : STD_LOGIC;
signal ADC_SRAM_Stb1 : STD_LOGIC;
--Registers
 --ADC
signal ADCControl_Reg : STD_LOGIC_VECTOR(6 downto 0);
signal GPIO_Reg : STD_LOGIC;
signal PDN_Reg: STD_LOGIC_VECTOR(1 downto 0);
signal ADCConCh1_Reg : STD_LOGIC_VECTOR(15 downto 0); 
signal ADCConCh2_Reg : STD_LOGIC_VECTOR(15 downto 0); 
signal ADCThrCh1_Reg: STD_LOGIC_VECTOR (12 downto 0);
signal ADCThrCh2_Reg: STD_LOGIC_VECTOR (12 downto 0);

--Clock Conditioner
signal PLLGOE: STD_LOGIC;
signal PLLR0_Reg: STD_LOGIC_VECTOR(31 downto 0);
signal PLLR1_Reg: STD_LOGIC_VECTOR(31 downto 0);
signal PLLR3_Reg: STD_LOGIC_VECTOR(31 downto 0);
signal PLLR4_Reg: STD_LOGIC_VECTOR(31 downto 0);
signal PLLR11_Reg: STD_LOGIC_VECTOR(31 downto 0);
signal PLLR13_Reg: STD_LOGIC_VECTOR(31 downto 0);
signal PLLR14_Reg: STD_LOGIC_VECTOR(31 downto 0);
signal PLLR15_Reg: STD_LOGIC_VECTOR(31 downto 0);

--SRAM
signal ADC_SRAM_Reg: STD_LOGIC_VECTOR (15 downto 0);

--ADC Channel 1 Signals
signal CH1_FIFO_RESET, CH1_FIFO_RESET_REG, CH1_FIFO_RESET_REG1: STD_LOGIC;
signal CH1_FIFO_RESET_REG2, CH1_FIFO_RESET_REG3: STD_LOGIC;
signal CH1_FIFOCount: STD_LOGIC_VECTOR(12 downto 0);
signal CH1_FIFO_DOUT: STD_LOGIC_VECTOR(15 downto 0);
signal CH1_THESH_LIMIT, CH1_WRFIFOEN, CH1_RDFIFOEN, CH1_Empty, CH1_Full: STD_LOGIC;
signal CH1_Stop_Conv: STD_LOGIC;

--ADC Channel 2 Signals
signal CH2_FIFO_RESET, CH2_FIFO_RESET_REG, CH2_FIFO_RESET_REG1: STD_LOGIC;
signal CH2_FIFO_RESET_REG2, CH2_FIFO_RESET_REG3: STD_LOGIC;
signal CH2_FIFOCount: STD_LOGIC_VECTOR(12 downto 0);
signal CH2_FIFO_DOUT: STD_LOGIC_VECTOR(15 downto 0);
signal CH2_THESH_LIMIT, CH2_WRFIFOEN, CH2_RDFIFOEN, CH2_Empty, CH2_Full: STD_LOGIC;
signal CH2_Stop_Conv: STD_LOGIC;

--GPIO Signals
signal GPIO_Out, GPIOOut_En: STD_LOGIC;
 
--Interrupt Signals 
signal ADCh1_Interrupt_Pending, ADCh2_Interrupt_Pending: STD_LOGIC;

--CLock Conditioner Programming signals
signal SerClk, SerCLK_En: STD_LOGIC;
signal SerCnt: STD_LOGIC_VECTOR(4 downto 0);
signal SerData: STD_LOGIC_VECTOR(31 downto 0);

signal SerCnt0, SerCnt1, SerCnt2, SerCnt3: STD_LOGIC;
signal SerCnt4, SerCnt5, SerCnt6, SerCnt7: STD_LOGIC;
signal SerCnt8, SerCnt9, SerCnt10, SerCnt11: STD_LOGIC;
signal SerCnt12, SerCnt13, SerCnt14, SerCnt15: STD_LOGIC;
signal SerCnt16, SerCnt17, SerCnt18, SerCnt19: STD_LOGIC;
signal SerCnt20, SerCnt21, SerCnt22, SerCnt23: STD_LOGIC;
signal SerCnt24, SerCnt25, SerCnt26, SerCnt27: STD_LOGIC;
signal SerCnt28, SerCnt29, SerCnt30, SerCnt31: STD_LOGIC;

signal Write_R0, Write_R1, Write_R3, Write_R4: STD_LOGIC;
signal Write_R11, Write_R13, Write_R14, Write_R15: STD_LOGIC;
signal Write_R0_HOLD, Write_R1_HOLD, Write_R3_HOLD: STD_LOGIC;
signal Write_R4_HOLD, Write_R11_HOLD, Write_R13_HOLD: STD_LOGIC;
signal Write_R14_HOLD, Write_R15_HOLD: STD_LOGIC;
signal SerEn, Ser_Latch, SerOut: STD_LOGIC;
signal DataMux: STD_LOGIC_VECTOR(7 downto 0);

signal Synch_EN, Synch_Complete: STD_LOGIC;
signal Synch_Counter: STD_LOGIC_VECTOR(4 downto 0);

--Clock Conditioner Reset Signals
signal Write_R0_Reg, Write_R0_LowEvent: STD_LOGIC;
signal CLKCON_RESET: STD_LOGIC;

--SRAM override signals
signal ADC_SRAM_CH0_EN, ADC_SRAM_CH1_EN: STD_LOGIC;

--Other signals
signal FPGACLKIn_En_Buf: STD_LOGIC;
signal FPGACLK_OUT_BUF: STD_LOGIC;
signal FPGACLKIN_TRANS_EN: STD_LOGIC;
signal CH1_Valid, CH2_Valid: STD_LOGIC;

--The ADCFIFO was generated by the Xilinx Core Generator.  ISE 10.1.01 is required to
--correctly compile and simulate this component.
component ADCFIFO
	port (
	din: IN std_logic_VECTOR(15 downto 0);  --FIFO data input
	rd_clk: IN std_logic; --Read Clock
	rd_en: IN std_logic;  --Read enable
	rst: IN std_logic;   -- Synch. Reset
	wr_clk: IN std_logic; --Write Clock
	wr_en: IN std_logic;  --Write Enable
	dout: OUT std_logic_VECTOR(15 downto 0); --FIFO Data Out
	empty: OUT std_logic; --Empty Flag
	full: OUT std_logic;  --Full Flag
	valid: OUT std_logic; --Read output is valid
	rd_data_count: OUT std_logic_VECTOR(12 downto 0)); -- # of samples available for reading
end component;

-- Synplicity black box declaration
--attribute syn_black_box : boolean;
--attribute syn_black_box of ADCFIFO: component is true;


BEGIN
 
----------------------Differential Xilinx Primitives-----------------


--Global Clock primitives
DataCLK1: IBUFDS_LVDS_25 
port map(
	I => AD_CLK1_P,
  IB => AD_CLK1_N,
	O => AD_CLK1_BUF
	--O => AD_CLK1
	);

DataCLK2: IBUFDS_LVDS_25 
port map(
	I => AD_CLK2_P,
	IB => AD_CLK2_N,
	O => AD_CLK2_BUF
	--O => AD_CLK2
	);

GLOBAL_ADCh1_CLK: BUFG
  port map (
  	    I => AD_CLK1_BUF,
            O => AD_CLK1
  	   ); 		
		
GLOBAL_ADCh2_CLK: BUFG
  port map (
  	    I => AD_CLK2_BUF,
            O => AD_CLK2
  	   ); 	

-- Channel 1 Data Bus primitives

AD1_D0: IBUFDS_LVDS_25
		 port map(
			I => AD_D1_P(0),
			IB => AD_D1_N(0),
			O => AD_D1(0)
		 );
AD1_D1: IBUFDS_LVDS_25
		 port map(
			I => AD_D1_P(1),
			IB => AD_D1_N(1),
			O => AD_D1(1)
		 );	
AD1_D2: IBUFDS_LVDS_25
		 port map(
			I => AD_D1_P(2),
			IB => AD_D1_N(2),
			O => AD_D1(2)
		 );
AD1_D3: IBUFDS_LVDS_25
		 port map(
			I => AD_D1_P(3),
			IB => AD_D1_N(3),
			O => AD_D1(3)
		 );
AD1_D4: IBUFDS_LVDS_25
		 port map(
			I => AD_D1_P(4),
			IB => AD_D1_N(4),
			O => AD_D1(4)
		 ); 
AD1_D5: IBUFDS_LVDS_25
		 port map(
			I => AD_D1_P(5),
			IB => AD_D1_N(5),
			O => AD_D1(5)
		 );
AD1_D6: IBUFDS_LVDS_25
		 port map(
			I => AD_D1_P(6),
			IB => AD_D1_N(6),
			O => AD_D1(6)
		 );		 
AD1_D7: IBUFDS_LVDS_25
		 port map(
			I => AD_D1_P(7),
			IB => AD_D1_N(7),
			O => AD_D1(7)
		 );		 
AD1_D8: IBUFDS_LVDS_25
		 port map(
			I => AD_D1_P(8),
			IB => AD_D1_N(8),
			O => AD_D1(8)
		 );
AD1_D9: IBUFDS_LVDS_25
		 port map(
			I => AD_D1_P(9),
			IB => AD_D1_N(9),
			O => AD_D1(9)
		 );
AD1_D10: IBUFDS_LVDS_25
		 port map(
			I => AD_D1_P(10),
			IB => AD_D1_N(10),
			O => AD_D1(10)
		 );
AD1_D11: IBUFDS_LVDS_25
		 port map(
			I => AD_D1_P(11),
			IB => AD_D1_N(11),
			O => AD_D1(11)
		 );
AD1_D12: IBUFDS_LVDS_25
		 port map(
			I => AD_D1_P(12),
			IB => AD_D1_N(12),
			O => AD_D1(12)
		 );
AD1_D13: IBUFDS_LVDS_25
		 port map(
			I => AD_D1_P(13),
			IB => AD_D1_N(13),
			O => AD_D1(13)
		 );
AD1_D14: IBUFDS_LVDS_25
		 port map(
			I => AD_D1_P(14),
			IB => AD_D1_N(14),
			O => AD_D1(14)
		 );
AD1_D15: IBUFDS_LVDS_25
		 port map(
			I => AD_D1_P(15),
			IB => AD_D1_N(15),
			O => AD_D1(15)
		 );

-- Channel 2 Data Bus primitives

AD2_D0: IBUFDS_LVDS_25
		 port map(
			I => AD_D2_P(0),
			IB => AD_D2_N(0),
			O => AD_D2(0)
		 );
AD2_D1: IBUFDS_LVDS_25
		 port map(
			I => AD_D2_P(1),
			IB => AD_D2_N(1),
			O => AD_D2(1)
		 );
AD2_D2: IBUFDS_LVDS_25
		 port map(
			I => AD_D2_P(2),
			IB => AD_D2_N(2),
			O => AD_D2(2)
		 );
AD2_D3: IBUFDS_LVDS_25
		 port map(
			I => AD_D2_P(3),
			IB => AD_D2_N(3),
			O => AD_D2(3)
		 );
AD2_D4: IBUFDS_LVDS_25
		 port map(
			I => AD_D2_P(4),
			IB => AD_D2_N(4),
			O => AD_D2(4)
		 );
AD2_D5: IBUFDS_LVDS_25
		 port map(
			I => AD_D2_P(5),
			IB => AD_D2_N(5),
			O => AD_D2(5)
		 );
AD2_D6: IBUFDS_LVDS_25
		 port map(
			I => AD_D2_P(6),
			IB => AD_D2_N(6),
			O => AD_D2(6)
		 );
AD2_D7: IBUFDS_LVDS_25
		 port map(
			I => AD_D2_P(7),
			IB => AD_D2_N(7),
			O => AD_D2(7)
		 );
AD2_D8: IBUFDS_LVDS_25
		 port map(
			I => AD_D2_P(8),
			IB => AD_D2_N(8),
			O => AD_D2(8)
		 );
AD2_D9: IBUFDS_LVDS_25
		 port map(
			I => AD_D2_P(9),
			IB => AD_D2_N(9),
			O => AD_D2(9)
		 );
AD2_D10: IBUFDS_LVDS_25
		 port map(
			I => AD_D2_P(10),
			IB => AD_D2_N(10),
			O => AD_D2(10)
		 );	
AD2_D11: IBUFDS_LVDS_25
		 port map(
			I => AD_D2_P(11),
			IB => AD_D2_N(11),
			O => AD_D2(11)
		 );
AD2_D12: IBUFDS_LVDS_25
		 port map(
			I => AD_D2_P(12),
			IB => AD_D2_N(12),
			O => AD_D2(12)
		 );
AD2_D13: IBUFDS_LVDS_25
		 port map(
			I => AD_D2_P(13),
			IB => AD_D2_N(13),
			O => AD_D2(13)
		 );
AD2_D14: IBUFDS_LVDS_25
		 port map(
			I => AD_D2_P(14),
			IB => AD_D2_N(14),
			O => AD_D2(14)
		 );
AD2_D15: IBUFDS_LVDS_25
		 port map(
			I => AD_D2_P(15),
			IB => AD_D2_N(15),
			O => AD_D2(15)
		 );


	 
 --AXM ID Register 8000H bits 13-15
 --  "001" -> AXM-EDK, AXM-D, or None
 --  "010" -> AXM-A30
AXM_ID <= "010";

-- AXM Read Strobes

   --Read FIFO 1 
   CH1_RDFIFOEN <= (ADCFifo_Adr and not ADS_n and not LW_R_n and (not LBE0_n and not LBE1_n)) or
	(SRAM_ENABLED and ADC_SRAM_CH0_EN and not FIFOtoSRAMPause);
	
	--Read FIFO 2 
   CH2_RDFIFOEN <= (ADCFifo_Adr and not ADS_n and not LW_R_n and (not LBE2_n and not LBE3_n)) or
   (SRAM_ENABLED and ADC_SRAM_CH1_EN and not FIFOtoSRAMPause);
	
	
-- AXM Write Strobes ------------------------------------------
 
--0x8100 
  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         ADCControl_Stb0 <= ADCControl_Adr and not ADS_n  and
	                        not LBE0_n and LW_R_n;
      end if;
  end process;
  
  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         ADCControl_Stb1 <= ADCControl_Adr and not ADS_n  and
	                        not LBE1_n and LW_R_n;
      end if;
  end process;
  
  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         ADCControl_Stb2 <= ADCControl_Adr and not ADS_n  and
	                        not LBE2_n and LW_R_n;
      end if;
  end process;
 
--0x8104  
  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         ADCConCh1_Stb0 <= ADCCh1_Adr and not ADS_n  and
	                        not LBE0_n and LW_R_n;
      end if;
  end process;
    
  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         ADCConCh1_Stb1 <= ADCCh1_Adr and not ADS_n  and
	                        not LBE1_n and LW_R_n;
      end if;
  end process;

  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         ADCConCh2_Stb2 <= ADCCh1_Adr and not ADS_n  and
	                        not LBE2_n and LW_R_n;
      end if;
  end process;  

  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         ADCConCh2_Stb3 <= ADCCh1_Adr and not ADS_n  and
	                        not LBE3_n and LW_R_n;
      end if;
  end process;

--0x8108
  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         ADCThrCh1_Stb0 <= ADCCh2_Adr and not ADS_n  and
	                        not LBE0_n and LW_R_n;
      end if;
  end process;
    
  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         ADCThrCh1_Stb1 <= ADCCh2_Adr and not ADS_n  and
	                        not LBE1_n and LW_R_n;
      end if;
  end process;

  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         ADCThrCh2_Stb2 <= ADCCh2_Adr and not ADS_n  and
	                        not LBE2_n and LW_R_n;
      end if;
  end process;  

  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         ADCThrCh2_Stb3 <= ADCCh2_Adr and not ADS_n  and
	                        not LBE3_n and LW_R_n;
      end if;
  end process; 
  
--0x8110  
  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         PLLControl_Stb0 <= PLLControl_Adr and not ADS_n  and
	                        not LBE0_n and LW_R_n;
      end if;
  end process;


--0x8114

  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         PLLR0_StbAll <= PLLR0_Adr and not ADS_n  and
	                        not LBE0_n and not LBE1_n and not LBE2_n
									and not LBE3_n and LW_R_n;
      end if;
  end process;

--0x8118

  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         PLLR1_StbAll <= PLLR1_Adr and not ADS_n  and
	                        not LBE0_n and not LBE1_n and not LBE2_n
									and not LBE3_n and LW_R_n;
      end if;
  end process;
  
--0x811C

  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         PLLR3_StbAll <= PLLR3_Adr and not ADS_n  and
	                        not LBE0_n and not LBE1_n and not LBE2_n
									and not LBE3_n and LW_R_n;
      end if;
  end process;

--0x8120

  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         PLLR4_StbAll <= PLLR4_Adr and not ADS_n  and
	                        not LBE0_n and not LBE1_n and not LBE2_n
									and not LBE3_n and LW_R_n;
      end if;
  end process;
  
--0x8124

  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         PLLR11_StbAll <= PLLR11_Adr and not ADS_n  and
	                        not LBE0_n and not LBE1_n and not LBE2_n
									and not LBE3_n and LW_R_n;
      end if;
  end process;

--0x8128

  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         PLLR13_StbAll <= PLLR13_Adr and not ADS_n  and
	                        not LBE0_n and not LBE1_n and not LBE2_n
									and not LBE3_n and LW_R_n;
      end if;
  end process; 
 
--0x812C

  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         PLLR14_StbAll <= PLLR14_Adr and not ADS_n  and
	                        not LBE0_n and not LBE1_n and not LBE2_n
									and not LBE3_n and LW_R_n;
      end if;
  end process;

--0x8130

  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         PLLR15_StbAll <= PLLR15_Adr and not ADS_n  and
	                        not LBE0_n and not LBE1_n and not LBE2_n
									and not LBE3_n and LW_R_n;
      end if;
  end process; 
   
--0x8134
  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         ADC_SRAM_Stb0<= ADC_SRAM_Adr and not ADS_n  and
	                        not LBE0_n and LW_R_n;
      end if;
  end process; 
  
 
  process (CLK) 
  begin
      if (CLK'event and CLK = '1') then
         ADC_SRAM_Stb1<= ADC_SRAM_Adr and not ADS_n  and
	                        not LBE1_n and LW_R_n;
      end if;
  end process; 



 
-- Registers ----------------------------------------------------------------

 --ADC Control Register 0x8100--
  
 --Bits 0,1: General Purpose I/O Control
 --Bit 2: Not Used
 --Bits 3,4: Clock Source Control
  process (CLK, RESET)
  begin
      if (RESET = '1') then
         ADCControl_Reg(1 downto 0) <= "00";         
      elsif (CLK'event and CLK = '1') then
         if (ADCControl_Stb0 = '1') then
            ADCControl_Reg(1 downto 0) <= LD(1 downto 0);
         else
            ADCControl_Reg(1 downto 0) <= ADCControl_Reg(1 downto 0);
         end if;
      end if;
  end process;
  
  ADCControl_Reg(2) <= '0';
  
  process (CLK, RESET)
  begin
      if (RESET = '1') then
         ADCControl_Reg(4 downto 3) <= "00";         
      elsif (CLK'event and CLK = '1') then
         if (ADCControl_Stb0 = '1') then
            ADCControl_Reg(4 downto 3) <= LD(4 downto 3);
         else
            ADCControl_Reg(4 downto 3) <= ADCControl_Reg(4 downto 3);
         end if;
      end if;
  end process;
 
--Bit 5: Global Channel 1 FIFO Enable bit.  Stop at threshold will reset. 
  process(CLK, RESET)
  begin
      if (RESET = '1') then 
			ADCControl_Reg(5) <= '0';
		elsif (CLK'event and CLK = '1') then
         if (ADCControl_Stb0 = '1') then
            ADCControl_Reg(5) <= LD(5);
         else
            ADCControl_Reg(5) <= ADCControl_Reg(5) and not CH1_Stop_Conv;
         end if;
      end if;
  end process;

--Bit 6: Global Channel 2 FIFO Enable bit.  Stop at threshold will reset. 
  process(CLK, RESET)
  begin
      if (RESET = '1') then 
			ADCControl_Reg(6) <= '0';
		elsif (CLK'event and CLK = '1') then
         if (ADCControl_Stb0 = '1') then
            ADCControl_Reg(6) <= LD(6);
         else
            ADCControl_Reg(6) <= ADCControl_Reg(6) and not CH2_Stop_Conv;
         end if;
      end if;
  end process;
  
--Bit 7: Not Used
--Bit 8: Active Low Powerdown for Channel 1 OpAmp
--Bit 9: Active Low Powerdown for Channel 2 OpAmp
  
  process (CLK, RESET)
  begin
      if (RESET = '1') then
         PDN_Reg <= "00";         
      elsif (CLK'event and CLK = '1') then
         if (ADCControl_Stb1 = '1') then
            PDN_Reg <= LD(9 downto 8);
         else
            PDN_Reg <= PDN_Reg;
         end if;
      end if;
  end process; 
  
  CH1_PDN <= PDN_Reg(0);
  CH2_PDN <= PDN_Reg(1);
  
--Bit 10: Active high Enable for FPGACLOCK_IN transceiver.
--        Setting this bit high will enable the transceiver between the 
--        CLKOUT0 output from the LMK3000 and the Virtex FPGA. 

  process (CLK, RESET)
  begin
      if (RESET = '1') then
         FPGACLKIN_TRANS_EN <= '0';         
      elsif (CLK'event and CLK = '1') then
         if (ADCControl_Stb1 = '1') then
            FPGACLKIN_TRANS_EN <= LD(10);
         else
            FPGACLKIN_TRANS_EN <= FPGACLKIN_TRANS_EN;
         end if;
      end if;
  end process; 
 
  FPGACLK_IN_EN <= not FPGACLKIN_TRANS_EN;  --invert logic

   
  --General Purpose I/O Status Register 0x8102--
  --Bit 0: General Purpose I/O Output Value
  process (CLK, RESET)
  begin
      if (RESET = '1') then
         GPIO_Reg <= '0';         
      elsif (CLK'event and CLK = '1') then
         if (ADCControl_Stb2 = '1') then
            GPIO_Reg <= LD(16);
         else
            GPIO_Reg <= GPIO_Reg;
         end if;
      end if;
  end process;
  
  
  --ADC Channel 1 Control/Status Register 0x8104--

--Bit 0: Channel 1 FIFO Enable bit.  Stop at threshold will reset.
--process (CLK, RESET)
--  begin
--      if (RESET = '1') then
--         ADCConCh1_Reg(0) <= '0';         
--      elsif (CLK'event and CLK = '1') then
--         if (ADCConCh1_Stb0 = '1') then
--            ADCConCh1_Reg(0) <= LD(0);
--         else
---           ADCConCh1_Reg(0) <= ADCConCh1_Reg(0)and not CH1_Stop_Conv;
--         end if;
--     end if;
--  end process;
ADCConCh1_Reg(0) <= '0';
--Bit 1: Data format.
--Bit 2: Duty Cycle Stabilizer Enable

  process (CLK, RESET)
  begin
      if (RESET = '1') then
         ADCConCh1_Reg(2 downto 1) <= "00";         
      elsif (CLK'event and CLK = '1') then
         if (ADCConCh1_Stb0 = '1') then
            ADCConCh1_Reg(2 downto 1) <= LD(2 downto 1);
         else
            ADCConCh1_Reg(2 downto 1) <= ADCConCh1_Reg(2 downto 1);
         end if;
      end if;
  end process;
 

 --Bits not registered
 ADCConCh1_Reg(9 downto 3) <= "0000000";
 
 --Bit 10: Stop ADC Conversion on Threshold
 --Bit 11: Interrupt Enable
  process (CLK, RESET)
  begin
      if (RESET = '1') then
         ADCConCh1_Reg(11 downto 10) <= "00";         
      elsif (CLK'event and CLK = '1') then
         if (ADCConCh1_Stb1 = '1') then
            ADCConCh1_Reg(11 downto 10) <= LD(11 downto 10);
         else
            ADCConCh1_Reg(11 downto 10) <= ADCConCh1_Reg(11 downto 10);
         end if;
      end if;
  end process;
 
  --Bits not registered
 ADCConCh1_Reg(15 downto 12) <= "0000";
 
 --ADC Channel 2 Control/Status Register 0x8106--
 
 --Bit 0: Channel 2 FIFO Enable bit.  Stop at threshold will reset.
--process (CLK, RESET)
--  begin
--      if (RESET = '1') then
--         ADCConCh2_Reg(0) <= '0';         
--      elsif (CLK'event and CLK = '1') then
--         if (ADCConCh2_Stb2 = '1') then
--            ADCConCh2_Reg(0) <= LD(0);
--         else
--            ADCConCh2_Reg(0) <= ADCConCh2_Reg(0)and not CH2_Stop_Conv;
--         end if;
--      end if;
--  end process;
ADCConCh2_Reg(0) <= '0';

--Bit 1: Data format.
--Bit 2: Duty Cycle Stabilizer Enable
  process (CLK, RESET)
  begin
      if (RESET = '1') then
         ADCConCh2_Reg(2 downto 1) <= "00";         
      elsif (CLK'event and CLK = '1') then
         if (ADCConCh2_Stb2 = '1') then
            ADCConCh2_Reg(2 downto 1) <= LD(18 downto 17);
         else
            ADCConCh2_Reg(2 downto 1) <= ADCConCh2_Reg(2 downto 1);
         end if;
      end if;
  end process;
 
  --Bits not registered
 ADCConCh2_Reg(9 downto 3) <= "0000000";
 
 --Bit 10: Stop ADC Conversion on Threshold
 --Bit 11: Interrupt Enable
  process (CLK, RESET)
  begin
      if (RESET = '1') then
         ADCConCh2_Reg(11 downto 10) <= "00";         
      elsif (CLK'event and CLK = '1') then
         if (ADCConCh2_Stb3 = '1') then
            ADCConCh2_Reg(11 downto 10) <= LD(27 downto 26);
         else
            ADCConCh2_Reg(11 downto 10) <= ADCConCh2_Reg(11 downto 10);
         end if;
      end if;
  end process;

  --Bits not registered
 ADCConCh2_Reg(15 downto 12) <= "0000";
 
 
   --ADC Channel 1 Threshold Register 0x8108--
  process (CLK, RESET)
  begin
      if (RESET = '1') then
         ADCThrCh1_Reg(7 downto 0) <= "00000000";         
      elsif (CLK'event and CLK = '1') then
         if (ADCThrCh1_Stb0 = '1') then
            ADCThrCh1_Reg(7 downto 0) <= LD(7 downto 0);
         else
            ADCThrCh1_Reg(7 downto 0) <= ADCThrCh1_Reg(7 downto 0);
         end if;
      end if;
  end process;
  
  process (CLK, RESET)
  begin
      if (RESET = '1') then
         ADCThrCh1_Reg(12 downto 8) <= "00000";         
      elsif (CLK'event and CLK = '1') then
         if (ADCThrCh1_Stb1 = '1') then
            ADCThrCh1_Reg(12 downto 8) <= LD(12 downto 8);
         else
            ADCThrCh1_Reg(12 downto 8) <= ADCThrCh1_Reg(12 downto 8);
         end if;
      end if;
  end process;
  
 
   --ADC Channel 2 Threshold Register 0x810A--
  process (CLK, RESET)
  begin
      if (RESET = '1') then
         ADCThrCh2_Reg(7 downto 0) <= "00000000";         
      elsif (CLK'event and CLK = '1') then
         if (ADCThrCh2_Stb2 = '1') then
            ADCThrCh2_Reg(7 downto 0) <= LD(23 downto 16);
         else
            ADCThrCh2_Reg(7 downto 0) <= ADCThrCh2_Reg(7 downto 0);
         end if;
      end if;
  end process;
  
  process (CLK, RESET)
  begin
      if (RESET = '1') then
         ADCThrCh2_Reg(12 downto 8) <= "00000";         
      elsif (CLK'event and CLK = '1') then
         if (ADCThrCh2_Stb3 = '1') then
            ADCThrCh2_Reg(12 downto 8) <= LD(28 downto 24);
         else
            ADCThrCh2_Reg(12 downto 8) <= ADCThrCh2_Reg(12 downto 8);
         end if;
      end if;
  end process;
  
  
 --------LMK03000C Control Register---------------------------------
 
 --Clock Conditioner Control Register
 

 process(CLK,  CLKCON_RESET)
	  begin
		if (CLKCON_RESET = '1') then
			PLLGOE <= '0';
		elsif (CLK'event and CLK = '1') then
			if (PLLControl_Stb0 = '1') then
				PLLGOE <= LD(4);
			else
				PLLGOE <= PLLGOE;
			end if;
		end if;
  end process;
 
  
  
--PLL R0 Register 0x8114--

  --Bits 0 to 3 are fixed
  PLLR0_Reg(3 downto 0) <= "0000";
  
  --Bits 4 to 18 are CLK0 controls
   process(CLK, CLKCON_RESET)
	  begin
		if (CLKCON_RESET = '1') then
			PLLR0_Reg(18 downto 4) <= "000000000000000";
		elsif (CLK'event and CLK = '1') then
			if (PLLR0_StbAll = '1') then
				PLLR0_Reg(18 downto 4) <= LD(18 downto 4);
			else
				PLLR0_Reg(18 downto 4) <= PLLR0_Reg(18 downto 4);
			end if;
		end if;
  end process;
   
  
  --Bits 19 to 30 are fixed
  PLLR0_Reg(30 downto 19) <= "000000000000";
  
  --Bit 31 is PLL reset
  process(CLK, CLKCON_RESET)
	  begin
		if (CLKCON_RESET = '1') then
			PLLR0_Reg(31) <= '0';
		elsif (CLK'event and CLK = '1') then
			if (PLLR0_StbAll = '1') then
				PLLR0_Reg(31) <= LD(31);
			else
				PLLR0_Reg(31) <= PLLR0_Reg(31);
			end if;
		end if;
  end process;
  
--PLL R1 Register 0x8118--

  --Bits 0 to 3 are fixed
  PLLR1_Reg(3 downto 0) <= "0001";
  
  --Bits 4 to 18 are CLK1 controls
  process(CLK, CLKCON_RESET)
	  begin
		if (CLKCON_RESET = '1') then
			PLLR1_Reg(18 downto 4) <= "000000000000000";
		elsif (CLK'event and CLK = '1') then
			if (PLLR1_StbAll = '1') then
				PLLR1_Reg(18 downto 4) <= LD(18 downto 4);
			else
				PLLR1_Reg(18 downto 4) <= PLLR1_Reg(18 downto 4);
			end if;
		end if;
  end process;
  
  --Bits 19 to 31 are fixed
  PLLR1_Reg(31 downto 19) <= "0000000000000";
  
--PLL R3 Register 0x811C--

  --Bits 0 to 3 are fixed
  PLLR3_Reg(3 downto 0) <= "0011";
  
  --Bits 4 to 18 are CLK3 controls
  process(CLK, CLKCON_RESET)
	  begin
		if (CLKCON_RESET= '1') then
			PLLR3_Reg(18 downto 4) <= "000000000000000";
		elsif (CLK'event and CLK = '1') then
			if (PLLR3_StbAll = '1') then
				PLLR3_Reg(18 downto 4) <= LD(18 downto 4);
			else
				PLLR3_Reg(18 downto 4) <= PLLR3_Reg(18 downto 4);
			end if;
		end if;
  end process;
  
  --Bits 19 to 31 are fixed
  PLLR3_Reg(31 downto 19) <= "0000000000000";
  
  
--PLL R4 Register 0x8120--

  --Bits 0 to 3 are fixed
  PLLR4_Reg(3 downto 0) <= "0100";
  
  --Bits 4 to 18 are CLK3 controls
  process(CLK, CLKCON_RESET)
	  begin
		if (CLKCON_RESET = '1') then
			PLLR4_Reg(18 downto 4) <= "000000000000000";
		elsif (CLK'event and CLK = '1') then
			if (PLLR4_StbAll = '1') then
				PLLR4_Reg(18 downto 4) <= LD(18 downto 4);
			else
				PLLR4_Reg(18 downto 4) <= PLLR4_Reg(18 downto 4);
			end if;
		end if;
  end process;
  
  --Bits 19 to 31 are fixed
  PLLR4_Reg(31 downto 19) <= "0000000000000";

    
--PLL R11 Register 0x8124--

  --Bits 0 to 3 are fixed
  PLLR11_Reg(3 downto 0) <= "1011";
  
  --Bits 4 to 14 are fixed
  PLLR11_Reg(14 downto 4) <= "00000000000";
  
  --Bits 15 is DIV4 control
  process(CLK, CLKCON_RESET)
	  begin
		if (CLKCON_RESET = '1') then
			PLLR11_Reg(15) <= '0';
		elsif (CLK'event and CLK = '1') then
			if (PLLR11_StbAll = '1') then
				PLLR11_Reg(15) <= LD(15);
			else
				PLLR11_Reg(15) <= PLLR11_Reg(15);
			end if;
		end if;
  end process;
  
  --Bits 16 to 31 are fixed
  PLLR11_Reg(31 downto 16) <= "0000000010000010";

--PLL R13 Register 0x8128--

  --Bits 0 to 3 are fixed
  PLLR13_Reg(3 downto 0) <= "1101";
  
 
  --Bits 4 to 21 are VCO control
  process(CLK, CLKCON_RESET)
	  begin
		if (CLKCON_RESET = '1') then
			PLLR13_Reg(21 downto 4) <= "000000000000000000";
		elsif (CLK'event and CLK = '1') then
			if (PLLR13_StbAll = '1') then
				PLLR13_Reg(21 downto 4) <= LD(21 downto 4);
			else
				PLLR13_Reg(21 downto 4) <= PLLR13_Reg(21 downto 4);
			end if;
		end if;
  end process;
  
  --Bits 22 to 31 are fixed
  PLLR13_Reg(31 downto 22) <= "0000001010";  
  
  
--PLL R14 Register 0x812C--

  --Bits 0 to 3 are fixed
  PLLR14_Reg(3 downto 0) <= "1110";
  
  --Bits 4 to 7 are fixed
  PLLR14_Reg(7 downto 4) <= "0000";
 
  --Bits 8 to 23 are PLL control
  process(CLK, CLKCON_RESET)
	  begin
		if (CLKCON_RESET = '1') then
			PLLR14_Reg(23 downto 8) <= "0000000000000000";
		elsif (CLK'event and CLK = '1') then
			if (PLLR14_StbAll = '1') then
				PLLR14_Reg(23 downto 8) <= LD(23 downto 8);
			else
				PLLR14_Reg(23 downto 8) <= PLLR14_Reg(23 downto 8);
			end if;
		end if;
  end process;
  
  --Bits 24 and 25 are fixed
  PLLR14_Reg(25 downto 24) <= "00"; 
  
  --Bits 26 to 28 are PLL control
  process(CLK, CLKCON_RESET)
	  begin
		if (CLKCON_RESET = '1') then
			PLLR14_Reg(28 downto 26) <= "000";
		elsif (CLK'event and CLK = '1') then
			if (PLLR14_StbAll = '1') then
				PLLR14_Reg(28 downto 26) <= LD(28 downto 26);
			else
				PLLR14_Reg(28 downto 26) <= PLLR14_Reg(28 downto 26);
			end if;
		end if;
  end process;
  
  --Bits 29 to 31 are fixed
  PLLR14_Reg(31 downto 29) <= "000";    
  
  
--PLL R15 Register 0x8130--

  --Bits 0 to 3 are fixed
  PLLR15_Reg(3 downto 0) <= "1111";
  
   --Bits 4 to 7 are fixed
  PLLR15_Reg(7 downto 4) <= "0000";
  
  --Bits 8 to 31 are PLL control
  process(CLK, CLKCON_RESET)
	  begin
		if (CLKCON_RESET = '1') then
			PLLR15_Reg(31 downto 8) <= "000000000000000000000000";
		elsif (CLK'event and CLK = '1') then
			if (PLLR15_StbAll = '1') then
				PLLR15_Reg(31 downto 8) <= LD(31 downto 8);
			else
				PLLR15_Reg(31 downto 8) <= PLLR15_Reg(31 downto 8);
			end if;
		end if;
  end process;
  
-----------SRAM Override Register---------------------------------------------
--0x8134

--Bit 0, ADC SRAM Enable
  process(CLK, RESET)
	  begin
		if (RESET = '1') then
			ADC_SRAM_Reg(0) <= '0';
		elsif (CLK'event and CLK = '1') then
			if (ADC_SRAM_Stb0 = '1') then
				ADC_SRAM_Reg(0) <= LD(0);
			else
				ADC_SRAM_Reg(0) <= ADC_SRAM_Reg(0);
			end if;
		end if;
  end process;
  
  ADC_OVERRIDE_EN <= ADC_SRAM_Reg(0);

--Bits 1 to 7, Not Used
ADC_SRAM_Reg(7 downto 1)  <= "0000000";

--Bit 8, ADC Channel 0 to SRAM Enable

  process(CLK, RESET)
	  begin
		if (RESET = '1') then
			ADC_SRAM_Reg(8) <= '0';
		elsif (CLK'event and CLK = '1') then
			if (ADC_SRAM_Stb1 = '1') then
				ADC_SRAM_Reg(8) <= LD(8);
			else
				ADC_SRAM_Reg(8) <= ADC_SRAM_Reg(8);
			end if;
		end if;
  end process;
  
  ADC_SRAM_CH0_EN <= ADC_SRAM_Reg(8);

--Bit 9, ADC Channel 1 to SRAM Enable

  process(CLK, RESET)
	  begin
		if (RESET = '1') then
			ADC_SRAM_Reg(9) <= '0';
		elsif (CLK'event and CLK = '1') then
			if (ADC_SRAM_Stb1 = '1') then
				ADC_SRAM_Reg(9) <= LD(9);
			else
				ADC_SRAM_Reg(9) <= ADC_SRAM_Reg(9);
			end if;
		end if;
  end process;
  
  ADC_SRAM_CH1_EN <= ADC_SRAM_Reg(9);
  
--Bits 10 to 15 Not Used
ADC_SRAM_Reg(15 downto 10)  <= "000000";


 
  
-----------CH 1 AD Controls---------------------------------------------------

--Data Format Select.  Logic 0 is offset binary.  Logic 1 is Two's Complement
AD_DFS(0) <= ADCConCh1_Reg(1);

--Duty Cycle Stabilizer.  Logic 0 is disabled.	Logic 1 is enbled.
AD_DCS(0) <= ADCConCh1_Reg(2);
	
--FIFO Enable (active high) from global bit (0x8100 bit 5) 
CH1_WRFIFOEn <= ADCControl_Reg(5) or ADCConCh1_Reg(0);

--FIFO reset must be at least four write clock cycles to guarentee that all flags, pointers,
--and data are set to zero.  

		
process(CLK, RESET)
begin
	if(RESET = '1') then
		CH1_FIFO_RESET_REG <= '1';
	elsif(CLK'event and CLK = '1') then
		CH1_FIFO_RESET_REG <= ADCConCh1_Stb0 and LD(5);
	end if;
end process;

process(CLK)
begin
	if(CLK'event and CLK = '1') then
		CH1_FIFO_RESET_REG1 <= CH1_FIFO_RESET_REG;
	end if;
end process;

process(CLK)
begin
	if(CLK'event and CLK = '1') then
		CH1_FIFO_RESET_REG2 <= CH1_FIFO_RESET_REG1;
	end if;
end process;

process(CLK)
begin
	if(CLK'event and CLK = '1') then
		CH1_FIFO_RESET_REG3 <= CH1_FIFO_RESET_REG2;
	end if;
end process;

CH1_FIFO_RESET <= CH1_FIFO_RESET_REG or CH1_FIFO_RESET_REG1 or
		CH1_FIFO_RESET_REG2 or CH1_FIFO_RESET_REG3;

--Register Chan 1 Data Inputs
--Channel 1 IOB Register must be clocked on rising edge of ADC1 clock. 
process(AD_CLK1) 
begin
	if(AD_CLK1'event and AD_CLK1='0') then --was 0
		AD_D1_Reg <=  AD_D1;
	end if;
end process;

 --Channel 1 FIFO 
 CH1_FIFO : ADCFIFO
		port map (
			din => AD_D1_Reg,  
			rd_clk => CLK,
			rd_en => CH1_RDFIFOEN,
			rst => CH1_FIFO_RESET, 
			wr_clk => (not AD_CLK1), --
			wr_en => CH1_WRFIFOEn,
			dout => CH1_FIFO_DOUT,
			empty => CH1_Empty,
			full => CH1_Full,
			valid => CH1_Valid,
			rd_data_count => CH1_FIFOCount);


-- Threshold comparitor
  process (CLK)  
  begin
	if(CLK'event and CLK = '1') then
      if ADCThrCh1_Reg < CH1_FIFOCount then
         CH1_THESH_LIMIT <= '1';         
      else 
         CH1_THESH_LIMIT <= '0';
      end if;
	end if;
  end process; 

--Stop Conversion condition met
  CH1_Stop_Conv <= (CH1_THESH_LIMIT and ADCConCh1_Reg(10));


	
-----------CH 2 AD Controls---------------------------------------------------

--Data Format Select.  Logic 0 is offset binary.  Logic 1 is Two's Complement
AD_DFS(1) <= ADCConCh2_Reg(1);

--Duty Cycle Stabilizer.  Logic 0 is disabled.	Logic 1 is enbled.
AD_DCS(1) <= ADCConCh2_Reg(2);
	
--FIFO Enable (active high) from global bit (0x8100 bit 6)
CH2_WRFIFOEn <= ADCControl_Reg(6);

--FIFO reset must be at least four clock cycles to guarentee that all flags, pointers,
--and data are set to zero.
process(CLK, RESET)
begin
	if(RESET = '1') then
		CH2_FIFO_RESET_REG <= '1';
	elsif(CLK'event and CLK = '1') then
		CH2_FIFO_RESET_REG <= ADCConCh2_Stb2 and LD(21);
	end if;
end process;

process(CLK)
begin
	if(CLK'event and CLK = '0') then
		CH2_FIFO_RESET_REG1 <= CH2_FIFO_RESET_REG;
	end if;
end process;

process(CLK)
begin
	if(CLK'event and CLK = '1') then
		CH2_FIFO_RESET_REG2 <= CH2_FIFO_RESET_REG1;
	end if;
end process;

process(CLK)
begin
	if(CLK'event and CLK = '1') then
		CH2_FIFO_RESET_REG3 <= CH2_FIFO_RESET_REG2;
	end if;
end process;

CH2_FIFO_RESET <= CH2_FIFO_RESET_REG or CH2_FIFO_RESET_REG1 or
		CH2_FIFO_RESET_REG2 or CH2_FIFO_RESET_REG3;

  
 CH2_FIFO : ADCFIFO
		port map (
			din => AD_D2_Reg,
			rd_clk => CLK,
			rd_en => CH2_RDFIFOEN,
			rst => CH2_FIFO_RESET,
			wr_clk => (not AD_CLK2), 
			wr_en => CH2_WRFIFOEN,
			dout => CH2_FIFO_DOUT,
			empty => CH2_Empty,
			full => CH2_Full,
			valid => CH2_Valid,
			rd_data_count => CH2_FIFOCount);
 


-- Threshold comparitor
  process (CLK)  
  begin
	if(CLK'event and CLK = '0') then
      if ADCThrCh2_Reg < CH2_FIFOCount then
         CH2_THESH_LIMIT <= '1';         
      else 
         CH2_THESH_LIMIT <= '0';
      end if;
	end if;
  end process; 

--Stop Conversion condition met
  CH2_Stop_Conv <= (CH2_THESH_LIMIT and ADCConCh2_Reg(10));

--Register Chan 2 Data Inputs
--Channel 2 IOB Register must be clocked on falling edge of ADC2 clock. 
process(AD_CLK2)  
begin
	
	if(AD_CLK2'event and AD_CLK2='0') then 
		AD_D2_Reg <=  AD_D2;
	end if;
end process;

--GPIO------------------------------------------------
GPIO_Out <= GPIO_Reg;
GPIOOut_En <= not ADCControl_Reg(1) and ADCControl_Reg(0);

process(GPIO_Out, GPIOOut_En)
begin
	if (GPIOOut_En = '1') then
		GPIO <= GPIO_Out;
		--GPIO <= FPGACLKIN;  For Testing purposes only
	else
		GPIO <= 'Z';
	end if;
end process;

--ExtInput_En is active low
ExtInput_En <= not  (not ADCControl_Reg(1) and not ADCControl_Reg(0));

--ExtOutput_En is active high
ExtOutput_En <=     (not ADCControl_Reg(1) and ADCControl_Reg(0));

--ExtClkOut_En is active low
ExtClkOut_En <= not (ADCControl_Reg(1) and not ADCControl_Reg(0));

--Clock Control---------------------------------------------------
 
 Crystal_En <= not ADCControl_Reg(4) and not ADCControl_Reg(3);
 FPGACLKIn_En_Buf <= not ADCControl_Reg(4) and ADCControl_Reg(3);
 ExtCLKIn_En <= ADCControl_Reg(4) and not ADCControl_Reg(3);

FPGACLKIn_En <= FPGACLKIn_En_Buf;
 ------------------------LMK03000C Logic---------------------------------

--Serial Clock generation. Clock must be less then 20MHz.
--15MHz in S6 example design

SerCLK <= AXM_SERCLK;
--		
--process(SerCLK, SerCLK_En)
--begin
--	if (SerCLK_EN = '0') then
--		SerCLKOut <= '0';  --Clock out to LMK03000C
--	else
--		SerCLKOut <= SerCLK;
--	end if;
--end process;

-- SerCLKOut is the clock used to program the LMK03000C. Due to Spartan 6 global clock routing
-- restrictions, the best approach for generating an external clock with a minimum of added
-- skew due to routing is to use a DDR output component.  One Input of the DDR is fixed to logic '1'
-- and the other is fixed to logic '0'. Then the clock and its inverse in fed from the PLL using global
-- clock resources.  Then on the rising edge of the generated clock the part will output 1 to the pin and on the
-- falling edge the component will output 0 to the pin. While unconventional this method is required 
-- to minimuize skew between the internal clock and to meeting timing requirements.    		
ODDR2_inst_SerCLK : ODDR2
generic map(
DDR_ALIGNMENT => "NONE", -- Sets output alignment to "NONE", "C0", "C1"
INIT => '0', -- Sets initial state of the Q output to '0' or '1'
SRTYPE => "SYNC") -- Specifies "SYNC" or "ASYNC" set/reset
port map (
Q => SerCLKOut, -- 1-bit output data
C0 => AXM_SERCLK, -- 1-bit clock input
C1 => not AXM_SERCLK, -- 1-bit clock input
CE => SerCLK_EN, -- 1-bit clock enable input
D0 => '1', -- 1-bit data input (associated with C0)
D1 => '0', -- 1-bit data input (associated with C1)
R =>'0', -- 1-bit reset input
S => '0' -- 1-bit set input
);

SerCLK_En <= SerEn or Ser_Latch;
 
--Parallel to Serial Conversion

--Serial bit Counter 0 to 31.  Counter will rollover back to 0.
process(SerCLK, RESET)
begin
	if(RESET = '1') then
		   SerCnt <= "00000";
	elsif(SerCLK'event and SerCLK = '0') then
		if(Ser_Latch = '1') then
			SerCnt <= SerCnt + 1;
		else
			SerCnt <= SerCnt;
		end if;
	end if;
end process;

--Counter decode 

SerCnt0 <=  not SerCnt(4) and not SerCnt(3) and not SerCnt(2) and not SerCnt(1) and not SerCnt(0);
SerCnt1 <=  not SerCnt(4) and not SerCnt(3) and not SerCnt(2) and not SerCnt(1) and     SerCnt(0);
SerCnt2 <=  not SerCnt(4) and not SerCnt(3) and not SerCnt(2) and     SerCnt(1) and not SerCnt(0);
SerCnt3 <=  not SerCnt(4) and not SerCnt(3) and not SerCnt(2) and     SerCnt(1) and     SerCnt(0);
SerCnt4 <=  not SerCnt(4) and not SerCnt(3) and     SerCnt(2) and not SerCnt(1) and not SerCnt(0);
SerCnt5 <=  not SerCnt(4) and not SerCnt(3) and     SerCnt(2) and not SerCnt(1) and     SerCnt(0);
SerCnt6 <=  not SerCnt(4) and not SerCnt(3) and     SerCnt(2) and     SerCnt(1) and not SerCnt(0);
SerCnt7 <=  not SerCnt(4) and not SerCnt(3) and     SerCnt(2) and     SerCnt(1) and     SerCnt(0);
SerCnt8 <=  not SerCnt(4) and     SerCnt(3) and not SerCnt(2) and not SerCnt(1) and not SerCnt(0);
SerCnt9 <=  not SerCnt(4) and     SerCnt(3) and not SerCnt(2) and not SerCnt(1) and     SerCnt(0);
SerCnt10 <=  not SerCnt(4) and     SerCnt(3) and not SerCnt(2) and     SerCnt(1) and not SerCnt(0);
SerCnt11 <=  not SerCnt(4) and     SerCnt(3) and not SerCnt(2) and     SerCnt(1) and     SerCnt(0);
SerCnt12 <=  not SerCnt(4) and     SerCnt(3) and     SerCnt(2) and not SerCnt(1) and not SerCnt(0);
SerCnt13 <=  not SerCnt(4) and     SerCnt(3) and     SerCnt(2) and not SerCnt(1) and     SerCnt(0);
SerCnt14 <=  not SerCnt(4) and     SerCnt(3) and     SerCnt(2) and     SerCnt(1) and not SerCnt(0);
SerCnt15 <=  not SerCnt(4) and     SerCnt(3) and     SerCnt(2) and     SerCnt(1) and     SerCnt(0);					

SerCnt16 <=      SerCnt(4) and not SerCnt(3) and not SerCnt(2) and not SerCnt(1) and not SerCnt(0);
SerCnt17 <=      SerCnt(4) and not SerCnt(3) and not SerCnt(2) and not SerCnt(1) and     SerCnt(0);
SerCnt18 <=      SerCnt(4) and not SerCnt(3) and not SerCnt(2) and     SerCnt(1) and not SerCnt(0);
SerCnt19 <=      SerCnt(4) and not SerCnt(3) and not SerCnt(2) and     SerCnt(1) and     SerCnt(0);
SerCnt20 <=      SerCnt(4) and not SerCnt(3) and     SerCnt(2) and not SerCnt(1) and not SerCnt(0);
SerCnt21 <=      SerCnt(4) and not SerCnt(3) and     SerCnt(2) and not SerCnt(1) and     SerCnt(0);
SerCnt22 <=      SerCnt(4) and not SerCnt(3) and     SerCnt(2) and     SerCnt(1) and not SerCnt(0);
SerCnt23 <=      SerCnt(4) and not SerCnt(3) and     SerCnt(2) and     SerCnt(1) and     SerCnt(0);
SerCnt24 <=      SerCnt(4) and     SerCnt(3) and not SerCnt(2) and not SerCnt(1) and not SerCnt(0);
SerCnt25 <=      SerCnt(4) and     SerCnt(3) and not SerCnt(2) and not SerCnt(1) and     SerCnt(0);
SerCnt26 <=      SerCnt(4) and     SerCnt(3) and not SerCnt(2) and     SerCnt(1) and not SerCnt(0);
SerCnt27 <=      SerCnt(4) and     SerCnt(3) and not SerCnt(2) and     SerCnt(1) and     SerCnt(0);
SerCnt28 <=      SerCnt(4) and     SerCnt(3) and     SerCnt(2) and not SerCnt(1) and not SerCnt(0);
SerCnt29 <=      SerCnt(4) and     SerCnt(3) and     SerCnt(2) and not SerCnt(1) and     SerCnt(0);
SerCnt30 <=      SerCnt(4) and     SerCnt(3) and     SerCnt(2) and     SerCnt(1) and not SerCnt(0);
SerCnt31 <=      SerCnt(4) and     SerCnt(3) and     SerCnt(2) and     SerCnt(1) and     SerCnt(0);							
			
--Serial Data is sent MSB first.
SerOut <= (SerCnt0 and SerData(31)) or (SerCnt1 and SerData(30)) or
          (SerCnt2 and SerData(29)) or (SerCnt3 and SerData(28)) or
          (SerCnt4 and SerData(27)) or (SerCnt5 and SerData(26)) or
          (SerCnt6 and SerData(25)) or (SerCnt7 and SerData(24)) or
			 (SerCnt8 and SerData(23)) or (SerCnt9 and SerData(22)) or
          (SerCnt10 and SerData(21)) or (SerCnt11 and SerData(20)) or
          (SerCnt12 and SerData(19)) or (SerCnt13 and SerData(18)) or
          (SerCnt14 and SerData(17)) or (SerCnt15 and SerData(16)) or
		    (SerCnt16 and SerData(15)) or (SerCnt17 and SerData(14)) or
          (SerCnt18 and SerData(13)) or (SerCnt19 and SerData(12)) or
          (SerCnt20 and SerData(11)) or (SerCnt21 and SerData(10)) or
			 (SerCnt22 and SerData(9)) or (SerCnt23 and SerData(8)) or
          (SerCnt24 and SerData(7)) or (SerCnt25 and SerData(6)) or
          (SerCnt26 and SerData(5)) or (SerCnt27 and SerData(4)) or
          (SerCnt28 and SerData(3)) or (SerCnt29 and SerData(2)) or
          (SerCnt30 and SerData(1)) or (SerCnt31 and SerData(0));

SerDataOut <= SerOut;


--SerEn is active high during serial programming.
SerEn <= Write_R0 or Write_R1 or Write_R3 or Write_R4 or
			Write_R11 or Write_R13 or Write_R14 or Write_R15;

DataMux(0) <= Write_R0 or Write_R0_HOLD;
DataMux(1) <= Write_R1 or Write_R1_HOLD;
DataMux(2) <= Write_R3 or Write_R3_HOLD;
DataMux(3) <= Write_R4 or Write_R4_HOLD;
DataMux(4) <= Write_R11 or Write_R11_HOLD;
DataMux(5) <= Write_R13 or Write_R13_HOLD;
DataMux(6) <= Write_R14 or Write_R14_HOLD;
DataMux(7) <= Write_R15 or Write_R15_HOLD;




--This process is a multiplexar that determines which register's data
--is used for serial programming.
process(DataMux, PLLR0_Reg, PLLR1_Reg, PLLR3_Reg, PLLR4_Reg,
			PLLR11_Reg, PLLR13_Reg, PLLR14_Reg, PLLR15_Reg)
begin
	case DataMux is 
		when "00000001" =>
			SerData <= PLLR0_Reg;
		when "00000010" =>
			SerData <= PLLR1_Reg;
		when "00000100" =>
			SerData <= PLLR3_Reg;
		when "00001000" =>
			SerData <= PLLR4_Reg;
		when "00010000" =>
			SerData <= PLLR11_Reg;
		when "00100000" =>
			SerData <= PLLR13_Reg;
		when "01000000" =>
			SerData <= PLLR14_Reg;
		when "10000000" =>
			SerData <= PLLR15_Reg;
		when others =>
			SerData <= "00000000000000000000000000000000";
		end case;
end process;
			
--The following are individual Write holds.  Each time a LMK0300C Register is
--writen to, the correcspong write signal will be active and remain that way until
--serial programming of the single register is complete.			
process(CLK, RESET)
begin
	if(RESET = '1') then
		Write_R0 <= '0';
	elsif (CLK'event and CLK = '1') then
		Write_R0 <= (PLLR0_StbAll and not SerEn) or (Write_R0 and not SerCnt31);-- or
						--Write_R0 and Ser_Latch);
	end if;
end process;

process(CLK, RESET)
begin
	if(RESET = '1') then
		Write_R1 <= '0';
	elsif (CLK'event and CLK = '1') then
		Write_R1 <= (PLLR1_StbAll and not SerEn) or (Write_R1 and not SerCnt31);
	end if;
end process;

process(CLK, RESET)
begin
	if(RESET = '1') then
		Write_R3 <= '0';
	elsif (CLK'event and CLK = '1') then
		Write_R3 <= (PLLR3_StbAll and not SerEn) or (Write_R3 and not SerCnt31);
	end if;
end process;

process(CLK, RESET)
begin
	if(RESET = '1') then
		Write_R4 <= '0';
	elsif (CLK'event and CLK = '1') then
		Write_R4 <= (PLLR4_StbAll and not SerEn) or (Write_R4 and not SerCnt31);
	end if;
end process;

process(CLK, RESET)
begin
	if(RESET = '1') then
		Write_R11 <= '0';
	elsif (CLK'event and CLK = '1') then
		Write_R11 <= (PLLR11_StbAll and not SerEn) or (Write_R11 and not SerCnt31);
	end if;
end process;

process(CLK, RESET)
begin
	if(RESET = '1') then
		Write_R13 <= '0';
	elsif (CLK'event and CLK = '1') then
		Write_R13 <= (PLLR13_StbAll and not SerEn) or (Write_R13 and not SerCnt31) ;
	end if;
end process;

process(CLK, RESET)
begin
	if(RESET = '1') then
		Write_R14 <= '0';
	elsif (CLK'event and CLK = '1') then
		Write_R14 <= (PLLR14_StbAll and not SerEn) or (Write_R14 and not SerCnt31);
	end if;
end process;

process(CLK, RESET)
begin
	if(RESET = '1') then
		Write_R15 <= '0';
	elsif (CLK'event and CLK = '1') then
		Write_R15 <= (PLLR15_StbAll and not SerEn) or (Write_R15 and not SerCnt31);
	end if;
end process;

--Write holds registers used to hold the data output active when SerCnt = 31.
-- This is required due to differences in the clock domain between the Write_Rxx
-- signals (CLK) and the Data Output that uses SerCLK (CLK/8)

process(CLK)
begin
	if (CLK'event and CLK = '1') then
		Write_R0_HOLD <= Write_R0 or (Write_R0_HOLD and SerCnt31);
	end if;
end process;

process(CLK)
begin
	if (CLK'event and CLK = '1') then
		Write_R1_HOLD <= Write_R1 or (Write_R1_HOLD and SerCnt31);
	end if;
end process;

process(CLK)
begin
	if (CLK'event and CLK = '1') then
		Write_R3_HOLD <= Write_R3 or (Write_R3_HOLD and SerCnt31);
	end if;
end process;

process(CLK)
begin
	if (CLK'event and CLK = '1') then
		Write_R4_HOLD <= Write_R4 or (Write_R4_HOLD and SerCnt31);
	end if;
end process;


process(CLK)
begin
	if (CLK'event and CLK = '1') then
		Write_R11_HOLD <= Write_R11 or (Write_R11_HOLD and SerCnt31);
	end if;
end process;

process(CLK)
begin
	if (CLK'event and CLK = '1') then
		Write_R13_HOLD <= Write_R13 or (Write_R13_HOLD and SerCnt31);
	end if;
end process;

process(CLK)
begin
	if (CLK'event and CLK = '1') then
		Write_R14_HOLD <= Write_R14 or (Write_R14_HOLD and SerCnt31);
	end if;
end process;

process(CLK)
begin
	if (CLK'event and CLK = '1') then
		Write_R15_HOLD <= Write_R15 or (Write_R15_HOLD and SerCnt31);
	end if;
end process;

--Serial Data Output Enable
process(SerCLK, RESET)
begin
	if(RESET = '1') then
		Ser_Latch <= '0';
	elsif(SerCLK'event and SerCLK = '0') then
		Ser_Latch <= SerEn or (Ser_Latch and not SerCnt31);
	end if;
end process;

SerLatch_n <= not Ser_Latch;


--Generate Clock Conditioner Reset Signal
-- Must reset all Clock Conditioner Registers if and only if logic 1 has been written
-- to bit 31 of R0 and programming has completed.

process(CLK)
begin
	if(CLK'event and CLK = '1') then
		Write_R0_Reg <= Write_R0;
	end if;
end process;

--Detect when Write_R0 goes low.
Write_R0_LowEvent <= not Write_R0 and Write_R0_Reg;

--Clock Conditioner Register Reset will be active for only one clock cycle.
process(CLK)
begin
	if(CLK'event and CLK = '1') then
		CLKCON_RESET <= Write_R0_LowEvent and PLLR0_Reg(31);
	end if;
end process;



--Generate Clock Generator Synch Pulse.  Pulse must be low for 2us to allow for 
-- slowest output clock of 1MHz.  Use CLK/8 with Max Freq of 16.25MHz.

process(SerCLK, RESET)
begin
	if (RESET = '1') then
		Synch_Counter <= "00000";
	elsif (SerCLK'event and SerCLK = '0') then
		if (Synch_En = '1') then
			Synch_Counter <= Synch_Counter + 1;
		else
			Synch_Counter <= Synch_Counter;
		end if;
	end if;
end process;

Synch_Complete <= Synch_Counter(0) and Synch_Counter(1) and Synch_Counter(2) and
						Synch_Counter(3) and Synch_Counter(4);
						
						
process(CLK, RESET)
begin
	if (RESET = '1') then
		Synch_EN <= '0';
	elsif(CLK'event and CLK = '1') then
		Synch_En <= (PLLControl_Stb0 and LD(5)) or (Synch_En and not Synch_Complete);
	end if;
end process;


LMK_SYNCH <= not Synch_En;
LMK_GOE <= PLLGOE;


--FPGA clock out to input of LMK03000C.  This clock is very noisy and should only
--be used for demos or proof of concept.
--15MHz in S6.

FPGACLK_OUT_BUF <= AXM_SERCLK;

process(FPGACLKIn_En_Buf, FPGACLK_OUT_BUF)
begin
	if(FPGACLKIn_En_Buf = '1') then
		FPGACLK_OUT <= FPGACLK_OUT_BUF;
	else
		FPGACLK_OUT <= 'Z';
	end if;
end process;


------ Interrupt Logic ----------------------------------------------------------

-- ADC Interrupt Pending bit
-- A pending ADC interrupt will remain active until ADC interrupts are disabled
-- via bit-11 of the ADC control register. Also reading the FIFO until it
-- has less data than the set threshold will remove the interrupt.

  ADCh1_Interrupt_Pending <= ADCConCh1_Reg(11) and CH1_THESH_LIMIT;
  ADCh2_Interrupt_Pending <= ADCConCh2_Reg(11) and CH2_THESH_LIMIT;

  --Interrupt Pending Flags
  Int_Status(0) <= ADCh1_Interrupt_Pending;
  Int_Status(1) <= ADCh2_Interrupt_Pending;
  
  

------ AXM-A30 Registers Readback Mux -------------------------------------------
 
  Read_Data(0) <=  (ADCControl_Reg(0) and ADCControl_Adr) or
						(ADCConCh1_Reg(0) and ADCCh1_Adr) or
						
						(ADCThrCh1_Reg(0) and ADCCh2_Adr) or
						(CH1_FIFO_DOUT(0) and ADCFifo_Adr) or
						(SerEn and PLLControl_Adr) or
						(PLLR0_Reg(0) and PLLR0_Adr) or
						(PLLR1_Reg(0) and PLLR1_Adr) or
						(PLLR3_Reg(0) and PLLR3_Adr) or
						(PLLR4_Reg(0) and PLLR4_Adr) or
						(PLLR11_Reg(0) and PLLR11_Adr) or
						(PLLR13_Reg(0) and PLLR13_Adr) or
						(PLLR14_Reg(0) and PLLR14_Adr) or
						(PLLR15_Reg(0) and PLLR15_Adr) or
						(ADC_SRAM_Reg(0) and ADC_SRAM_Adr);

  Read_Data(1) <=  (ADCControl_Reg(1) and ADCControl_Adr) or
						(ADCConCh1_Reg(1) and ADCCh1_Adr) or
						
						(ADCThrCh1_Reg(1) and ADCCh2_Adr) or
						(CH1_FIFO_DOUT(1) and ADCFifo_Adr) or
						(LOCK and PLLControl_Adr) or
						(PLLR0_Reg(1) and PLLR0_Adr) or
						(PLLR1_Reg(1) and PLLR1_Adr) or
						(PLLR3_Reg(1) and PLLR3_Adr) or
						(PLLR4_Reg(1) and PLLR4_Adr) or
						(PLLR11_Reg(1) and PLLR11_Adr) or
						(PLLR13_Reg(1) and PLLR13_Adr) or
						(PLLR14_Reg(1) and PLLR14_Adr) or
						(PLLR15_Reg(1) and PLLR15_Adr);
  
  Read_Data(2) <=   --(GPIO and ADCControl_Adr) or
						(ADCConCh1_Reg(2) and ADCCh1_Adr) or
						
						(ADCThrCh1_Reg(2) and ADCCh2_Adr) or
						(CH1_FIFO_DOUT(2) and ADCFifo_Adr)  or
						(PLLR0_Reg(2) and PLLR0_Adr) or
						(PLLR1_Reg(2) and PLLR1_Adr) or
						(PLLR3_Reg(2) and PLLR3_Adr) or
						(PLLR4_Reg(2) and PLLR4_Adr) or
						(PLLR11_Reg(2) and PLLR11_Adr) or
						(PLLR13_Reg(2) and PLLR13_Adr) or
						(PLLR14_Reg(2) and PLLR14_Adr) or
						(PLLR15_Reg(2) and PLLR15_Adr);
  
  Read_Data(3) <=  (ADCControl_Reg(3) and ADCControl_Adr) or
						
						(ADCThrCh1_Reg(3) and ADCCh2_Adr) or
						(CH1_FIFO_DOUT(3) and ADCFifo_Adr) or
						(PLLR0_Reg(3) and PLLR0_Adr) or
						(PLLR1_Reg(3) and PLLR1_Adr) or
						(PLLR3_Reg(3) and PLLR3_Adr) or
						(PLLR4_Reg(3) and PLLR4_Adr) or
						(PLLR11_Reg(3) and PLLR11_Adr) or
						(PLLR13_Reg(3) and PLLR13_Adr) or
						(PLLR14_Reg(3) and PLLR14_Adr) or
						(PLLR15_Reg(3) and PLLR15_Adr);
  
  Read_Data(4) <=  (ADCControl_Reg(4) and ADCControl_Adr) or
						
						(ADCThrCh1_Reg(4) and ADCCh2_Adr) or
						(CH1_FIFO_DOUT(4) and ADCFifo_Adr) or
						(PLLGOE and PLLControl_Adr) or
						(PLLR0_Reg(4) and PLLR0_Adr) or
						(PLLR1_Reg(4) and PLLR1_Adr) or
						(PLLR3_Reg(4) and PLLR3_Adr) or
						(PLLR4_Reg(4) and PLLR4_Adr) or
						(PLLR11_Reg(4) and PLLR11_Adr) or
						(PLLR13_Reg(4) and PLLR13_Adr) or
						(PLLR14_Reg(4) and PLLR14_Adr) or
						(PLLR15_Reg(4) and PLLR15_Adr);
  
  Read_Data(5) <= (ADCControl_Reg(5) and ADCControl_Adr) or
						
						(ADCThrCh1_Reg(5) and ADCCh2_Adr) or
						(CH1_FIFO_DOUT(5) and ADCFifo_Adr) or
						(PLLR0_Reg(5) and PLLR0_Adr) or
						(PLLR1_Reg(5) and PLLR1_Adr) or
						(PLLR3_Reg(5) and PLLR3_Adr) or
						(PLLR4_Reg(5) and PLLR4_Adr) or
						(PLLR11_Reg(5) and PLLR11_Adr) or
						(PLLR13_Reg(5) and PLLR13_Adr) or
						(PLLR14_Reg(5) and PLLR14_Adr) or
						(PLLR15_Reg(5) and PLLR15_Adr);
  
  Read_Data(6) <=  (ADCControl_Reg(6) and ADCControl_Adr) or
						
						(ADCThrCh1_Reg(6) and ADCCh2_Adr) or
						(CH1_FIFO_DOUT(6) and ADCFifo_Adr) or
						(PLLR0_Reg(6) and PLLR0_Adr) or
						(PLLR1_Reg(6) and PLLR1_Adr) or
						(PLLR3_Reg(6) and PLLR3_Adr) or
						(PLLR4_Reg(6) and PLLR4_Adr) or
						(PLLR11_Reg(6) and PLLR11_Adr) or
						(PLLR13_Reg(6) and PLLR13_Adr) or
						(PLLR14_Reg(6) and PLLR14_Adr) or
						(PLLR15_Reg(6) and PLLR15_Adr);
  
  Read_Data(7) <= (CH1_FIFO_DOUT(7) and ADCFifo_Adr) or
						
						(ADCThrCh1_Reg(7) and ADCCh2_Adr) or
						(PLLR0_Reg(7) and PLLR0_Adr) or
						(PLLR1_Reg(7) and PLLR1_Adr) or
						(PLLR3_Reg(7) and PLLR3_Adr) or
						(PLLR4_Reg(7) and PLLR4_Adr) or
						(PLLR11_Reg(7) and PLLR11_Adr) or
						(PLLR13_Reg(7) and PLLR13_Adr) or
						(PLLR14_Reg(7) and PLLR14_Adr) or
						(PLLR15_Reg(7) and PLLR15_Adr);
  
  Read_Data(8) <= (PDN_Reg(0) and ADCControl_Adr) or 
						(Reg_AD_OTR(0) and ADCCh1_Adr) or
						
						(ADCThrCh1_Reg(8) and ADCCh2_Adr) or
						(CH1_FIFO_DOUT(8) and ADCFifo_Adr) or
						(PLLR0_Reg(8) and PLLR0_Adr) or
						(PLLR1_Reg(8) and PLLR1_Adr) or
						(PLLR3_Reg(8) and PLLR3_Adr) or
						(PLLR4_Reg(8) and PLLR4_Adr) or
						(PLLR11_Reg(8) and PLLR11_Adr) or
						(PLLR13_Reg(8) and PLLR13_Adr) or
						(PLLR14_Reg(8) and PLLR14_Adr) or
						(PLLR15_Reg(8) and PLLR15_Adr) or
						(ADC_SRAM_Reg(8) and ADC_SRAM_Adr);
  
  Read_Data(9) <= (PDN_Reg(1) and ADCControl_Adr) or  
						
						(ADCThrCh1_Reg(9) and ADCCh2_Adr) or 
						(CH1_FIFO_DOUT(9) and ADCFifo_Adr) or
						(CH1_FIFO_DOUT(9) and ADCFifo_Adr) or
						(PLLR0_Reg(9) and PLLR0_Adr) or
						(PLLR1_Reg(9) and PLLR1_Adr) or
						(PLLR3_Reg(9) and PLLR3_Adr) or
						(PLLR4_Reg(9) and PLLR4_Adr) or
						(PLLR11_Reg(9) and PLLR11_Adr) or
						(PLLR13_Reg(9) and PLLR13_Adr) or
						(PLLR14_Reg(9) and PLLR14_Adr) or
						(PLLR15_Reg(9) and PLLR15_Adr) or
						(ADC_SRAM_Reg(9) and ADC_SRAM_Adr);
  
  Read_Data(10) <=(FPGACLKIN_TRANS_EN and ADCControl_Adr) or
				
						(ADCConCh1_Reg(10) and ADCCh1_Adr) or
						(ADCThrCh1_Reg(10) and ADCCh2_Adr) or
						(CH1_FIFO_DOUT(10) and ADCFifo_Adr) or
						(PLLR0_Reg(10) and PLLR0_Adr) or
						(PLLR1_Reg(10) and PLLR1_Adr) or
						(PLLR3_Reg(10) and PLLR3_Adr) or
						(PLLR4_Reg(10) and PLLR4_Adr) or
						(PLLR11_Reg(10) and PLLR11_Adr) or
						(PLLR13_Reg(10) and PLLR13_Adr) or
						(PLLR14_Reg(10) and PLLR14_Adr) or
						(PLLR15_Reg(10) and PLLR15_Adr);
  
  Read_Data(11) <= (ADCConCh1_Reg(11) and ADCCh1_Adr) or
						
						(ADCThrCh1_Reg(11) and ADCCh2_Adr) or
						(CH1_FIFO_DOUT(11) and ADCFifo_Adr) or
						(PLLR0_Reg(11) and PLLR0_Adr) or
						(PLLR1_Reg(11) and PLLR1_Adr) or
						(PLLR3_Reg(11) and PLLR3_Adr) or
						(PLLR4_Reg(11) and PLLR4_Adr) or
						(PLLR11_Reg(11) and PLLR11_Adr) or
						(PLLR13_Reg(11) and PLLR13_Adr) or
						(PLLR14_Reg(11) and PLLR14_Adr) or
						(PLLR15_Reg(11) and PLLR15_Adr);
  
  Read_Data(12) <= (ADCh1_Interrupt_Pending and ADCCh1_Adr) or
						
						(ADCThrCh1_Reg(12) and ADCCh2_Adr) or
						(CH1_FIFO_DOUT(12) and ADCFifo_Adr) or
						(PLLR0_Reg(12) and PLLR0_Adr) or
						(PLLR1_Reg(12) and PLLR1_Adr) or
						(PLLR3_Reg(12) and PLLR3_Adr) or
						(PLLR4_Reg(12) and PLLR4_Adr) or
						(PLLR11_Reg(12) and PLLR11_Adr) or
						(PLLR13_Reg(12) and PLLR13_Adr) or
						(PLLR14_Reg(12) and PLLR14_Adr) or
						(PLLR15_Reg(12) and PLLR15_Adr);
  
  Read_Data(13) <= (CH1_Empty and ADCCh1_Adr) or	
						
						(CH1_FIFO_DOUT(13) and ADCFifo_Adr) or
						(PLLR0_Reg(13) and PLLR0_Adr) or
						(PLLR1_Reg(13) and PLLR1_Adr) or
						(PLLR3_Reg(13) and PLLR3_Adr) or
						(PLLR4_Reg(13) and PLLR4_Adr) or
						(PLLR11_Reg(13) and PLLR11_Adr) or
						(PLLR13_Reg(13) and PLLR13_Adr) or
						(PLLR14_Reg(13) and PLLR14_Adr) or
						(PLLR15_Reg(13) and PLLR15_Adr);
  
  Read_Data(14) <= (CH1_THESH_LIMIT and ADCCh1_Adr) or
						
						(CH1_FIFO_DOUT(14) and ADCFifo_Adr) or
						(PLLR0_Reg(14) and PLLR0_Adr) or
						(PLLR1_Reg(14) and PLLR1_Adr) or
						(PLLR3_Reg(14) and PLLR3_Adr) or
						(PLLR4_Reg(14) and PLLR4_Adr) or
						(PLLR11_Reg(14) and PLLR11_Adr) or
						(PLLR13_Reg(14) and PLLR13_Adr) or
						(PLLR14_Reg(14) and PLLR14_Adr) or
						(PLLR15_Reg(14) and PLLR15_Adr);
  
  Read_Data(15) <= (CH1_Full and ADCCh1_Adr) or
						
						(CH1_FIFO_DOUT(15) and ADCFifo_Adr) or
						(PLLR0_Reg(15) and PLLR0_Adr) or
						(PLLR1_Reg(15) and PLLR1_Adr) or
						(PLLR3_Reg(15) and PLLR3_Adr) or
						(PLLR4_Reg(15) and PLLR4_Adr) or
						(PLLR11_Reg(15) and PLLR11_Adr) or
						(PLLR13_Reg(15) and PLLR13_Adr) or
						(PLLR14_Reg(15) and PLLR14_Adr) or
						(PLLR15_Reg(15) and PLLR15_Adr);
  
  Read_Data(16) <= (GPIO and ADCControl_Adr) or 
						(ADCConCh2_Reg(0) and ADCCh1_Adr) or
						(ADCThrCh2_Reg(0) and ADCCh2_Adr) or
						(CH2_FIFO_DOUT(0) and ADCFifo_Adr) or
						(PLLR0_Reg(16) and PLLR0_Adr) or
						(PLLR1_Reg(16) and PLLR1_Adr) or
						(PLLR3_Reg(16) and PLLR3_Adr) or
						(PLLR4_Reg(16) and PLLR4_Adr) or
						(PLLR11_Reg(16) and PLLR11_Adr) or
						(PLLR13_Reg(16) and PLLR13_Adr) or
						(PLLR14_Reg(16) and PLLR14_Adr) or
						(PLLR15_Reg(16) and PLLR15_Adr);
	
  Read_Data(17) <= 
						(ADCConCh2_Reg(1) and ADCCh1_Adr) or 
						
						(ADCThrCh2_Reg(1) and ADCCh2_Adr) or
						(CH2_FIFO_DOUT(1) and ADCFifo_Adr) or
						(PLLR0_Reg(17) and PLLR0_Adr) or
						(PLLR1_Reg(17) and PLLR1_Adr) or
						(PLLR3_Reg(17) and PLLR3_Adr) or
						(PLLR4_Reg(17) and PLLR4_Adr) or
						(PLLR11_Reg(17) and PLLR11_Adr) or
						(PLLR13_Reg(17) and PLLR13_Adr) or
						(PLLR14_Reg(17) and PLLR14_Adr) or
						(PLLR15_Reg(17) and PLLR15_Adr);
  
  Read_Data(18) <=
						(ADCConCh2_Reg(2) and ADCCh1_Adr) or
						
						(ADCThrCh2_Reg(2) and ADCCh2_Adr) or
						(CH2_FIFO_DOUT(2) and ADCFifo_Adr) or
						(PLLR0_Reg(18) and PLLR0_Adr) or
						(PLLR1_Reg(18) and PLLR1_Adr) or
						(PLLR3_Reg(18) and PLLR3_Adr) or
						(PLLR4_Reg(18) and PLLR4_Adr) or
						(PLLR11_Reg(18) and PLLR11_Adr) or
						(PLLR13_Reg(18) and PLLR13_Adr) or
						(PLLR14_Reg(18) and PLLR14_Adr) or
						(PLLR15_Reg(18) and PLLR15_Adr);
  
  Read_Data(19) <= 
						(ADCThrCh2_Reg(3) and ADCCh2_Adr) or
						(CH2_FIFO_DOUT(3) and ADCFifo_Adr) or
						(PLLR0_Reg(19) and PLLR0_Adr) or
						(PLLR1_Reg(19) and PLLR1_Adr) or
						(PLLR3_Reg(19) and PLLR3_Adr) or
						(PLLR4_Reg(19) and PLLR4_Adr) or
						(PLLR11_Reg(19) and PLLR11_Adr) or
						(PLLR13_Reg(19) and PLLR13_Adr) or
						(PLLR14_Reg(19) and PLLR14_Adr) or
						(PLLR15_Reg(19) and PLLR15_Adr);
  
  Read_Data(20) <= 
						(ADCThrCh2_Reg(4) and ADCCh2_Adr) or
						(CH2_FIFO_DOUT(4) and ADCFifo_Adr) or
						(PLLR0_Reg(20) and PLLR0_Adr) or
						(PLLR1_Reg(20) and PLLR1_Adr) or
						(PLLR3_Reg(20) and PLLR3_Adr) or
						(PLLR4_Reg(20) and PLLR4_Adr) or
						(PLLR11_Reg(20) and PLLR11_Adr) or
						(PLLR13_Reg(20) and PLLR13_Adr) or
						(PLLR14_Reg(20) and PLLR14_Adr) or
						(PLLR15_Reg(20) and PLLR15_Adr);
  
  Read_Data(21) <= 
						(ADCThrCh2_Reg(5) and ADCCh2_Adr) or
						(CH2_FIFO_DOUT(5) and ADCFifo_Adr) or
						(PLLR0_Reg(21) and PLLR0_Adr) or
						(PLLR1_Reg(21) and PLLR1_Adr) or
						(PLLR3_Reg(21) and PLLR3_Adr) or
						(PLLR4_Reg(21) and PLLR4_Adr) or
						(PLLR11_Reg(21) and PLLR11_Adr) or
						(PLLR13_Reg(21) and PLLR13_Adr) or
						(PLLR14_Reg(21) and PLLR14_Adr) or
						(PLLR15_Reg(21) and PLLR15_Adr);
  
  Read_Data(22) <= 
						(ADCThrCh2_Reg(6) and ADCCh2_Adr) or
						(CH2_FIFO_DOUT(6) and ADCFifo_Adr) or
						(PLLR0_Reg(22) and PLLR0_Adr) or
						(PLLR1_Reg(22) and PLLR1_Adr) or
						(PLLR3_Reg(22) and PLLR3_Adr) or
						(PLLR4_Reg(22) and PLLR4_Adr) or
						(PLLR11_Reg(22) and PLLR11_Adr) or
						(PLLR13_Reg(22) and PLLR13_Adr) or
						(PLLR14_Reg(22) and PLLR14_Adr) or
						(PLLR15_Reg(22) and PLLR15_Adr);
  
  Read_Data(23) <= 
						(ADCThrCh2_Reg(7) and ADCCh2_Adr) or
						(CH2_FIFO_DOUT(7) and ADCFifo_Adr) or
						(PLLR0_Reg(23) and PLLR0_Adr) or
						(PLLR1_Reg(23) and PLLR1_Adr) or
						(PLLR3_Reg(23) and PLLR3_Adr) or
						(PLLR4_Reg(23) and PLLR4_Adr) or
						(PLLR11_Reg(23) and PLLR11_Adr) or
						(PLLR13_Reg(23) and PLLR13_Adr) or
						(PLLR14_Reg(23) and PLLR14_Adr) or
						(PLLR15_Reg(23) and PLLR15_Adr);
  
  Read_Data(24) <= (Reg_AD_OTR(1) and ADCCh1_Adr) or 
						
						(ADCThrCh2_Reg(8) and ADCCh2_Adr) or
						(CH2_FIFO_DOUT(8) and ADCFifo_Adr) or
						(PLLR0_Reg(24) and PLLR0_Adr) or
						(PLLR1_Reg(24) and PLLR1_Adr) or
						(PLLR3_Reg(24) and PLLR3_Adr) or
						(PLLR4_Reg(24) and PLLR4_Adr) or
						(PLLR11_Reg(24) and PLLR11_Adr) or
						(PLLR13_Reg(24) and PLLR13_Adr) or
						(PLLR14_Reg(24) and PLLR14_Adr) or
						(PLLR15_Reg(24) and PLLR15_Adr);
  
  Read_Data(25) <= 
						(ADCThrCh2_Reg(9) and ADCCh2_Adr) or
						(CH2_FIFO_DOUT(9) and ADCFifo_Adr) or
						(PLLR0_Reg(25) and PLLR0_Adr) or
						(PLLR1_Reg(25) and PLLR1_Adr) or
						(PLLR3_Reg(25) and PLLR3_Adr) or
						(PLLR4_Reg(25) and PLLR4_Adr) or
						(PLLR11_Reg(25) and PLLR11_Adr) or
						(PLLR13_Reg(25) and PLLR13_Adr) or
						(PLLR14_Reg(25) and PLLR14_Adr) or
						(PLLR15_Reg(25) and PLLR15_Adr);
  
  Read_Data(26) <= (ADCConCh2_Reg(10) and ADCCh1_Adr) or
						
						(ADCThrCh2_Reg(10) and ADCCh2_Adr) or
						(CH2_FIFO_DOUT(10) and ADCFifo_Adr) or
						(PLLR0_Reg(26) and PLLR0_Adr) or
						(PLLR1_Reg(26) and PLLR1_Adr) or
						(PLLR3_Reg(26) and PLLR3_Adr) or
						(PLLR4_Reg(26) and PLLR4_Adr) or
						(PLLR11_Reg(26) and PLLR11_Adr) or
						(PLLR13_Reg(26) and PLLR13_Adr) or
						(PLLR14_Reg(26) and PLLR14_Adr) or
						(PLLR15_Reg(26) and PLLR15_Adr);
  
  Read_Data(27) <= (ADCConCh2_Reg(11) and ADCCh1_Adr) or
						
						(ADCThrCh2_Reg(11) and ADCCh2_Adr) or
						(CH2_FIFO_DOUT(11) and ADCFifo_Adr) or
						(PLLR0_Reg(27) and PLLR0_Adr) or
						(PLLR1_Reg(27) and PLLR1_Adr) or
						(PLLR3_Reg(27) and PLLR3_Adr) or
						(PLLR4_Reg(27) and PLLR4_Adr) or
						(PLLR11_Reg(27) and PLLR11_Adr) or
						(PLLR13_Reg(27) and PLLR13_Adr) or
						(PLLR14_Reg(27) and PLLR14_Adr) or
						(PLLR15_Reg(27) and PLLR15_Adr);						
  
  Read_Data(28) <= (ADCh2_Interrupt_Pending and ADCCh1_Adr) or 
						
						(ADCThrCh2_Reg(12) and ADCCh2_Adr) or
						(CH2_FIFO_DOUT(12) and ADCFifo_Adr) or
						(PLLR0_Reg(28) and PLLR0_Adr) or
						(PLLR1_Reg(28) and PLLR1_Adr) or
						(PLLR3_Reg(28) and PLLR3_Adr) or
						(PLLR4_Reg(28) and PLLR4_Adr) or
						(PLLR11_Reg(28) and PLLR11_Adr) or
						(PLLR13_Reg(28) and PLLR13_Adr) or
						(PLLR14_Reg(28) and PLLR14_Adr) or
						(PLLR15_Reg(28) and PLLR15_Adr);
  
  Read_Data(29) <= (CH2_Empty and ADCCh1_Adr) or 
						(CH2_FIFO_DOUT(13) and ADCFifo_Adr) or
						(PLLR0_Reg(29) and PLLR0_Adr) or
						(PLLR1_Reg(29) and PLLR1_Adr) or
						(PLLR3_Reg(29) and PLLR3_Adr) or
						(PLLR4_Reg(29) and PLLR4_Adr) or
						(PLLR11_Reg(29) and PLLR11_Adr) or
						(PLLR13_Reg(29) and PLLR13_Adr) or
						(PLLR14_Reg(29) and PLLR14_Adr) or
						(PLLR15_Reg(29) and PLLR15_Adr);
  
  Read_Data(30) <= (CH2_THESH_LIMIT and ADCCh1_Adr) or
						(CH2_FIFO_DOUT(14) and ADCFifo_Adr) or
						(PLLR0_Reg(30) and PLLR0_Adr) or
						(PLLR1_Reg(30) and PLLR1_Adr) or
						(PLLR3_Reg(30) and PLLR3_Adr) or
						(PLLR4_Reg(30) and PLLR4_Adr) or
						(PLLR11_Reg(30) and PLLR11_Adr) or
						(PLLR13_Reg(30) and PLLR13_Adr) or
						(PLLR14_Reg(30) and PLLR14_Adr) or
						(PLLR15_Reg(30) and PLLR15_Adr);
  
  Read_Data(31) <= (CH2_Full and ADCCh1_Adr) or 
						(CH2_FIFO_DOUT(15) and ADCFifo_Adr) or
						(PLLR0_Reg(31) and PLLR0_Adr) or
						(PLLR1_Reg(31) and PLLR1_Adr) or
						(PLLR3_Reg(31) and PLLR3_Adr) or
						(PLLR4_Reg(31) and PLLR4_Adr) or
						(PLLR11_Reg(31) and PLLR11_Adr) or
						(PLLR13_Reg(31) and PLLR13_Adr) or
						(PLLR14_Reg(31) and PLLR14_Adr) or
						(PLLR15_Reg(31) and PLLR15_Adr);
						
UnusedFrontIO <= "00000";

--SRAM Override Controls

process(ADC_SRAM_CH0_EN, CH1_FIFO_DOUT, CH2_FIFO_DOUT )
begin
	if (ADC_SRAM_CH0_EN = '1') then 
		SRAM_FIFO_DATA <= CH1_FIFO_DOUT;
	else 
		SRAM_FIFO_DATA <= CH2_FIFO_DOUT;
	end if;
end process;
		
FIFO_EMPTY_FLAG <= (CH1_Empty and ADC_SRAM_CH0_EN) or (CH2_Empty and ADC_SRAM_CH1_EN);

FIFODataValidFlag <= (CH1_Valid and ADC_SRAM_CH0_EN) or (CH2_Valid and ADC_SRAM_CH1_EN);

end  AXM_A30_arch;
