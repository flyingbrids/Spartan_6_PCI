----------------------------  XMC-Spartan 6  ------------------
---------------------------------------------------------------
--
-- XC6SLX150.vhd
--
-- Rev A  10/28/2010
--
-- Description:
-- This is the top-level of the Acromag example design. 
-- This block contains local bus logic, SRAM component,
-- front a rear I/O components, and controller component.
--
--
---------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

-- UNISIM library contains the component declarations for all
-- Xilinx primitives.
library UNISIM;
use UNISIM.VComponents.all;

entity XC6SLX150 is
   port (
    spare1: in STD_LOGIC;
    spare2: in STD_LOGIC;
    spare3: in STD_LOGIC;
	CLK_PCI_U7: in STD_LOGIC; -- PCIx clock from U5 PCIx interface FPGA
	FPGA_CLK: in STD_LOGIC; -- From on Board Clock Driver switch between Xtal
	                        -- and PLL generated Clock
	FPGA_CLK_PLL: in STD_LOGIC; -- From on Board XTAL for PLL Clock Generation
	PLL_CLK: out STD_LOGIC; -- Output from Xilinx to board (enable by setting
	                        -- USERo to logic low the default mode. (Min 19MHz Max 125MHz)
	USERo: out STD_LOGIC;  -- Output to U2 Clock Mux control (default Logic Low)
	LD: inout STD_LOGIC_VECTOR (31 downto 0) ; -- Local Data Bus
	LA: in STD_LOGIC_VECTOR (26 downto 2) ;    -- Local Address Bus  bit 26 is s_bar(2)
                                              -- LA(21) is max address line used in example design	
	LBE0_n, LBE1_n, LBE2_n, LBE3_n: in STD_LOGIC; -- Local Bus Byte Enables
	LRESET_n: in STD_LOGIC;  -- Local Reset from PCI bus
	PinADS_n: in STD_LOGIC;  -- Address Strobe from PCI bus
	RdyACK_n: in STD_LOGIC; -- Acknowledge of READYn from PCI bus interface chip U5 
	LW_R_n: in STD_LOGIC;  -- Local Bus Write / Read driven by PCI bus interface chip 
	READYn: out STD_LOGIC; -- Local Bus READYn 
	LINT_n: out STD_LOGIC; --  Asserted causes a PCI interrupt (INTA#).


	DACK0_n: in STD_LOGIC; -- PCI bus Interface chip U5 asserts to indicate DMA trans in process
	DACK1_n: in STD_LOGIC; -- PCI bus Interface chip U5 asserts to indicate DMA trans in process
	DREQ0_n: out STD_LOGIC; -- U7 Xilinx asserts to request a DMA trans on Ch 0
	DREQ1_n: out STD_LOGIC; -- U7 Xilinx asserts to request a DMA trans on Ch 1
	
-- External Temperature monitor
	TEMP_CS : out STD_LOGIC;
   TEMP_SCK : out STD_LOGIC;
	TEMP_SDO: in STD_LOGIC;

--Dual Port SRAM Control Signals
	SR_IO: inout STD_LOGIC_VECTOR (63 downto 0) ; -- SRAM Right Bank I/O Bus  Low 32 bits U4 upper 32 U19
	SR_IOP: inout STD_LOGIC_VECTOR (7 downto 0) ; -- SRAM Right Bank I/O Par Bits
 	SR_A: out STD_LOGIC_VECTOR (19 downto 0) ; -- SRAM Right Bank Address Bus to both U4 and U19
	SR1_CE0n: out STD_LOGIC;	 -- DP-SRAM Right Bank Chip Enable Active Low U4
	SR3_CE0n: out STD_LOGIC;	 -- DP-SRAM Right Bank Chip Enable Active Low U19
	SR1_CE1: out STD_LOGIC;	 -- DP-SRAM Right Bank Chip Enable Active Low U4
	SR3_CE1: out STD_LOGIC;	 -- DP-SRAM Right Bank Chip Enable Active Low	U19
	SR1_OEn: out STD_LOGIC;	 -- DP-SRAM Right Bank Output Enable Active Low U4
	SR3_OEn: out STD_LOGIC;	 -- DP-SRAM Right Bank Output Enable Active Low U19
	SR_BW0n: out STD_LOGIC;	 -- DP-SRAM Right Bank Individual Byte Write Select Active Low U4
	SR_BW1n: out STD_LOGIC;	 -- DP-SRAM Right Bank Individual Byte Write Select Active Low U4
	SR_BW2n: out STD_LOGIC;	 -- DP-SRAM Right Bank Individual Byte Write Select Active Low U4
	SR_BW3n: out STD_LOGIC;	 -- DP-SRAM Right Bank Individual Byte Write Select Active Low U4 
	SR_BW4n: out STD_LOGIC;	 -- DP-SRAM Right Bank Individual Byte Write Select Active Low U19
	SR_BW5n: out STD_LOGIC;	 -- DP-SRAM Right Bank Individual Byte Write Select Active Low U19
	SR_BW6n: out STD_LOGIC;	 -- DP-SRAM Right Bank Individual Byte Write Select Active Low U19
	SR_BW7n: out STD_LOGIC;	 -- DP-SRAM Right Bank Individual Byte Write Select Active Low U19
	SR_REPEATn: out STD_LOGIC; -- DP-SRAM Right Bank Repeat Last Registered Address Active Low  both U4 and U19
	SR_ADSn: out STD_LOGIC;	 -- DP-SRAM Right Bank Address Select Active Low  both U4 and U19
	SR_PL_FTn: out STD_LOGIC; -- DP-SRAM Right Bank PipeLine - Flow Through Select Line  both U4 and U19
	SR_CNTENn: out STD_LOGIC; -- DP-SRAM Right Bank Address Increment Active Low  both U4 and U19
	SR1_R_Wn: out STD_LOGIC;	 -- DP-SRAM Right Bank Read / Write low U4
	SR3_R_Wn: out STD_LOGIC;	 -- DP-SRAM Right Bank Read / Write low U19
	SR_INTn: in STD_LOGIC;	 -- DP-SRAM Right Bank Interrupt Active Low U4
	SR3_INTn: in STD_LOGIC;	 -- DP-SRAM Right Bank Interrupt Active Low U19
	SR_COLn: in STD_LOGIC;	 -- DP-SRAM Right Bank Collision Detect Active Low U4 
	SR3_COLn: in STD_LOGIC;	 -- DP-SRAM Right Bank Collision Detect Active Low U19
	SR_ZZ: out STD_LOGIC;	 -- DP_SRAM Right Bank Sleep Active High

-- I/O to AXM module Transceivers
--	FIO_DIG: inout STD_LOGIC_VECTOR(15 downto 0); -- Front Digital I/O
--	FDIR_DIG: out STD_LOGIC_VECTOR(15 downto 0);  -- Front Digital I/O Direction Control
--	FIO_DIFF: inout STD_LOGIC_VECTOR(29 downto 0); --Front Differential I/O Signals 
----for debug	FIO_DIFF: out STD_LOGIC_VECTOR(29 downto 0); --Front Differential I/O Signals 
--	FDIR_DIFF: out STD_LOGIC_VECTOR(29 downto 0); -- Front Differential I/O Dir Control 
--		
---- Not used Front I/O 
--   FN_VRN17: out STD_LOGIC;
--	FP_VRP17: out STD_LOGIC;
--	FP3_GC0: out STD_LOGIC;
--	FN3_GC1_VREF: out STD_LOGIC;
--	FN_CTRL1: out STD_LOGIC;
	
		--AXM-A30 I/O 
	AD_D1_P: in STD_LOGIC_VECTOR(15 downto 0);  --ADC Channel 1 Data Bus Positive terminal
	AD_D1_N: in STD_LOGIC_VECTOR(15 downto 0);  --ADC Channel 1 Data Bus Negative terminal
	AD_D2_P: in STD_LOGIC_VECTOR(15 downto 0);  --ADC Channel 2 Data Bus Positive terminal
	AD_D2_N: in STD_LOGIC_VECTOR(15 downto 0);  --ADC Channel 2 Data Bus Negative terminal
	AD_CLK1_P: in STD_LOGIC;  	--ADC Channel 1 Clock input Positive terminal
	AD_CLK1_N: in STD_LOGIC;	--ADC Channel 1 Clock input Negative terminal
	AD_CLK2_P: in STD_LOGIC;   --ADC Channel 2 Clock input Positive terminal
	AD_CLK2_N: in STD_LOGIC;   --ADC Channel 2 Clock input Negative terminal
	AD_OTR: in STD_LOGIC_VECTOR(1 downto 0);		--ADC Channel 1 & 2 Voltage Out of Range indicator
	AD_DCS: out STD_LOGIC_VECTOR(1 downto 0);	-- AD9446 Duty Cycle Stabilizer Active High  (both channels)
	AD_DFS: out STD_LOGIC_VECTOR(1 downto 0);	-- AD9446 Data Format Select (both channels) 
	
	GPIO: inout STD_LOGIC;  	   --External General Purpose Input/Output
	ExtInput_En: out STD_LOGIC;   --Set low to enable the GPIO as an Input
	ExtOutput_En: out STD_LOGIC;  --Set high to enable the GPIO as an Output
	
	ExtClkOut_En: out STD_LOGIC;  --Set low to feed LMK Clk1 out to the GPIO
	Crystal_En: out STD_LOGIC;    --Set high to enable opscillator to LMK03000C input
	ExtCLKIn_En: out STD_LOGIC;   --Set high to enable External Clock to LMK03000C input 
	FPGACLKIn_En: out STD_LOGIC;  --Set high to enable FPGA Clock to LMK03000C input
	
	SerDataOut: out STD_LOGIC;  --LMK03000C Serial Interface Data line
	SerCLKOut: out STD_LOGIC;	 --LMK03000C Serial Interface CLK (<20MHz)
	SerLatch_n: out STD_LOGIC;  --LMK03000C Serial Interface Enable(Latch)Active Low line  
	
	LMK_GOE: out STD_LOGIC;    --LMK03000C Global Output Enable Pin
	LMK_SYNCH: out STD_LOGIC;	--LMK03000C Synch Pin
	LOCK: in STD_LOGIC;        --LKM03000C Clock Lock Indicator Pin
	
	FPGACLKIN: in STD_LOGIC;	--Output from LMK03000C device to FPGA  Note that it is
										--converted from LVDS to single ended by a transceiver. 
										--This input is not utilized in this example design.
	FPGACLK_IN_EN: out STD_LOGIC; --Enable signal for output from LMK03000C device to FPGA  
											--Note that this enables an LVDS to single ended by a transceiver.
	FPGACLK_OUT: out STD_LOGIC; ---16.6667MHz for V4, 13.3MHz for V5
											--	clock that can be used as input for the LMK03000.
											--Use only for demo purposes.
		
	CH1_PDN: out STD_LOGIC;		--ADA4937 active low power-down pin Channel 1
	CH2_PDN: out STD_LOGIC;		--ADA4937 active low power-down pin Channel 2
	
--end AXM-A30 I/O

-- The following signals are used to interface to the Rear I/O LVDS port
   RP_IO0: in std_logic; 
   RN_IO1: in std_logic;  
   RP_IO2: out std_logic;
   RN_IO3: out std_logic;
   RP_IO4: out std_logic;
   RN_IO5: out std_logic;
   RP_IO6: in std_logic;
   RN_IO7: in std_logic;
   RP_IO8: out std_logic;
   RN_IO9: out std_logic;
   RP_IO10: in std_logic;  
   RN_IO11: in std_logic; 
   RP_IO12: out std_logic;
   RN_IO13: out std_logic;
   RP_IO14: in std_logic;
   RN_IO15: in std_logic;
   RP_IO16: out std_logic;
   RN_IO17: out std_logic;
   RP_IO18: in std_logic;
   RN_IO19: in std_logic;
   RP_IO20: out std_logic;
   RN_IO21: out std_logic;
   RP_IO22: in std_logic;
   RN_IO23: in std_logic;
   RP_IO24: out std_logic;
   RN_IO25: out std_logic;
   RP_IO26: in std_logic;
   RN_IO27: in std_logic;
   RP_IO28: out std_logic;
   RN_IO29: out std_logic;
   RP_IO30: in std_logic;  
   RN_IO31: in std_logic;  
   RP_IO32: out std_logic;
   RN_IO33: out std_logic;
   RP_IO34: in std_logic;
   RN_IO35: in std_logic;
   RP_IO36: out std_logic;
   RN_IO37: out std_logic;
   RP_IO38: in std_logic;
   RN_IO39: in std_logic;
   RP_IO40: out std_logic;
   RN_IO41: out std_logic;
   RP_IO42: in std_logic;
   RN_IO43: in std_logic;
   RP_IO44: out std_logic;
   RN_IO45: out std_logic;
   RP_IO46: in std_logic;  
   RN_IO47: in std_logic;  
   RP_IO48: out std_logic;
   RN_IO49: out std_logic;
   RP_IO50: in std_logic;
   RN_IO51: in std_logic;
   RP_IO52: out std_logic;
   RN_IO53: out std_logic;
   RP_IO54: in std_logic;
   RN_IO55: in std_logic;
   RP_IO56: out std_logic;
   RN_IO57: out std_logic;
   RP_IO58: in std_logic;
   RN_IO59: in std_logic;
   RP_IO60: out std_logic;
   RN_IO61: out std_logic;
   RP_IO62: in std_logic;
   RN_IO63: in std_logic);		

end XC6SLX150;

architecture XC6SLX150_arch of XC6SLX150 is
--for debug signal 
signal Temp_FIO_DIFF : STD_LOGIC_VECTOR (29 downto 0);
signal  SRR_INTn : STD_LOGIC;
signal  SRR_COLn : STD_LOGIC;

signal CLK : STD_LOGIC;
signal CLKPCIbus : STD_LOGIC;
signal RESET : STD_LOGIC; 
--signal RESETn : STD_LOGIC;
signal LRESET_Reg : STD_LOGIC; 
signal USERoI : STD_LOGIC; 
signal ADS_n, D_Ready : STD_LOGIC; 

-- Address Decode Signals -----------------------------------
signal Not_Used_address_lines : STD_LOGIC;
signal U7_Address : STD_LOGIC;
signal Base_Address : STD_LOGIC;
signal Flash_Base_Address : STD_LOGIC;
signal Not_Used_Space : STD_LOGIC;
signal Int_StatClear_Adr : STD_LOGIC; -- 0x8000

--AXM Address Decode Signals
--signal DiffReg31to0_Adr : STD_LOGIC; --0x8004
--signal DigReg15to0_Adr : STD_LOGIC; --0x8008
--signal DiffDirReg_Adr : STD_LOGIC; --0x800C
--signal DigDirReg_Adr : STD_LOGIC; --0x8010
--signal Int_Enable_Adr : STD_LOGIC; --0x8014
--signal Int_Type_Adr : STD_LOGIC; --0x8018
--signal Int_Polarity_Adr : STD_LOGIC; --0x801C

--Rear J4 Connector Address Decode Signals
signal Rear_LVDS_Rd_Adr : STD_LOGIC; --0x802C
signal Rear_LVDS_Wr_Adr : STD_LOGIC; --0x8030

-- DP_SRAM/DMA Decode signals
signal DMA_Control_Adr : STD_LOGIC; -- 0x8034
signal SRAM_Read_Adr : STD_LOGIC;  -- 0x8038
signal SRAM_Read_Adr2 : STD_LOGIC; -- 0x803C
signal SRAM_CONTROL_Adr: STD_LOGIC; -- 0x8040 
signal SRAM_IntAdr: STD_LOGIC; --0x8044
signal SRAM_DMA0Thr_Adr: STD_LOGIC; --0x8048
signal SRAM_DMA1Thr_Adr: STD_LOGIC; --0x804C
signal SRAM_Reset0_Adr: STD_LOGIC;  --0x8050
signal SRAM_Reset1_Adr: STD_LOGIC;  --0x8054
signal PMC_ID_Code_Adr: STD_LOGIC;  --0x8058

--Temperature Sensor Address Decode signals
signal TEMP_SENSOR_Data_Adr: STD_LOGIC; --0x8090

--AXM-A30 Address Decode Signals
signal ADCControl_Adr: STD_LOGIC; --0x8100
signal ADCCh1_Adr: STD_LOGIC; --0x8104
signal ADCCh2_Adr: STD_LOGIC; --0x8108
signal ADCFifo_Adr: STD_LOGIC; --0x810C
signal PLLControl_Adr: STD_LOGIC; --0x8110
signal PLLR0_Adr: STD_LOGIC; --0x8114
signal PLLR1_Adr: STD_LOGIC; --0x8118
signal PLLR3_Adr: STD_LOGIC; --0x811C
signal PLLR4_Adr: STD_LOGIC; --0x8120
signal PLLR11_Adr: STD_LOGIC; --0x8124
signal PLLR13_Adr: STD_LOGIC; --0x8128
signal PLLR14_Adr: STD_LOGIC; --0x812C
signal PLLR15_Adr: STD_LOGIC; --0x8130
signal ADC_SRAM_Adr: STD_LOGIC; --0x8134



-- Write Strobe signals --------------------------------------
signal Reset_Reg_Stb1 : STD_LOGIC;
signal Int_StatClear_Stb0: STD_LOGIC;
signal Int_StatClear_Stb1: STD_LOGIC;
signal SRAM_Strobe: STD_LOGIC;
signal DMA_Control_Stb0 : STD_LOGIC;

-- Local Bus Read/Write control Signals ------------------
signal Int_READY, READY, READY_RESET, Int_READY2 : STD_LOGIC;
signal READ_EN : STD_LOGIC;
signal RD_Data : STD_LOGIC_VECTOR (31 downto 0);
signal General_Read_Stb: STD_LOGIC;

-- Register Signals ---------------------------------------
signal Software_Reset : STD_LOGIC;
signal Software_Reset1 : STD_LOGIC;
signal Software_Reset2 : STD_LOGIC;
signal Software_Reset3 : STD_LOGIC;
signal IntStatA_Reg: STD_LOGIC_VECTOR(1 downto 0);

-- AXM Signals --------------------------------------------
signal AXM_RdData: STD_LOGIC_VECTOR(31 downto 0);
signal AXM_Strobe: STD_LOGIC;
signal AXM_ID: STD_LOGIC_VECTOR(2 downto 0);

-- DMA Signals --------------------------------------------
signal DMA_Request0 : STD_LOGIC;
signal DMA_Request1 : STD_LOGIC;

-- Digital Clock Manager Signals
signal ClockReset : STD_LOGIC;
signal Clock_Locked, Clock_Locked_Reg1, Clock_Locked_Reg2, Clock_Locked_Reg3 : STD_LOGIC;
signal CLKFX_W : STD_LOGIC;
signal CLK0_Out : STD_LOGIC; -- For Feedback
signal FB_CLK : STD_LOGIC;   -- For Feedback
signal FPGA_CLK_PLL_O : STD_LOGIC; -- Input to DCM from on board 125MHz Clock
signal CLK_TEMPERATURE, CLK_TEMPERATURE_BUF: STD_LOGIC; --2.34MHz output from DCM
signal Clock_Locked2: STD_LOGIC;



--SRAM signals --------------------------------------------
signal SRAM_Sel : STD_LOGIC;
signal SRR_SEL_ReadOnly : STD_LOGIC;
signal SRAM_ACK: STD_LOGIC; --Right port Acknowledge signal
signal SRAM_DATA_BUS : STD_LOGIC_Vector(63 downto 0);
signal SRAM_Parity_BUS: STD_LOGIC_VECTOR(7 downto 0);
signal SRR_W_Rn_Sel: STD_LOGIC;

signal SRAM_DMA0_REQ, SRAM_DMA1_REQ: STD_LOGIC;
signal SRAM_RdData: STD_LOGIC_VECTOR(31 downto 0);
signal SRAM_ENABLED: STD_LOGIC;
signal Start_SRAM_Read: STD_LOGIC;
signal Start_SRAM_Write: STD_LOGIC;
signal Start_SRAM_Write2: STD_LOGIC;

signal Rear_2_SRAM: STD_LOGIC_VECTOR(15 downto 0);
signal SRAM_Write_READY, SRAM_Write_READY_Delay: STD_LOGIC;
signal SRAM_BUSY: STD_LOGIC;

-- Rear IO Signals --------------------------------------------
signal Rear_RdData: STD_LOGIC_VECTOR(15 downto 0);
signal Rear_Strobe: STD_LOGIC;

-- Temperature Sensor Signals ---------------------------------
signal TEMP_Sen_Strobe : STD_LOGIC;
signal TEMP_DATA: STD_LOGIC_VECTOR(12 downto 0);

signal clkf_w, CLKFX_Wn, Int_READYn: std_logic;
signal SR_A_temp: STD_LOGIC_VECTOR(19 downto 0);
signal SR_IO_temp: STD_LOGIC_VECTOR(63 downto 0);
signal SR1_CE0n_temp, SR3_CE0n_temp: STD_LOGIC;

--AXM-A30 interface signals
signal SRAM_FIFO_DATA: STD_LOGIC_VECTOR(15 downto 0);
signal UnusedFrontIO: STD_LOGIC_VECTOR(4 downto 0);
signal Word_Sel: STD_LOGIC_VECTOR(1 downto 0);
signal SRAM_PAUSE, FIFO_EMPTY_FLAG: STD_LOGIC;
signal ADC_OVERRIDE_EN: STD_LOGIC;
signal ADC_SRAM_ENW0, ADC_SRAM_ENW1, ADC_SRAM_ENW2, ADC_SRAM_ENW3: STD_LOGIC;
signal SRAM_TEMP : STD_LOGIC_VECTOR(63 downto 0);
signal AXM_A30SerCLK_BUF, AXM_A30SerCLK : STD_LOGIC;
signal FIFOtoSRAMPause: STD_LOGIC;
signal FIFODataValidFlag: STD_LOGIC; 
signal Inc_Counter, Counter_reset: STD_LOGIC;
signal ADCFIFOWrite,ADC_SRAM_ENW3_reg, ADCFIFOWrite_Reg : STD_LOGIC;

component DP_SRAM 
port(

 --Global signal ---------------------------------------------------------------
	CLK: in STD_LOGIC;
   RESET: in STD_LOGIC;

--Right Bank SRAM pin connections ----------------------------------------------
	SRR_A: out STD_LOGIC_VECTOR (19 downto 0) ; -- SRAM Address Bus
	SRR1_CE0n: out STD_LOGIC;	 -- SRAM Chip Enable Active Low -- 1 are lower long word 0 to 31
	SRR1_OEn: out STD_LOGIC;	 -- SRAM Output Enable Active Low
	SRR3_CE0n: out STD_LOGIC;	 -- SRAM Chip Enable Active Low --3 are high long word 32 to 63
	SRR3_OEn: out STD_LOGIC;	 -- SRAM Output Enable Active Low
	SRR_BE0n: out STD_LOGIC;	 -- SRAM Individual Byte Write Select Active Low
	SRR_BE1n: out STD_LOGIC;	 -- SRAM Individual Byte Write Select Active Low
	SRR_BE2n: out STD_LOGIC;	 -- SRAM Individual Byte Write Select Active Low
	SRR_BE3n: out STD_LOGIC;	 -- SRAM Individual Byte Write Select Active Low
	SRR_BE4n: out STD_LOGIC;	 -- SRAM Individual Byte Write Select Active Low
	SRR_BE5n: out STD_LOGIC;	 -- SRAM Individual Byte Write Select Active Low
	SRR_BE6n: out STD_LOGIC;	 -- SRAM Individual Byte Write Select Active Low
	SRR_BE7n: out STD_LOGIC;	 -- SRAM Individual Byte Write Select Active Low
	SRR_ADSn: out STD_LOGIC; 	-- SRAM Address Strobe Enable Active Low
	SRR_PL_FTn: out STD_LOGIC;	-- SRAM Pipeline / Flow-Through Mode Sel Line
	SRR_CNTENn: out STD_LOGIC;	-- SRAM Counter Enable Active Low
	SRR_REPEATn: out STD_LOGIC;	-- SRAM Counter Repeat Active Low
	SRR_ZZ: out STD_LOGIC;		-- SRAM Sleep Mode Active High
	SRR_INTn: in STD_LOGIC;		-- SRAM Interrupt Flag Active Low
	SRR_COLn: in STD_LOGIC;		-- SRAM Collision Flag Active Low
 	SRR1_R_Wn: out STD_LOGIC;	 -- SRAM Read / Write low
 	SRR3_R_Wn: out STD_LOGIC;	 -- SRAM Read / Write low
 	SRR_IO: inout STD_LOGIC_VECTOR (63 downto 0); -- SRAM I/O Bus
	SRR_IOP: inout STD_LOGIC_VECTOR (7 downto 0); -- SRAM I/O Bus Parity Bits

	--for right port
 	SRR_I_Bus: in STD_LOGIC_VECTOR (63 downto 0); -- Right Port Data Bus 
	SRR_IOPar_Bus: inout STD_LOGIC_VECTOR (7 downto 0); -- Right Port Data Par

	SRR_SEL: in STD_LOGIC;	--Right port Select signal
	SRR_W_Rn_Sel: in STD_LOGIC;  --Right port Read / Write low  signal
	SRR_Ack: out STD_LOGIC;  --Right port Acknowledge signal
	--only single cycle pipeline reads are supported in this example. 
	
--Local Bus connections --------------------------------------------------------
	ADS_n: in STD_LOGIC;  -- Address Strobe from PCI U5
	LW_R_n: in STD_LOGIC;  -- Local Bus Write / Read driven by PCI U5
	LD: in STD_LOGIC_VECTOR(31 downto 0); --Local Data Bus
	LBE0_n: in STD_LOGIC; --Local Data Bus Byte 0 Enables active low
	LBE1_n: in STD_LOGIC; --Local Data Bus Byte 1 Enables active low
	LBE2_n: in STD_LOGIC; --Local Data Bus Byte 2 Enables active low
	LBE3_n: in STD_LOGIC; --Local Data Bus Byte 3 Enables active low
	
	READ_DATA: out STD_LOGIC_VECTOR(31 downto 0); --Readback data bus for SRAM Registers
	
	SRAM_Read_Adr : in STD_LOGIC; --SRAM Read Right Port Address Strobe LSW
	SRAM_Read_Adr2 : in STD_LOGIC; --SRAM Read Right Port Address Strobe MSW
	SRAM_CONTROL_Adr: in STD_LOGIC; --SRAM Control Register Address Strobe
	SRAM_IntAdr: in STD_LOGIC; --SRAM Internal Address Register Address Strobe
	SRAM_DMA0Thr_Adr: in STD_LOGIC; --SRAM DMA0 Threshold Register Address Strobe
	SRAM_DMA1Thr_Adr: in STD_LOGIC; --SRAM DMA1 Threshold Register Address Strobe
	SRAM_Reset0_Adr: in STD_LOGIC; --SRAM Address Reset Value Register 0 (DMA0 Threshold) 
	SRAM_Reset1_Adr: in STD_LOGIC; --SRAM Address Reset Value Register 1 (DMA1 Threshold) 
	DMA0_REQ: out STD_LOGIC;	 --SRAM DMA0 Request
	DMA1_REQ: out STD_LOGIC;	 --SRAM DMA1 Request
	SRAM_ENABLED: out STD_LOGIC;
	SRAM_BUSY: out STD_LOGIC;
	ADC_OVERRIDE: in STD_LOGIC
   );

end component;

--component AXM_D
--    Port (
--	CLK: in STD_LOGIC; --Local clock
--	RESET: in STD_LOGIC; --Global Reset signal (active high)
--	
--	ADS_n: in STD_LOGIC;  -- Address Strobe from PCI U5
--	LW_R_n: in STD_LOGIC;  -- Local Bus Write / Read driven by U5
--   LBE0_n: in STD_LOGIC; --Local Data Bus Byte 0 Enables active low
--	LBE1_n: in STD_LOGIC; --Local Data Bus Byte 1 Enables active low
--	LBE2_n: in STD_LOGIC; --Local Data Bus Byte 2 Enables active low
--	LBE3_n: in STD_LOGIC; --Local Data Bus Byte 3 Enables active low	 
--	LD : in  STD_LOGIC_VECTOR (31 downto 0); --Local Data Bus
--	READ_DATA: out STD_LOGIC_VECTOR(31 downto 0); --Readback data bus of AXM_D Registers
--	Int_Status: out STD_LOGIC_VECTOR(7 downto 0); -- Interrupt status of 8 diff channels
--	
--	DiffReg31to0_Adr: in STD_LOGIC;  -- Differential I/O Reg Address strobe
--	DigReg15to0_Adr: in STD_LOGIC;   -- Digital I/O Reg Address strobe
--	DiffDirReg_Adr: in STD_LOGIC;    -- Diff Direction Control Reg Address strobe
--	DigDirReg_Adr: in STD_LOGIC;     -- Digital Direction Control Reg Address strobe
--	Int_Enable_Adr: in STD_LOGIC;    -- Interrupt Enable Register Address strobe
--	Int_Type_Adr: in STD_LOGIC;      -- Interrupt Type Register Address strobe
--	Int_Polarity_Adr: in STD_LOGIC;  -- Interrupt Polarity Register Address strobe
--	Int_StatClear_Stb0: in STD_LOGIC;-- Interrupt Status/Clear Reg Write strobe
--	
--	IO_DIGITAL: inout STD_LOGIC_VECTOR(15 downto 0); -- Front Digital I/O
--	DIR_DIG: out STD_LOGIC_VECTOR(15 downto 0);  -- Front Digital I/O Direction Control
--	IO_DIFF: inout STD_LOGIC_VECTOR(29 downto 0); --Front Differential I/O Signals
--	DIR_DIFF: out STD_LOGIC_VECTOR(29 downto 0); -- Front Differential I/O Dir Control	
--	
--	AXM_ID: out STD_LOGIC_VECTOR(2 downto 0) -- AXM Module ID.
--	 );
--end component;

component AXM_A30
   port (
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
	ADCCh1_Adr: in STD_LOGIC;  -- ADC Channel 1 Control/Threshold Reg Strobe
	ADCCh2_Adr: in STD_LOGIC;   -- ADC Channel 2 Control/Threshold Reg Strobe
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
	ADC_SRAM_Adr: in STD_LOGIC;   --AXM-A30 SRAM Control Stobes
	
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
	FPGACLK_IN_EN: out STD_LOGIC;	--Enable signal for output from LMK03000C device to FPGA  
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
	FIFODataValidFlag: out STD_LOGIC --Flag that indicates that read data from FIFO is valid.
	);		
end component;


component RearLVDS
    Port (
	CLK: in STD_LOGIC; --Local clock
	RESET: in STD_LOGIC; --Global Reset signal (active high)
	
	ADS_n: in STD_LOGIC;  -- Address Strobe from U5 PCI bus interface chip
	LW_R_n: in STD_LOGIC;  -- Local Bus Write / Read driven by U5 PCI bus interface chip
   LBE0_n: in STD_LOGIC; --Local Data Bus Byte 0 Enables active low
   LBE1_n: in STD_LOGIC; --Local Data Bus Byte 1 Enables active low
	LD : in  STD_LOGIC_VECTOR (15 downto 0); --Local Data Bus
	READ_DATA: out STD_LOGIC_VECTOR(15 downto 0); --Readback data bus of Rear I/O Registers
	Rear_2_SRAM: out STD_LOGIC_VECTOR(15 downto 0); --Rear I/O channel 0:15 to SRAM input with addition logic needed to mux into SRAM
	
	Rear_LVDS_Rd_Adr: in STD_LOGIC;  -- Rear I/O Read Address strobe
	Rear_LVDS_Wr_Adr: in STD_LOGIC;   -- Rear I/O Write Address strobe
	
   RP_IO0: in std_logic; 
   RN_IO1: in std_logic; 
   RP_IO2: out std_logic;
   RN_IO3: out std_logic;
   RP_IO4: out std_logic;
   RN_IO5: out std_logic;
   RP_IO6: in std_logic;
   RN_IO7: in std_logic;
   RP_IO8: out std_logic;
   RN_IO9: out std_logic;
   RP_IO10: in std_logic; 
   RN_IO11: in std_logic; 
   RP_IO12: out std_logic;
   RN_IO13: out std_logic;
   RP_IO14: in std_logic;
   RN_IO15: in std_logic;
   RP_IO16: out std_logic;
   RN_IO17: out std_logic;
   RP_IO18: in std_logic;
   RN_IO19: in std_logic;
   RP_IO20: out std_logic;
   RN_IO21: out std_logic;
   RP_IO22: in std_logic;
   RN_IO23: in std_logic;
   RP_IO24: out std_logic;
   RN_IO25: out std_logic;
   RP_IO26: in std_logic;
   RN_IO27: in std_logic;
   RP_IO28: out std_logic;
   RN_IO29: out std_logic;
   RP_IO30: in std_logic; 
   RN_IO31: in std_logic; 
   RP_IO32: out std_logic;
   RN_IO33: out std_logic;
   RP_IO34: in std_logic;
   RN_IO35: in std_logic;
   RP_IO36: out std_logic;
   RN_IO37: out std_logic;
   RP_IO38: in std_logic;
   RN_IO39: in std_logic;
   RP_IO40: out std_logic;
   RN_IO41: out std_logic;
   RP_IO42: in std_logic;
   RN_IO43: in std_logic;
   RP_IO44: out std_logic;
   RN_IO45: out std_logic;
   RP_IO46: in std_logic; --LVSD input
   RN_IO47: in std_logic; --LVSD input
   RP_IO48: out std_logic;
   RN_IO49: out std_logic;
   RP_IO50: in std_logic;
   RN_IO51: in std_logic;
   RP_IO52: out std_logic;
   RN_IO53: out std_logic;
   RP_IO54: in std_logic;
   RN_IO55: in std_logic;
   RP_IO56: out std_logic;
   RN_IO57: out std_logic;
   RP_IO58: in std_logic;
   RN_IO59: in std_logic;
   RP_IO60: out std_logic;
   RN_IO61: out std_logic;
   RP_IO62: in std_logic;
   RN_IO63: in std_logic		
		 );         		
end component;

component SPI_Temp_Sensor
    Port ( 

	CLK: in STD_LOGIC; --2.34375MHz clock from DCM-
	RESET: in STD_LOGIC; --Global Reset signal (active high)
	
	READ_DATA: out STD_LOGIC_VECTOR(12 downto 0); --Readback data bus of Registers
		
   TEMP_CS : out  STD_LOGIC; --SPI Chip select Signal
   TEMP_SCK : out  STD_LOGIC;--SPI Clock Signal. Must be <=5MHz
   TEMP_SDO : in  STD_LOGIC -- SPI Data Input.  
			  
			  );
end component;
	
---------==========================================================================
begin
--for debug   
-- FIO_DIFF(0) <= ADS_n;
-- FIO_DIFF(1) <= READY;
-- FIO_DIFF(2) <= '1'; --DRDY_OUT;  
-- FIO_DIFF(3) <= LW_R_n;
-- FIO_DIFF(4) <= RdyACK_n;
-- FIO_DIFF(5) <= '1';--spare1; -- slv_bar(0)
-- FIO_DIFF(6) <= '1';--spare2; -- dma_rd
-- FIO_DIFF(7) <= '1';--spare3; -- dma_wr
-- FIO_DIFF(8) <= SR1_CE0n_temp;--LBE0_n; -- slv_readreq
-- FIO_DIFF(9) <= SR3_CE0n_temp;--LBE1_n; -- slv_writereq
-- FIO_DIFF(10) <= LBE2_n; -- slv_bar(2)
-- FIO_DIFF(11) <= LA(26); --int_request
-- FIO_DIFF(12) <= '1';--DWE; 
-- FIO_DIFF(13) <= '1';--DEN; 
-- FIO_DIFF(14) <= SR_A_temp(0);
-- FIO_DIFF(15) <= SR_A_temp(1);
-- FIO_DIFF(16) <= SR_A_temp(2);
-- FIO_DIFF(17) <= SR_A_temp(3);
-- FIO_DIFF(18) <= SR_A_temp(4);
-- FIO_DIFF(19) <= SR_A_temp(5);
-- FIO_DIFF(20) <= SR_A_temp(6);
-- FIO_DIFF(21) <= SR_A_temp(7);
-- FIO_DIFF(22) <= SR_IO_temp(32);
-- FIO_DIFF(23) <= SR_IO_temp(33);
-- FIO_DIFF(24) <= SR_IO_temp(34);
-- FIO_DIFF(25) <= SR_IO_temp(35);
-- FIO_DIFF(26) <= SR_IO_temp(36);
-- FIO_DIFF(27) <= SR_IO_temp(37);
-- FIO_DIFF(28) <= SR_IO_temp(38);
-- FIO_DIFF(29) <= SR_IO_temp(39);

 
 SRR_INTn <= SR_INTn or SR3_INTn;
 SRR_COLn <= SR_COLn or SR3_COLn;
 
 SR3_CE1 <= '1';
 SR1_CE1 <= '1';

--ADSn input assigned to FF to guarentee IOB placement.
FDRSE_inst_PinADS_n : FDSE
generic map (INIT => '1') -- Initial value of register (?0? or ?1?)
port map (
	Q => ADS_n, -- Data output
	C => CLK, -- Clock input
	CE => '1', -- Clock enable input (active high)
	D => PinADS_n, -- Data input
	S => '0' -- Synchronous set input (active high)
);  

U1_IBUFG: IBUFG
  port map (
  	    I => FPGA_CLK_PLL,
            O => FPGA_CLK_PLL_O
  	   );

--Spartan 6 DCM Block
-- Used to generate 75MHz local clock for all operations except SPI
  DCM_SP_inst : DCM_SP
generic map ( 
	CLKDV_DIVIDE => 5.0, -- CLKDV divide value -- 75MHz / 5 = 15Mhz
	-- (1.5,2,2.5,3,3.5,4,4.5,5,5.5,6,6.5,7,7.5,8,9,10,11,12,13,14,15,16).
	CLKFX_DIVIDE => 5, -- Divide value - D - (1-32) (3/5)*125MHz=75Mhz
	CLKFX_MULTIPLY => 3, -- Multiply value - M - (2-32) (3/5)*125MHz=75Mhz 
	CLKIN_DIVIDE_BY_2 => FALSE, -- CLKIN divide by two (TRUE/FALSE)
	CLKIN_PERIOD => 8.0, -- Input clock period specified in nS
	CLKOUT_PHASE_SHIFT => "NONE", -- Output phase shift (NONE, FIXED, VARIABLE)
	CLK_FEEDBACK => "1X", -- Feedback source (NONE, 1X, 2X)
	DESKEW_ADJUST => "SYSTEM_SYNCHRONOUS", -- SYSTEM_SYNCHRNOUS or SOURCE_SYNCHRONOUS
	DFS_FREQUENCY_MODE => "LOW", -- Unsupported - Do not change value
	DLL_FREQUENCY_MODE => "LOW", -- Unsupported - Do not change value
	DSS_MODE => "NONE", -- Unsupported - Do not change value
	DUTY_CYCLE_CORRECTION => TRUE, -- Unsupported - Do not change value
	FACTORY_JF => X"c080", -- Unsupported - Do not change value
	PHASE_SHIFT => 0, -- Amount of fixed phase shift (-255 to 255)
	STARTUP_WAIT => FALSE -- Delay config DONE until DCM LOCKED (TRUE/FALSE)
)
port map (
	CLK0 => CLK0_Out, -- 1-bit 0 degree clock output
	CLK180 => open, -- 1-bit 180 degree clock output
	CLK270 => open, -- 1-bit 270 degree clock output
	CLK2X => open, -- 1-bit 2X clock frequency clock output
	CLK2X180 => open, -- 1-bit 2X clock frequency, 180 degree clock output
	CLK90 => open, -- 1-bit 90 degree clock output
	CLKDV => AXM_A30SerCLK, -- 1-bit Divided clock output
	CLKFX => CLKFX_W, -- 1-bit Digital Frequency Synthesizer output (DFS)
	CLKFX180 => CLKFX_Wn, -- 1-bit 180 degree CLKFX output
	LOCKED => Clock_Locked, -- 1-bit DCM Lock Output
	PSDONE => open, -- 1-bit Phase shift done output
	STATUS => open, -- 8-bit DCM status output
	CLKFB => FB_CLK, -- 1-bit Clock feedback input
	CLKIN => FPGA_CLK_PLL_O, -- 1-bit Clock input
	DSSEN => open, -- 1-bit Unsupported
	PSCLK => '0', -- 1-bit Phase shift clock input
	PSEN => '0', -- 1-bit Phase shift enable
	PSINCDEC => open, -- 1-bit Phase shift increment/decrement input
	RST => ClockReset -- 1-bit Active high reset input
);
-- End of DCM_SP_inst instantiation


	
-- BUFG Instantiation for CLK0
U1_BUFG: BUFG
  port map (
  	    I => CLK0_Out,
            O => FB_CLK
  	   );
		
AXM_A30_SERCLK: BUFG
  port map (
  	    I => AXM_A30SerCLK,
            O => AXM_A30SerCLK_BUF
  	   );	


--PLL_CLK is the clock used by some external components that require the same clock as the Spartan 6 FPGA.
-- This clock goes to the SRAM, as well as the Xilinx LX30 FPGA for the local bus. Due to Spartan 6 global clock routing
-- restrictions, the best approach for generating an external high speed clock with a minimum of added
-- skew due to routing is to use a DDR output component.  One Input of the DDR is fixed to logic '1'
-- and the other is fixed to logic '0'. Then the clock and its inverse in fed from the PLL using global
-- clock resources.  Then on the rising edge of the generated clock the part will output 1 to the pin and on the
-- falling edge the component will output 0 to the pin. While unconventional this method is required 
-- to minimuize skew between the internal clock and to meeting timing requirements.    		
ODDR2_inst : ODDR2
generic map(
DDR_ALIGNMENT => "NONE", -- Sets output alignment to "NONE", "C0", "C1"
INIT => '0', -- Sets initial state of the Q output to '0' or '1'
SRTYPE => "SYNC") -- Specifies "SYNC" or "ASYNC" set/reset
port map (
Q => PLL_CLK, -- 1-bit output data
C0 => CLKFX_W, -- 1-bit clock input
C1 => CLKFX_Wn, -- 1-bit clock input
CE => '1', -- 1-bit clock enable input
D0 => '1', -- 1-bit data input (associated with C0)
D1 => '0', -- 1-bit data input (associated with C1)
R =>'0', -- 1-bit reset input
S => '0' -- 1-bit set input
);
     

U3_BUFG: BUFG  -- Output from Cypress clock drive. This can be
  port map (   -- 125MHz or PLL_CLK as selected by USERo
  	    I => FPGA_CLK,
            O => CLK
  	   );

U4_BUFG: BUFG  -- Output from U5 PCI bus Clock. 
  port map (   
  	    I => CLK_PCI_U7,
            O => CLKPCIbus
  	   );
	   

   -- DCM_CLKGEN: Frequency Aligned Digital Clock Manager
   --             Spartan-6
   -- Xilinx HDL Language Template, version 12.3
	--This is used to generate the 2.34375MHz SPI clock  for the temperature sensor.
	--This clock must be less then 5MHz for proper SPI operation.
   DCM_CLKGEN_inst : DCM_CLKGEN
   generic map (
      CLKFXDV_DIVIDE => 32,       -- CLKFXDV divide value (2, 4, 8, 16, 32) (3/5)*125MHz=75Mhz /32 = 2.34375MHz
      CLKFX_DIVIDE => 5,         -- Divide value - D - (1-256) (3/5)*125MHz=75Mhz
      CLKFX_MD_MAX => 0.6,       -- Specify maximum M/D ratio for timing anlysis
      CLKFX_MULTIPLY => 3,       -- Multiply value - M - (2-256) (3/5)*125MHz=75Mhz
      CLKIN_PERIOD => 8.0,       -- Input clock period specified in nS
      SPREAD_SPECTRUM => "NONE", -- Spread Spectrum mode "NONE", "CENTER_LOW_SPREAD", "CENTER_HIGH_SPREAD",
                                 -- "VIDEO_LINK_M0", "VIDEO_LINK_M1" or "VIDEO_LINK_M2" 
      STARTUP_WAIT => FALSE      -- Delay config DONE until DCM_CLKGEN LOCKED (TRUE/FALSE)
   )
   port map (
      CLKFX => open,         -- 1-bit Generated clock output
      CLKFX180 => open,   -- 1-bit Generated clock output 180 degree out of phase from CLKFX.
      CLKFXDV => CLK_TEMPERATURE,     -- 1-bit Divided clock output 2.34375MHz SPI clock
      LOCKED => Clock_Locked2,       -- 1-bit Locked output - feedback to freeze.
      PROGDONE => open,   -- 1-bit Active high output to indicate the successful re-programming
      STATUS => open,       -- 2-bit DCM_CLKGEN status
      CLKIN => FPGA_CLK_PLL_O,   -- 1-bit Input clock 125MHz
      FREEZEDCM => Clock_Locked2, -- 1-bit Prevents frequency adjustments to input clock
      PROGCLK => '0',     -- 1-bit Clock input for M/D reconfiguration
      PROGDATA => '0',   -- 1-bit Serial data input for M/D reconfiguration
      PROGEN => '0',       -- 1-bit Active high program enable
      RST => ClockReset     -- 1-bit Reset input pin
   );

   -- End of DCM_CLKGEN_inst instantiation
				
SPI_BUFG: BUFG  -- Output from DCM_CLKGEN_inst for SPI. 
  port map (   
  	    I => CLK_TEMPERATURE,
            O => CLK_TEMPERATURE_BUF
  	   );


DPSRAM: DP_SRAM
  port map (
 --Global signal ---------------------------------------------------------------
	CLK => CLK, --local bus clock used 
    RESET => RESET,

--Right Bank SRAM pin connections ----------------------------------------------
	SRR_A=> SR_A,
	SRR1_CE0n=> SR1_CE0n, -- 1 are lower long word 0 to 31
	SRR1_OEn=>	SR1_OEn,
	SRR3_CE0n=> SR3_CE0n, --3 are high long word 32 to 63
	SRR3_OEn=>	SR3_OEn,
	SRR_BE0n=> SR_BW0n,
	SRR_BE1n=> SR_BW1n,
	SRR_BE2n=> SR_BW2n,
	SRR_BE3n=> SR_BW3n,
	SRR_BE4n=> SR_BW4n,
	SRR_BE5n=> SR_BW5n,
	SRR_BE6n=> SR_BW6n,
	SRR_BE7n=> SR_BW7n,
	SRR_ADSn=> SR_ADSn,
	SRR_PL_FTn=> SR_PL_FTn,
	SRR_CNTENn=> SR_CNTENn,
	SRR_REPEATn=> SR_REPEATn,
	SRR_ZZ=> SR_ZZ,
	SRR_INTn=> SRR_INTn,
	SRR_COLn=> SRR_COLn,
 	SRR1_R_Wn=> SR1_R_Wn,
 	SRR3_R_Wn=> SR3_R_Wn,
 	SRR_IO=>	 SR_IO, 
	SRR_IOP=>	 SR_IOP(7 downto 0),

	--for right port
 	SRR_I_Bus=> SRAM_DATA_BUS,  
	SRR_IOPar_Bus=> SRAM_Parity_BUS, --Not used
		
	SRR_SEL => SRAM_SEL,
	SRR_W_Rn_Sel => SRR_W_Rn_Sel, 
	SRR_Ack => SRAM_ACK, --Right Port read acknowledge

--Local Bus connections ------------------------------------------------------
	ADS_n=> ADS_n,
	LW_R_n=> LW_R_n,
	LD => LD,
	LBE0_n => LBE0_n,
	LBE1_n => LBE1_n, 
	LBE2_n => LBE2_n, 
	LBE3_n => LBE3_n,  
	READ_DATA => SRAM_RdData,
	SRAM_Read_Adr => SRAM_Read_Adr,
	SRAM_Read_Adr2 => SRAM_Read_Adr2,
	SRAM_CONTROL_Adr => SRAM_CONTROL_Adr,
	SRAM_IntAdr => SRAM_IntAdr,
	SRAM_DMA0Thr_Adr => SRAM_DMA0Thr_Adr,
	SRAM_DMA1Thr_Adr => SRAM_DMA1Thr_Adr,
	SRAM_Reset0_Adr => SRAM_Reset0_Adr,
	SRAM_Reset1_Adr => SRAM_Reset1_Adr,
	DMA0_REQ => SRAM_DMA0_REQ,
	DMA1_REQ => SRAM_DMA1_REQ,
	SRAM_ENABLED => SRAM_ENABLED,
	SRAM_BUSY => SRAM_BUSY,
	ADC_OVERRIDE =>ADC_OVERRIDE_EN
    );

--AXM_Module : AXM_D 
--    Port Map (
--	 --Local connections
--	CLK=> CLK,
--	RESET=> RESET,
--	ADS_n => ADS_n,
--	LW_R_n => LW_R_n,
--    LBE0_n => LBE0_n,
--	LBE1_n => LBE1_n,
--	LBE2_n => LBE2_n,
--	LBE3_n => LBE3_n,
--	LD => LD,
--	READ_DATA => AXM_RdData,
--	Int_Status => IntStatA_Reg, 
--	
--	--Address Strobes
--	DiffReg31to0_Adr => DiffReg31to0_Adr,
--	DigReg15to0_Adr => DigReg15to0_Adr,
--	DiffDirReg_Adr => DiffDirReg_Adr,
--	DigDirReg_Adr => DigDirReg_Adr,
--	Int_Enable_Adr => Int_Enable_Adr,
--	Int_Type_Adr => Int_Type_Adr,
--	Int_Polarity_Adr => Int_Polarity_Adr,
--	
--	--Write Stobes
--	Int_StatClear_Stb0 => Int_StatClear_Stb0,
--	
--	--Front I/O Connections
--	IO_DIGITAL => FIO_DIG,
--	DIR_DIG => FDIR_DIG,
----for debug 
----IO_DIFF => Temp_FIO_DIFF(29 downto 0), 
--	IO_DIFF => FIO_DIFF(29 downto 0), --FIO_DIFF(29 downto 0), 
--	DIR_DIFF => FDIR_DIFF,
--
--	--AXM module ID
--	AXM_ID => AXM_ID
--	 );

AXMModule: AXM_A30 
   port map(

   CLK => CLK, --Local clock
	RESET => RESET, 
	ADS_n => ADS_n,
	LW_R_n => LW_R_n,
   LBE0_n => LBE0_n,
	LBE1_n =>LBE1_n,
	LBE2_n => LBE2_n, 
	LBE3_n => LBE3_n, 
	LD => LD,
	READ_DATA => AXM_RdData,
	Int_Status => IntStatA_Reg, 



	ADCControl_Adr => ADCControl_Adr, 
	ADCCh1_Adr => ADCCh1_Adr, 
	ADCCh2_Adr => ADCCh2_Adr,
	ADCFifo_Adr =>ADCFifo_Adr,
	PLLControl_Adr => PLLControl_Adr,
	PLLR0_Adr => PLLR0_Adr, 
	PLLR1_Adr => PLLR1_Adr, 
	PLLR3_Adr => PLLR3_Adr, 
	PLLR4_Adr => PLLR4_Adr, 
	PLLR11_Adr => PLLR11_Adr, 
	PLLR13_Adr => PLLR13_Adr, 
	PLLR14_Adr => PLLR14_Adr, 
	PLLR15_Adr => PLLR15_Adr,
	ADC_SRAM_Adr => ADC_SRAM_Adr,
	
	AXM_ID => AXM_ID,
	
	
--ADC pin connections -------------------------------------------------------

	AD_DCS => AD_DCS,
	AD_DFS => AD_DFS,
						

  AD_CLK1_P => AD_CLK1_P,
  AD_CLK1_N  => AD_CLK1_N,
  AD_CLK2_P => AD_CLK2_P,
  AD_CLK2_N => AD_CLK2_N,
	
	AD_D1_P => AD_D1_P, 
	AD_D1_N => AD_D1_N, 
   AD_D2_P => AD_D2_P,
	AD_D2_N => AD_D2_N,
	
	Reg_AD_OTR => AD_OTR, 

-- General Purpose I/O
	GPIO => GPIO,

-- Transceivers enable/disable	
   ExtInput_En => ExtInput_En,
	ExtOutput_En => ExtOutput_En,
	ExtClkOut_En => ExtClkOut_En,

  
-- LMK03000 Clock Input Control  

   Crystal_En => Crystal_En, 
	ExtCLKIn_En => ExtCLKIn_En, 
	FPGACLKIn_En => FPGACLKIn_En, 

--LMK03000 PLL Connections --------------------------------------------------

	SerDataOut => SerDataOut,
	SerCLKOut => SerCLKOut,
	SerLatch_n => SerLatch_n,
	
	LMK_GOE =>  LMK_GOE,
	LMK_SYNCH => LMK_SYNCH,
	LOCK =>	LOCK,
  
-- Other connections
	FPGACLKIN => FPGACLKIN,
	FPGACLK_IN_EN => FPGACLK_IN_EN,
	FPGACLK_OUT =>	FPGACLK_OUT,
	CH1_PDN => CH1_PDN,
	CH2_PDN => CH2_PDN,
	UnusedFrontIO => UnusedFrontIO,
	
--Connections for writing ADC data to SRAM
	SRAM_ENABLED => SRAM_ENABLED,
	SRAM_FIFO_DATA => SRAM_FIFO_DATA,
	FIFO_EMPTY_FLAG => FIFO_EMPTY_FLAG,
	ADC_OVERRIDE_EN => ADC_OVERRIDE_EN,
	
	AXM_SERCLK => AXM_A30SerCLK_BUF,
	FIFOtoSRAMPause => FIFOtoSRAMPause,
		
   FIFODataValidFlag => FIFODataValidFlag

	);


Rear_IO : RearLVDS 
    Port Map (
	 --Local connections
	CLK=> CLK,
	RESET=> RESET,
	ADS_n => ADS_n,
	LW_R_n => LW_R_n,
	LBE0_n => LBE0_n,
	LBE1_n => LBE1_n,
	LD(15 downto 0) => LD(15 downto 0),
	READ_DATA => Rear_RdData, 
	Rear_2_SRAM => Rear_2_SRAM,
	
	--Address Strobes
	Rear_LVDS_Rd_Adr => Rear_LVDS_Rd_Adr,
	Rear_LVDS_Wr_Adr => Rear_LVDS_Wr_Adr,

	-- Rear I/O connections
   RP_IO0 => RP_IO0,
   RN_IO1 => RN_IO1,
   RP_IO2 => RP_IO2,
   RN_IO3 => RN_IO3,
   RP_IO4 => RP_IO4,
   RN_IO5 => RN_IO5,
   RP_IO6 => RP_IO6,
   RN_IO7 => RN_IO7,
   RP_IO8 => RP_IO8,
   RN_IO9 => RN_IO9,
   RP_IO10 => RP_IO10,
   RN_IO11 => RN_IO11,
   RP_IO12 => RP_IO12,
   RN_IO13 => RN_IO13,
   RP_IO14 => RP_IO14,
   RN_IO15 => RN_IO15,
   RP_IO16 => RP_IO16,
   RN_IO17 => RN_IO17,
   RP_IO18 => RP_IO18,
   RN_IO19 => RN_IO19,
   RP_IO20 => RP_IO20,
   RN_IO21 => RN_IO21,
   RP_IO22 => RP_IO22,
   RN_IO23 => RN_IO23,
   RP_IO24 => RP_IO24,
   RN_IO25 => RN_IO25,
   RP_IO26 => RP_IO26,
   RN_IO27 => RN_IO27,
   RP_IO28 => RP_IO28,
   RN_IO29 => RN_IO29,
   RP_IO30 => RP_IO30,
   RN_IO31 => RN_IO31,
   RP_IO32 => RP_IO32,
   RN_IO33 => RN_IO33,
   RP_IO34 => RP_IO34,
   RN_IO35 => RN_IO35,
   RP_IO36 => RP_IO36,
   RN_IO37 => RN_IO37,
   RP_IO38 => RP_IO38,
   RN_IO39 => RN_IO39,
   RP_IO40 => RP_IO40,
   RN_IO41 => RN_IO41,
   RP_IO42 => RP_IO42,
   RN_IO43 => RN_IO43,
   RP_IO44 => RP_IO44,
   RN_IO45 => RN_IO45,
   RP_IO46 => RP_IO46,
   RN_IO47 => RN_IO47,
   RP_IO48 => RP_IO48,
   RN_IO49 => RN_IO49,
   RP_IO50 => RP_IO50,
   RN_IO51 => RN_IO51,
   RP_IO52 => RP_IO52,
   RN_IO53 => RN_IO53,
   RP_IO54 => RP_IO54,
   RN_IO55 => RN_IO55,
   RP_IO56 => RP_IO56,
   RN_IO57 => RN_IO57,
   RP_IO58 => RP_IO58,
   RN_IO59 => RN_IO59,
   RP_IO60 => RP_IO60,
   RN_IO61 => RN_IO61,
   RP_IO62 => RP_IO62,
   RN_IO63 => RN_IO63		
	 );

Temp_sensor: SPI_Temp_Sensor 
    Port Map( 

	CLK => CLK_TEMPERATURE_BUF, --2.34375MHz clock from DCM-
	RESET => RESET,
	
	READ_DATA => TEMP_DATA, 
		
   TEMP_CS => TEMP_CS,
   TEMP_SCK => TEMP_SCK,
   TEMP_SDO=> TEMP_SDO
			  
			  );


-- Note base address 2 (BAR2) is placed on LA(26)
--   U7_Space <= I_LA(26) and (I_LA(21) or I_LA(20) or I_LA(19) or I_LA(18) or
--                          	  I_LA(17) or I_LA(16) or I_LA(15));
-- Note all these address lines will be low for a flash memory access.
	 
-- Base Address Decode Logic Follows ----------------------------------------------
   Not_Used_address_lines <= LA(25) and LA(24) and LA(23) and LA(22); -- Reserved

--I_LA(26) <= s_bar(2);  -- Note base address 2 is placed on LA(26)
-- Flash Base_Address active for address range 0000 hex to 7FFF hex
   FLash_Base_Address <=   LA(26) and
                                      not LA(21) and not LA(20) and
								      not LA(19) and not LA(18) and not LA(17) and not LA(16) and
								      not LA(15);

-- Base_Address active for address range 8000 hex to 81FF hex 
-- Note base address 2 (BAR2) is placed on LA(26)
   Base_Address <= LA(26) and 
                          not LA(21) and not LA(20) and
						  not LA(19) and not LA(18) and not LA(17) and not LA(16) and
							  LA(15) and not LA(14) and not LA(13) and not LA(12) and
						  not LA(11) and not LA(10) and not LA(9);

-- Not_Used_Space active from 8200 hex to 3FFFFF hex
   Not_Used_Space <=   LA(26) and not (Base_Address or Flash_Base_address);
                  
   U7_Address <= Base_Address or Not_Used_Space;

-- Address Decode ----------------------------------------------------------
 
  Int_StatClear_Adr	<= not LA(8) and not LA(7) and not LA(6) and not LA(5) and not LA(4) and
                        not LA(3) and not LA(2) and Base_Address;  --0x8000
--  DiffReg31to0_Adr   <= not LA(8) and not LA(7) and not LA(6) and not LA(5) and not LA(4) and
--                        not LA(3) and     LA(2) and Base_Address;  --0x8004
--  DiffDirReg_Adr     <= not LA(8) and not LA(7) and not LA(6) and not LA(5) and not LA(4) and
--                            LA(3) and not LA(2) and Base_Address;  --0x8008
--  DigReg15to0_Adr    <= not LA(8) and not LA(7) and not LA(6) and not LA(5) and not LA(4) and
--                            LA(3) and     LA(2) and Base_Address;  --0x800C
--  DigDirReg_Adr      <= not LA(8) and not LA(7) and not LA(6) and not LA(5) and     LA(4) and
--                        not LA(3) and not LA(2) and Base_Address;  --0x8010
--  Int_Enable_Adr     <= not LA(8) and not LA(7) and not LA(6) and not LA(5) and     LA(4) and
--                        not LA(3) and     LA(2) and Base_Address;  --0x8014
--  Int_Type_Adr     	<= not LA(8) and not LA(7) and not LA(6) and not LA(5) and     LA(4) and
--                            LA(3) and not LA(2) and Base_Address;  --0x8018
--  Int_Polarity_Adr   <= not LA(8) and not LA(7) and not LA(6) and not LA(5) and     LA(4) and
--                            LA(3) and     LA(2) and Base_Address;  --0x801C
--  NU 					<= not LA(8) and not LA(7) and not LA(6) and     LA(5) and not LA(4) and
--                      not LA(3) and not LA(2) and Base_Address;  --0x8020
--  NU  					<= not LA(8) and not LA(7) and not LA(6) and     LA(5) and not LA(4) and
--                      not LA(3) and     LA(2) and Base_Address;  --0x8024
--  NU 					<= not LA(8) and not LA(7) and not LA(6) and     LA(5) and not LA(4) and
--                          LA(3) and not LA(2) and Base_Address;  --0x8028
  Rear_LVDS_Rd_Adr  	<= not LA(8) and not LA(7) and not LA(6) and     LA(5) and not LA(4) and
                            LA(3) and     LA(2) and Base_Address;  --0x802C  
  Rear_LVDS_Wr_Adr   <= not LA(8) and not LA(7) and not LA(6) and     LA(5) and     LA(4) and 
                        not LA(3) and not LA(2) and Base_Address;  --0x8030  

  DMA_Control_Adr   	<= not LA(8) and not LA(7) and not LA(6) and     LA(5) and     LA(4) and
                        not LA(3) and     LA(2) and Base_Address;  --0x8034
  SRAM_Read_Adr    	<= not LA(8) and not LA(7) and not LA(6) and     LA(5) and     LA(4) and 
                            LA(3) and not LA(2) and Base_Address;  --0x8038
  SRAM_Read_Adr2    	<= not LA(8) and not LA(7) and not LA(6) and     LA(5) and     LA(4) and 
                            LA(3) and     LA(2) and Base_Address;  --0x803C
  SRAM_CONTROL_Adr  	<= not LA(8) and not LA(7)   and   LA(6) and not LA(5) and not LA(4) and
                        not LA(3) and not LA(2) and Base_Address;  --0x8040
  SRAM_IntAdr       	<= not LA(8) and not LA(7) and     LA(6) and not LA(5) and not LA(4) and
                        not LA(3) and     LA(2) and Base_Address;  --0x8044
  SRAM_DMA0Thr_Adr  	<= not LA(8) and not LA(7) and     LA(6) and not LA(5) and not LA(4) and
                            LA(3) and not LA(2) and Base_Address;  --0x8048
  SRAM_DMA1Thr_Adr  	<= not LA(8) and not LA(7) and     LA(6) and not LA(5) and not LA(4) and
                       	    LA(3) and     LA(2) and Base_Address;  --0x804C
  SRAM_Reset0_Adr   	<= not LA(8) and not LA(7) and     LA(6) and not LA(5) and     LA(4) and
                        not LA(3) and not LA(2) and Base_Address;  --0x8050
  SRAM_Reset1_Adr 	<= not LA(8) and not LA(7) and     LA(6) and not LA(5) and     LA(4) and
                       	not LA(3) and     LA(2) and Base_Address;  --0x8054
  PMC_ID_Code_Adr <=      not LA(8) and not LA(7) and     LA(6) and not LA(5) and     LA(4) and
                            LA(3) and not LA(2) and Base_Address;  --0x8058
--  DDR_Ctrl_Adr <=       not LA(8) and not LA(7) and     LA(6) and not LA(5) and     LA(4) and
--                            LA(3) and     LA(2) and Base_Address;  --0x805C
--  DDR_Addr_Adr   	<=    not LA(8) and not LA(7) and     LA(6) and     LA(5) and not LA(4) and
--                        not LA(3) and not LA(2) and Base_Address;  --0x8060
--  DDR_ReadD0_Adr 	<=    not LA(8) and not LA(7) and     LA(6) and      LA(5) and not LA(4) and
--                       	not LA(3) and     LA(2) and Base_Address;  --0x8064
--  DDR_ReadD1_Adr <=     not LA(8) and not LA(7) and     LA(6) and     LA(5) and not LA(4) and
--                            LA(3) and not LA(2) and Base_Address;  --0x8068
--  DDR_ReadD2_Adr <=     not LA(8) and not LA(7) and     LA(6) and     LA(5) and not LA(4) and
--                            LA(3) and     LA(2) and Base_Address;  --0x806C
--  DDR_ReadD3_Adr  <=    not LA(8) and not LA(7) and     LA(6) and     LA(5) and     LA(4) and
--                        not LA(3) and not LA(2) and Base_Address;  --0x8070
--  DDR_WriteD0_Adr <=    not LA(8) and not LA(7) and     LA(6) and     LA(5) and     LA(4) and
--                       	not LA(3) and     LA(2) and Base_Address; --0x8074
--  DDR_WriteD1_Adr <=    not LA(8) and not LA(7) and     LA(6) and     LA(5) and     LA(4) and
--                            LA(3) and not LA(2) and Base_Address; --0x8078
--  DDR_WriteD2_Adr <=    not LA(8) and not LA(7) and     LA(6) and     LA(5) and     LA(4) and
--                            LA(3) and     LA(2) and Base_Address; --0x807C
--  DDR_WriteD3_Adr  <=   not LA(8) and     LA(7) and not LA(6) and not LA(5) and not LA(4) and
--                        not LA(3) and not LA(2) and Base_Address; --0x8080
--  DDR_Mask_Adr  <=      not LA(8) and     LA(7) and not LA(6) and not LA(5) and not LA(4) and
--                        not LA(3) and     LA(2) and Base_Address; --0x8084
--  SysMonStatCtrl_Adr <= not LA(8) and     LA(7) and not LA(6) and not LA(5) and not LA(4) and
--                            LA(3) and not LA(2) and Base_Address  and not LBE0_n; --0x8088
--  SysMonAddrReg_Adr <=  not LA(8) and     LA(7) and not LA(6) and not LA(5) and not LA(4) and
--                            LA(3) and     LA(2) and Base_Address; --0x808C
    TEMP_SENSOR_Data_Adr<= not LA(8) and     LA(7) and not LA(6) and not LA(5) and     LA(4) and
                          not LA(3) and not LA(2) and Base_Address; --0x8090
--AXM-A30 registers							  
  ADCControl_Adr  <=        LA(8) and 
								not LA(7) and not LA(6) and not LA(5) and not LA(4) and
                        not LA(3) and not LA(2) and Base_Address; --0x8100
  ADCCh1_Adr  <=            LA(8) and 
								not LA(7) and not LA(6) and not LA(5) and not LA(4) and
                        not LA(3) and     LA(2) and Base_Address; --0x8104								
  ADCCh2_Adr  <=            LA(8) and 
								not LA(7) and not LA(6) and not LA(5) and not LA(4) and
                            LA(3) and not LA(2) and Base_Address; --0x8108
  ADCFifo_Adr <=			    LA(8) and 
								not LA(7) and not LA(6) and not LA(5) and not LA(4) and
                            LA(3) and     LA(2) and Base_Address; --0x810C
  PLLControl_Adr <=		    LA(8) and 
								not LA(7) and not LA(6) and not LA(5) and     LA(4) and
                        not LA(3) and not LA(2) and Base_Address; --0x8110
  PLLR0_Adr     <=		    LA(8) and 
								not LA(7) and not LA(6) and not LA(5) and     LA(4) and
                        not LA(3) and     LA(2) and Base_Address; --0x8114
  PLLR1_Adr     <=		    LA(8) and 
								not LA(7) and not LA(6) and not LA(5) and     LA(4) and
                            LA(3) and not LA(2) and Base_Address; --0x8118							
  PLLR3_Adr     <=		    LA(8) and 
								not LA(7) and not LA(6) and not LA(5) and     LA(4) and
                            LA(3) and     LA(2) and Base_Address; --0x811C		
  PLLR4_Adr     <=		    LA(8) and 
								not LA(7) and not LA(6) and     LA(5) and not LA(4) and
                        not LA(3) and not LA(2) and Base_Address; --0x8120											 
  PLLR11_Adr     <=		    LA(8) and 
								not LA(7) and not LA(6) and     LA(5) and not LA(4) and
                        not LA(3) and     LA(2) and Base_Address; --0x8124
  PLLR13_Adr     <=		    LA(8) and 
								not LA(7) and not LA(6) and     LA(5) and not LA(4) and
                            LA(3) and not LA(2) and Base_Address; --0x8128
  PLLR14_Adr     <=		    LA(8) and 
								not LA(7) and not LA(6) and     LA(5) and not LA(4) and
                            LA(3) and     LA(2) and Base_Address; --0x812C
  PLLR15_Adr     <=		    LA(8) and 
								not LA(7) and not LA(6) and     LA(5) and     LA(4) and
                        not LA(3) and not LA(2) and Base_Address; --0x8130
  ADC_SRAM_Adr    <=		    LA(8) and 
								not LA(7) and not LA(6) and     LA(5) and     LA(4) and
                        not LA(3) and     LA(2) and Base_Address; --0x8134						  
							  
SRAM_Strobe <= SRAM_CONTROL_Adr or SRAM_IntAdr or SRAM_DMA0Thr_Adr or SRAM_DMA1Thr_Adr or
			SRAM_Reset0_Adr or SRAM_Reset1_Adr or SRAM_Read_Adr or SRAM_Read_Adr2;
--AXM-D0x
--AXM_Strobe <= DiffReg31to0_Adr or DigReg15to0_Adr or DiffDirReg_Adr or DigDirReg_Adr or
--				Int_Enable_Adr or Int_Type_Adr or Int_Polarity_Adr;

--AXM-A30
AXM_Strobe <= ADCControl_Adr or ADCCh1_Adr or ADCCh2_Adr or ADCFifo_Adr or PLLControl_Adr or
				PLLR0_Adr or PLLR1_Adr  or PLLR3_Adr  or PLLR4_Adr or PLLR11_Adr or
				PLLR13_Adr or PLLR14_Adr or PLLR15_Adr or ADC_SRAM_Adr;


Rear_Strobe <= Rear_LVDS_Wr_Adr or Rear_LVDS_Rd_Adr;

TEMP_Sen_Strobe <= TEMP_SENSOR_DATA_Adr;



-- Write Strobes ------------------------------------------------------------
  process (CLK)
  begin
      if (CLK'event and CLK = '1') then
         Int_StatClear_Stb0 <= Int_StatClear_Adr and not ADS_n  and
	                          not LBE0_n and LW_R_n;
      end if;
  end process;

  process (CLK)
  begin
      if (CLK'event and CLK = '1') then
         Int_StatClear_Stb1 <= Int_StatClear_Adr and not ADS_n  and
	                          not LBE1_n and LW_R_n;
      end if;
  end process;
  
  process (CLK, RESET)
  begin
      if (CLK'event and CLK = '1') then
			if (RESET = '1') then
				USERoI <= '0';   
         elsif (Int_StatClear_Stb1 = '1') then
            USERoI <= LD(8);
         else
            USERoI <= USERoI;
         end if;
      end if;
  end process;
  
  USERo <= USERoI;

  process (CLK)
  begin
      if (CLK'event and CLK = '1') then
         Reset_Reg_Stb1 <= Int_StatClear_Adr and not ADS_n  and
	                      not LBE3_n and LW_R_n;
      end if;
  end process;

  process (CLK)
  begin
      if (CLK'event and CLK = '1') then
         DMA_Control_Stb0 <= DMA_Control_Adr and not ADS_n  and
	                           not LBE0_n and LW_R_n;
      end if;
  end process;

 
  -- Software Reset ---------------------------------------------
  process (CLK, Clock_Locked_Reg3)
  begin
      if (Clock_Locked_Reg3 = '0') then
         Software_Reset1 <= '0';         
         Software_Reset2 <= '0';         
         Software_Reset3 <= '0';         
      elsif (CLK'event and CLK = '1') then
         Software_Reset1 <= (Reset_Reg_Stb1 and LD(31)) or 
	           (Software_Reset1 and Int_StatClear_Adr and not LBE3_n and LW_R_n); 
         Software_Reset2 <= Software_Reset1;
         Software_Reset3 <= Software_Reset2;
      end if;
  end process;
  
  Software_Reset <= Software_Reset1 or Software_Reset2 or Software_Reset3;

  process (CLKPCIbus)
--  process (FPGA_CLK_PLL_O)
  begin
      if (CLKPCIbus'event and CLKPCIbus = '1') then
--      if (FPGA_CLK_PLL_O'event and FPGA_CLK_PLL_O = '1') then
			ClockReset <=  not LRESET_n;
			Clock_Locked_Reg1 <= Clock_Locked;
			Clock_Locked_Reg2 <= Clock_Locked_Reg1;
			Clock_Locked_Reg3 <= Clock_Locked_Reg2;
      end if;
  end process;

  process (CLK)
  begin
      if (CLK'event and CLK = '1') then
         LRESET_Reg <= not LRESET_n;
      end if;
  end process;

  RESET <= LRESET_Reg or Software_Reset or not Clock_Locked_Reg3; 
 
  -- Read Data bus for read back to PCIbus

RD_Data(0) <=
--         (SystemMonitor_out(0) and SysMonStatCtrl_Adr) or --Not used carry over from XMC-VLX
         (DMA_Request0 and DMA_Control_Adr) or
			PMC_ID_Code_Adr or --ID A3
			(SRAM_RdData(0) and SRAM_Strobe) or
--			(DDR_RdData(0) and DDR_Strobe) or --Not used carry over from XMC-VLX
  			(AXM_RdData(0) and AXM_Strobe) or
--			(IntStatA_Reg(0) and Int_StatClear_Adr) or		
			(Rear_RdData(0) and Rear_Strobe) or
			(TEMP_DATA(0) and TEMP_Sen_Strobe); 
		
RD_Data(1) <=
--         (SystemMonitor_out(1) and SysMonStatCtrl_Adr) or
         (DMA_Request1 and DMA_Control_Adr) or
			PMC_ID_Code_Adr or --ID A3
			(SRAM_RdData(1) and SRAM_Strobe) or
--			(DDR_RdData(1) and DDR_Strobe) or
			(AXM_RdData(1) and AXM_Strobe) or
--			(IntStatA_Reg(1) and Int_StatClear_Adr) or		
			(Rear_RdData(1) and Rear_Strobe) or
			(TEMP_DATA(1) and TEMP_Sen_Strobe); 

RD_Data(2) <=  
--         (SystemMonitor_out(2) and SysMonStatCtrl_Adr) or
--			PMC_ID_Code_Adr or --ID A7
			(SRAM_RdData(2) and SRAM_Strobe) or
--			(DDR_RdData(2) and DDR_Strobe) or
			(AXM_RdData(2) and AXM_Strobe) or
--			(IntStatA_Reg(2) and Int_StatClear_Adr) or		
			(Rear_RdData(2) and Rear_Strobe) or
			(TEMP_DATA(2) and TEMP_Sen_Strobe); 

RD_Data(3) <=  
--         (SystemMonitor_out(3) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(3) and SRAM_Strobe) or
--			(DDR_RdData(3) and DDR_Strobe) or
            (AXM_RdData(3) and AXM_Strobe) or 
--			(IntStatA_Reg(3) and Int_StatClear_Adr) or		
			(Rear_RdData(3) and Rear_Strobe) or
			(TEMP_DATA(3) and TEMP_Sen_Strobe); 

RD_Data(4) <=  
--         (SystemMonitor_out(4) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(4) and SRAM_Strobe) or
--			(DDR_RdData(4) and DDR_Strobe) or
			(AXM_RdData(4) and AXM_Strobe) or
--			(IntStatA_Reg(4) and Int_StatClear_Adr) or		
			(Rear_RdData(4) and Rear_Strobe) or
			(TEMP_DATA(4) and TEMP_Sen_Strobe); 

RD_Data(5) <=  
--         (SystemMonitor_out(5) and SysMonStatCtrl_Adr) or
			PMC_ID_Code_Adr or --ID A3
			(SRAM_RdData(5) and SRAM_Strobe) or
--			(DDR_RdData(5) and DDR_Strobe) or
			(AXM_RdData(5) and AXM_Strobe) or 
--			(IntStatA_Reg(5) and Int_StatClear_Adr) or		
			(Rear_RdData(5) and Rear_Strobe) or
			(TEMP_DATA(5) and TEMP_Sen_Strobe); 

RD_Data(6) <=  
--         (SystemMonitor_out(6) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(6) and SRAM_Strobe) or
--			(DDR_RdData(6) and DDR_Strobe) or
			(AXM_RdData(6) and AXM_Strobe) or 
--			(IntStatA_Reg(6) and Int_StatClear_Adr) or		
			(Rear_RdData(6) and Rear_Strobe) or
			(TEMP_DATA(6) and TEMP_Sen_Strobe); 

RD_Data(7) <=  
--         (SystemMonitor_out(7) and SysMonStatCtrl_Adr) or
			PMC_ID_Code_Adr or --ID A3
			(SRAM_RdData(7) and SRAM_Strobe) or
--			(DDR_RdData(7) and DDR_Strobe) or
			(AXM_RdData(7) and AXM_Strobe) or 
--			(IntStatA_Reg(7) and Int_StatClear_Adr) or		
			(Rear_RdData(7) and Rear_Strobe) or
			(TEMP_DATA(7) and TEMP_Sen_Strobe); 

RD_Data(8) <=  
--         (SystemMonitor_out(8) and SysMonStatCtrl_Adr) or
			(USERoI and Int_StatClear_Adr) or
			(SRAM_RdData(8) and SRAM_Strobe) or
--			(DDR_RdData(8) and DDR_Strobe) or
			(AXM_RdData(8) and AXM_Strobe) or		
			(Rear_RdData(8) and Rear_Strobe) or
			(TEMP_DATA(8) and TEMP_Sen_Strobe); 
			
RD_Data(9) <=  
--         (SystemMonitor_out(9) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(9) and SRAM_Strobe) or
--			(DDR_RdData(9) and DDR_Strobe) or
			(AXM_RdData(9) and AXM_Strobe) or		
			(Rear_RdData(9) and Rear_Strobe) or
			(IntStatA_Reg(0) and Int_StatClear_Adr) or --AXM-A30 INT 
			(TEMP_DATA(9) and TEMP_Sen_Strobe); 
			
RD_Data(10) <=  
--         (SystemMonitor_out(10) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(10) and SRAM_Strobe) or
--			(DDR_RdData(10) and DDR_Strobe) or
			(AXM_RdData(10) and AXM_Strobe) or		
			(Rear_RdData(10) and Rear_Strobe) or
			(IntStatA_Reg(1) and Int_StatClear_Adr) or --AXM-A30 INT
			(TEMP_DATA(10) and TEMP_Sen_Strobe); 
			
RD_Data(11) <=  
--         (SystemMonitor_out(11) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(11) and SRAM_Strobe) or
--			(DDR_RdData(11) and DDR_Strobe) or
			(AXM_RdData(11) and AXM_Strobe) or		
			(Rear_RdData(11) and Rear_Strobe) or
			(TEMP_DATA(11) and TEMP_Sen_Strobe); 
			
RD_Data(12) <=  
 --        (SystemMonitor_out(12) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(12) and SRAM_Strobe) or
--			(DDR_RdData(12) and DDR_Strobe) or
			(AXM_RdData(12) and AXM_Strobe) or		
			(Rear_RdData(12) and Rear_Strobe) or
			(TEMP_DATA(12) and TEMP_Sen_Strobe); 
			
RD_Data(13) <=  
--         (SystemMonitor_out(13) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(13) and SRAM_Strobe) or
	--		(DDR_RdData(13) and DDR_Strobe) or
			(AXM_RdData(13) and AXM_Strobe) or
			(AXM_ID(0) and Int_StatClear_Adr) or				
			(Rear_RdData(13) and Rear_Strobe); 
			
RD_Data(14) <=  
--         (SystemMonitor_out(14) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(14) and SRAM_Strobe) or
--			(DDR_RdData(14) and DDR_Strobe) or
			(AXM_RdData(14) and AXM_Strobe) or
			(AXM_ID(1) and Int_StatClear_Adr) or				
			(Rear_RdData(14) and Rear_Strobe); 
			
RD_Data(15) <=  
--         (SystemMonitor_out(15) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(15) and SRAM_Strobe) or
--			(DDR_RdData(15) and DDR_Strobe) or
			(AXM_RdData(15) and AXM_Strobe) or
			(AXM_ID(2) and Int_StatClear_Adr) or				
			(Rear_RdData(15) and Rear_Strobe); 
			
RD_Data(16) <=  
 --        (SystemMonitor_out(16) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(16) and SRAM_Strobe) or
--			(DDR_RdData(16) and DDR_Strobe) or
			(AXM_RdData(16) and AXM_Strobe);
			
RD_Data(17) <=  
 --        (SystemMonitor_out(17) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(17) and SRAM_Strobe) or
--			(DDR_RdData(17) and DDR_Strobe) or
			(AXM_RdData(17) and AXM_Strobe);

RD_Data(18) <=  
--         (SystemMonitor_out(18) and SysMonStatCtrl_Adr) or
 			(SRAM_RdData(18) and SRAM_Strobe) or
--			(DDR_RdData(18) and DDR_Strobe) or
			(AXM_RdData(18) and AXM_Strobe);

RD_Data(19) <=  
 --        (SystemMonitor_out(19) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(19) and SRAM_Strobe) or
--			(DDR_RdData(19) and DDR_Strobe) or
			(AXM_RdData(19) and AXM_Strobe);			

RD_Data(20) <=  
--         (SystemMonitor_out(20) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(20) and SRAM_Strobe) or
--			(DDR_RdData(20) and DDR_Strobe) or
			(AXM_RdData(20) and AXM_Strobe);
			
RD_Data(21) <=  
--         (SystemMonitor_out(21) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(21) and SRAM_Strobe) or
--			(DDR_RdData(21) and DDR_Strobe) or
			(AXM_RdData(21) and AXM_Strobe);
			
RD_Data(22) <=  
 --        (SystemMonitor_out(22) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(22) and SRAM_Strobe) or
--			(DDR_RdData(22) and DDR_Strobe) or
			(AXM_RdData(22) and AXM_Strobe);


RD_Data(23) <=  
 --        (SystemMonitor_out(23) and SysMonStatCtrl_Adr) or
 			(SRAM_RdData(23) and SRAM_Strobe) or
--			(DDR_RdData(23) and DDR_Strobe) or
			(AXM_RdData(23) and AXM_Strobe);

RD_Data(24) <=  
 --        (SystemMonitor_out(24) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(24) and SRAM_Strobe) or
	--		(DDR_RdData(24) and DDR_Strobe) or
			(AXM_RdData(24) and AXM_Strobe);
			
RD_Data(25) <=  
 --        (SystemMonitor_out(25) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(25) and SRAM_Strobe) or
--			(DDR_RdData(25) and DDR_Strobe) or
			(AXM_RdData(25) and AXM_Strobe);

RD_Data(26) <=  
 --        (SystemMonitor_out(26) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(26) and SRAM_Strobe) or
--			(DDR_RdData(26) and DDR_Strobe) or
			(AXM_RdData(26) and AXM_Strobe);

RD_Data(27) <=  
  --       (SystemMonitor_out(27) and SysMonStatCtrl_Adr) or
            (not DACK0_n and Int_StatClear_Adr) or
			(SRAM_RdData(27) and SRAM_Strobe) or
--			(DDR_RdData(27) and DDR_Strobe) or
			(AXM_RdData(27) and AXM_Strobe);

RD_Data(28) <= 
 --        (SystemMonitor_out(28) and SysMonStatCtrl_Adr) or
            (not DACK1_n and Int_StatClear_Adr) or
			(SRAM_RdData(28) and SRAM_Strobe) or
--			(DDR_RdData(28) and DDR_Strobe) or
			(AXM_RdData(28) and AXM_Strobe);

RD_Data(29) <= 
  --       (SystemMonitor_out(29) and SysMonStatCtrl_Adr) or
 --           (phy_init_done and Int_StatClear_Adr) or
			(SRAM_RdData(29) and SRAM_Strobe) or
--			(DDR_RdData(29) and DDR_Strobe) or
			(AXM_RdData(29) and AXM_Strobe);

RD_Data(30) <=  
 --        (SystemMonitor_out(30) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(30) and SRAM_Strobe) or
--			(DDR_RdData(30) and DDR_Strobe) or
			(AXM_RdData(30) and AXM_Strobe);	

RD_Data(31) <=  
 --        (SystemMonitor_out(31) and SysMonStatCtrl_Adr) or
			(SRAM_RdData(31) and SRAM_Strobe) or
--			(DDR_RdData(31) and DDR_Strobe) or
			(AXM_RdData(31) and AXM_Strobe);		
						
--- 3 State Output from FPGA to PCI U5

  --Left Port SRAM reads are directly from DP SRAM chip.  
--  READ_EN <= Base_Address and not LW_R_n;
  READ_EN <= not LW_R_n;

   process (RD_Data, READ_EN)
   begin
     if READ_EN = '1' then
       LD <= RD_Data;
     else
       LD <= (others => 'Z');
     end if;
   end process;


 ---DMA Request Logic-------------------------------------------------- 
  process (CLK, RESET)
  begin
      if (RESET = '1') then
         DMA_Request0 <= '0';         
      elsif (CLK'event and CLK = '1') then
         DMA_Request0 <= (DMA_Control_Stb0 and LD(0)) or 
		                  SRAM_DMA0_REQ or 
						  (DMA_Request0 and DACK0_n);
      end if;	
  end process;
 -- PCI U5 asserts DACK0_n to indicate DMA channel 0 transfer in process

  process (CLK, RESET)
  begin
      if (CLK'event and CLK = '1') then
			if (RESET = '1') then
				DMA_Request1 <= '0';  
			else
				DMA_Request1 <= (DMA_Control_Stb0 and LD(1)) or
				  SRAM_DMA1_REQ or (DMA_Request1 and DACK1_n);
			end if;	
		end if;
  end process;
  -- PCI U5 asserts DACK1_n to indicate DMA channel 1 transfer in process

  DREQ0_n <= not DMA_Request0; -- Asserted to request Demand mode DMA transfer
  DREQ1_n <= not DMA_Request1; -- Asserted to request Demand mode DMA transfer

-- END DMA LOGIC-------------------------------------------------------


 -- Ready Generation -----------------------------------------------------------

  process (CLK)   -- For Ready generation on all reads to Base_Address
  begin		   -- except right port SRAM.
      if (CLK'event and CLK = '1') then
         General_Read_Stb <= Base_Address and not (SRAM_Read_Adr or SRAM_Read_Adr2)--or SysMonStatCtrl_Adr)
	                        and not ADS_n and not LW_R_n and
					 (not LBE0_n or not LBE1_n or not LBE2_n or not LBE3_n);
      end if;
  end process;

  process (CLK, LRESET_n)  --For Ready generation on all Write to Base_Address except for right port SRAM.
  begin                          
   if (CLK'event and CLK = '1') then
			if (  LRESET_n = '0') then
				Int_READY <= '0';   
			else
				Int_READY <= (Base_Address and not ADS_n and LW_R_n and not (SRAM_Read_Adr or SRAM_Read_Adr2) );-- and not SysMonStatCtrl_Adr);
			end if;
		end if;
  end process;
   

  READY_RESET <= (not LRESET_n or not U7_Address); 
  
-- Hold READY active until RdyACK_n goes active.

  D_Ready <= ( Int_Ready  or (not READY and RdyACK_n) or
				   (SRAM_ACK and not (ADC_OVERRIDE_EN and SRAM_Enabled)) or  --Right port read Acknowledge signal
					General_Read_Stb or  -- Base_Address Read
				  (Not_Used_Space and not ADS_n));-- or-- Not used space Read/Write

--  READYn is pulled up on the board.

process(CLK, READY_RESET)
	begin

	if(CLK'event and CLK = '1') then
		if(READY_RESET = '1') then
			READY <= '1';
		else
			READY <= not D_Ready;
		end if;
	end if;
end process;


  READYn <= READY;
	  
  -- End Ready Signal Generation--------------------------------------------

  
    -----SRAM CONTROL LOGIC-----------------------------------------------

  SRR_W_Rn_Sel <= SRAM_ENABLED; --Read of SRAM through FPGA is not allowed
						  -- when SRAM writes are enabled.

  --SRAM Read DATA Control

  process (CLK)
  begin
      if (CLK'event and CLK = '1') then
         Start_SRAM_Read <= (SRAM_Read_Adr or SRAM_Read_Adr2) and not ADS_n and
	                           not LBE0_n and not LBE1_n and not LBE2_n 
						  and not LBE3_n and not LW_R_n;
      end if;
  end process;

 -- SRAM DATA Control ---------
 
  process (CLK)
  begin
      if (CLK'event and CLK = '1') then
         Start_SRAM_Write <= SRAM_Read_Adr and not ADS_n and
	                           not LBE0_n and not LBE1_n and not LBE2_n 
						  and not LBE3_n and LW_R_n and not ADC_OVERRIDE_EN; --ADC Override disables all writes;
      end if;
  end process;

  process (CLK)
  begin
      if (CLK'event and CLK = '1') then
         Start_SRAM_Write2 <= SRAM_Read_Adr2 and not ADS_n and
	                           not LBE0_n and not LBE1_n and not LBE2_n 
						  and not LBE3_n and LW_R_n and not ADC_OVERRIDE_EN; --ADC Override disables all writes;
      end if;
  end process;
 
  process(CLK, RESET)
  begin

  	if (CLK'event and CLK = '1') then
		if (RESET = '1') then
			SRAM_SEL <= '0';
		else
			SRAM_SEL <= Start_SRAM_Write or Start_SRAM_Write2 or (not SRAM_ENABLED and Start_SRAM_Read)
								or (ADCFIFOWrite_Reg);
		end if;
	end if;
  end process;
  
 
  --ADC FIFO Empty and ADC Override Enabled
  --SRAM_Pause <= (FIFO_EMPTY_FLAG and ADC_OVERRIDE_EN);
  FIFOtoSRAMPause <= SRAM_BUSY and Word_Sel(1); -- tell FIFO to pause reads
  
  --2 bit counter Word_Sel with hold
  Inc_Counter <= ADC_OVERRIDE_EN and FIFODataValidFlag and SRAM_ENABLED;
  Counter_reset<= RESET or (not SRAM_ENABLED);
 
 process(CLK)
  begin
	if(CLK'event and CLK = '1') then
		if (Counter_reset = '1') then
			Word_sel <= "00";
		elsif (Inc_Counter = '1') then 
			Word_sel <= Word_sel + 1;
		else
			Word_sel <= Word_sel;
		end if;
	end if;
	end process;
			
  
--  process(CLK, RESET, SRAM_PAUSE)
--  begin
--	if(RESET = '1') then
--		Word_Sel(0) <= '0';
--	elsif (CLK'event and CLK = '1') then
--		if (SRAM_PAUSE = '0') then
--			Word_Sel(0) <= (SRAM_ENABLED and not Word_Sel(0) and (ADC_OVERRIDE_EN and FIFODataValidFlag)); 
--		else
--			Word_Sel(0) <= Word_Sel(0);
--		end if;
--	end if;
--  end process;
--  
--  process(CLK, RESET, SRAM_PAUSE)
--  begin
--	if(RESET = '1') then
--		Word_Sel(1) <= '0';
--	elsif (CLK'event and CLK = '1') then
--		if (SRAM_PAUSE = '0') then
--			Word_Sel(1) <= (SRAM_ENABLED and (Word_Sel(0) xor Word_Sel(1))) and (ADC_OVERRIDE_EN and FIFODataValidFlag); 
--		else
--			Word_Sel(1) <= Word_Sel(1);
--		end if;
--	end if;
--  end process;
--  

ADC_SRAM_ENW0 <= FIFODataValidFlag and not Word_Sel(1) and not Word_Sel(0) ;
ADC_SRAM_ENW1 <= FIFODataValidFlag and not Word_Sel(1) and     Word_Sel(0) ;
ADC_SRAM_ENW2 <= FIFODataValidFlag and     Word_Sel(1) and not Word_Sel(0) ;
ADC_SRAM_ENW3 <= FIFODataValidFlag and     Word_Sel(1) and     Word_Sel(0) ;

process(CLK)
begin
	if(CLK'event and CLK = '1') then
		if (RESET = '1') then
			ADC_SRAM_ENW3_reg <= '0';
		else
			ADC_SRAM_ENW3_reg <= Word_Sel(1) and Word_Sel(0);
		end if;
	end if;
end process;
		
ADCFIFOWrite <= ADC_SRAM_ENW3_reg and (not Word_Sel(1) and not Word_Sel(0)) ;

process(CLK)
begin
	if(CLK'event and CLK = '1') then
		if (RESET = '1') then
			ADCFIFOWrite_Reg <= '0';
		else
			ADCFIFOWrite_Reg  <= ADCFIFOWrite;
		end if;
	end if;
end process;

--Create cascade 64 bit register from 4 A/D values.
 process(CLK, ADC_SRAM_ENW0)
 begin
	if (CLK'event and CLK = '1') then
		if(ADC_SRAM_ENW0 = '1') then
			SRAM_TEMP(15 downto 0) <= SRAM_FIFO_DATA;
		else
			SRAM_TEMP(15 downto 0) <= SRAM_TEMP(15 downto 0);
		end if;
	end if;
 end process;
 
 process(CLK, ADC_SRAM_ENW1)
 begin
	if (CLK'event and CLK = '1') then
		if( ADC_SRAM_ENW1 = '1') then
			SRAM_TEMP(31 downto 16) <= SRAM_FIFO_DATA;
		else
			SRAM_TEMP(31 downto 16) <= SRAM_TEMP(31 downto 16);
		end if;
	end if;
 end process;	
 
 process(CLK, ADC_SRAM_ENW2)
 begin
	if (CLK'event and CLK = '1') then
		if(ADC_SRAM_ENW2 = '1') then
			SRAM_TEMP(47 downto 32) <= SRAM_FIFO_DATA;
		else
			SRAM_TEMP(47 downto 32) <= SRAM_TEMP(47 downto 32);
		end if;
	end if;
 end process;	
 
  
 process(CLK, ADC_SRAM_ENW3)
 begin
	if (CLK'event and CLK = '1') then
		if( ADC_SRAM_ENW3 = '1') then
			SRAM_TEMP(63 downto 48) <= SRAM_FIFO_DATA;
		else
			SRAM_TEMP(63 downto 48) <= SRAM_TEMP(63 downto 48);
		end if;
	end if;
 end process;	



  process (CLK, Start_SRAM_Write, ADC_SRAM_ENW3)
  begin
       if (CLK'event and CLK = '1') then
         if (Start_SRAM_Write = '1') then
            SRAM_DATA_BUS(15 downto 0) <= LD(15 downto 0);
			elsif (ADCFIFOWrite = '1') then
		      SRAM_DATA_BUS(15 downto 0) <= SRAM_TEMP(15 downto 0);
         else
            SRAM_DATA_BUS(15 downto 0) <= SRAM_DATA_BUS(15 downto 0);
         end if;
      end if;
  end process;

  process (CLK, Start_SRAM_Write, ADC_SRAM_ENW3)
  begin
       if (CLK'event and CLK = '1') then
         if (Start_SRAM_Write = '1') then
            SRAM_DATA_BUS(31 downto 16) <= LD(31 downto 16);
			elsif (ADCFIFOWrite = '1') then
		      SRAM_DATA_BUS(31 downto 16) <= SRAM_TEMP(31 downto 16);
         else
            SRAM_DATA_BUS(31 downto 16) <= SRAM_DATA_BUS(31 downto 16);
         end if;
      end if;
  end process;

  process (CLK, Start_SRAM_Write2, ADC_SRAM_ENW3)
  begin
       if (CLK'event and CLK = '1') then
         if (Start_SRAM_Write2 = '1') then
            SRAM_DATA_BUS(47 downto 32) <= LD(15 downto 0);
			elsif (ADCFIFOWrite = '1') then
		      SRAM_DATA_BUS(47 downto 32) <= SRAM_TEMP(47 downto 32);
         else
            SRAM_DATA_BUS(47 downto 32) <= SRAM_DATA_BUS(47 downto 32);
         end if;
      end if;
  end process;

  process (CLK, Start_SRAM_Write2, ADC_SRAM_ENW3)
  begin
       if (CLK'event and CLK = '1') then
         if (Start_SRAM_Write2 = '1') then
            SRAM_DATA_BUS(63 downto 48) <= LD(31 downto 16);
			elsif (ADCFIFOWrite = '1') then
		      SRAM_DATA_BUS(63 downto 48) <= SRAM_TEMP(63 downto 48);
         else
            SRAM_DATA_BUS(63 downto 48) <= SRAM_DATA_BUS(63 downto 48);
         end if;
      end if;
  end process;
    
  SRAM_Parity_BUS <= "ZZZZZZZZ"; 
			
  --------------------------------------------------------------------------

-- Not used Front I/O ------------------				  
--   FN_VRN17 <= '0';
--	FP_VRP17 <= '0';
--   FP_VRP17 <= spare1  or spare2 or spare3;
	
--for debug	
--FP_VRP17 <= Temp_FIO_DIFF(0) or Temp_FIO_DIFF(1) or Temp_FIO_DIFF(2) or 
--					 Temp_FIO_DIFF(3) or Temp_FIO_DIFF(4) or Temp_FIO_DIFF(5) or
--					 Temp_FIO_DIFF(6) or Temp_FIO_DIFF(7) or Temp_FIO_DIFF(8) or
--					 Temp_FIO_DIFF(9) or Temp_FIO_DIFF(10) or Temp_FIO_DIFF(11) or
--					 Temp_FIO_DIFF(12) or Temp_FIO_DIFF(13) or Temp_FIO_DIFF(14) or
--					 Temp_FIO_DIFF(15) or Temp_FIO_DIFF(16) or Temp_FIO_DIFF(17) or
--					 Temp_FIO_DIFF(18) or Temp_FIO_DIFF(19) or Temp_FIO_DIFF(20) or
--					 Temp_FIO_DIFF(21) or Temp_FIO_DIFF(22) or Temp_FIO_DIFF(23) or
--					 Temp_FIO_DIFF(24) or Temp_FIO_DIFF(25) or Temp_FIO_DIFF(26) or
--					 Temp_FIO_DIFF(27) or Temp_FIO_DIFF(28) or Temp_FIO_DIFF(29)or spare1  or spare2 or spare3;

--	FP3_GC0 <= '0';
--	FN3_GC1_VREF <= '0';
--	FN_CTRL1 <= '0';
--	FN_CTRL1 <= Not_Used_address_lines;  -- Temporary use of address lines
	
-- Interrupt status from AXM_Module : AXM_D 
--  LINT_n <= not (IntStatA_Reg(0) or IntStatA_Reg(1) or IntStatA_Reg(2) or
--					IntStatA_Reg(3) or IntStatA_Reg(4) or IntStatA_Reg(5) or
--					IntStatA_Reg(6) or IntStatA_Reg(7));
-- Interrupt status from AXM_Module : AXM-A30
    LINT_n <= not (IntStatA_Reg(0) or IntStatA_Reg(1));	
				  
end XC6SLX150_arch;

