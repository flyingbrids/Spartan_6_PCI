

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

Library UNISIM;
use UNISIM.vcomponents.all;

entity RearLVDS is
    Port (
	CLK: in STD_LOGIC; --Local clock
	RESET: in STD_LOGIC; --Global Reset signal (active high)
	
	ADS_n: in STD_LOGIC;  -- Address Strobe from PCI9056
	LW_R_n: in STD_LOGIC;  -- Local Bus Write / Read driven by PCI9056
   LBE0_n: in STD_LOGIC; --Local Data Bus Byte 0 Enables active low
   LBE1_n: in STD_LOGIC; --Local Data Bus Byte 1 Enables active low
	LD : in  STD_LOGIC_VECTOR (15 downto 0); --Local Data Bus
	READ_DATA: out STD_LOGIC_VECTOR(15 downto 0); --Readback data bus of Rear I/O Registers
	Rear_2_SRAM: out STD_LOGIC_VECTOR(15 downto 0); --Rear I/O channel 0:15 to SRAM
	
	Rear_LVDS_Rd_Adr: in STD_LOGIC;  -- Rear I/O Read Address strobe
	Rear_LVDS_Wr_Adr: in STD_LOGIC;   -- Rear I/O Write Address strobe
	
   RP_IO0: in std_logic; --LVSD input only
   RN_IO1: in std_logic; --LVSD input only
   RP_IO2: out std_logic;
   RN_IO3: out std_logic;
   RP_IO4: out std_logic;
   RN_IO5: out std_logic;
   RP_IO6: in std_logic;
   RN_IO7: in std_logic;
   RP_IO8: out std_logic;
   RN_IO9: out std_logic;
   RP_IO10: in std_logic; --LVSD input only
   RN_IO11: in std_logic; --LVSD input only
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
   RP_IO30: in std_logic; --LVSD input only
   RN_IO31: in std_logic; --LVSD input only
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
   RP_IO46: in std_logic; --LVSD input only
   RN_IO47: in std_logic; --LVSD input only
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
end RearLVDS;

architecture RearLVDS_arch of RearLVDS is

-- Write Strobe signals --------------------------------------
signal Wr_Reg_Stb0 : STD_LOGIC;
signal Wr_Reg_Stb1 : STD_LOGIC;

signal Read : STD_LOGIC_VECTOR (15 downto 0);

-- Register Signals ---------------------------------------
signal Wr_Reg : STD_LOGIC_VECTOR (15 downto 0); 

begin

lvds_0: IBUFDS_LVDS_25 port map
	    ( I  => RP_IO0,
	      IB => RN_IO1,
	      O  => Read(0) 
	    );		 		 
lvds_1: OBUFDS_LVDS_25 port map
	    ( O  => RP_IO2,
	      OB => RN_IO3,
	      I  => Wr_Reg(0) 
	    );		 		 
lvds_2: OBUFDS_LVDS_25 port map
	    ( O  => RP_IO4,
	      OB => RN_IO5,
	      I  => Wr_Reg(1) 
	    );
lvds_3: IBUFDS_LVDS_25 port map
	    ( I  => RP_IO6,
	      IB => RN_IO7,
	      O  => Read(1) 
	    );
lvds_4: OBUFDS_LVDS_25 port map
	    ( O  => RP_IO8,
	      OB => RN_IO9,
	      I  => Wr_Reg(2) 
	    );
lvds_5: IBUFDS_LVDS_25 port map
	    ( I  => RP_IO10,
	      IB => RN_IO11,
	      O  => Read(2) 
	    );
lvds_6: OBUFDS_LVDS_25 port map
	    ( O  => RP_IO12,
	      OB => RN_IO13,
	      I  => Wr_Reg(3) 
	    );
lvds_7: IBUFDS_LVDS_25 port map
	    ( I  => RP_IO14,
	      IB => RN_IO15,
	      O  => Read(3) 
	    );
lvds_8: OBUFDS_LVDS_25 port map
	    ( O  => RP_IO16,
	      OB => RN_IO17,
	      I  => Wr_Reg(4) 
	    );
lvds_9: IBUFDS_LVDS_25 port map
	    ( I  => RP_IO18,
	      IB => RN_IO19,
	      O  => Read(4) 
	    );
lvds_10: OBUFDS_LVDS_25 port map
	    ( O  => RP_IO20,
	      OB => RN_IO21,
	      I  => Wr_Reg(5) 
	    );
lvds_11: IBUFDS_LVDS_25 port map
	    ( I  => RP_IO22,
	      IB => RN_IO23,
	      O  => Read(5) 
	    );
lvds_12: OBUFDS_LVDS_25 port map
	    ( O  => RP_IO24,
	      OB => RN_IO25,
	      I  => Wr_Reg(6) 
	    );
lvds_13: IBUFDS_LVDS_25 port map
	    ( I  => RP_IO26,
	      IB => RN_IO27,
	      O  => Read(6) 
	    );
lvds_14: OBUFDS_LVDS_25 port map
	    ( O  => RP_IO28,
	      OB => RN_IO29,
	      I  => Wr_Reg(7) 
	    );
lvds_15: IBUFDS_LVDS_25 port map
	    ( I  => RP_IO30,
	      IB => RN_IO31,
	      O  => Read(7) 
	    );
lvds_16: OBUFDS_LVDS_25 port map
	    ( O  => RP_IO32,
	      OB => RN_IO33,
	      I  => Wr_Reg(8) 
	    );
lvds_17: IBUFDS_LVDS_25 port map
	    ( I  => RP_IO34,
	      IB => RN_IO35,
	      O  => Read(8) 
	    );
lvds_18: OBUFDS_LVDS_25 port map
	    ( O  => RP_IO36,
	      OB => RN_IO37,
	      I  => Wr_Reg(9) 
	    );
lvds_19: IBUFDS_LVDS_25 port map
	    ( I  => RP_IO38,
	      IB => RN_IO39,
	      O  => Read(9) 
	    );
lvds_20: OBUFDS_LVDS_25 port map
	    ( O  => RP_IO40,
	      OB => RN_IO41,
	      I  => Wr_Reg(10) 
	    );
lvds_21: IBUFDS_LVDS_25 port map
	    ( I  => RP_IO42,
	      IB => RN_IO43,
	      O  => Read(10) 
	    );
lvds_22: OBUFDS_LVDS_25 port map
	    ( O  => RP_IO44,
	      OB => RN_IO45,
	      I  => Wr_Reg(11) 
	    );
lvds_23: IBUFDS_LVDS_25 port map
	    ( I  => RP_IO46,
	      IB => RN_IO47,
	      O  => Read(11) 
	    );
lvds_24: OBUFDS_LVDS_25 port map
	    ( O  => RP_IO48,
	      OB => RN_IO49,
	      I  => Wr_Reg(12) 
	    );
lvds_25: IBUFDS_LVDS_25 port map
	    ( I  => RP_IO50,
	      IB => RN_IO51,
	      O  => Read(12) 
	    );
lvds_26: OBUFDS_LVDS_25 port map
	    ( O  => RP_IO52,
	      OB => RN_IO53,
	      I  => Wr_Reg(13) 
	    );
lvds_27: IBUFDS_LVDS_25 port map
	    ( I  => RP_IO54,
	      IB => RN_IO55,
	      O  => Read(13) 
	    );
lvds_28: OBUFDS_LVDS_25 port map
	    ( O  => RP_IO56,
	      OB => RN_IO57,
	      I  => Wr_Reg(14) 
	    );
lvds_29: IBUFDS_LVDS_25 port map
	    ( I  => RP_IO58,
	      IB => RN_IO59,
	      O  => Read(14) 
	    );
lvds_30: OBUFDS_LVDS_25 port map
	    ( O  => RP_IO60,
	      OB => RN_IO61,
	      I  => Wr_Reg(15) 
	    );
lvds_31: IBUFDS_LVDS_25 port map
	    ( I  => RP_IO62,
	      IB => RN_IO63,
	      O  => Read(15) 
	    );

-- Write Strobes ----------------------------------------------
  
  process (CLK)
  begin
      if (CLK'event and CLK = '1') then
         Wr_Reg_Stb0 <= Rear_LVDS_Wr_Adr and not ADS_n  and
	                        not LBE0_n and LW_R_n;
      end if;
  end process;

  process (CLK)
  begin
      if (CLK'event and CLK = '1') then
         Wr_Reg_Stb1 <= Rear_LVDS_Wr_Adr and not ADS_n  and
	                        not LBE1_n and LW_R_n;
      end if;
  end process;

   -- Registers ------------------------------------------------

  --Write Rear I/O Register 0x8030

  process (CLK, RESET)
  begin
      if (CLK'event and CLK = '1') then
			if (RESET = '1') then
				Wr_Reg(7 downto 0) <= "00000000";    
         elsif (Wr_Reg_Stb0 = '1') then
            Wr_Reg(7 downto 0) <= LD(7 downto 0);
         else
            Wr_Reg(7 downto 0) <= Wr_Reg(7 downto 0);
         end if;
      end if;
  end process;
  
  process (CLK, RESET)
  begin
     if (CLK'event and CLK = '1') then
			if (RESET = '1') then
				Wr_Reg(15 downto 8) <= "00000000"; 
         elsif (Wr_Reg_Stb1 = '1') then
            Wr_Reg(15 downto 8) <= LD(15 downto 8);
         else
            Wr_Reg(15 downto 8) <= Wr_Reg(15 downto 8);
         end if;
      end if;
  end process;
    
  Rear_2_SRAM <= Read(15 downto 0);
      
   -- Read Data bus for read back to PLX PCI9656

READ_DATA(0) <= (Read(0) and Rear_LVDS_Rd_Adr) or
                (Wr_Reg(0) and Rear_LVDS_Wr_Adr);
READ_DATA(1) <= (Read(1) and Rear_LVDS_Rd_Adr) or
                (Wr_Reg(1) and Rear_LVDS_Wr_Adr);
READ_DATA(2) <= (Read(2) and Rear_LVDS_Rd_Adr) or
                (Wr_Reg(2) and Rear_LVDS_Wr_Adr);
READ_DATA(3) <= (Read(3) and Rear_LVDS_Rd_Adr) or
                (Wr_Reg(3) and Rear_LVDS_Wr_Adr);
READ_DATA(4) <= (Read(4) and Rear_LVDS_Rd_Adr) or
                (Wr_Reg(4) and Rear_LVDS_Wr_Adr);
READ_DATA(5) <= (Read(5) and Rear_LVDS_Rd_Adr) or
                (Wr_Reg(5) and Rear_LVDS_Wr_Adr);
READ_DATA(6) <= (Read(6) and Rear_LVDS_Rd_Adr) or
                (Wr_Reg(6) and Rear_LVDS_Wr_Adr);
READ_DATA(7) <= (Read(7) and Rear_LVDS_Rd_Adr) or
                (Wr_Reg(7) and Rear_LVDS_Wr_Adr);
READ_DATA(8) <= (Read(8) and Rear_LVDS_Rd_Adr) or
                (Wr_Reg(8) and Rear_LVDS_Wr_Adr);
READ_DATA(9) <= (Read(9) and Rear_LVDS_Rd_Adr) or
                (Wr_Reg(9) and Rear_LVDS_Wr_Adr);
READ_DATA(10) <= (Read(10) and Rear_LVDS_Rd_Adr) or
                (Wr_Reg(10) and Rear_LVDS_Wr_Adr);
READ_DATA(11) <= (Read(11) and Rear_LVDS_Rd_Adr) or
                (Wr_Reg(11) and Rear_LVDS_Wr_Adr);
READ_DATA(12) <= (Read(12) and Rear_LVDS_Rd_Adr) or
                (Wr_Reg(12) and Rear_LVDS_Wr_Adr);
READ_DATA(13) <= (Read(13) and Rear_LVDS_Rd_Adr) or
                (Wr_Reg(13) and Rear_LVDS_Wr_Adr);
READ_DATA(14) <= (Read(14) and Rear_LVDS_Rd_Adr) or
                (Wr_Reg(14) and Rear_LVDS_Wr_Adr);
READ_DATA(15) <= (Read(15) and Rear_LVDS_Rd_Adr) or
                (Wr_Reg(15) and Rear_LVDS_Wr_Adr);
			
end RearLVDS_arch;

