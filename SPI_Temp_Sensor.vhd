----------------------------------------------------------------------------------
-- Company: Acromag	
-- Engineer: CAB
-- 
-- Create Date:    10:54:35 11/03/2010 
-- Design Name: 
-- Module Name:    SPI_Temp_Sensor - Behavioral 
-- Project Name: 
-- Target Devices: Spartan 6 SLX150
-- Tool versions: ISE 12.3
-- Description: SPI Interface for temperature sensor on XMC-SLX150.
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity SPI_Temp_Sensor is
    Port ( 

	CLK: in STD_LOGIC; --2.34375MHz clock from DCM-
	RESET: in STD_LOGIC; --Global Reset signal (active high)
	
	READ_DATA: out STD_LOGIC_VECTOR(12 downto 0); --Readback data bus of Registers
		
   TEMP_CS : out  STD_LOGIC; --SPI Chip select Signal
   TEMP_SCK : out  STD_LOGIC;--SPI Clock Signal. Must be <=5MHz
   TEMP_SDO : in  STD_LOGIC -- SPI Data Input.  
			  
			  );
end SPI_Temp_Sensor;

architecture SPI_Temp_Sensor_arch of SPI_Temp_Sensor is

type state is (InitSt, WaitSt, PrepSt, ReadSt, LatchSt);  --state machine 
signal current_state, next_state : state := InitSt; 

signal delay_count: integer range 2250000 downto 0 := 0; --0.96s delay counter
signal data_count: integer range 15 downto 0 := 0; --count 16 SPI bits

signal Ser_data: STD_LOGIC_VECTOR(15 downto 0);  -- Serial Data
signal Latch_data: STD_LOGIC_VECTOR(15 downto 0); -- Latched Serial Data Bus for reading
signal WaitOver, ReadEn, W1EN, Read_complete: STD_LOGIC; --state machine inputs.

signal CSOut: STD_LOGIC;

begin


-- SPI continuous read state machine for 
--1st state: reset
--2nd state: wait 0.96 seconds
--3rd state: Enable SPI chip select.
--4th state: read 16 bit value.
--5th state: latch temp data, then return to 2nd state.

state_mach: process (CLK, current_state, WaitOver, Read_Complete)
begin
	if(clk'event and clk = '1') then
	case current_state is
		when INITst =>
			W1EN <= '0';
			CSOut <= '1';
			ReadEN <= '0';
			Latch_DATA <= (others => '0');
			
			next_state <= WaitSt;
			
		when WaitSt =>
			W1EN <= '1';
			CSOut <= '1';
			ReadEN <= '0';
			Latch_DATA <= Latch_DATA;
			
			if WaitOver = '1' then
				next_state <= PrepSt;
			else
				next_state <= next_state;
			end if;
			
		when PrepSt =>
			W1EN <= '0';
			CSOut <= '0'; --disable CS one clock cycle early
			ReadEN <= '0';
			Latch_DATA <= Latch_DATA;
			
			next_state <= ReadSt;
			
		when ReadSt =>
			W1EN <= '0';
			CSOut <= '0';
			ReadEN <= '1';
			Latch_DATA <= Latch_DATA;
			
			if(read_complete = '1') then
				next_state <= LatchSt;
			else
				next_state <= 	next_state;
			end if;
			
		when LatchSt =>
			W1EN <= '0';
			CSOut <= '1';
			ReadEN <= '0';
			Latch_DATA <= Ser_Data;
			
			next_state <= WaitSt;
			
		end case;
	end if;
end process;

process(RESET, next_state)
begin
		if(RESET = '1') then
			current_state <= INITst;
		else
			current_state <= next_state;
		end if;
end process;
 
 
 -------Wait State operations--------
--counter for 0.96 second delay from clock.
--0.4267us * 2250000 = 0.96s
process(CLK, W1EN)
begin
	if(CLK'event and CLK = '1') then
		if W1EN = '1' then
			delay_count <= delay_count + 1;
		else
			delay_count <= 0;
		end if;
	end if;
end process;


--Generate 0.96 second delay from clock.
process(CLK, delay_count)
begin
	if(CLK'event and CLK = '1') then
		if (delay_count = 2250000) then 
			WaitOver <= '1';
		else
			WaitOver <= '0';
		end if;
	end if;
end process;
	
-------Read State Operations-------

--shift serial data in
process(CLK, Reset, CSOut)
begin
	if (CLK'event and CLK = '1') then
		if (Reset = '1') then
			Ser_Data <= (others => '0');
		elsif (CSOut = '0') then
			Ser_Data(15 downto 1) <= Ser_Data(14 downto 0);
			Ser_Data(0) <= TEMP_SDO;
		end if;
	end if;
end process;


--count data values
process(CLK, CSOut)
begin
	if(CLK'event and CLK='1') then
		if (CSOut = '0') then
			data_count <= data_count + 1;
		else
			data_count <= 0;
		end if;
	end if;
end process;

--flag when 16 bits are read.
process(CLK, data_count)
begin
	if(CLK'event and CLK = '1') then
		if(data_count = 14) then
			Read_complete <= '1';
		else
			Read_complete <= '0';
		end if;
	end if;
end process;

----SPI output signals---------

TEMP_SCK <= ReadEN and CLK; --Clock active for 16 clock cycles
TEMP_CS <= CSOut; --chip select --must be active low for read

----Temperate Data for register-------------
READ_DATA(12 downto 0) <= Latch_data(15 downto 3);


end SPI_Temp_Sensor_arch;

