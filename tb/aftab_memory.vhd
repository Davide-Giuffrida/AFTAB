-- **************************************************************************************
--	Filename:	aftab_memory.vhd
--	Project:	CNL_RISC-V
--  Version:	1.0
--	Date:		29 March 2022
--
-- Copyright (C) 2022 CINI Cybersecurity National Laboratory and University of Teheran
--
-- This source file may be used and distributed without
-- restriction provided that this copyright statement is not
-- removed from the file and that any derivative work contains
-- the original copyright notice and the associated disclaimer.
--
-- This source file is free software; you can redistribute it
-- and/or modify it under the terms of the GNU Lesser General
-- Public License as published by the Free Software Foundation;
-- either version 3.0 of the License, or (at your option) any
-- later version.
--
-- This source is distributed in the hope that it will be
-- useful, but WITHOUT ANY WARRANTY; without even the implied
-- warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
-- PURPOSE. See the GNU Lesser General Public License for more
-- details.
--
-- You should have received a copy of the GNU Lesser General
-- Public License along with this source; if not, download it
-- from https://www.gnu.org/licenses/lgpl-3.0.txt
--
-- **************************************************************************************
--
--	File content description:
--	Unique memory entity for the AFTAB core
--
-- **************************************************************************************

-- TODO: ADD THE CIRCUITRY TO PERFORM A READ OPERATION OVER A SINGLE BYTE (FILL THE MSB WITH 0s).

LIBRARY IEEE;
LIBRARY STD;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE IEEE.STD_LOGIC_TEXTIO.ALL;
USE STD.TEXTIO.ALL;


ENTITY aftab_memory IS
	GENERIC (
		dataWidth      : INTEGER := 16;
		addressWidth   : INTEGER := 32;
		actual_address : INTEGER := 13;
		size           : INTEGER := 2**actual_address -- 2^12 for data and 2^12 for instr, 4 K each
	);  
	PORT (
		clk           : IN  STD_LOGIC;
		rst           : IN  STD_LOGIC;
		readMem1      : IN  STD_LOGIC;
		readMem2      : IN  STD_LOGIC;
		writeMem2     : IN  STD_LOGIC;
		addressBus1   : IN  STD_LOGIC_VECTOR (addressWidth - 1 DOWNTO 0);
		addressBus2   : IN  STD_LOGIC_VECTOR (addressWidth - 1 DOWNTO 0);
		dataIn2       : IN  STD_LOGIC_VECTOR (dataWidth - 1 DOWNTO 0);
		dataOut1      : OUT STD_LOGIC_VECTOR (dataWidth - 1 DOWNTO 0);
		dataOut2      : OUT STD_LOGIC_VECTOR (dataWidth - 1 DOWNTO 0);
		log_en 		  : IN 	STD_LOGIC;
		ready1  	  : OUT STD_LOGIC; -- for the first port (used only to read instructions)
		ready2  	  : OUT STD_LOGIC; -- for the second port (both for reading/writing data)
		bytesPort1	  : IN STD_LOGIC;
		bytesPort2    : IN STD_LOGIC
	);
END aftab_memory;

ARCHITECTURE behavioral OF aftab_memory IS

	TYPE mem_type IS ARRAY (0 TO size - 1) OF STD_LOGIC_VECTOR (7 DOWNTO 0);
	SIGNAL mem : MEM_TYPE;

	-- Memory boundaries - change this according to the linker script: sw/ref/link.common.ld
	CONSTANT base_iram : INTEGER := 16#00000#; 
	CONSTANT end_iram : INTEGER := 16#FFFFF#;
	
	CONSTANT base_dram : INTEGER := 16#100000#; 
	CONSTANT end_dram : INTEGER := 16#100600#;

	CONSTANT base_dram_actual : INTEGER := 16#1000#;
	CONSTANT size_dram : INTEGER := 16#0FFF#;
	
BEGIN

RW : PROCESS(rst, clk, writeMem2, readMem1, readMem2, addressBus1, addressBus2, log_en)

		VARIABLE adr                 : STD_LOGIC_VECTOR(actual_address-1 DOWNTO 0);
		VARIABLE memline             : LINE;
		VARIABLE memline_log         : LINE;
		VARIABLE err_check           : FILE_OPEN_STATUS;
		VARIABLE linechar	         : CHARACTER;
		VARIABLE read_address	     : STD_LOGIC_VECTOR (31 DOWNTO 0);
		VARIABLE read_data           : STD_LOGIC_VECTOR (31 DOWNTO 0);
		FILE f                       : TEXT;
		FILE f_log                   : TEXT;
		variable index 				 : INTEGER:=0;	

	BEGIN

		dataOut1 <= (OTHERS => 'Z');
		dataOut2 <= (OTHERS => 'Z');

		IF rst = '1' THEN
			dataOut1 <= (OTHERS => 'Z');
			dataOut2 <= (OTHERS => 'Z');
			ready1 <= '1';
			ready2 <= '1';
			-- Load memory content from file
			mem <= (OTHERS => (OTHERS => '0'));
			FILE_OPEN(err_check, f, ("/home/davide/tesi/aftab/tb/slm_files/spi_stim.txt"), READ_MODE);
			IF err_check = open_ok THEN
				WHILE NOT ENDFILE (f) LOOP
					READLINE (f, memline);
					HREAD (memline, read_address);
					READ (memline, linechar); -- read character '_' 
					HREAD (memline, read_data);
					IF UNSIGNED(read_address) > end_iram THEN -- it is a data address (see file link.common.ld)
						adr := '1' & read_address(actual_address-2 DOWNTO 0);
					ELSE -- it is a program address
						adr := '0' & read_address(actual_address-2 DOWNTO 0);
					END IF;
					mem(TO_INTEGER(UNSIGNED(adr))) 	   <= read_data(7 DOWNTO 0);
					mem(TO_INTEGER(UNSIGNED(adr) + 1)) <= read_data(15 DOWNTO 8);
					mem(TO_INTEGER(UNSIGNED(adr) + 2)) <= read_data(23 DOWNTO 16);
					mem(TO_INTEGER(UNSIGNED(adr) + 3)) <= read_data(31 DOWNTO 24);
				END LOOP;
				FILE_CLOSE (f);
			ELSE
				assert 0=1 report "non existent file" severity failure;
			end if;
		END IF;
		IF log_en = '1' THEN

			FILE_OPEN(err_check, f_log, ("./slm_files/dram_dump.txt"), WRITE_MODE);
			index:=base_dram;
			WHILE index < (base_dram+size_dram-1) LOOP
				write(memline_log,to_hstring(to_signed(index, 32)),right,8);
				write(memline_log,'_',right,1);
				hwrite(memline_log, mem(index-base_dram+base_dram_actual+3)&mem(index-base_dram+base_dram_actual+2)&mem(index-base_dram+base_dram_actual+1)&mem(index-base_dram+base_dram_actual), right, 8);
      			writeline(f_log, memline_log);
				index:=index+4;				
			END LOOP;

			FILE_CLOSE (f_log);
		END IF;	
		IF (readMem1 = '1' OR readMem2 = '1') THEN
			-- reading for DARU1
			IF (readMem1 = '1') THEN
				IF UNSIGNED(addressBus1) > end_iram THEN -- data address
					adr := '1' & addressBus1(actual_address-2 DOWNTO 0);
				ELSE -- instruction address
					adr := '0' & addressBus1(actual_address-2 DOWNTO 0);
				END IF;
				dataOut1(7 DOWNTO 0)  <= mem(TO_INTEGER(UNSIGNED(adr)));
				-- read another byte only if the reading op has to be performed over 2 bytes
				IF (bytesPort1 = '1') THEN
					dataOut1(15 DOWNTO 8) <= mem(TO_INTEGER(UNSIGNED(adr)) + 1);
				ELSE
					dataOut1(15 DOWNTO 8) <= (OTHERS => '0');
				END IF;
			END IF;
			-- reading for DARU2
			IF (readMem2 = '1') THEN
				IF UNSIGNED(addressBus2) > end_iram THEN -- data address
					adr := '1' & addressBus2(actual_address-2 DOWNTO 0);
				ELSE -- instruction address
					adr := '0' & addressBus2(actual_address-2 DOWNTO 0);
				END IF;
				dataOut2(7 DOWNTO 0)  <= mem(TO_INTEGER(UNSIGNED(adr)));
				-- read another byte only if the reading op has to be performed over 2 bytes
				IF (bytesPort2 = '1') THEN
					dataOut2(15 DOWNTO 8) <= mem(TO_INTEGER(UNSIGNED(adr)) + 1);
				ELSE
					dataOut2(15 DOWNTO 8) <= (OTHERS => '0');
				END IF;
			END IF;

			IF (adr = "0011100111000") THEN
				assert false report "test done" severity failure;
			END IF;
			
		END IF;
		IF writeMem2 = '1' and falling_edge(clk) THEN
			IF UNSIGNED(addressBus2) > end_iram THEN 
				adr := '1' & addressBus2(actual_address-2 DOWNTO 0);
				mem(TO_INTEGER(UNSIGNED(adr))) <= dataIn2(7 DOWNTO 0);
				-- write another byte only if the reading op has to be performed over 2 bytes
				IF (bytesPort2 = '1') THEN
					mem(TO_INTEGER(UNSIGNED(adr)) + 1) <= dataIn2(15 DOWNTO 8);
				END IF;
			END IF;
			-- writing on instruction portion is inhibited
		END IF;
	
	END PROCESS;

END behavioral;
