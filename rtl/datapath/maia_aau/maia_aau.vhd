-- **************************************************************************************
--	Filename:	maia_aau.vhd
--	Project:	TETRISC 
--  Version:	1.0
--	History:	
--	Date:		16 February 2021
--
-- Copyright (C) 2021 CINI Cybersecurity National Laboratory and University of Teheran
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
--	Attached arithmetic unit of the TETRISC Maia core
--
-- **************************************************************************************

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;

ENTITY maia_aau IS
	GENERIC (n: INTEGER := 32);
	PORT ( 
		clk,rst : STD_LOGIC;
		ain  : IN STD_LOGIC_VECTOR (n-1 DOWNTO 0);
		bin  : IN STD_LOGIC_VECTOR (n-1 DOWNTO 0);
		startMultAAU, startDivideAAU : IN STD_LOGIC;
		signedSigned, signedUnsigned, unsignedUnsigned : IN STD_LOGIC;
		resAAU1, resAAU2: OUT STD_LOGIC_VECTOR (n-1 DOWNTO 0);
		completeAAU : OUT STD_LOGIC
	);
END ENTITY maia_aau;

ARCHITECTURE behavioral OF maia_aau IS 
	SIGNAL resMult : STD_LOGIC_VECTOR(2*n+1 DOWNTO 0);
	SIGNAL resMultL, resMultH   : STD_LOGIC_VECTOR(n-1 DOWNTO 0);
	SIGNAL quotient, Remainder  : STD_LOGIC_VECTOR(n-1 DOWNTO 0);
	SIGNAL in1Mult, in2Mult     : STD_LOGIC_VECTOR (n DOWNTO 0);
	SIGNAL doneMult, doneDiv    : STD_LOGIC;
	SIGNAL signedUnsignedbarDiv : STD_LOGIC;

BEGIN

	completeAAU <= doneMult OR doneDiv;
	in1Mult <= ('0' & ain)  WHEN (unsignedUnsigned = '1') ELSE (ain(n-1) & ain) 
							WHEN (signedSigned ='1' OR signedUnsigned='1');
	in2Mult <= ('0' & bin)  WHEN (unsignedUnsigned = '1' OR signedUnsigned='1') ELSE 
		   (bin (n-1) & bin) WHEN (signedSigned ='1');
		   
	Multiplication : ENTITY work.maia_booth_multiplier 
					    GENERIC MAP (len => 33)
						PORT MAP (clk => clk, rst => rst, startBooth => startMultAAU,
						M => in1Mult, Q => in2Mult, P => resMult, doneBooth => doneMult);
	resMultL <= resMult (n-1 DOWNTO 0);
	resMultH <= resMult (2*n-1 DOWNTO n);

	signedUnsignedbarDiv <= '1' WHEN (signedSigned = '1') ELSE '0' WHEN (unsignedUnsigned = '1');
	Division : ENTITY work.maia_su_divider
				   GENERIC MAP (len => 32)
				   PORT MAP (clk => clk, rst => rst, startSDiv => startDivideAAU, signedUnsignedbar => signedUnsignedbarDiv,
				   dividend => ain, divisor => bin,  doneSDiv => doneDiv, Qout => quotient, Remout => Remainder); 
						
	resAAU1 <= resMultH  WHEN (doneMult='1') ELSE 
			   quotient  WHEN (doneDiv='1')  ELSE (OTHERS => 'Z'); 
	resAAU2 <= resMultL  WHEN (doneMult='1') ELSE 
			   Remainder WHEN (doneDiv='1')  ELSE (OTHERS => 'Z');
			   
END ARCHITECTURE behavioral;


