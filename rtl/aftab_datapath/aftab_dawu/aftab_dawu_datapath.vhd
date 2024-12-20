-- **************************************************************************************
--	Filename:	aftab_dawu_datapath.vhd
--	Project:	CNL_RISC-V
--  Version:	1.0
--	Date:		25 March 2022
--
-- Copyright (C) 2022 CINI Cybersecurity National Laboratory and University of Tehran
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
--	Datapath of the Data Adjustment Write Unit (DAWU) of the AFTAB core
--
-- **************************************************************************************

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
-- use ieee.std_logic_unsigned.all;
USE IEEE.NUMERIC_STD.ALL;

ENTITY aftab_dawu_datapath IS
	GENERIC
		(len : INTEGER := 32);
	PORT
	(
		clk                 : IN  STD_LOGIC;
		rst                 : IN  STD_LOGIC;
		ldData              : IN  STD_LOGIC;
		enableData          : IN  STD_LOGIC;
		enableAddr          : IN  STD_LOGIC;
		incCnt              : IN  STD_LOGIC;
		zeroCnt             : IN  STD_LOGIC;
		initCnt             : IN  STD_LOGIC;
		ldNumBytes          : IN  STD_LOGIC;
		zeroNumBytes        : IN  STD_LOGIC;
		ldAddr              : IN  STD_LOGIC;
		zeroAddr            : IN  STD_LOGIC;
		zeroData            : IN  STD_LOGIC;
		nBytesIn            : IN  STD_LOGIC_VECTOR (1 DOWNTO 0);
		initValueCnt        : IN  STD_LOGIC_VECTOR (1 DOWNTO 0);
		dataIn              : IN  STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
		addrIn              : IN  STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
		checkMisalignedDAWU : IN  STD_LOGIC;
		storeMisalignedFlag : OUT STD_LOGIC;
		coCnt               : OUT STD_LOGIC;
		dataOut             : OUT STD_LOGIC_VECTOR (15 DOWNTO 0);
		addrOut             : OUT STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
		bytesToWrite		: OUT STD_LOGIC
	);
END ENTITY aftab_dawu_datapath;
--
ARCHITECTURE Behavioral OF aftab_dawu_datapath IS
	SIGNAL muxOut     : STD_LOGIC_VECTOR (15 DOWNTO 0);
	SIGNAL outReg0    : STD_LOGIC_VECTOR (15 DOWNTO 0);
	SIGNAL outReg1    : STD_LOGIC_VECTOR (15 DOWNTO 0);
	SIGNAL addrOutReg : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL writeAddr  : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL nBytesOut  : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL outCnt     : STD_LOGIC_VECTOR (0 DOWNTO 0);
	SIGNAL byteCnt     : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL outCnt_ext : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL bytesToWrite_temp : STD_LOGIC;
	SIGNAL totalByteCnt : STD_LOGIC;
BEGIN
	--Register Part
	nBytesReg : ENTITY WORK.aftab_register
		GENERIC
		MAP(len => 2)
		PORT MAP
		(
			clk    => clk,
			rst    => rst,
			zero   => zeroNumBytes,
			load   => ldNumBytes,
			inReg  => nBytesIn,
			outReg => nBytesOut);
		
	bytesToWrite_temp <= '0' when nBytesOut="00" else '1';
	totalByteCnt <= '0' when nBytesOut/="11" else '1';
	bytesToWrite <= bytesToWrite_temp;
	addrReg : ENTITY WORK.aftab_register
		GENERIC
		MAP(len => len)
		PORT
		MAP(
		clk    => clk,
		rst    => rst,
		zero   => zeroAddr,
		load   => ldAddr,
		inReg  => addrIn,
		outReg => addrOutReg);
	reg0 : ENTITY WORK.aftab_register
		GENERIC
		MAP(len => 16)
		PORT
		MAP(
		clk    => clk,
		rst    => rst,
		zero   => zeroData,
		load   => ldData,
		inReg  => dataIn (15 DOWNTO 0),
		outReg => outReg0);
	reg1 : ENTITY WORK.aftab_register
		GENERIC
		MAP(len => 16)
		PORT
		MAP(
		clk    => clk,
		rst    => rst,
		zero   => zeroData,
		load   => ldData,
		inReg  => dataIn (31 DOWNTO 16),
		outReg => outReg1);

	-- outcnt extended
	outCnt_ext <= '0' & outCnt;
	byteCnt <= outCnt & '0';

	--Counter Part
	Counter : ENTITY WORK.aftab_counter
		GENERIC
		MAP(len => 1)
		PORT
		MAP(
		clk       => clk,
		rst       => rst,
		zeroCnt   => zeroCnt,
		incCnt    => incCnt,
		initCnt   => initCnt,
		initValue => initValueCnt(0 DOWNTO 0),
		outCnt    => outCnt,
		coCnt     => OPEN);
	muxOut <= outReg0 WHEN outCnt(0) = '0' ELSE
		outReg1;
	coCnt <= '1' WHEN (outCnt(0) = totalByteCnt) ELSE '0';
	Adder : ENTITY WORK.aftab_opt_adder
		GENERIC
		MAP(len => 32)
		PORT
		MAP(
		A   => addrOutReg,
		B   => byteCnt,
		Sum => writeAddr);
	errorDecoder : ENTITY work.aftab_dawu_error_detector
		GENERIC
		MAP (len => len)
		PORT
		MAP (
		nBytes              => nBytesIn,
		addrIn              => addrIn (1 DOWNTO 0),
		checkMisalignedDAWU => checkMisalignedDAWU,
		storeMisalignedFlag => storeMisalignedFlag
		);
	addrOut <= writeAddr WHEN (enableAddr = '1') ELSE (OTHERS => 'Z');
	dataOut <= muxOut WHEN (enableData = '1') ELSE(OTHERS     => 'Z');
END ARCHITECTURE Behavioral;
