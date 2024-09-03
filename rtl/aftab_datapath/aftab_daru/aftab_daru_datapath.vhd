-- **************************************************************************************
--	Filename:	aftab_daru_datapath.vhd
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
--	Datapath of the Data Adjustment Read Unit (DARU) of the AFTAB core
--
-- **************************************************************************************

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
-- use ieee.std_logic_unsigned.all;
USE IEEE.NUMERIC_STD.ALL;

ENTITY aftab_daru_datapath IS
	GENERIC
		(len : INTEGER := 32);
	PORT
	(
		clk                 : IN  STD_LOGIC;
		rst                 : IN  STD_LOGIC;
		nBytes              : IN  STD_LOGIC_VECTOR (1 DOWNTO 0);
		initValueCnt        : IN  STD_LOGIC_VECTOR (1 DOWNTO 0);
		addrIn              : IN  STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
		memData             : IN  STD_LOGIC_VECTOR ((len/2) - 1 DOWNTO 0);
		select_incoming_data: IN  STD_LOGIC;
		zeroAddr            : IN  STD_LOGIC;
		ldAddr              : IN  STD_LOGIC;
		selldEn             : IN  STD_LOGIC;
		zeroNumBytes        : IN  STD_LOGIC;
		ldNumBytes          : IN  STD_LOGIC;
		zeroCnt             : IN  STD_LOGIC;
		incCnt              : IN  STD_LOGIC;
		initCnt             : IN  STD_LOGIC;
		initReading         : IN  STD_LOGIC;
		enableAddr          : IN  STD_LOGIC;
		enableData          : IN  STD_LOGIC;
		dataInstrBar        : IN  STD_LOGIC;
		checkMisalignedDARU : IN  STD_LOGIC;
		instrMisalignedFlag : OUT STD_LOGIC;
		loadMisalignedFlag  : OUT STD_LOGIC;
		coCnt               : OUT STD_LOGIC;
		dataOut             : OUT STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
		addrOut             : OUT STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
		readAddrOut			: OUT STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
		bytesToRead			: OUT STD_LOGIC
	);
END ENTITY aftab_daru_datapath;
--
ARCHITECTURE behavioral OF aftab_daru_datapath IS
	SIGNAL readAddr   : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL readAddrP  : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL dataIn     : STD_LOGIC_VECTOR (15 DOWNTO 0);
	SIGNAL outCnt     : STD_LOGIC_VECTOR (0 DOWNTO 0); -- FIXME: TO BE FIXED, REPLACE IT WITH A STD_LOGIC
	SIGNAL byteCnt    : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL outCnt_ext : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL nBytesOut  : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL outDecoder : STD_LOGIC_VECTOR (3 DOWNTO 0);
	SIGNAL dataOutHigh_reg : STD_LOGIC_VECTOR (15 DOWNTO 0);
	SIGNAL bytesToRead_temp : STD_LOGIC;
BEGIN
	dataIn <= memData WHEN enableData = '1' ELSE (OTHERS => 'Z');
	-- addrReg
	addrReg : ENTITY WORK.aftab_register
		GENERIC
		MAP(len => 32)
		PORT MAP
		(
			clk    => clk,
			rst    => rst,
			zero   => zeroAddr,
			load   => ldAddr,
			inReg  => addrIn,
			outReg => readAddr);
	-- to echo to the output the address of the instruction being read
	readAddrOut	<= readAddr;

	-- FIXME: TO BE CLEANED (AS BEFORE)
	outCnt_ext <= '0' & outCnt;
	byteCnt <= outCnt & '0';
	-- Decoder
	decoder : ENTITY WORK.aftab_decoder
		PORT
	MAP(
	inDecoder  => outCnt_ext,
	En         => selldEn,
	outDecoder => outDecoder);
	-- nByte Register
	nByteReg : ENTITY WORK.aftab_register
		GENERIC
		MAP(len => 2)
		PORT
		MAP(
		clk    => clk,
		rst    => rst,
		zero   => zeroNumBytes,
		load   => ldNumBytes,
		inReg  => nBytes,
		outReg => nBytesOut);
	-- bytesToRead is set to '0' when we have to read one byte per memory access (lbu,lb)
	bytesToRead_temp <= '0' when nBytesOut="00" else '1';
	bytesToRead <= bytesToRead_temp;
	-- Counter
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
	-- dataReg
	Reg0 : ENTITY WORK.aftab_register
		GENERIC
		MAP(len => 16)
		PORT
		MAP(
		clk    => clk,
		rst    => rst,
		zero   => initReading,
		load   => outDecoder (0),
		inReg  => dataIn,
		outReg => dataOut (15 DOWNTO 0));
	Reg1 : ENTITY WORK.aftab_register
		GENERIC
		MAP(len => 16)
		PORT
		MAP(
		clk    => clk,
		rst    => rst,
		zero   => initReading,
		load   => outDecoder (1),
		inReg  => dataIn,
		outReg => dataOutHigh_reg);

	-- mux to select either the data coming from register or the one read from memory
	dataOut (31 DOWNTO 16) <= dataIn WHEN select_incoming_data = '1' ELSE dataOutHigh_reg;
	Adder : ENTITY WORK.aftab_opt_adder
		GENERIC
		MAP(len => 32)
		PORT
		MAP(
		A   => readAddr,
		B   => byteCnt,
		Sum => readAddrP);
	coCnt <= '1' WHEN (outCnt(0) = bytesToRead_temp) ELSE '0';
	errorDecoder : ENTITY work.aftab_daru_error_detector
		GENERIC
		MAP(len => 32)
		PORT
		MAP(
		nBytes              => nBytes,
		-- addrIn              => addrIn(1 DOWNTO 0),
		addrIn              => readAddr(1 DOWNTO 0), -- in this way the instrMisalignedFlag is kept raised until the end of computation
		dataInstrBar        => dataInstrBar,
		checkMisalignedDARU => checkMisalignedDARU,
		instrMisalignedFlag => instrMisalignedFlag,
		loadMisalignedFlag  => loadMisalignedFlag);

	-- Tri-State
	addrOut <= readAddrP WHEN enableAddr = '1' ELSE (OTHERS => 'Z');
END ARCHITECTURE behavioral;
