-- **************************************************************************************
--	Filename:	aftab_daru_controller.vhd
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
--	Controller of the Data Adjustment Read Unit (DARU) of the AFTAB core
--
-- **************************************************************************************

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
ENTITY aftab_daru_controller IS
	PORT
	(
		clk          : IN  STD_LOGIC;
		rst          : IN  STD_LOGIC;
		sync_rst	 : IN  STD_LOGIC;
		dataInstrBar : IN  STD_LOGIC;
		startDARU    : IN  STD_LOGIC;
		coCnt        : IN  STD_LOGIC;
		memReady     : IN  STD_LOGIC;
		initCnt      : OUT STD_LOGIC;
		ldAddr       : OUT STD_LOGIC;
		zeroAddr     : OUT STD_LOGIC;
		zeroNumBytes : OUT STD_LOGIC;
		initReading  : OUT STD_LOGIC;
		ldNumBytes   : OUT STD_LOGIC;
		selldEn      : OUT STD_LOGIC;
		readMem      : OUT STD_LOGIC;
		enableAddr   : OUT STD_LOGIC;
		enableData   : OUT STD_LOGIC;
		incCnt       : OUT STD_LOGIC;
		zeroCnt      : OUT STD_LOGIC;
		select_incoming_data : OUT STD_LOGIC;
		completeDARU : OUT STD_LOGIC
	);
END ENTITY aftab_daru_controller;
--
ARCHITECTURE behavioral OF aftab_daru_controller IS
	TYPE state IS (waitforStart, waitforMemready, complete);
	SIGNAL pstate, nstate : state;
BEGIN
	PROCESS (pstate, startDARU, coCnt, memReady, dataInstrBar) BEGIN
		nstate <= waitforStart;
		CASE pstate IS
			WHEN waitforStart =>
				IF (startDARU = '1') THEN
					nstate <= waitforMemready;
				ELSE
					nstate <= waitforStart;
				END IF;
			WHEN waitforMemready =>
				IF ((coCnt AND memReady) = '1' AND startDARU = '1' AND dataInstrBar = '0') THEN
					nstate <= waitForMemReady;
				ELSIF (coCnt = '1' AND memReady = '1') THEN
					nstate <= complete;
				ELSE
					nstate <= waitforMemready;
				END IF;
			WHEN complete =>
				-- you restart the DARU immediately only if the DARU fetches instructions, otherwise
				-- you would restart fetching the same data you have just finished fetching!
				IF (startDARU = '1' AND dataInstrBar = '0') THEN
					nstate <= waitforMemready;
				ELSIF (dataInstrBar = '1') THEN -- DARU2 (for data)
					nstate <= complete;
				ELSE
					nstate <= waitforStart;
				END IF;
			WHEN OTHERS =>
				nstate <= waitforStart;
		END CASE;
	END PROCESS;
	PROCESS (pstate, startDARU, coCnt, memReady) BEGIN
		initCnt      <= '0';
		ldAddr       <= '0';
		zeroCnt      <= '0';
		zeroAddr     <= '0';
		zeroNumBytes <= '0';
		ldNumBytes   <= '0';
		selldEn      <= '0';
		readMem      <= '0';
		enableAddr   <= '0';
		enableData   <= '0';
		incCnt       <= '0';
		completeDARU <= '0';
		initReading  <= '0';
		select_incoming_data <= '0';
		CASE pstate IS
			WHEN waitforStart =>
				initCnt     <= startDARU;
				ldAddr      <= startDARU;
				ldNumBytes  <= startDARU;
				initReading <= startDARU;
				zeroAddr	<= NOT (startDARU); -- added to avoid blocking an excepting instruction
				-- zeroAddr	<= NOT (startDARU); -- added to avoid blocking an excepting instruction
			WHEN waitforMemready =>
				selldEn    <= memReady;
				enableData <= memReady;
				readMem    <= '1';
				enableAddr <= '1';
				incCnt     <= memReady;
				-- check if the FSM is going to transition to the complete state: if we are considering
				-- the DARU1 we have to return to waitForMemReady if there is another instruction waiting 
				IF (coCnt = '1' AND memReady = '1' AND dataInstrBar = '0') THEN
					completeDARU <= '1';
					select_incoming_data <= '1';
				END IF;
				IF ((coCnt AND memReady) = '1' AND startDARU = '1' AND dataInstrBar = '0') THEN
					initCnt     <= startDARU;
					ldAddr      <= startDARU;
					ldNumBytes  <= startDARU;
					initReading <= startDARU;
				END IF;
			WHEN complete =>
				completeDARU <= '1';
				-- in case you have to pass directly to the waitforMemReady
				IF (startDARU = '1' AND dataInstrBar = '0') THEN
					initCnt     <= startDARU;
					ldAddr      <= startDARU;
					ldNumBytes  <= startDARU;
					initReading <= startDARU;
				END IF;
			WHEN OTHERS =>
				initCnt      <= '0';
				ldAddr       <= '0';
				zeroCnt      <= '0';
				zeroAddr     <= '0';
				zeroNumBytes <= '0';
				ldNumBytes   <= '0';
				selldEn      <= '0';
				readMem      <= '0';
				enableAddr   <= '0';
				enableData   <= '0';
				enableData   <= '0';
				incCnt       <= '0';
				completeDARU <= '0';
				initReading  <= '0';
		END CASE;
	END PROCESS;
	PROCESS (clk, rst) BEGIN
		IF (rst = '1') THEN
			pstate <= waitforStart;
		ELSIF (clk = '1' AND clk'event) THEN
			IF (sync_rst = '1') THEN
				pstate <= waitforStart;
			ELSE
				pstate <= nstate;
			END IF;
		END IF;
	END PROCESS;
END ARCHITECTURE behavioral;
