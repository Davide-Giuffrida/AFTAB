-- **************************************************************************************
--	Filename:	aftab_testbench.vhd
--	Project:	CNL_RISC-V
--  Version:	1.0
--	History:
--	Date:		1 June 2021
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
--	Testbench for the AFTAB core
--
-- **************************************************************************************

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;

ENTITY aftab_testbench IS
END aftab_testbench;

ARCHITECTURE behavior OF aftab_testbench IS
	-- Core inputs
	SIGNAL clk                      : STD_LOGIC := '0';
	SIGNAL rst                      : STD_LOGIC := '0';
	SIGNAL memReady1                : STD_LOGIC := '0';
	SIGNAL memReady2                : STD_LOGIC := '0';
	SIGNAL memDataIn2               : STD_LOGIC_VECTOR(15 DOWNTO 0) := (OTHERS => 'Z');
	SIGNAL platformInterruptSignals : STD_LOGIC_VECTOR(15 DOWNTO 0) := (OTHERS => '0');
	SIGNAL machineExternalInterrupt : STD_LOGIC := '0';
	SIGNAL machineTimerInterrupt    : STD_LOGIC := '0';
	SIGNAL machineSoftwareInterrupt : STD_LOGIC := '0';
	SIGNAL userExternalInterrupt    : STD_LOGIC := '0';
	SIGNAL userTimerInterrupt       : STD_LOGIC := '0';
	SIGNAL userSoftwareInterrupt    : STD_LOGIC := '0';

	-- Core outputs
	SIGNAL memRead1            : STD_LOGIC;
	SIGNAL memRead2            : STD_LOGIC;
	SIGNAL memWrite            : STD_LOGIC;
	SIGNAL interruptProcessing : STD_LOGIC;
	SIGNAL memAddr1            : STD_LOGIC_VECTOR(31 DOWNTO 0) := (OTHERS => 'Z');
	SIGNAL memAddr2            : STD_LOGIC_VECTOR(31 DOWNTO 0) := (OTHERS => 'Z');
	SIGNAL memDataOut1         : STD_LOGIC_VECTOR(15 DOWNTO 0) := (OTHERS => 'Z');
	SIGNAL memDataOut2         : STD_LOGIC_VECTOR(15 DOWNTO 0) := (OTHERS => 'Z');
	SIGNAL bytesPort1		   : STD_LOGIC;
	SIGNAL bytesPort2		   : STD_LOGIC;
 

	SIGNAL log_en : STD_LOGIC := '0'; -- this signal is set up by run.tcl at the end of the simulation to dump memory

	CONSTANT clk_period : TIME := 30 ns;

BEGIN

	-- Instantiate the Unit Under Test (UUT)
	core : ENTITY WORK.aftab_core
		PORT MAP(
			clk                      => clk,
			rst                      => rst,
			memReady1       	     => memReady1,
			memReady2       	     => memReady2,
			memDataOut1              => memDataOut1,
			memDataOut2              => memDataOut2,
			memDataIn2               => memDataIn2,
			memRead1                 => memRead1,
			memRead2                 => memRead2,
			memWrite                 => memWrite,
			memAddr1                 => memAddr1,
			memAddr2                 => memAddr2,
			bytesPort1				 => bytesPort1,
			bytesPort2				 => bytesPort2,

			machineExternalInterrupt => machineExternalInterrupt, 
			machineTimerInterrupt    => machineTimerInterrupt,   
			machineSoftwareInterrupt => machineSoftwareInterrupt, 
			userExternalInterrupt    => userExternalInterrupt,   
			userTimerInterrupt       => userTimerInterrupt,      
			userSoftwareInterrupt    => userSoftwareInterrupt,   
			platformInterruptSignals => platformInterruptSignals, 
			interruptProcessing      => interruptProcessing
		);
	
	memory : ENTITY WORK.aftab_memory
		PORT MAP(
			clk           			 => clk,
			rst           			 => rst,
			readMem1      			 => memRead1,
			readMem2     			 => memRead2,
			writeMem2     			 => memWrite,
			addressBus1				 => memAddr1,
			addressBus2				 => memAddr2,
			dataIn2       			 => memDataIn2,
			dataOut1      			 => memDataOut1,
			dataOut2      			 => memDataOut2,
			log_en 		  			 => log_en,
			ready1  	  			 => memReady1, -- for the first port (used only to read instructions)
			ready2  	 			 => memReady2, -- for the second port (both for reading/writing data)
			bytesPort1				 => '1',
			bytesPort2				 => bytesPort2
		);

	clk_process : PROCESS
	BEGIN
		clk <= '0';
		WAIT FOR clk_period/2;
		clk <= '1';
		WAIT FOR clk_period/2;
	END PROCESS;
			
	-- Stimulus process
	stim_proc : PROCESS
	BEGIN
		-- hold reset state for 100 ns.
		WAIT FOR 120 ns;
		rst <= '1';
		WAIT FOR 40 ns;
		rst <= '0';
		-- uncomment this and set at proper time if you want to set interrupts signals on at some point
		WAIT FOR 100000 ns;
		machineExternalInterrupt <= '1';
		WAIT FOR 10000 ns;
		machineExternalInterrupt <= '0';
		WAIT FOR 30000 ns;
		userExternalInterrupt <= '1';
		WAIT FOR 10000 ns;
		userExternalInterrupt <= '0';
		WAIT FOR 30000 ns;
		userSoftwareInterrupt <= '1';
		WAIT FOR 10000 ns;
		userSoftwareInterrupt <= '0';
		WAIT FOR 30000 ns;
		userTimerInterrupt <= '1';
		WAIT FOR 10000 ns;
		userTimerInterrupt <= '0';
		WAIT FOR 30000 ns;
		machineSoftwareInterrupt <= '1';
		WAIT FOR 10000 ns;
		machineSoftwareInterrupt <= '0';
		WAIT FOR 30000 ns;
		machineTimerInterrupt <= '1';
		WAIT FOR 10000 ns;
		machineTimerInterrupt <= '0';
		--platformInterruptSignals <= X"0001";
		WAIT;

	END PROCESS;

END behavior;