-- **************************************************************************************
--	Filename:	aftab_controller.vhd
--	Project:	CNL_RISC-V
--  Version:	1.0
--	Date:		31 March 2022
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
--	Main controller of the AFTAB core
--
-- **************************************************************************************

-- TODO: REMEMBER SETTING int_ex='1' WHEN YOU NEED TO FORCE mirrorUser to mirrorUserCU

LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
-- use ieee.std_logic_unsigned.all;
USE WORK.constants.ALL;
ENTITY aftab_controller IS
	GENERIC
		(len : INTEGER := 32);
	PORT
	(
		-- general signals
		clk                            : IN  STD_LOGIC;
		rst                            : IN  STD_LOGIC;

		-- operation complete signals
		completedDAWU                  : IN STD_LOGIC;
		completedDARU1                 : IN STD_LOGIC;
		completedDARU2                 : IN STD_LOGIC;
		completedAAU                   : IN STD_LOGIC;
		is_AAU_used					   : IN STD_LOGIC;
		instructionDone				   : IN STD_LOGIC;
		int_ex 						   : IN STD_LOGIC;
		hazard_solved 				   : IN STD_LOGIC;
		is_store_in_mem				   : IN STD_LOGIC;
		branch_taken 				   : IN STD_LOGIC;
		DEC_valid					   : IN STD_LOGIC;
		EX_valid					   : IN STD_LOGIC;
		M_valid						   : IN STD_LOGIC;
		WB_valid					   : IN STD_LOGIC;
		WB_ret_from_epc				   : IN STD_LOGIC;

		-- instruction to be decoded
		IR                             : IN  STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
		
		-- control word
		writeRegFile                   : OUT STD_LOGIC;
		setOne                         : OUT STD_LOGIC;
		setZero                        : OUT STD_LOGIC;
		ComparedSignedUnsignedBar      : OUT STD_LOGIC;
		selPC                          : OUT STD_LOGIC;
		selI4                          : OUT STD_LOGIC;
		selJL                          : OUT STD_LOGIC;
		selBSU                         : OUT STD_LOGIC;
		selLLU                         : OUT STD_LOGIC;
		selASU                         : OUT STD_LOGIC;
		selAAU                         : OUT STD_LOGIC;
		selP1                          : OUT STD_LOGIC;
		selP2                          : OUT STD_LOGIC;
		selImm                         : OUT STD_LOGIC;
		ldByteSigned                   : OUT STD_LOGIC;
		ldHalfSigned                   : OUT STD_LOGIC;
		load                           : OUT STD_LOGIC;
		selShift                       : OUT STD_LOGIC_VECTOR (1 DOWNTO 0);
		addSubBar                      : OUT STD_LOGIC;
		pass                           : OUT STD_LOGIC;
		selAuipc                       : OUT STD_LOGIC;
		muxCode                        : OUT STD_LOGIC_VECTOR (11 DOWNTO 0);
		selLogic                       : OUT STD_LOGIC_VECTOR (1 DOWNTO 0);
		startDAWU                      : OUT STD_LOGIC;
		startDARU                 	   : OUT STD_LOGIC;
		startMultiplyAAU               : OUT STD_LOGIC;
		startDivideAAU                 : OUT STD_LOGIC;
		signedSigned                   : OUT STD_LOGIC;
		signedUnsigned                 : OUT STD_LOGIC;
		unsignedUnsigned               : OUT STD_LOGIC;
		selAAL                         : OUT STD_LOGIC;
		selAAH                         : OUT STD_LOGIC;
		dataInstrBar                   : OUT STD_LOGIC;
		nBytes                         : OUT STD_LOGIC_VECTOR (1 DOWNTO 0);
		selCSR                         : OUT STD_LOGIC;
		selImmCSR                      : OUT STD_LOGIC;
		selP1CSR                       : OUT STD_LOGIC;
		selReadWriteCSR                : OUT STD_LOGIC;
		clrCSR                         : OUT STD_LOGIC;
		setCSR                         : OUT STD_LOGIC;
		writeRB_inst				   : OUT STD_LOGIC; -- the signal which has to be set to perform a write op over the RB
		checkMisalignedDAWU            : OUT STD_LOGIC;
		selCSRAddrFromInst             : OUT STD_LOGIC;
		forced_RB_read				   : OUT STD_LOGIC; -- mux selection signal between the address to be read from write-back (as exception handling) and the one to be read in decode
		inst_type					   : OUT STD_LOGIC_VECTOR (2 DOWNTO 0);
		ret_from_epc				   : OUT STD_LOGIC;

		-- Interrupts
		interruptRaise                 : IN  STD_LOGIC;
		exceptionRaise                 : IN  STD_LOGIC;
		ecallFlag                      : OUT STD_LOGIC;
		illegalInstrFlag               : OUT STD_LOGIC;
		validAccessCSR                 : IN  STD_LOGIC;		
		readOnlyCSR                    : IN  STD_LOGIC;
		mirror                         : IN  STD_LOGIC;
		ldMieReg                       : IN  STD_LOGIC;
		ldMieUieField                  : IN  STD_LOGIC;
		delegationMode                 : IN  STD_LOGIC_VECTOR (1 DOWNTO 0);
		previousPRV                    : IN  STD_LOGIC_VECTOR (1 DOWNTO 0);
		modeTvec                       : IN  STD_LOGIC_VECTOR (1 DOWNTO 0);
		mipCCLdDisable                 : OUT STD_LOGIC;
		selCCMip_CSR                   : OUT STD_LOGIC;
		selCause_CSR                   : OUT STD_LOGIC;
		selPC_CSR                      : OUT STD_LOGIC;
		selTval_CSR                    : OUT STD_LOGIC;
		selMedeleg_CSR                 : OUT STD_LOGIC;
		selMideleg_CSR                 : OUT STD_LOGIC;
		ldValueCSR                     : OUT STD_LOGIC_VECTOR (2 DOWNTO 0);
		ldCntCSR                       : OUT STD_LOGIC;
		dnCntCSR                       : OUT STD_LOGIC;
		upCntCSR                       : OUT STD_LOGIC;
		ldDelegation                   : OUT STD_LOGIC;
		ldMachine                      : OUT STD_LOGIC;
		ldUser                         : OUT STD_LOGIC;
		loadMieReg                     : OUT STD_LOGIC;
		loadMieUieField                : OUT STD_LOGIC;
		mirrorUserCU                   : OUT STD_LOGIC;
		writeRegBank                   : OUT STD_LOGIC;
		selRomAddress                  : OUT STD_LOGIC;
		selMepc_CSR                    : OUT STD_LOGIC;
		selInterruptAddressDirect      : OUT STD_LOGIC;
		selInterruptAddressVectored    : OUT STD_LOGIC;
		machineStatusAlterationPreCSR  : OUT STD_LOGIC;
		userStatusAlterationPreCSR     : OUT STD_LOGIC;
		machineStatusAlterationPostCSR : OUT STD_LOGIC;
		userStatusAlterationPostCSR    : OUT STD_LOGIC;
		zeroCntCSR                     : OUT STD_LOGIC;

		-- pipeline registers
		GI2D_en						   : IN STD_LOGIC;
		GI2D_rst					   : IN STD_LOGIC;
		D2E_en						   : IN STD_LOGIC;
		D2E_rst					       : IN STD_LOGIC;
		E2M_en						   : IN STD_LOGIC;
		E2M_rst						   : IN STD_LOGIC;
		M2WB_en						   : IN STD_LOGIC;
		M2WB_rst					   : IN STD_LOGIC;

		-- hazards
		hazEX						   : IN STD_LOGIC;
		hazM						   : IN STD_LOGIC
	);
END aftab_controller;
--
ARCHITECTURE behavioral OF aftab_controller IS
	SIGNAL p_state, n_state      : state;
	SIGNAL func3                 : STD_LOGIC_VECTOR(2 DOWNTO 0);
	SIGNAL func7, opcode         : STD_LOGIC_VECTOR(6 DOWNTO 0);
	SIGNAL func12                : STD_LOGIC_VECTOR(11 DOWNTO 0);
	SIGNAL mretOrUretBar         : STD_LOGIC;
	-- TYPE state IS (idle, -- for all the instructions except the ones which need int/ex management                                                                                                                                            --LUI, AUIPC
	-- 	checkDelegation, updateTrapValue, updateMip, updateUip, updateCause, readMstatus, 
	-- 					 updateMstatus, updateUstatus, updateEpc, readTvec1, readTvec2, --Interrupt Entry States                                                                                                                      -- CSR Instructions
	-- 	retEpc, retReadMstatus, retUpdateMstatus, retUpdateUstatus,  --Interrupt Exit States                                                                                     --mret and uret instructions
	-- 	hazEX, hazM, -- for hazard management
	--  waitForStore                                                                                                                                            --ecall instruction
	-- );
BEGIN
	func3           <= IR(14 DOWNTO 12);
	func7           <= IR(31 DOWNTO 25);
	func12          <= IR(31 DOWNTO 20);
	opcode          <= IR(6 DOWNTO 0);
	mretOrUretBar   <= IR(29);
	StateTransition : PROCESS (p_state, completedDARU1, completedDARU2, is_AAU_used, instructionDone
							   completedDAWU, completedAAU, opcode, func3, func7, 
							   func12, exceptionRaise, interruptRaise, 
							   modeTvec, mirror, mretOrUretBar, 
							   previousPRV, delegationMode, ldMieUieField, ldMieReg, 
							   validAccessCSR, readOnlyCSR
							   ) 
	BEGIN
		n_state <= idle;
		CASE p_state IS
				--fetch
			WHEN idle =>
				-- you can transition to checkDelegation, hazEX, hazM, retEpc or remain in idle
				IF (WB_ret_from_epc = '1' AND WB_valid = '1') THEN
					n_state <= retEpc;
				ELSIF (interruptRaise = '1' OR exceptionRaise = '1') THEN
					-- TODO: CHECK IF YOU ARE RETURNING FROM AN EXCEPTION: IN CASE YOU ARE, YOU NEED TO TRANSITION TO retEpc
					n_state <= checkDelegation;
				ELSIF (hazEX = '1') THEN
					n_state <= hazEX;
				ELSIF (hazM = '1') THEN
					n_state <= hazM;
				END IF;
			WHEN checkDelegation =>
				IF (interruptRaise = '1') THEN
					n_state <= updateMip;
				ELSE
					n_state <= updateTrapValue;
				END IF;
			WHEN updateTrapValue =>
				n_state <= updateCause;
			WHEN updateMip =>
				n_state <= updateUip;
			WHEN updateUip =>
				n_state <= updateCause;
			WHEN updateCause =>
				n_state <= updateEpc;
			WHEN updateEpc =>
				n_state <= readTvec1;
			WHEN readTvec1 =>
				n_state <= readTvec2;
			WHEN readTvec2 =>
				n_state <= readMstatus;
			WHEN readMstatus =>
				n_state <= updateMstatus;
			WHEN updateMstatus =>
				n_state <= updateUstatus;
			WHEN updateUstatus =>
				n_state <= idle;
			WHEN retEpc =>
				n_state <= retReadMstatus;
			WHEN retReadMstatus =>
				n_state <= retUpdateMstatus;
			WHEN retUpdateMstatus =>
				n_state <= retUpdateUstatus;
			WHEN retUpdateUstatus =>
				n_state <= idle;
			WHEN hazEX =>
				-- you can transition to hazEX, idle or checkDelegation
				IF (interruptRaise = '1' OR exceptionRaise = '1') THEN
					n_state <= checkDelegation;
				ELSIF (hazard_solved = '1') THEN
					n_state <= idle;
				ELSE
					n_state <= hazEX;
				END IF;
			WHEN hazM =>
				-- you can transition to hazM, idle or checkDelegation
				IF (interruptRaise = '1' OR exceptionRaise = '1') THEN
					n_state <= checkDelegation;
				ELSIF (hazard_solved = '1') THEN
					n_state <= idle;
				ELSE
					n_state <= hazM;
				END IF;
			WHEN waitForStore =>
				-- at the end of interrupt handling, if the operation in memory is a store
				IF (completedDAWU = '1')  THEN
					n_state <= idle;
				ELSE
					n_state <= waitForStore;
				END IF;
			WHEN OTHERS => 
				n_state <= idle;
			-- WHEN fetch =>
			-- 	IF (exceptionRaise = '1' OR interruptRaise = '1') THEN
			-- 		n_state <= checkDelegation;
			-- 	ELSIF (instrMisalignedOut = '1') THEN
			-- 		n_state <= fetch;
			-- 	ELSE
			-- 		n_state <= getInstr;
			-- 	END IF;
			-- WHEN getInstr =>
			-- 	IF (completeDARU = '1') THEN
			-- 		n_state <= decode;
			-- 	ELSE
			-- 		n_state <= getInstr;
			-- 	END IF;
			-- 	--decode

			-- TODO: TO BE PLACED BELOW, IN THE PROCESS WHICH PRODUCES ALL THE CONTROL SIGNALS FOR EACH INSTRUCTION
			-- WHEN decode =>
			-- 	
			-- 	--load

			-- WHEN loadInstr1 =>
			-- 	n_state <= loadInstr2;
			-- WHEN loadInstr2 =>
			-- 	IF (loadMisalignedOut = '1') THEN
			-- 		n_state <= fetch;
			-- 	ELSE
			-- 		n_state <= getData;
			-- 	END IF;
			-- WHEN getData =>
			-- 	IF (completeDARU = '1') THEN
			-- 		n_state <= fetch;
			-- 	ELSE
			-- 		n_state <= getData;
			-- 	END IF;
			-- 	--store
			-- WHEN storeInstr1 =>
			-- 	n_state <= storeInstr2;
			-- WHEN storeInstr2 =>
			-- 	IF (storeMisalignedOut = '1') THEN
			-- 		n_state <= fetch;
			-- 	ELSE
			-- 		n_state <= putData;
			-- 	END IF;
			-- WHEN putData =>
			-- 	IF (completeDAWU = '1') THEN
			-- 		n_state <= fetch;
			-- 	ELSE
			-- 		n_state <= putData;
			-- 	END IF;
			-- 	--addSub
			-- WHEN addSub =>
			-- 	n_state <= fetch;
			-- 	--setCompare
			-- WHEN compare =>
			-- 	n_state <= fetch;
			-- 	--logical
			-- WHEN logical =>
			-- 	n_state <= fetch;
			-- 	--shift
			-- WHEN shift =>
			-- 	n_state <= fetch;
			-- 	--multiplyDivide
			-- WHEN multiplyDivide1 =>
			-- 	IF (dividedByZeroOut = '1') THEN
			-- 		n_state <= fetch;
			-- 	ELSE
			-- 		n_state <= multiplyDivide2;
			-- 	END IF;
			-- WHEN multiplyDivide2 =>
			-- 	IF (completeAAU = '1') THEN
			-- 		n_state <= fetch;
			-- 	ELSE
			-- 		n_state <= multiplyDivide2;
			-- 	END IF;
			-- 	--JAL
			-- WHEN JAL =>
			-- 	n_state <= fetch;
			-- 	--JALR
			-- WHEN JALR =>
			-- 	n_state <= fetch;
			-- 	--conditionalBranch
			-- WHEN conditionalBranch =>
			-- 	n_state <= fetch;
			-- WHEN LUI =>
			-- 	n_state <= fetch;
			-- WHEN CSR =>
			-- 	IF (mirror = '0') THEN
			-- 		n_state <= fetch;
			-- 	ELSE
			-- 		n_state <= mirrorCSR;
			-- 	END IF;
			-- WHEN mirrorCSR =>
			-- 	n_state <= fetch;
		END CASE;
	END PROCESS;
	ControlSignalsDecoder : PROCESS (p_state, completedDARU1, completedDARU2, is_AAU_used, instructionDone
							   completedDAWU, completedAAU, opcode, func3, func7, 
							   func12, exceptionRaise, interruptRaise, 
							   modeTvec, mirror, mretOrUretBar, 
							   previousPRV, delegationMode, ldMieUieField, ldMieReg, 
							   validAccessCSR, readOnlyCSR
							   ) 
	BEGIN

		-- initialize all the control signals
		writeRegFile                   <= '0';
		setOne                         <= '0';
		setZero                        <= '0';
		ComparedSignedUnsignedBar      <= '0';
		selPC                          <= '0';
		selI4                          <= '0';
		selJL                          <= '0';
		selBSU                         <= '0';
		selLLU                         <= '0';
		selASU                         <= '0';
		selAAU                         <= '0';
		selP1                          <= '0';
		selP2                          <= '0';
		selImm                         <= '0';
		ldByteSigned                   <= '0';
		ldHalfSigned                   <= '0';
		load                           <= '0';
		selShift                       <= (OTHERS => '0');
		addSubBar                      <= '0';
		pass                           <= '0';
		selAuipc                       <= '0';
		muxCode                        <= (OTHERS => '0');
		selLogic                       <= (OTHERS => '0');
		startDAWU                      <= '0';
		startDARU                 	   <= '0';
		startMultiplyAAU               <= '0';
		startDivideAAU                 <= '0';
		signedSigned                   <= '0';
		signedUnsigned                 <= '0';
		unsignedUnsigned               <= '0';
		selAAL                         <= '0';
		selAAH                         <= '0';
		dataInstrBar                   <= '0';
		nBytes                         <= (OTHERS => '0');
		selCSR                         <= '0';
		selImmCSR                      <= '0';
		selP1CSR                       <= '0';
		selReadWriteCSR                <= '0';
		clrCSR                         <= '0';
		setCSR                         <= '0';
		writeRB_inst				   <= '0'; -- the signal which has to be set to perform a write op over the RB
		checkMisalignedDAWU            <= '0';
		selCSRAddrFromInst             <= '0';
		forced_RB_read				   <= '0'; -- mux selection signal between the address to be read from write-back (as exception handling) and the one to be read in decode
		inst_type					   <= (OTHERS => '0');
		ecallFlag                      <= '0';
		illegalInstrFlag               <= '0';
		mipCCLdDisable                 <= '0';
		selCCMip_CSR                   <= '0';
		selCause_CSR                   <= '0';
		selPC_CSR                      <= '0';
		selTval_CSR                    <= '0';
		selMedeleg_CSR                 <= '0';
		selMideleg_CSR                 <= '0';
		ldValueCSR                     <= (OTHERS => '0');
		ldCntCSR                       <= '0';
		dnCntCSR                       <= '0';
		upCntCSR                       <= '0';
		ldDelegation                   <= '0';
		ldMachine                      <= '0';
		ldUser                         <= '0';
		loadMieReg                     <= '0';
		loadMieUieField                <= '0';
		mirrorUserCU                   <= '0';
		writeRegBank                   <= '0';
		selRomAddress                  <= '0';
		selMepc_CSR                    <= '0';
		selInterruptAddressDirect      <= '0';
		selInterruptAddressVectored    <= '0';
		machineStatusAlterationPreCSR  <= '0';
		userStatusAlterationPreCSR     <= '0';
		machineStatusAlterationPostCSR <= '0';
		userStatusAlterationPostCSR    <= '0';
		zeroCntCSR                     <= '0';

		-- produce all the signals that are part of the control word
		-- TODO: CHECK IF ALL THE NEWLY ADDED CONTROL SIGNALS ARE BEING DRIVEN CORRECTLY 
		-- FIXME: organize code in a different way to avoid repetitions of statements that set control signals
		IF (opcode = Loads) THEN
			MuxCode      <= iTypeImm;
			ldADR        <= '1';
			selJL        <= '1';
			selP1        <= '1';
			ldPC         <= '1';
			selI4        <= '1';
			dataInstrBar <= '1';
			--checkMisalignedDARU <= '1';
			selADR          <= '1';
			startDARU       <= '1'; -- no check is performed over read data alignment
			dataInstrBar    <= '1';
			nBytes          <= func3(1) & (func3(1) OR func3(0)); --nByte = 00 => Load Byte , nByte = 01 => Load Half, nByte = 11 => Load Word
		ELSIF (opcode = Stores) THEN
			muxCode <= sTypeImm;
			ldADR   <= '1';
			selJL   <= '1';
			selP1   <= '1'; -- added Gianluca
			ldDR    <= '1';
			ldPC    <= '1';
			selI4   <= '1';
			checkMisalignedDAWU <= '1';
			selADR              <= '1';
			startDAWU           <= '1'; -- no check is performed over the alignment of data to be written in memory
			nBytes              <= func3(1) & (func3(1) OR func3(0));
		ELSIF ((opcode = Arithmetic)) THEN
			IF (func7(0) = '1') THEN -- mul/div
				ldFlags          <= '1';
				-- illegalInstrFlag <= dividedByZeroOut; FIXME: WHY? THERE ARE TWO DIFFERENT EXCEPTIONS FOR DIVISION BY ZERO AND ILLEGAL INSTRUCTIONS...
				selP1            <= '1';
				selP2            <= '1';
				startDivideAAU   <= func3(2);
				startMultiplyAAU <= NOT(func3(2));
				IF (func3(2) = '0') THEN -- it is a multiplication
					IF (func3(1 DOWNTO 0) = "00") THEN
						signedSigned <= '1';
					ELSIF (func3(1 DOWNTO 0) = "01") THEN
						signedSigned <= '1';
					ELSIF (func3(1 DOWNTO 0) = "10") THEN
						signedUnsigned <= '1';
					ELSIF (func3(1 DOWNTO 0) = "11") THEN
						unsignedUnsigned <= '1';
					END IF;
				ELSIF (func3(2) = '1') THEN -- it is a division
					IF (func3(1 DOWNTO 0) = "00" OR func3(1 DOWNTO 0) = "10") THEN
						signedSigned <= '1';
					ELSIF (func3(1 DOWNTO 0) = "01" OR func3(1 DOWNTO 0) = "11") THEN
						unsignedUnsigned <= '1';
					END IF;
				END IF;
				IF (func3(2) = '0') THEN
					selAAL <= (NOT (func3(1) OR func3(0)));
					selAAH <= ((func3(1) OR func3(0)));
				ELSIF (func3(2) = '1') THEN
					selAAL <= (func3(1));     --Remainder
					selAAH <= NOT (func3(1)); --Quotient
				END IF;
			ELSIF (func3 = "000") THEN -- add/sub
				muxCode      <= iTypeImm;
				selImm       <= NOT(opcode(5));
				selP1        <= '1';
				selP2        <= opcode(5);
				addSubBar    <= func7(5) AND opcode(5);
				selASU       <= '1';
				writeRegFile <= '1';
				ldPC         <= '1';
				selI4        <= '1';
			ELSIF (func3 = "010" OR func3 = "011") THEN -- comparisons
				muxCode                   <= iTypeImm;
				selImm                    <= NOT(opcode(5));
				selP1                     <= '1';
				selP2                     <= opcode(5);
				comparedSignedUnsignedBar <= NOT(func3(0));
				ldPC  <= '1';
				selI4 <= '1';
				-- FIXME: setOne and setZero are set directly in the pipeline depending on the result of the comparison
				-- IF (lt = '1') THEN
				-- 	setOne <= '1';
				-- ELSE
				-- 	setZero <= '1';
				-- END IF;
			ELSIF (func3 = "100" OR func3 = "110" OR func3 = "111") THEN -- logical
				muxCode      <= iTypeImm;
				selImm       <= NOT(opcode(5));
				selP1        <= '1';
				selP2        <= opcode(5);
				selLogic     <= func3(1 DOWNTO 0);
				selLLu       <= '1';
				writeRegFile <= '1';
				ldPC         <= '1';
				selI4        <= '1';
			ELSIF (func3 = "001" OR func3 = "101") THEN -- shift
				muxCode      <= iTypeImm;
				selImm       <= NOT(opcode(5));
				selP1        <= '1';
				selP2        <= opcode(5);
				selShift     <= func3(2) & func7(5);
				selBsu       <= '1';
				writeRegFile <= '1';
				ldPC         <= '1';
				selI4        <= '1';
			END IF;
		ELSIF ((opcode = ImmediateArithmetic)) THEN --Immediate
			IF (func3 = "000") THEN -- add/sub
				muxCode      <= iTypeImm;
				selImm       <= NOT(opcode(5));
				selP1        <= '1';
				selP2        <= opcode(5);
				addSubBar    <= func7(5) AND opcode(5);
				selASU       <= '1';
				writeRegFile <= '1';
				ldPC         <= '1';
				selI4        <= '1';
			ELSIF (func3 = "010" OR func3 = "011") THEN -- comparisons
				muxCode                   <= iTypeImm;
				selImm                    <= NOT(opcode(5));
				selP1                     <= '1';
				selP2                     <= opcode(5);
				comparedSignedUnsignedBar <= NOT(func3(0));
				ldPC  <= '1';
				selI4 <= '1';
				-- FIXME: setOne and setZero are set directly in the pipeline depending on the result of the comparison
				-- IF (lt = '1') THEN
				-- 	setOne <= '1';
				-- ELSE
				-- 	setZero <= '1';
				-- END IF;
			ELSIF (func3 = "100" OR func3 = "110" OR func3 = "111") THEN -- logicals
				muxCode      <= iTypeImm;
				selImm       <= NOT(opcode(5));
				selP1        <= '1';
				selP2        <= opcode(5);
				selLogic     <= func3(1 DOWNTO 0);
				selLLu       <= '1';
				writeRegFile <= '1';
				ldPC         <= '1';
				selI4        <= '1';
			ELSIF (func3 = "001" OR func3 = "101") THEN -- shift
				muxCode      <= iTypeImm;
				selImm       <= NOT(opcode(5));
				selP1        <= '1';
				selP2        <= opcode(5);
				selShift     <= func3(2) & func7(5);
				selBsu       <= '1';
				writeRegFile <= '1';
				ldPC         <= '1';
				selI4        <= '1';
			END IF;
		ELSIF (opcode = JumpAndLink) THEN -- JAL
			muxCode      <= jTypeImm;
			selInc4PC    <= '1';
			writeRegFile <= '1';
			selPC        <= '1';
			selAdd       <= '1';
			ldPC         <= '1';
		ELSIF (opcode = JumpAndLinkRegister) THEN -- JALR
			muxCode      <= iTypeImm;
			selInc4PC    <= '1';
			writeRegFile <= '1';
			selJL        <= '1';
			ldPC         <= '1';
			selP1        <= '1';
			selAdd       <= '1';
		ELSIF (opcode = Branch) THEN -- conditional branch
			muxCode                   <= bTypeImm;
			selP1                     <= '1';
			selP2                     <= '1';
			comparedSignedUnsignedBar <= NOT(func3(1));
			selPC                     <= '1';
			ldPC                      <= '1';
		ELSIF (opcode = LoadUpperImmediate OR opcode = AddUpperImmediatePC) THEN -- LUI
			muxCode      <= uTypeImm;
			selImm       <= '1';
			selASU       <= '1';
			writeRegFile <= '1';
			ldPC         <= '1';
			selI4        <= '1';
			pass         <= opcode(5);
			--addSubBar    <= NOT (opcode(5)); -- modified Gianluca
			addSubBar    <= '0';
			selAuipc     <= NOT (opcode(5));
		ELSIF (opcode = SystemAndCSR) THEN
			IF (func3 = "000" AND func12 = X"000") THEN
				ecallFlag <= '1';
			ELSIF (func3 = "000" AND(func12 = X"302" OR func12 = X"002")) THEN
				n_state <= retEpc; -- mret or uret Instructions
			ELSE -- CSR instructions
				selCSRAddrFromInst <= '1';
				selCSR             <= '1' AND validAccessCSR;
				writeRegFile       <= '1' AND validAccessCSR;
				writeRegBank       <= '1' AND (validAccessCSR AND NOT(readOnlyCSR));
				selP1CSR           <= NOT (func3(2)) AND (validAccessCSR AND NOT(readOnlyCSR));
				selImmCSR          <= func3(2) AND (validAccessCSR AND NOT(readOnlyCSR));
				selReadWriteCSR    <= NOT(func3(1)) AND (validAccessCSR AND NOT(readOnlyCSR));
				setCSR             <= NOT(func3(0)) AND (validAccessCSR AND NOT(readOnlyCSR));
				clrCSR             <= (func3(1) AND func3(0)) AND (validAccessCSR AND NOT(readOnlyCSR));
				loadMieReg         <= ldMieReg AND validAccessCSR; -- TODO: CHANGE IT TO SUPPORT MIRRORING
				loadMieUieField    <= ldMieUieField AND validAccessCSR; -- TODO: CHANGE IT TO SUPPORT MIRRORING
				illegalInstrFlag   <= NOT(validAccessCSR); -- TODO: VERIFY IF IT MAY WORK...
				--illegalInstrFlag   <= NOT(validAccessCSR) OR readOnlyCSR;
				-- FIXME: FIND A WAY TO CHANGE THE CONTROL SIGNALS DURING MIRRORING (a possibility could be the one to 
				-- set validAccessCSR and readOnlyCSR to transparent values)
				selCSRAddrFromInst <= '1';
				selCSR             <= '1';
				writeRegBank       <= '1';
				selP1CSR           <= NOT (func3(2));
				selImmCSR          <= func3(2);
				selReadWriteCSR    <= NOT(func3(1));
				setCSR             <= NOT(func3(0));
				clrCSR             <= func3(1) AND func3(0);
				mirrorUser         <= '1';
			END IF;
		ELSE
			n_state <= idle;
		END IF;

		-- default value for all enable signals and reset signals
		GI2D_en_temp <= '0';
		D2E_en_temp <= '0';
		E2M_en_temp <= '0';
		M2WB_en_temp <= '0';
		GI2D_rst <= '0';
		D2E_rst <= '0';
		E2M_rst <= '0';
		M2WB_rst <= '0';

		-- activation conditions for enable signals (TODO: PLACE THE ENABLES_temp/VALID BITS IN THE SENSITIVITY LIST SINCE THEY ARE ACTIVATED IN A CHAIN)
		
		-- as for D2E_en, with an additional check on completedDARU1 which is a set asynchronously set when memory read is complete
		IF ((D2E_en_temp = '1' OR D_valid = '1') AND completedDARU1 = '1') THEN
			GI2D_en_temp   <= '1';
		END IF;
		-- as for E2M_en, but there are no conditions which require multiple cycles to be solved
		IF ((E2M_en_temp = '1' OR E_valid = '0') AND D_valid = '1') THEN
			D2E_en_temp    <= '1';
		END IF;
		-- a valid instruction can be promoted in memory when either the instruction currently in memory is invalid or it
		-- is moving in write-back and it has completed execution in EX (wait for 1 cycle for non AAU, wait until completedAAU set otherwise)
		IF ((M2WB_en_temp = '1' OR M_valid = '0') AND E_valid = '1' AND (is_AAU_used = '0' OR completedAAU = '1')) THEN
			E2M_en_temp    <= '1';
		END IF;
		-- completedDAWU and completedDARU2 are raised also when the MEM stage is occupied by an
		-- instruction which doesn't require memory access
		IF (instructionDone = '1' AND (completedDAWU = '1' OR completedDARU2 = '1') AND M_valid = '1') THEN
			M2WB_en_temp   <= '1';
		END IF;

		CASE p_state IS
			WHEN idle =>
				IF (WB_ret_from_epc = '1' AND WB_valid = '1') THEN
					GI2D_en_temp   <= '0';
					D2E_en_temp    <= '0';
					E2M_en_temp    <= '0';
					M2WB_en_temp   <= '0';
				ELSIF (interruptRaise = '1' OR exceptionRaise = '1') THEN
					GI2D_en_temp   <= '0';
					D2E_en_temp    <= '0';
					E2M_en_temp    <= '0';
					M2WB_en_temp   <= '0';
				ELSIF (hazEX = '1') THEN
					GI2D_en_temp   <= '0';
					D2E_en_temp    <= '0';
				ELSIF (hazM = '1') THEN
					GI2D_en_temp   <= '0';
					D2E_en_temp    <= '0';
				END IF;
			-- exception and interrupt handling states
			WHEN checkDelegation =>
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				selMideleg_CSR <= interruptRaise; -- ok, interruptRaise must remain high until the interrupt is served
				selMedeleg_CSR <= exceptionRaise; -- as before, for exceptionRaise
				ldDelegation   <= '1';
				mipCCLdDisable <= '1';
			WHEN updateTrapValue =>
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				int_ex 		   <= '1'; -- select mirrorUser from CU
				mirrorUserCU   <= NOT(delegationMode(0));
				selRomAddress  <= '1';
				ldValueCSR     <= "001";
				ldCntCSR       <= '1';
				selTval_CSR    <= '1';
				writeRegBank   <= '1';
				mipCCLdDisable <= '1';
			WHEN updateMip => -- don't count up while you are here, afterward you have to update the UIP
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				int_ex 		   <= '1';
				mirrorUser     <= '0';
				selRomAddress  <= '1';
				selCCMip_CSR   <= '1';
				writeRegBank   <= '1';
				mipCCLdDisable <= '1';
			WHEN updateUip => --new
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				int_ex 		   <= '1';
				selRomAddress  <= '1';
				mirrorUser     <= '1';
				upCntCSR       <= '1';
				selCCMip_CSR   <= '1';
				writeRegBank   <= '1';
				mipCCLdDisable <= '1';
			WHEN updateCause =>
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				int_ex		   <= '1';
				mirrorUser     <= NOT(delegationMode(0));
				selRomAddress  <= '1';
				upCntCSR       <= '1';
				selCause_CSR   <= '1';
				writeRegBank   <= '1';
				mipCCLdDisable <= '1';
			WHEN updateEpc =>
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				int_ex 		   <= '1';
				mirrorUser     <= NOT(delegationMode(0));
				selRomAddress  <= '1';
				upCntCSR       <= '1';
				selPC_CSR      <= '1';
				writeRegBank   <= '1';
				mipCCLdDisable <= '1';
			WHEN readTvec1 =>
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				mirrorUser 	   <= '1';
				mirrorUser     <= NOT(delegationMode(0));
				selRomAddress  <= '1';
				mipCCLdDisable <= '1';
			WHEN readTvec2 =>
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				int_ex		   <= '1';
				selRomAddress  <= '1';
				mirrorUser     <= NOT(delegationMode(0));
				ldMachine      <= delegationMode(0);
				ldUser         <= NOT(delegationMode(0));
				--selMepc_CSR    <= '1';
				upCntCSR       <= '1';
				ldPC           <= '1';
				mipCCLdDisable <= '1';
				zeroFlags      <= exceptionRaise;
				IF (modeTvec = "00") THEN
					selInterruptAddressDirect <= '1';
				ELSIF (modeTvec = "01") THEN
					selInterruptAddressVectored <= '1';
				END IF;
			WHEN readMstatus =>
				GI2D_en_temp   				  <= '0';
				D2E_en_temp    				  <= '0';
				E2M_en_temp    				  <= '0';
				M2WB_en_temp   				  <= '0';
				int_ex		   				  <= '1';
				mirrorUser     				  <= '0';
				selRomAddress  				  <= '1';
				mipCCLdDisable 				  <= '1';
			WHEN updateMstatus =>
				GI2D_en_temp   				  <= '0';
				D2E_en_temp    				  <= '0';
				E2M_en_temp    				  <= '0';
				M2WB_en_temp   				  <= '0';
				int_ex					      <= '1';
				mirrorUser                    <= '0';
				selRomAddress                 <= '1';
				loadMieUieField               <= ldMieUieField;
				machineStatusAlterationPreCSR <= delegationMode(0);
				userStatusAlterationPreCSR    <= NOT(delegationMode(0));
				writeRegBank                  <= '1';
				mipCCLdDisable                <= '1';
			WHEN updateUstatus =>
				GI2D_en_temp   				  <= '0';
				D2E_en_temp    				  <= '0';
				E2M_en_temp    				  <= '0';
				M2WB_en_temp   				  <= '0';
				int_ex						  <= '1';
				mirrorUser                    <= '1';
				selRomAddress                 <= '1';
				mirrorUser                    <= '1';
				machineStatusAlterationPreCSR <= delegationMode(0);
				userStatusAlterationPreCSR    <= NOT(delegationMode(0));
				writeRegBank                  <= '1';
				mipCCLdDisable                <= '1';
			    ldMachine      				  <= delegationMode(0);
				ldUser                        <= NOT(delegationMode(0));
			WHEN retEpc =>
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				mirrorUser    <= NOT(mretOrUretBar);
				selRomAddress <= '1';
			    ldValueCSR <= "100";
			    ldCntCSR   <= '1';
			WHEN retReadMstatus =>
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				selMepc_CSR   <= '1';
				ldPC          <= '1';
				mirrorUser    <= '0';
				selRomAddress <= '1';
			WHEN retUpdateMstatus =>
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				mirrorUser    <= '0';
				selRomAddress <= '1';
				IF (mretOrUretBar = '1') THEN
					ldMachine <= previousPRV(0);
					ldUser    <= NOT(previousPRV(0));
				ELSE
					ldMachine <= '0';
					ldUser    <= '1';
				END IF;
				loadMieUieField                <= ldMieUieField;
				machineStatusAlterationPostCSR <= mretOrUretBar;
				userStatusAlterationPostCSR    <= NOT(mretOrUretBar);
				writeRegBank                   <= '1';
			WHEN retUpdateUstatus =>
				GI2D_en_temp   				   <= '0';
				D2E_en_temp    				   <= '0';
				E2M_en_temp    				   <= '0';
				M2WB_en_temp   				   <= '0';
				selRomAddress                  <= '1';
				mirrorUser                     <= '1';
				machineStatusAlterationPostCSR <= mretOrUretBar;
				userStatusAlterationPostCSR    <= NOT(mretOrUretBar);
				writeRegBank                   <= '1';
				zeroCntCSR                     <= '1';
			-- WHEN fetch =>
			-- 	IF (exceptionRaise = '1') THEN
			-- 		ldValueCSR     <= "111";
			-- 		ldCntCSR       <= '1';
			-- 		selMedeleg_CSR <= '1';
			-- 	ELSIF (interruptRaise = '1') THEN
			-- 		zeroCntCSR     <= '1';
			-- 		mipCCLdDisable <= '1';
			-- 		selMideleg_CSR <= '1';
			-- 	ELSE
			-- 		nBytes              <= "11";
			-- 		selPCJ              <= '1';
			-- 		dataInstrBar        <= '0';
			-- 		selPC               <= '1';
			-- 		startDARU           <= '1';
			-- 		ldFlags             <= '1';
			-- 		checkMisalignedDARU <= '1';
			-- 	END IF;
			-- WHEN getInstr =>
			-- 	dataInstrBar <= '0';
			-- 	IF (completeDARU = '1') THEN
			-- 		ldIR <= '1';
			-- 	ELSE
			-- 		ldIR <= '0';
			-- 	END IF;
			-- 	--decode
			-- WHEN decode =>
			-- 	ldFlags <= '1';
			-- 	IF (opcode = Loads OR opcode = Stores OR opcode = Arithmetic OR
			-- 		opcode = ImmediateArithmetic OR opcode = JumpAndLink OR
			-- 		opcode = JumpAndLinkRegister OR opcode = Branch OR
			-- 		opcode = LoadUpperImmediate OR opcode = AddUpperImmediatePC OR
			-- 		opcode = SystemAndCSR) THEN
			-- 		illegalInstrFlag <= '0';
			-- 	ELSE
			-- 		illegalInstrFlag <= '1';
			-- 	END IF;
			-- 	IF (opcode = SystemAndCSR) THEN --mret or uret
			-- 		IF (func3 = "000" AND (func12 = X"302" OR func12 = X"002")) THEN
			-- 			ldValueCSR <= "010";
			-- 			ldCntCSR   <= '1';
			-- 		END IF;
			-- 		IF (func3 /= "000") THEN
			-- 			selCSRAddrFromInst <= '1';
			-- 		END IF;
			-- 	END IF;
			-- WHEN getData =>
			-- 	ldByteSigned <= NOT(func3(2)) AND NOT(func3(1)) AND NOT(func3(0));
			-- 	--ldHalfSigned <= NOT(func3(2)) AND func3(0); -- modified Gianluca
			-- 	ldHalfSigned <= NOT(func3(2)) and  NOT(func3(1)) and func3(0);
			-- 	load         <= func3(2) OR func3(1);
			-- 	dataInstrBar <= '1';
			-- 	IF (completeDARU = '1') THEN
			-- 		writeRegFile <= '1';
			-- 		selDARU      <= '1';
			-- 	ELSE
			-- 		writeRegFile <= '0';
			-- 		selDARU      <= '0';
			-- 	END IF;
			-- 	--store
			-- WHEN storeInstr1 =>
			-- 	muxCode <= sTypeImm;
			-- 	ldADR   <= '1';
			-- 	selJL   <= '1';
			-- 	selP1   <= '1'; -- added Gianluca
			-- 	ldDR    <= '1';
			-- 	ldPC    <= '1';
			-- 	selI4   <= '1';
			-- WHEN storeInstr2 =>
			-- 	checkMisalignedDAWU <= '1';
			-- 	ldFlags             <= '1';
			-- 	selADR              <= '1';
			-- 	startDAWU           <= NOT(storeMisalignedOut);
			-- 	nBytes              <= func3(1) & (func3(1) OR func3(0));
			-- WHEN putData =>
			-- 	--
			-- 	--addSub
			-- WHEN addSub  =>
			-- 	muxCode      <= iTypeImm;
			-- 	selImm       <= NOT(opcode(5));
			-- 	selP1        <= '1';
			-- 	selP2        <= opcode(5);
			-- 	addSubBar    <= func7(5) AND opcode(5);
			-- 	selASU       <= '1';
			-- 	writeRegFile <= '1';
			-- 	ldPC         <= '1';
			-- 	selI4        <= '1';
			-- 	--setCompare
			-- WHEN compare =>
			-- 	--logical
			-- WHEN logical =>
			-- 	muxCode      <= iTypeImm;
			-- 	selImm       <= NOT(opcode(5));
			-- 	selP1        <= '1';
			-- 	selP2        <= opcode(5);
			-- 	selLogic     <= func3(1 DOWNTO 0);
			-- 	selLLu       <= '1';
			-- 	writeRegFile <= '1';
			-- 	ldPC         <= '1';
			-- 	selI4        <= '1';
			-- 	--shift
			-- WHEN shift =>
			-- 	muxCode      <= iTypeImm;
			-- 	selImm       <= NOT(opcode(5));
			-- 	selP1        <= '1';
			-- 	selP2        <= opcode(5);
			-- 	selShift     <= func3(2) & func7(5);
			-- 	selBsu       <= '1';
			-- 	writeRegFile <= '1';
			-- 	ldPC         <= '1';
			-- 	selI4        <= '1';
			-- 	--multiplyDivide
			-- WHEN multiplyDivide1 =>
			-- 	ldFlags          <= '1';
			-- 	illegalInstrFlag <= dividedByZeroOut;
			-- 	selP1            <= '1';
			-- 	selP2            <= '1';
			-- 	startDivideAAU   <= func3(2);
			-- 	startMultiplyAAU <= NOT(func3(2));
			-- 	IF (func3(2) = '0') THEN -- it is a multiplication
			-- 		IF (func3(1 DOWNTO 0) = "00") THEN
			-- 			signedSigned <= '1';
			-- 		ELSIF (func3(1 DOWNTO 0) = "01") THEN
			-- 			signedSigned <= '1';
			-- 		ELSIF (func3(1 DOWNTO 0) = "10") THEN
			-- 			signedUnsigned <= '1';
			-- 		ELSIF (func3(1 DOWNTO 0) = "11") THEN
			-- 			unsignedUnsigned <= '1';
			-- 		END IF;
			-- 	ELSIF (func3(2) = '1') THEN -- it is a division
			-- 		IF (func3(1 DOWNTO 0) = "00" OR func3(1 DOWNTO 0) = "10") THEN
			-- 			signedSigned <= '1';
			-- 		ELSIF (func3(1 DOWNTO 0) = "01" OR func3(1 DOWNTO 0) = "11") THEN
			-- 			unsignedUnsigned <= '1';
			-- 		END IF;
			-- 	END IF;
			-- WHEN multiplyDivide2 =>
			-- 	selP1 <= '1'; -- added Gianluca start
			-- 	selP2 <= '1';
			-- 	IF (func3(2) = '0') THEN
			-- 		IF (func3(1 DOWNTO 0) = "00") THEN
			-- 			signedSigned <= '1';
			-- 		ELSIF (func3(1 DOWNTO 0) = "01") THEN
			-- 			signedSigned <= '1';
			-- 		ELSIF (func3(1 DOWNTO 0) = "10") THEN
			-- 			signedUnsigned <= '1';
			-- 		ELSIF (func3(1 DOWNTO 0) = "11") THEN
			-- 			unsignedUnsigned <= '1';
			-- 		END IF;
			-- 	ELSIF (func3(2) = '1') THEN
			-- 		IF (func3(1 DOWNTO 0) = "00" OR func3(1 DOWNTO 0) = "10") THEN
			-- 			signedSigned <= '1';
			-- 		ELSIF (func3(1 DOWNTO 0) = "01" OR func3(1 DOWNTO 0) = "11") THEN
			-- 			unsignedUnsigned <= '1';
			-- 		END IF;
			-- 	END IF; -- added Gianluca end
			-- 	IF (completeAAU = '1') THEN
			-- 		IF (func3(2) = '0') THEN
			-- 			selAAL <= (NOT (func3(1) OR func3(0)));
			-- 			selAAH <= ((func3(1) OR func3(0)));
			-- 		ELSIF (func3(2) = '1') THEN
			-- 			selAAL <= (func3(1));     --Remainder
			-- 			selAAH <= NOT (func3(1)); --Quotient
			-- 		END IF;
			-- 	ELSE
			-- 		selAAL <= '0';
			-- 		selAAH <= '0';
			-- 	END IF;
			-- 	selAAU       <= completeAAU;
			-- 	writeRegFile <= completeAAU;
			-- 	ldPC         <= completeAAU;
			-- 	selI4        <= completeAAU;
			-- WHEN JAL =>
			-- 	muxCode      <= jTypeImm;
			-- 	selInc4PC    <= '1';
			-- 	writeRegFile <= '1';
			-- 	selPC        <= '1';
			-- 	selAdd       <= '1';
			-- 	ldPC         <= '1';
			-- WHEN JALR =>
			-- 	muxCode      <= iTypeImm;
			-- 	selInc4PC    <= '1';
			-- 	writeRegFile <= '1';
			-- 	selJL        <= '1';
			-- 	ldPC         <= '1';
			-- 	selP1        <= '1';
			-- 	selAdd       <= '1';
			-- 	--conditionalBranch
			-- WHEN conditionalBranch =>
			-- 	muxCode                   <= bTypeImm;
			-- 	selP1                     <= '1';
			-- 	selP2                     <= '1';
			-- 	comparedSignedUnsignedBar <= NOT(func3(1));
			-- 	selPC                     <= '1';
			-- 	ldPC                      <= '1';
			-- 	IF (func3(2) = '1' AND func3(0) = '0') THEN --BLT, BLTU
			-- 		IF (lt = '1') THEN
			-- 			selADD <= '1';
			-- 		ELSE
			-- 			selI4 <= '1';
			-- 		END IF;
			-- 	ELSIF (func3(2) = '1' AND func3(0) = '1') THEN --BGE, BGEU
			-- 		IF (gt = '1' OR eq = '1') THEN
			-- 			selADD <= '1';
			-- 		ELSE
			-- 			selI4 <= '1';
			-- 		END IF;
			-- 	ELSIF (func3(2) = '0' AND func3(0) = '0') THEN --BEQ
			-- 		IF (eq = '1') THEN
			-- 			selADD <= '1';
			-- 		ELSE
			-- 			selI4 <= '1';
			-- 		END IF;
			-- 	ELSIF (func3(2) = '0' AND func3(0) = '1') THEN --BNE
			-- 		IF (eq = '0') THEN
			-- 			selADD <= '1';
			-- 		ELSE
			-- 			selI4 <= '1';
			-- 		END IF;
			-- 	END IF;
			-- 	--LUI and AUIPC
			-- WHEN LUI =>
			-- 	muxCode      <= uTypeImm;
			-- 	selImm       <= '1';
			-- 	selASU       <= '1';
			-- 	writeRegFile <= '1';
			-- 	ldPC         <= '1';
			-- 	selI4        <= '1';
			-- 	pass         <= opcode(5);
			-- 	--addSubBar    <= NOT (opcode(5)); -- modified Gianluca
			-- 	addSubBar    <= '0';
			-- 	selAuipc     <= NOT (opcode(5));
			-- WHEN CSR =>
			-- 	selCSRAddrFromInst <= '1';
			-- 	selCSR             <= '1' AND validAccessCSR;
			-- 	writeRegFile       <= '1' AND validAccessCSR;
			-- 	writeRegBank       <= '1' AND (validAccessCSR AND NOT(readOnlyCSR));
			-- 	selP1CSR           <= NOT (func3(2)) AND (validAccessCSR AND NOT(readOnlyCSR));
			-- 	selImmCSR          <= func3(2) AND (validAccessCSR AND NOT(readOnlyCSR));
			-- 	selReadWriteCSR    <= NOT(func3(1)) AND (validAccessCSR AND NOT(readOnlyCSR));
			-- 	setCSR             <= NOT(func3(0)) AND (validAccessCSR AND NOT(readOnlyCSR));
			-- 	clrCSR             <= (func3(1) AND func3(0)) AND (validAccessCSR AND NOT(readOnlyCSR));
			-- 	ldPC               <= '1' AND (validAccessCSR AND NOT(readOnlyCSR));
			-- 	selI4              <= '1' AND (validAccessCSR AND NOT(readOnlyCSR));
			-- 	loadMieReg         <= ldMieReg AND validAccessCSR;
			-- 	loadMieUieField    <= ldMieUieField AND validAccessCSR;
			-- 	illegalInstrFlag   <= NOT(validAccessCSR);
			-- 	--illegalInstrFlag   <= NOT(validAccessCSR) OR readOnlyCSR;
			-- 	ldFlags            <= '1';
			-- WHEN mirrorCSR =>
			-- 	selCSRAddrFromInst <= '1';
			-- 	selCSR             <= '1';
			-- 	writeRegBank       <= '1';
			-- 	selP1CSR           <= NOT (func3(2));
			-- 	selImmCSR          <= func3(2);
			-- 	selReadWriteCSR    <= NOT(func3(1));
			-- 	setCSR             <= NOT(func3(0));
			-- 	clrCSR             <= func3(1) AND func3(0);
			-- 	mirrorUser         <= '1';
			-- WHEN retEpc =>
			-- 	mirrorUser    <= NOT(mretOrUretBar);
			-- 	selRomAddress <= '1';
			--     ldValueCSR <= "100";
			--     ldCntCSR   <= '1';
			-- WHEN retReadMstatus =>
			-- 	selMepc_CSR   <= '1';
			-- 	ldPC          <= '1';
			-- 	mirrorUser    <= '0';
			-- 	selRomAddress <= '1';
			-- WHEN retUpdateMstatus =>
			-- 	mirrorUser    <= '0';
			-- 	selRomAddress <= '1';
			-- 	IF (mretOrUretBar = '1') THEN
			-- 		ldMachine <= previousPRV(0);
			-- 		ldUser    <= NOT(previousPRV(0));
			-- 	ELSE
			-- 		ldMachine <= '0';
			-- 		ldUser    <= '1';
			-- 	END IF;
			-- 	loadMieUieField                <= ldMieUieField;
			-- 	machineStatusAlterationPostCSR <= mretOrUretBar;
			-- 	userStatusAlterationPostCSR    <= NOT(mretOrUretBar);
			-- 	writeRegBank                   <= '1';
			-- WHEN retUpdateUstatus =>
			-- 	selRomAddress                  <= '1';
			-- 	mirrorUser                     <= '1';
			-- 	machineStatusAlterationPostCSR <= mretOrUretBar;
			-- 	userStatusAlterationPostCSR    <= NOT(mretOrUretBar);
			-- 	writeRegBank                   <= '1';
			-- 	zeroCntCSR                     <= '1';
			-- WHEN ecall =>
			-- 	ecallFlag <= '1';
			-- 	ldFlags   <= '1';
			--WHEN checkDelegation =>
			--	selMideleg_CSR <= interruptRaise;
			--	selMedeleg_CSR <= exceptionRaise;
			--	ldDelegation   <= '1';
			--	mipCCLdDisable <= '1';
			--WHEN updateTrapValue =>
			--	mirrorUser     <= NOT(delegationMode(0)); --
			--	selRomAddress  <= '1';
			--	ldValueCSR     <= "001";
			--	ldCntCSR       <= '1';
			--	selTval_CSR    <= '1';
			--	writeRegBank   <= '1';
			--	mipCCLdDisable <= '1';
			--WHEN updateMip => -- don't count up while you are here, afterward you have to update the UIP
			--	mirrorUser     <= '0';--
			--	selRomAddress  <= '1';
			--	selCCMip_CSR   <= '1';
			--	writeRegBank   <= '1';
			--	mipCCLdDisable <= '1';
			--WHEN updateUip => --new
			--	selRomAddress  <= '1';
			--	mirrorUser     <= '1';
			--	upCntCSR       <= '1';
			--	selCCMip_CSR   <= '1';
			--	writeRegBank   <= '1';
			--	mipCCLdDisable <= '1';
			--WHEN updateCause =>
			--	mirrorUser     <= NOT(delegationMode(0));
			--	selRomAddress  <= '1';
			--	upCntCSR       <= '1';
			--	selCause_CSR   <= '1';
			--	writeRegBank   <= '1';
			--	mipCCLdDisable <= '1';
			--WHEN updateEpc =>
			--	mirrorUser     <= NOT(delegationMode(0));
			--	selRomAddress  <= '1';
			--	upCntCSR       <= '1';
			--	selPC_CSR      <= '1';
			--	writeRegBank   <= '1';
			--	mipCCLdDisable <= '1';
			--WHEN readTvec1 =>
			--	mirrorUser     <= NOT(delegationMode(0));
			--	selRomAddress  <= '1';
			--	mipCCLdDisable <= '1';
			--WHEN readTvec2 =>
			--	selRomAddress  <= '1';
			--	mirrorUser     <= NOT(delegationMode(0));
			--	ldMachine      <= delegationMode(0);
			--	ldUser         <= NOT(delegationMode(0));
			--	--selMepc_CSR    <= '1';
			--	upCntCSR       <= '1';
			--	ldPC           <= '1';
			--	mipCCLdDisable <= '1';
			--	zeroFlags      <= exceptionRaise;
			--	IF (modeTvec = "00") THEN
			--		selInterruptAddressDirect <= '1';
			--	ELSIF (modeTvec = "01") THEN
			--		selInterruptAddressVectored <= '1';
			--	END IF;
			--WHEN readMstatus =>
			--	mirrorUser     <= '0';
			--	selRomAddress  <= '1';
			--	mipCCLdDisable <= '1';
			--WHEN updateMstatus =>
			--	mirrorUser                    <= '0';
			--	selRomAddress                 <= '1';
			--	loadMieUieField               <= ldMieUieField;
			--	machineStatusAlterationPreCSR <= delegationMode(0);
			--	userStatusAlterationPreCSR    <= NOT(delegationMode(0));
			--	writeRegBank                  <= '1';
			--	mipCCLdDisable                <= '1';
			--WHEN updateUstatus => --new
			--	mirrorUser                    <= '1';
			--	selRomAddress                 <= '1';
			--	mirrorUser                    <= '1';
			--	machineStatusAlterationPreCSR <= delegationMode(0);
			--	userStatusAlterationPreCSR    <= NOT(delegationMode(0));
			--	writeRegBank                  <= '1';
			--	mipCCLdDisable                <= '1';
			--    ldMachine      				  <= delegationMode(0);
			--	ldUser                        <= NOT(delegationMode(0));
			--WHEN OTHERS =>
			--	selPC                          <= '0';
			--	selI4                          <= '0';
			--	selP2                          <= '0';
			--	selJL                          <= '0';
			--	selPCJ                         <= '0';
			--	selADR                         <= '0';
			--	selImm                         <= '0';
			--	selAdd                         <= '0';
			--	selInc4PC                      <= '0';
			--	selBSU                         <= '0';
			--	selLLU                         <= '0';
			--	selASU                         <= '0';
			--	selAAU                         <= '0';
			--	selDARU                        <= '0';
			--	dataInstrBar                   <= '0';
			--	writeRegFile                   <= '0';
			--	addSubBar                      <= '0';
			--	comparedsignedunsignedbar      <= '0';
			--	ldIR                           <= '0';
			--	ldADR                          <= '0';
			--	ldPC                           <= '0';
			--	ldDr                           <= '0';
			--	ldByteSigned                   <= '0';
			--	ldHalfSigned                   <= '0';
			--	load                           <= '0';
			--	setOne                         <= '0';
			--	setZero                        <= '0';
			--	startDARU                      <= '0';
			--	startDAWU                      <= '0';
			--	startMultiplyAAU               <= '0';
			--	startDivideAAU                 <= '0';
			--	signedSigned                   <= '0';
			--	signedUnsigned                 <= '0';
			--	unsignedUnsigned               <= '0';
			--	selAAL                         <= '0';
			--	selAAH                         <= '0';
			--	muxCode                        <= (OTHERS => '0');
			--	nBytes                         <= "00";
			--	selLogic                       <= "00";
			--	selShift                       <= "00";
			--	pass                           <= '0';
			--	selAuipc                       <= '0';
			--	selP1                          <= '0';
			--	selTval_CSR                    <= '0';
			--	zeroCntCSR                     <= '0';
			--	mipCCLdDisable                 <= '0';
			--	upCntCSR                       <= '0';
			--	dnCntCSR                       <= '0';
			--	selRomAddress                  <= '0';
			--	selCCMip_CSR                   <= '0';
			--	writeRegBank                   <= '0';
			--	selCause_CSR                   <= '0';
			--	machineStatusAlterationPostCSR <= '0';
			--	userStatusAlterationPostCSR    <= '0';
			--	selPC_CSR                      <= '0';
			--	ldValueCSR                     <= "000";
			--	selCSRAddrFromInst             <= '0';
			--	selP1CSR                       <= '0';
			--	selImmCSR                      <= '0';
			--	selReadWriteCSR                <= '0';
			--	setCSR                         <= '0';
			--	clrCSR                         <= '0';
			--	selCSR                         <= '0';
			--	selMepc_CSR                    <= '0';
			--	ldCntCSR                       <= '0';
			--	selInterruptAddressDirect      <= '0';
			--	selInterruptAddressVectored    <= '0';
			--	ldFlags                        <= '0';
			--	zeroFlags                      <= '0';
			--	illegalInstrFlag               <= '0';
			--	checkMisalignedDARU            <= '0';
			--	checkMisalignedDAWU            <= '0';
			--	ecallFlag                      <= '0';
			--	machineStatusAlterationPreCSR  <= '0';
			--	userStatusAlterationPreCSR     <= '0';
			--	selMedeleg_CSR                 <= '0';
			--	selMideleg_CSR                 <= '0';
			--	ldUser                         <= '0';
			--	ldMachine                      <= '0';
			--	mirrorUser                     <= '0';
			--	ldDelegation                   <= '0';
			--	loadMieUieField                <= '0';
			--	loadMieReg                     <= '0';
		END CASE;
	END PROCESS;
	sequential : PROCESS (clk, rst) BEGIN
		IF rst = '1' THEN
			p_state <= fetch;
		ELSIF (clk = '1' AND clk'EVENT) THEN
			p_state <= n_state;
		END IF;
	END PROCESS sequential;
END behavioral;
