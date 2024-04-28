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

-- TODO: MAKE SURE THAT validAccessCSR AND readOnlyCSR ARE RELATED TO THE INSTRUCTION IN
-- WRITE-BACK, BUT THE EXCEPTION HAS TO BE THROWN WHEN THE INSTRUCTION IS IN DECODE. THE
-- WRITING OPERATION ON THE MIRROR REGISTER SHOULD BE PERFORMED ONLY IF THE VALID FLAGS
-- HAVE BEEN SET BEFORE.
-- DONE: ADD MULTIPLEXERS BETWEEN THE SIGNAL PRODUCED BY THE MIRRORING CIRCUITRY AND THE 
-- COMBINATIONAL PROCESS WHICH HANDLES THE REMAINING FUNCTIONALITIES.
-- TODO: THERE IS NO NEED TO PERFORM THE CHECK OVER opcode(5) IN ARITHMETIC INSTRUCTIONS
-- WHEN CHOOSING IF WE HAVE TO SELECT THE IMMEDIATE OR THE SECOND REGISTER, THIS CHECK WAS
-- ACTUALLY NEEDED ONLY WHEN THE FLOW WAS REDIRECTED TO THE SAME PIECE OF CODE BOTH FOR 
-- R TYPE AND I TYPE INSTRUCTIONS.
-- TODO: REVIEW ALL CONTROL SIGNALS TO ELIMINATE THE UNNEEDED ONES AND ADD THE NEW ONES.
-- DONE: ADD LOGIC TO HANDLE MISPREDICTIONS
-- DONE: SET selJL EVERY TIME YOU NEED AS A FIRST OPERAND A REGISTER THAT IS NOT THE PC

LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.std_logic_unsigned.ALL;
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
		hazard_solved 				   : IN STD_LOGIC;
		is_store_in_mem				   : IN STD_LOGIC;
		is_load_in_mem				   : IN STD_LOGIC;
		branch_taken 				   : IN STD_LOGIC;
		DEC_valid					   : IN STD_LOGIC;
		EX_valid					   : IN STD_LOGIC;
		M_valid						   : IN STD_LOGIC;
		WB_valid					   : IN STD_LOGIC;
		WB_ret_from_epc				   : IN STD_LOGIC;
		WB_isCSRInstruction			   : IN STD_LOGIC;
		WB_validAccessCSR			   : IN STD_LOGIC;

		-- instruction to be decoded
		IR                             : IN STD_LOGIC_VECTOR (len - 1 DOWNTO 0);

		-- func3 field of the instruction currently in write-back (needed for CSR instructions)
		WB_func3					   : IN STD_LOGIC_VECTOR (2 DOWNTO 0);
		
		-- control word
		writeRegFile                   : OUT STD_LOGIC;
		setZeroOrOne                   : OUT STD_LOGIC;
		ComparedSignedUnsignedBar      : OUT STD_LOGIC;
		selPC                          : OUT STD_LOGIC;
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
		selALU						   : OUT STD_LOGIC;
		selPC4						   : OUT STD_LOGIC;
		selMem						   : OUT STD_LOGIC;
		cmp_selALUop2				   : OUT STD_LOGIC;
		cmp_selop2				   	   : OUT STD_LOGIC;
		isCSRInstruction			   : OUT STD_LOGIC;

		-- Interrupts
		CSR_from_WB					   : OUT STD_LOGIC;
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
		instructionDoneCSR			   : OUT STD_LOGIC;

		-- pipeline registers
		GI2D_en						   : OUT STD_LOGIC;
		GI2D_rst					   : OUT STD_LOGIC;
		D2E_en						   : OUT STD_LOGIC;
		D2E_rst					       : OUT STD_LOGIC;
		E2M_en						   : OUT STD_LOGIC;
		E2M_rst						   : OUT STD_LOGIC;
		M2WB_en						   : OUT STD_LOGIC;
		M2WB_rst					   : OUT STD_LOGIC;

		-- hazards
		hazEX						   : IN STD_LOGIC;
		hazM						   : IN STD_LOGIC
	);
END aftab_controller;
--
ARCHITECTURE behavioral OF aftab_controller IS
	
	SIGNAL p_state, n_state      : state;
	SIGNAL CSR_p_state, CSR_n_state : CSR_state;
	SIGNAL func3                 : STD_LOGIC_VECTOR(2 DOWNTO 0);
	SIGNAL func7, opcode         : STD_LOGIC_VECTOR(6 DOWNTO 0);
	SIGNAL func12                : STD_LOGIC_VECTOR(11 DOWNTO 0);
	SIGNAL mretOrUretBar         : STD_LOGIC;
	SIGNAL GI2D_en_temp			 : STD_LOGIC;
	SIGNAL D2E_en_temp			 : STD_LOGIC;
	SIGNAL E2M_en_temp			 : STD_LOGIC;
	SIGNAL M2WB_en_temp			 : STD_LOGIC;
	SIGNAL writeRegFile_CSR		 : STD_LOGIC;
	SIGNAL writeRegFile_RF		 : STD_LOGIC;
	SIGNAL mirrorUserCU_exint	 : STD_LOGIC;
	SIGNAL mirrorUserCU_inst	 : STD_LOGIC;
	SIGNAL writeRegBank_exint	 : STD_LOGIC;
	SIGNAL writeRegBank_CSR		 : STD_LOGIC;
	SIGNAL loadMieUieField_exint : STD_LOGIC;
	SIGNAL loadMieUieField_CSR	 : STD_LOGIC;
BEGIN
	func3           <= IR(14 DOWNTO 12);
	func7           <= IR(31 DOWNTO 25);
	func12          <= IR(31 DOWNTO 20);
	opcode          <= IR(6 DOWNTO 0);
	mretOrUretBar   <= IR(29);
	StateTransition : PROCESS (p_state, completedDARU1, completedDARU2, is_AAU_used, instructionDone,
							   completedDAWU, completedAAU, opcode, func3, func7, 
							   func12, exceptionRaise, interruptRaise, 
							   modeTvec, mirror, mretOrUretBar, 
							   previousPRV, delegationMode, ldMieUieField, ldMieReg, 
							   validAccessCSR, readOnlyCSR, branch_taken, hazard_solved
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
					n_state <= checkDelegation;
				ELSIF (branch_taken = '1') THEN
					n_state <= idle;
				-- EXCEPTIONS IN EXECUTE HAVE AN HIGHER PRIO, BECAUSE WHEN THE INSTRUCTION WHICH PRODUCES THE RESULT IS
				-- PROMOTED IN MEMORY THEN IT RAISES A MEMORY HAZARD!
				ELSIF (hazEX = '1') THEN
					n_state <= hazEX_state;
				ELSIF (hazM = '1') THEN
					n_state <= hazM_state;
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
				-- you move to the waitForStore state only if you are currently handling an interrupt and there is
				-- a store instruction in MEM which hasn't completed execution yet.
				IF (is_store_in_mem = '1' AND completedDAWU = '0' AND exceptionRaise = '0') THEN
					n_state <= waitForStore;
				ELSE
					n_state <= idle;
				END IF;
			WHEN retEpc =>
				n_state <= retReadMstatus;
			WHEN retReadMstatus =>
				n_state <= retUpdateMstatus;
			WHEN retUpdateMstatus =>
				n_state <= retUpdateUstatus;
			WHEN retUpdateUstatus =>
				n_state <= idle;
			WHEN hazEX_state =>
				-- you can transition to hazEX, idle or checkDelegation
				IF (interruptRaise = '1' OR exceptionRaise = '1') THEN
					n_state <= checkDelegation;
				ELSIF (hazard_solved = '1' OR branch_taken = '1') THEN
					n_state <= idle;
				ELSE
					n_state <= hazEX_state;
				END IF;
			WHEN hazM_state =>
				-- you can transition to hazM, idle or checkDelegation
				IF (interruptRaise = '1' OR exceptionRaise = '1') THEN
					n_state <= checkDelegation;
				ELSIF (hazard_solved = '1' OR branch_taken = '1') THEN
					n_state <= idle;
				ELSE
					n_state <= hazM_state;
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
		END CASE;
	END PROCESS;

	-- FSM TO HANDLE CSR OPERATIONS
	CSR_FSM_combinational: PROCESS (CSR_p_state, ldMieReg, ldMieUieField, WB_isCSRInstruction, readOnlyCSR, WB_validAccessCSR, mirror, p_state)
	BEGIN
		mirrorUserCU_inst  <= '0';
		-- writeRegFile_CSR   <= '0';
		writeRegBank_CSR   <= '0';
		selP1CSR           <= '0';
		selImmCSR          <= '0';
		selReadWriteCSR    <= '0';
		setCSR             <= '0';
		clrCSR             <= '0';
		loadMieReg         <= '0';
		loadMieUieField_CSR<= '0';
		CSR_n_state 	   <= CSR_p_state;
		instructionDoneCSR <= '0';
		CASE CSR_p_state IS
			WHEN idle =>
				IF (WB_isCSRInstruction = '1') THEN
					-- writeRegFile_CSR   <= '1' AND WB_validAccessCSR;
					writeRegBank_CSR   <= '1' AND (WB_validAccessCSR AND NOT(readOnlyCSR));
					selP1CSR           <= NOT (WB_func3(2)) AND (WB_validAccessCSR AND NOT(readOnlyCSR));
					selImmCSR          <= WB_func3(2) AND (WB_validAccessCSR AND NOT(readOnlyCSR));
					selReadWriteCSR    <= NOT(WB_func3(1)) AND (WB_validAccessCSR AND NOT(readOnlyCSR));
					setCSR             <= NOT(WB_func3(0)) AND (WB_validAccessCSR AND NOT(readOnlyCSR));
					clrCSR             <= (WB_func3(1) AND WB_func3(0)) AND (WB_validAccessCSR AND NOT(readOnlyCSR));
					loadMieReg         <= ldMieReg AND WB_validAccessCSR;
					loadMieUieField_CSR<= ldMieUieField AND WB_validAccessCSR;
					instructionDoneCSR <= '1';
					IF (mirror = '1') THEN
						CSR_n_state    <= mirror_state;
						instructionDoneCSR <= '0';
					END IF; 
				END IF;
			WHEN mirror_state =>
				-- selCSRAddrFromInst <= '1';
				writeRegBank_CSR   <= '1';
				selP1CSR           <= NOT (WB_func3(2));
				selImmCSR          <= WB_func3(2);
				selReadWriteCSR    <= NOT(WB_func3(1));
				setCSR             <= NOT(WB_func3(0));
				clrCSR             <= WB_func3(1) AND WB_func3(0);
				mirrorUserCU_inst  <= '1';
				instructionDoneCSR <= '1';
				CSR_n_state 	   <= idle;
		END CASE;
	END PROCESS;

	CSR_FSM_sequential: PROCESS (clk, rst)
	BEGIN
		IF (rst = '1') THEN
			CSR_p_state <= idle;
		ELSIF (rising_edge (clk)) THEN
			CSR_p_state <= CSR_n_state;
		END IF;
	END PROCESS;

	-- MULTIPLEXERS BETWEEN THE SIGNALS DRIVEN BY THE MIRRORING CIRCUITRY AND THE MAIN COMBINATIONAL PROCESS
	mirrorUserCU <= mirrorUserCU_inst WHEN p_state = idle OR p_state = hazEX_state OR p_state = hazM_state ELSE
					mirrorUserCU_exint;

	writeRegBank <= writeRegBank_CSR WHEN p_state = idle OR p_state = hazEX_state OR p_state = hazM_state ELSE
					writeRegBank_exint;
	
	loadMieUieField <= loadMieUieField_CSR WHEN p_state = idle OR p_state = hazEX_state OR p_state = hazM_state ELSE
					loadMieUieField_exint;

	ControlSignalsDecoder : PROCESS (p_state, completedDARU1, completedDARU2, is_AAU_used, instructionDone, branch_taken,
							   completedDAWU, completedAAU, opcode, func3, func7, 
							   func12, exceptionRaise, interruptRaise, 
							   modeTvec, mirror, mretOrUretBar, 
							   previousPRV, delegationMode, ldMieUieField, ldMieReg, 
							   validAccessCSR, readOnlyCSR,
							   EX_valid, M_valid, DEC_valid, M2WB_en_temp, E2M_en_temp, D2E_en_temp
							   ) 
	BEGIN

		-- initialize all the control signals
		-- TODO: REMOVE THE ONES THAT ARE DRIVEN BY THE CSR FSM
		writeRegFile               	   <= '0';
		setZeroOrOne 			  	   <= '0';
		ComparedSignedUnsignedBar      <= '0';
		selPC                          <= '0';
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
		nBytes                         <= (OTHERS => '0');
		selCSR                         <= '0';
		writeRB_inst				   <= '0'; -- the signal which has to be set to perform a write op over the RB
		checkMisalignedDAWU            <= '0';
		selCSRAddrFromInst             <= '0';
		forced_RB_read				   <= '0'; -- mux selection signal between the address to be read from write-back (as exception handling) and the one to be read in decode
		inst_type					   <= (OTHERS => '0');
		ecallFlag                      <= '0';
		illegalInstrFlag               <= '0'; -- THE DEFAULT VALUE IS 'VALID', IT IS RESET IF THE INSTRUCTION DOESN'T HAVE A VALID OPCODE
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
		loadMieUieField_exint          <= '0';
		mirrorUserCU_exint             <= '0';
		writeRegBank_exint             <= '0';
		selRomAddress                  <= '0';
		selMepc_CSR                    <= '0';
		selInterruptAddressDirect      <= '0';
		selInterruptAddressVectored    <= '0';
		machineStatusAlterationPreCSR  <= '0';
		userStatusAlterationPreCSR     <= '0';
		machineStatusAlterationPostCSR <= '0';
		userStatusAlterationPostCSR    <= '0';
		zeroCntCSR                     <= '0';
		ret_from_epc				   <= '0';
		selALU						   <= '0';
		selPC4						   <= '0';
		selMem						   <= '0';
		cmp_selALUop2				   <= '0';
		cmp_selop2				       <= '0';
		isCSRInstruction   			   <= '0';
		CSR_from_WB					   <= '0';

		-- produce all the signals that are part of the control word
		-- TODO: CHECK IF ALL THE NEWLY ADDED CONTROL SIGNALS ARE BEING DRIVEN CORRECTLY 
		-- FIXME: organize code in a different way to avoid repetitions of statements that set control signals
		IF (opcode = Loads) THEN
			inst_type 	 <= I_type;
			MuxCode      <= iTypeImm;
			selJL        <= '1';
			selP1        <= '1';
			selImm 		 <= '1';
			selASU 		 <= '1'; -- to use to adder to compute the address
			startDARU    <= '1'; -- no check is performed over read data alignment
			nBytes       <= func3(1) & (func3(1) OR func3(0)); --nByte = 00 => Load Byte , nByte = 01 => Load Half, nByte = 11 => Load Word
			ldByteSigned <= NOT(func3(2)) AND NOT(func3(1)) AND NOT(func3(0));
			--ldHalfSigned <= NOT(func3(2)) AND func3(0); -- modified Gianluca
			ldHalfSigned <= NOT(func3(2)) and  NOT(func3(1)) and func3(0);
			load         <= func3(2) OR func3(1);
			writeRegFile <= '1';
			selMem		 <= '1';
		ELSIF (opcode = Stores) THEN
			inst_type <= S_type;
			muxCode <= sTypeImm;
			selJL   <= '1';
			selP1   <= '1'; -- added Gianluca
			selImm 	<= '1';
			selASU	<= '1';
			checkMisalignedDAWU <= '1';
			startDAWU           <= '1'; -- no check is performed over the alignment of data to be written in memory
			nBytes              <= func3(1) & (func3(1) OR func3(0));
		ELSIF ((opcode = Arithmetic)) THEN
			IF (func7(0) = '1') THEN -- mul/div
				-- illegalInstrFlag <= dividedByZeroOut; FIXME: WHY? THERE ARE TWO DIFFERENT EXCEPTIONS FOR DIVISION BY ZERO AND ILLEGAL INSTRUCTIONS...
				inst_type 		 <= R_type;
				selP1            <= '1';
				selJL 			 <= '1';
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
				IF (func3(2) = '0') THEN -- multiplication
					selAAL <= (NOT (func3(1) OR func3(0)));
					selAAH <= ((func3(1) OR func3(0)));
				ELSIF (func3(2) = '1') THEN -- division
					selAAL <= (func3(1));     --Remainder
					selAAH <= NOT (func3(1)); --Quotient
				END IF;
				selAAU       <= '1';
				writeRegFile <= '1';
				selALU		 <= '1';
			ELSIF (func3 = "000") THEN -- add/sub
				IF (opcode(5) = '1') THEN  -- ADD
					inst_type <= R_type;
				ELSE -- ADDI
					inst_type <= I_type;
				END IF;
				muxCode      <= iTypeImm;
				selImm       <= NOT(opcode(5));
				selP1        <= '1';
				selJL		 <= '1';
				selP2        <= opcode(5);
				addSubBar    <= func7(5) AND opcode(5);
				selASU       <= '1';
				selALU		 <= '1';
				writeRegFile <= '1';
			ELSIF (func3 = "010" OR func3 = "011") THEN -- comparisons
				IF (opcode(5) = '1') THEN  -- ADD
					inst_type <= R_type;
				ELSE -- ADDI
					inst_type <= I_type;
				END IF;
				muxCode                   <= iTypeImm;
				selImm                    <= NOT(opcode(5));
				selP1                     <= '1';
				selJL					  <= '1';
				selP2                     <= opcode(5);
				comparedSignedUnsignedBar <= NOT(func3(0));
				setZeroOrOne 			  <= '1';
				cmp_selALUop2			  <= '1';
			ELSIF (func3 = "100" OR func3 = "110" OR func3 = "111") THEN -- logical
				IF (opcode(5) = '1') THEN  -- ADD
					inst_type <= R_type;
				ELSE -- ADDI
					inst_type <= I_type;
				END IF;
				muxCode      <= iTypeImm;
				selImm       <= NOT(opcode(5));
				selP1        <= '1';
				selJL		 <= '1';
				selP2        <= opcode(5);
				selLogic     <= func3(1 DOWNTO 0);
				selLLU       <= '1';
				selALU 		 <= '1';
				writeRegFile <= '1';
			ELSIF (func3 = "001" OR func3 = "101") THEN -- shift
				IF (opcode(5) = '1') THEN  -- ADD
					inst_type <= R_type;
				ELSE -- ADDI
					inst_type <= I_type;
				END IF;
				muxCode      <= iTypeImm;
				selImm       <= NOT(opcode(5));
				selP1        <= '1';
				selJL		 <= '1';
				selP2        <= opcode(5);
				selShift     <= func3(2) & func7(5);
				selBSU       <= '1';
				selALU 		 <= '1';
				writeRegFile <= '1';
			END IF;
		ELSIF ((opcode = ImmediateArithmetic)) THEN --Immediate
			IF (func3 = "000") THEN -- add/sub
				IF (opcode(5) = '1') THEN  -- ADD
					inst_type <= R_type;
				ELSE -- ADDI
					inst_type <= I_type;
				END IF;
				muxCode      <= iTypeImm;
				selImm       <= NOT(opcode(5));
				selP1        <= '1';
				selJL		 <= '1';
				selP2        <= opcode(5);
				addSubBar    <= func7(5) AND opcode(5);
				selASU       <= '1';
				selALU		 <= '1';
				writeRegFile <= '1';
			ELSIF (func3 = "010" OR func3 = "011") THEN -- comparisons
				IF (opcode(5) = '1') THEN  -- ADD
					inst_type <= R_type;
				ELSE -- ADDI
					inst_type <= I_type;
				END IF;
				muxCode                   <= iTypeImm;
				selImm                    <= NOT(opcode(5));
				selP1                     <= '1';
				selJL					  <= '1';
				selP2                     <= opcode(5);
				comparedSignedUnsignedBar <= NOT(func3(0));
				cmp_selALUop2			  <= '1';
				setZeroOrOne 			  <= '1';
			ELSIF (func3 = "100" OR func3 = "110" OR func3 = "111") THEN -- logicals
				IF (opcode(5) = '1') THEN  -- ADD
					inst_type <= R_type;
				ELSE -- ADDI
					inst_type <= I_type;
				END IF;
				muxCode      <= iTypeImm;
				selImm       <= NOT(opcode(5));
				selP1        <= '1';
				selJL	     <= '1';
				selP2        <= opcode(5);
				selLogic     <= func3(1 DOWNTO 0);
				selLLU       <= '1';
				selALU		 <= '1';
				writeRegFile <= '1';
			ELSIF (func3 = "001" OR func3 = "101") THEN -- shift
				IF (opcode(5) = '1') THEN  -- ADD
					inst_type <= R_type;
				ELSE -- ADDI
					inst_type <= I_type;
				END IF;
				muxCode      <= iTypeImm;
				selImm       <= NOT(opcode(5));
				selP1        <= '1';
				selJL		 <= '1';
				selP2        <= opcode(5);
				selShift     <= func3(2) & func7(5);
				selBSU       <= '1';
				selALU		 <= '1';
				writeRegFile <= '1';
			END IF;
		ELSIF (opcode = JumpAndLink) THEN -- JAL
			inst_type 	 <= J_type;
			muxCode      <= jTypeImm;
			writeRegFile <= '1';
			selPC        <= '1'; -- use PC as first operand
			selImm		 <= '1';
			selASU		 <= '1';
			selPC4 		 <= '1'; -- write the value of PC+4 in the selected register
		ELSIF (opcode = JumpAndLinkRegister) THEN -- JALR
			inst_type 	 <= J_type;
			muxCode      <= iTypeImm;
			-- selInc4PC    <= '1'; TODO: VERIFY WHAT THIS ASSIGNMENT WAS NEEDED FOR
			writeRegFile <= '1';
			selJL        <= '1';
			selImm		 <= '1';
			selASU		 <= '1';
			selP1        <= '1';
			selPC4		 <= '1';
		-- DONE: REVISE THE DATAPATH FOR CONDITIONAL BRANCHES, YOU NEED TO FIND A WAY TO USE BOTH THE ADDER AND THE 
		-- COMPARISON CIRCUITRY WITH DIFFERENT OPERANDS:
		-- FOR BEQ: ADDER (PC,imm), CMP(reg,reg)
		-- FOR SLT: CMP(reg,reg)
		-- FOR SLTI: CMP(reg,imm)
		ELSIF (opcode = Branch) THEN -- conditional branch
			-- the actual check over the branch result is performed in the datapath, but we still need to
			-- inform the CU about the result since there could be the need to reset the pipeline
			inst_type				  <= B_type;
			muxCode                   <= bTypeImm;
			selP1                     <= '1';
			selImm                    <= '1';
			comparedSignedUnsignedBar <= NOT(func3(1));
			selPC                     <= '1';
			cmp_selop2			  	  <= '1';
			selASU					  <= '1';
		ELSIF (opcode = LoadUpperImmediate OR opcode = AddUpperImmediatePC) THEN -- LUI
			inst_type 	 <= U_type;
			muxCode      <= uTypeImm;
			selImm       <= '1';
			selASU       <= '1';
			selALU		 <= '1';
			writeRegFile <= '1';
			pass         <= opcode(5);
			--addSubBar    <= NOT (opcode(5)); -- modified Gianluca
			addSubBar    <= '0';
			selAuipc     <= NOT (opcode(5));
			selJL		 <= '1';
		ELSIF (opcode = SystemAndCSR) THEN
			IF (func3 = "000" AND func12 = X"000") THEN
				inst_type <= I_type;
				ecallFlag <= '1';
			ELSIF (func3 /= "000" OR (func12 /= X"302" AND func12 /= X"002")) THEN
				 -- CSR instructions: only the read operation is performed directly in decode
				writeRegFile 	   <= '1';
				selCSRAddrFromInst <= '1';
				selCSR             <= '1' AND validAccessCSR;
				illegalInstrFlag   <= NOT(validAccessCSR);
				selALU			   <= '1';
				isCSRInstruction   <= '1';
				inst_type		   <= I_type;
			ELSE
				inst_type 		   <= S_type;
				ret_from_epc 	   <= '1';
			END IF;
		ELSE
			-- invalid instruction
			IF (DEC_valid = '1') THEN
				illegalInstrFlag <= '1';
			END IF;
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

		-- activation conditions for enable signals
		
		-- as for D2E_en, with an additional check on completedDARU1 which is a set asynchronously set when memory read is complete
		IF ((D2E_en_temp = '1' OR DEC_valid = '0') AND completedDARU1 = '1') THEN
			GI2D_en_temp   <= '1';
		END IF;
		-- as for E2M_en, but there are no conditions which require multiple cycles to be solved
		IF ((E2M_en_temp = '1' OR EX_valid = '0') AND DEC_valid = '1') THEN
			D2E_en_temp    <= '1';
		END IF;
		-- a valid instruction can be promoted in memory when either the instruction currently in memory is invalid or it
		-- is moving in write-back and it has completed execution in EX (wait for 1 cycle for non AAU, wait until completedAAU set otherwise)
		IF ((M2WB_en_temp = '1' OR M_valid = '0') AND EX_valid = '1' AND (is_AAU_used = '0' OR completedAAU = '1')) THEN
			E2M_en_temp    <= '1';
		END IF;
		-- completedDAWU and completedDARU2 are raised also when the MEM stage is occupied by an
		-- instruction which doesn't require memory access
		IF (instructionDone = '1' AND ((completedDAWU = '1' AND is_store_in_mem = '1') OR (completedDARU2 = '1' AND is_load_in_mem = '1') OR (is_store_in_mem = '0' AND is_load_in_mem = '0')) AND M_valid = '1') THEN
			M2WB_en_temp   <= '1';
		END IF;

		-- misprediction handling
		IF (branch_taken = '1') THEN
			GI2D_rst <= '1';
			D2E_rst  <= '1';
			IF (instructionDone = '1') THEN
				E2M_rst <= '1'; -- E2M is reset only if the instruction in memory can pass to WB
			END IF;
		END IF;

		CASE p_state IS
			WHEN idle =>
				IF (WB_ret_from_epc = '1' AND WB_valid = '1') THEN --mret/uret
					GI2D_en_temp   <= '0';
					D2E_en_temp    <= '0';
					E2M_en_temp    <= '0';
					M2WB_en_temp   <= '0';
			 		ldValueCSR 	   <= "010";
					ldCntCSR   	   <= '1';
				-- EXCEPTIONS HAVE AN HIGHER PRIORITY THAN INTERRUPTS!
				ELSIF (exceptionRaise = '1') THEN
					GI2D_en_temp   <= '0';
					D2E_en_temp    <= '0';
					E2M_en_temp    <= '0';
					M2WB_en_temp   <= '0';
					ldValueCSR     <= "111";
			 		ldCntCSR       <= '1';
			 		selMedeleg_CSR <= '1';
				ELSIF (interruptRaise = '1') THEN
					GI2D_en_temp   <= '0';
					D2E_en_temp    <= '0';
					E2M_en_temp    <= '0';
					M2WB_en_temp   <= '0';
					zeroCntCSR     <= '1';
			 		mipCCLdDisable <= '1';
			 		selMideleg_CSR <= '1';
				ELSIF (hazEX = '1') THEN
					GI2D_en_temp   <= '0';
					D2E_en_temp    <= '0';
				ELSIF (hazM = '1') THEN
					GI2D_en_temp   <= '0';
					D2E_en_temp    <= '0';
				END IF;
			-- exception and interrupt handling states
			WHEN checkDelegation =>
				CSR_from_WB	   <= '1';
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				selMideleg_CSR <= interruptRaise; -- ok, interruptRaise must remain high until the interrupt is served
				selMedeleg_CSR <= exceptionRaise; -- as before, for exceptionRaise
				ldDelegation   <= '1';
				mipCCLdDisable <= '1';
			WHEN updateTrapValue =>
				CSR_from_WB	   <= '1';
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				mirrorUserCU_exint   <= NOT(delegationMode(0));
				selRomAddress  <= '1';
				ldValueCSR     <= "001";
				ldCntCSR       <= '1';
				selTval_CSR    <= '1';
				writeRegBank_exint   <= '1';
				mipCCLdDisable <= '1';
			WHEN updateMip => -- don't count up while you are here, afterward you have to update the UIP
				CSR_from_WB	   <= '1';
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				mirrorUserCU_exint   <= '0';
				selRomAddress  <= '1';
				selCCMip_CSR   <= '1';
				writeRegBank_exint   <= '1';
				mipCCLdDisable <= '1';
			WHEN updateUip => --new
				CSR_from_WB	   <= '1';
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				selRomAddress  <= '1';
				mirrorUserCU_exint   <= '1';
				upCntCSR       <= '1';
				selCCMip_CSR   <= '1';
				writeRegBank_exint   <= '1';
				mipCCLdDisable <= '1';
			WHEN updateCause =>
				CSR_from_WB	   <= '1';
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				mirrorUserCU_exint   <= NOT(delegationMode(0));
				selRomAddress  <= '1';
				upCntCSR       <= '1';
				selCause_CSR   <= '1';
				writeRegBank_exint   <= '1';
				mipCCLdDisable <= '1';
			WHEN updateEpc =>
				CSR_from_WB	   <= '1';
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				mirrorUserCU_exint   <= NOT(delegationMode(0));
				selRomAddress  <= '1';
				upCntCSR       <= '1';
				selPC_CSR      <= '1';
				writeRegBank_exint   <= '1';
				mipCCLdDisable <= '1';
			WHEN readTvec1 =>
				CSR_from_WB	   <= '1';
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				mirrorUserCU_exint   <= NOT(delegationMode(0));
				selRomAddress  <= '1';
				mipCCLdDisable <= '1';
			WHEN readTvec2 =>
				CSR_from_WB	   <= '1';
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				selRomAddress  <= '1';
				mirrorUserCU_exint   <= NOT(delegationMode(0));
				ldMachine      <= delegationMode(0);
				ldUser         <= NOT(delegationMode(0));
				selMepc_CSR    <= '1'; -- TODO: WHY WAS THIS COMMENTED?
				upCntCSR       <= '1';
				mipCCLdDisable <= '1';
				IF (modeTvec = "00") THEN
					selInterruptAddressDirect <= '1';
				ELSIF (modeTvec = "01") THEN
					selInterruptAddressVectored <= '1';
				END IF;
			WHEN readMstatus =>
				CSR_from_WB	   				  <= '1';
				GI2D_en_temp   				  <= '0';
				D2E_en_temp    				  <= '0';
				E2M_en_temp    				  <= '0';
				M2WB_en_temp   				  <= '0';
				mirrorUserCU_exint     			  <= '0';
				selRomAddress  				  <= '1';
				mipCCLdDisable 				  <= '1';
			WHEN updateMstatus =>
				CSR_from_WB	   				  <= '1';
				GI2D_en_temp   				  <= '0';
				D2E_en_temp    				  <= '0';
				E2M_en_temp    				  <= '0';
				M2WB_en_temp   				  <= '0';
				mirrorUserCU_exint                  <= '0';
				selRomAddress                 <= '1';
				loadMieUieField_exint               <= ldMieUieField;
				machineStatusAlterationPreCSR <= delegationMode(0);
				userStatusAlterationPreCSR    <= NOT(delegationMode(0));
				writeRegBank_exint                  <= '1';
				mipCCLdDisable                <= '1';
			WHEN updateUstatus =>
				CSR_from_WB	   				  <= '1';
				GI2D_en_temp   				  <= '0';
				D2E_en_temp    				  <= '0';
				E2M_en_temp    				  <= '0';
				M2WB_en_temp   				  <= '0';
				mirrorUserCU_exint                  <= '1';
				selRomAddress                 <= '1';
				machineStatusAlterationPreCSR <= delegationMode(0);
				userStatusAlterationPreCSR    <= NOT(delegationMode(0));
				writeRegBank_exint                  <= '1';
				mipCCLdDisable                <= '1';
			    ldMachine      				  <= delegationMode(0);
				ldUser                        <= NOT(delegationMode(0));
			WHEN retEpc =>
				CSR_from_WB	   <= '1';
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				mirrorUserCU_exint   <= NOT(mretOrUretBar);
				selRomAddress  <= '1';
			    ldValueCSR 	   <= "100";
			    ldCntCSR   	   <= '1';
			WHEN retReadMstatus =>
				CSR_from_WB	   <= '1';
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				selMepc_CSR    <= '1';
				mirrorUserCU_exint   <= '0';
				selRomAddress  <= '1';
			WHEN retUpdateMstatus =>
				CSR_from_WB	   <= '1';
				GI2D_en_temp   <= '0';
				D2E_en_temp    <= '0';
				E2M_en_temp    <= '0';
				M2WB_en_temp   <= '0';
				mirrorUserCU_exint   <= '0';
				selRomAddress  <= '1';
				IF (mretOrUretBar = '1') THEN
					ldMachine <= previousPRV(0);
					ldUser    <= NOT(previousPRV(0));
				ELSE
					ldMachine <= '0';
					ldUser    <= '1';
				END IF;
				loadMieUieField_exint                <= ldMieUieField;
				machineStatusAlterationPostCSR <= mretOrUretBar;
				userStatusAlterationPostCSR    <= NOT(mretOrUretBar);
				writeRegBank_exint                   <= '1';
			WHEN retUpdateUstatus =>
				CSR_from_WB	   				   <= '1';
				GI2D_en_temp   				   <= '0';
				D2E_en_temp    				   <= '0';
				E2M_en_temp    				   <= '0';
				M2WB_en_temp   				   <= '0';
				selRomAddress                  <= '1';
				mirrorUserCU_exint                   <= '1';
				machineStatusAlterationPostCSR <= mretOrUretBar;
				userStatusAlterationPostCSR    <= NOT(mretOrUretBar);
				writeRegBank_exint                   <= '1';
				zeroCntCSR                     <= '1';
				GI2D_rst 					   <= '1';
				D2E_rst 					   <= '1';
				E2M_rst 					   <= '1';
				M2WB_rst 					   <= '1';
			WHEN hazEX_state =>
				-- you can transition to hazEX, idle or checkDelegation
				IF (WB_ret_from_epc = '1' AND WB_valid = '1') THEN --mret/uret
					GI2D_en_temp   <= '0';
					D2E_en_temp    <= '0';
					E2M_en_temp    <= '0';
					M2WB_en_temp   <= '0';
			 		ldValueCSR 	   <= "010";
					ldCntCSR   	   <= '1';
				ELSIF (exceptionRaise = '1') THEN
					GI2D_en_temp   <= '0';
					D2E_en_temp    <= '0';
					E2M_en_temp    <= '0';
					M2WB_en_temp   <= '0';
					ldValueCSR     <= "111";
			 		ldCntCSR       <= '1';
			 		selMedeleg_CSR <= '1';
				ELSIF (interruptRaise = '1') THEN
					GI2D_en_temp   <= '0';
					D2E_en_temp    <= '0';
					E2M_en_temp    <= '0';
					M2WB_en_temp   <= '0';
					zeroCntCSR     <= '1';
			 		mipCCLdDisable <= '1';
			 		selMideleg_CSR <= '1';
				ELSIF (hazard_solved /= '1') THEN
					GI2D_en_temp   <= '0';
					D2E_en_temp    <= '0';
				END IF;
			WHEN hazM_state =>
				-- you can transition to hazM, idle or checkDelegation
				IF (WB_ret_from_epc = '1' AND WB_valid = '1') THEN --mret/uret
					GI2D_en_temp   <= '0';
					D2E_en_temp    <= '0';
					E2M_en_temp    <= '0';
					M2WB_en_temp   <= '0';
			 		ldValueCSR 	   <= "010";
					ldCntCSR   	   <= '1';
				ELSIF (exceptionRaise = '1') THEN
					GI2D_en_temp   <= '0';
					D2E_en_temp    <= '0';
					E2M_en_temp    <= '0';
					M2WB_en_temp   <= '0';
					ldValueCSR     <= "111";
			 		ldCntCSR       <= '1';
			 		selMedeleg_CSR <= '1';
				ELSIF (interruptRaise = '1') THEN
					GI2D_en_temp   <= '0';
					D2E_en_temp    <= '0';
					E2M_en_temp    <= '0';
					M2WB_en_temp   <= '0';
					zeroCntCSR     <= '1';
			 		mipCCLdDisable <= '1';
			 		selMideleg_CSR <= '1';
				ELSIF (hazard_solved /= '1') THEN
					GI2D_en_temp   <= '0';
					D2E_en_temp    <= '0';
				END IF;
			WHEN waitForStore =>
				CSR_from_WB	   	   <= '1';
				-- at the end of interrupt handling, if the operation in memory is a store
				-- DONE: HANDLE THE PASSAGE BETWEEN INTERRUPT SEQUENCES AND waitForStore 
				-- (DURING EXCEPTIONS HANDLING AND M/URET THERE IS NO NEED TO WAIT FOR A STORE INSTRUCTION TO COMPLETE 
				-- SINCE THE PIPELINE IS STOPPED WHEN THE EXCEPTION/M/URET IS DETECTED)
				IF (completedDAWU = '1')  THEN
					GI2D_en_temp   <= '0';
					D2E_en_temp    <= '0';
					E2M_en_temp    <= '0';
					M2WB_en_temp   <= '0';
					-- DON'T RESET THE GI2D, IT HAS ALREADY BEEN DONE BEFORE (RESETTING IT WOULD LOAD IN THE PC A WRONG VALUE)
					D2E_rst   	   <= '1';
					E2M_rst   	   <= '1';
					M2WB_rst	   <= '1';
				ELSE
					GI2D_en_temp   <= '0';
					D2E_en_temp    <= '0';
					E2M_en_temp    <= '0';
					M2WB_en_temp   <= '0';
				END IF;
		END CASE;
	END PROCESS;

	-- driving of the enable signals depending on the temp values
	GI2D_en <= GI2D_en_temp;
	D2E_en  <= D2E_en_temp;
	E2M_en  <= E2M_en_temp;
	M2WB_en <= M2WB_en_temp;

	sequential : PROCESS (clk, rst) BEGIN
		IF rst = '1' THEN
			p_state <= idle;
		ELSIF (clk = '1' AND clk'EVENT) THEN
			p_state <= n_state;
		END IF;
	END PROCESS sequential;
END behavioral;
