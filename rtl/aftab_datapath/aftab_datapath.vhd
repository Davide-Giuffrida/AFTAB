-- **************************************************************************************
--	Filename:	aftab_datapath.vhd
--	Project:	CNL_RISC-V
--  Version:	1.0
--	Date:		05 April 2022
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
--	Datapath of the AFTAB core
--
-- **************************************************************************************

-- STRUCTURAL MODIFICATIONS NEEDED: ------------------

-- TODO: FIX DARU AND DAWU TO MAKE SURE DATA ARE NOT DELETED DURING OPERATION AND THE FSM IS NOT
-- RESTARTED WHEN DATA ARE STALLED INSIDE THE MEMORY STAGE.

-- TODO: MAKE THE DARU FSM A MEALY-MOORE MIXED ONE, SO THAT IT IS POSSIBLE TO REDUCE THE DURATION
-- OF THE MEMORY FETCH: THIS COULD BE PARTICULARLY CRITICAL FOR THE FIRST DARU, WHICH COULD FETCH
-- INSTRUCTIONS IN TWO CLOCK CYCLES (INSTEAD OF HAVING TO WAIT FOR AN ADDITIONAL ONE BEFORE RAISING
-- THE completedDARU SIGNAL)

-- TODO: WHEN CHECKING FOR HAZARDS/BYPASSES YOU HAVE TO RAISE THE FLAG CORRESPONDING TO THE DEPENDENCY
-- THAT HAS BEEN DETECTED ONLY IF THE OPERATION WHICH PERFORMS THE WRITE HAS THE CORRESPONDING CONTROL
-- SIGNAL ON. (writeRegFile or writeRegBank)

-- TODO: REMOVE writeRB_inst

-- TODO: BYPASS UNIT BETWEEN WB AND DECODE IS NECESSARY IF WE WANT TO AVOID ADDING AN HAZARD DETECTION
-- UNIT TO SPOT DEPENDENCIES BETWEEN WB AND DECODE. PAY ATTENTION TO CONSIDER THAT SOME CSR INSTRUCTIONS
-- HAVE A MIRRORING PHASE IN WRITEBACK: IT IS NOT NECESSARY TO WAIT FOR BOTH THE WRITE-BACK CYCLES BEFORE
-- PROMOTING THE INSTRUCTION TO EXECUTE SINCE YOU CAN SIMPLY READ THE VALUE IN THE M2WB REGISTER, BUT PAY
-- ATTENTION TO IDENTIFY THE CORRECT HAZARDS: YOU CAN COMPARE THE LSbs, BUT PAY ATTENTION TO THE CASE IN
-- WHICH YOU ARE READING A MACHINE REGISTER AND WRITING A USER ONE --> IN THIS CASE YOU SHOULDN'T THROW AN
-- HAZARD, BECAUSE MIRRORING IS ONLY FROM MACHINE TO USER.

-- TODO: VERIFY THAT ALL THE REGISTERS CAN BE DRIVEN FROM THE CU WHEN HANDLING EXCEPTIONS/INTERRUPTS 
-- AND UNPACK THE INSTRUCTION AT EACH STAGE!

-- DONE: MAKE THE REGISTER BANK ASYNCHRONOUS IN READ, OTHERWISE IT IS NOT GOING TO BE ABLE TO READ THE DELEGATION
-- REGISTER DURING THE checkDelegation STATE! YOU HAVE TO MAKE AN ADDITIONAL SIGNAL BEFORE THE curr REG.
-- UPDATE THE MIDELEG AND MEDELEG INPUTS FOR ICCD TOO, THEY HAVE TO BE ASSIGNED THE VALUE READ DIRECTLY FROM
-- THE REGISTER BANK DURING INT/EX HANDLING

-- FIXME: THERE IS A PROBLEM WITH MIRRORING AS IT IS CURRENTLY IMPLEMENTED: HOW TO UPDATE THE MACHINE REGISTER
-- IF WE ARE MODIFYING THE USER ONE? IN THE CURRENT IMPLEMENTATION IT IS IMPOSSIBLE TO DO SO BY RELYING EXCLUSIVELY
-- ON MIRRORING, BECAUSE THE MIRROR PHASE ACTS ONLY ON THE USER REGISTER (IT WOULD OVERWRITE THE USER REGISTER AGAIN).
-- WHAT WE COULD DO IS TO MODIFY THE MACHINE REGISTER FIRST, SO THAT THE MIRRORING PHASE WILL ACT ON THE CORRESPONDING
-- USER REGISTER AUTOMATICALLY.
-- ----> AFTER AN ANALYSIS OF THE ISA: THE REASON WHY MIRRORING HAS BEEN DONE ONLY IN ONE DIRECTION IS THAT
-- MACHINE SHOULD BE ABLE TO MODIFY EVERYTHING ABOUT USER MODE, WHILE USER MODE IS ABLE TO MODIFY ONLY USER REGS.
-- THIS MEANS THAT IN CASE WE MODIFY USTATUS, WE DON'T AFFECT THE USER BITS IN MSTATUS. THE AFTAB IS STILL NOT
-- COHERENT WITH THE ISA, BECAUSE THERE ARE NOT BITS RELATED TO USER MODE IN MSTATUS. THE USER IS ABLE TO READ
-- AND MODIFY THE REGISTER USTATUS, BUT THE MODIFICATIONS WILL HAVE NO EFFECT.

-- DONE: ENSURE THAT CSR INSTRUCTIONS ARE ABLE TO PERFORM A WRITE OPERATION OVER BOTH THE DESTINATION
-- REGISTER AND THE CSR ONE (WHICH IS USED BOTH AS AN OPERAND AND A DESTINATION)

-- TODO: IF THERE IS A MISPREDICTION WHILE AN INSTRUCTION WHICH INVOLVES MIRRORING IS IN WRITE-BACK
-- YOU NEED TO AVOID DELETING THE WRITE-BACK CONTENTS FOR AN ADDITIONAL CLOCK CYCLE.
-- ----> IN THEORY YOU CAN SIMPLY AVOID RESETTING THE M2WB REGISTER (DISABLE M2WB_en AND MW2B_rst).
-- BUT YOU HAVE TO TAKE INTO ACCOUNT THE CASE IN WHICH YOU HAVE A MIRRORING INSTRUCTION IN WRITE-BACK:
-- IN THIS CASE YOU SHOULDN'T DELETE THE INSTRUCTION IN MEMORY TOO (DEACTIVATE THE ENABLE BUT DO NOT
-- RESET THE E2M REGISTER) --> NOT OPTIMAL, YOU ARE RESETTING THE PC EVERY CYCLE UNTIL THE BRANCH
-- LEAVES MEMORY. ---> PARTIALLY DONE, TO BE VERIFIED

-- TODO:/DONE REVISE INTERRUPT AND EXCEPTION HANDLING. IN ORDER TO MINIMIZE INTERRUPT LATENCY IT COULD
-- BE POSSIBLE TO RAISE THE INTERRUPT IMMEDIATELY AFTER THE COMPLETION OF THE INSTRUCTION THAT WAS
-- LOCATED IN WRITE-BACK WHEN THE INTERRUPT WAS THROWN (WAIT FOR A VALID INSTRUCTION TO COME): WHEN
-- THE INSTRUCTION COMPLETES YOU CAN STOP THE PIPELINE AND LOAD THE ISR, SAVING AS A RETURN ADDRESS
-- THE PC+4 OF THE WRITE-BACK INSTRUCTION (OR THE INSTRUCTION IN MEMORY IF IT WAS A STORE).
-- IN THIS WAY YOU AVOID THE PROPAGATION OF THE INTERRUPT FLAG THROUGH THE WHOLE PIPELINE.
-- FOR THE EXCEPTION YOU STILL NEED PROPAGATION THROUGH THE PIPELINE, BECAUSE AN EXCEPTION IS 
-- ASSOCIATED TO THE RELATED INSTRUCTION. YOU CAN STOP THE PIPELINE AS SOON AS AN EXCEPTION IS RAISED
-- (FOR THE STAGES THAT COME BEFORE THE EXCEPTION) AND SAVE THE CAUSE IN A NON-PIPELINE REGISTER.
-- THE EXCEPTION GLOBAL STATE BLOCKS EVERY STAGE BEFORE THE EXCEPTION AND IT ALLOWS THE PROPAGATION
-- OF THE EXCEPTING INSTRUCTION: IF THERE IS A MISPREDICTION, THEN THE EXCEPTION STATE IS CLEARED.
-- HOW TO HANDLE INTERRUPTS, EXCEPTIONS AND HAZARDS:

-- 1) DEFINE AS A NEW CONDITION FOR RESETTING A REGISTER X2Y_rst OR (X2Y_en' AND Y2Zen) --> DONE

-- 2) DEFINE A VALID BIT FOR AN INSTRUCTION PASSING THROUGH A SPECIFIC STAGE OF THE PIPELINE. THIS IS
-- NEEDED BECAUSE YOU NEED TO IDENTIFY IF AN INSTRUCTION PASSING IN WRITE-BACK IS VALID, SINCE A
-- POSSIBLE INTERRUPT SHOULD BE HANDLED WHEN THE INSTRUCTION IS OVER. --> DONE

-- HAZARDS: YOU HAVE TO STALL THE D2E REGISTER AND THE PREVIOUS ONES UNTIL THE HAZARD IS SOLVED,
-- WHICH MEANS THAT YOU HAVE TO WAIT FOR THE HAZARD FLAG TO REACH WRITE-BACK. HAZARD HANDLING 
-- REQUIRES AN ADDITIONAL STATE FOR THE CU, WHERE THE ENABLE SIGNALS G2DI_en AND D2E_en ARE KEPT LOW.
-- FROM THIS STATE WE CAN MOVE TO THE NORMAL STATE AND THE INTERRUPT/EXCEPTION MANAGEMENT STATES.

-- INTERRUPTS: YOU HAVE TO STALL ALL THE PIPELINE REGISTERS (low enable) WHILE THE INTERRUPT IS BEING
-- HANDLED (THE INSTRUCTION IN WRITE-BACK COMPLETES). THE LAST REGISTER (M2WB) MUST NOT BE RESET WHILE
-- THE HANDLING IS IN PROGRESS, BECAUSE WE WOULD LOSE THE PCPLUS4 VALUE. IN ORDER TO DO SO WE CAN DEFINE
-- THE ENABLE CONDITION FOR THIS REGISTER AS: instructionDone AND (NOT(interruptHandling) AND NOT(exceptionHandling))
-- WHERE THE XHandling SIGNALS ARE RAISED BY THE NORMAL "MEALY" STATE WHEN interruptRaise AND instructionDone
-- OR exceptionFlag AND instructionDone, WHERE exceptionFlag IS THE FLAG PROPAGATING THROUGH THE PIPELINE.
-- IF AN HAZARD HAPPENS WHEN YOU START HANDLING THE INTERRUPT YOU CAN SIMPLY PRIORITIZE MOVING TO THE 
-- INTERRUPT HANDLING STATE INSTEAD OF THE HAZARD HANDLING ONE, BECAUSE IT IS NOT IMPORTANT TO HANDLE THE 
-- HAZARD WHILE HANDLING THE INTERRUPT (EVERYTHING WILL BE STALLED AND RESET ANYWAY). IF A MISPREDICTION
-- HAPPENS WHILE YOU ARE STARTING TO HANDLE THE INTERRUPT THEN THE STATE WILL MOVE TO NORMAL AND NOTHING
-- WILL HAPPEN UNTIL THE NEXT INSTRUCTION REACHES WRITE-BACK. WHEN YOU SELECT THE PCPLUS4 TO BE WRITTEN IN
-- XEPC YOU NEED TO TAKE EITHER THE ONE FROM WRITE-BACK OR THE ONE FROM MEMORY (IF THE INSTRUCTION IN MEMORY IS
-- A STORE): YOU NEED TO IMPLEMENT AN ADDITIONAL STATE TO THE HW HANDLING SEQUENCE TO WAIT FOR THE STORE 
-- INSTRUCTION TO COMPLETE BEFORE TAKING THE PCPLUS4 FROM THE MEMORY STAGE.

-- EXCEPTIONS: YOU HAVE TO STALL ALL THE STAGES BEFORE THE EXCEPTION WHEN THE EXCEPTION ITSELF IS RAISED.
-- THIS CAN'T BE DONE BY A CU STAGE, SO YOU NEED SOME CIRCUITRY IN THE PIPELINE WHICH RECEIVES THE EXCEPTION
-- FLAGS AND STORES THE STAGE WHERE THE LATEST EXCEPTION WAS THROWN. IN CASE A NEW EXCEPTION IS THROWN LATER
-- YOU NEED TO OVERWRITE THE VALUE WITH THE NEW ONE. THIS CIRCUITRY STALLS ALL THE STAGES BEFORE THE EXCEPTION
-- AND IT IS RESET ONLY BY THE rst COMING FROM THE CU (WHEN MISPREDICTING OR STARTING EX/INT HANDLERS). IF 
-- AN INTERRUPT AND AN EXCEPTION ARE COMPETING IN WRITE-BACK YOU NEED TO HANDLE THE EXCEPTION AND NEGLECT THE
-- INTERRUPT, WHICH WILL BE HANDLED AFTERWARD. HAZARDS ARE HANDLED AS USUAL, BUT THEY SHOULD NOT BE RAISED BY
-- THE EXCEPTING INSTRUCTION (TO OPTIMIZE PERFORMANCES). EXCEPTING INSTRUCTION SHOULD NOT BE ABLE TO MODIFY
-- THE MICROARCHITECTURAL STATE OF THE SYSTEM, SO YOU SHOULD AVOID PERFORMING MEMORY READ OR WRITE-BACK WITH
-- THOSE INSTRUCTIONS (YOU COULD USE A FLAG THAT PROPAGATES THROUGH THE PIPELINE).

-- DONE: CHANGE THE PORTS TO INCLUDE THE NEWLY ADDED SIGNALS

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE WORK.constants.ALL;
ENTITY aftab_datapath IS
	GENERIC
		(len : INTEGER := 32);
	PORT
	(
		-- general signals
		clk                            : IN  STD_LOGIC;
		rst                            : IN  STD_LOGIC;

		-- control word
		writeRegFile                   : IN  STD_LOGIC;
		setZeroOrOne                   : IN  STD_LOGIC;
		ComparedSignedUnsignedBar      : IN  STD_LOGIC;
		selPC                          : IN  STD_LOGIC;
		selJL                          : IN  STD_LOGIC;
		selBSU                         : IN  STD_LOGIC;
		selLLU                         : IN  STD_LOGIC;
		selASU                         : IN  STD_LOGIC;
		selAAU                         : IN  STD_LOGIC;
		selP1                          : IN  STD_LOGIC;
		selP2                          : IN  STD_LOGIC;
		selImm                         : IN  STD_LOGIC;
		ldByteSigned                   : IN  STD_LOGIC;
		ldHalfSigned                   : IN  STD_LOGIC;
		load                           : IN  STD_LOGIC;
		selShift                       : IN  STD_LOGIC_VECTOR (1 DOWNTO 0);
		addSubBar                      : IN  STD_LOGIC;
		pass                           : IN  STD_LOGIC;
		selAuipc                       : IN  STD_LOGIC;
		muxCode                        : IN  STD_LOGIC_VECTOR (11 DOWNTO 0);
		selLogic                       : IN  STD_LOGIC_VECTOR (1 DOWNTO 0);
		startDAWU                      : IN  STD_LOGIC;
		startDARU                 	   : IN  STD_LOGIC;
		startMultiplyAAU               : IN  STD_LOGIC;
		startDivideAAU                 : IN  STD_LOGIC;
		signedSigned                   : IN  STD_LOGIC;
		signedUnsigned                 : IN  STD_LOGIC;
		unsignedUnsigned               : IN  STD_LOGIC;
		selAAL                         : IN  STD_LOGIC;
		selAAH                         : IN  STD_LOGIC;
		nBytes                         : IN  STD_LOGIC_VECTOR (1 DOWNTO 0);
		selCSR                         : IN  STD_LOGIC;
		writeRB_inst				   : IN  STD_LOGIC; -- the signal which has to be set to perform a write op over the RB
		checkMisalignedDAWU            : IN  STD_LOGIC;
		selCSRAddrFromInst             : IN  STD_LOGIC;
		forced_RB_read				   : IN  STD_LOGIC; -- mux selection signal between the address to be read from write-back (as exception handling) and the one to be read in decode
		inst_type					   : IN  STD_LOGIC_VECTOR (2 DOWNTO 0);
		ret_from_epc				   : IN  STD_LOGIC;
		selALU						   : IN  STD_LOGIC;
		selPC4						   : IN  STD_LOGIC;
		selMem						   : IN  STD_LOGIC;
		cmp_selALUop2				   : IN  STD_LOGIC;
		cmp_selop2				   	   : IN  STD_LOGIC;
		isCSRInstruction			   : IN  STD_LOGIC;

		-- memory signals
		writeMemDAWU                   : OUT STD_LOGIC;
		readMemDARU1                   : OUT STD_LOGIC;
		readMemDARU2                   : OUT STD_LOGIC;
		memReady1                      : IN  STD_LOGIC;  -- for the two read ports (one for GI and one for MEM)
		memReady2                      : IN  STD_LOGIC;
		memDataOut1                    : IN  STD_LOGIC_VECTOR (15 DOWNTO 0); -- data read from the first read port
		memDataOut2                    : IN  STD_LOGIC_VECTOR (15 DOWNTO 0);
		dataDAWU                       : OUT STD_LOGIC_VECTOR (15 DOWNTO 0);
		memAddr1                       : OUT STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
		memAddr2                       : OUT STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
		bytesToReadDARU1			   : OUT STD_LOGIC; -- for the first memory port (accessed only from GI stage)
		bytesPerMemAccess			   : OUT STD_LOGIC; -- for the second memory port
		
		-- instruction to be decoded
		IR                             : OUT STD_LOGIC_VECTOR (len - 1 DOWNTO 0); -- used to send to the CU the instruction to be decoded
		
		-- func3 field of the instruction currently in write-back
		WB_func3					   : OUT STD_LOGIC_VECTOR (2 DOWNTO 0);

		-- operation complete signals (and related signals to notify the CU about the instruction which reside in the pipeline)
		completedDAWU_def              : OUT STD_LOGIC;
		completedDARU1_def             : OUT STD_LOGIC;
		completedDARU2_def             : OUT STD_LOGIC;
		completedAAU                   : OUT STD_LOGIC;
		is_AAU_used					   : OUT STD_LOGIC;
		instructionDone				   : OUT STD_LOGIC;
		hazard_solved 				   : OUT STD_LOGIC;
		is_store_in_mem				   : OUT STD_LOGIC;
		is_load_in_mem				   : OUT STD_LOGIC;
		branch_taken 				   : OUT STD_LOGIC;
		DEC_valid					   : OUT STD_LOGIC;
		EX_valid					   : OUT STD_LOGIC;
		M_valid						   : OUT STD_LOGIC;
		WB_valid					   : OUT STD_LOGIC;
		WB_ret_from_epc				   : OUT STD_LOGIC;
		WB_isCSRInstruction			   : OUT STD_LOGIC;
		WB_validAccessCSR			   : OUT STD_LOGIC;

		--CSR and Interrupt inputs and outputs --> driven directly by the CU when needed
		CSR_from_WB					   : IN  STD_LOGIC;
		machineExternalInterrupt       : IN  STD_LOGIC;
		machineTimerInterrupt          : IN  STD_LOGIC;
		machineSoftwareInterrupt       : IN  STD_LOGIC;
		userExternalInterrupt          : IN  STD_LOGIC;
		userTimerInterrupt             : IN  STD_LOGIC;
		userSoftwareInterrupt          : IN  STD_LOGIC;
		platformInterruptSignals       : IN  STD_LOGIC_VECTOR (15 DOWNTO 0);
		ldValueCSR                     : IN  STD_LOGIC_VECTOR (2 DOWNTO 0);
		mipCCLdDisable                 : IN  STD_LOGIC;
		selPC_CSR                      : IN  STD_LOGIC;
		selTval_CSR                    : IN  STD_LOGIC;
		selMedeleg_CSR                 : IN  STD_LOGIC;
		selMideleg_CSR                 : IN  STD_LOGIC;
		selCCMip_CSR                   : IN  STD_LOGIC;
		selCause_CSR                   : IN  STD_LOGIC;
		selMepc_CSR                    : IN  STD_LOGIC;
		selInterruptAddressDirect      : IN  STD_LOGIC;
		selInterruptAddressVectored    : IN  STD_LOGIC;
		writeRegBank                   : IN  STD_LOGIC; -- driven directly by the CU, the actual write enable signal is writeRegBank OR write_RB_inst
		dnCntCSR                       : IN  STD_LOGIC;
		upCntCSR                       : IN  STD_LOGIC;
		ldCntCSR                       : IN  STD_LOGIC;
		zeroCntCSR                     : IN  STD_LOGIC;
		ldDelegation                   : IN  STD_LOGIC;
		ldMachine                      : IN  STD_LOGIC;
		ldUser                         : IN  STD_LOGIC;
		loadMieReg                     : IN  STD_LOGIC;
		loadMieUieField                : IN  STD_LOGIC;
		mirrorUserCU                   : IN  STD_LOGIC;
		machineStatusAlterationPreCSR  : IN  STD_LOGIC;
		userStatusAlterationPreCSR     : IN  STD_LOGIC;
		machineStatusAlterationPostCSR : IN  STD_LOGIC;
		userStatusAlterationPostCSR    : IN  STD_LOGIC;
		selRomAddress                  : IN  STD_LOGIC;
		ecallFlag                      : IN  STD_LOGIC; -- set by the CU when an ecall is in decode
		illegalInstrFlag               : IN  STD_LOGIC;
		instructionDoneCSR			   : IN  STD_LOGIC;
		validAccessCSR                 : OUT STD_LOGIC;
		readOnlyCSR                    : OUT STD_LOGIC;
		mirror                         : OUT STD_LOGIC;
		ldMieReg                       : OUT STD_LOGIC;
		ldMieUieField                  : OUT STD_LOGIC;
		interruptRaise                 : OUT STD_LOGIC;
		exceptionRaise                 : OUT STD_LOGIC;
		delegationMode                 : OUT STD_LOGIC_VECTOR (1 DOWNTO 0);
		previousPRV                    : OUT STD_LOGIC_VECTOR (1 DOWNTO 0);
		modeTvec                       : OUT STD_LOGIC_VECTOR (1 DOWNTO 0);
		selP1CSR                       : IN  STD_LOGIC;
		selImmCSR                      : IN  STD_LOGIC;
		setCSR                         : IN  STD_LOGIC;
		selReadWriteCSR                : IN  STD_LOGIC;
		clrCSR                         : IN  STD_LOGIC;

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
		hazEX						   : OUT STD_LOGIC;
		hazM						   : OUT STD_LOGIC
	);
END ENTITY aftab_datapath;
ARCHITECTURE behavioral OF aftab_datapath IS

	-- type definitions
	TYPE ex_pre_handling_state IS (no_ex, ex_misaligned_inst, ex_ecall_illegal_inst_or_ret, ex_divided_by_zero);

	SIGNAL immediate                     : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL inst                          : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL resAAH                        : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL resAAL                        : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL p1                            : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL p2                            : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL writeData                     : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL addResult                     : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL lluResult                     : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL asuResult                     : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL aauResult                     : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL bsuResult                     : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL outADR                        : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL outMux2                       : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL inPC                          : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL outPC                         : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL inc4PC                        : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL addrIn                        : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	--CSR Signals
	SIGNAL exceptionRaiseTemp            : STD_LOGIC;
	SIGNAL interruptRaiseTemp            : STD_LOGIC;
	SIGNAL CCmieField                    : STD_LOGIC;
	SIGNAL CCuieField                    : STD_LOGIC;
	SIGNAL mipCCLd                       : STD_LOGIC;
	SIGNAL instrMisalignedFlag           : STD_LOGIC;
	SIGNAL dividedByZeroFlag             : STD_LOGIC;
	SIGNAL mirrorUserBar                 : STD_LOGIC;
	SIGNAL mirrorUstatus                 : STD_LOGIC;
	SIGNAL mirrorUie                     : STD_LOGIC;
	SIGNAL mirrorUip                     : STD_LOGIC;
	SIGNAL curPRV                        : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL cntOutput                     : STD_LOGIC_VECTOR (2 DOWNTO 0);
	SIGNAL tempFlags                     : STD_LOGIC_VECTOR (5 DOWNTO 0);
	SIGNAL causeCodeTemp                 : STD_LOGIC_VECTOR (5 DOWNTO 0);
	SIGNAL preAddressRegBank             : STD_LOGIC_VECTOR (11 DOWNTO 0);
	SIGNAL mirrorAddress                 : STD_LOGIC_VECTOR (11 DOWNTO 0);
	SIGNAL addressRegBank                : STD_LOGIC_VECTOR (11 DOWNTO 0);
	SIGNAL outAddr                       : STD_LOGIC_VECTOR (11 DOWNTO 0);
	SIGNAL interruptSources              : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL CCmip                         : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL outCSR                        : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL inCSR                         : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL causeCode                     : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL trapValue                     : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL CCmie                         : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL interruptStartAddressDirect   : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL interruptStartAddressVectored : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL validAddressCSR               : STD_LOGIC;
	SIGNAL GI2D_rst_def					 : STD_LOGIC;
	SIGNAL DARU1_en						 : STD_LOGIC;
	SIGNAL ldPC						 	 : STD_LOGIC;
	SIGNAL E2M_ALU_res_curr				 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL D2E_outCSR_next				 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL F2GI_PC_plus4_next			 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL F2GI_PC_plus4_curr			 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL DARU1_en_def					 : STD_LOGIC;
	SIGNAL dataDARU1					 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL GI2D_PC_next					 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL GI2D_en_def					 : STD_LOGIC;
	SIGNAL completedDARU1_stored		 : STD_LOGIC;
	SIGNAL completedDARU1				 : STD_LOGIC;
	SIGNAL GI2D_instr_next			     : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL GI2D_PC_plus4_next			 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL GI2D_ex_flag_next			 : STD_LOGIC;
	SIGNAL GI2D_ex_flag_curr			 : STD_LOGIC;
	SIGNAL D2E_en_def					 : STD_LOGIC;
	SIGNAL D2E_rst_def					 : STD_LOGIC;
	SIGNAL GI2D_instr_curr			     : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL M2WB_ex_flag_curr			 : STD_LOGIC;
	SIGNAL WB_writeRegFile				 : STD_LOGIC;
	SIGNAL writeRegFile_in				 : STD_LOGIC;
	SIGNAL M2WB_lt_curr					 : STD_LOGIC;
	SIGNAL WB_setZeroOrOne				 : STD_LOGIC;
	SIGNAL setOne_in					 : STD_LOGIC;
	SIGNAL setZero_in					 : STD_LOGIC;
	SIGNAL M2WB_instr_curr				 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL writeAddressRegBank			 : STD_LOGIC_VECTOR (11 DOWNTO 0);
	SIGNAL WB_AddressRegBank			 : STD_LOGIC_VECTOR (11 DOWNTO 0);
	SIGNAL op1_p1_PC					 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL D2E_ALU_op2_next				 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL D2E_ALU_op1_next				 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL D2E_ex_flag_next				 : STD_LOGIC;
	SIGNAL D2E_ex_flag_curr				 : STD_LOGIC;
	SIGNAL E2M_en_def					 : STD_LOGIC;
	SIGNAL completedAAU_stored			 : STD_LOGIC;
	SIGNAL E_startMultiplyAAU			 : STD_LOGIC;
	SIGNAL startMultiplyAAU_def			 : STD_LOGIC;
	SIGNAL E_startDivideAAU				 : STD_LOGIC;
	SIGNAL startDivideAAU_def			 : STD_LOGIC;
	SIGNAL D2E_ALU_op1_curr				 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL D2E_ALU_op2_curr				 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL E_signedSigned				 : STD_LOGIC;
	SIGNAL E_signedUnsigned				 : STD_LOGIC;
	SIGNAL E_unsignedUnsigned			 : STD_LOGIC;
	SIGNAL E2M_rst_def					 : STD_LOGIC;
	SIGNAL E_selAAH						 : STD_LOGIC;
	SIGNAL E_selAAL						 : STD_LOGIC;
	SIGNAL E_selLogic					 : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL E_selShift					 : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL D2E_op2_curr					 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL E_cmp_selALUop2				 : STD_LOGIC;
	SIGNAL E_cmp_selop2					 : STD_LOGIC;
	SIGNAL CMP_op2						 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL D2E_op1_curr					 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL E_ComparedSignedUnsignedBar	 : STD_LOGIC;
	SIGNAL E2M_lt_next					 : STD_LOGIC;
	SIGNAL E2M_eq_next					 : STD_LOGIC;
	SIGNAL E2M_gt_next					 : STD_LOGIC;
	SIGNAL E_addSubBar					 : STD_LOGIC;
	SIGNAL E_pass						 : STD_LOGIC;
	SIGNAL E2M_ALU_res_next				 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL E_selBSU						 : STD_LOGIC;
	SIGNAL E_selLLU						 : STD_LOGIC;
	SIGNAL E_selASU						 : STD_LOGIC;
	SIGNAL E_selAAU						 : STD_LOGIC;
	SIGNAL E_selCSR						 : STD_LOGIC;
	SIGNAL D2E_outCSR_curr				 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL E2M_ex_flag_next				 : STD_LOGIC;
	SIGNAL M_opcode						 : STD_LOGIC_VECTOR (6 DOWNTO 0);
	SIGNAL M_func3						 : STD_LOGIC_VECTOR (2 DOWNTO 0);
	SIGNAL E2M_lt_curr					 : STD_LOGIC;
	SIGNAL E2M_eq_curr					 : STD_LOGIC;
	SIGNAL E2M_gt_curr					 : STD_LOGIC;
	SIGNAL DAWU_en						 : STD_LOGIC;
	SIGNAL M_nBytes						 : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL E2M_op2_curr					 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL M_checkMisalignedDAWU		 : STD_LOGIC;
	SIGNAL memAddrDAWU					 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL completedDAWU				 : STD_LOGIC;
	SIGNAL E2M_ex_flag_curr				 : STD_LOGIC;
	SIGNAL M_startDAWU					 : STD_LOGIC;
	SIGNAL M_startDARU					 : STD_LOGIC;
	SIGNAL M2WB_rst_def					 : STD_LOGIC;
	SIGNAL writeMem						 : STD_LOGIC;
	SIGNAL completedDAWU_stored			 : STD_LOGIC;
	SIGNAL DARU2_en						 : STD_LOGIC;
	SIGNAL completedDARU2				 : STD_LOGIC;
	SIGNAL completedDARU2_stored		 : STD_LOGIC;
	SIGNAL dataDARU2					 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL memAddrDARU2					 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL M2WB_MEM_res_next			 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL M2WB_MEM_res_curr			 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL M2WB_ALU_res_curr			 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL M2WB_PC_plus4_curr			 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL M_ldByteSigned				 : STD_LOGIC;
	SIGNAL M_ldHalfSigned				 : STD_LOGIC;
	SIGNAL M_load						 : STD_LOGIC;
	SIGNAL M2WB_hazard_flag_curr		 : STD_LOGIC;
	SIGNAL WB_selALU					 : STD_LOGIC;
	SIGNAL WB_selPC4					 : STD_LOGIC;
	SIGNAL WB_selMem					 : STD_LOGIC;
	SIGNAL WB_selCSRAddrFromInst		 : STD_LOGIC;
	SIGNAL mirrorUser					 : STD_LOGIC;
	SIGNAL WB_selP1CSR					 : STD_LOGIC;
	SIGNAL WB_selImmCSR					 : STD_LOGIC;
	SIGNAL WB_selReadWriteCSR			 : STD_LOGIC;
	SIGNAL WB_clrCSR					 : STD_LOGIC;
	SIGNAL WB_setCSR					 : STD_LOGIC;
	SIGNAL M2WB_op1_curr				 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL CSR_PC						 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL outCSR_write_val				 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL PC_from_WB					 : STD_LOGIC;
	SIGNAL E2M_PC_plus4_curr			 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL M2WB_outCSR_curr				 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL outCSR_reg					 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL M2WB_ecall_flag_curr			 : STD_LOGIC;
	SIGNAL M2WB_divided_by_zero_flag_curr: STD_LOGIC;
	SIGNAL M2WB_illegal_instruction_flag_curr: STD_LOGIC;
	SIGNAL M2WB_instr_misaligned_flag_curr: STD_LOGIC;
	SIGNAL M2WB_valid_curr				 : STD_LOGIC;
	SIGNAL D2E_instr_curr				 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL hazEX_first_operand			 : STD_LOGIC_VECTOR (4 DOWNTO 0);
	SIGNAL hazEX_second_operand			 : STD_LOGIC_VECTOR (4 DOWNTO 0);
	SIGNAL hazEX_result					 : STD_LOGIC_VECTOR (4 DOWNTO 0);
	SIGNAL hazEX_first_operand_en		 : STD_LOGIC;
	SIGNAL hazEX_second_operand_en		 : STD_LOGIC;
	SIGNAL hazEX_result_en				 : STD_LOGIC;
	SIGNAL hazEX_zero_first_operand		 : STD_LOGIC;
	SIGNAL hazEX_zero_second_operand	 : STD_LOGIC;
	SIGNAL hazEX_CSR_second_operand	 	 : STD_LOGIC_VECTOR (11 DOWNTO 0);
	SIGNAL hazEX_CSR_result			 	 : STD_LOGIC_VECTOR (11 DOWNTO 0);
	SIGNAL hazEX_CSR_second_operand_en	 : STD_LOGIC;
	SIGNAL hazEX_CSR_result_en			 : STD_LOGIC;
	SIGNAL hazEX_CSR_mirror				 : STD_LOGIC;
	SIGNAL D2E_inst_type_curr			 : STD_LOGIC_VECTOR (2 DOWNTO 0);
	SIGNAL M2WB_PC_curr					 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL E2M_instr_curr				 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL E2M_inst_type_curr			 : STD_LOGIC_VECTOR (2 DOWNTO 0);
	SIGNAL hazM_first_operand			 : STD_LOGIC_VECTOR (4 DOWNTO 0);
	SIGNAL hazM_second_operand			 : STD_LOGIC_VECTOR (4 DOWNTO 0);
	SIGNAL hazM_result					 : STD_LOGIC_VECTOR (4 DOWNTO 0);
	SIGNAL hazM_first_operand_en		 : STD_LOGIC;
	SIGNAL hazM_second_operand_en		 : STD_LOGIC;
	SIGNAL hazM_result_en				 : STD_LOGIC;
	SIGNAL hazM_zero_first_operand		 : STD_LOGIC;
	SIGNAL hazM_zero_second_operand	 	 : STD_LOGIC;
	SIGNAL hazM_CSR_second_operand	 	 : STD_LOGIC_VECTOR (11 DOWNTO 0);
	SIGNAL hazM_CSR_result			 	 : STD_LOGIC_VECTOR (11 DOWNTO 0);
	SIGNAL hazM_CSR_second_operand_en	 : STD_LOGIC;
	SIGNAL hazM_CSR_result_en			 : STD_LOGIC;
	SIGNAL hazM_CSR_mirror				 : STD_LOGIC;
	SIGNAL E2M_hazard_flag_next			 : STD_LOGIC;
	SIGNAL E2M_hazard_flag_curr			 : STD_LOGIC;
	SIGNAL M2WB_hazard_flag_next		 : STD_LOGIC;
	SIGNAL GI2D_valid_curr				 : STD_LOGIC;
	SIGNAL GI2D_PC_curr					 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL D2E_PC_curr					 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL GI2D_PC_plus4_curr			 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL D2E_PC_plus4_curr			 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL GI2D_instr_misaligned_flag_curr: STD_LOGIC;
	SIGNAL D2E_valid_curr				 : STD_LOGIC;
	SIGNAL D2E_instr_misaligned_flag_curr: STD_LOGIC;
	SIGNAL D2E_illegal_instruction_flag_curr: STD_LOGIC;
	SIGNAL D2E_ecall_flag_curr			 : STD_LOGIC;
	SIGNAL D2E_ctrl_word_curr			 : STD_LOGIC_VECTOR (63 DOWNTO 0);
	SIGNAL D2E_ctrl_word_next			 : STD_LOGIC_VECTOR (63 DOWNTO 0);
	SIGNAL E2M_valid_curr				 : STD_LOGIC;
	SIGNAL E2M_instr_misaligned_flag_curr: STD_LOGIC;
	SIGNAL E2M_outCSR_curr				 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL E2M_op1_curr					 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL E2M_PC_curr					 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL E2M_illegal_instruction_flag_curr: STD_LOGIC;
	SIGNAL E2M_ecall_flag_curr			 : STD_LOGIC;
	SIGNAL E2M_ctrl_word_curr			 : STD_LOGIC_VECTOR (63 DOWNTO 0);
	SIGNAL E2M_divided_by_zero_flag_curr : STD_LOGIC;
	SIGNAL M2WB_inst_type_curr			 : STD_LOGIC_VECTOR (2 DOWNTO 0);
	SIGNAL M2WB_ctrl_word_curr			 : STD_LOGIC_VECTOR (63 DOWNTO 0);
	SIGNAL M2WB_gt_curr					 : STD_LOGIC;
	SIGNAL M2WB_eq_curr					 : STD_LOGIC;
	SIGNAL F2GI_valid_curr				 : STD_LOGIC;
	SIGNAL EH_state_next				 : ex_pre_handling_state;
	SIGNAL EH_state_curr				 : ex_pre_handling_state;
	SIGNAL func3						 : STD_LOGIC_VECTOR (2 DOWNTO 0);
	SIGNAL func7						 : STD_LOGIC_VECTOR (6 DOWNTO 0);
	SIGNAL func12						 : STD_LOGIC_VECTOR (11 DOWNTO 0);
	SIGNAL opcode 						 : STD_LOGIC_VECTOR (6 DOWNTO 0);
	SIGNAL D2E_ALU_op1_next_pre_bypass	 : STD_LOGIC_VECTOR (31 DOWNTO 0);
	SIGNAL D2E_ALU_op2_next_pre_bypass	 : STD_LOGIC_VECTOR (31 DOWNTO 0);
	SIGNAL bypass_first_operand			 : STD_LOGIC_VECTOR (4 DOWNTO 0);
	SIGNAL bypass_second_operand		 : STD_LOGIC_VECTOR (4 DOWNTO 0);
	SIGNAL bypass_CSR_second_operand	 : STD_LOGIC_VECTOR (11 DOWNTO 0);
	SIGNAL bypass_result				 : STD_LOGIC_VECTOR (4 DOWNTO 0);
	SIGNAL bypass_CSR_result			 : STD_LOGIC_VECTOR (11 DOWNTO 0);
	SIGNAL bypass_first_operand_en		 : STD_LOGIC;
	SIGNAL bypass_second_operand_en		 : STD_LOGIC;
	SIGNAL bypass_CSR_second_operand_en	 : STD_LOGIC;
	SIGNAL bypass_result_en				 : STD_LOGIC;
	SIGNAL bypass_CSR_result_en			 : STD_LOGIC;
	SIGNAL bypass_CSR_mirror 			 : STD_LOGIC;
	SIGNAL bypass_first					 : STD_LOGIC;
	SIGNAL bypass_second				 : STD_LOGIC;
	SIGNAL bypass_second_CSR			 : STD_LOGIC;
	SIGNAL bypass_zero_first_operand	 : STD_LOGIC;
	SIGNAL bypass_zero_second_operand	 : STD_LOGIC;
	SIGNAL D2E_op1_next					 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL D2E_op2_next					 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL D2E_validAccessCSR_curr		 : STD_LOGIC;
	SIGNAL E2M_validAccessCSR_curr		 : STD_LOGIC;
	SIGNAL M2WB_validAccessCSR_curr		 : STD_LOGIC;
	SIGNAL D2E_outCSR_next_pre_bypass	 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);

	-- unused control signals
	SIGNAL E_writeRegFile			  	 : STD_LOGIC;
	SIGNAL E_setZeroOrOne				 : STD_LOGIC;
	SIGNAL E_selPC						 : STD_LOGIC;
	SIGNAL E_selJL						 : STD_LOGIC;
	SIGNAL E_selP1						 : STD_LOGIC;
	SIGNAL E_selP2						 : STD_LOGIC;
	SIGNAL E_selImm						 : STD_LOGIC;
	SIGNAL E_ldByteSigned				 : STD_LOGIC;
	SIGNAL E_ldHalfSigned				 : STD_LOGIC;
	SIGNAL E_load						 : STD_LOGIC;
	SIGNAL E_selAuipc					 : STD_LOGIC;
	SIGNAL E_muxCode					 : STD_LOGIC_VECTOR (11 DOWNTO 0);				
	SIGNAL E_startDAWU					 : STD_LOGIC;				
	SIGNAL E_startDARU					 : STD_LOGIC;				
	SIGNAL E_nBytes						 : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL E_writeRB_inst				 : STD_LOGIC;				
	SIGNAL E_checkMisalignedDAWU		 : STD_LOGIC;
	SIGNAL E_selCSRAddrFromInst			 : STD_LOGIC;
	SIGNAL E_forced_RB_read				 : STD_LOGIC;
	SIGNAL E_inst_type					 : STD_LOGIC_VECTOR (2 DOWNTO 0);
	SIGNAL E_ret_from_epc				 : STD_LOGIC;
	SIGNAL E_selALU						 : STD_LOGIC;				
	SIGNAL E_selPC4						 : STD_LOGIC;				
	SIGNAL E_selMem						 : STD_LOGIC;				
	SIGNAL E_isCSRInstruction			 : STD_LOGIC;
	SIGNAL E_func3						 : STD_LOGIC_VECTOR (2 DOWNTO 0);				
	SIGNAL E_func7						 : STD_LOGIC_VECTOR (6 DOWNTO 0);				
	SIGNAL E_func12						 : STD_LOGIC_VECTOR (11 DOWNTO 0);
	SIGNAL E_opcode						 : STD_LOGIC_VECTOR (6 DOWNTO 0);

	SIGNAL M_writeRegFile			  	 : STD_LOGIC;
	SIGNAL M_setZeroOrOne				 : STD_LOGIC;
	SIGNAL M_ComparedSignedUnsignedBar	 : STD_LOGIC;
	SIGNAL M_selPC						 : STD_LOGIC;
	SIGNAL M_selJL						 : STD_LOGIC;
	SIGNAL M_selBSU						 : STD_LOGIC;
	SIGNAL M_selLLU						 : STD_LOGIC;
	SIGNAL M_selASU						 : STD_LOGIC;
	SIGNAL M_selAAU						 : STD_LOGIC;
	SIGNAL M_selP1						 : STD_LOGIC;
	SIGNAL M_selP2						 : STD_LOGIC;
	SIGNAL M_selImm						 : STD_LOGIC;
	SIGNAL M_selShift					 : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL M_addSubBar					 : STD_LOGIC;
	SIGNAL M_pass						 : STD_LOGIC;
	SIGNAL M_selAuipc					 : STD_LOGIC;
	SIGNAL M_muxCode					 : STD_LOGIC_VECTOR (11 DOWNTO 0);				
	SIGNAL M_selLogic					 : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL M_startMultiplyAAU			 : STD_LOGIC;
	SIGNAL M_startDivideAAU				 : STD_LOGIC;
	SIGNAL M_signedSigned				 : STD_LOGIC;
	SIGNAL M_signedUnsigned				 : STD_LOGIC;
	SIGNAL M_unsignedUnsigned			 : STD_LOGIC;
	SIGNAL M_selAAL						 : STD_LOGIC;
	SIGNAL M_selAAH						 : STD_LOGIC;
	SIGNAL M_selCSR						 : STD_LOGIC;
	SIGNAL M_writeRB_inst				 : STD_LOGIC;
	SIGNAL M_selCSRAddrFromInst			 : STD_LOGIC;
	SIGNAL M_forced_RB_read				 : STD_LOGIC;
	SIGNAL M_inst_type					 : STD_LOGIC_VECTOR (2 DOWNTO 0);
	SIGNAL M_ret_from_epc				 : STD_LOGIC;
	SIGNAL M_selALU						 : STD_LOGIC;				
	SIGNAL M_selPC4						 : STD_LOGIC;				
	SIGNAL M_selMem						 : STD_LOGIC;	
	SIGNAL M_cmp_selALUop2				 : STD_LOGIC;			
	SIGNAL M_cmp_selop2					 : STD_LOGIC;			
	SIGNAL M_isCSRInstruction			 : STD_LOGIC;			
	SIGNAL M_func7						 : STD_LOGIC_VECTOR (6 DOWNTO 0);				
	SIGNAL M_func12						 : STD_LOGIC_VECTOR (11 DOWNTO 0);

	SIGNAL WB_ComparedSignedUnsignedBar	 : STD_LOGIC;
	SIGNAL WB_selPC						 : STD_LOGIC;
	SIGNAL WB_selJL						 : STD_LOGIC;
	SIGNAL WB_selBSU					 : STD_LOGIC;
	SIGNAL WB_selLLU					 : STD_LOGIC;
	SIGNAL WB_selASU					 : STD_LOGIC;
	SIGNAL WB_selAAU					 : STD_LOGIC;
	SIGNAL WB_selP1						 : STD_LOGIC;
	SIGNAL WB_selP2						 : STD_LOGIC;
	SIGNAL WB_selImm					 : STD_LOGIC;
	SIGNAL WB_ldByteSigned				 : STD_LOGIC;
	SIGNAL WB_ldHalfSigned				 : STD_LOGIC;
	SIGNAL WB_load						 : STD_LOGIC;
	SIGNAL WB_selShift					 : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL WB_addSubBar					 : STD_LOGIC;
	SIGNAL WB_pass						 : STD_LOGIC;
	SIGNAL WB_selAuipc					 : STD_LOGIC;
	SIGNAL WB_muxCode					 : STD_LOGIC_VECTOR (11 DOWNTO 0);				
	SIGNAL WB_selLogic					 : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL WB_startDAWU					 : STD_LOGIC;				
	SIGNAL WB_startDARU					 : STD_LOGIC;
	SIGNAL WB_startMultiplyAAU			 : STD_LOGIC;
	SIGNAL WB_startDivideAAU			 : STD_LOGIC;
	SIGNAL WB_signedSigned				 : STD_LOGIC;
	SIGNAL WB_signedUnsigned			 : STD_LOGIC;
	SIGNAL WB_unsignedUnsigned			 : STD_LOGIC;
	SIGNAL WB_selAAL					 : STD_LOGIC;
	SIGNAL WB_selAAH					 : STD_LOGIC;
	SIGNAL WB_nBytes					 : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL WB_selCSR					 : STD_LOGIC;
	SIGNAL WB_writeRB_inst				 : STD_LOGIC;
	SIGNAL WB_checkMisalignedDAWU		 : STD_LOGIC;
	SIGNAL WB_forced_RB_read			 : STD_LOGIC;
	SIGNAL WB_inst_type					 : STD_LOGIC_VECTOR (2 DOWNTO 0);
	SIGNAL WB_cmp_selALUop2				 : STD_LOGIC;			
	SIGNAL WB_cmp_selop2				 : STD_LOGIC;					
	SIGNAL WB_func7						 : STD_LOGIC_VECTOR (6 DOWNTO 0);				
	SIGNAL WB_func12					 : STD_LOGIC_VECTOR (11 DOWNTO 0);			
	SIGNAL WB_opcode					 : STD_LOGIC_VECTOR (6 DOWNTO 0);			

BEGIN

	-- NEW DATAPATH
	
	-- FETCH STAGE: FOR NOW WITHOUT BRANCH PREDICTION -------------------------------------------------------
	-- NOTE: FETCH AND GET_INSTR ARE JOINED (THE PIPELINE REGISTER FOR PC IS INSIDE DARU) AND THE ENABLE FOR 
	-- THIS REGISTER IS ON ONLY WHEN DARU IS NOT OPERATING OR IT IS TERMINATING OPERATION IN THE CURRENT CYCLE

	-- ldPC is given by the pipeline enabled signal, but if there are mispredictions on branches you have to clear F2GI
	-- GI2D_rst is raised when there are mispredictions, exceptions or interrupts
	ldPC <= (DARU1_en AND NOT(GI2D_rst_def)) OR GI2D_rst;

	-- PC register
	regPC : ENTITY WORK.aftab_register
		GENERIC
		MAP(len => len)
		PORT
		MAP(
		clk    => clk,
		rst    => rst,
		zero   => '0',
		load   => ldPC, -- when you start processing a new instruction you can increase the PC value, but you have to do it when there is a misprediction too
		inReg  => inPC,
		outReg => outPC);

	-- PC+4 computation
	i4PC : ENTITY WORK.aftab_adder
		GENERIC
		MAP(len => len)
		PORT
		MAP(
		Cin       => '0',
		A         => outPC,
		B => (31 DOWNTO 3 => '0') & "100",
		addResult => inc4PC,
		carryOut  => OPEN);

	-- selection of the new PC value, depending on which instruction is being executed
	-- the selection signals are driven by the CU depending on which conditions are verified (pay attention to priorities)
	-- if the instruction in memory is a taken branch --> E2M_ALU_res_curr
	-- if there is no branch nor interrupt/exception --> inc4PC, increment the PC normally
	-- if there is an exception/return from exception being handled after write-back --> outCSR
	-- if there is an interrupt being handled (base) --> interruptStartAddressDirect
	-- if // (vectored) --> interruptStartAddressVectored
	-- TODO: MAKE SURE THAT EVERYTHING WORKS CORRECTLY, SO THAT IT IS POSSIBLE TO FORCE A VALUE IN THE PC.
	inPC <= E2M_ALU_res_curr WHEN branch_taken = '1' ELSE -- branch_taken from BPU in memory stage
		D2E_outCSR_next WHEN selMepc_CSR = '1' ELSE -- driven from exception handling phase
		interruptStartAddressDirect WHEN selInterruptAddressDirect = '1' ELSE -- driven from interrupt handling phase
		interruptStartAddressVectored WHEN selInterruptAddressVectored = '1' ELSE -- as before 
		inc4PC;

	-- the PC value to save in MEPC when performing ecall or branches, to be passed through the pipeline
	F2GI_PC_plus4_next <= inc4PC;

	-- GET_INSTR STAGE: -----------------------------------------------------------------------------------

	daru1 : ENTITY WORK.aftab_daru
		PORT
		MAP(
		clk                 => clk,
		rst                 => rst,
		sync_rst 			=> GI2D_rst, -- if there is a misprediction you need to reset the daru state synchronously
		startDARU           => DARU1_en_def,
		nBytes              => "11", -- 4 since it is a 32 bits word (3 considering also the byte fetched on the 0)
		addrIn              => outPC, -- you retrieve the instruction corresponding to the PC value passed from fetch 
		memData             => memDataOut1, -- the data retrieved from memory at each read operation (async read)
		memReady            => memReady1, -- core, stall if the memory is not ready
		dataInstrBar        => '0', -- data or instruction bar (set to '0' because this DARU is used exclusively for instructions)
		checkMisalignedDARU => '1', -- raised if the DARU has to throw an exception when the addresses are misaligned, here it should be 1 
		instrMisalignedFlag => instrMisalignedFlag, -- the exception has to be thrown (set an exception flag which has to propagate through the pipeline until it reaches WB)
													-- if the exception is raised then all the stages before should be blocked
		loadMisalignedFlag  => OPEN,
		completeDARU        => completedDARU1, -- controller, to notify that we have to move the instruction to the next stage
		dataOut             => dataDARU1, -- the 32 bits word (instruction), kept available until the cycle after raising startDARU
		addrOut             => memAddr1, -- the address being read
		readAddrOut			=> GI2D_PC_next, -- the address of the instruction being read
		readMem             => readMemDARU1, -- the control signal which enables memory read
		bytesToRead			=> bytesToReadDARU1); -- the bytes to be read per memory access (0 if 1, 1 if 2): if you read 1 byte then the MSB will be zeroed --> TODO: TO BE SENT TO MEMORY

	-- condition under which a new fetch operation has to be started: either there is an instruction leaving or there is nothing being read in the DARU
	-- the second operand of the OR represents the case in which there is nothing being executed or waiting to be forwarded in decode
	DARU1_en <= (GI2D_en_def AND NOT(GI2D_rst_def)) OR (NOT(readMemDARU1) AND NOT(completedDARU1_def)); -- def instead of stored

	-- a flip flop to hold the completeDARU1 flag, otherwise it would be lost if there was a stall in decode
	complete_FF: process(clk,rst) BEGIN
		-- async reset for the whole pipeline
		IF(rst='1') THEN
			completedDARU1_stored <= '0';
		ELSIF(rising_edge(clk)) THEN
			-- reset condition (when instruction passes to decode or the pipeline is reset)
			IF((GI2D_en_def='1' AND GI2D_rst_def='0') OR GI2D_rst='1') THEN
				completedDARU1_stored <= '0';
			-- set the register to 1 when an instruction has been fetched
			ELSIF(completedDARU1='1') THEN
				completedDARU1_stored <= '1';
			END IF;
		END IF;
	END PROCESS;

	-- completedDARU1 signal to be sent to CU
	completedDARU1_def <= completedDARU1 OR completedDARU1_stored;

	-- THE CONTROLLER HAS TO GENERATE THE GI2D_en AS (completedDARU1 or completedDARU1_stored) and not(hazards) and unit_available
	-- INSTRUCTION REGISTER (inside the pipeline)
	GI2D_instr_next <= dataDARU1;
	-- PC_plus4 passes through the pipeline
	GI2D_PC_plus4_next <= F2GI_PC_plus4_curr;
	-- exception raised flag: if it reaches the WB then the exception has to be handled (otherwise it was raised by an instruction which shouldn't have been executed)
	GI2D_ex_flag_next <= instrMisalignedFlag;
	-- the definitive version of the GI2D_rst:
	-- you reset either because there is a pipelined forced reset or because the register is not enabled while the following one is
	GI2D_rst_def <= GI2D_rst OR (NOT(GI2D_en_def) AND D2E_en_def);

	----------------------------------------------------------------------------------------------------------
	-- DECODE STAGE: -----------------------------------------------------------------------------------------
	----------------------------------------------------------------------------------------------------------

	-- send the instruction to CU to produce the control word
	IR <= GI2D_instr_curr;
	inst <= GI2D_instr_curr;

	-- ISSEU: to extend the immediate retrieved from the instruction
	immSelSignEx : ENTITY WORK.aftab_isseu
		PORT
		MAP(
		IR7     => GI2D_instr_curr (7),
		IR20    => GI2D_instr_curr (20),
		IR31    => GI2D_instr_curr (31),
		IR11_8  => GI2D_instr_curr (11 DOWNTO 8),
		IR19_12 => GI2D_instr_curr (19 DOWNTO 12),
		IR24_21 => GI2D_instr_curr (24 DOWNTO 21),
		IR30_25 => GI2D_instr_curr (30 DOWNTO 25),
		selI    => muxCode (0),
		selS    => muxCode (1),
		selBUJ  => muxCode (2),
		selIJ   => muxCode (3),
		selSB   => muxCode (4),
		selU    => muxCode (5),
		selISBJ => muxCode (6),
		selIS   => muxCode (7),
		selB    => muxCode (8),
		selJ    => muxCode (9),
		selISB  => muxCode (10),
		selUJ   => muxCode (11),
		Imm     => immediate);

	-- writeRegFile input to the RF
	-- TODO: ADJUST IT TO KEEP INTO ACCOUNT A WRITE OPERATION STARTED BY CU DURING EXCEPTION HANDLING
	writeRegFile_in <= WB_writeRegFile AND NOT(M2WB_ex_flag_curr);

	-- setOne input to the RF
	setOne_in <= NOT(M2WB_lt_curr) AND WB_setZeroOrOne;

	-- setZero input to the RF 
	setZero_in <= M2WB_lt_curr AND WB_setZeroOrOne;

	-- register file: to fetch the operands and update the destination (during write-back)
	registerFile : ENTITY WORK.aftab_register_file
		GENERIC
		MAP(len => len)
		PORT MAP
		(
		clk          => clk,
		rst          => rst,
		setZero      => setZero_in, -- for slt,slti,sltu (in write-back)
		setOne       => setOne_in, -- for slt,slti,sltu (in write-back)
		rs1          => GI2D_instr_curr (19 DOWNTO 15),
		rs2          => GI2D_instr_curr (24 DOWNTO 20),
		rd           => M2WB_instr_curr (11 DOWNTO 7),
		writeData    => writeData, -- output of a mux which selects between memory out and ALU out
		writeRegFile => writeRegFile_in, -- the control signal for the RF related to the ins currently in write-back
		p1           => p1, -- output1
		p2           => p2); -- output2

	-- CSR bank: accessed during decode and writeback (and exception management)
	register_bank : ENTITY WORK.aftab_register_bank
		GENERIC
		MAP (len => len)
		PORT
		MAP(
		clk              => clk,
		rst              => rst,
		writeRegBank     => writeRegBank, -- 1 if we have to overwrite the register in the regBank (in write-back or during exception handling)
		addressRegBank   => addressRegBank, -- address of the register to be accessed
		writeAddressRegBank => writeAddressRegBank, -- address of the register to be written
		inputRegBank     => inCSR, -- input to be written (driven from CSRISL, which contains the logic needed to perform operations over the CSRs)
		loadMieReg       => loadMieReg, -- if a new value has to be loaded in the CCMIEreg (risen by CU when ldMieReg is high and the access is valid)
		loadMieUieField  => loadMieUieField, -- if you have to overwrite the enable bit for M/U mode (the CC version), raised as before
		outRegBank       => D2E_outCSR_next_pre_bypass, -- the value read from the register bank (the read is performed SYNCHRONOUSLY)
		mirrorUstatus    => mirrorUstatus, -- if you update MSTATUS you have to update USTATUS too, because this last register is simply a mirror of MSTATUS with limited accessibility
		mirrorUie        => mirrorUie,
		mirrorUip        => mirrorUip,
		mirror           => mirror, -- to be sent to CU to determine if the state afterward has to be used to update the mirror register
		ldMieReg         => ldMieReg,
		ldMieUieField    => ldMieUieField,
		outMieFieldCCreg => CCmieField, -- global enable bit for interrupts in machine mode
		outUieFieldCCreg => CCuieField, -- global enable bit for interrupts in user mode
		outMieCCreg      => CCmie -- interrupt enables word
		);

	-- if CSR register is not implemented the access cannot be considered valid (Illegal Instr).
	-- modified Luca
	-- ONLY THE DECODE ACCESS CAN THROW AN ILLEGAL_INSTR EXCEPTION, THE WRITE-BACK ACCESS IS COMPLETELY HANDLED BY CU
	-- the instructions which have been implemented overwrite the same CSR used as operand, so we don't need two CSRACTRL
	csr_address_ctrl: ENTITY WORK.aftab_csr_address_ctrl
	PORT MAP
	(
		addressRegBank => addressRegBank,
		validAddressCSR => validAddressCSR
	);

	-- if the access is invalid then the CU will throw an illegalInstructionException
	--validAccessCSR <= '1' WHEN (curPRV >= addressRegBank(9 DOWNTO 8)) ELSE '0'; -- changed Luca
	validAccessCSR <= '1' WHEN ( curPRV >= addressRegBank(9 DOWNTO 8) AND validAddressCSR = '1') ELSE '0';

	-- DONE: CU should raise the writeRegBank only if the register that we are trying to write (the same one we are trying to read) is actually writeable
	readOnlyCSR    <= '1' WHEN (writeAddressRegBank(11 DOWNTO 10) = "11") ELSE '0';

	-- address of the register to be read in decode or write-back (either the one from the instruction or the one driven by the CSRC-CSRAD chain)
	addressRegBank <= WB_AddressRegBank WHEN forced_RB_read='1' ELSE -- we force a read operation over a generic register during exception/interrupt handling
					  GI2D_instr_curr(31 DOWNTO 20);

	-- mux to select as a first operand the value from RF or the PC (this last one is used only by AUIPC)
	mux6 : ENTITY WORK.aftab_multiplexer
		GENERIC
		MAP(len => len)
		PORT
		MAP(
		a  => D2E_op1_next,
		b  => GI2D_PC_curr,
		s0 => selP1,
		s1 => selAuipc,
		w  => op1_p1_PC);
	
	-- mux to select either the immediate or the value from the RF as a second operand
	mux5 : ENTITY WORK.aftab_multiplexer
		GENERIC
		MAP(len => len)
		PORT
		MAP(
		a  => D2E_op2_next,
		b  => immediate,
		s0 => selP2,
		s1 => selImm,
		w  => D2E_ALU_op2_next);

	-- multiplexer which selects the value of the register from the RF in case we have to execute a JALR (rs1 in JALR encoding)
	mux2 : ENTITY WORK.aftab_multiplexer
		GENERIC
		MAP(len => len)
		PORT
		MAP(
		a  => op1_p1_PC,
		b  => GI2D_PC_curr,
		s0 => selJL,
		s1 => selPC,
		w  => D2E_ALU_op1_next);

	-- FIXME: BYPASS LOGIC: used to select directly the value written in write-back by the instruction which is 
	-- responsible for the hazard. This allows us to forward in EX the instruction currently in DEC while
	-- the instruction that produces its operand is completing write-back.

	bypass_operands_result_retrieval: PROCESS(inst_type, GI2D_instr_curr, M2WB_instr_curr, M2WB_inst_type_curr) BEGIN

		bypass_first_operand <= (OTHERS => '0');
		bypass_second_operand <= (OTHERS => '0');
		bypass_result <= (OTHERS => '0');
		bypass_first_operand_en <= '0';
		bypass_second_operand_en <= '0';
		bypass_result_en <= '0';
		bypass_zero_first_operand <= '0';
		bypass_zero_second_operand <= '0';
		bypass_CSR_second_operand_en <= '0';
		bypass_CSR_result_en <= '0';
		bypass_CSR_mirror <= '0';

		-- operands selection based on instruction type
		IF (inst_type = R_type) THEN -- arithmetic instructions
			bypass_first_operand <= GI2D_instr_curr(19 DOWNTO 15);
			bypass_first_operand_en <= '1';
			bypass_second_operand <= GI2D_instr_curr(24 DOWNTO 20);
			bypass_second_operand_en <= '1';
		ELSIF (inst_type = I_type) THEN -- arithmetic instruction with an immediate as an operand
			-- 1110011(2) = 73h
			IF (GI2D_instr_curr(6 DOWNTO 0) = "1110011" AND GI2D_instr_curr(14 DOWNTO 12) /= "000") THEN
				-- this covers all CSR instructions except for ecall which can't raise hazards since it 
				-- performs computation and read registers during write-back
				-- the check is implemented by reading the opcode and the funct3 field
				bypass_first_operand <= GI2D_instr_curr(19 DOWNTO 15);
				bypass_first_operand_en <= '1';
				bypass_CSR_second_operand_en <= '1';
				bypass_CSR_second_operand <= GI2D_instr_curr(31 DOWNTO 20);
			ELSIF (GI2D_instr_curr(6 DOWNTO 0) /= "1110011") THEN
				-- this covers all the other I instructions (except for ecall)
				bypass_first_operand <= GI2D_instr_curr(19 DOWNTO 15);
				bypass_first_operand_en <= '1';
				-- normal I instructions use the extended immediate as the second operand
				bypass_second_operand_en <= '0';
			END IF;
		ELSIF (inst_type = S_type) THEN -- store instructions
			-- this covers everything but mret and uret
			IF (GI2D_instr_curr(6 DOWNTO 0) /= "1110011") THEN
				bypass_first_operand <= GI2D_instr_curr(19 DOWNTO 15);
				bypass_first_operand_en <= '1';
				bypass_second_operand <= GI2D_instr_curr(24 DOWNTO 20);
				bypass_second_operand_en <= '1';
			END IF;
		ELSIF (inst_type = B_type) THEN -- conditional branches
			bypass_first_operand <= GI2D_instr_curr(19 DOWNTO 15);
			bypass_first_operand_en <= '1';
			bypass_second_operand <= GI2D_instr_curr(24 DOWNTO 20);
			bypass_second_operand_en <= '1';
		ELSIF (inst_type = U_type) THEN -- auipc, lui (the load the upper bits in a register)
			bypass_first_operand_en <= '0';
			bypass_second_operand_en <= '0';
		ELSIF (inst_type = J_type) THEN -- unconditional branches
			bypass_first_operand_en <= '0';
			bypass_second_operand_en <= '0';
		END IF;

		-- result selection based on instruction type
		IF (M2WB_inst_type_curr = R_type) THEN
			bypass_result <= M2WB_instr_curr(11 DOWNTO 7);
			bypass_result_en <= '1';
		ELSIF (M2WB_inst_type_curr = I_type) THEN
			-- this covers all CSR instructions but ecall
			-- all CSR instructions share with the classic I instructions the position of the result, but they update
			-- the value of the CSR too
			IF (M2WB_instr_curr(6 DOWNTO 0) = "1110011" and M2WB_instr_curr(14 DOWNTO 12) /= "000") THEN
				-- consider instructions which update registers that are mirrored (yoou should raise an hazard also if you are accessing USTATUS and MSTATUS is being modified by an instruction which has already been decoded)
				IF (M2WB_instr_curr(27 DOWNTO 20) = x"00" OR M2WB_instr_curr(27 DOWNTO 20) = x"04" OR M2WB_instr_curr(27 DOWNTO 20) = x"44") THEN
					bypass_CSR_mirror <= '1'; 
				END IF;
				bypass_CSR_result <= M2WB_instr_curr(31 DOWNTO 20);
				bypass_CSR_result_en <= '1';
			END IF;
			bypass_result <= M2WB_instr_curr(11 DOWNTO 7);
			bypass_result_en <= '1';
		ELSIF (M2WB_inst_type_curr = S_type) THEN
			bypass_result_en <= '0';
		ELSIF (M2WB_inst_type_curr = B_type) THEN
			bypass_result_en <= '0';
		ELSIF (M2WB_inst_type_curr = U_type) THEN
			bypass_result <= M2WB_instr_curr(11 DOWNTO 7);
			bypass_result_en <= '1';
		ELSIF (M2WB_inst_type_curr = J_type) THEN -- both jal and jalr store the return address in a register
			bypass_result <= M2WB_instr_curr(11 DOWNTO 7);
			bypass_result_en <= '1';
		END IF;

		-- register zero detection
		IF (GI2D_instr_curr(19 DOWNTO 15) = "00000") THEN
			bypass_zero_first_operand <= '1';
		END IF;
		IF (GI2D_instr_curr(24 DOWNTO 20) = "00000") THEN
			bypass_zero_second_operand <= '1';
		END IF;
	END PROCESS;

	-- the process for handling the bypass involving the operands and the results determined before 
	bypass_handling: PROCESS(bypass_first_operand, bypass_first_operand_en, bypass_second_operand, bypass_second_operand_en, bypass_result, 
	bypass_result_en, bypass_zero_first_operand, bypass_zero_second_operand, bypass_CSR_second_operand, bypass_CSR_second_operand_en,
	bypass_CSR_result, bypass_CSR_result_en) BEGIN
		bypass_first <= '0';
		bypass_second <= '0';
		bypass_second_CSR <= '0';
		-- comparison between first operand and result
		IF (bypass_first_operand = bypass_result AND bypass_first_operand_en = '1' AND bypass_result_en = '1' AND bypass_zero_first_operand = '0') THEN
			bypass_first <= '1';
		-- comparison between second operand and result (considering also a comparison between a CSR result and a CSR second operand)
		-- remember not to bypass if you are reading a machine register and you are updating the user one
		END IF;
		IF (bypass_second_operand = bypass_result AND bypass_second_operand_en = '1' AND bypass_result_en = '1' AND bypass_zero_second_operand = '0') THEN
			bypass_second <= '1';
		END IF;
		IF (((bypass_CSR_mirror = '0' AND bypass_CSR_second_operand = bypass_CSR_result) OR (bypass_CSR_mirror = '1' AND bypass_CSR_second_operand(7 DOWNTO 0) = bypass_CSR_result (7 DOWNTO 0) AND NOT(bypass_CSR_second_operand(9 DOWNTO 8) = "11" AND bypass_CSR_result(9 DOWNTO 8) = "00")))
			AND bypass_CSR_second_operand_en = '1' AND bypass_CSR_result_en = '1') THEN
			bypass_second_CSR <= '1';
		END IF;
	END PROCESS;

	-- bypass multiplexers
	D2E_op1_next <= p1 WHEN bypass_first = '0' ELSE writeData;
	D2E_op2_next <= p2 WHEN bypass_second = '0' ELSE writeData;
	D2E_outCSR_next <= D2E_outCSR_next_pre_bypass WHEN bypass_second_CSR = '0' ELSE inCSR;

	-- compute the exception flag for the D2E
	D2E_ex_flag_next <= GI2D_ex_flag_curr OR ecallFlag OR illegalInstrFlag;

	-- the complete D2E_en
	D2E_rst_def <= D2E_rst OR (NOT(D2E_en_def) AND E2M_en_def);


	-- EXECUTE STAGE ------------------------------------------------------------------------------------------

	-- startMultiplyAAU is reset when a multiplication is stalling in the EX stage
	startMultiplyAAU_def <= E_startMultiplyAAU AND (NOT(completedAAU) AND NOT(completedAAU_stored));

	-- the same applies for divisions
	startDivideAAU_def <= E_startDivideAAU AND (NOT(completedAAU) AND NOT(completedAAU_stored));
	
	aau : ENTITY WORK.aftab_aau
		GENERIC
		MAP (len => len)
		PORT
		MAP(
		clk               => clk,
		rst               => rst,
		sync_rst 		  => D2E_rst_def, -- reset the AAU state synchronously when a misprediction/exception/interrupt occurs or when the instruction leaves EXE
		ain               => D2E_ALU_op1_curr,
		bin               => D2E_ALU_op1_curr,
		startMultAAU      => startMultiplyAAU_def, -- control signal to start the mul (can be kept high during the whole multiplication, must be reset when completedAAU is asserted)
		startDivideAAU    => startDivideAAU_def, -- the same for the division
		SignedSigned      => E_signedSigned,
		SignedUnsigned    => E_signedUnsigned,
		UnsignedUnsigned  => E_unsignedUnsigned,
		resAAU1           => resAAH,
		resAAU2           => resAAL,
		dividedByZeroFlag => dividedByZeroFlag, -- the division is never started, so the flag remains on while the instruction is kept in EX
		completeAAU       => completedAAU); -- to be sent to the CU to build the D2E_en

	-- flag that is set when the operation in EX relies on AAU for computation
	is_AAU_used <= E_startMultiplyAAU OR E_startDivideAAU;
	
	-- FLAG TO PRESERVE completedAAU IN CASE THE PIPELINE IS STALLED (THE RESULT IS NOT LOST)
	completedAAU_register: PROCESS (clk, rst) BEGIN
		IF (rst = '1') THEN
			completedAAU_stored <= '0';
		ELSIF (rising_edge(clk)) THEN
			-- reset if pipeline is sync reset or if the instruction leaves execute
			IF (D2E_rst = '1' OR (E2M_en_def = '1' AND E2M_rst_def = '0')) THEN 
				completedAAU_stored <= '0';
			ELSIF (completedAAU = '1') THEN
				completedAAU_stored <= '1';
			END IF;
		END IF;
	END PROCESS;

	mux9 :  aauResult <= resAAH WHEN E_selAAH = '1' ELSE
		                 resAAL WHEN E_selAAL = '1' ELSE (OTHERS => '0');

	LLU : ENTITY WORK.aftab_llu
		GENERIC
		MAP(len => len)
		PORT
		MAP(
		ain      => D2E_ALU_op1_curr,
		bin      => D2E_ALU_op2_curr,
		selLogic => E_selLogic,
		result   => lluResult);
	
	BSU : ENTITY WORK.aftab_barrel_shifter
		GENERIC
		MAP(len => len)
		PORT
		MAP(
		shIn  => D2E_ALU_op1_curr,
		nSh   => D2E_ALU_op2_curr (4 DOWNTO 0),
		selSh => E_selShift,
		shOut => bsuResult);
	
	-- multiplexer to select the second input of the comparison circuitry
	muxcmp : ENTITY WORK.aftab_multiplexer
		GENERIC
		MAP(len => len)
		PORT
		MAP(
		a  => D2E_ALU_op2_curr,
		b  => D2E_op2_curr,
		s0 => E_cmp_selALUop2,
		s1 => E_cmp_selop2,
		w  => CMP_op2);

	comparator : ENTITY WORK.aftab_comparator
		GENERIC
		MAP(len => len)
		PORT
		MAP(
		ain                      => D2E_op1_curr,
		bin                      => CMP_op2,
		CompareSignedUnsignedBar => E_ComparedSignedUnsignedBar,
		Lt                       => E2M_lt_next,
		Eq                       => E2M_eq_next,
		Gt                       => E2M_gt_next);
	
	addSub : ENTITY WORK.aftab_adder_subtractor
		GENERIC
		MAP(len => len)
		PORT
		MAP(
		a      => D2E_ALU_op1_curr,
		b      => D2E_ALU_op2_curr,
		subSel => E_addSubBar,
		pass   => E_pass,
		cout   => OPEN,
		outRes => asuResult);

	E2M_ALU_res_next <= bsuResult WHEN E_selBSU = '1' ELSE
			lluResult WHEN E_selLLU = '1' ELSE
			asuResult WHEN E_selASU = '1' ELSE
			aauResult WHEN E_selAAU = '1' ELSE
			D2E_outCSR_curr WHEN E_selCSR = '1' ELSE (OTHERS => '0');

	-- FIXME: enable signal for the E2M register is activated by the ((completeAAU AND selAAU) OR selCSR OR selLLU OR selASU OR selBSU) AND M2WB_en 
	
	-- compute the exception flag for the E2M
	E2M_ex_flag_next <= D2E_ex_flag_curr OR dividedByZeroFlag;

	-- the complete E2M_rst
	E2M_rst_def <= E2M_rst OR (NOT(E2M_en_def) AND M2WB_en);

	-- MEMORY STAGE -------------------------------------------------------------------------------------------

	-- BRANCH HANDLING AND INTERFACE WITH THE CU (based on the conditions verified in the EXE stage)
	-- since there is no branch prediction, the static prediction that is made is always "untaken"
	-- IMPORTANT: SINCE THERE IS THE NEED TO IMPLEMENT JAL AND JALR IT IS NECESSARY TO AVOID CLEARING THE M2WB PIPELINE REG,
	-- BECAUSE THE BRANCH HAS TO PROPAGATE THROUGH IT TO REACH THE WRITE-BACK STAGE (WHERE THE WRITE OP IS PERFORMED)
	branch_outcome: process(M_opcode ,M_func3, E2M_lt_curr, E2M_gt_curr, E2M_eq_curr) BEGIN
		branch_taken <= '0';
		IF (M_opcode = branch) THEN 
			IF (M_func3(2) = '1' AND M_func3(0) = '0') THEN --BLT, BLTU
				IF (E2M_lt_curr = '1') THEN
					branch_taken <= '1';
				END IF;
			ELSIF (M_func3(2) = '1' AND M_func3(0) = '1') THEN --BGE, BGEU
				IF (E2M_gt_curr = '1' OR E2M_eq_curr = '1') THEN
					branch_taken <= '1';
				END IF;
			ELSIF (M_func3(2) = '0' AND M_func3(0) = '0') THEN --BEQ
				IF (E2M_eq_curr = '1') THEN
					branch_taken <= '1';
				END IF;
			ELSIF (M_func3(2) = '0' AND M_func3(0) = '1') THEN --BNE
				IF (E2M_eq_curr = '0') THEN
					branch_taken <= '1';
				END IF;
			END IF;
		ELSIF (M_opcode = JumpAndLink) THEN
			branch_taken <= '1';
		ELSIF (M_opcode = JumpAndLinkRegister) THEN
			branch_taken <= '1';
		END IF;
	END PROCESS;

	-- there is no need to sync reset here, because in case of misprediction there will be no load/store istruction in memory

	-- to interface with the memory write port
	dawu : ENTITY WORK.aftab_dawu
		PORT
		MAP(
		clk                 => clk,
		rst                 => rst,
		sync_rst 			=> M2WB_en,
		startDAWU           => DAWU_en,
		memReady            => memReady2, -- 1 if the second memory port is ready
		nBytes              => M_nBytes,
		addrIn              => E2M_ALU_res_curr,
		dataIn              => E2M_op2_curr, -- it corresponds to the output of the register file for the second register
		checkMisalignedDAWU => M_checkMisalignedDAWU, -- for now the misaligned write operation is unhandled
		addrOut             => memAddrDAWU, -- the address where to write dataDAWU
		dataOut             => dataDAWU, -- data to be sent to memory
		storeMisalignedFlag => OPEN,
		writeMem            => writeMemDAWU, -- the second memory port can be used either as a write port or a read port
		completeDAWU        => completedDAWU, -- controller
		bytesToWrite		=> bytesPerMemAccess); 

	-- start a new write op from memory when the instruction is a store and either the M2WB_en is high (so the current instruction
	-- is leaving the memory stage) or there is no instruction in the DAWU (if there is an instruction you shouldn't overwrite it)
	-- DONE: IN THE CU YOU SHOULD CHECK IF AN INSTRUCTION IS A STORE AND WHETHER IT THROWS AN EXCEPTION OR NOT
	DAWU_en <= M_startDAWU AND NOT(E2M_ex_flag_curr) AND ((M2WB_en AND NOT(M2WB_rst_def)) OR (NOT(writeMemDAWU) AND NOT(completedDAWU_stored)));

	completeDAWU_register: PROCESS (clk, rst) BEGIN
		IF (rst = '1') THEN
			completedDAWU_stored <= '0';
		ELSE
			IF (E2M_rst = '1' OR (E2M_en_def = '1' AND E2M_rst_def = '0')) THEN
				completedDAWU_stored <= '0';
			ELSIF (completedDAWU = '1') THEN
				completedDAWU_stored <= '1';
			END IF;
		END IF;
	END PROCESS;

	-- completed DAWU signal (to the CU, to notify that the memory write can be considered as complete)
	completedDAWU_def <= completedDAWU OR completedDAWU_stored OR NOT(M_startDAWU); -- you have to consider the case in which DAWU is not used

	-- signal to notify the CU that the instruction currently in memory is a store (so there is the need to wait for additional cycles when handling an
	-- interrupt, since at the end of the handling sequence we need to make sure that the store is completed)
	is_store_in_mem <= M_startDAWU;
	is_load_in_mem <= M_startDARU;

	-- to interface with the memory read port
	daru2 : ENTITY WORK.aftab_daru
		PORT
		MAP(
		clk                 => clk,
		rst                 => rst,
		sync_rst 			=> M2WB_en,
		startDARU           => DARU2_en, --controller
		nBytes              => M_nBytes,
		addrIn              => E2M_ALU_res_curr,
		memData             => memDataOut2,
		memReady            => memReady2, --core
		dataInstrBar        => '1', -- this DARU is used exclusively for data
		checkMisalignedDARU => '0', -- access to data can be misaligned
		instrMisalignedFlag => OPEN,
		loadMisalignedFlag  => OPEN,
		completeDARU        => completedDARU2, -- controller
		dataOut             => dataDARU2, -- we can enable forwarding of the data read to the output of the DAWU
		addrOut             => memAddrDARU2, -- as before
		readMem             => readMemDARU2,  -- core
		bytesToRead			=> bytesPerMemAccess);

	-- start a new read op from memory when the instruction is a load and either the M2WB_en is high (so the current instruction
	-- is leaving the memory stage) or there is no instruction in the DARU (if there is an instruction you shouldn't overwrite it)
	DARU2_en <= M_startDARU AND NOT(E2M_ex_flag_curr) AND ((M2WB_en AND NOT(M2WB_rst_def)) OR (NOT(readMemDARU2) AND NOT(completedDARU2_stored)));

	completeDARU2_register: PROCESS (clk, rst) BEGIN
		IF (rst = '1') THEN
			completedDARU2_stored <= '0';
		ELSE
			IF (E2M_rst = '1' OR (E2M_en_def = '1' AND E2M_rst_def = '0')) THEN
				completedDARU2_stored <= '0';
			ELSIF (completedDARU2 = '1') THEN
				completedDARU2_stored <= '1';
			END IF;
		END IF;
	END PROCESS;

	-- completedDARU signal, to notify the CU about the completed memory read operation
	completedDARU2_def <= completedDARU2 OR completedDARU2_stored OR NOT(M_startDARU);

	-- to sign extend the value read from memory
	sulu : ENTITY WORK.aftab_sulu
		GENERIC
		MAP (len => len)
		PORT
		MAP(
		loadByteSigned => M_ldByteSigned,
		loadHalfSigned => M_ldHalfSigned,
		load           => M_load,
		dataIn         => dataDARU2,
		dataOut        => M2WB_MEM_res_next);

	-- exceptionHandling and interruptHandling are "Mealy set" by Normal and "Moore set" by int/ex Handling states. 
	-- TODO: CHECK IF IT IS OK USING exception/interruptRaise IN THIS PART OF THE CODE
	M2WB_rst_def <= M2WB_rst OR (NOT(M2WB_en) AND instructionDone AND NOT(exceptionRaise) AND NOT(interruptRaise));

	-- select address to be sent to memory between the one produced by DARU and the one produced by DAWU
	memAddr2 <= memAddrDARU2 WHEN readMemDARU2 = '1' ELSE memAddrDAWU;
	
	-----------------------------------------------------------------------------------------------------------
	--- WRITE-BACK STAGE AND OUTSIDE THE PIPELINE -------------------------------------------------------------
	-----------------------------------------------------------------------------------------------------------	

	-- validAccessCSR coming from WB
	WB_validAccessCSR <= M2WB_validAccessCSR_curr;

	-- hazard solved flag: raised when the instruction that produces the register needed by the one in DEC completes
	hazard_solved <= M2WB_hazard_flag_curr AND instructionDone;

	-- selection of the data to be written in the destination register in the RF
	mux10 :	writeData <= M2WB_ALU_res_curr WHEN WB_selALU = '1' ELSE
			   M2WB_PC_plus4_curr WHEN WB_selPC4 = '1' ELSE
			   M2WB_MEM_res_curr WHEN WB_selMem = '1' ELSE
			   (OTHERS => '0');

	-- address of the register to be modified in the CSR register bank
	writeAddressRegBank <= WB_AddressRegBank;

	-- to produce all the addresses of the registers to be modified during write-back handling of exceptions/interrupts
	CSRCounter : ENTITY WORK.aftab_csr_counter
		GENERIC
		MAP (len => 3)
		PORT
		MAP(
		clk     => clk,
		rst     => rst,
		dnCnt   => dnCntCSR, -- all these signals are driven directly by the CU during exception handling states
		upCnt   => upCntCSR,
		ldCnt   => ldCntCSR,
		zeroCnt => zeroCntCSR,
		ldValue => ldValueCSR,
		outCnt  => cntOutput
		);
	CSRAddressingDecoder : ENTITY WORK.aftab_csr_addressing_decoder
		PORT
		MAP(
		cntOutput => cntOutput,
		outAddr   => outAddr
		);

	-- to select the address of the register to be written/read (in write-back) from register_bank
	mux7 : preAddressRegBank <= M2WB_instr_curr(31 DOWNTO 20) WHEN WB_selCSRAddrFromInst = '1' ELSE
								outAddr WHEN selRomAddress = '1' ELSE
								X"302" WHEN selMedeleg_CSR = '1' ELSE
								X"303" WHEN selMideleg_CSR = '1' ELSE (OTHERS => '0');

	-- address of the mirror register for the CSR to be updated 
	mirrorAddress <= "0000" & preAddressRegBank(7 DOWNTO 0);
		
	mirrorUserBar <= NOT(mirrorUserCU);

	-- this multiplexer selects the address of the register to be overwritten
	mux8 : ENTITY WORK.aftab_multiplexer
		GENERIC
		MAP(len => 12)
		PORT
		MAP(
		a  => preAddressRegBank,
		b  => mirrorAddress,
		s0 => mirrorUserBar,
		s1 => mirrorUserCU,
		w  => WB_AddressRegBank); -- the address of the register that has to be read or overwritten

	-- enable signal for the RB write operation: the write has to be done either when the instruction requires it or when it is
	-- part of the handling sequence for an exception
	-- you shouldn't overwrite the register in write-back when the corresponding instruction threw an exception
	-- FIXME: FIX THIS WRITE ENABLE, WHEN DEALING WITH AN EXCEPTION/INT THE SIGNAL PRODUCED FOR THE INSTRUCTION SHOULD BECOME IRRELEVANT
	-- ----> writeRegBank is directly driven by the CU, so there is no need to recognize when the register bank is being used for 
	-- exception handling and when it is being used as a part of the the execution of a CSR instruction
	-- writeRB_inst_or_ex <= writeRegBank OR (WB_writeRB_inst AND NOT(M2WB_ex_flag_curr));

	--CSR Units -------------------------------------------------------------------------------------------------

	-- signal to enable or disable mipCC update (disabled during interrupt hardware handling)
	mipCCLd          <= NOT (mipCCLdDisable);

	-- all interrupt sources from the outside
	interruptSources <= platformInterruptSignals & "0000" & machineExternalInterrupt &
						"00" & userExternalInterrupt & machineTimerInterrupt & 
						"00" & userTimerInterrupt & machineSoftwareInterrupt & 
						"00" & userSoftwareInterrupt;

	interSrcSynchReg : ENTITY work.aftab_register
		GENERIC
		MAP(len => 32)
		PORT
		MAP(
		clk    => clk,
		rst    => rst,
		zero   => '0',
		load   => mipCCLd,
		inReg  => interruptSources,
		outReg => CCmip -- this register is a carbon copy of the real MIP, which is updated when the interrupt is handled
		);

	-- to select the next value to be written in the CSR register bank
	CSRISL : ENTITY WORK.aftab_csr_isl
		GENERIC
		MAP (len => len)
		PORT
		MAP(
		selP1                          => selP1CSR, -- all the following signals are control signals generated by CU during writeback
		selIm                          => selImmCSR,
		selReadWrite                   => selReadWriteCSR,
		clr                            => clrCSR,
		set                            => setCSR,
		selPC                          => selPC_CSR, -- when overwriting XEPC during exception handling, driven by CU
		selmip                         => selCCMip_CSR, -- when rewriting CCMip, during int/ex handling
		selCause                       => selCause_CSR, -- as before
		selTval                        => selTval_CSR, -- as before
		machineStatusAlterationPreCSR  => machineStatusAlterationPreCSR, -- driven by int/ex handling states
		userStatusAlterationPreCSR     => userStatusAlterationPreCSR, -- as before
		machineStatusAlterationPostCSR => machineStatusAlterationPostCSR, -- as before
		userStatusAlterationPostCSR    => userStatusAlterationPostCSR, -- as before
		mirrorUstatus                  => mirrorUstatus, -- when you mirror you have to write a masked value, removing the privileged bits
		mirrorUie                      => mirrorUie, -- these signals are driven by the register bank, which determines if a mirror update has to be done
		mirrorUip                      => mirrorUip,
		mirrorUser                     => mirrorUserCU,
		curPRV                         => curPRV, -- current privilege level (user or machine)
		ir19_15                        => M2WB_instr_curr (19 DOWNTO 15),
		CCmip                          => CCmip, -- current value for the carbon copy of MIP
		causeCode                      => causeCode, -- the cause resolved by the ICCD
		trapValue                      => trapValue, -- the trap number resolved by the ICCD
		P1                             => M2WB_op1_curr, -- the first operand read directly from the RF
		PC                             => CSR_PC, -- MUX TO SELECT EITHER WRITE-BACK OR MEMORY
		outCSR                         => outCSR_write_val, -- MUX between the value read during ex/int handling and the one read during decode
		previousPRV                    => previousPRV, -- ???
		inCSR                          => inCSR); -- the value to be written in the CSR RB

	-- drive the PC_from_WB signal high when you are handling an interrupt and there is a store in memory (if exceptionRaise is high
	-- then you will handle the exception first, because it has a higher priority)
	PC_from_WB <= M_startDAWU AND interruptRaise AND NOT(exceptionRaise);

	-- The PC value to be loaded in the CSR register
	-- It could be either the PC value from write-back or the one from memory (in case there is a store in memory and we
	-- are handling an interrupt). Before reading the value from memory we have to make sure that the store is concluded,
	-- so an additional state waitForStore is needed when handling interrupts.
	CSR_PC <= M2WB_PC_plus4_curr WHEN PC_from_WB = '0' ELSE E2M_PC_plus4_curr;

	-- outCSR value to be written in CSR RB: the D2E_outCSR_curr stores the value directly read from the register bank,
	-- while the M2WB_outCSR_curr stores the one which propagated through the pipeline coming from DEC.
	outCSR_write_val <= M2WB_outCSR_curr WHEN CSR_from_WB = '0' ELSE outCSR_reg;
	
	-- register to store the value read from the CSR register bank
	outCSR_register : ENTITY work.aftab_register
		GENERIC
		MAP(len => 32)
		PORT
		MAP(
		clk    => clk,
		rst    => rst,
		zero   => '0',
		load   => '1',
		inReg  => D2E_outCSR_next,
		outReg => outCSR_reg
		);

	interrCheckCauseDetection : ENTITY WORK.aftab_iccd
		GENERIC
		MAP(len => len)
		PORT
		MAP(
		clk            => clk,
		rst            => rst,
		inst           => inst,
		outPC          => M2WB_PC_curr, -- NEEDED IF A MISALIGNED ADDRESS EXCEPTION HAS BEEN THROWN
		outADR         => M2WB_ALU_res_curr, -- NOT USED, could be used when identifying data address misaligned exceptions
		mipCC          => CCmip,
		mieCC          => CCmie,
		midelegCSR     => D2E_outCSR_next, -- the read output of the CSR RB is normally sent to register D2E_outCSR 
		medelegCSR     => D2E_outCSR_next,
		ldDelegation   => ldDelegation, -- driven by CU during int/ex handling
		ldMachine      => ldMachine, -- as before
		ldUser         => ldUser, -- as before
		mieFieldCC     => CCmieField,
		uieFieldCC     => CCuieField,
		tempFlags      => tempFlags, -- it collects all the flags associated to the possible exception causes
		interruptRaise => interruptRaiseTemp,
		exceptionRaise => exceptionRaiseTemp,
		delegationMode => delegationMode, -- to CU during int/exception handling, it determines which privilege mode has to be set to handle the selected exception/interrupt
		curPRV         => curPRV, -- computed privilege level
		causeCode      => causeCode,
		trapValue      => trapValue
		);

	-- NOT NEEDED
	-- instrMisalignedOut <= instrMisalignedFlag;
	-- loadMisalignedOut  <= '0'; --not used
	-- storeMisalignedOut <= '0'; --not used
	-- dividedByZeroOut   <= dividedByZeroFlag;
	
	interruptRaise <= interruptRaiseTemp;
	exceptionRaise <= exceptionRaiseTemp;
	
	causeCodeTemp <= causeCode(31) & causeCode (4 DOWNTO 0);

	interruptStartAddressGenerator : ENTITY WORK.aftab_isagu
		GENERIC
		MAP(len => len)
		PORT
		MAP(
		tvecBase                      => D2E_outCSR_next,
		causeCode                     => causeCodeTemp,
		modeTvec                      => modeTvec,
		interruptStartAddressDirect   => interruptStartAddressDirect,
		interruptStartAddressVectored => interruptStartAddressVectored
		);

	-- the exception flags as they are computed in write-back
	tempFlags   <= M2WB_ecall_flag_curr & M2WB_divided_by_zero_flag_curr & 
						  M2WB_illegal_instruction_flag_curr & M2WB_instr_misaligned_flag_curr & 
						  '0' & '0';
	-- an interrupt is raised when the condition applies and when the instruction in write-back terminates.
	interruptRaise <= interruptRaiseTemp AND instructionDone AND M2WB_valid_curr;
	-- the same applies for exceptions
	exceptionRaise <= exceptionRaiseTemp AND instructionDone AND M2WB_valid_curr;

	-- HAZARD DETECTION -----------------------------------------------------------------------------
	-- when an haz signal is raised, the CU should stop the pipeline for the number of cycles that is 
	-- necessary to solve the hazard (to compute the result to be used as an operand for the stalled
	-- instruction). We could use a signal which is injected in the pipeline so that the CU resumes
	-- the operation of the stalled stages when this signal reaches the write-back stage. The flag 
	-- should be set on the earlier stage in which there is a dependency. Pipeline registers should
	-- not be reset as the instructions advance through the pipeline, because the hazard must be
	-- detected until it is solved (the hazard signal coming in write-back should override the 
	-- hazard flag). When the hazard is solved you have to reset the E2M register (E2M_rst), because otherwise
	-- you would detect another hazard between the stalled memory stage and the new decode, and reset
	-- all the hazard flags.

	-- the CU receives: hazEX and hazM
	-- the CU produces: en and rst signals for the stages in the pipeline
	-- it moves between two states: no_haz, haz_detected (it returns to no_haz when the haz is solved or the
	-- pipeline is reset by a misprediction/interrupt/exception)

	-- HAZARDS DECODE - EXECUTE ---------------------
	-- DONE: DEFINE ALL INSTRUCTION TYPES AS CONSTANTS
	-- WE DON'T NEED TO CONSIDER ALSO ecall (I TYPE) AND mret/uret (S TYPE) INSTRUCTIONS, BECAUSE THEY PERFORM 
	-- COMPUTATION IN WRITE-BACK (WHEN ALL THE OTHER INSTRUCTIONS HAVE ALREADY TERMINATED EXECUTION)
	hazardEX_operands_result_retrieval: PROCESS(inst_type, GI2D_instr_curr, D2E_instr_curr, D2E_inst_type_curr) BEGIN

		hazEX_first_operand <= (OTHERS => '0');
		hazEX_second_operand <= (OTHERS => '0');
		hazEX_result <= (OTHERS => '0');
		hazEX_first_operand_en <= '0';
		hazEX_second_operand_en <= '0';
		hazEX_result_en <= '0';
		hazEX_zero_first_operand <= '0';
		hazEX_zero_second_operand <= '0';
		hazEX_CSR_second_operand_en <= '0';
		hazEX_CSR_result_en <= '0';
		hazEX_CSR_mirror <= '0';

		-- operands selection based on instruction type
		IF (inst_type = R_type) THEN -- arithmetic instructions
			hazEX_first_operand <= GI2D_instr_curr(19 DOWNTO 15);
			hazEX_first_operand_en <= '1';
			hazEX_second_operand <= GI2D_instr_curr(24 DOWNTO 20);
			hazEX_second_operand_en <= '1';
		ELSIF (inst_type = I_type) THEN -- arithmetic instruction with an immediate as an operand
			-- 1110011(2) = 73h
			IF (GI2D_instr_curr(6 DOWNTO 0) = "1110011" AND GI2D_instr_curr(14 DOWNTO 12) /= "000") THEN
				-- this covers all CSR instructions except for ecall which can't raise hazards since it 
				-- performs computation and read registers during write-back
				-- the check is implemented by reading the opcode and the funct3 field
				hazEX_first_operand <= GI2D_instr_curr(19 DOWNTO 15);
				hazEX_first_operand_en <= '1';
				hazEX_CSR_second_operand_en <= '1';
				hazEX_CSR_second_operand <= GI2D_instr_curr(31 DOWNTO 20);
			ELSIF (GI2D_instr_curr(6 DOWNTO 0) /= "1110011") THEN
				-- this covers all the other I instructions (except for ecall)
				hazEX_first_operand <= GI2D_instr_curr(19 DOWNTO 15);
				hazEX_first_operand_en <= '1';
				-- normal I instructions use the extended immediate as the second operand
				hazEX_second_operand_en <= '0';
			END IF;
		ELSIF (inst_type = S_type) THEN -- store instructions
			-- this covers everything but mret and uret
			IF (GI2D_instr_curr(6 DOWNTO 0) /= "1110011") THEN
				hazEX_first_operand <= GI2D_instr_curr(19 DOWNTO 15);
				hazEX_first_operand_en <= '1';
				hazEX_second_operand <= GI2D_instr_curr(24 DOWNTO 20);
				hazEX_second_operand_en <= '1';
			END IF;
		ELSIF (inst_type = B_type) THEN -- conditional branches
			hazEX_first_operand <= GI2D_instr_curr(19 DOWNTO 15);
			hazEX_first_operand_en <= '1';
			hazEX_second_operand <= GI2D_instr_curr(24 DOWNTO 20);
			hazEX_second_operand_en <= '1';
		ELSIF (inst_type = U_type) THEN -- auipc, lui (the load the upper bits in a register)
			hazEX_first_operand_en <= '0';
			hazEX_second_operand_en <= '0';
		ELSIF (inst_type = J_type) THEN -- unconditional branches
			hazEX_first_operand_en <= '0';
			hazEX_second_operand_en <= '0';
		END IF;

		-- result selection based on instruction type
		IF (D2E_inst_type_curr = R_type) THEN
			hazEX_result <= D2E_instr_curr(11 DOWNTO 7);
			hazEX_result_en <= '1';
		ELSIF (D2E_inst_type_curr = I_type) THEN
			-- this covers all CSR instructions but ecall
			-- all CSR instructions share with the classic I instructions the position of the result, but they update
			-- the value of the CSR too
			IF (D2E_instr_curr(6 DOWNTO 0) = "1110011" and D2E_instr_curr(14 DOWNTO 12) /= "000") THEN
				-- consider instructions which update registers that are mirrored (yoou should raise an hazard also if you are accessing USTATUS and MSTATUS is being modified by an instruction which has already been decoded)
				IF (D2E_instr_curr(27 DOWNTO 20) = x"00" OR D2E_instr_curr(27 DOWNTO 20) = x"04" OR D2E_instr_curr(27 DOWNTO 20) = x"44") THEN
					hazEX_CSR_mirror <= '1'; 
				END IF;
				hazEX_CSR_result <= D2E_instr_curr(31 DOWNTO 20);
				hazEX_CSR_result_en <= '1';
			END IF;
			hazEX_result <= D2E_instr_curr(11 DOWNTO 7);
			hazEX_result_en <= '1';
		ELSIF (D2E_inst_type_curr = S_type) THEN
			hazEX_result_en <= '0';
		ELSIF (D2E_inst_type_curr = B_type) THEN
			hazEX_result_en <= '0';
		ELSIF (D2E_inst_type_curr = U_type) THEN
			hazEX_result <= D2E_instr_curr(11 DOWNTO 7);
			hazEX_result_en <= '1';
		ELSIF (D2E_inst_type_curr = J_type) THEN -- both jal and jalr store the return address in a register
			hazEX_result <= D2E_instr_curr(11 DOWNTO 7);
			hazEX_result_en <= '1';
		END IF;

		-- register zero detection
		IF (GI2D_instr_curr(19 DOWNTO 15) = "00000") THEN
			hazEX_zero_first_operand <= '1';
		END IF;
		IF (GI2D_instr_curr(24 DOWNTO 20) = "00000") THEN
			hazEX_zero_second_operand <= '1';
		END IF;
	END PROCESS;

	-- the process for handling the hazard involving the operands and the results determined before 
	hazardEX_handling: PROCESS(hazEX_first_operand, hazEX_first_operand_en, hazEX_second_operand, hazEX_second_operand_en, hazEX_result, 
	hazEX_result_en, hazEX_zero_first_operand, hazEX_zero_second_operand, hazEX_CSR_second_operand, hazEX_CSR_second_operand_en,
	hazEX_CSR_result, hazEX_CSR_result_en, D2E_ex_flag_next) BEGIN
		hazEX <= '0';
		-- comparison between first operand and result
		IF (D2E_ex_flag_next = '0' AND hazEX_first_operand = hazEX_result AND hazEX_first_operand_en = '1' AND hazEX_result_en = '1' AND hazEX_zero_first_operand = '0') THEN
			hazEX <= '1';
		-- comparison between second operand and result (considering also a comparison between a CSR result and a CSR second operand)
		ELSIF (D2E_ex_flag_next = '1' AND ((hazEX_second_operand = hazEX_result AND hazEX_second_operand_en = '1' AND hazEX_result_en = '1' AND hazEX_zero_second_operand = '0')
			OR (((hazEX_CSR_mirror = '0' AND hazEX_CSR_second_operand = hazEX_CSR_result) OR (hazEX_CSR_mirror = '1' AND hazEX_CSR_second_operand(7 DOWNTO 0) = hazEX_CSR_result (7 DOWNTO 0))) AND hazEX_CSR_second_operand_en = '1' AND hazEX_CSR_result_en = '1' ))) THEN
			hazEX <= '1';
		END IF;
	END PROCESS;

	-- HAZARDS DECODE - MEMORY ----------------------
	hazardM_operands_result_retrieval: PROCESS(inst_type, GI2D_instr_curr, E2M_instr_curr, E2M_inst_type_curr) BEGIN

		hazM_first_operand <= (OTHERS => '0');
		hazM_second_operand <= (OTHERS => '0');
		hazM_result <= (OTHERS => '0');
		hazM_first_operand_en <= '0';
		hazM_second_operand_en <= '0';
		hazM_result_en <= '0';
		hazM_zero_first_operand <= '0';
		hazM_zero_second_operand <= '0';
		hazM_CSR_second_operand_en <= '0';
		hazM_CSR_result_en <= '0';
		hazM_CSR_mirror <= '0';

		-- operands selection based on instruction type
		IF (inst_type = R_type) THEN -- arithmetic instructions
			hazM_first_operand <= GI2D_instr_curr(19 DOWNTO 15);
			hazM_first_operand_en <= '1';
			hazM_second_operand <= GI2D_instr_curr(24 DOWNTO 20);
			hazM_second_operand_en <= '1';
		ELSIF (inst_type = I_type) THEN -- arithmetic instruction with an immediate as an operand
			IF (GI2D_instr_curr(6 DOWNTO 0) = "1110011" AND GI2D_instr_curr(14 DOWNTO 12) /= "000") THEN
				-- this covers all CSR instructions except for ecall which can't raise hazards since it 
				-- performs computation and read registers during write-back
				-- the check is implemented by reading the opcode and the funct3 field
				hazM_first_operand <= GI2D_instr_curr(19 DOWNTO 15);
				hazM_first_operand_en <= '1';
				hazM_CSR_second_operand_en <= '1';
				hazM_CSR_second_operand <= GI2D_instr_curr(31 DOWNTO 20);
			ELSIF (GI2D_instr_curr(6 DOWNTO 0) /= "1110011") THEN
				-- this covers all the other I instructions (except for ecall)
				hazM_first_operand <= GI2D_instr_curr(19 DOWNTO 15);
				hazM_first_operand_en <= '1';
				-- normal I instructions use the extended immediate as the second operand
				hazM_second_operand_en <= '0';
			END IF;
		ELSIF (inst_type = S_type) THEN -- store instructions
			-- this covers everything but mret and uret
			IF (GI2D_instr_curr(6 DOWNTO 0) /= "1110011") THEN
				hazM_first_operand <= GI2D_instr_curr(19 DOWNTO 15);
				hazM_first_operand_en <= '1';
				hazM_second_operand <= GI2D_instr_curr(24 DOWNTO 20);
				hazM_second_operand_en <= '1';
			END IF;
		ELSIF (inst_type = B_type) THEN -- conditional branches
			hazM_first_operand <= GI2D_instr_curr(19 DOWNTO 15);
			hazM_first_operand_en <= '1';
			hazM_second_operand <= GI2D_instr_curr(24 DOWNTO 20);
			hazM_second_operand_en <= '1';
		ELSIF (inst_type = U_type) THEN -- auipc, lui (the load the upper bits in a register)
			hazM_first_operand_en <= '0';
			hazM_second_operand_en <= '0';
		ELSIF (inst_type = J_type) THEN -- unconditional branches
			hazM_first_operand_en <= '0';
			hazM_second_operand_en <= '0';
		END IF;

		-- result selection based on instruction type
		IF (E2M_inst_type_curr = R_type) THEN
			hazM_result <= E2M_instr_curr(11 DOWNTO 7);
			hazM_result_en <= '1';
		ELSIF (E2M_inst_type_curr = I_type) THEN
			-- this covers all CSR instructions but ecall
			-- all CSR instructions share with the classic I instructions the position of the result, but they update
			-- the value of the CSR too
			IF (E2M_instr_curr(6 DOWNTO 0) = "1110011" and E2M_instr_curr(14 DOWNTO 12) /= "000") THEN
				-- consider instructions which update registers that are mirrored (yoou should raise an hazard also if you are accessing USTATUS and MSTATUS is being modified by an instruction which has already been decoded)
				IF (E2M_instr_curr(27 DOWNTO 20) = x"00" OR E2M_instr_curr(27 DOWNTO 20) = x"04" OR E2M_instr_curr(27 DOWNTO 20) = x"44") THEN
					hazM_CSR_mirror <= '1'; 
				END IF;
				hazM_CSR_result <= E2M_instr_curr(31 DOWNTO 20);
				hazM_CSR_result_en <= '1';
			END IF;
			hazM_result <= E2M_instr_curr(11 DOWNTO 7);
			hazM_result_en <= '1';
		ELSIF (E2M_inst_type_curr = S_type) THEN
			hazM_result_en <= '0';
		ELSIF (E2M_inst_type_curr = B_type) THEN
			hazM_result_en <= '0';
		ELSIF (E2M_inst_type_curr = U_type) THEN
			hazM_result <= E2M_instr_curr(11 DOWNTO 7);
			hazM_result_en <= '1';
		ELSIF (E2M_inst_type_curr = J_type) THEN -- both jal and jalr store the return address in a register
			hazM_result <= E2M_instr_curr(11 DOWNTO 7);
			hazM_result_en <= '1';
		END IF;

		-- register zero detection
		IF (GI2D_instr_curr(19 DOWNTO 15) = "00000") THEN
			hazM_zero_first_operand <= '1';
		END IF;
		IF (GI2D_instr_curr(24 DOWNTO 20) = "00000") THEN
			hazM_zero_second_operand <= '1';
		END IF;
	END PROCESS;

	-- the process for handling the hazard involving the operands and the results determined before 
	hazardM_handling: PROCESS(hazM_first_operand, hazM_first_operand_en, hazM_second_operand, hazM_second_operand_en, hazM_result, 
	hazM_result_en, hazM_zero_first_operand, hazM_zero_second_operand, hazM_CSR_second_operand, hazM_CSR_second_operand_en,
	hazM_CSR_result, hazM_CSR_result_en, D2E_ex_flag_next) BEGIN
		hazM <= '0';
		-- comparison between first operand and result
		IF (D2E_ex_flag_next = '0' AND hazM_first_operand = hazM_result AND hazM_first_operand_en = '1' AND hazM_result_en = '1' AND hazM_zero_first_operand = '0') THEN
			hazM <= '1';
		-- comparison between second operand and result (considering also a comparison between a CSR result and a CSR second operand)
		ELSIF (D2E_ex_flag_next = '0' AND ((hazM_second_operand = hazM_result AND hazM_second_operand_en = '1' AND hazM_result_en = '1' AND hazM_zero_second_operand = '0')
			OR (((hazM_CSR_mirror = '0' AND hazM_CSR_second_operand = hazM_CSR_result) OR (hazM_CSR_mirror = '1' AND hazM_CSR_second_operand(7 DOWNTO 0) = hazM_CSR_result (7 DOWNTO 0))) AND hazM_CSR_second_operand_en = '1' AND hazM_CSR_result_en = '1'))) THEN
			hazM <= '1';
		END IF;
	END PROCESS;

	-- HAZARD FLAGS: to be propagated through the pipeline
	-- E2M flag is raised when there is an hazard DEC-EX (the flag will actually be set only when the instruction in EX passes to MEM)
	E2M_hazard_flag_next <= '1' WHEN hazEX = '1' ELSE '0';
	-- M2WB flag is raised when there is a DEC-MEM hazard or when the previous DEC-EX hazard is leaving memory
	M2WB_hazard_flag_next <= '1' WHEN hazM = '1' OR E2M_hazard_flag_curr = '1' ELSE '0';
	
	-- PIPELINE REGISTERS ---------------------------------------------------------------------------
	-- FIXME: REMOVE USELESS REGISTERS

	pipeline_registers: PROCESS (clk, rst) BEGIN
		-- async reset
		IF (rst = '1') THEN
			-- reset everything
			F2GI_PC_plus4_curr <= (OTHERS => '0');
			GI2D_valid_curr <= '0';
			GI2D_ex_flag_curr <= '0';
			GI2D_instr_curr <= (OTHERS => '0');
			GI2D_PC_plus4_curr <= (OTHERS => '0');
			GI2D_PC_curr <= (OTHERS => '0');
			GI2D_instr_misaligned_flag_curr <= '0';
			GI2D_valid_curr <= '0';
			D2E_validAccessCSR_curr <= '0';
			D2E_valid_curr <= '0';
			D2E_instr_misaligned_flag_curr <= '0';
			D2E_ALU_op1_curr <= (OTHERS => '0');
			D2E_ALU_op2_curr <= (OTHERS => '0');
			D2E_outCSR_curr <= (OTHERS => '0');
			D2E_op1_curr <= (OTHERS => '0');
			D2E_op2_curr <= (OTHERS => '0');
			D2E_PC_plus4_curr <= (OTHERS => '0'); -- needed by ecall
			D2E_PC_curr <= (OTHERS => '0'); -- needed by exception management (you need to pass it as a parameter to the trap)
			D2E_inst_type_curr <= (OTHERS => '0');
			D2E_instr_curr <= (OTHERS => '0');
			D2E_illegal_instruction_flag_curr <= '0';
			D2E_ecall_flag_curr <= '0';
			D2E_ctrl_word_curr <= (OTHERS => '0');
			D2E_ex_flag_curr <= '0';
			E2M_validAccessCSR_curr <= '0';
			E2M_valid_curr <= '0';
			E2M_hazard_flag_curr <= '0';
			E2M_instr_misaligned_flag_curr <= '0';
			E2M_ALU_res_curr <= (OTHERS => '0');
			E2M_outCSR_curr <= (OTHERS => '0'); -- to propagate the value read from the CSR bank
			E2M_op1_curr <= (OTHERS => '0');
			E2M_op2_curr <= (OTHERS => '0');
			E2M_PC_plus4_curr <= (OTHERS => '0'); -- needed by ecall
			E2M_PC_curr <= (OTHERS => '0');
			E2M_inst_type_curr <= (OTHERS => '0');
			E2M_instr_curr <= (OTHERS => '0');
			E2M_illegal_instruction_flag_curr <= '0';
			E2M_ecall_flag_curr <= '0';
			E2M_ctrl_word_curr <= (OTHERS => '0');
			E2M_ex_flag_curr <= '0';
			E2M_lt_curr <= '0';
			E2M_gt_curr <= '0';
			E2M_eq_curr <= '0';
			E2M_divided_by_zero_flag_curr <= '0';
			M2WB_validAccessCSR_curr <= '0';
			M2WB_valid_curr <= '0';
			M2WB_hazard_flag_curr <= '0';
			M2WB_instr_misaligned_flag_curr <= '0';
			M2WB_op1_curr <= (OTHERS => '0');
			M2WB_ALU_res_curr <= (OTHERS => '0');
			M2WB_outCSR_curr <= (OTHERS => '0'); -- to propagate the value read from the CSR bank
			M2WB_PC_plus4_curr <= (OTHERS => '0'); -- needed by ecall
			M2WB_PC_curr <= (OTHERS => '0');
			M2WB_inst_type_curr <= (OTHERS => '0');
			M2WB_instr_curr <= (OTHERS => '0');
			M2WB_illegal_instruction_flag_curr <= '0';
			M2WB_ecall_flag_curr <= '0';
			M2WB_ctrl_word_curr <= (OTHERS => '0');
			M2WB_ex_flag_curr <= '0';
			M2WB_lt_curr <= '0';
			M2WB_gt_curr <= '0';
			M2WB_eq_curr <= '0';
			M2WB_divided_by_zero_flag_curr <= '0';
			M2WB_MEM_res_curr <= (OTHERS => '0');

		ELSIF (rising_edge(clk)) THEN

			-- F2GI pipeline register: reset under the same condition as GI2D
			-- TODO: WRONG CONDITION, YOU DON'T WANT THE F2GI TO RESET WHEN AN INSTRUCTION LEAVES DECODE!
			-- IF (GI2D_rst_def = '1') THEN
			-- 	F2GI_PC_plus4_curr <= (OTHERS => '0');
			-- 	F2GI_valid_curr <= '0';
			-- the F2GI register is enabled when a new instruction is sent to the DARU1
			-- ELSIF (DARU1_en = '1') THEN
			IF (DARU1_en = '1') THEN
				F2GI_valid_curr <= '1';
				F2GI_PC_plus4_curr <= F2GI_PC_plus4_next;
			END IF;

			-- GI2D pipeline registers
			IF (GI2D_rst_def = '1') THEN
				GI2D_valid_curr <= '0';
				GI2D_ex_flag_curr <= '0';
				GI2D_instr_curr <= (OTHERS => '0');
				GI2D_PC_plus4_curr <= (OTHERS => '0');
				GI2D_PC_curr <= (OTHERS => '0');
				GI2D_instr_misaligned_flag_curr <= '0';
				GI2D_valid_curr <= '0';
			ELSIF (GI2D_en_def = '1') THEN
				GI2D_valid_curr <= F2GI_valid_curr; -- a new valid instruction is propagating to DEC
				GI2D_ex_flag_curr <= GI2D_ex_flag_next;
				GI2D_instr_curr <= GI2D_instr_next;
				GI2D_PC_plus4_curr <= GI2D_PC_plus4_next;
				GI2D_PC_curr <= GI2D_PC_next; -- taken directly from the output of the DARU1
				GI2D_instr_misaligned_flag_curr <= instrMisalignedFlag; -- kept high until the instruction leaves the DARU and the GI stage
				GI2D_valid_curr <= '1';
			END IF;

			-- D2E pipeline registers
			IF (D2E_rst_def = '1') THEN
				D2E_validAccessCSR_curr <= '0';
				D2E_valid_curr <= '0';
				D2E_instr_misaligned_flag_curr <= '0';
				D2E_ALU_op1_curr <= (OTHERS => '0');
				D2E_ALU_op2_curr <= (OTHERS => '0');
				D2E_outCSR_curr <= (OTHERS => '0');
				D2E_op1_curr <= (OTHERS => '0');
				D2E_op2_curr <= (OTHERS => '0');
				D2E_PC_plus4_curr <= (OTHERS => '0'); -- needed by ecall
				D2E_PC_curr <= (OTHERS => '0');
				D2E_inst_type_curr <= (OTHERS => '0');
				D2E_instr_curr <= (OTHERS => '0');
				D2E_illegal_instruction_flag_curr <= '0';
				D2E_ecall_flag_curr <= '0';
				D2E_ctrl_word_curr <= (OTHERS => '0');
				D2E_ex_flag_curr <= '0';
			ELSIF (D2E_en_def = '1') THEN
				D2E_validAccessCSR_curr <= validAccessCSR;
				D2E_valid_curr <= GI2D_valid_curr;
				D2E_ex_flag_curr <= D2E_ex_flag_next;
				D2E_instr_misaligned_flag_curr <= GI2D_instr_misaligned_flag_curr;
				D2E_ALU_op1_curr <= D2E_ALU_op1_next;
				D2E_ALU_op2_curr <= D2E_ALU_op2_next;
				D2E_outCSR_curr <= D2E_outCSR_next;
				D2E_op1_curr <= D2E_op1_next; -- NEEDED BY CSR INSTRUCTIONS
				D2E_op2_curr <= D2E_op2_next; -- NEEDED BY STORE INSTRUCTIONS
				D2E_PC_plus4_curr <= GI2D_PC_plus4_curr; -- needed by ecall
				D2E_PC_curr <= GI2D_PC_curr;
				D2E_inst_type_curr <= inst_type; -- from CU
				D2E_instr_curr <= GI2D_instr_curr;
				D2E_illegal_instruction_flag_curr <= illegalInstrFlag;
				D2E_ecall_flag_curr <= ecallFlag;
				D2E_ctrl_word_curr <= D2E_ctrl_word_next; -- assembled in the following section
			END IF;

			-- E2M pipeline registers
			IF (E2M_rst_def = '1') THEN
				E2M_validAccessCSR_curr <= '0';
				E2M_valid_curr <= '0';
				E2M_hazard_flag_curr <= '0';
				E2M_instr_misaligned_flag_curr <= '0';
				E2M_ALU_res_curr <= (OTHERS => '0');
				E2M_outCSR_curr <= (OTHERS => '0'); -- to propagate the value read from the CSR bank
				E2M_op1_curr <= (OTHERS => '0');
				E2M_op2_curr <= (OTHERS => '0');
				E2M_PC_plus4_curr <= (OTHERS => '0'); -- needed by ecall
				E2M_PC_curr <= (OTHERS => '0');
				E2M_inst_type_curr <= (OTHERS => '0');
				E2M_instr_curr <= (OTHERS => '0');
				E2M_illegal_instruction_flag_curr <= '0';
				E2M_ecall_flag_curr <= '0';
				E2M_ctrl_word_curr <= (OTHERS => '0');
				E2M_ex_flag_curr <= '0';
				E2M_lt_curr <= '0';
				E2M_gt_curr <= '0';
				E2M_eq_curr <= '0';
				E2M_divided_by_zero_flag_curr <= '0';
			ELSIF (E2M_en_def = '1') THEN
				E2M_validAccessCSR_curr <= D2E_validAccessCSR_curr;
				E2M_valid_curr <= D2E_valid_curr;
				E2M_hazard_flag_curr <= E2M_hazard_flag_next;
				E2M_instr_misaligned_flag_curr <= D2E_instr_misaligned_flag_curr;
				E2M_ALU_res_curr <= E2M_ALU_res_next;
				E2M_outCSR_curr <= D2E_outCSR_curr; -- to propagate the value read from the CSR bank
				E2M_op1_curr <= D2E_op1_curr;
				E2M_op2_curr <= D2E_op2_curr;
				E2M_PC_plus4_curr <= D2E_PC_plus4_curr; -- needed by ecall
				E2M_PC_curr <= D2E_PC_curr;
				E2M_inst_type_curr <= D2E_inst_type_curr;
				E2M_instr_curr <= D2E_instr_curr;
				E2M_illegal_instruction_flag_curr <= D2E_illegal_instruction_flag_curr;
				E2M_ecall_flag_curr <= D2E_ecall_flag_curr;
				E2M_ctrl_word_curr <= D2E_ctrl_word_curr;
				E2M_ex_flag_curr <= E2M_ex_flag_next;
				E2M_lt_curr <= E2M_lt_next;
				E2M_gt_curr <= E2M_gt_next;
				E2M_eq_curr <= E2M_eq_next;
				E2M_divided_by_zero_flag_curr <= dividedByZeroFlag;
			END IF;

			-- M2WB pipeline registers
			IF (M2WB_rst_def = '1') THEN
				M2WB_validAccessCSR_curr <= '0';
				M2WB_valid_curr <= '0';
				M2WB_hazard_flag_curr <= '0';
				M2WB_instr_misaligned_flag_curr <= '0';
				M2WB_op1_curr <= (OTHERS => '0');
				M2WB_ALU_res_curr <= (OTHERS => '0');
				M2WB_outCSR_curr <= (OTHERS => '0'); -- to propagate the value read from the CSR bank
				M2WB_PC_plus4_curr <= (OTHERS => '0'); -- needed by ecall
				M2WB_PC_curr <= (OTHERS => '0');
				M2WB_inst_type_curr <= (OTHERS => '0');
				M2WB_instr_curr <= (OTHERS => '0');
				M2WB_illegal_instruction_flag_curr <= '0';
				M2WB_ecall_flag_curr <= '0';
				M2WB_ctrl_word_curr <= (OTHERS => '0');
				M2WB_ex_flag_curr <= '0';
				M2WB_lt_curr <= '0';
				M2WB_gt_curr <= '0';
				M2WB_eq_curr <= '0';
				M2WB_divided_by_zero_flag_curr <= '0';
				M2WB_MEM_res_curr <= (OTHERS => '0');
			ELSIF (M2WB_en = '1') THEN
				M2WB_validAccessCSR_curr <= E2M_validAccessCSR_curr;
				M2WB_valid_curr <= E2M_valid_curr;
				M2WB_hazard_flag_curr <= M2WB_hazard_flag_next;
				M2WB_instr_misaligned_flag_curr <= E2M_instr_misaligned_flag_curr;
				M2WB_op1_curr <= E2M_op1_curr;
				M2WB_ALU_res_curr <= E2M_ALU_res_curr;
				M2WB_outCSR_curr <= E2M_outCSR_curr; -- to propagate the value read from the CSR bank
				M2WB_PC_plus4_curr <= E2M_PC_plus4_curr; -- needed by ecall
				M2WB_PC_curr <= E2M_PC_curr;
				M2WB_inst_type_curr <= E2M_inst_type_curr;
				M2WB_instr_curr <= E2M_instr_curr;
				M2WB_illegal_instruction_flag_curr <= E2M_illegal_instruction_flag_curr;
				M2WB_ecall_flag_curr <= E2M_ecall_flag_curr;
				M2WB_ctrl_word_curr <= E2M_ctrl_word_curr;
				M2WB_ex_flag_curr <= E2M_ex_flag_curr; -- FIXME: TO BE FIXED IF DATA MISALIGNED EXCEPTIONS ARE IMPLEMENTED
				M2WB_lt_curr <= E2M_lt_curr;
				M2WB_gt_curr <= E2M_gt_curr;
				M2WB_eq_curr <= E2M_eq_curr;
				M2WB_divided_by_zero_flag_curr <= E2M_divided_by_zero_flag_curr;
				M2WB_MEM_res_curr <= M2WB_MEM_res_next;
			END IF;

		END IF;
	END PROCESS;

	-- valid bits for each stage of the pipeline
	DEC_valid <= GI2D_valid_curr;
	EX_valid <= D2E_valid_curr;
	M_valid <= E2M_valid_curr;
	WB_valid <= M2WB_valid_curr;

	-- EXCEPTION PRE-HANDLING LOGIC -----------------------------------------------------------------
	-- DONE: FIX IT TO HANDLE MRET AND URET: FOR THESE INSTRUCTIONS THE BRANCH IS PERFORMED DURING WRITE-BACK,
	-- SO WE HAVE TO MAKE SURE THAT NO STORE IS ABLE TO REACH MEMORY (BLOCK THE PIPELINE AS YOU WOULD DO FOR
	-- AN ECALL) --> use the ret_from_epc flag
	
	-- DRIVEN SIGNALS: DARU1_en_def, GI2D_en_def, D2E_en_def, E2M_en_def
	-- inputs: exception flags, rst, GI2D_rst (CU issued reset, when mispredicting or terminating hw exception handling)
	EH_combinational_logic: PROCESS (EH_state_curr, instrMisalignedFlag, illegalInstrFlag, ecallFlag, dividedByZeroFlag,
	 ret_from_epc, DARU1_en, GI2D_en, D2E_en, E2M_en) BEGIN
		EH_state_next <= EH_state_curr;
		DARU1_en_def <= DARU1_en;
		GI2D_en_def <= GI2D_en;
		D2E_en_def <= D2E_en;
		E2M_en_def <= E2M_en;
		CASE EH_state_curr IS
			WHEN no_ex =>
				IF (instrMisalignedFlag = '1') THEN
					DARU1_en_def <= '0';
					EH_state_next <= ex_misaligned_inst;
				END IF;
				IF (illegalInstrFlag = '1' OR ecallFlag = '1' OR ret_from_epc = '1') THEN
					DARU1_en_def <= '0';
					GI2D_en_def <= '0';
					EH_state_next <= ex_ecall_illegal_inst_or_ret;
				END IF;
				IF (dividedByZeroFlag = '1') THEN
					DARU1_en_def <= '0';
					GI2D_en_def <= '0';
					D2E_en_def <= '0';
					EH_state_next <= ex_divided_by_zero;
				END IF;
			WHEN ex_misaligned_inst =>
				DARU1_en_def <= '0';
				IF (illegalInstrFlag = '1' OR ecallFlag = '1' OR ret_from_epc = '1') THEN
					GI2D_en_def <= '0';
					EH_state_next <= ex_ecall_illegal_inst_or_ret;
				END IF;
				IF (dividedByZeroFlag = '1') THEN
					GI2D_en_def <= '0';
					D2E_en_def <= '0';
					EH_state_next <= ex_divided_by_zero;
				END IF;
			WHEN ex_ecall_illegal_inst_or_ret =>
				DARU1_en_def <= '0';
				GI2D_en_def <= '0';
				IF (dividedByZeroFlag = '1') THEN
					D2E_en_def <= '0';
					EH_state_next <= ex_divided_by_zero;
				END IF;
			WHEN ex_divided_by_zero =>
				DARU1_en_def <= '0';
				GI2D_en_def <= '0';
				D2E_en_def <= '0';
		END CASE;
	END PROCESS;

	EH_sequential_logic: PROCESS (clk, rst) BEGIN
		IF (rst = '1') THEN
			EH_state_curr <= no_ex;
		ELSIF (rising_edge(clk)) THEN
			IF(GI2D_rst = '1') THEN
				EH_state_curr <= no_ex;
			ELSE
				EH_state_curr <= EH_state_next;
			END IF;
		END IF;
	END PROCESS;


	-- MIRRORING CIRCUITRY --------------------------------------------------------------------------

	-- CIRCUIT TO CHECK IF THE WRITE-BACK HAS TO REPEATED ON THE MIRRORED VERSION OF THE ORIGINAL REGISTER.
	-- IF THIS HAS TO BE DONE, SET A FLAG TO 1 AND SEND THE FLAG AS instructionDone, OTHERWISE YOU CAN
	-- SEND A ONE DIRECTLY (MUX DRIVEN BY mirror BETWEEN '1' AND flagCurr, WHERE flagNext <= '1' AND THE ENABLE
	-- SIGNAL IS mirror AND M2WB_valid_curr AND THE RESET ONE IS M2WB_en OR M2WB_rst_def) 

	instructionDone <= '1' WHEN (((mirror = '0' OR writeRegBank = '0') AND M2WB_valid_curr = '1') OR M2WB_valid_curr = '0') ELSE instructionDoneCSR;

	-- MC_register: PROCESS (clk, rst) BEGIN
	-- 	IF (rst = '1') THEN
	-- 		MC_flag_curr <= '0';
	-- 	ELSIF (rising_edge(clk)) THEN
	-- 		-- sync reset condition: when the write-back over the mirror completes or the pipeline is reset
	-- 		IF (M2WB_en = '1' OR M2WB_rst_def = '1') THEN
	-- 			MC_flag_curr <= '0';
	-- 		ELSIF (mirror = '1' AND M2WB_valid_curr = '1') THEN
	-- 			MC_flag_curr <= '1';
	-- 		END IF;	
	-- 	END IF;
	-- END PROCESS;

	-- CONTROL WORD AND INSTRUCTION PACKING AND UNPACKING -------------------------------------------
	-- TODO: REMOVE UNNECESSARY SIGNALS

	-- DECODE TO EXECUTE

	D2E_ctrl_word_next(0) 					<= writeRegFile;
	D2E_ctrl_word_next(1) 					<= setZeroOrOne;
	D2E_ctrl_word_next(2) 					<= ComparedSignedUnsignedBar;
	D2E_ctrl_word_next(3) 					<= selPC;
	D2E_ctrl_word_next(4) 					<= selJL;
	D2E_ctrl_word_next(5) 					<= selBSU;
	D2E_ctrl_word_next(6) 					<= selLLU;
	D2E_ctrl_word_next(7) 					<= selASU;
	D2E_ctrl_word_next(8) 					<= selAAU;
	D2E_ctrl_word_next(9) 					<= selP1;
	D2E_ctrl_word_next(10) 					<= selP2;
	D2E_ctrl_word_next(11) 					<= selImm;
	D2E_ctrl_word_next(12) 					<= ldByteSigned;
	D2E_ctrl_word_next(13) 					<= ldHalfSigned;
	D2E_ctrl_word_next(14) 					<= load;
	D2E_ctrl_word_next(16 DOWNTO 15) 		<= selShift; -- 2 bits
	D2E_ctrl_word_next(17) 					<= addSubBar;
	D2E_ctrl_word_next(18) 					<= pass;
	D2E_ctrl_word_next(19) 					<= selAuipc;
	D2E_ctrl_word_next(31 DOWNTO 20) 		<= muxCode; -- 12 bits
	D2E_ctrl_word_next(33 DOWNTO 32) 		<= selLogic; -- 2 bits
	D2E_ctrl_word_next(34) 					<= startDAWU;
	D2E_ctrl_word_next(35) 					<= startDARU;
	D2E_ctrl_word_next(36) 					<= startMultiplyAAU;
	D2E_ctrl_word_next(37) 					<= startDivideAAU;
	D2E_ctrl_word_next(38) 					<= signedSigned;
	D2E_ctrl_word_next(39) 					<= signedUnsigned;
	D2E_ctrl_word_next(40) 					<= unsignedUnsigned;
	D2E_ctrl_word_next(41) 					<= selAAL;
	D2E_ctrl_word_next(42) 					<= selAAH;
	D2E_ctrl_word_next(44 DOWNTO 43) 		<= nBytes; -- 2 bits
	D2E_ctrl_word_next(45) 					<= selCSR;
	D2E_ctrl_word_next(46) 					<= writeRB_inst;
	D2E_ctrl_word_next(47) 					<= checkMisalignedDAWU;
	D2E_ctrl_word_next(48) 					<= selCSRAddrFromInst;
	D2E_ctrl_word_next(49) 					<= forced_RB_read;
	D2E_ctrl_word_next(52 DOWNTO 50) 		<= inst_type; -- 3 bits
	D2E_ctrl_word_next(53) 					<= ret_from_epc;
	D2E_ctrl_word_next(54) 					<= selALU;
	D2E_ctrl_word_next(55) 					<= selPC4;
	D2E_ctrl_word_next(56) 					<= selMem;
	D2E_ctrl_word_next(57) 					<= cmp_selALUop2;
	D2E_ctrl_word_next(58) 					<= cmp_selop2;
	D2E_ctrl_word_next(59) 					<= isCSRInstruction;
	-- instruction fields
	func3           						<= GI2D_instr_curr(14 DOWNTO 12);
	func7           						<= GI2D_instr_curr(31 DOWNTO 25);
	func12          						<= GI2D_instr_curr(31 DOWNTO 20);
	opcode          						<= GI2D_instr_curr(6 DOWNTO 0);

	-- EXECUTE

	E_writeRegFile							<= D2E_ctrl_word_curr(0);
	E_setZeroOrOne							<= D2E_ctrl_word_curr(1);
	E_ComparedSignedUnsignedBar				<= D2E_ctrl_word_curr(2);
	E_selPC									<= D2E_ctrl_word_curr(3);
	E_selJL									<= D2E_ctrl_word_curr(4);
	E_selBSU								<= D2E_ctrl_word_curr(5);
	E_selLLU								<= D2E_ctrl_word_curr(6);
	E_selASU								<= D2E_ctrl_word_curr(7);
	E_selAAU								<= D2E_ctrl_word_curr(8);
	E_selP1									<= D2E_ctrl_word_curr(9);
	E_selP2									<= D2E_ctrl_word_curr(10);
	E_selImm								<= D2E_ctrl_word_curr(11);
	E_ldByteSigned							<= D2E_ctrl_word_curr(12);
	E_ldHalfSigned							<= D2E_ctrl_word_curr(13);
	E_load									<= D2E_ctrl_word_curr(14);
	E_selShift								<= D2E_ctrl_word_curr(16 DOWNTO 15); -- 2 bits
	E_addSubBar								<= D2E_ctrl_word_curr(17);
	E_pass									<= D2E_ctrl_word_curr(18);
	E_selAuipc								<= D2E_ctrl_word_curr(19);
	E_muxCode								<= D2E_ctrl_word_curr(31 DOWNTO 20); -- 12 bits
	E_selLogic								<= D2E_ctrl_word_curr(33 DOWNTO 32); -- 2 bits
	E_startDAWU								<= D2E_ctrl_word_curr(34);
	E_startDARU								<= D2E_ctrl_word_curr(35);
	E_startMultiplyAAU						<= D2E_ctrl_word_curr(36);
	E_startDivideAAU						<= D2E_ctrl_word_curr(37);
	E_signedSigned							<= D2E_ctrl_word_curr(38);
	E_signedUnsigned						<= D2E_ctrl_word_curr(39);
	E_unsignedUnsigned						<= D2E_ctrl_word_curr(40);
	E_selAAL								<= D2E_ctrl_word_curr(41);
	E_selAAH								<= D2E_ctrl_word_curr(42);
	E_nBytes								<= D2E_ctrl_word_curr(44 DOWNTO 43); -- 2 bits
	E_selCSR								<= D2E_ctrl_word_curr(45);
	E_writeRB_inst							<= D2E_ctrl_word_curr(46);
	E_checkMisalignedDAWU					<= D2E_ctrl_word_curr(47);
	E_selCSRAddrFromInst					<= D2E_ctrl_word_curr(48);
	E_forced_RB_read						<= D2E_ctrl_word_curr(49);
	E_inst_type								<= D2E_ctrl_word_curr(52 DOWNTO 50); -- 3 bits
	E_ret_from_epc							<= D2E_ctrl_word_curr(53);
	E_selALU								<= D2E_ctrl_word_curr(54);
	E_selPC4								<= D2E_ctrl_word_curr(55);
	E_selMem								<= D2E_ctrl_word_curr(56);
	E_cmp_selALUop2							<= D2E_ctrl_word_curr(57);
	E_cmp_selop2							<= D2E_ctrl_word_curr(58);
	E_isCSRInstruction						<= D2E_ctrl_word_curr(59);
	-- instruction fields
	E_func3           						<= D2E_instr_curr(14 DOWNTO 12);
	E_func7           						<= D2E_instr_curr(31 DOWNTO 25);
	E_func12          						<= D2E_instr_curr(31 DOWNTO 20);
	E_opcode          						<= D2E_instr_curr(6 DOWNTO 0);

	-- MEMORY

	M_writeRegFile							<= E2M_ctrl_word_curr(0);
	M_setZeroOrOne							<= E2M_ctrl_word_curr(1);
	M_ComparedSignedUnsignedBar				<= E2M_ctrl_word_curr(2);
	M_selPC									<= E2M_ctrl_word_curr(3);
	M_selJL									<= E2M_ctrl_word_curr(4);
	M_selBSU								<= E2M_ctrl_word_curr(5);
	M_selLLU								<= E2M_ctrl_word_curr(6);
	M_selASU								<= E2M_ctrl_word_curr(7);
	M_selAAU								<= E2M_ctrl_word_curr(8);
	M_selP1									<= E2M_ctrl_word_curr(9);
	M_selP2									<= E2M_ctrl_word_curr(10);
	M_selImm								<= E2M_ctrl_word_curr(11);
	M_ldByteSigned							<= E2M_ctrl_word_curr(12);
	M_ldHalfSigned							<= E2M_ctrl_word_curr(13);
	M_load									<= E2M_ctrl_word_curr(14);
	M_selShift								<= E2M_ctrl_word_curr(16 DOWNTO 15); -- 2 bits
	M_addSubBar								<= E2M_ctrl_word_curr(17);
	M_pass									<= E2M_ctrl_word_curr(18);
	M_selAuipc								<= E2M_ctrl_word_curr(19);
	M_muxCode								<= E2M_ctrl_word_curr(31 DOWNTO 20); -- 12 bits
	M_selLogic								<= E2M_ctrl_word_curr(33 DOWNTO 32); -- 2 bits
	M_startDAWU								<= E2M_ctrl_word_curr(34);
	M_startDARU								<= E2M_ctrl_word_curr(35);
	M_startMultiplyAAU						<= E2M_ctrl_word_curr(36);
	M_startDivideAAU						<= E2M_ctrl_word_curr(37);
	M_signedSigned							<= E2M_ctrl_word_curr(38);
	M_signedUnsigned						<= E2M_ctrl_word_curr(39);
	M_unsignedUnsigned						<= E2M_ctrl_word_curr(40);
	M_selAAL								<= E2M_ctrl_word_curr(41);
	M_selAAH								<= E2M_ctrl_word_curr(42);
	M_nBytes								<= E2M_ctrl_word_curr(44 DOWNTO 43); -- 2 bits
	M_selCSR								<= E2M_ctrl_word_curr(45);
	M_writeRB_inst							<= E2M_ctrl_word_curr(46);
	M_checkMisalignedDAWU					<= E2M_ctrl_word_curr(47);
	M_selCSRAddrFromInst					<= E2M_ctrl_word_curr(48);
	M_forced_RB_read						<= E2M_ctrl_word_curr(49);
	M_inst_type								<= E2M_ctrl_word_curr(52 DOWNTO 50); -- 3 bits
	M_ret_from_epc							<= E2M_ctrl_word_curr(53);
	M_selALU								<= E2M_ctrl_word_curr(54);
	M_selPC4								<= E2M_ctrl_word_curr(55);
	M_selMem								<= E2M_ctrl_word_curr(56);
	M_cmp_selALUop2							<= E2M_ctrl_word_curr(57);
	M_cmp_selop2							<= E2M_ctrl_word_curr(58);
	M_isCSRInstruction						<= E2M_ctrl_word_curr(59);
	-- instruction fields
	M_func3           						<= E2M_instr_curr(14 DOWNTO 12);
	M_func7           						<= E2M_instr_curr(31 DOWNTO 25);
	M_func12          						<= E2M_instr_curr(31 DOWNTO 20);
	M_opcode          						<= E2M_instr_curr(6 DOWNTO 0);

	-- WRITE-BACK

	WB_writeRegFile							<= M2WB_ctrl_word_curr(0);
	WB_setZeroOrOne							<= M2WB_ctrl_word_curr(1);
	WB_ComparedSignedUnsignedBar			<= M2WB_ctrl_word_curr(2);
	WB_selPC								<= M2WB_ctrl_word_curr(3);
	WB_selJL								<= M2WB_ctrl_word_curr(4);
	WB_selBSU								<= M2WB_ctrl_word_curr(5);
	WB_selLLU								<= M2WB_ctrl_word_curr(6);
	WB_selASU								<= M2WB_ctrl_word_curr(7);
	WB_selAAU								<= M2WB_ctrl_word_curr(8);
	WB_selP1								<= M2WB_ctrl_word_curr(9);
	WB_selP2								<= M2WB_ctrl_word_curr(10);
	WB_selImm								<= M2WB_ctrl_word_curr(11);
	WB_ldByteSigned							<= M2WB_ctrl_word_curr(12);
	WB_ldHalfSigned							<= M2WB_ctrl_word_curr(13);
	WB_load									<= M2WB_ctrl_word_curr(14);
	WB_selShift								<= M2WB_ctrl_word_curr(16 DOWNTO 15); -- 2 bits
	WB_addSubBar							<= M2WB_ctrl_word_curr(17);
	WB_pass									<= M2WB_ctrl_word_curr(18);
	WB_selAuipc								<= M2WB_ctrl_word_curr(19);
	WB_muxCode								<= M2WB_ctrl_word_curr(31 DOWNTO 20); -- 12 bits
	WB_selLogic								<= M2WB_ctrl_word_curr(33 DOWNTO 32); -- 2 bits
	WB_startDAWU							<= M2WB_ctrl_word_curr(34);
	WB_startDARU							<= M2WB_ctrl_word_curr(35);
	WB_startMultiplyAAU						<= M2WB_ctrl_word_curr(36);
	WB_startDivideAAU						<= M2WB_ctrl_word_curr(37);
	WB_signedSigned							<= M2WB_ctrl_word_curr(38);
	WB_signedUnsigned						<= M2WB_ctrl_word_curr(39);
	WB_unsignedUnsigned						<= M2WB_ctrl_word_curr(40);
	WB_selAAL								<= M2WB_ctrl_word_curr(41);
	WB_selAAH								<= M2WB_ctrl_word_curr(42);
	WB_nBytes								<= M2WB_ctrl_word_curr(44 DOWNTO 43); -- 2 bits
	WB_selCSR								<= M2WB_ctrl_word_curr(45);
	WB_writeRB_inst							<= M2WB_ctrl_word_curr(46);
	WB_checkMisalignedDAWU					<= M2WB_ctrl_word_curr(47);
	WB_selCSRAddrFromInst					<= M2WB_ctrl_word_curr(48);
	WB_forced_RB_read						<= M2WB_ctrl_word_curr(49);
	WB_inst_type							<= M2WB_ctrl_word_curr(52 DOWNTO 50); -- 3 bits
	WB_ret_from_epc							<= M2WB_ctrl_word_curr(53);
	WB_selALU								<= M2WB_ctrl_word_curr(54);
	WB_selPC4								<= M2WB_ctrl_word_curr(55);
	WB_selMem								<= M2WB_ctrl_word_curr(56);
	WB_cmp_selALUop2						<= M2WB_ctrl_word_curr(57);
	WB_cmp_selop2							<= M2WB_ctrl_word_curr(58);
	WB_isCSRInstruction						<= M2WB_ctrl_word_curr(59);
	-- instruction fields
	WB_func3           						<= M2WB_instr_curr(14 DOWNTO 12);
	WB_func7           						<= M2WB_instr_curr(31 DOWNTO 25);
	WB_func12          						<= M2WB_instr_curr(31 DOWNTO 20);
	WB_opcode          						<= M2WB_instr_curr(6 DOWNTO 0);

	-- WASTELAND ------------------------------------------------------------------------------------

	-- IR             <= inst;

	-- NOT NEEDED: DIRECTLY EMBEDDED IN THE PIPELINE
	-- regIR : ENTITY WORK.aftab_register
	-- 	GENERIC
	-- 	MAP(len => len)
	-- 	PORT
	-- 	MAP(
	-- 	clk    => clk,
	-- 	rst    => rst,
	-- 	zero   => zeroIR,
	-- 	load   => ldIR,
	-- 	inReg  => dataDARU,
	-- 	outReg => inst);

	-- NOT NEEDED: THE COMPUTATION OF THE ADDRESS IS DONE DIRECTLY IN EXECUTE THROUGH THE ADDER IN THE DATAPATH
	-- adder : ENTITY WORK.aftab_adder
	-- 	GENERIC
	-- 	MAP(len => len)
	-- 	PORT
	-- 	MAP(
	-- 	Cin       => '0',
	-- 	A         => immediate,
	-- 	B         => outMux2,
	-- 	addResult => addResult,
	-- 	carryOut  => OPEN);
		
	-- NOT NEEDED: AS IR
	-- regADR : ENTITY WORK.aftab_register
	-- 	GENERIC
	-- 	MAP(len => len)
	-- 	PORT
	-- 	MAP(
	-- 	clk    => clk,
	-- 	rst    => rst,
	-- 	zero   => zeroADR,
	-- 	load   => ldADR,
	-- 	inReg  => addResult,
	-- 	outReg => outADR);

	-- NOT NEEDED: EMBEDDED IN THE PIPELINE
	-- regDR : ENTITY WORK.aftab_register
	-- 	GENERIC
	-- 	MAP(len => len)
	-- 	PORT
	-- 	MAP(
	-- 	clk    => clk,
	-- 	rst    => rst,
	-- 	zero   => zeroDR,
	-- 	load   => ldDR,
	-- 	inReg  => p2,
	-- 	outReg => dataDAWU);
	
	-- NOT NEEDED: THERE ARE TWO DARUS, SO THERE IS NO NEED TO DISTINGUISH BETWEEN THE ACCES DONE IN FETCH AND THE ONE DONE IN MEMORY
	-- mux3 : ENTITY WORK.aftab_multiplexer
	-- 		GENERIC MAP(len => len)
	-- 		PORT MAP(
	-- 			a => outADR,
	-- 			b => outMux2,
	-- 			s0 => selADR, 
	-- 			s1 => selPCJ, 
	-- 			w => addrIn);
	
		
END ARCHITECTURE behavioral;
