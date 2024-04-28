-- **************************************************************************************
--	Filename:	aftab_core.vhd
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
-- File content description:
-- Top entity of the AFTAB core
--
-- **************************************************************************************

LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
ENTITY aftab_core IS
	GENERIC
		(len : INTEGER := 32);
	PORT
	(
		clk                      : IN  STD_LOGIC;
		rst                      : IN  STD_LOGIC;
		memReady1       	     : IN  STD_LOGIC;
		memReady2       	     : IN  STD_LOGIC;
		memDataOut1              : IN  STD_LOGIC_VECTOR (15 DOWNTO 0);
		memDataOut2              : IN STD_LOGIC_VECTOR (15 DOWNTO 0);
		memDataIn2               : OUT STD_LOGIC_VECTOR (15 DOWNTO 0);
		memRead1                 : OUT STD_LOGIC;
		memRead2                 : OUT STD_LOGIC;
		memWrite                 : OUT STD_LOGIC;
		memAddr1                 : OUT STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
		memAddr2                 : OUT STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
		bytesPort1				 : OUT STD_LOGIC;
		bytesPort2				 : OUT STD_LOGIC;
		--interrupt inputs and outputs
		machineExternalInterrupt : IN  STD_LOGIC;
		machineTimerInterrupt    : IN  STD_LOGIC;
		machineSoftwareInterrupt : IN  STD_LOGIC;
		userExternalInterrupt    : IN  STD_LOGIC;
		userTimerInterrupt       : IN  STD_LOGIC;
		userSoftwareInterrupt    : IN  STD_LOGIC;
		platformInterruptSignals : IN  STD_LOGIC_VECTOR (15 DOWNTO 0);
		interruptProcessing      : OUT STD_LOGIC
	);
END ENTITY;
--
ARCHITECTURE procedural OF aftab_core IS
	SIGNAL selPC                          : STD_LOGIC;
	SIGNAL selI4                          : STD_LOGIC;
	SIGNAL selP2                          : STD_LOGIC;
	SIGNAL selP1                          : STD_LOGIC;
	SIGNAL selJL                          : STD_LOGIC;
	SIGNAL selADR                         : STD_LOGIC;
	SIGNAL selPCJ                         : STD_LOGIC;
	SIGNAL selImm                         : STD_LOGIC;
	SIGNAL selAdd                         : STD_LOGIC;
	SIGNAL selI4PC                        : STD_LOGIC;
	SIGNAL selInc4pc                      : STD_LOGIC;
	SIGNAL selData                        : STD_LOGIC;
	SIGNAL selBSU                         : STD_LOGIC;
	SIGNAL selLLU                         : STD_LOGIC;
	SIGNAL selDARU                        : STD_LOGIC;
	SIGNAL selASU                         : STD_LOGIC;
	SIGNAL selAAU                         : STD_LOGIC;
	SIGNAL shr                            : STD_LOGIC;
	SIGNAL shl                            : STD_LOGIC;
	SIGNAL dataInstrBar                   : STD_LOGIC;
	SIGNAL writeRegFile                   : STD_LOGIC;
	SIGNAL addSubBar                      : STD_LOGIC;
	SIGNAL pass                           : STD_LOGIC;
	SIGNAL selAuipc                       : STD_LOGIC;
	SIGNAL comparedsignedunsignedbar      : STD_LOGIC;
	SIGNAL ldIR                           : STD_LOGIC;
	SIGNAL ldADR                          : STD_LOGIC;
	SIGNAL ldPC                           : STD_LOGIC;
	SIGNAL ldDr                           : STD_LOGIC;
	SIGNAL ldByteSigned                   : STD_LOGIC;
	SIGNAL ldHalfSigned                   : STD_LOGIC;
	SIGNAL load                           : STD_LOGIC;
	SIGNAL setOne                         : STD_LOGIC;
	SIGNAL setZero                        : STD_LOGIC;
	SIGNAL startDARU                      : STD_LOGIC;
	SIGNAL startDAWU                      : STD_LOGIC;
	SIGNAL completeDARU                   : STD_LOGIC;
	SIGNAL completeDAWU                   : STD_LOGIC;
	SIGNAL startMultiplyAAU               : STD_LOGIC;
	SIGNAL startDivideAAU                 : STD_LOGIC;
	SIGNAL completeAAU                    : STD_LOGIC;
	SIGNAL signedSigned                   : STD_LOGIC;
	SIGNAL signedUnsigned                 : STD_LOGIC;
	SIGNAL unsignedUnsigned               : STD_LOGIC;
	SIGNAL selAAL                         : STD_LOGIC;
	SIGNAL selAAH                         : STD_LOGIC;
	SIGNAL eq                             : STD_LOGIC;
	SIGNAL gt                             : STD_LOGIC;
	SIGNAL lt                             : STD_LOGIC;
	SIGNAL dataerror                      : STD_LOGIC;
	SIGNAL nBytes                         : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL selLogic                       : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL selShift                       : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL muxCode                        : STD_LOGIC_VECTOR (11 DOWNTO 0);
	SIGNAL modeTvec                       : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL previousPRV                    : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL delegationMode                 : STD_LOGIC_VECTOR (1 DOWNTO 0);
	SIGNAL IR                             : STD_LOGIC_VECTOR (31 DOWNTO 0);
	SIGNAL selCSR                         : STD_LOGIC;
	SIGNAL interruptRaise                 : STD_LOGIC;
	SIGNAL mipCCLdDisable                 : STD_LOGIC;
	SIGNAL ldValueCSR                     : STD_LOGIC_VECTOR(2 DOWNTO 0);
	SIGNAL selImmCSR                      : STD_LOGIC;
	SIGNAL selReadWriteCSR                : STD_LOGIC;
	SIGNAL selP1CSR                       : STD_LOGIC;
	SIGNAL clrCSR                         : STD_LOGIC;
	SIGNAL setCSR                         : STD_LOGIC;
	SIGNAL selPC_CSR                      : STD_LOGIC;
	SIGNAL selCCMip_CSR                   : STD_LOGIC;
	SIGNAL selCause_CSR                   : STD_LOGIC;
	SIGNAL selMepc_CSR                    : STD_LOGIC;
	SIGNAL machineStatusAlterationPreCSR  : STD_LOGIC;
	SIGNAL userStatusAlterationPreCSR     : STD_LOGIC;
	SIGNAL machineStatusAlterationPostCSR : STD_LOGIC;
	SIGNAL userStatusAlterationPostCSR    : STD_LOGIC;
	SIGNAL writeRegBank                   : STD_LOGIC;
	SIGNAL dnCntCSR                       : STD_LOGIC;
	SIGNAL upCntCSR                       : STD_LOGIC;
	SIGNAL ldCntCSR                       : STD_LOGIC;
	SIGNAL zeroCntCSR                     : STD_LOGIC;
	SIGNAL ldFlags                        : STD_LOGIC;
	SIGNAL zeroFlags                      : STD_LOGIC;
	SIGNAL ldDelegation                   : STD_LOGIC;
	SIGNAL ldMachine                      : STD_LOGIC;
	SIGNAL ldUser                         : STD_LOGIC;
	SIGNAL loadMieReg                     : STD_LOGIC;
	SIGNAL loadMieUieField                : STD_LOGIC;
	SIGNAL mirrorUser                     : STD_LOGIC;
	SIGNAL selCSRAddrFromInst             : STD_LOGIC;
	SIGNAL selRomAddress                  : STD_LOGIC;
	SIGNAL validAccessCSR                 : STD_LOGIC;
	SIGNAL readOnlyCSR                    : STD_LOGIC;
	SIGNAL selInterruptAddressDirect      : STD_LOGIC;
	SIGNAL selInterruptAddressVectored    : STD_LOGIC;
	SIGNAL ecallFlag                      : STD_LOGIC;
	SIGNAL illegalInstrFlag               : STD_LOGIC;
	SIGNAL instrMisalignedOut             : STD_LOGIC;
	SIGNAL loadMisalignedOut              : STD_LOGIC;
	SIGNAL storeMisalignedOut             : STD_LOGIC;
	SIGNAL selTval_CSR                    : STD_LOGIC;
	SIGNAL exceptionRaise                 : STD_LOGIC;
	SIGNAL checkMisalignedDARU            : STD_LOGIC;
	SIGNAL checkMisalignedDAWU            : STD_LOGIC;
	SIGNAL dividedByZeroOut               : STD_LOGIC;
	SIGNAL mirror                         : STD_LOGIC;
	SIGNAL ldMieReg                       : STD_LOGIC;
	SIGNAL ldMieUieField                  : STD_LOGIC;
	SIGNAL selMedeleg_CSR                 : STD_LOGIC;
	SIGNAL selMideleg_CSR                 : STD_LOGIC;
	SIGNAL setZeroOrOne					  : STD_LOGIC;
	SIGNAL writeRB_inst					  : STD_LOGIC;
	SIGNAL forced_RB_read				  : STD_LOGIC;
	SIGNAL inst_type					  : STD_LOGIC_VECTOR (2 DOWNTO 0);
	SIGNAL ret_from_epc					  : STD_LOGIC;
	SIGNAL selALU						  : STD_LOGIC;
	SIGNAL selPC4						  : STD_LOGIC;
	SIGNAL selMem						  : STD_LOGIC;
	SIGNAL cmp_selALUop2				  : STD_LOGIC;
	SIGNAL cmp_selop2					  : STD_LOGIC;
	SIGNAL isCSRInstruction				  : STD_LOGIC;
	SIGNAL WB_func3						  : STD_LOGIC_VECTOR (2 DOWNTO 0);
	SIGNAL completedDAWU				  : STD_LOGIC;
	SIGNAL completedDARU1				  : STD_LOGIC;
	SIGNAL completedDARU2				  : STD_LOGIC;
	SIGNAL completedAAU					  : STD_LOGIC;
	SIGNAL is_AAU_used					  : STD_LOGIC;
	SIGNAL instructionDone				  : STD_LOGIC;
	SIGNAL hazard_solved				  : STD_LOGIC;
	SIGNAL is_store_in_mem				  : STD_LOGIC;
	SIGNAL is_load_in_mem				  : STD_LOGIC;
	SIGNAL branch_taken					  : STD_LOGIC;
	SIGNAL DEC_valid					  : STD_LOGIC;
	SIGNAL EX_valid						  : STD_LOGIC;
	SIGNAL M_valid						  : STD_LOGIC;
	SIGNAL WB_valid						  : STD_LOGIC;
	SIGNAL WB_ret_from_epc				  : STD_LOGIC;
	SIGNAL CSR_from_WB					  : STD_LOGIC;
	SIGNAL sel_Tval_CSR					  : STD_LOGIC;
	SIGNAL mirrorUserCU					  : STD_LOGIC;
	SIGNAL instructionDoneCSR			  : STD_LOGIC;
	SIGNAL GI2D_en						  : STD_LOGIC;	
	SIGNAL GI2D_rst						  : STD_LOGIC;	
	SIGNAL D2E_en						  : STD_LOGIC;	
	SIGNAL D2E_rst						  : STD_LOGIC;	
	SIGNAL E2M_en						  : STD_LOGIC;	
	SIGNAL E2M_rst						  : STD_LOGIC;	
	SIGNAL M2WB_en						  : STD_LOGIC;	
	SIGNAL M2WB_rst						  : STD_LOGIC;
	SIGNAL hazEX						  : STD_LOGIC;	
	SIGNAL hazM							  : STD_LOGIC;	
	SIGNAL WB_isCSRInstruction			  : STD_LOGIC;
	SIGNAL WB_validAccessCSR			  : STD_LOGIC;

BEGIN
	datapathAFTAB : ENTITY WORK.aftab_datapath
		PORT MAP
		(
			-- general signals
			clk                            => clk,
			rst                            => rst,

			-- control word
			writeRegFile                   => writeRegFile,
			setZeroOrOne                   => setZeroOrOne,
			ComparedSignedUnsignedBar      => ComparedSignedUnsignedBar,
			selPC                          => selPC,
			selJL                          => selJL,
			selBSU                         => selBSU,
			selLLU                         => selLLU,
			selASU                         => selASU,
			selAAU                         => selAAU,
			selP1                          => selP1,
			selP2                          => selP2,
			selImm                         => selImm,
			ldByteSigned                   => ldByteSigned,
			ldHalfSigned                   => ldHalfSigned,
			load                           => load,
			selShift                       => selShift,
			addSubBar                      => addSubBar,
			pass                           => pass,
			selAuipc                       => selAuipc,
			muxCode                        => muxCode,
			selLogic                       => selLogic,
			startDAWU                      => startDAWU,
			startDARU                 	   => startDARU,
			startMultiplyAAU               => startMultiplyAAU,
			startDivideAAU                 => startDivideAAU,
			signedSigned                   => signedSigned,
			signedUnsigned                 => signedUnsigned,
			unsignedUnsigned               => unsignedUnsigned,
			selAAL                         => selAAL,
			selAAH                         => selAAH,
			nBytes                         => nBytes,
			selCSR                         => selCSR,
			writeRB_inst				   => writeRB_inst, -- TODO: to be removed
			checkMisalignedDAWU            => checkMisalignedDAWU,
			selCSRAddrFromInst             => selCSRAddrFromInst,
			forced_RB_read				   => forced_RB_read, -- mux selection signal between the address to be read from write-back (as exception handling) and the one to be read in decode
			inst_type					   => inst_type,
			ret_from_epc				   => ret_from_epc,
			selALU						   => selALU,
			selPC4						   => selPC4,
			selMem						   => selMem,
			cmp_selALUop2				   => cmp_selALUop2,
			cmp_selop2				   	   => cmp_selop2,
			isCSRInstruction			   => isCSRInstruction,

			-- memory signals
			writeMemDAWU                   => memWrite,
			readMemDARU1                   => memRead1,
			readMemDARU2                   => memRead2,
			memReady1                      => memReady1,  -- for the two read ports (one for GI and one for MEM)
			memReady2                      => memReady2,
			memDataOut1                    => memDataOut1, -- data read from the first read port
			memDataOut2                    => memDataOut2,
			dataDAWU                       => memDataIn2,
			memAddr1                       => memAddr1,
			memAddr2                       => memAddr2,
			bytesToReadDARU1			   => bytesPort1, -- for the first memory port (accessed only from GI stage)
			bytesPerMemAccess			   => bytesPort2, -- for the second memory port
			
			-- instruction to be decoded
			IR                             => IR, -- used to send to the CU the instruction to be decoded
			
			-- func3 field of the instruction currently in write-back
			WB_func3					   => WB_func3,

			-- operation complete signals (and related signals to notify the CU about the instruction which reside in the pipeline)
			completedDAWU_def              => completedDAWU,
			completedDARU1_def             => completedDARU1,
			completedDARU2_def             => completedDARU2,
			completedAAU                   => completedAAU,
			is_AAU_used					   => is_AAU_used,
			instructionDone				   => instructionDone,
			hazard_solved 				   => hazard_solved,
			is_store_in_mem				   => is_store_in_mem,
			is_load_in_mem				   => is_load_in_mem,
			branch_taken 				   => branch_taken,
			DEC_valid					   => DEC_valid,
			EX_valid					   => EX_valid,
			M_valid						   => M_valid,
			WB_valid					   => WB_valid,
			WB_ret_from_epc				   => WB_ret_from_epc,
			WB_isCSRInstruction			   => WB_isCSRInstruction,
			WB_validAccessCSR			   => WB_validAccessCSR,

			--CSR and Interrupt inputs and outputs --> driven directly by the CU when needed
			CSR_from_WB					   => CSR_from_WB,
			machineExternalInterrupt       => machineExternalInterrupt,
			machineTimerInterrupt          => machineTimerInterrupt,
			machineSoftwareInterrupt       => machineSoftwareInterrupt,
			userExternalInterrupt          => userExternalInterrupt,
			userTimerInterrupt             => userTimerInterrupt,
			userSoftwareInterrupt          => userSoftwareInterrupt,
			platformInterruptSignals       => platformInterruptSignals,
			ldValueCSR                     => ldValueCSR,
			mipCCLdDisable                 => mipCCLdDisable,
			selPC_CSR                      => selPC_CSR,
			selTval_CSR                    => sel_Tval_CSR,
			selMedeleg_CSR                 => selMedeleg_CSR,
			selMideleg_CSR                 => selMideleg_CSR,
			selCCMip_CSR                   => selCCMip_CSR,
			selCause_CSR                   => selCause_CSR,
			selMepc_CSR                    => selMepc_CSR,
			selInterruptAddressDirect      => selInterruptAddressDirect,
			selInterruptAddressVectored    => selInterruptAddressVectored,
			writeRegBank                   => writeRegBank, -- driven directly by the CU, the actual write enable signal is writeRegBank OR write_RB_inst
			dnCntCSR                       => dnCntCSR,
			upCntCSR                       => upCntCSR,
			ldCntCSR                       => ldCntCSR,
			zeroCntCSR                     => zeroCntCSR,
			ldDelegation                   => ldDelegation,
			ldMachine                      => ldMachine,
			ldUser                         => ldUser,
			loadMieReg                     => loadMieReg,
			loadMieUieField                => loadMieUieField,
			mirrorUserCU                   => mirrorUserCU,
			machineStatusAlterationPreCSR  => machineStatusAlterationPreCSR,
			userStatusAlterationPreCSR     => userStatusAlterationPreCSR,
			machineStatusAlterationPostCSR => machineStatusAlterationPostCSR,
			userStatusAlterationPostCSR    => userStatusAlterationPostCSR,
			selRomAddress                  => selRomAddress,
			ecallFlag                      => ecallFlag, -- set by the CU when an ecall is in decode
			illegalInstrFlag               => illegalInstrFlag,
			instructionDoneCSR			   => instructionDoneCSR,
			validAccessCSR                 => validAccessCSR,
			readOnlyCSR                    => readOnlyCSR,
			mirror                         => mirror,
			ldMieReg                       => ldMieReg,
			ldMieUieField                  => ldMieUieField,
			interruptRaise                 => interruptRaise,
			exceptionRaise                 => exceptionRaise,
			delegationMode                 => delegationMode,
			previousPRV                    => previousPRV,
			modeTvec                       => modeTvec,
			selP1CSR                       => selP1CSR,
			selImmCSR                      => selImmCSR,
			setCSR                         => setCSR,
			selReadWriteCSR                => selReadWriteCSR,
			clrCSR                         => clrCSR,

			-- pipeline registers
			GI2D_en						   => GI2D_en,
			GI2D_rst					   => GI2D_rst,
			D2E_en						   => D2E_en,
			D2E_rst					       => D2E_rst,
			E2M_en						   => E2M_en,
			E2M_rst						   => E2M_rst,
			M2WB_en						   => M2WB_en,
			M2WB_rst					   => M2WB_rst,

			-- hazards
			hazEX						   => hazEX,
			hazM						   => hazM
		);
	controllerAFTAB : ENTITY WORK.aftab_controller
		PORT MAP(
			-- general signals
			clk                            => clk,
			rst                            => rst,

			-- operation complete signals
			completedDAWU                  => completedDAWU,
			completedDARU1                 => completedDARU1,
			completedDARU2                 => completedDARU2,
			completedAAU                   => completedAAU,
			is_AAU_used					   => is_AAU_used,
			instructionDone				   => instructionDone,
			hazard_solved 				   => hazard_solved,
			is_store_in_mem				   => is_store_in_mem,
			is_load_in_mem				   => is_load_in_mem,
			branch_taken 				   => branch_taken,
			DEC_valid					   => DEC_valid,
			EX_valid					   => EX_valid,
			M_valid						   => M_valid,
			WB_valid					   => WB_valid,
			WB_ret_from_epc				   => WB_ret_from_epc,
			WB_isCSRInstruction			   => WB_isCSRInstruction,
			WB_validAccessCSR			   => WB_validAccessCSR,

			-- instruction to be decoded
			IR                             => IR,

			-- func3 field of the instruction currently in write-back (needed for CSR instructions)
			WB_func3					   => WB_func3,
			
			-- control word
			writeRegFile                   => writeRegFile,
			setZeroOrOne                   => setZeroOrOne,
			ComparedSignedUnsignedBar      => ComparedSignedUnsignedBar,
			selPC                          => selPC,
			selJL                          => selJL,
			selBSU                         => selBSU,
			selLLU                         => selLLU,
			selASU                         => selASU,
			selAAU                         => selAAU,
			selP1                          => selP1,
			selP2                          => selP2,
			selImm                         => selImm,
			ldByteSigned                   => ldByteSigned,
			ldHalfSigned                   => ldHalfSigned,
			load                           => load,
			selShift                       => selShift,
			addSubBar                      => addSubBar,
			pass                           => pass,
			selAuipc                       => selAuipc,
			muxCode                        => muxCode,
			selLogic                       => selLogic,
			startDAWU                      => startDAWU,
			startDARU                 	   => startDARU,
			startMultiplyAAU               => startMultiplyAAU,
			startDivideAAU                 => startDivideAAU,
			signedSigned                   => signedSigned,
			signedUnsigned                 => signedUnsigned,
			unsignedUnsigned               => unsignedUnsigned,
			selAAL                         => selAAL,
			selAAH                         => selAAH,
			nBytes                         => nBytes,
			selCSR                         => selCSR,
			selImmCSR                      => selImmCSR,
			selP1CSR                       => selP1CSR,
			selReadWriteCSR                => selReadWriteCSR,
			clrCSR                         => clrCSR,
			setCSR                         => setCSR,
			writeRB_inst				   => writeRB_inst, -- the signal which has to be set to perform a write op over the RB
			checkMisalignedDAWU            => checkMisalignedDAWU,
			selCSRAddrFromInst             => selCSRAddrFromInst,
			forced_RB_read				   => forced_RB_read, -- mux selection signal between the address to be read from write-back (as exception handling) and the one to be read in decode
			inst_type					   => inst_type,
			ret_from_epc				   => ret_from_epc,
			selALU						   => selALU,
			selPC4						   => selPC4,
			selMem						   => selMem,
			cmp_selALUop2				   => cmp_selALUop2,
			cmp_selop2				   	   => cmp_selop2,
			isCSRInstruction			   => isCSRInstruction,

			-- Interrupts
			CSR_from_WB					   => CSR_from_WB,
			interruptRaise                 => interruptRaise,
			exceptionRaise                 => exceptionRaise,
			ecallFlag                      => ecallFlag,
			illegalInstrFlag               => illegalInstrFlag,
			validAccessCSR                 => validAccessCSR,
			readOnlyCSR                    => readOnlyCSR,
			mirror                         => mirror,
			ldMieReg                       => ldMieReg,
			ldMieUieField                  => ldMieUieField,
			delegationMode                 => delegationMode,
			previousPRV                    => previousPRV,
			modeTvec                       => modeTvec,
			mipCCLdDisable                 => mipCCLdDisable,
			selCCMip_CSR                   => selCCMip_CSR,
			selCause_CSR                   => selCause_CSR,
			selPC_CSR                      => selPC_CSR,
			selTval_CSR                    => selTval_CSR,
			selMedeleg_CSR                 => selMedeleg_CSR,
			selMideleg_CSR                 => selMideleg_CSR,
			ldValueCSR                     => ldValueCSR,
			ldCntCSR                       => ldCntCSR,
			dnCntCSR                       => dnCntCSR,
			upCntCSR                       => upCntCSR,
			ldDelegation                   => ldDelegation,
			ldMachine                      => ldMachine,
			ldUser                         => ldUser,
			loadMieReg                     => loadMieReg,
			loadMieUieField                => loadMieUieField,
			mirrorUserCU                   => mirrorUserCU,
			writeRegBank                   => writeRegBank,
			selRomAddress                  => selRomAddress,
			selMepc_CSR                    => selMepc_CSR,
			selInterruptAddressDirect      => selInterruptAddressDirect,
			selInterruptAddressVectored    => selInterruptAddressVectored,
			machineStatusAlterationPreCSR  => machineStatusAlterationPreCSR,
			userStatusAlterationPreCSR     => userStatusAlterationPreCSR,
			machineStatusAlterationPostCSR => machineStatusAlterationPostCSR,
			userStatusAlterationPostCSR    => userStatusAlterationPostCSR,
			zeroCntCSR                     => zeroCntCSR,
			instructionDoneCSR			   => instructionDoneCSR,

			-- pipeline registers
			GI2D_en						   => GI2D_en,
			GI2D_rst					   => GI2D_rst,
			D2E_en						   => D2E_en,
			D2E_rst					       => D2E_rst,
			E2M_en						   => E2M_en,
			E2M_rst						   => E2M_rst,
			M2WB_en						   => M2WB_en,
			M2WB_rst					   => M2WB_rst,

			-- hazards
			hazEX						   => hazEX,
			hazM						   => hazM
	);
	interruptProcessing <= mipCCLdDisable;
END ARCHITECTURE procedural;
