-- **************************************************************************************
--	Filename:	aftab_datapath.vhd
--	Project:	CNL_RISC-V
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
--	Datapath of the AFTAB core
--
-- **************************************************************************************
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
ENTITY aftab_datapath IS
	GENERIC (len : INTEGER := 32);
	PORT (
		clk    						: IN STD_LOGIC;
		rst            			    : IN STD_LOGIC;
		writeRegFile			    : IN STD_LOGIC;
		setOne						: IN STD_LOGIC;
		setZero 					: IN STD_LOGIC;
		ComparedSignedUnsignedBar 	: IN STD_LOGIC;
		selPC 						: IN STD_LOGIC;
		selPCJ					    : IN STD_LOGIC;
		selI4 						: IN STD_LOGIC;
		selAdd 						: IN STD_LOGIC;
		selJL 						: IN STD_LOGIC;
		selADR 						: IN STD_LOGIC;
		selInc4PC 					: IN STD_LOGIC;
		selBSU 						: IN STD_LOGIC;
		selLLU 						: IN STD_LOGIC;
		selASU 						: IN STD_LOGIC;
		selAAU 						: IN STD_LOGIC;
		selDARU 					: IN STD_LOGIC;
		selP1 						: IN STD_LOGIC;
		selP2 						: IN STD_LOGIC;
		selImm     					: IN STD_LOGIC;               
		ldPC 						: IN STD_LOGIC;
		zeroPC 						: IN STD_LOGIC;
		ldADR 						: IN STD_LOGIC;
		zeroADR 					: IN STD_LOGIC;
		ldDR 						: IN STD_LOGIC;
		zeroDR 						: IN STD_LOGIC;
		ldIR 						: IN STD_LOGIC;
		zeroIR 						: IN STD_LOGIC;
		ldByteSigned 				: IN STD_LOGIC;
		ldHalfSigned 				: IN STD_LOGIC;
		load         				: IN STD_LOGIC;
		selShift       				: IN STD_LOGIC_VECTOR (1 DOWNTO 0);--
		addSubBar 					: IN STD_LOGIC;
		pass 						: IN STD_LOGIC;
		selAuipc            		: IN  STD_LOGIC;
		muxCode      				: IN  STD_LOGIC_VECTOR (11 DOWNTO 0);
		selLogic  					: IN  STD_LOGIC_VECTOR (1 DOWNTO 0);
		startDAWU 					: IN STD_LOGIC;
		startDARU 					: IN STD_LOGIC;
		memReady         			: IN  STD_LOGIC;
		startMultiplyAAU 			: IN STD_LOGIC;
		startDivideAAU 				: IN STD_LOGIC;
		signedSigned 				: IN STD_LOGIC;
		signedUnsigned 				: IN STD_LOGIC;
		unsignedUnsigned 			: IN STD_LOGIC;
		selAAL 						: IN STD_LOGIC;
		selAAH 						: IN  STD_LOGIC;
		completeAAU     			: OUT STD_LOGIC;
		nBytes          			: IN  STD_LOGIC_VECTOR (1 DOWNTO 0);
		memDataIn       			: IN  STD_LOGIC_VECTOR (7 DOWNTO 0);
		memAddrDAWU 			    : OUT STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
		memAddrDARU  				: OUT STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
		IR                			: OUT STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
		memDataOut         			: OUT STD_LOGIC_VECTOR (7 DOWNTO 0);
		lt  						: OUT STD_LOGIC;
		eq  						: OUT STD_LOGIC;
		gt  						: OUT STD_LOGIC;
		completeDAWU  				: OUT STD_LOGIC;
		completeDARU  				: OUT STD_LOGIC;
		readMem  					: OUT STD_LOGIC;
		writeMem  					: OUT STD_LOGIC;
		dataError       			: OUT STD_LOGIC;
		
		--CSR and Interrupt inputs
		selCSR   					: IN  STD_LOGIC;
		externalInterrupt		    : IN  STD_LOGIC;
		timerInterrupt 			    : IN  STD_LOGIC;
		softwareInterrupt			: IN  STD_LOGIC;
		ldValueCSR					: IN  STD_LOGIC_VECTOR (2 DOWNTO 0);
		mipCCLdDisable			    : IN  STD_LOGIC;
		selImmCSR					: IN  STD_LOGIC;
		selP1CSR					: IN  STD_LOGIC;
		selReadWriteCSR				: IN  STD_LOGIC;
		clrCSR						: IN  STD_LOGIC;
		setCSR						: IN  STD_LOGIC;
		selPC_CSR					: IN  STD_LOGIC;
		selCCMip_CSR				: IN  STD_LOGIC;
		selCause_CSR				: IN  STD_LOGIC;
		selMepc_CSR					: IN  STD_LOGIC;
		selInterruptAddressDirect  	: IN  STD_LOGIC;
		selInterruptAddressVectored	: IN  STD_LOGIC;
		statusAlterationPreCSR		: IN  STD_LOGIC;
		statusAlterationPostCSR		: IN  STD_LOGIC;
		writeRegBank				: IN  STD_LOGIC;
		dnCntCSR					: IN  STD_LOGIC;
		upCntCSR					: IN  STD_LOGIC;
		ldCntCSR					: IN  STD_LOGIC;
		zeroCntCSR					: IN  STD_LOGIC;
		selCSRAddrFromInst			: IN  STD_LOGIC;
		selRomAddress				: IN  STD_LOGIC;
		interruptRaise				: OUT  STD_LOGIC;
		modeMtvec					: OUT  STD_LOGIC_VECTOR (1 DOWNTO 0)	
	);
END ENTITY aftab_datapath;
ARCHITECTURE behavioral OF aftab_datapath IS
	SIGNAL immediate  : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL inst       : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL resAAH     : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL resAAL 	  : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL instrError : STD_LOGIC;
	SIGNAL p1         : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL p2         : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL writeData  : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL outMux6    : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL dataDARU   : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL dataDAWU   : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL adjDARU 	  : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL addResult  : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL lluResult  : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL asuResult  : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL aauResult  : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL bsuResult  : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL outADR     : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL outMux2    : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL inPC  	  : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL outPC      : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL inc4PC     : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL outMux5    : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL addrIn     : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);

	--CSR Signals
	SIGNAL CCmieField 					 : STD_LOGIC;
	SIGNAL mipCCLd 						 : STD_LOGIC;
	SIGNAL cntInput  					 : STD_LOGIC_VECTOR (2 DOWNTO 0);
	SIGNAL addressRegBank  				         : STD_LOGIC_VECTOR (11 DOWNTO 0);
	SIGNAL outAddr  					 : STD_LOGIC_VECTOR (11 DOWNTO 0);
	SIGNAL interruptSources  			         : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL CCmip  						 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL outCSR  						 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL inCSR  						 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL causeCode  					 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL CCmie 						 : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL interruptStartAddressDirect   : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
	SIGNAL interruptStartAddressVectored : STD_LOGIC_VECTOR (len - 1 DOWNTO 0);
						
					

BEGIN
	IR <= inst;
	registerFile : ENTITY WORK.aftab_register_file
		GENERIC MAP(len => len)
		PORT MAP(
			clk => clk, 
			rst => rst,
			setZero => setZero,
			setOne => setOne,
			rs1 => inst (19 DOWNTO 15), 
			rs2 => inst (24 DOWNTO 20),
			rd => inst (11 DOWNTO 7), 
			writeData => writeData,
			writeRegFile => writeRegFile, 
			p1 => p1,
			p2 => p2);
			
	regIR : ENTITY WORK.aftab_register
		GENERIC MAP(len => len)
		PORT MAP(
			clk => clk, 
			rst => rst, 
			zero => zeroIR,
			load => ldIR,
			inReg => dataDARU, 
			outReg => inst);
			
	immSelSignEx : ENTITY WORK.aftab_imm_sel_sign_ext
		PORT MAP(
			IR7 => inst (7), 
			IR20 => inst (20), 
			IR31 => inst (31),
			IR11_8 => inst (11 DOWNTO 8), 
			IR19_12 => inst (19 DOWNTO 12),
			IR24_21 => inst (24 DOWNTO 21), 
			IR30_25 => inst (30 DOWNTO 25),
			selI => muxCode (0), 
			selS => muxCode (1),
			selBUJ => muxCode (2),
			selIJ => muxCode (3),
			selSB => muxCode (4), 
			selU => muxCode (5),
			selISBJ => muxCode (6),
			selIS => muxCode (7),
			selB => muxCode (8), 
			selJ => muxCode (9),
			selISB => muxCode (10), 
			selUJ => muxCode (11),
			Imm => immediate);
			
	adder : ENTITY WORK.aftab_adder
		GENERIC MAP(len => len)
		PORT MAP(
			Cin => '0', 
			A => immediate, 
			B => outMux2,
			addResult => addResult,
			carryOut => OPEN);

---	mux1 : 
	inPC <= addResult 						  WHEN 	selAdd = '1'      			   	 ELSE
			inc4PC 							  WHEN 	selI4 = '1'       			   	 ELSE
			outCSR 							  WHEN selMepc_CSR = '1'  			   	 ELSE 
			interruptStartAddressDirect  	  WHEN selInterruptAddressDirect = '1' 	 ELSE
		    interruptStartAddressVectored  	  WHEN selInterruptAddressVectored = '1' ELSE (OTHERS => '0');
			
			
	regPC : ENTITY WORK.aftab_register
		GENERIC MAP(len => len)
		PORT MAP(
			clk => clk, 
			rst => rst, 
			zero => zeroPC,
			load => ldPC, 
			inReg => inPC,
			outReg => outPC);
			
	mux2 : ENTITY WORK.aftab_multiplexer
			GENERIC MAP(len => len)
			PORT MAP(
				a => outMux6,
				b => outPC,
				s0 => selJL,
				s1 => selPC, 
				w => outMux2);
				
	mux3 : ENTITY WORK.aftab_multiplexer
			GENERIC MAP(len => len)
			PORT MAP(
				a => outADR,
				b => outMux2,
				s0 => selADR, 
				s1 => selPCJ, 
				w => addrIn);
					
	regADR : ENTITY WORK.aftab_register
				GENERIC MAP(len => len)
				PORT MAP(
					clk => clk,
					rst => rst, 
					zero => zeroADR,
					load => ldADR,
					inReg => addResult, 
					outReg => outADR);
					
	i4PC : ENTITY WORK.aftab_adder
			GENERIC MAP(len => len)
			PORT MAP(
				Cin => '0', 
				A => outPC,
				B => (31 DOWNTO 3 => '0') & "100",
				addResult => inc4PC,
				carryOut => OPEN);
				
	writeData <= inc4PC    WHEN selInc4PC = '1' ELSE
				 bsuResult WHEN selBSU = '1'    ELSE
				 lluResult WHEN selLLU = '1'    ELSE
				 asuResult WHEN selASU = '1'    ELSE
				 aauResult WHEN selAAU = '1'    ELSE
				 adjDARU   WHEN selDARU = '1'   ELSE 
				 outCSR    WHEN selCSR = '1'   ELSE (OTHERS => '0');
				 
	regDR : ENTITY WORK.aftab_register
			 GENERIC MAP(len => len)
			 PORT MAP(
				clk => clk,
				rst => rst,
				zero => zeroDR,
				load => ldDR,
				inReg => p2, 
				outReg => dataDAWU);
				
	mux5 : ENTITY WORK.aftab_multiplexer
			GENERIC MAP(len => len)
			PORT MAP(
				a => p2, 
				b => immediate,
				s0 => selP2, 
				s1 => selImm,
				w => outMux5);
				
	mux6 : ENTITY WORK.aftab_multiplexer
			GENERIC MAP(len => len)
			PORT MAP(
				a => p1, 
				b => outPC, 
				s0 => selP1, 
				s1 => selAuipc,
				w => outMux6);
				
	LLU : ENTITY WORK.aftab_llu
			GENERIC MAP(len => len)
			PORT MAP(
				ain => outMux6,
				bin => outMux5,
				selLogic => selLogic, 
				result => lluResult);
				
	BSU : ENTITY WORK.aftab_barrel_shifter
			GENERIC MAP(len => len)
			PORT MAP(
				shIn => outMux6,
				nSh => outMux5 (4 DOWNTO 0),
				selSh => selShift, 
				shOut => bsuResult);
				
	comparator : ENTITY WORK.aftab_comparator
					GENERIC MAP(len => len)
					PORT MAP(
						ain => outMux6, 
						bin => outMux5, 
						CompareSignedUnsignedBar => ComparedSignedUnsignedBar,
						Lt => lt,
						Eq => eq,
						Gt => gt);
						
	addSub : ENTITY WORK.aftab_adder_subtractor
				GENERIC MAP(len => len)
				PORT MAP(
					a => outMux6,
					b => outMux5,
					subSel => addSubBar,
					pass => pass,
					cout => OPEN,
					outRes => asuResult); 

	aau : ENTITY WORK.aftab_aau 
			GENERIC MAP (len => len) 
			PORT MAP(
				clk => clk,
				rst => rst,
				ain => outMux6,
				bin => p2,
				startMultAAU => startMultiplyAAU,
				startDivideAAU => startDivideAAU,
				SignedSigned => signedSigned,
				SignedUnsigned => signedUnsigned,
				UnsignedUnsigned => unsignedUnsigned,
				resAAU1 => resAAH,
				resAAU2 => resAAL,
				completeAAU => completeAAU);
				
	aauResult <= resAAH WHEN selAAH = '1' ELSE
				 resAAL WHEN selAAL = '1' ELSE (OTHERS => '0');
	dawu : ENTITY WORK.aftab_dawu
			PORT MAP(
				clk => clk,
				rst => rst,
				startDAWU => startDAWU,
				memReady => memReady, 
				nBytes => nBytes, 
				addrIn => addrIn,
				dataIn => dataDAWU, 
				addrOut => memAddrDAWU,
				dataOut => memDataOut,
				writeMem => writeMem, 
				dataError => open, 
				completeDAWU => completeDAWU);
				
	daru : ENTITY WORK.aftab_daru
		    PORT MAP(
				clk => clk, 
				rst => rst, 
				startDARU => startDARU, 
				nBytes => nBytes,
				addrIn => addrIn,
				memData => memDataIn,
				memReady => memReady,
				completeDARU => completeDARU,
				dataOut => dataDARU, 
				addrOut => memAddrDARU, 
				dataError => open,
				instrError => instrError,
				readMem => readMem);
				
	suau : ENTITY WORK.aftab_sulu 
			GENERIC MAP (len => len)
				PORT MAP(
					loadByteSigned => ldByteSigned, 
					loadHalfSigned => ldHalfSigned, 
					load => load,
					dataIn => dataDARU, 
					dataOut => adjDARU);
--CSR Units
	--interruptSourceSynchronizationRegister
    mipCCLd <=	NOT (mipCCLdDisable);
	interruptSources <= "00000000000000000000" & externalInterrupt & "000" & timerInterrupt & "000" & softwareInterrupt & "000";
	interSrcSynchReg: ENTITY work.aftab_register  
			GENERIC MAP(len => 32)
			    PORT MAP(
				clk =>    clk ,
				rst =>    rst ,
				zero =>   '0' ,
				load =>   mipCCLd ,
				inReg =>  interruptSources   ,
				outReg => CCmip
			);
	ISLFCSR: ENTITY WORK.aftab_ISLFCSR 
			GENERIC MAP (len => len)
				PORT MAP(
					selP1 =>selP1CSR ,
					selIm =>selImmCSR ,
					selReadWrite =>selReadWriteCSR ,
					clr =>clrCSR ,
					set => setCSR,
					selPC => selPC_CSR,
					selmip => selCCMip_CSR,
					selCause => selCause_CSR,
					statusAlterationPre => statusAlterationPreCSR ,
					statusAlterationPost => statusAlterationPostCSR,
					ir19_15 => inst (19 DOWNTO 15),
					CCmip => CCmip,
					causeCode => causeCode,
					P1 => p1 ,
					PC => outPC,
					outCSR => outCSR,
					inCSR => inCSR);

	register_bank: ENTITY WORK.aftab_register_bank 
			GENERIC MAP (len => len)
			PORT MAP(
				clk =>   clk   ,
				rst => rst,
				writeRegBank =>   writeRegBank  ,
				addressRegBank => addressRegBank  ,
				inputRegBank =>   inCSR  ,
				outRegBank =>     outCSR ,
				outMieFieldCCreg =>     CCmieField ,
				outMieCCreg      =>     CCmie 
				);	
	--modeMtvec <= outCSR (1 DOWNTO 0);
	
	CSRCounter: ENTITY WORK.CSRCounter 
			GENERIC MAP (len => 3)
			PORT MAP(
				clk => clk,
				rst => rst ,
				dnCnt => dnCntCSR,
				upCnt => upCntCSR ,
				ldCnt => ldCntCSR,
				zeroCnt => zeroCntCSR ,
				ldValue => ldValueCSR,
				outCnt => cntInput
			);	

	-- ROM : ENTITY WORK.CSRAddressingROM_constant 
			-- PORT MAP(
				-- clk => clk,
				-- inputAddr => index,
				-- outdata => RomAddress
			-- );

	CSRAddressingDecoder : ENTITY WORK.CSRAddressingDecoder 
			 PORT MAP(
				cntInput => cntInput,
				outAddr => outAddr
			);			
			
	mux7 : ENTITY WORK.aftab_multiplexer
			GENERIC MAP(len => 12)
			PORT MAP(
				a => inst(31 DOWNTO 20), 
				b => outAddr, 
				s0 => selCSRAddrFromInst, 
				s1 => selRomAddress,
				w => addressRegBank);
	
	interrCheckCauseDetection: ENTITY WORK.aftab_interrCheckCauseDetection
				GENERIC MAP(len => len)
				PORT MAP(
					mipCC => CCmip ,
					mieCC => CCmie ,
					mieFieldCC => CCmieField ,
					interruptRaise => interruptRaise ,
					causeCode => causeCode
				);	

	interruptStartAddressGenerator: ENTITY WORK.aftab_iagu
			GENERIC MAP(len => len)
			PORT MAP(
				mtvecBase 	=>	outCSR  ,
				causeCode   => causeCode (5 DOWNTO 0)    ,
				interruptStartAddressDirect    => interruptStartAddressDirect  , 
				interruptStartAddressVectored  => interruptStartAddressVectored 
			);			
					
END ARCHITECTURE behavioral;
