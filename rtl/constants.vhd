library ieee;
use ieee.std_logic_1164.all;

package CONSTANTS is
        CONSTANT iTypeImm            : STD_LOGIC_VECTOR(11 DOWNTO 0) := "010011001001";
		CONSTANT sTypeImm            : STD_LOGIC_VECTOR(11 DOWNTO 0) := "010011010010";
		CONSTANT uTypeImm            : STD_LOGIC_VECTOR(11 DOWNTO 0) := "100000100100";
		CONSTANT jTypeImm            : STD_LOGIC_VECTOR(11 DOWNTO 0) := "101001001100";
		CONSTANT bTypeImm            : STD_LOGIC_VECTOR(11 DOWNTO 0) := "010101010100";
		CONSTANT Loads               : STD_LOGIC_VECTOR(6 DOWNTO 0)  := "0000011";
		CONSTANT Stores              : STD_LOGIC_VECTOR(6 DOWNTO 0)  := "0100011";
		CONSTANT Arithmetic          : STD_LOGIC_VECTOR(6 DOWNTO 0)  := "0110011";
		CONSTANT ImmediateArithmetic : STD_LOGIC_VECTOR(6 DOWNTO 0)  := "0010011";
		CONSTANT JumpAndLink         : STD_LOGIC_VECTOR(6 DOWNTO 0)  := "1101111";
		CONSTANT JumpAndLinkRegister : STD_LOGIC_VECTOR(6 DOWNTO 0)  := "1100111";
		CONSTANT Branch              : STD_LOGIC_VECTOR(6 DOWNTO 0)  := "1100011";
		CONSTANT LoadUpperImmediate  : STD_LOGIC_VECTOR(6 DOWNTO 0)  := "0110111";
		CONSTANT AddUpperImmediatePC : STD_LOGIC_VECTOR(6 DOWNTO 0)  := "0010111";
		CONSTANT SystemAndCSR        : STD_LOGIC_VECTOR(6 DOWNTO 0)  := "1110011";
        TYPE state IS (idle, -- for all the instructions except the ones which need int/ex management                                                                                                                                            --LUI, AUIPC
		checkDelegation, updateTrapValue, updateMip, updateUip, updateCause, readMstatus, 
						 updateMstatus, updateUstatus, updateEpc, readTvec1, readTvec2, --Interrupt Entry States                                                                                                                      -- CSR Instructions
		retEpc, retReadMstatus, retUpdateMstatus, retUpdateUstatus,  --Interrupt Exit States
		hazEX_state, hazM_state, -- for hazard management
		waitForStore
		);
		TYPE CSR_state IS (idle, mirror_state);
		CONSTANT R_type				 : STD_LOGIC_VECTOR(2 DOWNTO 0) := "000";
		CONSTANT I_type				 : STD_LOGIC_VECTOR(2 DOWNTO 0) := "001";
		CONSTANT S_type				 : STD_LOGIC_VECTOR(2 DOWNTO 0) := "010";
		CONSTANT U_type				 : STD_LOGIC_VECTOR(2 DOWNTO 0) := "011";
		CONSTANT B_type				 : STD_LOGIC_VECTOR(2 DOWNTO 0) := "100";
		CONSTANT J_type				 : STD_LOGIC_VECTOR(2 DOWNTO 0) := "101";
end CONSTANTS;