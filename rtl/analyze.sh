rm -rf build
mkdir build
cd build

/home/davide/Downloads/ghdl-gha-ubuntu-22.04-gcc/bin/ghdl -a --ieee=synopsys -fexplicit ../constants.vhd ../aftab_datapath/aftab_register.vhd ../aftab_datapath/aftab_multiplexer.vhd ../aftab_datapath/aftab_comparator.vhd ../aftab_datapath/aftab_counter.vhd ../aftab_datapath/aftab_isseu.vhd \
../aftab_datapath/aftab_full_adder.vhd ../aftab_datapath/aftab_half_adder.vhd ../aftab_datapath/aftab_one_bit_register.vhd ../aftab_datapath/aftab_opt_adder.vhd ../aftab_datapath/aftab_adder.vhd \
../aftab_datapath/aftab_adder_subtractor.vhd ../aftab_datapath/aftab_decoder.vhd ../aftab_datapath/aftab_barrel_shifter.vhd ../aftab_datapath/aftab_llu.vhd ../aftab_datapath/aftab_sulu.vhd \
../aftab_datapath/aftab_register_file.vhd ../aftab_datapath/aftab_csr/aftab_csr_address_ctrl.vhd ../aftab_datapath/aftab_csr/aftab_csr_address_logic.vhd ../aftab_datapath/aftab_csr/aftab_csr_addressing_decoder.vhd \
../aftab_datapath/aftab_csr/aftab_csr_counter.vhd ../aftab_datapath/aftab_csr/aftab_csr_isl.vhd ../aftab_datapath/aftab_csr/aftab_csr_registers.vhd ../aftab_datapath/aftab_csr/aftab_iccd.vhd \
../aftab_datapath/aftab_csr/aftab_isagu.vhd ../aftab_datapath/aftab_csr/aftab_register_bank.vhd ../aftab_datapath/aftab_aau/aftab_shift_register.vhd \
../aftab_datapath/aftab_aau/aftab_booth_multiplier/aftab_booth_multiplier_controller.vhd ../aftab_datapath/aftab_aau/aftab_booth_multiplier/aftab_booth_multiplier_datapath.vhd \
../aftab_datapath/aftab_aau/aftab_booth_multiplier/aftab_booth_multiplier.vhd ../aftab_datapath/aftab_aau/aftab_su_divider/aftab_divider_controller.vhd \
../aftab_datapath/aftab_aau/aftab_su_divider/aftab_divider_datapath.vhd ../aftab_datapath/aftab_aau/aftab_su_divider/aftab_divider.vhd ../aftab_datapath/aftab_aau/aftab_su_divider/aftab_tcl.vhd \
../aftab_datapath/aftab_aau/aftab_su_divider/aftab_su_divider.vhd ../aftab_datapath/aftab_aau/aftab_aau.vhd ../aftab_datapath/aftab_daru/aftab_daru_controller.vhd \
../aftab_datapath/aftab_daru/aftab_daru_error_detector.vhd ../aftab_datapath/aftab_daru/aftab_daru_datapath.vhd ../aftab_datapath/aftab_daru/aftab_daru.vhd \
../aftab_datapath/aftab_dawu/aftab_dawu_controller.vhd ../aftab_datapath/aftab_dawu/aftab_dawu_error_detector.vhd ../aftab_datapath/aftab_dawu/aftab_dawu_datapath.vhd ../aftab_datapath/aftab_dawu/aftab_dawu.vhd \
../aftab_datapath/aftab_datapath.vhd ../aftab_controller.vhd ../aftab_core.vhd

# ghdl -e --std=08 --ieee=synopsys -fexplicit aftab_core