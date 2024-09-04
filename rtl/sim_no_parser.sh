#!/bin/sh
vlog aftab_core.v
vcom -2008 ../tb/aftab_memory.vhd
vcom ../tb/aftab_testbench.vhd
rm vsim_vcd.vcd
# vsim work.aftab_testbench -do "vcd file vsim_vcd.vcd; vcd add /aftab_testbench/core/*; run 600000 ns"
vsim work.aftab_testbench -do "add wave /aftab_testbench/*; add wave /aftab_testbench/core/*; run 600000 ns"