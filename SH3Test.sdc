derive_pll_clocks

# The CPU PLL is runtime-reconfigured across 75..112 MHz (the test schedule)
# while the compile-time base config stays at the IP's parameterized value.
# Constrain the PLL output at the 102.8 MHz TARGET (period 9.727 ns): the
# fitter closes timing at the frequency the core must reach, and the
# 105/108/112 steps then measure the real silicon margin beyond closure.
# No -name: the clock keeps its auto-derived name, so the sys_top.sdc clock
# groups and the false paths below keep matching.
create_clock -period 9.727 [get_pins -compatibility_mode {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}]

derive_clock_uncertainty

# pll2 (the 53.69 MHz video/overlay clock) is a separate VCO and physically
# asynchronous to every other clock in the design. sys_top.sdc's clock groups
# only cover a core PLL named "pll", so declare pll2 async to everything else
# here. All crossings are 2/3-stage synchronizers or quasi-static display
# values (same CDC style as upstream).
set_clock_groups -asynchronous -group [get_clocks {emu|pll2|pll2_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}]

set_false_path -from [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}] -to [get_clocks {emu|pll2|pll2_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}]
set_false_path -from [get_clocks {emu|pll2|pll2_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}] -to [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}]
