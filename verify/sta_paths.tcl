# Post-fit timing reports for the SH-3 stress suite (quartus_sta -t sta_paths.tcl).
# sta_intra_pll.txt: the CPU-clock critical paths — the input to the next
# fitter iteration; cross-reference failing paths against the stream (A-E)
# that errors first on hardware.
project_open SH3Test
create_timing_netlist -model slow
read_sdc
update_timing_netlist
set pll_clk {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}
report_timing -setup -from_clock [get_clocks $pll_clk] -to_clock [get_clocks $pll_clk] -npaths 20 -detail full_path -file sta_intra_pll.txt
report_timing -setup -to_clock [get_clocks {emu|pll2|pll2_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}] -npaths 6 -detail summary -file sta_to_pll2.txt
project_close
