# Failing setup/recovery paths of the system clock, for the job log (our fork's CI)
project_open Model3
create_timing_netlist -model slow
read_sdc
update_timing_netlist
set clk [get_clocks {emu|pll|pll_inst|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk}]
foreach kind {setup recovery} {
  set fh [open "fail_$kind.txt" w]
  if {$kind eq "setup"} {
    set paths [get_timing_paths -setup -npaths 50000 -nworst 1 -less_than_slack 0.0 -to_clock $clk]
  } else {
    set paths [get_timing_paths -recovery -npaths 50000 -nworst 1 -less_than_slack 0.0 -to_clock $clk]
  }
  foreach_in_collection p $paths {
    set s [get_path_info $p -slack]
    set f [get_node_info -name [get_path_info $p -from]]
    set t [get_node_info -name [get_path_info $p -to]]
    puts $fh "$s\t$f\t$t"
  }
  close $fh
}
project_close
