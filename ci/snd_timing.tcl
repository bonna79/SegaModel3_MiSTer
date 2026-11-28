# The sound bridge's worst paths (setup and hold, into and out of u_snd), for the job log.
project_open Model3
create_timing_netlist -model slow
read_sdc
update_timing_netlist
report_timing -setup -npaths 8 -detail summary -to [get_registers {*u_snd*}] -file snd_setup_to.rpt
report_timing -setup -npaths 8 -detail summary -from [get_registers {*u_snd*}] -file snd_setup_from.rpt
report_timing -hold -npaths 4 -detail summary -to [get_registers {*u_snd*}] -file snd_hold_to.rpt
report_timing -recovery -npaths 4 -detail summary -to [get_registers {*u_snd*}] -file snd_recovery_to.rpt
delete_timing_netlist
project_close
