set design "ahb2apb_bridge"

analyze -format verilog {apb_master_fsm.v ahb2apb_bridge.v ahb_capture_fsm.v timeout_monitor.v resp_error_mapper.v cdc_handshake.v addr_decoder.v }

elaborate $design 


source constraints_sdc.sdc
link
check_design
write -format verilog -hier -output unmapped/$design.v
write -format ddc -hierarchy -output unmapped/$design.ddc
compile_ultra
check_timing
write -format verilog -hierarchy -output netlist/netlist_$design.v
write -format ddc -hierarchy -output mapped/$design.ddc
cd report
report_timing > timing_report_$design.rpt
report_area > area_report_$design.rpt
cd ..
