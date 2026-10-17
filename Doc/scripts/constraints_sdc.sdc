create_clock -name hclk -period 10 [get_ports hclk]

create_clock -name pclk -period 40 [get_ports pclk]

set_clock_groups -asynchronous -group [get_clocks hclk] -group [get_clocks pclk]


set_clock_uncertainty -setup 4  [get_clocks pclk]

set_clock_uncertainty -setup 1 [get_clocks hclk]


set_input_delay -clock hclk -max 6 [get_ports {hsel hready haddr htrans hwrite hsize hwdata}]
set_input_delay -clock hclk -min 0.3 [get_ports {hsel hready haddr htrans hwrite hsize hwdata}]


set_output_delay -clock hclk -min 0.3 [get_ports {hrdata hreadyout hresp}]
set_output_delay -clock hclk -max 6 [get_ports {hrdata hreadyout hresp}]


set_input_delay -clock pclk -max 24 [get_ports {prdata_bus pready_bus pslverr_bus}]
set_input_delay -clock pclk -min 1.2 [get_ports {prdata_bus pready_bus pslverr_bus}]


set_output_delay -clock pclk -min 1.2 [get_ports {paddr pwrite pwdata pstrb}]
set_output_delay -clock pclk -max 24 [get_ports {paddr pwrite pwdata pstrb}]


