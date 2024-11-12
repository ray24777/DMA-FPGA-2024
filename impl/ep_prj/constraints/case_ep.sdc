create_clock -name {aux_clk} -period 40.000 -waveform {0.000 20.000} [get_ports {app_auxclk}]

#set_clock_groups -exclusive -group [get_clocks {aux_clk}] -group [get_clocks {u_sgdma_subsys/u_pcie_support/u_pcie_ep_core/core_clk}] -group [get_clocks {u_sgdma_subsys/u_pcie_support/u_pcie_ep_core/pcie_user_clk}]
