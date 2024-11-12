create_clock -name core_clk -period 8 -waveform {0 1} [get_pins uep_PH1_PHY_PCIE.core_clk]
create_generated_clock  -name  pcie_user_clk -source  [get_pins uep_PH1_PHY_PCIE.core_clk ] -phase 0 -multiply_by 1 -duty_cycle 25  [get_pins {u_pcie_ep_core_ph1_clock_ctrl/u_pcie_ep_core_ph1_core_pll/pll_inst.clkc[0]}]
set_clock_groups  -exclusive -group [get_clocks {pcie_user_clk core_clk}]
set_false_path -from [get_pins uep_PH1_PHY_PCIE.core_rst_n]
