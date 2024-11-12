`ifdef DEBUG_MODE
	`include "para_def.vh"
`endif
module status_sig(
	input   core_rst_n,
	input   core_clk,
	output  core_clk_led_o,
	output  app_app_ltssm_enable_o
	
 );
localparam DATA_IF_CNT = 12'd1520;
reg   [23:0] 	core_clk_led_cnt;

always @ (posedge core_clk,negedge core_rst_n) //negedge sync_core_rst_n[1]
begin
    if(!core_rst_n)
	   core_clk_led_cnt <= `DLY 24'd0;
    else
	   core_clk_led_cnt <= `DLY core_clk_led_cnt + 23'd1;
end
assign core_clk_led_o = core_clk_led_cnt[23];

assign app_app_ltssm_enable_o = 1'b1;
endmodule
