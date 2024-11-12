`ifdef DEBUG_MODE
	`include "para_def.vh"
`endif
module app_tst_ctrl(
	input	wire   usr_rst_n,
	input	wire   usr_clk,

	input   wire   [1:0]   usr_lp0rw_md_i,
    input   wire   s0_axis_c2h_run_i,
    input   wire   m0_axis_h2c_run_i,
	output  wire   cfg_regrw_run_o,
	output  wire   [1:0]   dma_regrw_run_o,
	output  wire    usr_h2c0w_run_o,
    output  wire    usr_c2h0r_run_o,
	output  wire   usr_regrw_run_o,
    output  wire   usr_lp0rw_run_o
);
//---Constant definition;
localparam INIT_DLY = 12'd4010;
//---Variable definition;
reg   [11:0]   cnt_dly;
reg   init_en;
wire  init_over;
reg   cfg_regrw_run; 
reg   [1:0]    dma_regrw_run;
//reg   usr_h2c0w_run;
//reg   usr_c2h0r_run;
reg   usr_regrw_run; 
reg   usr_lp0rw_run; //synthesis max_fanout=10
reg	  usr_h2c0w_run;
reg	  usr_c2h0r_run;
always@(posedge usr_clk,negedge usr_rst_n )
begin 
	if(!usr_rst_n) 
		cnt_dly <= `DLY 12'd0;
	else if(init_over) 
		cnt_dly <= `DLY 12'd0;
	else if(init_en)
		cnt_dly <= `DLY cnt_dly + 12'd1;
	else ;
end 
always@(posedge usr_clk,negedge usr_rst_n )
begin 
	if(!usr_rst_n) 
		init_en <= `DLY 1'b1;
	else if(init_over) 
		init_en <= `DLY 1'b0;
	else ;
end 
assign init_over = (cnt_dly==INIT_DLY-1) ? 1'b1 : 1'b0;

always@(posedge usr_clk,negedge usr_rst_n )
begin 
	if(!usr_rst_n) begin
		cfg_regrw_run <= `DLY 1'b0;
		dma_regrw_run <= `DLY 2'b00;
		usr_regrw_run <= `DLY 1'b0;
	end
	else begin
    	case (cnt_dly)
        	12'd100:  usr_regrw_run <= `DLY 1'b1;
        	12'd1000: cfg_regrw_run <= `DLY 1'b0;
		    12'd2000: dma_regrw_run[0] <= `DLY 1'b1;
            12'd3000: dma_regrw_run[1] <= `DLY 1'b1;
			default: ;
        endcase
	end
end
assign cfg_regrw_run_o = cfg_regrw_run;
assign dma_regrw_run_o = dma_regrw_run;

assign usr_lp0rw_run_o = usr_lp0rw_run;
assign usr_regrw_run_o = usr_regrw_run;


always@(posedge usr_clk,negedge usr_rst_n )
begin
    if (!usr_rst_n)
        usr_lp0rw_run <= `DLY 1'b0;
    else
        usr_lp0rw_run <= `DLY (&usr_lp0rw_md_i);
end

always@(posedge usr_clk,negedge usr_rst_n )
begin
    if (!usr_rst_n)
        usr_h2c0w_run <= `DLY 1'b0;
    else
        usr_h2c0w_run <= `DLY ~usr_lp0rw_run & m0_axis_h2c_run_i;
end
assign usr_h2c0w_run_o = usr_h2c0w_run;

always@(posedge usr_clk,negedge usr_rst_n )
begin
    if ( !usr_rst_n )
        usr_c2h0r_run <= `DLY 1'b0;
    else
        usr_c2h0r_run <= `DLY ~usr_lp0rw_run & s0_axis_c2h_run_i;
end
assign usr_c2h0r_run_o = usr_c2h0r_run;

endmodule
