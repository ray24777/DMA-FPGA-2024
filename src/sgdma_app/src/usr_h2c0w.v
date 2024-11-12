`ifdef DEBUG_MODE
	`include "para_def.vh"
	`include "dt_para.vh"
`endif
module	usr_h2c0w(
   	input   wire   usr_clk,
	input   wire   usr_rst_n,

	input   wire   usr_h2c0w_run_i,
	input   wire   m0_axis_h2c_rst_i,
    
	output  wire   m0_axis_h2c_tready_o,
    input   wire   [`DATA_WIDTH-1:0]   m0_axis_h2c_tdata_i,
    input   wire   [`KEEP_WIDTH-1:0]   m0_axis_h2c_tkeep_i,
    input   wire   [`KEEP_WIDTH-1:0]   m0_axis_h2c_tuser_i,
    input   wire   m0_axis_h2c_tlast_i,
    input   wire   m0_axis_h2c_tvalid_i,
	
	output	wire   usr_h2c0irq_req_o,				
	input	wire   usr_h2c0irq_ack_i,	
	output  wire   [31:0]   usr_h2c0err_num_o
`ifdef DEBUG_MODE
	,output  wire   [`DT_WD*`IDT_NUM-1:0]   h2c_fm_odt
`endif
);
localparam DLT_DT = `DATA_WIDTH/8'd64;
//---Variable definition;
reg   h2c0w_run_d1;
wire  h2c0w_run_falling;
reg   h2c0_trdy;
wire  h2c0_dt_vld;
reg   h2c0_wren;
wire  shake_ok;
reg   [`DATA_WIDTH-1:0]   h2c0_din;
reg   [`DATA_WIDTH-1:0]   h2c0_tst_dt;
reg   [31:0]   usr_h2c0err_num;
integer j;
reg   [`DT_WD-1:0]   h2c_fm_odt_x[`IDT_NUM-1:0];
always@(posedge usr_clk,negedge usr_rst_n)
begin 
	if(!usr_rst_n) 
		h2c0w_run_d1 <= `DLY 1'b0;
	else 
		h2c0w_run_d1 <= `DLY usr_h2c0w_run_i;
end
assign h2c0w_run_falling = (~usr_h2c0w_run_i) & h2c0w_run_d1;
always@(posedge usr_clk,negedge usr_rst_n)
begin 
	if(!usr_rst_n) 
		h2c0_trdy <= `DLY 1'b0;
	else if(h2c0w_run_falling | m0_axis_h2c_rst_i)
		h2c0_trdy <= `DLY 1'b0;
//`ifndef DEBUG_MODE
	else if(usr_h2c0w_run_i)
		h2c0_trdy <= `DLY 1'b1;//---($random%2)
//`else
//    else if(~usr_h2c0w_run_i)
//		h2c0_trdy <= `DLY 1'b1;//---($random%2)
//`endif
	else ;
end
assign m0_axis_h2c_tready_o = h2c0_trdy;
//---C2H Mode;
assign h2c0_dt_vld = h2c0_trdy & m0_axis_h2c_tvalid_i;
always@(posedge usr_clk,negedge usr_rst_n)
begin 
	if(!usr_rst_n) 
		h2c0_wren <= `DLY 1'b0;
	else 
		h2c0_wren <= `DLY h2c0_dt_vld;
end
//---Secondly,generate test data;
`ifdef DEBUG_MODE
always@(posedge usr_clk,negedge usr_rst_n)
begin 
	if(!usr_rst_n)
		h2c0_tst_dt <= `DLY {`DATA_WIDTH{1'b0}};
	else if(h2c0w_run_falling | m0_axis_h2c_rst_i)
		h2c0_tst_dt <= `DLY {`DATA_WIDTH{1'b0}};
	else if(h2c0_wren)
		h2c0_tst_dt <= `DLY h2c0_tst_dt + 1'b1;
	else ;
end
`else
always@(posedge usr_clk,negedge usr_rst_n)
begin 
	if(!usr_rst_n)
		h2c0_tst_dt <= `DLY {`DATA_WIDTH{1'b0}};
	else if(h2c0w_run_falling | m0_axis_h2c_rst_i)
		h2c0_tst_dt <= `DLY {`DATA_WIDTH{1'b0}};
	else if(h2c0_wren)
		h2c0_tst_dt <= `DLY h2c0_tst_dt + {24'd0,DLT_DT};
	else ;
end
`endif
always@(posedge usr_clk,negedge usr_rst_n)
begin 
	if(!usr_rst_n) 
		h2c0_din <= `DLY {`DATA_WIDTH{1'b0}};
	else if(h2c0_dt_vld)
		h2c0_din <= `DLY m0_axis_h2c_tdata_i[`DATA_WIDTH-1:0];//don't use tkeep?
	else ;
end
`ifndef DEBUG_MODE
//---Compare test data with h2c data, and output test results;
always@(posedge usr_clk,negedge usr_rst_n)
begin 
	if(!usr_rst_n)
		usr_h2c0err_num <= `DLY 32'd0;
	else if(h2c0w_run_falling | m0_axis_h2c_rst_i)
		usr_h2c0err_num <= `DLY 32'd0;
	else if(h2c0_wren==1'b1 && h2c0_tst_dt[31:0]!=h2c0_din[31:0])
		usr_h2c0err_num <= `DLY usr_h2c0err_num + 32'd1;
	else ;
end
assign usr_h2c0err_num_o  = usr_h2c0err_num;
assign h2c_fm_odt = 'd0;
`else 
always@(posedge usr_clk or negedge usr_rst_n)
begin
	if(!usr_rst_n) begin
		for (j=0;j<`IDT_NUM;j=j+1)
		begin
			h2c_fm_odt_x[j] <= {`DT_WD{1'b0}};
		end
	end
	else if(h2c0_wren) 
		h2c_fm_odt_x[h2c0_tst_dt] <= h2c0_din;
	else ;
end
generate
genvar h2c_nidt;
	for(h2c_nidt=0;h2c_nidt<`IDT_NUM;h2c_nidt=h2c_nidt+1) 
	begin:H2C_IDT_PACK
        assign h2c_fm_odt[h2c_nidt*`DT_WD+`DT_WD-1:h2c_nidt*`DT_WD] = h2c_fm_odt_x[h2c_nidt];
	end
endgenerate
assign usr_h2c0err_num_o = 32'd0;
`endif
assign usr_h2c0irq_req_o = 1'b0;

//debug
reg  [7:0]  check_dat;
reg         cop_err0;//synthesis no_keep
reg         cop_err1;//synthesis no_keep
reg         cop_err2;//synthesis no_keep
reg         cop_err3;//synthesis no_keep
reg         cop_err4;//synthesis no_keep
reg         cop_err5;//synthesis no_keep
reg         cop_err6;//synthesis no_keep
reg         cop_err7;//synthesis no_keep

assign shake_ok = m0_axis_h2c_tready_o & m0_axis_h2c_tvalid_i;

always @ ( posedge usr_clk or negedge usr_rst_n )
begin
	if(!usr_rst_n)
    	check_dat <= 8'd0;
    else if(h2c0w_run_falling | m0_axis_h2c_rst_i)
    	check_dat <= 8'd0;
    else if ( shake_ok )
   	begin
       case ( m0_axis_h2c_tkeep_i )
           8'h01: check_dat <= check_dat + 8'd1;
           8'h03: check_dat <= check_dat + 8'd2;
           8'h07: check_dat <= check_dat + 8'd3;
           8'h0f: check_dat <= check_dat + 8'd4;
           8'h1f: check_dat <= check_dat + 8'd5;
           8'h3f: check_dat <= check_dat + 8'd6;
           8'h7f: check_dat <= check_dat + 8'd7;
           8'hff: check_dat <= check_dat + 8'd8;
           default: ;
       endcase
    end
end

always @ ( posedge usr_clk or negedge usr_rst_n )
begin	
    if(!usr_rst_n)
    	cop_err0 <= 1'b0;
    else if ( shake_ok && m0_axis_h2c_tkeep_i[0] )
    begin
    	if ( check_dat == m0_axis_h2c_tdata_i[7:0] )
			cop_err0 <= 1'b0;
        else
        	cop_err0 <= 1'b1;
    end
    else
    	;
end

always @ ( posedge usr_clk or negedge usr_rst_n )
begin	
    if(!usr_rst_n)
    	cop_err1 <= 1'b0;
    else if ( shake_ok && m0_axis_h2c_tkeep_i[1] )
    begin
    	if ( check_dat + 8'd1 == m0_axis_h2c_tdata_i[15:8] )
			cop_err1 <= 1'b0;
        else
        	cop_err1 <= 1'b1;
    end
    else
    	;
end

always @ ( posedge usr_clk or negedge usr_rst_n )
begin	
    if(!usr_rst_n)
    	cop_err2 <= 1'b0;
    else if ( shake_ok && m0_axis_h2c_tkeep_i[2] )
    begin
    	if ( check_dat + 8'd2 == m0_axis_h2c_tdata_i[23:16] )
			cop_err2 <= 1'b0;
        else
        	cop_err2 <= 1'b1;
    end
    else
    	;
end

always @ ( posedge usr_clk or negedge usr_rst_n )
begin	
    if(!usr_rst_n)
    	cop_err3 <= 1'b0;
    else if ( shake_ok && m0_axis_h2c_tkeep_i[3] )
    begin
    	if ( check_dat + 8'd3 == m0_axis_h2c_tdata_i[31:24] )
			cop_err3 <= 1'b0;
        else
        	cop_err3 <= 1'b1;
    end
    else
    	;
end

always @ ( posedge usr_clk or negedge usr_rst_n )
begin	
    if(!usr_rst_n)
    	cop_err4 <= 1'b0;
    else if ( shake_ok && m0_axis_h2c_tkeep_i[4] )
    begin
    	if ( check_dat + 8'd4 == m0_axis_h2c_tdata_i[39:32] )
			cop_err4 <= 1'b0;
        else
        	cop_err4 <= 1'b1;
    end
    else
    	;
end

always @ ( posedge usr_clk or negedge usr_rst_n )
begin	
    if(!usr_rst_n)
    	cop_err5 <= 1'b0;
    else if ( shake_ok && m0_axis_h2c_tkeep_i[5] )
    begin
    	if ( check_dat + 8'd5 == m0_axis_h2c_tdata_i[47:40] )
			cop_err5 <= 1'b0;
        else
        	cop_err5 <= 1'b1;
    end
    else
    	;
end

always @ ( posedge usr_clk or negedge usr_rst_n )
begin	
    if(!usr_rst_n)
    	cop_err6 <= 1'b0;
    else if ( shake_ok && m0_axis_h2c_tkeep_i[6] )
    begin
    	if ( check_dat + 8'd6 == m0_axis_h2c_tdata_i[55:48] )
			cop_err6 <= 1'b0;
        else
        	cop_err6 <= 1'b1;
    end
    else
    	;
end

always @ ( posedge usr_clk or negedge usr_rst_n )
begin	
    if(!usr_rst_n)
    	cop_err7 <= 1'b0;
    else if ( shake_ok && m0_axis_h2c_tkeep_i[7] )
    begin
    	if ( check_dat + 8'd7 == m0_axis_h2c_tdata_i[63:56] )
			cop_err7 <= 1'b0;
        else
        	cop_err7 <= 1'b1;
    end
    else
    	;
end

endmodule
