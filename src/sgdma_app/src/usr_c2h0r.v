`ifdef DEBUG_MODE
	`include "para_def.vh"
	`include "dt_para.vh"
`endif

module	usr_c2h0r(
   	input   wire   usr_clk,
	input   wire   usr_rst_n,
	
	input   wire   usr_c2h0r_run_i,
	input   wire   s0_axis_c2h_rst_i,
`ifdef DEBUG_MODE	
	input   wire   [`DT_WD*`ODT_NUM-1:0]   c2h_fm_idt,
`endif
	input   wire   s0_axis_c2h_tready_i,
    output  wire   [`DATA_WIDTH-1:0]  s0_axis_c2h_tdata_o,
    output  wire   [`KEEP_WIDTH-1:0]  s0_axis_c2h_tkeep_o,
    output  wire   [`KEEP_WIDTH-1:0]  s0_axis_c2h_tuser_o,
    output  wire   s0_axis_c2h_tlast_o,
    output  wire   s0_axis_c2h_tvalid_o,
	
	output	wire   usr_c2h0irq_req_o,				
	input	wire   usr_c2h0irq_ack_i,
	output  wire   usr_c2h0err_o
);

//---Constant definition;
localparam C2H_TRDY_TIME = 8'd200,
		   C2H_DLY_TIME = 8'd1,
		   C2H_PKT_LEN = 13'd4096;//????
localparam DLT_DT = `DATA_WIDTH/8'd64;   
//---Variable definition;
reg    c2h0r_run_d1;
wire   c2h0r_run_rising;
wire   c2h0_hs_ok;
wire   c2h0_tlastv;
reg    c2h0_nxt_strt;
wire   [12:0]  c2h0_len_x;
reg    [12:0]  c2h0_cnt_len;
reg    c2h0_tvalid;
reg    c2h0_tlast;
reg    [31:0]   c2h0_tdata0;
reg    [31:0]   c2h0_tdata1;
wire   c2h0_strt;
//reg    c2h0_strt_d1;
reg    c2h0_tvalid_d1;
wire   c2h0_tvalid_rising;
reg    c2h0_tvalid_rgn;
reg    [7:0]   c2h0_cnt_clk;
wire   c2h0_trdy_timeout;
reg    usr_c2h0err;
wire   [`DT_WD-1:0]   c2h_fm_idt_x[`ODT_NUM-1:0];
wire   c2h0_pkt_stp_x;

`ifdef CASE4 
    localparam C2H_PKT_NUM = 8'd8;
    reg    c2h0_pkt_stp;
    reg    [7:0]   c2h0_cnt_pkt;
    always @(posedge usr_clk,negedge usr_rst_n)
    begin
	    if(!usr_rst_n) 
		    c2h0_cnt_pkt <= `DLY 8'd0;
	    else if(s0_axis_c2h_rst_i)
		    c2h0_cnt_pkt <= `DLY 8'd0;
	    else if(c2h0_tlast)
		    c2h0_cnt_pkt <= `DLY c2h0_cnt_pkt + 8'd1;
	    else ;
    end
    always @(posedge usr_clk,negedge usr_rst_n)
    begin
	    if(!usr_rst_n) 
		    c2h0_pkt_stp <= `DLY 1'b0;
	    else if(s0_axis_c2h_rst_i)
		    c2h0_pkt_stp <= `DLY 1'b0;
	    else if(c2h0_tlast==1'b1 && c2h0_cnt_pkt==C2H_PKT_NUM-1)
		    c2h0_pkt_stp <= `DLY 1'b1;
	    else ;
    end
    assign c2h0_pkt_stp_x = c2h0_pkt_stp;
`else
    assign c2h0_pkt_stp_x = 1'b0;
`endif


assign usr_c2h0irq_req_o = 1'b0;
//---Capture the rising edge and falling of c2h0r_run_i
// so that we can konow the start and end of the process;
always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n)
		c2h0r_run_d1 <= `DLY 1'b0;
	else 
		c2h0r_run_d1 <= `DLY usr_c2h0r_run_i;
end
assign c2h0r_run_rising = (~c2h0r_run_d1) & usr_c2h0r_run_i;

assign c2h0_hs_ok = c2h0_tvalid & s0_axis_c2h_tready_i;
assign c2h0_tlastv = c2h0_tlast & c2h0_hs_ok;

//---Generate tlp delay for every packet which the range is 0cycle 20 4095cycle; 
/*always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n)
		c2h0_dly_en <= `DLY 1'b0;
	else if(c2h0_dly_over | s0_axis_c2h_rst_i)
		c2h0_dly_en <= `DLY 1'b0;
	else if(c2h0_tlastv)
		c2h0_dly_en <= `DLY 1'b1;
	else ;
end
always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n)
		c2h0_cnt_dly <= `DLY 13'd0;
	else if(c2h0_dly_over | s0_axis_c2h_rst_i)
		c2h0_cnt_dly <= `DLY 13'd0;
	else if(c2h0_dly_en)
		c2h0_cnt_dly <= `DLY c2h0_cnt_dly + 13'd1;
	else ;
end
assign c2h0_dly_over = (c2h0_dly_en==1'b1&&c2h0_cnt_dly==C2H_DLY_TIME-1) ? 1'b1 : 1'b0;
*/	
always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n) 
		c2h0_nxt_strt <= `DLY 1'b0;
	else 
		c2h0_nxt_strt <= `DLY (~c2h0_pkt_stp_x) & c2h0_tlastv;
end
assign c2h0_strt = c2h0r_run_rising | c2h0_nxt_strt;
/*always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n) 
		c2h0_strt_d1 <= `DLY 1'b0;
	else 
		c2h0_strt_d1 <= `DLY c2h0_strt;
end*/
//---Generate tlp length which the range is 1byte to 4096byte;
always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n)
		c2h0_cnt_len <= `DLY 13'd0;
	else if(c2h0_tlastv | s0_axis_c2h_rst_i)
		c2h0_cnt_len <= `DLY 13'd0;
	else if(c2h0_hs_ok)
		c2h0_cnt_len <= `DLY c2h0_cnt_len + 13'd1;
	else ;
end

always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n)
		c2h0_tvalid <= `DLY 1'b0;
	else if(c2h0_tlastv | s0_axis_c2h_rst_i)
		c2h0_tvalid <= `DLY 1'b0;
	else if(c2h0_strt)
		c2h0_tvalid <= `DLY 1'b1;
	else ;
end
assign c2h0_len_x = C2H_PKT_LEN;
assign c2h0_lst_dt_pre = (c2h0_hs_ok==1'b1&&c2h0_cnt_len==c2h0_len_x[12:`NBYTE_WIDTH]-'d2) ? 1'b1 : 1'b0;
always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n) 
		c2h0_tlast <= `DLY 1'b0;
	else if(c2h0_tlastv | s0_axis_c2h_rst_i)
		c2h0_tlast <= `DLY 1'b0;
	else if(c2h0_len_x[12:`NBYTE_WIDTH]>'d1) begin
		if(c2h0_lst_dt_pre)
			c2h0_tlast <= `DLY 1'b1;
		else ;
	end
	else if(c2h0_strt)
		c2h0_tlast <= `DLY 1'b1;
	else ;
end
`ifndef DEBUG_MODE
    always @(posedge usr_clk,negedge usr_rst_n)
    begin
	    if(!usr_rst_n)
		    c2h0_tdata0 <= `DLY 32'd0;
	    else if(s0_axis_c2h_rst_i)
		    c2h0_tdata0 <= `DLY 32'd0;
	    else if(c2h0_hs_ok) 
		    c2h0_tdata0 <= `DLY c2h0_tdata0 + {24'd0,DLT_DT};
	    else ;
    end
`else
    always @(posedge usr_clk,negedge usr_rst_n)
    begin
	    if(!usr_rst_n)
		    c2h0_tdata0 <= `DLY 32'd0;
	    else if(s0_axis_c2h_rst_i)
		    c2h0_tdata0 <= `DLY 32'd0;
	    else if(c2h0_hs_ok) 
		    c2h0_tdata0 <= `DLY c2h0_tdata0 + 1'b1;
	    else ;
    end
`endif

`ifndef DEBUG_MODE
	`ifdef PH1A400_DEV
		always @(posedge usr_clk,negedge usr_rst_n)
		begin
			if(!usr_rst_n)
				c2h0_tdata1 <= `DLY 32'd0;
			else if(s0_axis_c2h_rst_i)
				c2h0_tdata1 <= `DLY 32'd1;
			else if(c2h0_hs_ok) 
				c2h0_tdata1 <= `DLY c2h0_tdata1 + {24'd0,DLT_DT};
			else ;
		end
		assign s0_axis_c2h_tdata_o  = {{2{c2h0_tdata1}},{2{c2h0_tdata0}}};
  	`else
      	assign s0_axis_c2h_tdata_o  = {2{c2h0_tdata0}};
  	`endif
`else
    generate 
    genvar c2h0_ndt;
	    for(c2h0_ndt=0;c2h0_ndt<`ODT_NUM;c2h0_ndt=c2h0_ndt+1) 
	    begin:C2H0_DT_UNPACK
		    assign c2h_fm_idt_x[c2h0_ndt] = c2h_fm_idt[c2h0_ndt*`DT_WD+`DT_WD-1:c2h0_ndt*`DT_WD];
	    end
    endgenerate
    assign s0_axis_c2h_tdata_o  = c2h_fm_idt_x[c2h0_tdata0];
`endif

assign s0_axis_c2h_tvalid_o = c2h0_tvalid;
assign s0_axis_c2h_tlast_o = c2h0_tlast;
assign s0_axis_c2h_tkeep_o= {`KEEP_WIDTH{1'b1}};
assign s0_axis_c2h_tuser_o  = {`KEEP_WIDTH{1'b0}};

////////////////////////////////
//---Check the error of tready;
////////////////////////////////
always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n)
		c2h0_tvalid_d1 <= `DLY 1'b0;
	else 
		c2h0_tvalid_d1 <= `DLY c2h0_tvalid;
end
assign c2h0_tvalid_rising = (~c2h0_tvalid_d1) & c2h0_tvalid;
always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n)
		c2h0_tvalid_rgn <= `DLY 1'b0;
	else if(c2h0_hs_ok | c2h0_trdy_timeout)
		c2h0_tvalid_rgn <= `DLY 1'b0;
	else if(c2h0_tvalid_rising)
		c2h0_tvalid_rgn <= `DLY 1'b1;
	else ;
end
always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n)
		c2h0_cnt_clk <= `DLY 8'd0;
	else if(c2h0_trdy_timeout)
		c2h0_cnt_clk <= `DLY 8'd0;
	else if(c2h0_tvalid_rgn)
		c2h0_cnt_clk <= `DLY c2h0_cnt_clk + 8'd1;
	else ;
end
assign c2h0_trdy_timeout = (c2h0_cnt_clk==C2H_TRDY_TIME-1) ? 1'b1 : 1'b0;
always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n)
		usr_c2h0err <= `DLY 1'b0;
	else if(s0_axis_c2h_rst_i)
		usr_c2h0err <= `DLY 1'b0;
	else if(c2h0_trdy_timeout)
		usr_c2h0err <= `DLY 1'b1;
	else ;
end
assign usr_c2h0err_o = usr_c2h0err;


endmodule
