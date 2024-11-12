/*===========================================*\
Filename         : cfg_regwr.v
Author           : 
Project name     : cfg_regrw.v
Description      : Test reading and writing of configuration space at zero address;
Called by        : sgdma_app.v
Revision History : v1.0
Email            : bingsong.wang@anlogic.com
Modified         : 
Copyright(c)Shanghai Anlu Information Technology Co.,Ltd
\*===========================================*/
module cfg_regrw(
   	input   wire   usr_clk,
	input   wire   usr_rst_n,
		
	output  wire   [18:0] 	cfg_mgmt_addr_o,
    output  wire   cfg_mgmt_write_o,
    output  wire   [31:0] 	cfg_mgmt_write_data_o,
    output  wire   [3 :0] 	cfg_mgmt_byte_enable_o,
    output  wire   cfg_mgmt_read_o,    
    input   wire   [31:0] 	cfg_mgmt_read_data_i,    
    input   wire   cfg_mgmt_read_write_done_i,
    output  wire   cfg_mgmt_type1_cfg_reg_access_o,
	
	input   wire   cfg_regrw_run_i,
	output	wire   pcie_cfg_tst_o					
);
//---Constant definition;
localparam TEST_CFG_ADDR = 19'd60;//---0x3c

//---Variable definition;
reg   cfg_mgmt_read;
reg   [31:0]   pcie_cfg_data;
reg   cfg_mgmt_write;
reg   [31:0]   cfg_mgmt_write_data;
reg   read_done;
reg   pcie_cfg_tst;
reg   cfg_regrw_run_d1;
wire  cfg_regrw_run_rising;
always@(posedge usr_clk,negedge usr_rst_n )
begin 
	if(!usr_rst_n)
    	cfg_regrw_run_d1 <= `DLY 1'b0;
    else 
    	cfg_regrw_run_d1 <= `DLY cfg_regrw_run_i;
end 
assign cfg_regrw_run_rising = (~cfg_regrw_run_d1) & cfg_regrw_run_i;
assign cfg_regrw_run_falling= (~cfg_regrw_run_i) & cfg_regrw_run_d1;
//---Firstly,read zero address;
always@(posedge usr_clk,negedge usr_rst_n )
begin 
	if(!usr_rst_n) 
		cfg_mgmt_read <= `DLY 1'b0;	
	else if((cfg_mgmt_read_write_done_i&cfg_mgmt_read) | cfg_regrw_run_falling) 	
		cfg_mgmt_read <= `DLY 1'b0;		
	else if(cfg_regrw_run_rising)		
		cfg_mgmt_read <= `DLY 1'b1;		
	else ;		
end	
assign cfg_mgmt_addr_o = TEST_CFG_ADDR;
always@(posedge usr_clk,negedge usr_rst_n )
begin 
	if(!usr_rst_n) 
		pcie_cfg_data <= `DLY 32'd0;	
	else if(cfg_mgmt_read_write_done_i & cfg_mgmt_read)		
		pcie_cfg_data <= `DLY cfg_mgmt_read_data_i;		
	else ;		
end		
always@(posedge usr_clk,negedge usr_rst_n )
begin 
	if(!usr_rst_n) 
		pcie_cfg_tst <= `DLY 1'b0;
	else 
		pcie_cfg_tst <= `DLY |pcie_cfg_data;	
end
assign pcie_cfg_tst_o = pcie_cfg_tst;//---If it is not equal to zero, the reading and writing is normal;

assign cfg_mgmt_byte_enable_o = 4'd0;
assign cfg_mgmt_type1_cfg_reg_access_o = 1'b0;
//---Then,write zero address;
always@(posedge usr_clk,negedge usr_rst_n )
begin 
	if(!usr_rst_n) 
		cfg_mgmt_write <= `DLY 1'b0;	
	else if((cfg_mgmt_read_write_done_i&cfg_mgmt_write) | cfg_regrw_run_falling)		
		cfg_mgmt_write <= `DLY 1'b0;		
	else if(read_done)		
		cfg_mgmt_write <= `DLY 1'b1;		
	else ;			
end	
always@(posedge usr_clk,negedge usr_rst_n )
begin 
	if(!usr_rst_n) 
		cfg_mgmt_write_data <= `DLY 32'd0;	
	else if(read_done)		
		cfg_mgmt_write_data <= `DLY pcie_cfg_data;	
	else ;		
end	
always@(posedge usr_clk,negedge usr_rst_n )
begin 
	if(!usr_rst_n) 
		read_done <= `DLY 1'b0;	
	else 		
		read_done <= `DLY cfg_mgmt_read_write_done_i & cfg_mgmt_read;	
end	

assign cfg_mgmt_write_data_o = cfg_mgmt_write_data; 
assign cfg_mgmt_write_o = cfg_mgmt_write;
assign cfg_mgmt_read_o = cfg_mgmt_read;


endmodule







	





