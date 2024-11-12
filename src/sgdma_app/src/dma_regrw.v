/*===========================================*\
Filename         : dma_regwr.v
Author           : 
Project name     : sgdma_app
Description      : 
Called by        : sgdma_subsystem_tst.v
Revision History : v1.0
Email            : 
Modified         : 
Copyright(c)Shanghai Anlu Information Technology Co.,Ltd
\*===========================================*/
`ifdef DEBUG_MODE
	`include "para_def.vh"
`endif
module	dma_regrw(
	input	wire   usr_rst_n,
	input	wire   usr_clk,
			
	output  wire   [31: 0] 	s_axil_awaddr_o	,
	output  wire   [2 : 0] 	s_axil_awprot_o	,
	output  wire   s_axil_awvalid_o	,		
	input   wire   s_axil_awready_i	,		
	output  wire   [31: 0] 	s_axil_wdata_o	,	
	output  wire   [3 : 0] 	s_axil_wstrb_o	,	
	output  wire   s_axil_wvalid_o	,		
	input   wire   s_axil_wready_i	,	
	input   wire   s_axil_bvalid_i	,	
	input   wire   [1 : 0] 	s_axil_bresp_i	,	
	output  wire   s_axil_bready_o	,	
	output  wire   [31: 0] 	s_axil_araddr_o	,	
	output  wire   [2 : 0] 	s_axil_arprot_o	,	
	output  wire   s_axil_arvalid_o	,		
	input   wire   s_axil_arready_i	,	
	input   wire   [31: 0] 	s_axil_rdata_i	,	
	input   wire   [1 : 0] 	s_axil_rresp_i	,	
	input   wire   s_axil_rvalid_i	,	
	output  wire   s_axil_rready_o	,
		
	input   wire   [1:0]    dma_regrw_run_i,	
	output  wire   [31:0]   h2c0_chn_stts_o,
	output  wire   [31:0]   c2h0_chn_stts_o
);
//---Constant definition;
localparam C2H0_CHN_STATES = 16'h1_0_40,
		   H2C0_CHN_STATES = 16'h0_0_40;
//---Variable definition;
reg    s_axil_arvalid;
wire   ar_handshake_ok;
reg    [31:0]    s_axil_araddr;
wire   r_handshake_ok;
reg    [31:0]    h2c0_chn_stts;
reg    [31:0]    c2h0_chn_stts;
wire   c2h0_stts_rdp;
wire   h2c0_stts_rdp;
reg    [1:0]     dma_regrw_run_d1;
wire   [1:0]     dma_regrw_run_rising;
always@(posedge usr_clk,negedge usr_rst_n)
begin 
	if(!usr_rst_n) 
    	dma_regrw_run_d1 <= `DLY 2'b00;
    else 
    	dma_regrw_run_d1 <= `DLY dma_regrw_run_i;
end
assign dma_regrw_run_rising = (~dma_regrw_run_d1) & dma_regrw_run_i;
assign c2h0_stts_rdp = dma_regrw_run_rising[0];
assign h2c0_stts_rdp = dma_regrw_run_rising[1];
//---Only test reading data from DMA Controller;
assign ar_handshake_ok = s_axil_arvalid & s_axil_arready_i;
always@(posedge usr_clk,negedge usr_rst_n)
begin 
	if(!usr_rst_n) 
		s_axil_arvalid <= `DLY 1'b0;
	else if(ar_handshake_ok) 
		s_axil_arvalid <= `DLY 1'b0;
	else if(c2h0_stts_rdp | h2c0_stts_rdp)
		s_axil_arvalid <= `DLY 1'b1;
	else ;
end 
always@(posedge usr_clk,negedge usr_rst_n)
begin 
	if(!usr_rst_n) 
		s_axil_araddr <= `DLY 16'd0;
	else begin
		case ({c2h0_stts_rdp,h2c0_stts_rdp})
			2'b10: s_axil_araddr <= `DLY C2H0_CHN_STATES;
			2'b01: s_axil_araddr <= `DLY H2C0_CHN_STATES;
			default: ;
		endcase
	end
end
assign r_handshake_ok = s_axil_rready_o & s_axil_rvalid_i;
always@(posedge usr_clk,negedge usr_rst_n)
begin 
	if(!usr_rst_n) begin
		h2c0_chn_stts <= `DLY 32'd0;
		c2h0_chn_stts <= `DLY 32'd0;
	end
	else if(r_handshake_ok) begin
		case (s_axil_araddr)
			C2H0_CHN_STATES: c2h0_chn_stts <= `DLY s_axil_rdata_i;
			H2C0_CHN_STATES: h2c0_chn_stts <= `DLY s_axil_rdata_i;
			default: ;
		endcase
	end
	else ;
end
assign h2c0_chn_stts_o = h2c0_chn_stts;
assign c2h0_chn_stts_o = c2h0_chn_stts;
assign s_axil_arprot_o = 3'b000;
assign s_axil_araddr_o = {16'd0,s_axil_araddr};
assign s_axil_arvalid_o = s_axil_arvalid;
assign s_axil_rready_o = 1'b1;

assign s_axil_awaddr_o = 32'd0;
assign s_axil_awprot_o = 3'd0;
assign s_axil_awvalid_o = 1'b0;
assign s_axil_wdata_o = 32'd0;
assign s_axil_wstrb_o = 4'hf;
assign s_axil_wvalid_o = 1'b0;
assign s_axil_bready_o = 1'b0;
endmodule

