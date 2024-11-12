/*===========================================*\
Filename         : usr_regwr.v
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
module usr_regrw(
	input    wire   usr_rst_n,
	input	 wire   usr_clk,
			  
 	input   wire   m0_axis_h2c_rst_i,
	input   wire   s0_axis_c2h_rst_i,
                 
	input	 wire   usr_regrw_run_i,
//  output   wire   data_rst_flag,
	input    wire   [31:0]   m_axil_awaddr_i,
	input 	 wire   [2:0]    m_axil_awprot_i,	
	input 	 wire   m_axil_awvalid_i,		
	output   wire   m_axil_awready_o,		
	input 	 wire   [31:0]   m_axil_wdata_i,	
	input 	 wire   [3:0] 	  m_axil_wstrb_i,	
	input 	 wire   m_axil_wvalid_i,		
	output   wire   m_axil_wready_o,	
	output   wire   m_axil_bvalid_o,	
	output   wire   [1:0]    m_axil_bresp_o,	
	input 	 wire   m_axil_bready_i,	
	input 	 wire   [31:0]   m_axil_araddr_i,	
	input 	 wire   [2:0]    m_axil_arprot_i,	
	input    wire   m_axil_arvalid_i,		
	output   wire   m_axil_arready_o,	
	output   wire   [31:0]   m_axil_rdata_o,	
	output   wire   [1:0]    m_axil_rresp_o,	
	output   wire   m_axil_rvalid_o,		
	input    wire   m_axil_rready_i,
    
    input    wire   [31:0]   usr_h2c0err_num_i,
	output   wire   [1:0]    usr_lp0rw_md_o
);
//---Variable definition;
reg    m_axil_awready;
wire   aw_handshake_ok;
reg    [19:0]   bram_wraddr;
reg    m_axil_wready;
wire   w_handshake_ok;
reg    [31:0]   bram_di;
wire   [31:0]   bram_do;
reg    wt_wren_rgn0;
reg    wt_wren_rgn1;
wire   bram_wren;
wire   b_handshake_ok;
reg    m_axil_bvalid;
reg    [1:0]    m_axil_bresp;
reg    m_axil_arready;
wire   ar_handshake_ok;
wire   [19:0]   bram_rdaddr;
wire   bram_rden;
reg    bram_rden_d1;
reg    m_axil_rvalid;
wire   r_handshake_ok;
reg    [31:0]   m_axil_rdata;
reg    m_axil_rresp;
reg    data_rst_flag_temp;
reg    data_rst_flag_temp_ff;
reg    [19:0]   bram_rdaddr_d1;
reg    regrw_run_d1;
wire   regrw_run_falling;

always@(posedge usr_clk,negedge usr_rst_n)
begin 
	if(!usr_rst_n) 
		regrw_run_d1 <= `DLY 1'b0;
	else 
		regrw_run_d1 <= `DLY usr_regrw_run_i;
end
assign regrw_run_falling = (~usr_regrw_run_i) & regrw_run_d1;
//---Write address channel;
always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n)
		m_axil_awready <= `DLY 1'b0;
	else 
		m_axil_awready <= `DLY ~m_axil_awready;//---($random%2);
end
assign m_axil_awready_o = m_axil_awready;
assign aw_handshake_ok = m_axil_awready & m_axil_awvalid_i;
always @(posedge usr_clk)
begin
	if(aw_handshake_ok)
		bram_wraddr <= `DLY m_axil_awaddr_i[19:0];
	else ;
end
//---Write data channel;
always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n)
		m_axil_wready <= `DLY 1'b0;
	else 
		m_axil_wready <= `DLY ~m_axil_wready;//---($random%2);
end
assign m_axil_wready_o = m_axil_wready;
assign w_handshake_ok = m_axil_wready & m_axil_wvalid_i;
always @(posedge usr_clk)
begin
	if(w_handshake_ok)
		bram_di <= `DLY m_axil_wdata_i;
	else ;
end
always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n)
		wt_wren_rgn0 <= `DLY 1'b0;
	else if(bram_wren | regrw_run_falling)
		wt_wren_rgn0 <= `DLY 1'b0;
	else if(aw_handshake_ok)
		wt_wren_rgn0 <= `DLY 1'b1;
	else ;
end
always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n)
		wt_wren_rgn1 <= `DLY 1'b0;
	else if(bram_wren | regrw_run_falling)
		wt_wren_rgn1 <= `DLY 1'b0;
	else if(w_handshake_ok)
		wt_wren_rgn1 <= `DLY 1'b1;
	else ;
end	
assign bram_wren = wt_wren_rgn0 & wt_wren_rgn1;
//---Write response channel;
assign b_handshake_ok = m_axil_bvalid & m_axil_bready_i;
always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n)
		m_axil_bvalid <= `DLY 1'b0;
	else if(b_handshake_ok | regrw_run_falling)
		m_axil_bvalid <= `DLY 1'b0;
	else if(bram_wren)
		m_axil_bvalid <= `DLY 1'b1;
	else ;
end
assign m_axil_bvalid_o = m_axil_bvalid;
always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n)
		m_axil_bresp <= `DLY 2'b00;
	else if(regrw_run_falling)
		m_axil_bresp <= `DLY 2'b00;
	else if(bram_wren)
		m_axil_bresp <= `DLY (bram_wraddr[19:16]!=`USR_BS_ADDRH) ? 2'b11 : 2'b00;
	else ;
end
assign m_axil_bresp_o = m_axil_bresp;

ram512x32 u_ram512x32( 
	.dia(bram_di), 
	.addra(bram_wraddr[10:2]), 
	.cea(bram_wren), 
	.clka(usr_clk),
    .dob(Data_RAM_RD_Data), 
	.addrb(Data_RAM_RD_Addr), 
	.ceb(1'b1)
    /*
	.dob(bram_do), 
	.addrb(bram_rdaddr[10:2]), 
	.ceb(bram_rden)*/
);

wire  [7:0]   Num0;
wire  [7:0]   Num1;
wire  [7:0]   Num2;
wire  [7:0]   Num3;
wire  [7:0]   Num4;
wire  [7:0]   Num5;
wire  [7:0]   Num6;
wire  [7:0]   Num7;
wire  [7:0]   Num8;
wire  [7:0]   Num9;

wire ACC_VALID_1;
assign ACC_VALID_1 = (bram_di == 1'b1) & (bram_wraddr[9:2] == 8'h32);

wire ACC_VALID_2;
assign ACC_VALID_2 = (bram_di == 1'b1) & (bram_wraddr[9:2] == 8'h65);

wire ACC_VALID_3;
assign ACC_VALID_3 = (bram_di == 1'b1) & (bram_wraddr[9:2] == 8'h98);

wire ACC_VALID_4;
assign ACC_VALID_4 = (bram_di == 1'b1) & (bram_wraddr[9:2] == 8'hcb);

wire ACC_VALID_5;
assign ACC_VALID_5 = (bram_di == 1'b1) & (bram_wraddr[9:2] == 8'hfd);

wire  [7:0]   Data_RAM_RD_Addr_1;
wire  [23:0]  Data_RAM_RD_Data_1;
wire  [7:0]   Num0_1;
wire  [7:0]   Num1_1;
wire  [7:0]   Num2_1;
wire  [7:0]   Num3_1;
wire  [7:0]   Num4_1;
wire  [7:0]   Num5_1;
wire  [7:0]   Num6_1;
wire  [7:0]   Num7_1;
wire  [7:0]   Num8_1;
wire  [7:0]   Num9_1;

ACC     ACC_1(
        .clk                            (usr_clk),
        .rstn                           (usr_rst_n),
        .AccValid_i                     (ACC_VALID_1),
        .AccReady_o                     (ACC_READY_1),
        .DataRamAddr_o                  (Data_RAM_RD_Addr_1),
        .DataRamData_i                  (Data_RAM_RD_Data_1),
        .Num0_o                         (Num0_1),
        .Num1_o                         (Num1_1),
        .Num2_o                         (Num2_1),
        .Num3_o                         (Num3_1),
        .Num4_o                         (Num4_1),
        .Num5_o                         (Num5_1),
        .Num6_o                         (Num6_1),
        .Num7_o                         (Num7_1),
        .Num8_o                         (Num8_1),
        .Num9_o                         (Num9_1)
);

wire  [7:0]   Data_RAM_RD_Addr_2;
wire  [23:0]  Data_RAM_RD_Data_2;
wire  [7:0]   Num0_2;
wire  [7:0]   Num1_2;
wire  [7:0]   Num2_2;
wire  [7:0]   Num3_2;
wire  [7:0]   Num4_2;
wire  [7:0]   Num5_2;
wire  [7:0]   Num6_2;
wire  [7:0]   Num7_2;
wire  [7:0]   Num8_2;
wire  [7:0]   Num9_2;

ACC     ACC_2(
        .clk                            (usr_clk),
        .rstn                           (usr_rst_n),
        .AccValid_i                     (ACC_VALID_2),
        .AccReady_o                     (ACC_READY_2),
        .DataRamAddr_o                  (Data_RAM_RD_Addr_2),
        .DataRamData_i                  (Data_RAM_RD_Data_2),
        .Num0_o                         (Num0_2),
        .Num1_o                         (Num1_2),
        .Num2_o                         (Num2_2),
        .Num3_o                         (Num3_2),
        .Num4_o                         (Num4_2),
        .Num5_o                         (Num5_2),
        .Num6_o                         (Num6_2),
        .Num7_o                         (Num7_2),
        .Num8_o                         (Num8_2),
        .Num9_o                         (Num9_2)
);

wire  [7:0]   Data_RAM_RD_Addr_3;
wire  [23:0]  Data_RAM_RD_Data_3;
wire  [7:0]   Num0_3;
wire  [7:0]   Num1_3;
wire  [7:0]   Num2_3;
wire  [7:0]   Num3_3;
wire  [7:0]   Num4_3;
wire  [7:0]   Num5_3;
wire  [7:0]   Num6_3;
wire  [7:0]   Num7_3;
wire  [7:0]   Num8_3;
wire  [7:0]   Num9_3;

ACC     ACC_3(
        .clk                            (usr_clk),
        .rstn                           (usr_rst_n),
        .AccValid_i                     (ACC_VALID_3),
        .AccReady_o                     (ACC_READY_3),
        .DataRamAddr_o                  (Data_RAM_RD_Addr_3),
        .DataRamData_i                  (Data_RAM_RD_Data_3),
        .Num0_o                         (Num0_3),
        .Num1_o                         (Num1_3),
        .Num2_o                         (Num2_3),
        .Num3_o                         (Num3_3),
        .Num4_o                         (Num4_3),
        .Num5_o                         (Num5_3),
        .Num6_o                         (Num6_3),
        .Num7_o                         (Num7_3),
        .Num8_o                         (Num8_3),
        .Num9_o                         (Num9_3)
);
/*
wire  [7:0]   Data_RAM_RD_Addr_4;
wire  [23:0]  Data_RAM_RD_Data_4;
wire  [7:0]   Num0_4;
wire  [7:0]   Num1_4;
wire  [7:0]   Num2_4;
wire  [7:0]   Num3_4;
wire  [7:0]   Num4_4;
wire  [7:0]   Num5_4;
wire  [7:0]   Num6_4;
wire  [7:0]   Num7_4;
wire  [7:0]   Num8_4;
wire  [7:0]   Num9_4;

ACC     ACC_4(
        .clk                            (usr_clk),
        .rstn                           (usr_rst_n),
        .AccValid_i                     (ACC_VALID_4),
        .AccReady_o                     (ACC_READY_4),
        .DataRamAddr_o                  (Data_RAM_RD_Addr_4),
        .DataRamData_i                  (Data_RAM_RD_Data_4),
        .Num0_o                         (Num0_4),
        .Num1_o                         (Num1_4),
        .Num2_o                         (Num2_4),
        .Num3_o                         (Num3_4),
        .Num4_o                         (Num4_4),
        .Num5_o                         (Num5_4),
        .Num6_o                         (Num6_4),
        .Num7_o                         (Num7_4),
        .Num8_o                         (Num8_4),
        .Num9_o                         (Num9_4)
);

wire  [7:0]   Data_RAM_RD_Addr_5;
wire  [23:0]  Data_RAM_RD_Data_5;
wire  [7:0]   Num0_5;
wire  [7:0]   Num1_5;
wire  [7:0]   Num2_5;
wire  [7:0]   Num3_5;
wire  [7:0]   Num4_5;
wire  [7:0]   Num5_5;
wire  [7:0]   Num6_5;
wire  [7:0]   Num7_5;
wire  [7:0]   Num8_5;
wire  [7:0]   Num9_5;

ACC     ACC_5(
        .clk                            (usr_clk),
        .rstn                           (usr_rst_n),
        .AccValid_i                     (ACC_VALID_5),
        .AccReady_o                     (ACC_READY_5),
        .DataRamAddr_o                  (Data_RAM_RD_Addr_5),
        .DataRamData_i                  (Data_RAM_RD_Data_5),
        .Num0_o                         (Num0_5),
        .Num1_o                         (Num1_5),
        .Num2_o                         (Num2_5),
        .Num3_o                         (Num3_5),
        .Num4_o                         (Num4_5),
        .Num5_o                         (Num5_5),
        .Num6_o                         (Num6_5),
        .Num7_o                         (Num7_5),
        .Num8_o                         (Num8_5),
        .Num9_o                         (Num9_5)
);*/
//--------------------------------------------------
// Result test
//--------------------------------------------------
wire [7:0] result_test;
assign result_test = (bram_wraddr[9:2] == 8'hff) ? bram_di : 8'h00;
                    
//--------------------------------------------------
// READ OUT
//--------------------------------------------------
wire [31:0] bram_do_temp;
assign  bram_do_temp  =  (bram_rdaddr_d1[9:2]   ==  8'h0a)   ?   {4{result_test}}:   (
                    (bram_rdaddr_d1[9:2]   ==  8'h00)   ?   {4{Num0_1}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h01)   ?   {4{Num1_1}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h02)   ?   {4{Num2_1}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h03)   ?   {4{Num3_1}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h04)   ?   {4{Num4_1}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h05)   ?   {4{Num5_1}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h06)   ?   {4{Num6_1}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h07)   ?   {4{Num7_1}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h08)   ?   {4{Num8_1}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h09)   ?   {4{Num9_1}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h10)   ?   {4{Num0_2}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h11)   ?   {4{Num1_2}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h12)   ?   {4{Num2_2}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h13)   ?   {4{Num3_2}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h14)   ?   {4{Num4_2}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h15)   ?   {4{Num5_2}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h16)   ?   {4{Num6_2}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h17)   ?   {4{Num7_2}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h18)   ?   {4{Num8_2}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h19)   ?   {4{Num9_2}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h20)   ?   {4{Num0_3}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h21)   ?   {4{Num1_3}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h22)   ?   {4{Num2_3}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h23)   ?   {4{Num3_3}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h24)   ?   {4{Num4_3}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h25)   ?   {4{Num5_3}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h26)   ?   {4{Num6_3}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h27)   ?   {4{Num7_3}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h28)   ?   {4{Num8_3}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h29)   ?   {4{Num9_3}}      :   
                     /*   
                    (bram_rdaddr_d1[9:2]   ==  8'h30)   ?   {4{Num0_4}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h31)   ?   {4{Num1_4}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h32)   ?   {4{Num2_4}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h33)   ?   {4{Num3_4}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h34)   ?   {4{Num4_4}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h35)   ?   {4{Num5_4}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h36)   ?   {4{Num6_4}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h37)   ?   {4{Num7_4}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h38)   ?   {4{Num8_4}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h39)   ?   {4{Num9_4}}      :   
                    
                    (bram_rdaddr_d1[9:2]   ==  8'h40)   ?   {4{Num0_5}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h41)   ?   {4{Num1_5}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h42)   ?   {4{Num2_5}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h43)   ?   {4{Num3_5}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h44)   ?   {4{Num4_5}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h45)   ?   {4{Num5_5}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h46)   ?   {4{Num6_5}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h47)   ?   {4{Num7_5}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h48)   ?   {4{Num8_5}}      :   (
                    (bram_rdaddr_d1[9:2]   ==  8'h49)   ?   {4{Num9_5}}      :    
                    */                
                    32'b0))))))))))))))))))))))))))))));//))))))))));//))))))))));
                    
assign bram_do =  bram_rden_d1 ? bram_do_temp : 32'b0;             
//---Read address channel;
always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n)
		m_axil_arready <= `DLY 1'b0;
	else 
		m_axil_arready <= `DLY ~m_axil_arready;//---($random%2);
end
assign m_axil_arready_o = m_axil_arready;
assign ar_handshake_ok = m_axil_arvalid_i & m_axil_arready;
assign bram_rdaddr = m_axil_araddr_i[19:0];
assign bram_rden = ar_handshake_ok;
always @(posedge usr_clk)
begin
	if(ar_handshake_ok)
	    bram_rdaddr_d1 <= `DLY bram_rdaddr;
    else ;
end
always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n) 
		bram_rden_d1 <= `DLY 1'b0;
	else 
		bram_rden_d1 <= `DLY bram_rden;
end
//---Read response channel;
always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n)
		m_axil_rvalid <= `DLY 1'b0;
	else if(r_handshake_ok | regrw_run_falling)
		m_axil_rvalid <= `DLY 1'b0;
	else if(bram_rden_d1)
		m_axil_rvalid <= `DLY 1'b1;
	else ;
end
assign m_axil_rvalid_o = m_axil_rvalid;
assign r_handshake_ok = m_axil_rvalid & m_axil_rready_i;

always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n)
		m_axil_rdata <= `DLY 32'd0;
	else if(bram_rden_d1) begin
    	if(bram_rdaddr_d1[15:2]==14'd1) 
        	m_axil_rdata <= `DLY usr_h2c0err_num_i;
        else	
        	m_axil_rdata <= `DLY bram_do;
    end
	else ;
end

assign m_axil_rdata_o = m_axil_rdata;
always @(posedge usr_clk,negedge usr_rst_n)
begin
	if(!usr_rst_n)
		m_axil_rresp <= `DLY 2'b00;
	else if(regrw_run_falling)
		m_axil_rresp <= `DLY 2'b00;
	else if(bram_rden_d1)
		m_axil_rresp <= `DLY (bram_rdaddr_d1[19:16]!=`USR_BS_ADDRH) ? 2'b11 : 2'b00;
	else ;
end
assign m_axil_rresp_o = m_axil_rresp;

//---Define loopback mode;
reg   [1:0]   usr_lp0rw_md;
always@(posedge usr_clk,negedge usr_rst_n)
begin 
	if(!usr_rst_n) 
    	usr_lp0rw_md <= `DLY 2'b00;
    else if(bram_wren==1'b1 && bram_wraddr[15:0]==16'd0)
		usr_lp0rw_md <= bram_di[1:0];
	else ;
end
assign usr_lp0rw_md_o = usr_lp0rw_md;
/*
assign Num0 = ACC_VALID_1 ? Num0_1 : (
              ACC_VALID_2 ? Num0_2 : (
              ACC_VALID_3 ? Num0_3 : (
              ACC_VALID_4 ? Num0_4 : (
              ACC_VALID_5 ? Num0_5 : 8'h00))));

assign Num1 = ACC_VALID_1 ? Num1_1 : (
              ACC_VALID_2 ? Num1_2 : (
              ACC_VALID_3 ? Num1_3 : (
              ACC_VALID_4 ? Num1_4 : (
              ACC_VALID_5 ? Num1_5 : 8'h00))));

assign Num2 = ACC_VALID_1 ? Num2_1 : (
              ACC_VALID_2 ? Num2_2 : (
              ACC_VALID_3 ? Num2_3 : (
              ACC_VALID_4 ? Num2_4 : (
              ACC_VALID_5 ? Num2_5 : 8'h00))));

assign Num3 = ACC_VALID_1 ? Num3_1 : (
              ACC_VALID_2 ? Num3_2 : (
              ACC_VALID_3 ? Num3_3 : (
              ACC_VALID_4 ? Num3_4 : (
              ACC_VALID_5 ? Num3_5 : 8'h00))));

assign Num4 = ACC_VALID_1 ? Num4_1 : (
              ACC_VALID_2 ? Num4_2 : (
              ACC_VALID_3 ? Num4_3 : (
              ACC_VALID_4 ? Num4_4 : (
              ACC_VALID_5 ? Num4_5 : 8'h00))));

assign Num5 = ACC_VALID_1 ? Num5_1 : (
              ACC_VALID_2 ? Num5_2 : (
              ACC_VALID_3 ? Num5_3 : (
              ACC_VALID_4 ? Num5_4 : (
              ACC_VALID_5 ? Num5_5 : 8'h00))));

assign Num6 = ACC_VALID_1 ? Num6_1 : (
              ACC_VALID_2 ? Num6_2 : (
              ACC_VALID_3 ? Num6_3 : (
              ACC_VALID_4 ? Num6_4 : (
              ACC_VALID_5 ? Num6_5 : 8'h00))));

assign Num7 = ACC_VALID_1 ? Num7_1 : (
              ACC_VALID_2 ? Num7_2 : (
              ACC_VALID_3 ? Num7_3 : (
              ACC_VALID_4 ? Num7_4 : (
              ACC_VALID_5 ? Num7_5 : 8'h00))));

assign Num8 = ACC_VALID_1 ? Num8_1 : (
              ACC_VALID_2 ? Num8_2 : (
              ACC_VALID_3 ? Num8_3 : (
              ACC_VALID_4 ? Num8_4 : (
              ACC_VALID_5 ? Num8_5 : 8'h00))));

assign Num9 = ACC_VALID_1 ? Num9_1 : (
              ACC_VALID_2 ? Num9_2 : (
              ACC_VALID_3 ? Num9_3 : (
              ACC_VALID_4 ? Num9_4 : (
              ACC_VALID_5 ? Num9_5 : 8'h00))));*/

assign Data_RAM_RD_Addr = ACC_VALID_1 ? Data_RAM_RD_Addr_1 : (
                          ACC_VALID_2 ? Data_RAM_RD_Addr_2 + 8'h33 : (
                          ACC_VALID_3 ? Data_RAM_RD_Addr_3 + 8'h66 : 8'h00 ));
//                          ACC_VALID_4 ? Data_RAM_RD_Addr_4 + 8'h99 : (8'h00 ))));
//                          ACC_VALID_5 ? Data_RAM_RD_Addr_5 + 8'hcc : 8'h00 ))));

assign Data_RAM_RD_Data_1 = ACC_VALID_1 ? Data_RAM_RD_Data : 24'h000000;
assign Data_RAM_RD_Data_2 = ACC_VALID_2 ? Data_RAM_RD_Data : 24'h000000;
assign Data_RAM_RD_Data_3 = ACC_VALID_3 ? Data_RAM_RD_Data : 24'h000000;
//assign Data_RAM_RD_Data_4 = ACC_VALID_4 ? Data_RAM_RD_Data : 24'h000000;
//assign Data_RAM_RD_Data_5 = ACC_VALID_5 ? Data_RAM_RD_Data : 24'h000000;

endmodule
/*	
	output   [2:0]    usr_prg_payload_o,
	output   [2:0]    usr_prg_read_o,
	
	input    [31:0]   usr_h2c0_nerr_i, 
	input	 [2:0]	  usr_eff_payload_i,
	input	 [2:0]	  usr_eff_read_i	
reg	   [31:0]	usr_wraddr;

reg    [2:0]    usr_prg_payload;
reg    [2:0]    usr_prg_read;
reg    m_axil_bvalid;
reg    m_axil_rvalid;
reg    [31:0]   m_axil_rdata;

assign m_axil_awready_o = 1'b1;  

always @(posedge usr_clk)
begin 
	if( m_axil_awvalid_i &  m_axil_awready_o )
		usr_wraddr	<= m_axil_awaddr_i;
	else ;
end

assign m_axil_wready_o = 1'b1;

always@(posedge usr_clk or negedge usr_rst_n) 
begin
	if(!usr_rst_n) begin
		usr_prg_payload <= 3'b001;
		usr_prg_read <= 3'b001;
	end
	else if(m_axil_wvalid_i & m_axil_wready_o) begin
		case (usr_wraddr[15:0])
			USR_PRG_PAYLOAD : usr_prg_payload <= m_axil_wdata_i[2:0];
			USR_PRG_READ : usr_prg_read <= m_axil_wdata_i[2:0];
			default: ;
		endcase
	end
	else ;
end
assign usr_prg_payload_o = usr_prg_payload;
assign usr_prg_read_o = usr_prg_read;

always@(posedge usr_clk or negedge usr_rst_n ) 
begin 
	if(!usr_rst_n)  
		m_axil_bvalid <= 1'b0;
	else if(m_axil_bvalid & m_axil_bready_i)  
		m_axil_bvalid <= 1'b0;
	else if(m_axil_wvalid_i & m_axil_wready_o)  
		m_axil_bvalid <= 1'b1;
	else ;
end
assign m_axil_bvalid_o = m_axil_bvalid;
assign m_axil_bresp_o = 2'b00;


assign m_axil_arready_o = 1'b1;

always@(posedge usr_clk or negedge usr_rst_n)
begin 
	if(!usr_rst_n)
		m_axil_rvalid <= 1'b0;
	else if(m_axil_rvalid & m_axil_rready_i)
		m_axil_rvalid <= 1'b0;
	else if(m_axil_arvalid_i & m_axil_arready_o)
		m_axil_rvalid <= 1'b1;
	else ;
end
assign m_axil_rvalid_o = m_axil_rvalid;

always@(posedge usr_clk or negedge usr_rst_n ) 
begin 
	if(!usr_rst_n)
		m_axil_rdata <= 32'd0;
	else if(m_axil_arvalid_i & m_axil_arready_o) begin
		case(m_axil_araddr_i[15:0])
			USR_PRG_PAYLOAD : m_axil_rdata <= {26'd0,usr_eff_payload_i,3'd0};
			USR_PRG_READ : m_axil_rdata <= {26'd0,usr_eff_read_i,3'd0};
			USR_H2C0_NERR : m_axil_rdata <= usr_h2c0_nerr_i;
			default : ; 
		endcase
	end
	else ;
end
assign m_axil_rdata_o = m_axil_rdata;
assign	m_axil_rresp_o = 2'b00;*/
