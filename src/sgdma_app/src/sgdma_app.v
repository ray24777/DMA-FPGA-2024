/*===========================================*\
Filename         : sgdma_app.v
Author           : 
Project name     : sgdma_subsystem_tst
Description      : 
Called by        : sgdma_subsystem_tst.v
Revision History : v1.0
Email            : bingsong.wang@anlogic.com
Modified         : 1.初版；2021-03-19
Copyright(c)Shanghai Anlu Information Technology Co.,Ltd
\*===========================================*/
/*
测试流程：1.驱动初始化完成后，APP通过cfg_mgmt总线读取`MAX_PAYLOAD_SIZE和max_read_request_size，
			依据这两个参数确认包大小；
          2.驱动配置好DMA读描述符表后，开始启动DMA读，m0_axis_h2c返回的数据存储到FIFOZ，FIFO大小暂定16个TLP大小（256*16*64bit），
		  3.APP接收完数据后，通过usrrq给出中断；
		  4.驱动配置好DMA写描述符表后，开始DMA写，FIFO中的数据通过s0_axis_c2h接口传递到PCIe；
		  5.APP发完数据后给，通过usrrq给出中断
		  6.PCIe比较上面两次的数据情况，给出测试结果。
		  注意：上述两次中断分别用bit0和bit1；
		        需要实现AXI-Lite master和slave，后面要用；
                usrrq中断由用户发起，没有发完数据也可以产生中断，需要考虑这种情况；				
*/
`ifdef DEBUG_MODE
	`include "para_def.vh"
	`include "dt_para.vh"
`endif

module sgdma_app
(
	input   wire   usr_clk,
	input   wire   usr_rst_n,
	
`ifdef AXIS_BUS0_EN
	input   wire   s0_axis_c2h_rst,	
    input   wire   m0_axis_h2c_rst,
    input   wire   s0_axis_c2h_run,
    input   wire   m0_axis_h2c_run,

	output  wire   m0_axis_h2c_tready,	
    input   wire   [`DATA_WIDTH-1:0] m0_axis_h2c_tdata,	
    input   wire   [`KEEP_WIDTH-1:0] m0_axis_h2c_tkeep,	
    input   wire   [`KEEP_WIDTH-1:0] m0_axis_h2c_tuser,	
    input   wire   m0_axis_h2c_tlast,	
    input   wire   m0_axis_h2c_tvalid,	
    
	input   wire   s0_axis_c2h_tready,        
    output  wire   [`DATA_WIDTH-1:0] s0_axis_c2h_tdata,
    output  wire   [`KEEP_WIDTH-1:0] s0_axis_c2h_tkeep,
    output  wire   [`KEEP_WIDTH-1:0] s0_axis_c2h_tuser,
    output  wire   s0_axis_c2h_tlast,
    output  wire   s0_axis_c2h_tvalid,
`endif
`ifdef AXIS_BUS1_EN
	output  wire   m1_axis_h2c_tready,
    input   wire   [`DATA_WIDTH-1:0] m1_axis_h2c_tdata,
    input   wire   [`KEEP_WIDTH-1:0] m1_axis_h2c_tkeep,
    input   wire   [`KEEP_WIDTH-1:0] m1_axis_h2c_tuser,
    input   wire   m1_axis_h2c_tlast,
    input   wire   m1_axis_h2c_tvalid,
	input   wire   s1_axis_c2h_tready,
    output  wire   [`DATA_WIDTH-1:0] s1_axis_c2h_tdata,
    output  wire   [`KEEP_WIDTH-1:0] s1_axis_c2h_tkeep,
    output  wire   [`KEEP_WIDTH-1:0] s1_axis_c2h_tuser,
    output  wire   s1_axis_c2h_tlast,
    output  wire   s1_axis_c2h_tvalid,

`endif
`ifdef AXI4_BUS0_EN
	output  wire   m_axi_arready,
	input   wire   [3:0] m_axi_arid,
	input   wire   [`DATA_WIDTH-1:0] m_axi_araddr,
	//specifies the number of data transfers that occur within each burst.
	input   wire   [7:0] m_axi_arlen,
	//specifies the maximum number of data bytes to transfer in each beat. 011-8byte,100-16byte
	input   wire   [2:0] m_axi_arsize,
	input   wire   [1:0] m_axi_arburst,
	input   wire   [2:0] m_axi_arprot,
	input   wire   m_axi_arvalid,
	input   wire   m_axi_arlock,
	input   wire   [3:0] m_axi_arcache,
	
	output  wire   [3:0] m_axi_rid,
	output  wire   [`DATA_WIDTH-1:0]m_axi_rdata,
	output  wire   [1:0] m_axi_rresp,
	output  wire   m_axi_rlast,
	output  wire   m_axi_rvalid,
	input   wire   m_axi_rready,
	
	output  wire   m_axi_awready,
	input   wire   [3:0] m_axi_awid,
	input   wire   [`DATA_WIDTH-1:0] m_axi_awaddr,
	input   wire   [7:0] m_axi_awlen,
	input   wire   [2:0] m_axi_awsize,
	input   wire   [1:0] m_axi_awburst,
	input   wire   [2:0] m_axi_awprot,
	input   wire   m_axi_awvalid,
	input   wire   m_axi_awlock,
	input   wire   [3:0] m_axi_awcache,
	input   wire   [`DATA_WIDTH-1:0] m_axi_wdata,
	input   wire   [7:0] m_axi_wstrb,
	output  wire   m_axi_wready,
	input   wire   m_axi_wlast,
	input   wire   m_axi_wvalid,
	output  wire   [3:0] m_axi_bid,
	output  wire   [1:0] m_axi_bresp,
	output  wire   m_axi_bvalid,
	input   wire   m_axi_bready,
`endif
`ifdef CFG_MGMT_EN 
	output  wire   [18:0] cfg_mgmt_addr,
	output  wire   cfg_mgmt_write,
	output  wire   [31:0] cfg_mgmt_write_data,
	output  wire   [3:0] 	cfg_mgmt_byte_enable,	
	output  wire   cfg_mgmt_read,	
	input   wire   [31:0] cfg_mgmt_read_data,	
	input   wire   cfg_mgmt_read_write_done,
	output  wire   cfg_mgmt_type1_cfg_reg_access,
`endif
`ifdef USR_IRQ_EN
	output  wire   [`DMA_USR_IRQ-1:0]	usr_irq_req,
	input   wire   [`DMA_USR_IRQ-1:0]	usr_irq_ack,
	input   wire   msi_enable,
	input   wire   [2:0] 	msi_vector_width,
`endif     
`ifdef AXIL_MBUS0_EN
	input 	wire   [31:0] m_axil_awaddr,
	input 	wire   [2:0]  m_axil_awprot,	
	input 	wire   m_axil_awvalid,		
	output  wire   m_axil_awready,		
	input 	wire   [31:0] m_axil_wdata,	
	input 	wire   [3:0] 	m_axil_wstrb,	
	input 	wire   m_axil_wvalid,		
	output  wire   m_axil_wready,	
	output  wire   m_axil_bvalid,	
	output  wire   [1:0] 	m_axil_bresp,	
	input 	wire   m_axil_bready,	
	input 	wire   [31:0] m_axil_araddr,	
	input 	wire   [2:0]  m_axil_arprot,	
	input   wire   m_axil_arvalid,		
	output  wire   m_axil_arready,	
	output  wire   [31:0] m_axil_rdata,	
	output  wire   [1:0] 	m_axil_rresp,	
	output  wire   m_axil_rvalid,		
	input   wire   m_axil_rready,
`endif
`ifdef AXIL_SBUS0_EN
	output  wire   [31:0] s_axil_awaddr,
	output  wire   [2 :0] s_axil_awprot,
	output  wire   s_axil_awvalid,		
	input   wire   s_axil_awready,		
	output  wire   [31:0] s_axil_wdata,	
	output  wire   [3:0]  s_axil_wstrb,	
	output  wire   s_axil_wvalid,		
	input   wire   s_axil_wready,	
	input   wire   s_axil_bvalid,	
	input   wire   [1:0]  s_axil_bresp,	
	output  wire   s_axil_bready,	
	output  wire   [31:0] s_axil_araddr,	
	output  wire   [2:0]  s_axil_arprot,	
	output  wire   s_axil_arvalid,		
	input   wire   s_axil_arready,	
	input   wire   [31:0] s_axil_rdata,	
	input   wire   [1:0]  s_axil_rresp,	
	input   wire   s_axil_rvalid,	
	output  wire   s_axil_rready
`endif
//	output  wire   pcie_cfg_tst_o,
`ifdef DEBUG_MODE
   ,output  wire   [`DT_WD*`IDT_NUM-1:0]   h2c_fm_odt,
	input   wire   [`DT_WD*`ODT_NUM-1:0]   c2h_fm_idt
`endif
);
//---Variable definition---
wire   [31:0] c2h0_chn_stts;
wire   [31:0] h2c0_chn_stts;
wire   [31:0] usr_h2c0err_num;
wire   [1:0]  usr_lp0rw_md;
wire   cfg_regrw_run;
wire   [1:0]  dma_regrw_run;
wire   usr_h2c0w_run;
wire   usr_regrw_run;
wire   usr_c2h0r_run;
wire   usr_lp0rw_run;

wire   pcie_cfg_test;
wire   usr_h2c0irq_req;
wire   usr_h2c0irq_ack;
wire   usr_c2h0irq_req;
wire   usr_c2h0irq_ack;
wire   data_rst_flag;
//wire   [1:0]   usr_tst_mode; 

wire   [`DATA_WIDTH-1:0] s0_axis_c2ha_tdata;
wire   [`KEEP_WIDTH-1:0] s0_axis_c2ha_tkeep;
wire   [`KEEP_WIDTH-1:0] s0_axis_c2ha_tuser;
wire   s0_axis_c2ha_tlast;
wire   s0_axis_c2ha_tvalid;
wire   m0_axis_h2ca_tready;
/*
`ifdef C2H_TEST_MODE 
assign usr_tst_mode = 2'b01;
`elsif H2C_TEST_MODE 
assign usr_tst_mode = 2'b10;
`else
assign usr_tst_mode = 2'b11;
`endif*/
app_tst_ctrl u_tst_ctrl(
	.usr_rst_n(usr_rst_n),
	.usr_clk(usr_clk),
    
	.usr_lp0rw_md_i(usr_lp0rw_md),
	.cfg_regrw_run_o(cfg_regrw_run),
    .s0_axis_c2h_run_i(s0_axis_c2h_run),
    .m0_axis_h2c_run_i(m0_axis_h2c_run),
	.dma_regrw_run_o(dma_regrw_run),
	.usr_h2c0w_run_o(usr_h2c0w_run),
    .usr_c2h0r_run_o(usr_c2h0r_run),
	.usr_regrw_run_o(usr_regrw_run),
	.usr_lp0rw_run_o(usr_lp0rw_run)
	
);
/*Instantiate the cfg_regrw.v to copmplete the 
read and write test of configuration space;*/
cfg_regrw u_cfg_regrw(
   	.usr_clk(usr_clk),
	.usr_rst_n(usr_rst_n),
                                                                    
	.cfg_mgmt_addr_o(cfg_mgmt_addr),
    .cfg_mgmt_write_o(cfg_mgmt_write),
    .cfg_mgmt_write_data_o(cfg_mgmt_write_data),
    .cfg_mgmt_byte_enable_o(cfg_mgmt_byte_enable),
    .cfg_mgmt_read_o(cfg_mgmt_read),    
    .cfg_mgmt_read_data_i(cfg_mgmt_read_data),    
    .cfg_mgmt_read_write_done_i(cfg_mgmt_read_write_done),
    .cfg_mgmt_type1_cfg_reg_access_o(cfg_mgmt_type1_cfg_reg_access),

	.cfg_regrw_run_i(cfg_regrw_run),
	.pcie_cfg_tst_o()
);

/*Instantiate the usr_axis0_datarw.v to receive 
or send test data;*/
/*usr_axis0_datarw u_axis0_datarw(

   	.usr_clk(usr_clk),
	.usr_rst_n(usr_rst_n),
	.usr_datarw_run_i(usr_datarw_run),
	 
	.m0_axis_h2c_tready_o(m0_axis_h2c_tready),
    .m0_axis_h2c_tdata_i(m0_axis_h2c_tdata),
    .m0_axis_h2c_tkeep_i(m0_axis_h2c_tkeep),
    .m0_axis_h2c_tuser_i(m0_axis_h2c_tuser),
    .m0_axis_h2c_tlast_i(m0_axis_h2c_tlast),
    .m0_axis_h2c_tvalid_i(m0_axis_h2c_tvalid),	
	                       
	.usr_c2h_len_i(usr_c2h_len_o), 	
	.usr_c2h_len_en_i(usr_c2h_len_en_o),                   
	.s0_axis_c2h_tready_i(s0_axis_c2h_tready),
    .s0_axis_c2h_tdata_o(s0_axis_c2h_tdata),
    .s0_axis_c2h_tkeep_o(s0_axis_c2h_tkeep),
    .s0_axis_c2h_tuser_o(s0_axis_c2h_tuser),
    .s0_axis_c2h_tlast_o(s0_axis_c2h_tlast),
    .s0_axis_c2h_tvalid_o(s0_axis_c2h_tvalid),
   
	.m0_test_mode_i(`TEST_MODE),

	.usr_prg_payload_i(3'b001),
	.usr_prg_read_i(3'b001),
	.usr_eff_payload_o(),
	.usr_eff_read_o(),
                                                                
	.usr_irq_req_o(usr_irq_req),				
	.usr_irq_ack_i(usr_irq_ack),
	.mrd_err_num_o(mrd_err_num)
);*/
/*
`ifdef LOOP_TEST_MODE
assign m0_axis_h2c_tready = s0_axis_c2h_tready;
assign s0_axis_c2h_tdata = m0_axis_h2c_tdata;
assign s0_axis_c2h_tkeep = m0_axis_h2c_tkeep;
assign s0_axis_c2h_tuser = m0_axis_h2c_tuser;
assign s0_axis_c2h_tlast = m0_axis_h2c_tlast;
assign s0_axis_c2h_tvalid = m0_axis_h2c_tvalid;
assign usr_c2h0irq_req = 1'b0;
assign usr_c2h0err = 1'b0;
assign usr_h2c0irq_req = 1'b0;
assign usr_h2c0err_num = 32'd0;
`elsif H2C_TEST_MODE*/
usr_h2c0w u_usr_h2c0w(
   	.usr_rst_n(usr_rst_n),
	.usr_clk(usr_clk),
	
	.usr_h2c0w_run_i(usr_h2c0w_run),
	.m0_axis_h2c_rst_i(m0_axis_h2c_rst),
    
	.m0_axis_h2c_tready_o(m0_axis_h2ca_tready),
    .m0_axis_h2c_tdata_i(m0_axis_h2c_tdata),
    .m0_axis_h2c_tkeep_i(m0_axis_h2c_tkeep),
    .m0_axis_h2c_tuser_i(m0_axis_h2c_tuser),
    .m0_axis_h2c_tlast_i(m0_axis_h2c_tlast),
    .m0_axis_h2c_tvalid_i(m0_axis_h2c_tvalid),	
   
	.usr_h2c0irq_req_o(usr_h2c0irq_req),				
	.usr_h2c0irq_ack_i(usr_h2c0irq_ack),	
	.usr_h2c0err_num_o(usr_h2c0err_num)
`ifdef DEBUG_MODE	
	,.h2c_fm_odt(h2c_fm_odt)
`endif
);
/*assign usr_c2h0irq_req = 1'b0;
assign usr_c2h0err = 1'b0;
`else //C2H_TEST_MODE*/
usr_c2h0r u_usr_c2h0r(
   	.usr_rst_n(usr_rst_n),
	.usr_clk(usr_clk),
	.usr_c2h0r_run_i(usr_c2h0r_run),
    
	.s0_axis_c2h_rst_i(s0_axis_c2h_rst),
`ifdef DEBUG_MODE		
	.c2h_fm_idt(c2h_fm_idt),
`endif
	.s0_axis_c2h_tready_i(s0_axis_c2h_tready),
    .s0_axis_c2h_tdata_o(s0_axis_c2ha_tdata),
    .s0_axis_c2h_tkeep_o(s0_axis_c2ha_tkeep),
    .s0_axis_c2h_tuser_o(s0_axis_c2ha_tuser),
    .s0_axis_c2h_tlast_o(s0_axis_c2ha_tlast),
    .s0_axis_c2h_tvalid_o(s0_axis_c2ha_tvalid),

    
	.usr_c2h0irq_req_o(usr_c2h0irq_req),				
	.usr_c2h0irq_ack_i(usr_c2h0irq_ack),
	.usr_c2h0err_o(usr_c2h0err)
);
/*assign usr_h2c0irq_req = 1'b0;
assign usr_h2c0err_num = 32'd0;
assign h2c_fm_odt = 'd0;
`endif*/

usr_lp0rw u_usr_lp0rw(
    .usr_lp0rw_run_i(usr_lp0rw_run),
    
    .m0_axis_h2c_tdata_i(m0_axis_h2c_tdata),
    .m0_axis_h2c_tkeep_i(m0_axis_h2c_tkeep),
    .m0_axis_h2c_tuser_i(m0_axis_h2c_tuser),
    .m0_axis_h2c_tlast_i(m0_axis_h2c_tlast),
    .m0_axis_h2c_tvalid_i(m0_axis_h2c_tvalid),	
    .m0_axis_h2ca_tready_i(m0_axis_h2ca_tready),
    
    .s0_axis_c2ha_tdata_i(s0_axis_c2ha_tdata),
    .s0_axis_c2ha_tkeep_i(s0_axis_c2ha_tkeep),
    .s0_axis_c2ha_tuser_i(s0_axis_c2ha_tuser),
    .s0_axis_c2ha_tlast_i(s0_axis_c2ha_tlast),
    .s0_axis_c2ha_tvalid_i(s0_axis_c2ha_tvalid),
    .s0_axis_c2h_tready_i(s0_axis_c2h_tready),
    
    .m0_axis_h2c_tready_o(m0_axis_h2c_tready),
    .s0_axis_c2h_tdata_o(s0_axis_c2h_tdata),
    .s0_axis_c2h_tkeep_o(s0_axis_c2h_tkeep),
    .s0_axis_c2h_tuser_o(s0_axis_c2h_tuser),
    .s0_axis_c2h_tlast_o(s0_axis_c2h_tlast),
    .s0_axis_c2h_tvalid_o(s0_axis_c2h_tvalid)
);

assign usr_irq_req = {usr_c2h0irq_req,usr_h2c0irq_req};
assign {usr_h2c0irq_ack,usr_c2h0irq_ack} = usr_irq_ack;
//---Instantiate the usr_regrw.v to test reading and writing registers;

usr_regrw u_usr_regrw(
	.usr_rst_n(usr_rst_n),
	.usr_clk(usr_clk),
    
    .s0_axis_c2h_rst_i(s0_axis_c2h_rst),
    .m0_axis_h2c_rst_i(m0_axis_h2c_rst),
    
	.usr_regrw_run_i(usr_regrw_run),
	.m_axil_awaddr_i(m_axil_awaddr),
	.m_axil_awprot_i(m_axil_awprot),	
	.m_axil_awvalid_i(m_axil_awvalid),		
	.m_axil_awready_o(m_axil_awready),		
	.m_axil_wdata_i	(m_axil_wdata),	
	.m_axil_wstrb_i	(m_axil_wstrb),	
	.m_axil_wvalid_i(m_axil_wvalid),		
	.m_axil_wready_o(m_axil_wready),	
	.m_axil_bvalid_o(m_axil_bvalid),	
	.m_axil_bresp_o	(m_axil_bresp),	
	.m_axil_bready_i(m_axil_bready),	
	.m_axil_araddr_i(m_axil_araddr),	
	.m_axil_arprot_i(m_axil_arprot),	
	.m_axil_arvalid_i(m_axil_arvalid),		
	.m_axil_arready_o(m_axil_arready),	
	.m_axil_rdata_o	(m_axil_rdata),	
	.m_axil_rresp_o	(m_axil_rresp),	
	.m_axil_rvalid_o(m_axil_rvalid),		
	.m_axil_rready_i(m_axil_rready),
	
    .usr_h2c0err_num_i(usr_h2c0err_num),
    .usr_lp0rw_md_o(usr_lp0rw_md)
/*	
	.usr_prg_payload_o(usr_prg_payload),
	.usr_prg_read_o(usr_prg_read),
	.usr_eff_payload_i(usr_eff_payload),
	.usr_eff_read_i(usr_eff_read)
*/	
);

/*
axi2ahb  u_axi2ahb (
	 .aclk(usr_clk),
	 .aresetn(usr_rst_n),
	 //axi side
     .us_axi_awid(m_axi_awid),
     .us_axi_awlen(m_axi_awlen),
     .us_axi_awsize(m_axi_awsize),
     .us_axi_awburst(m_axi_awburst),
     .us_axi_awcache(m_axi_awcache),
     .us_axi_awaddr(m_axil_awaddr),
     .us_axi_awprot(m_axil_awprot),
     .us_axi_awvalid(m_axil_awvalid),
     .us_axi_awready(m_axil_awready),
     .us_axi_awlock(m_axi_awlock),
     .us_axi_wdata(m_axil_wdata),
     .us_axi_wstrb(m_axil_wstrb),
     .us_axi_wlast(m_axi_wlast),
     .us_axi_wvalid(m_axil_wvalid),
     .us_axi_wready(m_axil_wready),
     .us_axi_bid(m_axi_bid),
     .us_axi_bresp(m_axil_bresp),
     .us_axi_bvalid(m_axil_bvalid),
     .us_axi_bready(m_axil_bready),
     .us_axi_arid(m_axi_arid),
     .us_axi_araddr(m_axil_araddr),
     .us_axi_arprot(m_axil_arprot),
     .us_axi_arcache(m_axi_arcache),
     .us_axi_arvalid(m_axil_arvalid),
     .us_axi_arlen(m_axi_arlen),
     .us_axi_arsize(m_axi_arsize),
     .us_axi_arburst(m_axi_arburst),
     .us_axi_arlock(m_axi_arlock),
     .us_axi_arready(m_axil_arready),
     .us_axi_rid(m_axi_rid),
     .us_axi_rdata(m_axil_rdata),
     .us_axi_rresp(m_axil_rresp),
     .us_axi_rvalid(m_axil_rvalid),
     .us_axi_rlast(m_axi_rlast),
     .us_axi_rready(m_axil_rready),
     //ahb side
     .ds_ahb_haddr(HADDR_ACCC),
     .ds_ahb_hwrite(HWRITE_ACCC),
     .ds_ahb_hsize(HSIZE_ACCC),
     .ds_ahb_hburst(),
     .ds_ahb_hprot(HPROT_ACCC),
     .ds_ahb_htrans(HTRANS_ACCC),
     .ds_ahb_hmastlock(),
     .ds_ahb_hwdata(HWDATA_ACCC),
     .ds_ahb_hready(HREADY_ACCC),
     .ds_ahb_hrdata(HRDATA_ACCC),
     .ds_ahb_hresp(HRESP_ACCC)
);


AHBlite_ACC ACC_Interface(
        .HCLK                           (usr_clk),
        .HRESETn                        (usr_rst_n),
        .HADDR                          (HADDR_ACCC),
        .HPROT                          (HPROT_ACCC),
        .HSEL                           (HSEL_ACCC),
        .HSIZE                          (HSIZE_ACCC),
        .HTRANS                         (HTRANS_ACCC),
        .HWRITE                         (HWRITE_ACCC),
        .HRDATA                         (HRDATA_ACCC),
        .HREADY                         (HREADY_ACCC),
        .HREADYOUT                      (HREADYOUT_ACCC),
        .HRESP                          (HRESP_ACCC),
        .HWDATA                         (HWDATA_ACCC),
        .Data_RAM_RD_Addr               (Data_RAM_RD_Addr),
        .Data_RAM_RD_Data               (Data_RAM_RD_Data),
        .Weight_RAM_RD_Addr             (Weight_RAM_RD_Addr),
        .Weight_RAM_RD_Data             (Weight_RAM_RD_Data),
        .Num0                           (Num0),
        .Num1                           (Num1),
        .Num2                           (Num2),
        .Num3                           (Num3),
        .Num4                           (Num4),
        .Num5                           (Num5),
        .Num6                           (Num6),
        .Num7                           (Num7),
        .Num8                           (Num8),
        .Num9                           (Num9),
        .ACC_READY                      (ACC_READY),
        .ACC_VALID                      (ACC_VALID)
);

ACC     ACC(
        .clk                            (usr_clk),
        .rstn                           (usr_rst_n),
        .AccValid_i                     (ACC_VALID),
        .AccReady_o                     (ACC_READY),
        .DataRamAddr_o                  (Data_RAM_RD_Addr),
        .DataRamData_i                  (Data_RAM_RD_Data),
        .WtRamAddr_o                    (Weight_RAM_RD_Addr),
        .WtRamData_i                    (Weight_RAM_RD_Data),
        .Num0_o                         (Num0),
        .Num1_o                         (Num1),
        .Num2_o                         (Num2),
        .Num3_o                         (Num3),
        .Num4_o                         (Num4),
        .Num5_o                         (Num5),
        .Num6_o                         (Num6),
        .Num7_o                         (Num7),
        .Num8_o                         (Num8),
        .Num9_o                         (Num9)
);*/

dma_regrw u_dma_regrw(

	.usr_rst_n(usr_rst_n),
	.usr_clk(usr_clk),
                                                                    
	.s_axil_awaddr_o(s_axil_awaddr),
    .s_axil_awprot_o(s_axil_awprot),
    .s_axil_awvalid_o(s_axil_awvalid),
    .s_axil_awready_i(s_axil_awready),
    .s_axil_wdata_o(s_axil_wdata),
    .s_axil_wstrb_o(s_axil_wstrb),
    .s_axil_wvalid_o(s_axil_wvalid),
    .s_axil_wready_i(s_axil_wready),
    .s_axil_bvalid_i(s_axil_bvalid),
    .s_axil_bresp_i(s_axil_bresp),
    .s_axil_bready_o(s_axil_bready),
    .s_axil_araddr_o(s_axil_araddr),
    .s_axil_arprot_o(s_axil_arprot),
    .s_axil_arvalid_o(s_axil_arvalid),
    .s_axil_arready_i(s_axil_arready),
    .s_axil_rdata_i(s_axil_rdata),
    .s_axil_rresp_i(s_axil_rresp),
    .s_axil_rvalid_i(s_axil_rvalid),
    .s_axil_rready_o(s_axil_rready),
               
    .dma_regrw_run_i(dma_regrw_run),                                           
	.c2h0_chn_stts_o(c2h0_chn_stts),
	.h2c0_chn_stts_o(h2c0_chn_stts)

);

endmodule
