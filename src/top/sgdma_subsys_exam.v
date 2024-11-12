`ifdef DEBUG_MODE
	`include "para_def.vh"
	`include "dt_para.vh"
`endif
module sgdma_subsys_exam(/*AUTOARG*/
	input   app_auxclk,
	input   app_power_up_rst_n,
	
	input   [`REFCLK_WIDTH-1:0]  refclk_p,
	input   [`REFCLK_WIDTH-1:0]  refclk_n,
	output  [`LINK_WIDTH-1:0]    txp,
	output  [`LINK_WIDTH-1:0]    txn,  
	input   [`LINK_WIDTH-1:0]    rxp,
	input   [`LINK_WIDTH-1:0]    rxn,
`ifdef DEBUG_MODE
    input   [`DT_WD*`ODT_NUM-1:0]   c2h_fm_idt,
    output  [`DT_WD*`IDT_NUM-1:0]   h2c_fm_odt,    
`endif   
    output  core_clk_led,
    output  user_lnk_up,
    
    input   SI5345_CLKIN_P,
    input   SI5345_INTRB_N, //This pin is asserted low when a change in device status has occurred    
    input   SI5345_LOLB_N , //This output pin indicates when the DSPLL is locked (high) or out of lock (low). 
    output  SI5345_FINC   , //step-up the output frequency
    output  SI5345_FDEC   , //step-down the output frequency  
    output  SI5345_IN_SEL0, //The IN_SEL[1:0] pins are used in manual pin ...
    output  SI5345_IN_SEL1, //controlled mode to select the active clock input
    output  SI5345_OEB    , //This pin disables all outputs when held high.
    output  SI5345_RSTB_N , //Active low input that performs power-on reset (POR) of the device.
    inout   SI5345_SCK_O  , //This pin functions as the serial clock input for I2C
    inout   SI5345_SDA_IO , //This is the bidirectional data pin (SDA) for the I2C mode
    
//    input   I_CPU_JTAG_TCK ,
//	input   I_CPU_JTAG_TDI ,
//	output  O_CPU_JTAG_TDO ,
//	input   I_CPU_JTAG_TMS ,
    output  O_mcu_uart_tx  ,
    input   I_mcu_uart_rx  
);     
 	
`ifdef AXIS_BUS0_EN
	wire   s0_axis_c2h_rst;
    wire   m0_axis_h2c_rst;
    wire   s0_axis_c2h_run;
    wire   m0_axis_h2c_run;
    
	wire   s0_axis_c2h_tready;
    wire   [`DATA_WIDTH-1:0]    s0_axis_c2h_tdata;
    wire   [`KEEP_WIDTH-1:0]    s0_axis_c2h_tkeep;
    wire   [`KEEP_WIDTH-1:0]    s0_axis_c2h_tuser;
    wire   s0_axis_c2h_tlast;
    wire   s0_axis_c2h_tvalid;	
	
	wire   m0_axis_h2c_tready;
    wire   [`DATA_WIDTH-1:0]    m0_axis_h2c_tdata;
    wire   [`KEEP_WIDTH-1:0]    m0_axis_h2c_tkeep;
    wire   [`KEEP_WIDTH-1:0]    m0_axis_h2c_tuser;
    wire   m0_axis_h2c_tlast;
    wire   m0_axis_h2c_tvalid;
`endif
`ifdef AXIS_BUS1_EN
	wire   s1_axis_c2h_tready;
    wire   [`DATA_WIDTH-1:0]    s1_axis_c2h_tdata;
    wire   [`KEEP_WIDTH-1:0]    s1_axis_c2h_tkeep;
    wire   [`KEEP_WIDTH-1:0]    s1_axis_c2h_tuser;
    wire   s1_axis_c2h_tlast;
    wire   s1_axis_c2h_tvalid;
	wire   m1_axis_h2c_tready;
    wire   [`DATA_WIDTH-1:0]    m1_axis_h2c_tdata;
    wire   [`KEEP_WIDTH-1:0]    m1_axis_h2c_tkeep;
    wire   [`KEEP_WIDTH-1:0]    m1_axis_h2c_tuser;
    wire   m1_axis_h2c_tlast;
    wire   m1_axis_h2c_tvalid;
`endif
`ifdef AXI4_BUS0_EN
	//---Write Address Interface---
	wire  m_axi_awready;
	wire  [3:0] m_axi_awid;
	wire  [63:0] m_axi_awaddr;
	wire  [7:0] m_axi_awlen;
	wire  [2:0] m_axi_awsize;
	wire  [1:0] m_axi_awburst;
	wire  [2:0] m_axi_awprot;
	wire  m_axi_awvalid;
	wire  m_axi_awlock;
	wire  [3:0] m_axi_awcache;
	//---Write Interface---
	wire  [63:0] m_axi_wdata;
	wire  [7:0] m_axi_wstrb;
	wire  m_axi_wready;
	wire  m_axi_wlast;
	wire  m_axi_wvalid;
	//---Write Response Interface---
	wire  [3:0] m_axi_bid;
	wire  [1:0] m_axi_bresp;
	wire  m_axi_bvalid;
	wire  m_axi_bready;
	//---Read Address Interface---
	wire  m_axi_arready;
	wire  [3:0] m_axi_arid;
	wire  [63:0] m_axi_araddr;
	wire  [7:0] m_axi_arlen;
	wire  [2:0] m_axi_arsize;
	wire  [1:0] m_axi_arburst;
	wire  [2:0] m_axi_arprot;
	wire  m_axi_arvalid;
	wire  m_axi_arlock;
	wire  [3:0] m_axi_arcache;
	//---Read Interface---
	wire  [3:0] m_axi_rid;
	wire  [63:0] m_axi_rdata;
	wire  [1:0] m_axi_rresp;
	wire  m_axi_rlast;
	wire  m_axi_rvalid;
	wire  m_axi_rready;
	
`endif
`ifdef USR_IRQ_EN
	wire  [`DMA_USR_IRQ-1:0] usr_irq_req;
	wire  [`DMA_USR_IRQ-1:0] usr_irq_ack;
	wire  msi_enable;
	wire  [2:0] msi_vector_width;
`endif
`ifdef AXIL_MBUS0_EN
	wire 	[31:0] m_axil_awaddr;
	wire 	[2:0]  m_axil_awprot;	
	wire 	m_axil_awvalid;	
	wire 	m_axil_awready;	
	wire 	[31:0] m_axil_wdata;	
	wire 	[3:0]  m_axil_wstrb;	
	wire 	m_axil_wvalid;	
	wire 	m_axil_wready;	
	wire 	m_axil_bvalid;	
	wire 	[1:0]  m_axil_bresp;	
	wire 	m_axil_bready;	
	wire 	[31:0] m_axil_araddr;	
	wire 	[2:0]  m_axil_arprot;	
	wire 	m_axil_arvalid;	
	wire 	m_axil_arready;	
	wire 	[31:0] m_axil_rdata;	
	wire 	[1:0]  m_axil_rresp;	
	wire 	m_axil_rvalid;		
	wire 	m_axil_rready;
`endif	
`ifdef AXIL_SBUS0_EN
	wire  [31:0] s_axil_awaddr;
	wire  [2:0] s_axil_awprot;
	wire  s_axil_awvalid;	
	wire  s_axil_awready;	
	wire  [31:0] s_axil_wdata;	
	wire  [3:0] s_axil_wstrb;	
	wire  s_axil_wvalid;	
	wire  s_axil_wready;	
	wire  s_axil_bvalid;	
	wire  [1:0] s_axil_bresp;	
	wire  s_axil_bready;	
	wire  [31:0] s_axil_araddr;	
	wire  [2:0] s_axil_arprot;	
	wire  s_axil_arvalid;	
	wire  s_axil_arready;	
	wire  [31:0] s_axil_rdata;	
	wire  [1:0] s_axil_rresp;	
	wire  s_axil_rvalid;	
	wire  s_axil_rready;
`endif
`ifdef CFG_MGMT_EN
	wire   [18:0] cfg_mgmt_addr;
	wire   cfg_mgmt_write;
	wire   [31:0] cfg_mgmt_write_data;
	wire   [3:0] cfg_mgmt_byte_enable;	
	wire   cfg_mgmt_read;	
	wire   [31:0] cfg_mgmt_read_data;	
	wire   cfg_mgmt_read_write_done;
	wire   cfg_mgmt_type1_cfg_reg_access;
`endif

wire Si5345_wait_over;//synthesis keep
//wire   app_auxclk;
wire   user_resetn;
//wire   user_lnk_up;
wire   user_clk;
//wire   phy_link_up;

//wire   [31:0]  usr_c2h_len_o;	
//wire   usr_c2h_len_en_o;

//PH1_PHY_GCLK app_auxclk_bufg(.clkin({app_auxclk_in,1'b1}),.clkout(app_auxclk),.cen({1'b0,1'b1}),.seln({1'b0,1'b1}),.drct({1'b0,1'b1}));

sgdma_subsys u_sgdma_subsys(
    .app_auxclk(app_auxclk),
    .app_power_up_rst_n(app_power_up_rst_n ),
    .refclk_p(refclk_p),
    .refclk_n(refclk_n),
    .txp(txp),
    .txn(txn),
    .rxp(rxp),
    .rxn(rxn),
    .user_lnk_up(user_lnk_up),
    .user_clk(user_clk),
    .user_rstn(user_resetn),
    .local_int(1'b0),            
    .ltssm_state(),
                        
`ifdef AXI4_BUS0_EN
	.m_axi_awready(m_axi_awready),
    .m_axi_wready(m_axi_wready),
    .m_axi_bid(m_axi_bid),
    .m_axi_bresp(m_axi_bresp),
    .m_axi_bvalid(m_axi_bvalid),
    .m_axi_arready(m_axi_arready),
    .m_axi_rid(m_axi_rid),
    .m_axi_rdata(m_axi_rdata),
//  .m_axi_ruser(),
    .m_axi_rresp(m_axi_rresp),
    .m_axi_rlast(m_axi_rlast),
    .m_axi_rvalid(m_axi_rvalid),
    .m_axi_awid(m_axi_awid),
    .m_axi_awaddr(m_axi_awaddr),
    .m_axi_awlen(m_axi_awlen),
    .m_axi_awsize(m_axi_awsize),
    .m_axi_awburst(m_axi_awburst),
    .m_axi_awprot(m_axi_awprot),
    .m_axi_awvalid(m_axi_awvalid),
    .m_axi_awlock(m_axi_awlock),
    .m_axi_awcache(m_axi_awcache),
    .m_axi_wdata(m_axi_wdata),
//  .m_axi_wuser(),
    .m_axi_wstrb(m_axi_wstrb),
    .m_axi_wlast(m_axi_wlast),
    .m_axi_wvalid(m_axi_wvalid),
    .m_axi_bready(m_axi_bready),
    .m_axi_arid(m_axi_arid),
    .m_axi_araddr(m_axi_araddr),
    .m_axi_arlen(m_axi_arlen),
    .m_axi_arsize(m_axi_arsize),
    .m_axi_arburst(m_axi_arburst),
    .m_axi_arprot(m_axi_arprot),
    .m_axi_arvalid(m_axi_arvalid),
    .m_axi_arlock(m_axi_arlock),
    .m_axi_arcache(m_axi_arcache),
    .m_axi_rready(m_axi_rready),
`endif
`ifdef AXIL_MBUS0_EN	
	//---axi-lite---
	.m_axil_awaddr(m_axil_awaddr),
//  .m_axil_awuser(),
    .m_axil_awprot(m_axil_awprot),
    .m_axil_awvalid(m_axil_awvalid),
    .m_axil_awready(m_axil_awready),
    .m_axil_wdata(m_axil_wdata),
    .m_axil_wstrb(m_axil_wstrb),
    .m_axil_wvalid(m_axil_wvalid),
    .m_axil_wready(m_axil_wready),
    .m_axil_bvalid(m_axil_bvalid),
    .m_axil_bresp(m_axil_bresp),
    .m_axil_bready(m_axil_bready),
    .m_axil_araddr(m_axil_araddr),
//  .m_axil_aruser(),
    .m_axil_arprot(m_axil_arprot),
    .m_axil_arvalid(m_axil_arvalid),
    .m_axil_arready(m_axil_arready),
    .m_axil_rdata(m_axil_rdata),
    .m_axil_rresp(m_axil_rresp),
    .m_axil_rvalid(m_axil_rvalid),
    .m_axil_rready(m_axil_rready),
`endif	
`ifdef AXIL_SBUS0_EN
	.s_axil_awaddr(s_axil_awaddr),
    .s_axil_awprot(s_axil_awprot),
    .s_axil_awvalid(s_axil_awvalid),
    .s_axil_awready(s_axil_awready),
    .s_axil_wdata(s_axil_wdata),
    .s_axil_wstrb(s_axil_wstrb),
    .s_axil_wvalid(s_axil_wvalid),
    .s_axil_wready(s_axil_wready),
    .s_axil_bvalid(s_axil_bvalid),
    .s_axil_bresp(s_axil_bresp),
    .s_axil_bready(s_axil_bready),
    .s_axil_araddr(s_axil_araddr),
    .s_axil_arprot(s_axil_arprot),
    .s_axil_arvalid(s_axil_arvalid),
    .s_axil_arready(s_axil_arready),
    .s_axil_rdata(s_axil_rdata),
    .s_axil_rresp(s_axil_rresp),
    .s_axil_rvalid(s_axil_rvalid),
    .s_axil_rready(s_axil_rready),
`endif
	//---2路axistream---
`ifdef AXIS_BUS0_EN	
  	.s0_axis_c2h_rst(s0_axis_c2h_rst),
    .m0_axis_h2c_rst(m0_axis_h2c_rst),
    .s0_axis_c2h_run(s0_axis_c2h_run),
    .m0_axis_h2c_run(m0_axis_h2c_run),   
      
	.s0_axis_c2h_tdata(s0_axis_c2h_tdata),
    .s0_axis_c2h_tlast(s0_axis_c2h_tlast),
    .s0_axis_c2h_tvalid(s0_axis_c2h_tvalid),
    .s0_axis_c2h_tready(s0_axis_c2h_tready),
    .s0_axis_c2h_tuser(s0_axis_c2h_tuser),
    .s0_axis_c2h_tkeep(s0_axis_c2h_tkeep),
    .m0_axis_h2c_tdata(m0_axis_h2c_tdata),
    .m0_axis_h2c_tlast(m0_axis_h2c_tlast),
    .m0_axis_h2c_tvalid(m0_axis_h2c_tvalid),
    .m0_axis_h2c_tready(m0_axis_h2c_tready),
    .m0_axis_h2c_tuser(m0_axis_h2c_tuser),
    .m0_axis_h2c_tkeep(m0_axis_h2c_tkeep),
`endif	
`ifdef AXIS_BUS1_EN
    .s1_axis_c2h_tdata(s1_axis_c2h_tdata),
    .s1_axis_c2h_tlast(s1_axis_c2h_tlast),
    .s1_axis_c2h_tvalid(s1_axis_c2h_tvalid),
    .s1_axis_c2h_tready(s1_axis_c2h_tready),
    .s1_axis_c2h_tuser(s1_axis_c2h_tuser),
    .s1_axis_c2h_tkeep(s1_axis_c2h_tkeep),
    .m1_axis_h2c_tdata(m1_axis_h2c_tdata),
    .m1_axis_h2c_tlast(m1_axis_h2c_tlast),
    .m1_axis_h2c_tvalid(m1_axis_h2c_tvalid),
    .m1_axis_h2c_tready(m1_axis_h2c_tready),
    .m1_axis_h2c_tuser(m1_axis_h2c_tuser),
    .m1_axis_h2c_tkeep(m1_axis_h2c_tkeep),
`endif	    
`ifdef USR_IRQ_EN	
	//---usr-irq->msi---
    .usr_irq_req(usr_irq_req),
//  .usr_irq_function_number(4'd0),
    .usr_irq_ack(usr_irq_ack),
    .msi_enable(msi_enable),
//  .msix_enable(),
    .msi_vector_width(msi_vector_width),	
`endif	
`ifdef CFG_MGMT_EN
	//---cfg_mgmt->dbi---
    .cfg_mgmt_addr(cfg_mgmt_addr),
    .cfg_mgmt_write(cfg_mgmt_write),
    .cfg_mgmt_write_data(cfg_mgmt_write_data),
    .cfg_mgmt_byte_enable(cfg_mgmt_byte_enable),
    .cfg_mgmt_read(cfg_mgmt_read),
    .cfg_mgmt_read_data(cfg_mgmt_read_data),
    .cfg_mgmt_read_write_done(cfg_mgmt_read_write_done),
    .cfg_mgmt_type1_cfg_reg_access(cfg_mgmt_type1_cfg_reg_access),
`endif	
    
    //---test led
    .core_clk_led(core_clk_led)    
	//.app_auxclk_led(app_auxclk_led),  
	//.cr_clk_led(cr_clk_led)       
);    	
 







sgdma_app u_sgdma_app(
	.usr_rst_n(user_resetn),
	.usr_clk(user_clk),
`ifdef DEBUG_MODE 
    .c2h_fm_idt(c2h_fm_idt),	
    .h2c_fm_odt(h2c_fm_odt),
`endif
`ifdef AXIS_BUS0_EN
	.m0_axis_h2c_tready(m0_axis_h2c_tready),
    .m0_axis_h2c_tdata(m0_axis_h2c_tdata),
    .m0_axis_h2c_tkeep(m0_axis_h2c_tkeep),
    .m0_axis_h2c_tuser(m0_axis_h2c_tuser),
    .m0_axis_h2c_tlast(m0_axis_h2c_tlast),
    .m0_axis_h2c_tvalid(m0_axis_h2c_tvalid),	
      	
    .s0_axis_c2h_rst(s0_axis_c2h_rst),
    .m0_axis_h2c_rst(m0_axis_h2c_rst),
    .s0_axis_c2h_run(s0_axis_c2h_run),
    .m0_axis_h2c_run(m0_axis_h2c_run),
    
	.s0_axis_c2h_tready(s0_axis_c2h_tready),
    .s0_axis_c2h_tdata(s0_axis_c2h_tdata),
    .s0_axis_c2h_tkeep(s0_axis_c2h_tkeep),
    .s0_axis_c2h_tuser(s0_axis_c2h_tuser),
    .s0_axis_c2h_tlast(s0_axis_c2h_tlast),
    .s0_axis_c2h_tvalid(s0_axis_c2h_tvalid),
`endif    

`ifdef AXIS_BUS1_EN
	.s1_axis_c2h_tready(s1_axis_c2h_tready),
    .s1_axis_c2h_tdata(s1_axis_c2h_tdata),
    .s1_axis_c2h_tkeep(s1_axis_c2h_tkeep),
    .s1_axis_c2h_tuser(s1_axis_c2h_tuser),
    .s1_axis_c2h_tlast(s1_axis_c2h_tlast),
    .s1_axis_c2h_tvalid(s1_axis_c2h_tvalid),
	.m1_axis_h2c_tready(m1_axis_h2c_tready),
    .m1_axis_h2c_tdata(m1_axis_h2c_tdata),
    .m1_axis_h2c_tkeep(m1_axis_h2c_tkeep),
    .m1_axis_h2c_tuser(m1_axis_h2c_tuser),
    .m1_axis_h2c_tlast(m1_axis_h2c_tlast),
    .m1_axis_h2c_tvalid(m1_axis_h2c_tvalid),
`endif
`ifdef AXI4_BUS0_EN
	.m_axi_awready(m_axi_awready),
	.m_axi_wready(m_axi_wready),
	.m_axi_bid(m_axi_bid),
	.m_axi_bresp(m_axi_bresp),
	.m_axi_bvalid(m_axi_bvalid),
	.m_axi_arready(m_axi_arready),
	.m_axi_rid(m_axi_rid),
	.m_axi_rdata(m_axi_rdata),
	.m_axi_rresp(m_axi_rresp),
	.m_axi_rlast(m_axi_rlast),
	.m_axi_rvalid(m_axi_rvalid),
	.m_axi_awid(m_axi_awid),
//	.m_axi_awid(m_axi_awid),
//	.m_axi_awid(m_axi_awid),
	.m_axi_awaddr(m_axi_awaddr),
	.m_axi_awlen(m_axi_awlen),
	.m_axi_awsize(m_axi_awsize),
	.m_axi_awburst(m_axi_awburst),
	.m_axi_awprot(m_axi_awprot),
	.m_axi_awvalid(m_axi_awvalid),
	.m_axi_awlock(m_axi_awlock),
	.m_axi_awcache(m_axi_awcache),
	.m_axi_wdata(m_axi_wdata),
	.m_axi_wstrb(m_axi_wstrb),
	.m_axi_wlast(m_axi_wlast),
	.m_axi_wvalid(m_axi_wvalid),
	.m_axi_bready(m_axi_bready),
	.m_axi_arid(m_axi_arid),
	.m_axi_araddr(m_axi_araddr),
	.m_axi_arlen(m_axi_arlen),
	.m_axi_arsize(m_axi_arsize),
	.m_axi_arburst(m_axi_arburst),
	.m_axi_arprot(m_axi_arprot),
	.m_axi_arvalid(m_axi_arvalid),
	.m_axi_arlock(m_axi_arlock),
	.m_axi_arcache(m_axi_arcache),
	.m_axi_rready(m_axi_rready),
`endif	

`ifdef CFG_MGMT_EN 
	.cfg_mgmt_addr(cfg_mgmt_addr),
	.cfg_mgmt_write(cfg_mgmt_write),
	.cfg_mgmt_write_data(cfg_mgmt_write_data),
	.cfg_mgmt_byte_enable(cfg_mgmt_byte_enable),	
	.cfg_mgmt_read(cfg_mgmt_read),	
	.cfg_mgmt_read_data(cfg_mgmt_read_data),	
	.cfg_mgmt_read_write_done(cfg_mgmt_read_write_done),
	.cfg_mgmt_type1_cfg_reg_access(cfg_mgmt_type1_cfg_reg_access),
`endif
`ifdef USR_IRQ_EN
	.usr_irq_req(usr_irq_req),
	.usr_irq_ack(usr_irq_ack),
	.msi_enable(msi_enable),
	.msi_vector_width(msi_vector_width),
`endif
`ifdef AXIL_MBUS0_EN
	.m_axil_awaddr(m_axil_awaddr),
	.m_axil_awprot(m_axil_awprot),	
	.m_axil_awvalid(m_axil_awvalid),		
	.m_axil_awready(m_axil_awready),		
	.m_axil_wdata(m_axil_wdata),	
	.m_axil_wstrb(m_axil_wstrb),	
	.m_axil_wvalid(m_axil_wvalid),		
	.m_axil_wready(m_axil_wready),	
	.m_axil_bvalid(m_axil_bvalid),	
	.m_axil_bresp(m_axil_bresp),	
	.m_axil_bready(m_axil_bready),	
	.m_axil_araddr(m_axil_araddr),	
	.m_axil_arprot(m_axil_arprot),	
	.m_axil_arvalid(m_axil_arvalid),		
	.m_axil_arready(m_axil_arready),	
	.m_axil_rdata(m_axil_rdata),	
	.m_axil_rresp(m_axil_rresp),	
	.m_axil_rvalid(m_axil_rvalid),		
	.m_axil_rready(m_axil_rready)
`endif
`ifdef AXIL_SBUS0_EN	
	,.s_axil_awaddr(s_axil_awaddr),
	.s_axil_awprot(s_axil_awprot),
	.s_axil_awvalid(s_axil_awvalid),		
	.s_axil_awready(s_axil_awready),		
	.s_axil_wdata(s_axil_wdata),	
	.s_axil_wstrb(s_axil_wstrb),	
	.s_axil_wvalid(s_axil_wvalid),		
	.s_axil_wready(s_axil_wready),	
	.s_axil_bvalid(s_axil_bvalid),	
	.s_axil_bresp(s_axil_bresp),	
	.s_axil_bready(s_axil_bready),	
	.s_axil_araddr(s_axil_araddr),	
	.s_axil_arprot(s_axil_arprot),	
	.s_axil_arvalid(s_axil_arvalid),		
	.s_axil_arready(s_axil_arready),	
	.s_axil_rdata(s_axil_rdata),	
	.s_axil_rresp(s_axil_rresp),	
	.s_axil_rvalid(s_axil_rvalid),	
	.s_axil_rready(s_axil_rready)
`endif	
//	,.pcie_cfg_test_o()
);

riscv_top u_riscv_top(
    .clk            (app_auxclk		),
    .rst_n          (1'b1),

    .O_mcu_uart_tx  (O_mcu_uart_tx  ),
    .I_mcu_uart_rx  (I_mcu_uart_rx  ),
                                    
//    .SI5345_CLKIN_P (SI5345_CLKIN_P ),
//    .test_led       (       ),
                                    
    .SI5345_INTRB_N (SI5345_INTRB_N ), //This pin is asserted low when a change in device status has occurred    
    .SI5345_LOLB_N  (SI5345_LOLB_N  ), //This output pin indicates when the DSPLL is locked (high) or out of lock (low). 
    .SI5345_FINC    (SI5345_FINC    ), //step-up the output frequency
    .SI5345_FDEC    (SI5345_FDEC    ), //step-down the output frequency  
    .SI5345_IN_SEL0 (SI5345_IN_SEL0 ), //The IN_SEL[1:0] pins are used in manual pin ...
    .SI5345_IN_SEL1 (SI5345_IN_SEL1 ), //controlled mode to select the active clock input
    .SI5345_OEB     (SI5345_OEB     ), //This pin disables all outputs when held high.
    .SI5345_RSTB_N  (SI5345_RSTB_N  ), //Active low input that performs power-on reset (POR) of the device.
    .SI5345_SCK_O   (SI5345_SCK_O   ), //This pin functions as the serial clock input for I2C
    .SI5345_SDA_IO  (SI5345_SDA_IO  )   //This is the bidirectional data pin (SDA) for the I2C mode
                                    
//	.I_CPU_JTAG_TCK (I_CPU_JTAG_TCK ),
//	.I_CPU_JTAG_TDI (I_CPU_JTAG_TDI ),
//	.O_CPU_JTAG_TDO (O_CPU_JTAG_TDO ),
//	.I_CPU_JTAG_TMS (I_CPU_JTAG_TMS )
);   
   
endmodule 
