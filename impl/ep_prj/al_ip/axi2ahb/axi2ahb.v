module axi2ahb
(
  input                         aclk,
  input                         aresetn,
  input   [3:0]                 us_axi_awid,
  input   [7:0]                 us_axi_awlen,
  input   [2:0]                 us_axi_awsize,
  input   [1:0]                 us_axi_awburst,
  input   [3:0]                 us_axi_awcache,
  input   [31:0]                us_axi_awaddr,
  input   [2:0]                 us_axi_awprot,
  input                         us_axi_awvalid,
  output                        us_axi_awready,
  input                         us_axi_awlock,
  input   [31:0]                us_axi_wdata,
  input   [3:0]                 us_axi_wstrb,
  input                         us_axi_wlast,
  input                         us_axi_wvalid,
  output                        us_axi_wready,
  output  [3:0]                 us_axi_bid,
  output  [1:0]                 us_axi_bresp,
  output                        us_axi_bvalid,
  input                         us_axi_bready,
  input   [3:0]                 us_axi_arid,
  input   [31:0]                us_axi_araddr,
  input   [2:0]                 us_axi_arprot,
  input   [3:0]                 us_axi_arcache,
  input                         us_axi_arvalid,
  input   [7:0]                 us_axi_arlen,
  input   [2:0]                 us_axi_arsize,
  input   [1:0]                 us_axi_arburst,
  input                         us_axi_arlock,
  output                        us_axi_arready,
  output  [3:0]                 us_axi_rid,
  output  [31:0]                us_axi_rdata,
  output  [1:0]                 us_axi_rresp,
  output                        us_axi_rvalid,
  output                        us_axi_rlast,
  input                         us_axi_rready,
  output  [31:0]                ds_ahb_haddr,
  output                        ds_ahb_hwrite,
  output  [2:0]                 ds_ahb_hsize,
  output  [2:0]                 ds_ahb_hburst,
  output  [3:0]                 ds_ahb_hprot,
  output  [1:0]                 ds_ahb_htrans,
  output                        ds_ahb_hmastlock,
  output  [31:0]                ds_ahb_hwdata,
  input                         ds_ahb_hready,
  input   [31:0]                ds_ahb_hrdata,
  input                         ds_ahb_hresp
);

  axi_ahblite_top_45df1ab5d994
  #(
      .AL_US_AXI_ADDR_WIDTH(32),
      .AL_US_AXI_DATA_WIDTH(32),
      .AL_US_AXI_ID_WIDTH(4),
      .AL_DS_AHB_ADDR_WIDTH(32),
      .AL_DS_AHB_DATA_WIDTH(32),
      .AL_SUPPORTS_NARROW_BURST(0),
      .AL_TIMEOUT_CYCLE(0)
  )axi_ahblite_top_45df1ab5d994_Inst
  (
      .aclk(aclk),
      .aresetn(aresetn),
      .us_axi_awid(us_axi_awid),
      .us_axi_awlen(us_axi_awlen),
      .us_axi_awsize(us_axi_awsize),
      .us_axi_awburst(us_axi_awburst),
      .us_axi_awcache(us_axi_awcache),
      .us_axi_awaddr(us_axi_awaddr),
      .us_axi_awprot(us_axi_awprot),
      .us_axi_awvalid(us_axi_awvalid),
      .us_axi_awready(us_axi_awready),
      .us_axi_awlock(us_axi_awlock),
      .us_axi_wdata(us_axi_wdata),
      .us_axi_wstrb(us_axi_wstrb),
      .us_axi_wlast(us_axi_wlast),
      .us_axi_wvalid(us_axi_wvalid),
      .us_axi_wready(us_axi_wready),
      .us_axi_bid(us_axi_bid),
      .us_axi_bresp(us_axi_bresp),
      .us_axi_bvalid(us_axi_bvalid),
      .us_axi_bready(us_axi_bready),
      .us_axi_arid(us_axi_arid),
      .us_axi_araddr(us_axi_araddr),
      .us_axi_arprot(us_axi_arprot),
      .us_axi_arcache(us_axi_arcache),
      .us_axi_arvalid(us_axi_arvalid),
      .us_axi_arlen(us_axi_arlen),
      .us_axi_arsize(us_axi_arsize),
      .us_axi_arburst(us_axi_arburst),
      .us_axi_arlock(us_axi_arlock),
      .us_axi_arready(us_axi_arready),
      .us_axi_rid(us_axi_rid),
      .us_axi_rdata(us_axi_rdata),
      .us_axi_rresp(us_axi_rresp),
      .us_axi_rvalid(us_axi_rvalid),
      .us_axi_rlast(us_axi_rlast),
      .us_axi_rready(us_axi_rready),
      .ds_ahb_haddr(ds_ahb_haddr),
      .ds_ahb_hwrite(ds_ahb_hwrite),
      .ds_ahb_hsize(ds_ahb_hsize),
      .ds_ahb_hburst(ds_ahb_hburst),
      .ds_ahb_hprot(ds_ahb_hprot),
      .ds_ahb_htrans(ds_ahb_htrans),
      .ds_ahb_hmastlock(ds_ahb_hmastlock),
      .ds_ahb_hwdata(ds_ahb_hwdata),
      .ds_ahb_hready(ds_ahb_hready),
      .ds_ahb_hrdata(ds_ahb_hrdata),
      .ds_ahb_hresp(ds_ahb_hresp)
  );
endmodule
