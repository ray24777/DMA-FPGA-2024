`timescale  1ns/1ps
`define DLY #1

`define PH1A400_DEV 1'b1 //PH1A Series
`define REFCLK_WIDTH 2'd1

//`define C2H_TEST_MODE 1

`define MAX_PAYLOAD_SIZE 10'd512
`define HEADER_WIDTH 8'd128
`define DATA_WIDTH 8'd128
`define KEEP_WIDTH (`DATA_WIDTH/4'd8) 
`define NBYTE_WIDTH $clog2(`KEEP_WIDTH)
`define DSCP_WIDTH 9'd256 
`define LINK_WIDTH 4'd2    

`define C2H_CHN_NUM 2'd1 
`define H2C_CHN_NUM 2'd1 
`define DMA_CHN_NUM (`C2H_CHN_NUM+`H2C_CHN_NUM)

`define C2H_USR_IRQ 3'd1
`define H2C_USR_IRQ 3'd1
`define DMA_USR_IRQ (`C2H_USR_IRQ+`H2C_USR_IRQ)
 
`define H2C_STRT_ID 8'd0 
`define H2C_ID_NUM 8'd32 
`define H2C_NID_WD $clog2(`H2C_ID_NUM)
`define C2H_STRT_ID 8'd32
`define C2H_ID_NUM 8'd32  
`define H2C_SEP_NUM 4'd8
`define H2C_SNUM_WD $clog2(`H2C_SEP_NUM)
  
`define DSCP_MAGIC 16'had4b
`define WB_MAGIC 16'h52b4

//`define FIFO_AFULL_VAL 10'd508
 
`define MRD_3DW 8'b000_00000
`define MRD_4DW 8'b001_00000 
`define MWR_3DW 8'b010_00000
`define MWR_4DW 8'b011_00000 

`define REQUESTER_ID 16'h0100 
`define CPLD_FMT_TYPE 8'b010_01010 

`define AXIS_BUS0_EN 1'b1
//`define AXIS_BUS1_EN 1'b0
//`define AXI4_BUS0_EN 1'b0
`define AXIL_MBUS0_EN 1'b1
`define AXIL_SBUS0_EN 1'b1
`define USR_IRQ_EN 1'b1
`define MSI_EN   1'b1
`define CFG_MGMT_EN 1'b1

`define DMA_BS_ADDRH 1'b0
`define USR_BS_ADDRH 1'b1

`define XALI0_SIG_EN
`define XALI1_SIG_EN
//`define TRGT1_SIG_EN
`define BYPASS_SIG_EN
`define ELBI_SIG_EN
`define DBI_SIG_EN

`define FIFO_DEPTH (13'd4096/`KEEP_WIDTH)
`define FIFO_DEP_WD $clog2(`FIFO_DEPTH)
