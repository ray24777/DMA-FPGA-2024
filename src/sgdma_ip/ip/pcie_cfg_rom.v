/************************************************************\
 **     Copyright (c) 2012-2023 Anlogic Inc.
 **  All Right Reserved.\
\************************************************************/
/************************************************************\
 ** Log	:	This file is generated by Anlogic IP Generator.
 ** File	:	/users/bswang/svn/sgdma128/src/sgdma_ip/ip/pcie_cfg_rom.v
 ** Date	:	2023 11 02
 ** TD version	:	5.6.88061
\************************************************************/

`timescale 1ns / 1ps

module pcie_cfg_rom ( doa, addra, ocea, clka, rsta );

	output [67:0] doa;

	input  [6:0] addra;
	input  ocea;
	input  clka;
	input  rsta;




	PH1_LOGIC_ERAM #( .DATA_WIDTH_A(68),
				.ADDR_WIDTH_A(7),
				.DATA_DEPTH_A(128),
				.DATA_WIDTH_B(68),
				.ADDR_WIDTH_B(7),
				.DATA_DEPTH_B(128),
				.MODE("SP"),
				.REGMODE_A("OUTREG"),
				.IMPLEMENT("20K"),
				.DEBUGGABLE("NO"),
				.PACKABLE("NO"),
				.INIT_FILE("pcie_ep_core.dat"),
				.FILL_ALL("NONE"))
			inst(
				.dia({68{1'b0}}),
				.dib({68{1'b0}}),
				.addra(addra),
				.addrb({7{1'b0}}),
				.cea(1'b1),
				.ceb(1'b0),
				.ocea(ocea),
				.oceb(1'b0),
				.clka(clka),
				.clkb(1'b0),
				.wea(1'b0),
				.web(1'b0),
				.bea(1'b0),
				.beb(1'b0),
				.rsta(rsta),
				.rstb(1'b0),
				.doa(doa),
				.dob());


endmodule