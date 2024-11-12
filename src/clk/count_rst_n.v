`timescale 1ns / 1ps
//********************************************************************** 
// -------------------------------------------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>Copyright Notice<<<<<<<<<<<<<<<<<<<<<<<<<<<< 
// ------------------------------------------------------------------- 
//             /\ --------------- 
//            /  \ ------------- 
//           / /\ \ -----------
//          / /  \ \ ---------
//         / /    \ \ ------- 
//        / /      \ \ ----- 
//       / /_ _ _   \ \ --- 
//      /_ _ _ _ _\  \_\ -
//*********************************************************************** 
// Author: suluyang 
// Email:luyang.su@anlogic.com 
// Date:2020/06/20 
// Description: 
// 
// 
// 
// 
// web：www.anlogic.com 
//------------------------------------------------------------------- 
//*********************************************************************/
module count_rst_n#
(
	parameter num = 32'h0000ffff
)(
	input 	clk_i,
	// input 	rst_n_i,
	output  reg rstb_n,
	output  reg rst_o
);

reg[31:0]  cnt = 32'd0;//synthesis keep
reg        cnt_valid=1;

reg [3:0] cnt1;
wire      end_cnt1;
wire      add_cnt1;

//计数器2
always @(posedge clk_i )
begin
	if(add_cnt1)begin
		if(end_cnt1)
			cnt1 <= 0;
		else
			cnt1 <= cnt1 + 1;
	end
end

assign add_cnt1 = (cnt == num ) && (cnt_valid);
assign end_cnt1 = add_cnt1 && cnt1 == 10;

always@(posedge clk_i )
begin
	if(end_cnt1)
		cnt_valid <=  1'd0;
end

always@(posedge clk_i )
begin
	if(cnt == num)
		cnt <=  'd0;
	else if(cnt_valid)
		cnt <=  cnt + 1'd1 ;
end

always @(posedge clk_i )
begin
	if(end_cnt1)
	begin
		rstb_n <= 1;
		rst_o  <= 1;
	end
	else if(add_cnt1 && cnt1 == 1)
	begin
		rstb_n <= 0;
		rst_o  <= 0;
	end	
	else if(add_cnt1 && cnt1 == 2)
	begin
		rstb_n <= 1;
		rst_o  <= 0;
	end			
end	

endmodule

