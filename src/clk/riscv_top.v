
`include "soft_mcu/soft_ps_wrapper.v"
module riscv_top (
    input   wire    clk,
    input   wire    rst_n,

    output wire   O_mcu_uart_tx,
    input wire    I_mcu_uart_rx,
    
    input SI5345_CLKIN_P,
    output reg test_led,
    
    input   wire  SI5345_INTRB_N, //This pin is asserted low when a change in device status has occurred    
    input   wire  SI5345_LOLB_N , //This output pin indicates when the DSPLL is locked (high) or out of lock (low). 
    output  wire  SI5345_FINC   , //step-up the output frequency
    output  wire  SI5345_FDEC   , //step-down the output frequency  
    output  wire  SI5345_IN_SEL0, //The IN_SEL[1:0] pins are used in manual pin ...
    output  wire  SI5345_IN_SEL1, //controlled mode to select the active clock input
    output  wire  SI5345_OEB    , //This pin disables all outputs when held high.
    output  wire  SI5345_RSTB_N , //Active low input that performs power-on reset (POR) of the device.
    inout   wire  SI5345_SCK_O  , //This pin functions as the serial clock input for I2C
    inout   wire  SI5345_SDA_IO,   //This is the bidirectional data pin (SDA) for the I2C mode

	input wire I_CPU_JTAG_TCK,
	input wire I_CPU_JTAG_TDI,
	output wire O_CPU_JTAG_TDO,
	input wire I_CPU_JTAG_TMS
);

assign          SI5345_FINC     = 0;//
assign          SI5345_FDEC     = 0;//
assign          SI5345_OEB      = 0;//
assign          SI5345_IN_SEL0  = 0;//
assign          SI5345_IN_SEL1  = 0;//

wire       S_clk_50m;
wire       S_pll_lock;
wire       soft_rst;

assign       S_clk_50m = clk;
assign       S_pll_lock= ~rst_n;

 
assign SI5345_RSTB_N = 1;


reg [31:0]cnt='d0;

always@(posedge SI5345_CLKIN_P)
begin
	if(cnt==200_000_000)
    	cnt<='d0;
    else
		cnt<=cnt+1;
end

always@(posedge SI5345_CLKIN_P)
begin
	if(cnt==100_000_000)
    	test_led <=1'b1;
    else if(cnt==0)
    	test_led <=1'b0;    
end

soft_ps_wrapper#(
    .SYSCLK               ( 25_000_000           ),
    .DEV_SERIES           ( "PH1"                ),
    .CORE_TYPE            ( "MEDIUM"             ),
    .TCM0_SIZE            ( 32*1024              ),
    .TCM0_INITFILE        ( "Si5345_IIC_IN3_100M_OUT2_100M.bin.mif" ),
    .TCM0_RAMSTYLE        ( "20K"                ),
    .UART1_BAUDRATE       ( 115200               ),
    .MTIME_ENABLE         ( 1'b0                 ),
    .GPIO_PINNUM          ( 32                   ),
    .GPIO_INTENABLE       ( 1'b1                 ),
    .GPIO_INTCFG          ( {45'h0,3'b101,48'h0} )
)u_soft_ps_wrapper(
    .I_clk                ( S_clk_50m            ),
    .I_rst                ( S_pll_lock           ),
    
    .O_uart1_tx           ( O_mcu_uart_tx        ),
    .I_uart1_rx           ( I_mcu_uart_rx        ),

    .IO_i2c_scl           (SI5345_SCK_O   ),
    .IO_i2c_sda           (SI5345_SDA_IO  ),
    .I_mcu_jtag_tck(I_CPU_JTAG_TCK),
    .I_mcu_jtag_tdi(I_CPU_JTAG_TDI),
    .O_mcu_jtag_tdo(O_CPU_JTAG_TDO),
    .I_mcu_jtag_tms(I_CPU_JTAG_TMS)
);

endmodule