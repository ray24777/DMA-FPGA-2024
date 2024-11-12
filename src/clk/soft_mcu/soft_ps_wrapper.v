
module soft_ps_wrapper #(
        parameter SYSCLK         = 70000000             ,
        parameter DEV_SERIES     = "PH1"                ,
        parameter CORE_TYPE      = "MEDIUM"             ,
        parameter TCM0_SIZE      = 32*1024              ,//fixed address at 0x80000000,BOOTRAM`RAM_VOL
        parameter TCM0_INITFILE  = "mcu_test.bin.mif"   ,//NONE    
        parameter TCM0_RAMSTYLE  = "20K"                ,    
        parameter UART1_BAUDRATE = 115200               ,
        parameter MTIME_ENABLE   = 1'b0                 ,	
        parameter GPIO_PINNUM    = 32                   ,
        parameter GPIO_INTENABLE = 1'b1                 , 
        parameter GPIO_INTCFG    = {45'h0,3'b101,48'h0} 
    )(
    input wire                   I_clk,
    input wire                   I_rst,
              
    input wire                   I_mcu_jtag_tms,
    input wire                   I_mcu_jtag_tdi,
    output wire                  O_mcu_jtag_tdo,
    input wire                   I_mcu_jtag_tck,

    input wire[GPIO_PINNUM-1:0]  I_gpio_in,
    output wire[GPIO_PINNUM-1:0] O_gpio_out,
    output wire[GPIO_PINNUM-1:0] O_gpio_out_en,

    input wire                   I_spi_miso,
    output wire                  O_spi_mosi,
    output wire                  O_spi_clk,
    output wire                  O_spi_cs,

    input wire                   I_spi2_miso,
    output wire                  O_spi2_mosi,
    output wire                  O_spi2_clk,
    output wire                  O_spi2_cs,

    output wire                  O_uart1_tx,
    input wire                   I_uart1_rx,

    output wire                  O_uart2_tx,
    input wire                   I_uart2_rx,

    inout wire                   IO_i2c_scl,  
    inout wire                   IO_i2c_sda,  

    output wire[19:0]            O_user_apb_PADDR,
    output wire                  O_user_apb_PSEL,
    output wire                  O_user_apb_PENABLE,
    output wire                  O_user_apb_PWRITE,
    output wire[31:0]            O_user_apb_PWDATA,
    input  wire                  I_user_apb_PREADY,
    input  wire[31:0]            I_user_apb_PRDATA,
    input  wire                  I_user_apb_PSLVERROR,
    input  wire                  I_user_apb_int
);
    

    wire                  S_i2c_scl_i;  
    wire                  S_i2c_scl_o;  
    wire                  S_i2c_scl_dir;
    wire                  S_i2c_sda_i;  
    wire                  S_i2c_sda_o;  
    wire                  S_i2c_sda_dir;


    assign IO_i2c_scl  = S_i2c_scl_dir ? S_i2c_scl_o : 1'bz;
    assign S_i2c_scl_i = IO_i2c_scl;

    assign IO_i2c_sda  = S_i2c_sda_dir ? S_i2c_sda_o : 1'bz;
    assign S_i2c_sda_i = IO_i2c_sda;


    RV_SoC_Top#(
        .SYSCLK         ( SYSCLK         ),
        .DEV_SERIES     ( DEV_SERIES     ),
        .CORE_TYPE      ( CORE_TYPE      ),
        .TCM0_SIZE      ( TCM0_SIZE      ),//fixed address at 0x80000000,BOOTRAM`RAM_VOL
        .TCM0_INITFILE  ( TCM0_INITFILE  ),//NONE    
        .TCM0_RAMSTYLE  ( TCM0_RAMSTYLE  ),    
        .UART1_BAUDRATE ( UART1_BAUDRATE ),
        .MTIME_ENABLE   ( MTIME_ENABLE   ),	
        .GPIO_PINNUM    ( GPIO_PINNUM    ),
        .GPIO_INTENABLE ( GPIO_INTENABLE ), 
        .GPIO_INTCFG    ( GPIO_INTCFG    )
    )SoC_Demo(
        //Clock&Reset    
        .clk                  ( I_clk                ),
        .reset                ( I_rst                ),
        //GPIO A
        .io_gpioA_read        ( I_gpio_in            ),
        .io_gpioA_write       ( O_gpio_out           ),
        .io_gpioA_writeEnable ( O_gpio_out_en        ),
        //SPI 
        .spi_miso             ( I_spi_miso           ),
        .spi_mosi             ( O_spi_mosi           ),
        .spi_mclk             ( O_spi_clk            ),
        .spi_mcs              ( O_spi_cs             ),        
    
        .spi2_miso            ( I_spi2_miso          ),
        .spi2_mosi            ( O_spi2_mosi          ),
        .spi2_mclk            ( O_spi2_clk           ),
        .spi2_mcs             ( O_spi2_cs            ),   
        //UART 
        .uart1_tx             ( O_uart1_tx           ),
        .uart1_rx             ( I_uart1_rx           ),

        .uart2_tx             ( O_uart2_tx           ),
        .uart2_rx             ( I_uart2_rx           ),    
        //I2C 
        .i2c_scl_i            ( S_i2c_scl_i          ),
        .i2c_scl_o            ( S_i2c_scl_o          ),
        .i2c_scl_dir          ( S_i2c_scl_dir        ),
        .i2c_sda_i            ( S_i2c_sda_i          ),
        .i2c_sda_o            ( S_i2c_sda_o          ),
        .i2c_sda_dir          ( S_i2c_sda_dir        ),
 
        .jtag_tms             ( I_mcu_jtag_tms       ),
        .jtag_tdi             ( I_mcu_jtag_tdi       ),
        .jtag_tdo             ( O_mcu_jtag_tdo       ),
        .jtag_tck             ( I_mcu_jtag_tck       ),
        //user APB 
        .userio_apb_PADDR     ( O_user_apb_PADDR     ),
        .userio_apb_PSEL      ( O_user_apb_PSEL      ),
        .userio_apb_PENABLE   ( O_user_apb_PENABLE   ),
        .userio_apb_PWRITE    ( O_user_apb_PWRITE    ),
        .userio_apb_PWDATA    ( O_user_apb_PWDATA    ),
        .userio_apb_PREADY    ( I_user_apb_PREADY    ),
        .userio_apb_PRDATA    ( I_user_apb_PRDATA    ),
        .userio_apb_PSLVERROR ( I_user_apb_PSLVERROR ),
        .userio_apb_int       ( I_user_apb_in        )

    );

endmodule



module RV_SoC_Top
#(
    parameter SYSCLK=50000000,
    DEV_SERIES="PH1",
	CORE_TYPE="SMALL",//SMALL/MIDDLE
    MTIME_ENABLE=1'b0,
    TCM0_SIZE=8192,//fixed address at 0x80000000,BOOTRAM
    TCM0_RAMSTYLE="9K(FAST)",
    TCM0_INITFILE="NONE",
    UART1_BAUDRATE=921600,
    UART1_DATABIT=8,
    UART1_PARITY="None",
    UART1_STOPBIT="1",
    GPIO_PINNUM=32,
    GPIO_INTENABLE = 1'b0 ,
    GPIO_INTCFG={45'h0,3'b101,48'h0},
    SPI_DEVNUM=3'd1
)
(
    //GPIO A
    input      [GPIO_PINNUM-1:0]   io_gpioA_read,
    output     [GPIO_PINNUM-1:0]   io_gpioA_write,
    output     [GPIO_PINNUM-1:0]   io_gpioA_writeEnable,
    //SPI 
    input spi_miso,
    output spi_mosi,
    output spi_mclk,
    output spi_mcs,
	//SPI2	    
	input spi2_miso,
    output spi2_mosi,
    output spi2_mclk,
    output spi2_mcs,
    //I2C 
    input i2c_scl_i,
    output i2c_scl_o,
    output i2c_scl_dir,
    input i2c_sda_i,
    output i2c_sda_o,
    output i2c_sda_dir,
    //UART1
    output uart1_tx,
    input uart1_rx,
    //UART2
    output uart2_tx,
    input uart2_rx,
    //User APB
    output     [19:0]   userio_apb_PADDR,
    output     [0:0]    userio_apb_PSEL,
    output              userio_apb_PENABLE,
    output              userio_apb_PWRITE,
    output     [31:0]   userio_apb_PWDATA,
    input               userio_apb_PREADY,
    input      [31:0]   userio_apb_PRDATA,
    input               userio_apb_PSLVERROR,
    input               userio_apb_int,
    //JTAG
    input               jtag_tms,
    input               jtag_tdi,
    output              jtag_tdo,
    input               jtag_tck,
    //Clock&Reset
    input               clk,
    input               reset
    
);
    wire       [31:0]   iBusAhb_HADDR;
    wire       [2:0]    iBusAhb_HSIZE;
    wire       [2:0]    iBusAhb_HBURST;
    wire       [1:0]    iBusAhb_HTRANS;
    wire       [31:0]   iBusAhb_HRDATA;
    wire                iBusAhb_HREADY;
    wire                iBusAhb_HRESP;
    wire       [31:0]   dBusAhb_HADDR;
    wire                dBusAhb_HWRITE;
    wire       [2:0]    dBusAhb_HSIZE;
    wire       [2:0]    dBusAhb_HBURST;
    wire       [1:0]    dBusAhb_HTRANS;
    wire       [31:0]   dBusAhb_HWDATA;
    wire       [31:0]   dBusAhb_HRDATA;
    wire                dBusAhb_HREADY;
    wire                dBusAhb_HRESP;  
    wire [5:0] int_arr;
    wire SysReset;
//AHB_Core#(
//    .CORE_TYPE(CORE_TYPE),//SMALL/MIDDLE
//    .INT_NUM(6),
//    .TIC_MTIME_SEL(MTIME_ENABLE),
//    .CLSIC_BASEADDR(16'h1000)//HIGH 16 BIT!!!
//)
eMCU CORE(
    .INTERRUPT_i(int_arr),
    .iBusAhb_HADDR(iBusAhb_HADDR),
    .iBusAhb_HSIZE(iBusAhb_HSIZE),
    .iBusAhb_HBURST(iBusAhb_HBURST),
    .iBusAhb_HTRANS(iBusAhb_HTRANS),
    .iBusAhb_HRDATA(iBusAhb_HRDATA),
    .iBusAhb_HREADY(iBusAhb_HREADY),
    .iBusAhb_HRESP(iBusAhb_HRESP),
    .dBusAhb_HADDR(dBusAhb_HADDR),
    .dBusAhb_HWRITE(dBusAhb_HWRITE),
    .dBusAhb_HSIZE(dBusAhb_HSIZE),
    .dBusAhb_HBURST(dBusAhb_HBURST),
    .dBusAhb_HTRANS(dBusAhb_HTRANS),
    .dBusAhb_HWDATA(dBusAhb_HWDATA),
    .dBusAhb_HRDATA(dBusAhb_HRDATA),
    .dBusAhb_HREADY_OUT(dBusAhb_HREADY),
    .dBusAhb_HRESP(dBusAhb_HRESP),
    .CORE_TMS(jtag_tms),
    .CORE_TDI(jtag_tdi),
    .CORE_TDO(jtag_tdo),
    .CORE_TCK(jtag_tck),
    .SYS_CLK(clk),
    .TIM_CLK(clk),
    .SYS_RST_o(SysReset),
    .POR_RST_i(reset)
);


wire iTCM_HSEL,iXIP_HSEL;
wire iTCM_HREADY,iXIP_HREADY;
wire iTCM_HRESP,iXIP_HRESP;
wire [31:0]iXIP_HRDATA,iTCM_HRDATA;

AHB_DecMux8#(
  .DEV0ENABLE(1'b1),
  .DEV0BASE  (32'h8000_0000),
  .DEV0RANGE ($clog2(TCM0_SIZE)),
  .DEV1ENABLE(1'b0),
  .DEV2ENABLE(1'b0),
  .DEV3ENABLE(1'b0),
  .DEV4ENABLE(1'b0),
  .DEV5ENABLE(1'b0),
  .DEV6ENABLE(1'b0),
  .DEV7ENABLE(1'b0)
)IMUX
(
    .HCLK(clk),
    .HRESETn(!SysReset),
    .HADDR(iBusAhb_HADDR),
    .HSELx0(iTCM_HSEL),
    .HSELx1(iXIP_HSEL),
    .HRDATAx0(iTCM_HRDATA),
    .HRDATAx1(iXIP_HRDATA),
    .HRDATAx2(32'h0),
    .HRDATAx3(32'h0),
    .HRDATAx4(32'h0),
    .HRDATAx5(32'h0),
    .HRDATAx6(32'h0),
    .HRDATAx7(32'h0),

    .HREADYx0(iTCM_HREADY),
    .HREADYx1(iXIP_HREADY),
    .HREADYx2(1'b1),
    .HREADYx3(1'b1),
    .HREADYx4(1'b1),
    .HREADYx5(1'b1),
    .HREADYx6(1'b1),
    .HREADYx7(1'b1),

    .HRESPx0(iTCM_HRESP),
    .HRESPx1(iXIP_HRESP),
    .HRESPx2(1'b0),
    .HRESPx3(1'b0),
    .HRESPx4(1'b0),
    .HRESPx5(1'b0),
    .HRESPx6(1'b0),
    .HRESPx7(1'b0),

    .HREADY(iBusAhb_HREADY),
    .HRESP(iBusAhb_HRESP),
    .HRDATA(iBusAhb_HRDATA)
);

wire TCM_HSEL,APB_HSEL,dXIP_HSEL;
wire TCM_HREADY,APB_HREADY,dXIP_HREADY;
wire TCM_HRESP,APB_HRESP,dXIP_HRESP;
wire [31:0]TCM_HRDATA,APB_HRDATA,dXIP_HRDATA;

AHB_DecMux8#(
  .DEV0ENABLE(1'b1),
  .DEV0BASE  (32'h8000_0000),
  .DEV0RANGE ($clog2(TCM0_SIZE)),
  .DEV1ENABLE(1'b0),
  .DEV2ENABLE(1'b1),
  .DEV2BASE  (32'hF000_0000),
  .DEV2RANGE (21),
  .DEV3ENABLE(1'b0),
  .DEV4ENABLE(1'b0),
  .DEV5ENABLE(1'b0),
  .DEV6ENABLE(1'b0),
  .DEV7ENABLE(1'b0)
)DMUX
(
    .HCLK(clk),
    .HRESETn(!SysReset),
    .HADDR(dBusAhb_HADDR),
    .HSELx0(dTCM_HSEL),
    .HSELx1(dXIP_HSEL),
    .HSELx2(APB_HSEL),

    .HRDATAx0(TCM_HRDATA),
    .HRDATAx1(dXIP_HRDATA),
    .HRDATAx2(APB_HRDATA),
    .HRDATAx3(32'h0),
    .HRDATAx4(32'h0),
    .HRDATAx5(32'h0),
    .HRDATAx6(32'h0),
    .HRDATAx7(32'h0),

    .HREADYx0(TCM_HREADY),
    .HREADYx1(dXIP_HREADY),
    .HREADYx2(APB_HREADY),
    .HREADYx3(1'b1),
    .HREADYx4(1'b1),
    .HREADYx5(1'b1),
    .HREADYx6(1'b1),
    .HREADYx7(1'b1),

    .HRESPx0(TCM_HRESP),
    .HRESPx1(dXIP_HRESP),
    .HRESPx2(APB_HRESP),
    .HRESPx3(1'b0),
    .HRESPx4(1'b0),
    .HRESPx5(1'b0),
    .HRESPx6(1'b0),
    .HRESPx7(1'b0),

    .HREADY(dBusAhb_HREADY),
    .HRESP(dBusAhb_HRESP),
    .HRDATA(dBusAhb_HRDATA)
);

AHB_DPMEM#(
  .TCM_SIZE(TCM0_SIZE), //In Bytes
  .TCM_RAMSTYLE(TCM0_RAMSTYLE),
  .DEV_SERIES(DEV_SERIES),
  .INIT_FILE(TCM0_INITFILE)
)TCM0(
    //port A for I fetch
    .AhbA_HSEL(iTCM_HSEL),
    .AhbA_HADDR(iBusAhb_HADDR),
    .AhbA_HWRITE(1'b0),
    .AhbA_HSIZE(3'b010),
    .AhbA_HBURST(iBusAhb_HBURST),
    .AhbA_HTRANS(iBusAhb_HTRANS),
    .AhbA_HWDATA(32'hxxxxxxxx),
    .AhbA_HRDATA(iTCM_HRDATA),
    .AhbA_HREADY_OUT(iTCM_HREADY),
    .AhbA_HRESP(iTCM_HRESP),
    //port B for D side
    .AhbB_HSEL(dTCM_HSEL),
    .AhbB_HADDR(dBusAhb_HADDR),
    .AhbB_HWRITE(dBusAhb_HWRITE),
    .AhbB_HSIZE(dBusAhb_HSIZE),
    .AhbB_HBURST(dBusAhb_HBURST),
    .AhbB_HTRANS(dBusAhb_HTRANS),
    .AhbB_HWDATA(dBusAhb_HWDATA),
    .AhbB_HRDATA(TCM_HRDATA),
    .AhbB_HREADY_OUT(TCM_HREADY),
    .AhbB_HRESP(TCM_HRESP),
    .AHB_CLK(clk),
    .AHB_RST_i(SysReset)
);

wire internal_miso,internal_mosi,internal_mclk,internal_mcs;
wire internal_xip_clk;
wire internal_xip_cs;
wire [3:0]internal_xip_o;
wire [3:0]internal_xip_dir;

assign iXIP_HRDATA=32'h0;
assign iXIP_HREADY=1'b1;
assign iXIP_HRESP=1'b0;
assign dXIP_HRDATA=32'h0;
assign dXIP_HREADY=1'b1;
assign dXIP_HRESP=1'b0;
assign internal_miso=spi_miso;
assign spi_mosi=internal_mosi;
assign spi_mclk=internal_mclk;
assign spi_mcs=internal_mcs;


//APB is Data-ONLY 
wire     [20:0]   system_apb_PADDR;
wire     [0:0]    system_apb_PSEL;
wire              system_apb_PENABLE;
wire              system_apb_PREADY;
wire              system_apb_PWRITE;
wire     [31:0]   system_apb_PWDATA;
wire     [31:0]   system_apb_PRDATA;
wire              system_apb_PSLVERROR;

AHB_APB_sync APB_Bridge
(
    .AHB2APB_HSEL(APB_HSEL),
    .AHB2APB_HADDR(dBusAhb_HADDR),
    .AHB2APB_HWRITE(dBusAhb_HWRITE),
    .AHB2APB_HTRANS(dBusAhb_HTRANS),
    .AHB2APB_HSIZE(dBusAhb_HSIZE),
    .AHB2APB_HBURST(dBusAhb_HBURST),
    .AHB2APB_HWDATA(dBusAhb_HWDATA),
    .AHB_RST_i(SysReset),
    .AHB_CLK(clk),
    .AHB2APB_HREADY_OUT(APB_HREADY),
    .AHB2APB_HRESP(APB_HRESP),
    .AHB2APB_HRDATA(APB_HRDATA),
    .AHB2APB_PADDR(system_apb_PADDR),
    .AHB2APB_PSEL(system_apb_PSEL),
    .AHB2APB_PENABLE(system_apb_PENABLE),
    .AHB2APB_PREADY(system_apb_PREADY),
    .AHB2APB_PWRITE(system_apb_PWRITE),
    .AHB2APB_PWDATA(system_apb_PWDATA),
    .AHB2APB_PRDATA(system_apb_PRDATA),
    .AHB2APB_PSLVERR(system_apb_PSLVERROR)
);


//APB BusMux
wire UART1_PREADY,
     GPIO_PREADY,SPI_PREADY,SPI2_PREADY,
     I2C_PREADY,USER_APB_PREADY;
wire UART1_PSLVERROR,
     GPIO_PSLVERROR,SPI_PSLVERROR,SPI2_PSLVERROR,
     I2C_PSLVERROR,USER_APB_PSLVERROR;
wire [31:0] UART1_PRDATA,
            GPIO_PRDATA,SPI_PRDATA,SPI2_PRDATA,
            I2C_PRDATA,USER_APB_PRDATA;

APB_DecMux8 #(
    .DEV0ENABLE(1'b1),
    .DEV0BASE  (32'h8000_0000),
    .DEV0RANGE (8),
    .DEV1ENABLE(1'b1),
    .DEV1BASE  (32'h8000_0100),
    .DEV1RANGE (8),
    .DEV2ENABLE(1'b1),
    .DEV2BASE  (32'h8000_0200),
    .DEV2RANGE (8),
    .DEV3ENABLE(1'b1),
    .DEV3BASE  (32'h8000_0300),
    .DEV3RANGE (8),
    .DEV4ENABLE(1'b1),
    .DEV4BASE  (32'h8010_0000),
    .DEV4RANGE (20),
    .DEV5ENABLE(1'b1),    
    .DEV5BASE  (32'h8000_0400),
	.DEV5RANGE (8),
    .DEV6ENABLE(1'b0),
    .DEV7ENABLE(1'b0)
)
PMUX(
    .PADDR({system_apb_PSEL,10'h00,system_apb_PADDR}),

    .PRDATAx0(UART1_PRDATA),
    .PRDATAx1(GPIO_PRDATA),
    .PRDATAx2(SPI_PRDATA),
    .PRDATAx3(I2C_PRDATA),
    .PRDATAx4(USER_APB_PRDATA),
    .PRDATAx5(32'hxxxxxxxx),
    .PRDATAx6(32'hxxxxxxxx),
    .PRDATAx7(32'hxxxxxxxx),

    .PSELx0(UART1_SEL),
    .PSELx1(GPIO_SEL),
    .PSELx2(SPI_SEL),
    .PSELx3(I2C_SEL),
    .PSELx4(userio_apb_PSEL),    
	.PSELx5(SPI2_PSEL),

    .PREADYx0(UART1_PREADY),
    .PREADYx1(GPIO_PREADY),
    .PREADYx2(SPI_PREADY),
    .PREADYx3(I2C_PREADY),
    .PREADYx4(USER_APB_PREADY),
    .PREADYx5(SPI2_PREADY),
    .PREADYx6(1'b1),
    .PREADYx7(1'b1),

    .PSLVERRORx0(1'b0),
    .PSLVERRORx1(1'b0),
    .PSLVERRORx2(1'b0),
    .PSLVERRORx3(1'b0),
    .PSLVERRORx4(USER_APB_PSLVERROR),
    .PSLVERRORx5(1'b0),
    .PSLVERRORx6(1'b0),
    .PSLVERRORx7(1'b0),

    .PREADY(system_apb_PREADY),
    .PSLVERROR(system_apb_PSLVERROR),
    .PRDATA(system_apb_PRDATA)
);

//APB Peripheral Gen
wire uart1_int,spi_finishint,i2c_int,usrapb_int,gpio_int;
//UART1
APB_UART #(
    .STYLE("FIXED"),
    .CLKRATE(SYSCLK),
    .BAUDRATE(UART1_BAUDRATE),
    .DATA_BIT(UART1_DATABIT),//数据位宽
    .PARITY_BIT(UART1_PARITY),//校验方式--"None","Odd","Even"
    .STOP_BIT(UART1_STOPBIT)//停止位--"1","1.5","2"
)
APB_UART1(
    .APB2UART_PADDR(system_apb_PADDR),
    .APB2UART_PSEL(UART1_SEL),
    .APB2UART_PENABLE(system_apb_PENABLE),
    .APB2UART_PREADY(UART1_PREADY),
    .APB2UART_PWRITE(system_apb_PWRITE),
    .APB2UART_PWDATA(system_apb_PWDATA),
    .APB2UART_PRDATA(UART1_PRDATA),
    .TxD(uart1_tx),
    .RxD(uart1_rx),
    .APB2UART_INTERRUPT(uart1_int),
    .APB_CLK(clk),
    .APB_RST_i(SysReset)
);  
//Selectable Peripherals Generation

APB_GPIO #(
    .GPIO_NUM(GPIO_PINNUM),
    .GPIO_INTENABLE(GPIO_INTENABLE),
    .GPIO_INTCFG(GPIO_INTCFG)
) 
APB_GPIOA
(
    .PADDR(system_apb_PADDR),
    .PSEL(GPIO_SEL),
    .PENABLE(system_apb_PENABLE),
    .PREADY(GPIO_PREADY),
    .PWRITE(system_apb_PWRITE),
    .PWDATA(system_apb_PWDATA),
    .PRDATA(GPIO_PRDATA),
    .PSLVERROR(GPIO_PSLVERROR),
    .io_gpio_read(io_gpioA_read),
    .io_gpio_write(io_gpioA_write),
    .io_gpio_writeEnable(io_gpioA_writeEnable),
    .io_value(),
    .io_interrupt(gpio_int),
    .clk(clk),
    .rst(SysReset)
);
      

APB_SPI #(.DEVNUM(SPI_DEVNUM))
APB_SPI1
(
    .APB2SPI_PADDR(system_apb_PADDR),
    .APB2SPI_PSEL(SPI_SEL),
    .APB2SPI_PENABLE(system_apb_PENABLE),
    .APB2SPI_PREADY(SPI_PREADY),
    .APB2SPI_PWRITE(system_apb_PWRITE),
    .APB2SPI_PWDATA(system_apb_PWDATA),
    .APB2SPI_PRDATA(SPI_PRDATA),
    .APB2SPI_INTERRUPT(spi_finishint),
    .APB_CLK(clk),
    .APB_RST_i(SysReset),
    .MISO_i(spi_miso),
    .MOSI_o(spi_mosi),
    .SCK_O(spi_mclk),
    .SS_O(spi_mcs)
);

APB_SPI #(.DEVNUM(1'b1))
APB_SPI2
(
    .APB2SPI_PADDR(system_apb_PADDR),
    .APB2SPI_PSEL(SPI2_PSEL),
    .APB2SPI_PENABLE(system_apb_PENABLE),
    .APB2SPI_PREADY(SPI2_PREADY),
    .APB2SPI_PWRITE(system_apb_PWRITE),
    .APB2SPI_PWDATA(system_apb_PWDATA),
    .APB2SPI_PRDATA(SPI2_PRDATA),
    .APB2SPI_INTERRUPT(spi2_finishint),
    .APB_CLK(clk),
    .APB_RST_i(SysReset),
    .MISO_i(spi2_miso),
    .MOSI_o(spi2_mosi),
    .SCK_O(spi2_mclk),
    .SS_O(sp2i_mcs)
);

APB_IIC_MASTER APB_I2CMASTER
(
    .APB_CLK(clk),
    .APB_RST_i(SysReset),
    .APB2IIC_PADDR(system_apb_PADDR),
    .APB2IIC_PWRITE(system_apb_PWRITE),
    .APB2IIC_PSEL(I2C_SEL),
    .APB2IIC_PENABLE(system_apb_PENABLE),
    .APB2IIC_PWDATA(system_apb_PWDATA),
    .APB2IIC_PRDATA(I2C_PRDATA),
    .APB2IIC_PREADY(I2C_PREADY),
    .APB2IIC_INTERRUPT(i2c_int),
    .SCL_I(i2c_scl_i),       // SCL-line input
    .SCL_O(i2c_scl_o),       // SCL-line output (always 1'b0)
    .SCL_T(i2c_scl_dir),    // SCL-line output enable (active low)
    .SDA_I(i2c_sda_i),       // SDA-line input
    .SDA_O(i2c_sda_o),       // SDA-line output (always 1'b0)
    .SDA_T(i2c_sda_dir)    // SDA-line output enable (active low)
);
assign int_arr={1'b0,usrapb_int,i2c_int,gpio_int,spi_finishint,uart1_int};
assign USER_APB_PREADY=userio_apb_PREADY;
assign USER_APB_PRDATA=userio_apb_PRDATA;
assign USER_APB_PSLVERROR=userio_apb_PSLVERROR;
assign usrapb_int=userio_apb_int;
assign userio_apb_PADDR=system_apb_PADDR;
assign userio_apb_PENABLE=system_apb_PENABLE;
assign userio_apb_PWRITE=system_apb_PWRITE;
assign userio_apb_PWDATA=system_apb_PWDATA;

endmodule


module AHB_DecMux8
#(
  parameter DEV0ENABLE=1'b1,
            DEV0BASE  =32'h1000_0000,
            DEV0RANGE =20,
            DEV1ENABLE=1'b1,
            DEV1BASE  =32'h1000_0000,
            DEV1RANGE =20,
            DEV2ENABLE=1'b1,
            DEV2BASE  =32'h1000_0000,
            DEV2RANGE =4,
            DEV3ENABLE=1'b1,
            DEV3BASE  =32'h1000_0000,
            DEV3RANGE =4,
            DEV4ENABLE=1'b1,
            DEV4BASE  =32'h1000_0000,
            DEV4RANGE =4,
            DEV5ENABLE=1'b1,
            DEV5BASE  =32'h1000_0000,
            DEV5RANGE =4,
            DEV6ENABLE=1'b1,
            DEV6BASE  =32'h1000_0000,
            DEV6RANGE =4,
            DEV7ENABLE=1'b1,
            DEV7BASE  =32'h1000_0000,
            DEV7RANGE =4
)
(
  // -------------
  // Input pins //
  // -------------
  // Clock and reset
  input HCLK,
  input HRESETn,

  input [31:0] HADDR,
  // Slave datas
  input [31:0] HRDATAx0,
  input [31:0] HRDATAx1,
  input [31:0] HRDATAx2,
  input [31:0] HRDATAx3,
  input [31:0] HRDATAx4,
  input [31:0] HRDATAx5,
  input [31:0] HRDATAx6,
  input [31:0] HRDATAx7,
  // Control signals
  output HSELx0,
  output HSELx1,
  output HSELx2,
  output HSELx3,
  output HSELx4,
  output HSELx5,
  output HSELx6,
  output HSELx7,
  // Slave responses
  input HREADYx0,
  input HREADYx1,
  input HREADYx2,
  input HREADYx3,
  input HREADYx4,
  input HREADYx5,
  input HREADYx6,
  input HREADYx7,

  input HRESPx0,
  input HRESPx1,
  input HRESPx2,
  input HRESPx3,
  input HRESPx4,
  input HRESPx5,
  input HRESPx6,
  input HRESPx7, 
  // --------------
  // Output pins //
  // --------------
  output reg HREADY,
  output reg HRESP,
  output reg [31:0] HRDATA
);
  assign HSELx0= ((HADDR & (32'hFFFF_FFFF << DEV0RANGE)) ==  (DEV0BASE & (32'hFFFF_FFFF << DEV0RANGE))) & DEV0ENABLE;
  assign HSELx1= ((HADDR & (32'hFFFF_FFFF << DEV1RANGE)) ==  (DEV1BASE & (32'hFFFF_FFFF << DEV1RANGE))) & DEV1ENABLE;
  assign HSELx2= ((HADDR & (32'hFFFF_FFFF << DEV2RANGE)) ==  (DEV2BASE & (32'hFFFF_FFFF << DEV2RANGE))) & DEV2ENABLE;
  assign HSELx3= ((HADDR & (32'hFFFF_FFFF << DEV3RANGE)) ==  (DEV3BASE & (32'hFFFF_FFFF << DEV3RANGE))) & DEV3ENABLE;
  assign HSELx4= ((HADDR & (32'hFFFF_FFFF << DEV4RANGE)) ==  (DEV4BASE & (32'hFFFF_FFFF << DEV4RANGE))) & DEV4ENABLE;
  assign HSELx5= ((HADDR & (32'hFFFF_FFFF << DEV5RANGE)) ==  (DEV5BASE & (32'hFFFF_FFFF << DEV5RANGE))) & DEV5ENABLE;
  assign HSELx6= ((HADDR & (32'hFFFF_FFFF << DEV6RANGE)) ==  (DEV6BASE & (32'hFFFF_FFFF << DEV6RANGE))) & DEV6ENABLE;
  assign HSELx7= ((HADDR & (32'hFFFF_FFFF << DEV7RANGE)) ==  (DEV7BASE & (32'hFFFF_FFFF << DEV7RANGE))) & DEV7ENABLE;
  wire HSELx8;
  assign HSELx8=0;
reg [3:0] slave_select;
  // Slave select register update
  always @ (posedge HCLK or negedge HRESETn) begin
    if(!HRESETn) begin
      slave_select <= 4'b0;
    end
    else begin
      // If HREADY is high, update slave_select
      if(HREADY) begin
        case({HSELx8,HSELx7, HSELx6, HSELx5, HSELx4, HSELx3, HSELx2, HSELx1, HSELx0})
          9'h001: slave_select <= 4'h0;
          9'h002: slave_select <= 4'h1;
          9'h004: slave_select <= 4'h2;
          9'h008: slave_select <= 4'h3;
          9'h010: slave_select <= 4'h4;
          9'h020: slave_select <= 4'h5;
          9'h040: slave_select <= 4'h6;
          9'h080: slave_select <= 4'h7;
          9'h100: slave_select <= 4'h8;
          // Default is for default slave
          default: slave_select <= 4'h8;
        endcase
      end
    end
  end
  always @ (*) begin
    case(slave_select)
      4'h0: HRDATA = HRDATAx0;
      4'h1: HRDATA = HRDATAx1;
      4'h2: HRDATA = HRDATAx2;
      4'h3: HRDATA = HRDATAx3;
      4'h4: HRDATA = HRDATAx4;
      4'h5: HRDATA = HRDATAx5;
      4'h6: HRDATA = HRDATAx6;
      4'h7: HRDATA = HRDATAx7;
      default: HRDATA = 32'hx;
    endcase
  end
  // Response multiplexor
  always @ (*) begin
    case(slave_select)
      4'h0: begin
        HRESP = HRESPx0;
        HREADY = HREADYx0;
        end
      4'h1: begin
        HRESP = HRESPx1;
        HREADY = HREADYx1;
      end
      4'h2: begin
        HRESP = HRESPx2;
        HREADY = HREADYx2;
      end
      4'h3: begin
        HRESP = HRESPx3;
        HREADY = HREADYx3;
      end
      4'h4: begin
        HRESP = HRESPx4;
        HREADY = HREADYx4;
      end
      4'h5: begin
        HRESP = HRESPx5;
        HREADY = HREADYx5;
      end
      4'h6: begin
        HRESP = HRESPx6;
        HREADY = HREADYx6;
      end
    4'h7: begin
        HRESP = HRESPx7;
        HREADY = HREADYx7;
      end
      default: begin
        HRESP = 1'b0;
        HREADY = 1'b1;
      end
    endcase
  end
endmodule


module AHB_DecMux8
#(
  parameter DEV0ENABLE=1'b1,
            DEV0BASE  =32'h1000_0000,
            DEV0RANGE =20,
            DEV1ENABLE=1'b1,
            DEV1BASE  =32'h1000_0000,
            DEV1RANGE =20,
            DEV2ENABLE=1'b1,
            DEV2BASE  =32'h1000_0000,
            DEV2RANGE =4,
            DEV3ENABLE=1'b1,
            DEV3BASE  =32'h1000_0000,
            DEV3RANGE =4,
            DEV4ENABLE=1'b1,
            DEV4BASE  =32'h1000_0000,
            DEV4RANGE =4,
            DEV5ENABLE=1'b1,
            DEV5BASE  =32'h1000_0000,
            DEV5RANGE =4,
            DEV6ENABLE=1'b1,
            DEV6BASE  =32'h1000_0000,
            DEV6RANGE =4,
            DEV7ENABLE=1'b1,
            DEV7BASE  =32'h1000_0000,
            DEV7RANGE =4
)
(
  // -------------
  // Input pins //
  // -------------
  // Clock and reset
  input HCLK,
  input HRESETn,

  input [31:0] HADDR,
  // Slave datas
  input [31:0] HRDATAx0,
  input [31:0] HRDATAx1,
  input [31:0] HRDATAx2,
  input [31:0] HRDATAx3,
  input [31:0] HRDATAx4,
  input [31:0] HRDATAx5,
  input [31:0] HRDATAx6,
  input [31:0] HRDATAx7,
  // Control signals
  output HSELx0,
  output HSELx1,
  output HSELx2,
  output HSELx3,
  output HSELx4,
  output HSELx5,
  output HSELx6,
  output HSELx7,
  // Slave responses
  input HREADYx0,
  input HREADYx1,
  input HREADYx2,
  input HREADYx3,
  input HREADYx4,
  input HREADYx5,
  input HREADYx6,
  input HREADYx7,

  input HRESPx0,
  input HRESPx1,
  input HRESPx2,
  input HRESPx3,
  input HRESPx4,
  input HRESPx5,
  input HRESPx6,
  input HRESPx7, 
  // --------------
  // Output pins //
  // --------------
  output reg HREADY,
  output reg HRESP,
  output reg [31:0] HRDATA
);
  assign HSELx0= ((HADDR & (32'hFFFF_FFFF << DEV0RANGE)) ==  (DEV0BASE & (32'hFFFF_FFFF << DEV0RANGE))) & DEV0ENABLE;
  assign HSELx1= ((HADDR & (32'hFFFF_FFFF << DEV1RANGE)) ==  (DEV1BASE & (32'hFFFF_FFFF << DEV1RANGE))) & DEV1ENABLE;
  assign HSELx2= ((HADDR & (32'hFFFF_FFFF << DEV2RANGE)) ==  (DEV2BASE & (32'hFFFF_FFFF << DEV2RANGE))) & DEV2ENABLE;
  assign HSELx3= ((HADDR & (32'hFFFF_FFFF << DEV3RANGE)) ==  (DEV3BASE & (32'hFFFF_FFFF << DEV3RANGE))) & DEV3ENABLE;
  assign HSELx4= ((HADDR & (32'hFFFF_FFFF << DEV4RANGE)) ==  (DEV4BASE & (32'hFFFF_FFFF << DEV4RANGE))) & DEV4ENABLE;
  assign HSELx5= ((HADDR & (32'hFFFF_FFFF << DEV5RANGE)) ==  (DEV5BASE & (32'hFFFF_FFFF << DEV5RANGE))) & DEV5ENABLE;
  assign HSELx6= ((HADDR & (32'hFFFF_FFFF << DEV6RANGE)) ==  (DEV6BASE & (32'hFFFF_FFFF << DEV6RANGE))) & DEV6ENABLE;
  assign HSELx7= ((HADDR & (32'hFFFF_FFFF << DEV7RANGE)) ==  (DEV7BASE & (32'hFFFF_FFFF << DEV7RANGE))) & DEV7ENABLE;
  wire HSELx8;
  assign HSELx8=0;
reg [3:0] slave_select;
  // Slave select register update
  always @ (posedge HCLK or negedge HRESETn) begin
    if(!HRESETn) begin
      slave_select <= 4'b0;
    end
    else begin
      // If HREADY is high, update slave_select
      if(HREADY) begin
        case({HSELx8,HSELx7, HSELx6, HSELx5, HSELx4, HSELx3, HSELx2, HSELx1, HSELx0})
          9'h001: slave_select <= 4'h0;
          9'h002: slave_select <= 4'h1;
          9'h004: slave_select <= 4'h2;
          9'h008: slave_select <= 4'h3;
          9'h010: slave_select <= 4'h4;
          9'h020: slave_select <= 4'h5;
          9'h040: slave_select <= 4'h6;
          9'h080: slave_select <= 4'h7;
          9'h100: slave_select <= 4'h8;
          // Default is for default slave
          default: slave_select <= 4'h8;
        endcase
      end
    end
  end
  always @ (*) begin
    case(slave_select)
      4'h0: HRDATA = HRDATAx0;
      4'h1: HRDATA = HRDATAx1;
      4'h2: HRDATA = HRDATAx2;
      4'h3: HRDATA = HRDATAx3;
      4'h4: HRDATA = HRDATAx4;
      4'h5: HRDATA = HRDATAx5;
      4'h6: HRDATA = HRDATAx6;
      4'h7: HRDATA = HRDATAx7;
      default: HRDATA = 32'hx;
    endcase
  end
  // Response multiplexor
  always @ (*) begin
    case(slave_select)
      4'h0: begin
        HRESP = HRESPx0;
        HREADY = HREADYx0;
        end
      4'h1: begin
        HRESP = HRESPx1;
        HREADY = HREADYx1;
      end
      4'h2: begin
        HRESP = HRESPx2;
        HREADY = HREADYx2;
      end
      4'h3: begin
        HRESP = HRESPx3;
        HREADY = HREADYx3;
      end
      4'h4: begin
        HRESP = HRESPx4;
        HREADY = HREADYx4;
      end
      4'h5: begin
        HRESP = HRESPx5;
        HREADY = HREADYx5;
      end
      4'h6: begin
        HRESP = HRESPx6;
        HREADY = HREADYx6;
      end
    4'h7: begin
        HRESP = HRESPx7;
        HREADY = HREADYx7;
      end
      default: begin
        HRESP = 1'b0;
        HREADY = 1'b1;
      end
    endcase
  end
endmodule


module AHB_DPMEM
#(parameter TCM_SIZE=8192, //In Bytes
  DEV_SERIES="EG4",
  TCM_RAMSTYLE="9K",
  INIT_FILE="NONE"
)
(
    //port A
    input              AhbA_HSEL,
    input     [31:0]   AhbA_HADDR,
    input              AhbA_HWRITE,
    input     [2:0]    AhbA_HSIZE,
    input     [2:0]    AhbA_HBURST,
    input     [1:0]    AhbA_HTRANS,
    input     [31:0]   AhbA_HWDATA,
    output    [31:0]   AhbA_HRDATA,
    input              AhbA_HREADY_IN,
    output             AhbA_HREADY_OUT,
    output             AhbA_HRESP,
    //port B
    input              AhbB_HSEL,
    input     [31:0]   AhbB_HADDR,
    input              AhbB_HWRITE,
    input     [2:0]    AhbB_HSIZE,
    input     [2:0]    AhbB_HBURST,
    input     [1:0]    AhbB_HTRANS,
    input     [31:0]   AhbB_HWDATA,
    output    [31:0]   AhbB_HRDATA,
    input              AhbB_HREADY_IN,
    output             AhbB_HREADY_OUT,
    output             AhbB_HRESP,
    input AHB_CLK,AHB_RST_i
);
localparam TCM_DEPTH = TCM_SIZE/4;
localparam TCM_ADDRBitWid = $clog2(TCM_SIZE);
localparam TCM_RealBitWid=TCM_ADDRBitWid-2;

wire [TCM_RealBitWid-1:0]Addr_A,Addr_B;
reg [TCM_RealBitWid-1:0]wAddr_reg_A,wAddr_reg_B;
reg write_A,write_B;
reg [3:0]wCmd_mask_A,wCmd_mask_B;
//reg [1:0]hsize_writea,hsize_writeb;
assign Addr_A=(write_A) ? wAddr_reg_A : 
                  AhbA_HADDR[TCM_ADDRBitWid-1:2];
assign Addr_B=(write_B) ? wAddr_reg_B : 
                  AhbB_HADDR[TCM_ADDRBitWid-1:2];
//hsize_logic
always @(posedge AHB_CLK or posedge AHB_RST_i) //write management block
begin
  if(AHB_RST_i)
  begin
    write_A<=1'b0;
    write_B<=1'b0;
    wAddr_reg_A<=0;
    wAddr_reg_B<=0;
    wCmd_mask_A<=0;
    wCmd_mask_B<=0;
  end
  else
  begin
      if(AhbA_HSEL & AhbA_HWRITE & (AhbA_HTRANS!=2'b00))
        begin
          write_A<=1'b1;
          wAddr_reg_A<=AhbA_HADDR[TCM_ADDRBitWid-1:2];
          wCmd_mask_A<=hsize2mask(AhbA_HSIZE,AhbA_HADDR[1:0]);
        end
      else
        begin
          write_A<=1'b0;
          wCmd_mask_A<=0;
        end
      if(AhbB_HSEL & AhbB_HWRITE & (AhbB_HTRANS!=2'b00))
      begin
        write_B<=1'b1;
        wAddr_reg_B<=AhbB_HADDR[TCM_ADDRBitWid-1:2];
        wCmd_mask_B<=hsize2mask(AhbB_HSIZE,AhbB_HADDR[1:0]);
      end
      else
      begin
        write_B<=1'b0; 
        wCmd_mask_B<=0;
      end   
  end
end
always@(posedge AHB_CLK)
begin
  if(wCmd_mask_A!=0)
  begin
    $display("TCM Port A Write Event! WMask=%b WAddr=%h Commited Data=%h \n",wCmd_mask_A,{Addr_A,2'b00},AhbA_HWDATA);
  end
  if(wCmd_mask_B!=0)
  begin
    $display("TCM Port B Write Event! WMask=%b WAddr=%h Commited Data=%h \n",wCmd_mask_B,{Addr_B,2'b00},AhbB_HWDATA);
  end
end
DPBRAM #(
  .INIT_FILE(INIT_FILE),
  .DATA_WIDTH(32),
  .DEV_SERIES(DEV_SERIES),
  .RAM_STYLE(TCM_RAMSTYLE),
  .ADDR_WIDTH(TCM_RealBitWid),
  .DATA_DEPTH(TCM_DEPTH)
  )
RAM_CORE( 
	  .doa(AhbA_HRDATA), 
    .dia(AhbA_HWDATA), 
    .addra(Addr_A), 
    .cea(1'b1), //(AhbA_HSEL & (AhbA_HTRANS!=2'b00))|write_A
    .clka(AHB_CLK), 
    .wea(wCmd_mask_A),
	  .dob(AhbB_HRDATA), 
    .dib(AhbB_HWDATA), 
    .addrb(Addr_B), 
    .ceb(1'b1), //(AhbB_HSEL & (AhbB_HTRANS!=2'b00))|write_B
    .clkb(AHB_CLK), 
    .web(wCmd_mask_B)
);
//when exiting write mode, a bubble is NEEDED
//打拍，用于处理W->R切换的场景，产生流水线等待
  assign AhbA_HREADY_OUT=((!AhbA_HWRITE) & write_A)?1'b0:1'b1;
  assign AhbA_HRESP=1'b0;
  assign AhbB_HREADY_OUT=((!AhbB_HWRITE) & write_B)?1'b0:1'b1;
  assign AhbB_HRESP=1'b0;

  function [3:0]hsize2mask;
    input [2:0]hsize;
    input [1:0]haddr;
    reg [3:0]maskgen;
    begin
      case (hsize)
        3'b000:maskgen=4'h1;
        3'b001:maskgen=4'h3;
        3'b010:maskgen=4'hF;
        default:maskgen=4'hx;
      endcase
      hsize2mask = maskgen << haddr;
    end 
  endfunction
endmodule



module DPBRAM ( 
	doa, dia, addra, cea, clka, wea,
	dob, dib, addrb, ceb, clkb, web
);


	parameter DATA_WIDTH = 32; 
	parameter ADDR_WIDTH = 10;
	parameter DATA_DEPTH = 1024;
	parameter DEV_SERIES = "EG4";
	parameter RAM_STYLE = "32K";
	parameter INIT_FILE="NONE";

	localparam REGMODE_A    = "NOREG";
	localparam REGMODE_B    = "NOREG";
	localparam WRITEMODE_A  = "NORMAL";
	localparam WRITEMODE_B  = "NORMAL";

	output [DATA_WIDTH-1:0] doa;
	output [DATA_WIDTH-1:0] dob;
	input  [DATA_WIDTH-1:0] dia;
	input  [DATA_WIDTH-1:0] dib;
	input  [ADDR_WIDTH-1:0] addra;
	input  [ADDR_WIDTH-1:0] addrb;
	input  [3:0] wea;
	input  [3:0] web;
	input  cea;
	input  ceb;
	input  clka;
	input  clkb;


generate
	case(DEV_SERIES)
		"AL3":
		begin : BRAM_AL3
			AL_LOGIC_BRAM #( 
				.DATA_WIDTH_A(DATA_WIDTH),
				.DATA_WIDTH_B(DATA_WIDTH),
				.ADDR_WIDTH_A(ADDR_WIDTH),
				.ADDR_WIDTH_B(ADDR_WIDTH),
				.DATA_DEPTH_A(DATA_DEPTH),
				.DATA_DEPTH_B(DATA_DEPTH),
				.BYTE_ENABLE(8),
				.BYTE_A(4),
				.BYTE_B(4),
				.MODE("DP"),
				.REGMODE_A(REGMODE_A),
				.REGMODE_B(REGMODE_B),
				.WRITEMODE_A(WRITEMODE_A),
				.WRITEMODE_B(WRITEMODE_B),
				.RESETMODE("SYNC"),
				.IMPLEMENT("9K"),
				.INIT_FILE(INIT_FILE),
				.FILL_ALL("NONE"))
			inst(
				.dia(dia),
				.dib(dib),
				.addra(addra),
				.addrb(addrb),
				.cea(cea),
				.ceb(ceb),
				.ocea(1'b0),
				.oceb(1'b0),
				.clka(clka),
				.clkb(clkb),
				.wea(1'b0),
				.bea(wea),
				.web(1'b0),
				.beb(web),
				.rsta(1'b0),
				.rstb(1'b0),
				.doa(doa),
				.dob(dob));
		end
		"EG4":
		begin : BRAM_EG4
			EG_LOGIC_BRAM #( 
				.DATA_WIDTH_A(DATA_WIDTH),
				.DATA_WIDTH_B(DATA_WIDTH),
				.ADDR_WIDTH_A(ADDR_WIDTH),
				.ADDR_WIDTH_B(ADDR_WIDTH),
				.DATA_DEPTH_A(DATA_DEPTH),
				.DATA_DEPTH_B(DATA_DEPTH),
				.BYTE_ENABLE(8),
				.BYTE_A(4),
				.BYTE_B(4),
				.MODE("DP"),
				.REGMODE_A(REGMODE_A),
				.REGMODE_B(REGMODE_B),
				.WRITEMODE_A(WRITEMODE_A),
				.WRITEMODE_B(WRITEMODE_B),
				.RESETMODE("SYNC"),
				.IMPLEMENT(RAM_STYLE),
				.INIT_FILE(INIT_FILE),
				.FILL_ALL("NONE"))
			inst(
				.dia(dia),
				.dib(dib),
				.addra(addra),
				.addrb(addrb),
				.cea(cea),
				.ceb(ceb),
				.ocea(1'b0),
				.oceb(1'b0),
				.clka(clka),
				.clkb(clkb),
				.wea(1'b0),
				.bea(wea),
				.web(1'b0),
				.beb(web),
				.rsta(1'b0),
				.rstb(1'b0),
				.doa(doa),
				.dob(dob));
		end
		"EF2":
			if(RAM_STYLE=="256K")
			begin :BRAM256K_EF2
				EF2_PHY_BRAM256K #(
				.MODE("DP"),
				.REGMODE_A(REGMODE_A),
				.REGMODE_B(REGMODE_B),
				.WRITEMODE_A(WRITEMODE_A),
				.WRITEMODE_B(WRITEMODE_B),
				.INIT_FILE(INIT_FILE))
				inst(
					.dia(dia),
					.dib(dib),
					.cea(cea),
					.ceb(ceb),
					.ocea(1'b0),
					.oceb(1'b0),
					.clka(clka),
					.clkb(clkb),
					.wea((wea!=0)),
					.web((web!=0)),
					.rsta(1'b0),
					.rstb(1'b0),
					.addra(addra),
					.addrb(addrb),
					.weabyte(wea),
					.webbyte(web),
					.doa(doa),
					.dob(dob));
			end
			else
			begin : BRAM_EF2
				EF2_LOGIC_BRAM #( 
					.DATA_WIDTH_A(DATA_WIDTH),
					.DATA_WIDTH_B(DATA_WIDTH),
					.ADDR_WIDTH_A(ADDR_WIDTH),
					.ADDR_WIDTH_B(ADDR_WIDTH),
					.DATA_DEPTH_A(DATA_DEPTH),
					.DATA_DEPTH_B(DATA_DEPTH),
					.BYTE_ENABLE(8),
					.BYTE_A(4),
					.BYTE_B(4),
					.MODE("DP"),
					.REGMODE_A(REGMODE_A),
					.REGMODE_B(REGMODE_B),
					.WRITEMODE_A(WRITEMODE_A),
					.WRITEMODE_B(WRITEMODE_B),
					.RESETMODE("SYNC"),
					.IMPLEMENT(RAM_STYLE),
					.INIT_FILE(INIT_FILE),
					.FILL_ALL("NONE"))
				inst(
					.dia(dia),
					.dib(dib),
					.addra(addra),
					.addrb(addrb),
					.cea(cea),
					.ceb(ceb),
					.ocea(1'b0),
					.oceb(1'b0),
					.clka(clka),
					.clkb(clkb),
					.wea(1'b0),
					.bea(wea),
					.web(1'b0),
					.beb(web),
					.rsta(1'b0),
					.rstb(1'b0),
					.doa(doa),
					.dob(dob));
			end
		"EF3":
		begin : BRAM_EF3
			EF3_LOGIC_BRAM #( .DATA_WIDTH_A(DATA_WIDTH),
				.DATA_WIDTH_B(DATA_WIDTH),
				.ADDR_WIDTH_A(ADDR_WIDTH),
				.ADDR_WIDTH_B(ADDR_WIDTH),
				.DATA_DEPTH_A(DATA_DEPTH),
				.DATA_DEPTH_B(DATA_DEPTH),
				.BYTE_ENABLE(8),
				.BYTE_A(4),
				.BYTE_B(4),
				.MODE("DP"),
				.REGMODE_A(REGMODE_A),
				.REGMODE_B(REGMODE_B),
				.WRITEMODE_A(WRITEMODE_A),
				.WRITEMODE_B(WRITEMODE_B),
				.RESETMODE("SYNC"),
				.IMPLEMENT(RAM_STYLE),
				.INIT_FILE(INIT_FILE),
				.FILL_ALL("NONE"))
			inst(
				.dia(dia),
				.dib(dib),
				.addra(addra),
				.addrb(addrb),
				.cea(cea),
				.ceb(ceb),
				.ocea(1'b0),
				.oceb(1'b0),
				.clka(clka),
				.clkb(clkb),
				.wea(1'b0),
				.bea(wea),
				.web(1'b0),
				.beb(web),
				.rsta(1'b0),
				.rstb(1'b0),
				.doa(doa),
				.dob(dob));
		end
		"PH1":
		begin
			PH1_LOGIC_ERAM #( .DATA_WIDTH_A(DATA_WIDTH),
					.ADDR_WIDTH_A(ADDR_WIDTH),
					.DATA_DEPTH_A(DATA_DEPTH),
					.DATA_WIDTH_B(DATA_WIDTH),
					.ADDR_WIDTH_B(ADDR_WIDTH),
					.DATA_DEPTH_B(DATA_DEPTH),
					.BYTE_ENABLE(8),
					.BYTE_A(4),
					.BYTE_B(4),
					.MODE("DP"),
					.REGMODE_A(REGMODE_A),
					.REGMODE_B(REGMODE_B),
					.WRITEMODE_A(WRITEMODE_A),
					.WRITEMODE_B(WRITEMODE_B),
					.IMPLEMENT(RAM_STYLE),
					.ECC_ENCODE("DISABLE"),
					.ECC_DECODE("DISABLE"),
					.CLKMODE("ASYNC"),
					.SSROVERCE("DISABLE"),
					.OREGSET_A("SET"),
					.OREGSET_B("SET"),
					.RESETMODE_A("ASYNC"),
					.RESETMODE_B("ASYNC"),
					.ASYNC_RESET_RELEASE_A("SYNC"),
					.ASYNC_RESET_RELEASE_B("SYNC"),
					.INIT_FILE(INIT_FILE),
					.FILL_ALL("NONE"))
				inst(
					.dia(dia),
					.dib(dib),
					.addra(addra),
					.addrb(addrb),
					.cea(cea),
					.ceb(ceb),
					.ocea(1'b0),
					.oceb(1'b0),
					.clka(clka),
					.clkb(clkb),
					.wea((wea!=0)),
					.bea((wea!=0)?wea:4'hf),
					.web((web!=0)),
					.beb((web!=0)?web:4'hf),
					.rsta(1'b0),
					.rstb(1'b0),
					.doa(doa),
					.ecc_sbiterr(open),
					.ecc_dbiterr(open),
					.dob(dob));
		end
		"SIM":
		begin : SIMULATION_RAM
			reg doa_reg,dob_reg;
		  	reg [31:0]BRAM32[DATA_DEPTH-1:0];
			wire [31:0]loopback,loopbackb,writesig,writesigb;
			integer i;
			initial
			begin 
				//for(i=0;i<DATA_DEPTH;i=i+1)
				//    BRAM32[i]<=0;
				$readmemh(INIT_FILE,BRAM32);
				#5 doa_reg<=0;dob_reg<=0;
			end
			assign loopback=BRAM32[addra];
			assign loopbackb=BRAM32[addrb]; 
			assign writesig[7:0]  =wea[0]?dia[7:0]:loopback[7:0];
			assign writesig[15:8] =wea[1]?dia[15:8]:loopback[15:8];
			assign writesig[23:16]=wea[2]?dia[23:16]:loopback[23:16];
			assign writesig[31:24]=wea[3]?dia[31:24]:loopback[31:24];

			assign writesigb[7:0]  =web[0]?dib[7:0]:loopbackb[7:0];
			assign writesigb[15:8] =web[1]?dib[15:8]:loopbackb[15:8];
			assign writesigb[23:16]=web[2]?dib[23:16]:loopbackb[23:16];
			assign writesigb[31:24]=web[3]?dib[31:24]:loopbackb[31:24];

			always@(posedge clka)
			begin
				if(wea!=0)
				begin
					BRAM32[addra]<=writesig;
				end
				doa_reg<=BRAM32[addra];
			end
			always@(posedge clkb)
			begin
				if(web!=0)
				begin
					BRAM32[addrb]<=writesigb;
				end
				dob_reg<=BRAM32[addrb];
			end
			assign doa=doa_reg;
			assign dob=dob_reg;
		end
	endcase
endgenerate
endmodule


module AHB_APB_sync
#(parameter APB_ADDRWID=21)//
(
  // --------------------------
  // Input pins: AHB signals //
  // --------------------------
  input AHB_RST_i,
  input AHB_CLK,

  input        AHB2APB_HSEL,
  input [31:0] AHB2APB_HADDR,
  input        AHB2APB_HWRITE,
  input [1:0]  AHB2APB_HTRANS,
  input [2:0]  AHB2APB_HSIZE,
  input [2:0]  AHB2APB_HBURST,
  input [31:0] AHB2APB_HWDATA,
  input        AHB2APB_HREADY_IN,
  output       AHB2APB_HREADY_OUT,
  output reg   AHB2APB_HRESP,
  output reg [31:0] AHB2APB_HRDATA,
  //APB
  output reg [APB_ADDRWID-1:0] AHB2APB_PADDR,
  output reg        AHB2APB_PSEL,
  output reg        AHB2APB_PENABLE,
  input             AHB2APB_PREADY,
  output reg        AHB2APB_PWRITE,
  output reg [31:0] AHB2APB_PWDATA,
  input      [31:0] AHB2APB_PRDATA,
  input             AHB2APB_PSLVERR

);
localparam IDLE   = 2'b00;
localparam SETUP  = 2'b01;
localparam ACCESS = 2'b10;
reg [1:0]state,state_next;

always @(posedge AHB_CLK) 
begin
  AHB2APB_PWDATA<=AHB2APB_HWDATA;
  AHB2APB_HRDATA<=AHB2APB_PRDATA;
  AHB2APB_HRESP<=AHB2APB_PSLVERR;
end
always @(posedge AHB_CLK) //PADDR
    if(state_next==SETUP)
      AHB2APB_PADDR<=AHB2APB_HADDR[APB_ADDRWID-1:0];
    else
      AHB2APB_PADDR<=AHB2APB_PADDR;

always @(posedge AHB_CLK or posedge AHB_RST_i) //PWRITE
  if (AHB_RST_i) 
    AHB2APB_PWRITE<=0;
  else 
    case (state_next)
      SETUP: 
      begin 
        AHB2APB_PWRITE<=AHB2APB_HWRITE;
      end
      ACCESS:
        AHB2APB_PWRITE<=AHB2APB_PWRITE;
      default:
        AHB2APB_PWRITE<=1'b0;
    endcase

always @(posedge AHB_CLK or posedge AHB_RST_i) //PENABLE
  if (AHB_RST_i) 
    AHB2APB_PENABLE<=0;
  else 
    AHB2APB_PENABLE<=(state_next==ACCESS);
  
always @(posedge AHB_CLK or posedge AHB_RST_i) //PSEL
  if (AHB_RST_i) 
    AHB2APB_PSEL<=0;
  else 
    AHB2APB_PSEL<=(state_next!=IDLE);

always @(*)//FSM State Condition
  case (state)
  IDLE: 
    if(AHB2APB_HSEL & AHB2APB_HTRANS!=2'b00)
      state_next=SETUP;
    else 
      state_next=IDLE;
  SETUP: 
    state_next=ACCESS;
  ACCESS:
    if(!AHB2APB_PREADY)
      state_next=ACCESS;
    else 
      state_next=IDLE;
  default: state_next=IDLE;
  endcase

always @(posedge AHB_CLK or posedge AHB_RST_i) //FSM Transfer blk
  if (AHB_RST_i) 
    state<=IDLE;
  else
    state<=state_next;

assign AHB2APB_HREADY_OUT=(state==IDLE);
endmodule


module APB_DecMux8
#(
  parameter DEV0ENABLE=1'b1,
            DEV0BASE  =32'h1000_0000,
            DEV0RANGE =20,
            DEV1ENABLE=1'b1,
            DEV1BASE  =32'h1000_0000,
            DEV1RANGE =20,
            DEV2ENABLE=1'b1,
            DEV2BASE  =32'h1000_0000,
            DEV2RANGE =4,
            DEV3ENABLE=1'b1,
            DEV3BASE  =32'h1000_0000,
            DEV3RANGE =4,
            DEV4ENABLE=1'b1,
            DEV4BASE  =32'h1000_0000,
            DEV4RANGE =4,
            DEV5ENABLE=1'b1,
            DEV5BASE  =32'h1000_0000,
            DEV5RANGE =4,
            DEV6ENABLE=1'b1,
            DEV6BASE  =32'h1000_0000,
            DEV6RANGE =4,
            DEV7ENABLE=1'b1,
            DEV7BASE  =32'h1000_0000,
            DEV7RANGE =4
)
(
  // -------------
  // Input pins //
  // -------------
  // Clock and reset
  input [31:0] PADDR,
  // Slave datas
  input [31:0] PRDATAx0,
  input [31:0] PRDATAx1,
  input [31:0] PRDATAx2,
  input [31:0] PRDATAx3,
  input [31:0] PRDATAx4,
  input [31:0] PRDATAx5,
  input [31:0] PRDATAx6,
  input [31:0] PRDATAx7,
  // Control signals
  output PSELx0,
  output PSELx1,
  output PSELx2,
  output PSELx3,
  output PSELx4,
  output PSELx5,
  output PSELx6,
  output PSELx7,
  // Slave responses
  input PREADYx0,
  input PREADYx1,
  input PREADYx2,
  input PREADYx3,
  input PREADYx4,
  input PREADYx5,
  input PREADYx6,
  input PREADYx7,

  input PSLVERRORx0,
  input PSLVERRORx1,
  input PSLVERRORx2,
  input PSLVERRORx3,
  input PSLVERRORx4,
  input PSLVERRORx5,
  input PSLVERRORx6,
  input PSLVERRORx7, 
  // --------------
  // Output pins //
  // --------------
  output reg PREADY,
  output reg PSLVERROR,
  output reg [31:0] PRDATA
);
  assign PSELx0= ((PADDR & (32'hFFFF_FFFF << DEV0RANGE)) ==  (DEV0BASE & (32'hFFFF_FFFF << DEV0RANGE))) & DEV0ENABLE;
  assign PSELx1= ((PADDR & (32'hFFFF_FFFF << DEV1RANGE)) ==  (DEV1BASE & (32'hFFFF_FFFF << DEV1RANGE))) & DEV1ENABLE;
  assign PSELx2= ((PADDR & (32'hFFFF_FFFF << DEV2RANGE)) ==  (DEV2BASE & (32'hFFFF_FFFF << DEV2RANGE))) & DEV2ENABLE;
  assign PSELx3= ((PADDR & (32'hFFFF_FFFF << DEV3RANGE)) ==  (DEV3BASE & (32'hFFFF_FFFF << DEV3RANGE))) & DEV3ENABLE;
  assign PSELx4= ((PADDR & (32'hFFFF_FFFF << DEV4RANGE)) ==  (DEV4BASE & (32'hFFFF_FFFF << DEV4RANGE))) & DEV4ENABLE;
  assign PSELx5= ((PADDR & (32'hFFFF_FFFF << DEV5RANGE)) ==  (DEV5BASE & (32'hFFFF_FFFF << DEV5RANGE))) & DEV5ENABLE;
  assign PSELx6= ((PADDR & (32'hFFFF_FFFF << DEV6RANGE)) ==  (DEV6BASE & (32'hFFFF_FFFF << DEV6RANGE))) & DEV6ENABLE;
  assign PSELx7= ((PADDR & (32'hFFFF_FFFF << DEV7RANGE)) ==  (DEV7BASE & (32'hFFFF_FFFF << DEV7RANGE))) & DEV7ENABLE;
  wire PSELx8;
  assign PSELx8=0;
reg [3:0] slave_select;
  // Slave select register update
  always @ (*) 
    case({PSELx8,PSELx7, PSELx6, PSELx5, PSELx4, PSELx3, PSELx2, PSELx1, PSELx0})
      9'h001: slave_select <= 4'h0;
      9'h002: slave_select <= 4'h1;
      9'h004: slave_select <= 4'h2;
      9'h008: slave_select <= 4'h3;
      9'h010: slave_select <= 4'h4;
      9'h020: slave_select <= 4'h5;
      9'h040: slave_select <= 4'h6;
      9'h080: slave_select <= 4'h7;
      9'h100: slave_select <= 4'h8;
      // Default is for default slave
      default: slave_select <= 4'h8;
    endcase
  
  always @ (*) begin
    case(slave_select)
      4'h0: PRDATA = PRDATAx0;
      4'h1: PRDATA = PRDATAx1;
      4'h2: PRDATA = PRDATAx2;
      4'h3: PRDATA = PRDATAx3;
      4'h4: PRDATA = PRDATAx4;
      4'h5: PRDATA = PRDATAx5;
      4'h6: PRDATA = PRDATAx6;
      4'h7: PRDATA = PRDATAx7;
      default: PRDATA = 32'hx;
    endcase
  end
  
  // Response multiplexor
  always @ (*) begin
    case(slave_select)
      4'h0: begin
        PSLVERROR = PSLVERRORx0;
        PREADY = PREADYx0;
        end
      4'h1: begin
        PSLVERROR = PSLVERRORx1;
        PREADY = PREADYx1;
      end
      4'h2: begin
        PSLVERROR = PSLVERRORx2;
        PREADY = PREADYx2;
      end
      4'h3: begin
        PSLVERROR = PSLVERRORx3;
        PREADY = PREADYx3;
      end
      4'h4: begin
        PSLVERROR = PSLVERRORx4;
        PREADY = PREADYx4;
      end
      4'h5: begin
        PSLVERROR = PSLVERRORx5;
        PREADY = PREADYx5;
      end
      4'h6: begin
        PSLVERROR = PSLVERRORx6;
        PREADY = PREADYx6;
      end
    4'h7: begin
        PSLVERROR = PSLVERRORx7;
        PREADY = PREADYx7;
      end
      default: begin
        PSLVERROR = 1'b0;
        PREADY = 1'b1;
      end
    endcase
  end

endmodule



module APB_UART
#(
  parameter STYLE = "CONFIGURABLE",//FIXED
  parameter TXFIFO_EN=1'b1 ,
  parameter RXFIFO_EN=1'b0 , 
  parameter CLKRATE = 50000000 ,
  parameter BAUDRATE = 921600 ,
  parameter DATA_BIT   = 8,//数据位宽
  parameter PARITY_BIT = "None",//校验方式--"None","Odd","Even"
  parameter STOP_BIT   = "1"//停止位--"1","1.5","2"
)
( input      [3:0]    APB2UART_PADDR,
  input      [0:0]    APB2UART_PSEL,
  input               APB2UART_PENABLE,
  output reg          APB2UART_PREADY,
  input               APB2UART_PWRITE,
  input      [31:0]   APB2UART_PWDATA,
  output reg [31:0]   APB2UART_PRDATA,
  output reg          APB2UART_INTERRUPT,
  input               APB_CLK,
  input               APB_RST_i,
  input               RxD,
  output              TxD
  );
localparam UART_CFG = (STYLE=="CONFIGURABLE")?"IO":"FIXED";
wire read,write;
wire tx_write,tx_en;
wire rx_err,rx_vld,tx_rdy;
wire [4:0]int_gated;
wire txfifo_write,rxfifo_read;
wire txfifo_empty,txfifo_full,tx_about_empty;
wire rxfifo_empty,rxfifo_full;
wire tx_halfway,rx_halfway;
wire [7:0]rx_data,tx_data,rx_busdat;

//control signals
reg [13:0]freqdiff;
reg [1:0]fifoen,stopsel,parisel;
reg [5:0]int_ctrl;
reg [4:0]int_pending;

assign read  = APB2UART_PSEL & APB2UART_PENABLE & (!APB2UART_PWRITE);
assign write = APB2UART_PSEL & APB2UART_PENABLE & APB2UART_PWRITE;
assign tx_en = !txfifo_empty;//WIPtx_about_empty

assign txfifo_write = write & (APB2UART_PADDR[3:2]==0);
assign rxfifo_read  =  read & (APB2UART_PADDR[3:2]==0) ;
                      //read & (APB2UART_PADDR==0) & (!APB2UART_PREADY);//或者读空等待填充APB2UART_PSEL & (!APB2UART_PENABLE) & (!APB2UART_PWRITE)
uart_raw  #(
    .CLKRATE(CLKRATE),
    .BAUDRATE(BAUDRATE),
    .DATA_BIT(DATA_BIT),
    .PARITY_BIT(PARITY_BIT),
    .STOP_BIT(STOP_BIT),
    .CONFIG(UART_CFG))
UART_IP(
  .clk(APB_CLK),
  .rst(APB_RST_i),
  .rxd(RxD),
  .tx_data(tx_data),
  .tx_en(tx_en),
  .rx_data(rx_data),
  .rx_err(rx_err),
  .rx_vld(rx_vld),
  .tx_rdy(tx_rdy),
  .txd(TxD),
  .param_fdiv(freqdiff),
  .param_stop(stopsel),
  .param_pari(parisel)
);
SyncStreamFIFO UART_TxFIFO
(
    .clk(APB_CLK),
    .rst(APB_RST_i),
    .fifoen(fifoen[0] & TXFIFO_EN),
    .ren(tx_rdy),
    .wen(txfifo_write),
    .wdata(APB2UART_PWDATA[7:0]),
    .rdata(tx_data),
    .full(txfifo_full),
    .empty(txfifo_empty),
    //.about_empty(tx_about_empty),
    .halfway(tx_halfway)
);
SyncStreamFIFO UART_RxFIFO
(
    .clk(APB_CLK),
    .rst(APB_RST_i),
    .fifoen(fifoen[1] & RXFIFO_EN),
    .ren(rxfifo_read),
    .wen(rx_vld),
    .wdata(rx_data),
    .rdata(rx_busdat),
    .full(rxfifo_full),
    .empty(rxfifo_empty),
    .halfway(rx_halfway)
);
//pending events generation
assign int_gated={rx_err,(rx_halfway & RXFIFO_EN),(tx_halfway & TXFIFO_EN),rxfifo_full,txfifo_empty} 
                    & (int_ctrl[4:0] & {5{int_ctrl[5]}}) ;
always@(posedge APB_CLK or posedge APB_RST_i)
begin
  if(APB_RST_i)
    int_pending<=0;
  else 
    int_pending<=(int_pending | int_gated)  // 本行为同步边沿采样（？）
            &(~({5{(write && (APB2UART_PADDR[3:2]==2'b01))}} & APB2UART_PWDATA[4:0]));//本行是清除逻辑
end

always @(*) //read port
begin
  case (APB2UART_PADDR[3:2])
    2'b00:APB2UART_PRDATA={24'h000000,rx_busdat};
    2'b01:APB2UART_PRDATA={25'h0000000,!rxfifo_empty,!txfifo_full,int_pending};
    2'b10:APB2UART_PRDATA={int_ctrl[5],2'bxx,int_ctrl[4:0],4'hx,fifoen,parisel,stopsel,freqdiff};
    default:APB2UART_PRDATA=32'h00000000;
  endcase
end

always@(posedge APB_CLK or posedge APB_RST_i)//ctrl reg write
begin
  if(APB_RST_i)
    {int_ctrl[5],int_ctrl[4:0],fifoen,parisel,stopsel,freqdiff}<=(26'h03_00010 & {6'h3F,RXFIFO_EN,TXFIFO_EN,{18{UART_CFG=="IO"}}});
  else if(write & (APB2UART_PADDR[3:2]==2'b10))
    {int_ctrl[5],int_ctrl[4:0],fifoen,parisel,stopsel,freqdiff}<={APB2UART_PWDATA[31],APB2UART_PWDATA[28:24],(APB2UART_PWDATA[19:0] & {RXFIFO_EN,TXFIFO_EN,{18{UART_CFG=="IO"}}})};
  else 
    {int_ctrl[5],int_ctrl[4:0],fifoen,parisel,stopsel,freqdiff}<={int_ctrl[5],int_ctrl[4:0],({fifoen,parisel,stopsel,freqdiff} & {RXFIFO_EN,TXFIFO_EN,{18{UART_CFG=="IO"}}})};
end

always@(posedge APB_CLK or posedge APB_RST_i)//INT GEN
begin
    if(APB_RST_i)
    APB2UART_INTERRUPT<=1'b0;
  else 
    APB2UART_INTERRUPT<=!(int_pending==5'h00);
end

always@(posedge APB_CLK or posedge APB_RST_i)//READY GEN
begin
  if(APB_RST_i)
    APB2UART_PREADY<=1'b1;
  else if(APB2UART_PSEL & APB2UART_PENABLE & (APB2UART_PADDR[3:2]==0) & (!APB2UART_PWRITE))
    APB2UART_PREADY<=!rxfifo_empty;
  else if(APB2UART_PSEL & APB2UART_PENABLE & (APB2UART_PADDR[3:2]==0) & APB2UART_PWRITE)
    APB2UART_PREADY<=!txfifo_full;
  else 
    APB2UART_PREADY<=1'b1;
end

endmodule



module uart_raw
#(
	parameter CLKRATE = 50000000 ,
    parameter BAUDRATE = 921600 ,
    parameter DATA_BIT   = 8,//数据位宽
	parameter PARITY_BIT = "None",//校验方式--"None","Odd","Even"
	parameter STOP_BIT   = "1",//停止位--"1","1.5","2"
    parameter CONFIG     = "FIXED"//FIXED使用parameter配置，IO使用暴露接口进行参数配置
)
( input clk,rst,
  output txd,
  input [DATA_BIT-1:0]tx_data,
  input tx_en,
  output tx_rdy,
  output [DATA_BIT-1:0]rx_data,
  output rx_err,
  output rx_vld,
  input rxd,
  //Cfg interface
  input [13:0]param_fdiv,
  input [1:0]param_stop,
  input [1:0]param_pari
  
);
uart_tx#(
    .CLKRATE(CLKRATE),
    .BAUDRATE(BAUDRATE),
    .DATA_BIT(DATA_BIT),//数据位宽
	.PARITY_BIT(PARITY_BIT),//校验方式--"None","Odd","Even"
	.STOP_BIT(STOP_BIT),//停止位--"1","1.5","2"
    .CONFIG(CONFIG)
)TX_MODULE
(
    .clk(clk),
    .rst(rst),
    .tx_data(tx_data),
    .tx_en(tx_en),
    .tx_rdy(tx_rdy),
    .txd(txd),
	.param_fdiv(param_fdiv),
  	.param_stop(param_stop),
  	.param_pari(param_pari)
);
uart_rx_deglitched#(
    .CLKRATE(CLKRATE),
    .BAUDRATE(BAUDRATE),
    .DATA_BIT(DATA_BIT),//数据位宽
	.PARITY_BIT(PARITY_BIT),//校验方式--"None","Odd","Even"
	.STOP_BIT(STOP_BIT),//停止位--"1","1.5","2"
    .CONFIG(CONFIG)
)RX_MODULE
(
    .clk(clk),
    .rst(rst),
    .rx_data(rx_data),
    .rx_err(rx_err),
    .rx_vld(rx_vld),
    .rxd(rxd),
	.param_fdiv(param_fdiv),
  	.param_stop(param_stop),
  	.param_pari(param_pari)
);

endmodule


module uart_tx
#(
    parameter CLKRATE = 50000000 ,
    parameter BAUDRATE = 1000000 ,
    parameter DATA_BIT   = 8,//数据位宽
	parameter PARITY_BIT = "Odd",//校验方式--"None","Odd","Even"
	parameter STOP_BIT   = "2",//停止位--"1","1.5","2"
    parameter CONFIG     = "FIXED"//FIXED使用parameter配置，IO使用暴露接口进行参数配置
)
(
    input clk,rst,
    input [DATA_BIT-1:0]tx_data,
    input tx_en,
    output tx_rdy,
    output reg txd,
    input [13:0]param_fdiv,
    input [1:0]param_stop,
    input [1:0]param_pari
);

localparam CLKDIV = CLKRATE/BAUDRATE;//T/BAUD_RATE
localparam FDIV_WID=(CONFIG=="FIXED")?$clog2(CLKDIV):14;
wire [FDIV_WID-1:0]fdiv_coeff;
wire [1:0]parity_setting,stop_setting;
wire tx_parity;

assign fdiv_coeff=(CONFIG=="FIXED")?CLKDIV:param_fdiv;
assign parity_setting=
        (CONFIG=="FIXED")?
        (
            (PARITY_BIT=="None")?2'b00:
            ((PARITY_BIT=="Odd")?2'b11:2'b10)
        ):param_pari;
assign stop_setting=
        (CONFIG=="FIXED")?
        (
            (STOP_BIT=="1.5")?2'b10:
            ((PARITY_BIT=="2")?2'b11:2'b00)
        ):param_stop;

//Tx shifter chain
reg [DATA_BIT+2:0] tx_shifter;
reg [DATA_BIT+2:0] tx_valid_sft;
reg [2:0]tx_stop_sft;
wire tx_in_progress;
wire tx_valid;

//Tx Baud rate gen
reg [FDIV_WID-1:0]fdiv_cnt;
reg baud_flag;

assign tx_parity = (parity_setting==2'b11) ? (~^tx_data) : (^tx_data);
assign tx_valid = tx_valid_sft[0];
assign tx_in_progress = tx_valid_sft[0] | tx_stop_sft[0];

//Tx Baud rate gen
always @(posedge clk) 
begin
    if(baud_flag | rst)
        fdiv_cnt <= 0; 
    else
        fdiv_cnt <= fdiv_cnt + tx_in_progress;
end

always@(posedge clk or posedge rst)
begin
    if(rst)
        baud_flag<=0;
    else
        baud_flag<=(fdiv_cnt==( fdiv_coeff - 2));
end

//Tx Circuit
always @(posedge clk or posedge rst)
begin
    if(rst)
        txd<=1'b1;
    else if(tx_valid)
        txd<=tx_shifter[0];
    else 
        txd<=txd;
end

always @(posedge clk) //Data shifter
begin
    if(tx_valid & baud_flag)
        tx_shifter<=tx_shifter>>1;
    else if(tx_en & tx_rdy)
    begin
        if(parity_setting[1]) 
            tx_shifter <= {1'b1,tx_parity,tx_data,1'b0};//frame with parity
        else 
            tx_shifter <= {2'b1,tx_data,1'b0};//frame wo parity
    end
end

always @(posedge clk) //valid shifter
begin
    if(rst)
        tx_valid_sft<=0;
    else if(tx_valid & baud_flag)
        tx_valid_sft <= (tx_valid_sft >> 1);
    else if(tx_en & tx_rdy)
    begin
        if(parity_setting[1]) 
            tx_valid_sft <= {(DATA_BIT+3){1'b1}};
        else  
            tx_valid_sft <= {1'b0,{(DATA_BIT+2){1'b1}}};
    end
end

//deal with 1.5 stop bits
reg half_baud_flag;
always @(posedge clk) 
begin
    if( stop_setting == 2'b10 ) 
        half_baud_flag <= (fdiv_cnt[FDIV_WID-2:0]==(fdiv_coeff>>1) - 2);
    else 
        half_baud_flag<=1'b0;
end

always @(posedge clk) //stop bit shifter
begin
    if(rst)tx_stop_sft<=0;
    else if(tx_in_progress&(!tx_valid))
    begin
        if(half_baud_flag & stop_setting == 2'b10 )
        begin
            tx_stop_sft<=tx_stop_sft>>1;
        end
        else if(baud_flag) 
            tx_stop_sft<=tx_stop_sft>>1;
    end
    else if(tx_en & tx_rdy)
    begin
        case (stop_setting)
            2'b10: tx_stop_sft<=2'b11;//1.5bit
            2'b11:tx_stop_sft<=2'b01;//2bit
            default: tx_stop_sft<=2'b00;//1bit
        endcase
    end
end

//rdy generation
assign tx_rdy=!tx_in_progress;
endmodule


module uart_rx_deglitched
#(
    parameter CLKRATE = 50000000 ,
    parameter BAUDRATE = 115200 ,
    parameter DATA_BIT   = 8,//数据位宽
	parameter PARITY_BIT = "Odd",//校验方式--"None","Odd","Even"
	parameter STOP_BIT   = "2",//停止位--"1","1.5","2"
    parameter CONFIG     = "FIXED"//FIXED使用parameter配置，IO使用暴露接口进行参数配置
)(
	input rst,
	input clk,

	input rxd,
	output reg [DATA_BIT-1:0] rx_data,
	output reg rx_vld,
	output reg rx_err,
	input [13:0]param_fdiv,
    input [1:0]param_stop,
    input [1:0]param_pari
);

	localparam CLKDIV = CLKRATE/BAUDRATE/16;//T/BAUD_RATE
	localparam FDIV_WID=(CONFIG=="FIXED")?($clog2(CLKDIV)-4):10;
	localparam  				RX_IDLE    = 0,
                				RX_START   = 1,
                				RX_DATA    = 2,
		        				RX_PARITY  = 3,
								RX_STOP    = 4;

// ====================================================================
// Parameter/wire/reg
// =========================================cnt==0=========================
	reg							rxd_ff;
	reg 						rxd_ff2;
	reg							rxd_ff3;
	reg 						rxd_ff4;
	reg 						rxd_ff5;
	reg			[7:0] 			cnt_16;
	reg			[5:0]			deglitch_cnt;
	reg							deglitch_error;
	reg			[2:0]        	current_state;
	reg 		[2:0]			next_state;
	reg							stop_error;
	reg							parity_error;
	reg							rx_parity_bit_lcl;
	reg							rx_parity_bit_rcv;
	reg [FDIV_WID-1:0]fdiv_cnt;
	wire [FDIV_WID-1:0]fdiv_coeff;
	wire [1:0]parity_setting;	
	wire [1:0]stop_setting;
	wire 		[5:0] 			stop_tap_num;
	wire						rxd_16;
	wire clk_en;
	
	assign fdiv_coeff=((CONFIG=="FIXED")?CLKDIV:(param_fdiv>>4))-1;
	assign parity_setting=(CONFIG=="FIXED")?
			(   (PARITY_BIT=="None")?2'b00:
				((PARITY_BIT=="Odd")?2'b11:2'b10)
			):param_pari;	
	assign stop_setting=
        (CONFIG=="FIXED")?
        (
            (STOP_BIT=="1.5")?2'b10:
            ((PARITY_BIT=="2")?2'b11:2'b00)
        ):param_stop;
	assign stop_tap_num = 6'd1;
	//(stop_setting == 2'b11)?6'd28:((stop_setting == 2'b10)?6'd20:6'd12);
	assign clk_en=(fdiv_cnt==fdiv_coeff);
	always@(posedge clk or posedge rst)
	begin
		if(rst)
			fdiv_cnt<=0;
		else if(clk_en)
			fdiv_cnt<=0;
		else
			fdiv_cnt<=fdiv_cnt+1;
	end
// ====================================================================
// rxd_ff,rxd_ff2,rxd_ff3,rxd_ff4,rxd_ff5
// ====================================================================
always @(posedge clk or posedge rst) 
	begin
	if (rst)	
		begin	
		rxd_ff  <= 1'b0;
		rxd_ff2 <= 1'b0;
		rxd_ff3 <= 1'b0;
		rxd_ff4 <= 1'b0;
		rxd_ff5 <= 1'b0;
		end
	else 
		begin
		rxd_ff  <= rxd;
		rxd_ff2 <= rxd_ff;
		rxd_ff3 <= rxd_ff2;
		rxd_ff4 <= rxd_ff3;
		rxd_ff5 <= rxd_ff4;
		end 
	end 
	
// ====================================================================
// rxd_H2L
// ====================================================================
assign rxd_H2L = ((rxd_ff2+rxd_ff3+rxd_ff4)==2'h0)&&(rxd_ff5==1'b1);

// --------------------------------------------------------------------
// Rx state machine
// --------------------------------------------------------------------
// initial state, current_state<=next_state
always @(posedge clk or posedge rst) 
 	begin
 	if(rst)		
 		current_state <= RX_IDLE ;
 	else
 		current_state <= next_state ;
 	end

	always @ (*) 
	begin
		case(current_state)
			RX_IDLE:
			begin
				if(rxd_H2L)
				begin
					next_state = RX_START;
				end
				else
				begin
					next_state = RX_IDLE;
				end
			end
			RX_START:
			begin
				if(cnt_16 == 8'd16)
				begin
					next_state = RX_DATA;				
				end
				else
				begin
					next_state = RX_START;
				end
			end
			RX_DATA:
			begin
				if(cnt_16 == {DATA_BIT+4'd1,4'h0})//(DATA_BIT+1)*16
				begin
					if (parity_setting == 2'b00)
					begin
						next_state <= RX_STOP;
					end
					else
					begin
						next_state <= RX_PARITY;
					end				
				end
				else
				begin
					next_state = RX_DATA ;
				end			
			end
			RX_PARITY:
			begin
				if(cnt_16 == {DATA_BIT+4'd2,4'h0})//(DATA_BIT+2)*16
				begin
					next_state = RX_STOP;
				end
				else
				begin
					next_state = RX_PARITY;				
				end
			end				
			RX_STOP:
			begin
				if(cnt_16 == {DATA_BIT+4'd2,4'h0}+stop_tap_num)//(DATA_BIT+2)*16+stop_tap_num
				begin
					if(rxd_H2L)
					begin
						next_state <= RX_START;					
					end
					else
					begin
						next_state <= RX_IDLE;					
					end
				end
				else if(cnt_16 > {DATA_BIT+4'd2,4'h0}+stop_tap_num[5:1])//(DATA_BIT+2)*16 + stop_tap_num/2
				begin
					if(rxd_H2L)
					begin
						next_state <= RX_START;					
					end
					else
					begin
						next_state <= RX_STOP;					
					end				
				end
				else
				begin
					next_state <= RX_STOP;				
				end
			end
			default:
			begin
				next_state = RX_IDLE;			
			end
		endcase
	end

	//cnt_16
	always @(posedge clk or posedge rst) 
	begin
		if(rst)
		begin
			cnt_16 <= 8'd0;		
		end
		else
		begin
			case(next_state)
				RX_IDLE:
				begin
					cnt_16 <= 8'd0;			
				end
				RX_START,
				RX_DATA,
				RX_PARITY,
				RX_STOP:
				begin
					if(clk_en)
					begin
						cnt_16 <= cnt_16 + 8'd1;
					end		
				end		
			endcase
		end
	end
	
	//rx_data, rx_parity_bit_lcl, rx_parity_bit_rcv
	always @(posedge clk or posedge rst) 
	begin
		if(rst)
		begin
			rx_data <= 0;
			rx_parity_bit_lcl <= 1'b0;
			rx_parity_bit_rcv <= 1'b0;
		end
		else
		begin
			case(next_state)
				RX_START:
				begin
					rx_parity_bit_lcl <= 1'b0;
				end
				RX_DATA:
				begin
					if(clk_en)
					begin
						if(cnt_16[3:0]==4'hF)
						begin
							rx_data <= {rxd_16,rx_data[DATA_BIT-1:1]};
							rx_parity_bit_lcl <= rx_parity_bit_lcl^rxd_16;//XOR gates output what is equivalent to a check bit that creates an even parity
						end
					end
				end
				RX_PARITY:
				begin
					if(clk_en)
					begin
						if(cnt_16[3:0]==4'hF)
						begin
							rx_parity_bit_rcv <= rxd_16;
						end
					end				
				end
			endcase
		end
	end
	
	//deglitch_cnt
	always @(posedge clk or posedge rst) 
	begin
		if(rst)
		begin
			deglitch_cnt <= 6'd0;
		end
		else if(rx_vld)
		begin
			deglitch_cnt <= 6'd0;		
		end
		else if(clk_en)
		begin
			case(current_state)
				RX_START,
				RX_DATA,
				RX_PARITY:
				begin
					if(cnt_16[3:0]==4'hF)
					begin
						deglitch_cnt <= 6'd0;				
					end
					else
					begin
						deglitch_cnt <= deglitch_cnt+rxd_ff4;//rxd_ff4 还是 xd_ff3？		
					end				
				end
				RX_STOP:
				begin
					deglitch_cnt <= deglitch_cnt+rxd_ff4;			
				end
			endcase
		end
	end
	
	//rxd_16
	assign rxd_16 = (deglitch_cnt<6'd8)?1'b0:1'b1;

	//deglitch_error
	always @(posedge clk or posedge rst) 
	begin
		if (rst)
		begin		
			deglitch_error <= 1'b0;
		end
		else if(rx_vld)
		begin
			deglitch_error <= 1'b0;		
		end
		else if((current_state == RX_START)
			  ||(current_state == RX_DATA)
			  ||(current_state == RX_PARITY))//deglitch_error doesn't happen in RX_STOP state
		begin
			if(cnt_16[3:0]==4'd0)
			begin
				if(deglitch_cnt==6'd8)
				begin
					deglitch_error <= 1'b1;
				end
			end		
		end
	end

	//parity_error
	always @(posedge clk or posedge rst) 
	begin
		if(rst)
		begin
			parity_error <= 1'b0;
		end
		else if(rx_vld)
		begin
			parity_error <= 1'b0;		
		end
		else if(next_state == RX_STOP)
		begin
			if(parity_setting == 2'b10)
			begin
				parity_error <= rx_parity_bit_lcl!=rx_parity_bit_rcv;
			end
			else
			begin
				parity_error <= rx_parity_bit_lcl==rx_parity_bit_rcv;
			end
		end
	end
	
	//stop_error
	always @(posedge clk or posedge rst) 
	begin
		if(rst)
		begin
			stop_error <= 1'b0;
		end
		else if(rx_vld)
		begin
			stop_error <= 1'b0;		
		end
		else if(current_state == RX_STOP)
		begin
			if(cnt_16 < {DATA_BIT+4'd2,4'h0}+stop_tap_num[5:1])//(DATA_BIT+2)*16 + stop_tap_num/2
			begin
				if(rxd_H2L)
				begin
					stop_error <= 1'b1;					
				end
			end
			else if(deglitch_cnt==0)
			begin
				stop_error <= 1'b1;			
			end
		end
	end
	
	//rx_vld, rx_err
	always @(posedge clk or posedge rst) 
	begin
		if(rst)
		begin
			rx_vld <= 1'b0;
			rx_err <= 1'b0;
		end
		else if((current_state == RX_STOP)
			  &&((next_state == RX_IDLE)
			   ||(next_state == RX_START)))
		begin
			rx_vld <= 1'b1;
			rx_err <= deglitch_error | parity_error | stop_error;
		end
		else
		begin
			rx_vld <= 1'b0;
			rx_err <= 1'b0;			
		end		
	end

endmodule


module SyncStreamFIFO
#(
    parameter   DWID = 8,
                DDEPTH = 64
)
(
    input clk,rst,
    input ren,wen,
    input fifoen,
    input [DWID-1:0]wdata,
    output[DWID-1:0]rdata,
    output reg full,empty,halfway,
    output about_empty,about_full
);
localparam CNTWID = $clog2(DDEPTH);

reg [CNTWID-1:0] wptr;
reg [CNTWID-1:0] rptr;
wire [CNTWID-1:0] wptr_next,rptr_next;
reg [DWID-1:0]memory[DDEPTH-1:0];
reg [7:0]rsav;
wire full_cmp,empty_cmp;
wire wen_internal,ren_internal;

assign wen_internal=wen&fifoen;
assign ren_internal=ren&fifoen;
assign full_cmp=(rptr_next=={!wptr_next[CNTWID-1],wptr_next[CNTWID-2:0]});
assign empty_cmp=(wptr_next==rptr_next);
assign wptr_next=(full)?wptr:wptr+wen_internal;
assign rptr_next=(empty)?rptr:rptr+ren_internal;
assign about_empty=(fifoen)?empty_cmp:empty;
assign rdata=(fifoen)?memory[rptr]:rsav;
always@(posedge clk or posedge rst)//PTRs
begin
    if(rst) 
    begin
        wptr<=0;
        rptr<=0;
    end
    else
    begin
        if(wen_internal) 
            wptr<=wptr_next;
        else 
            wptr<=wptr;
        if(ren_internal)
            rptr<=rptr_next;
        else
            rptr<=rptr;
    end
end

always@(posedge clk)//data
begin
    if(fifoen)
    begin
        if(wen) memory[wptr]<=wdata;
    end
    else if(wen&empty)rsav<=wdata;
end

always@(posedge clk or posedge rst)//Full & Empty
begin
    if(rst) 
    begin
        full<=0;
        empty<=1'b1;
    end
    else if(fifoen)
    begin
        //if(wen) 
            full<=full_cmp;
        //if(ren)
            empty<=empty_cmp;
    end
    else
    begin
        if(wen & !full)
        begin
            full<=1'b1;
            empty<=1'b0;
        end
        else if(ren & !empty)
        begin
            full<=1'b0;
            empty<=1'b1;
        end
        else
            begin
                full<=full;
                empty<=empty;
            end
    end
end

always@(posedge clk or posedge rst)//Halfway
begin
    if(rst) 
        halfway<=0;
    else
        halfway<=((wptr[CNTWID-1] ^ rptr[CNTWID-1]) & (!halfway));
end

endmodule


module SyncStreamFIFO
#(
    parameter   DWID = 8,
                DDEPTH = 64
)
(
    input clk,rst,
    input ren,wen,
    input fifoen,
    input [DWID-1:0]wdata,
    output[DWID-1:0]rdata,
    output reg full,empty,halfway,
    output about_empty,about_full
);
localparam CNTWID = $clog2(DDEPTH);

reg [CNTWID-1:0] wptr;
reg [CNTWID-1:0] rptr;
wire [CNTWID-1:0] wptr_next,rptr_next;
reg [DWID-1:0]memory[DDEPTH-1:0];
reg [7:0]rsav;
wire full_cmp,empty_cmp;
wire wen_internal,ren_internal;

assign wen_internal=wen&fifoen;
assign ren_internal=ren&fifoen;
assign full_cmp=(rptr_next=={!wptr_next[CNTWID-1],wptr_next[CNTWID-2:0]});
assign empty_cmp=(wptr_next==rptr_next);
assign wptr_next=(full)?wptr:wptr+wen_internal;
assign rptr_next=(empty)?rptr:rptr+ren_internal;
assign about_empty=(fifoen)?empty_cmp:empty;
assign rdata=(fifoen)?memory[rptr]:rsav;
always@(posedge clk or posedge rst)//PTRs
begin
    if(rst) 
    begin
        wptr<=0;
        rptr<=0;
    end
    else
    begin
        if(wen_internal) 
            wptr<=wptr_next;
        else 
            wptr<=wptr;
        if(ren_internal)
            rptr<=rptr_next;
        else
            rptr<=rptr;
    end
end

always@(posedge clk)//data
begin
    if(fifoen)
    begin
        if(wen) memory[wptr]<=wdata;
    end
    else if(wen&empty)rsav<=wdata;
end

always@(posedge clk or posedge rst)//Full & Empty
begin
    if(rst) 
    begin
        full<=0;
        empty<=1'b1;
    end
    else if(fifoen)
    begin
        //if(wen) 
            full<=full_cmp;
        //if(ren)
            empty<=empty_cmp;
    end
    else
    begin
        if(wen & !full)
        begin
            full<=1'b1;
            empty<=1'b0;
        end
        else if(ren & !empty)
        begin
            full<=1'b0;
            empty<=1'b1;
        end
        else
            begin
                full<=full;
                empty<=empty;
            end
    end
end

always@(posedge clk or posedge rst)//Halfway
begin
    if(rst) 
        halfway<=0;
    else
        halfway<=((wptr[CNTWID-1] ^ rptr[CNTWID-1]) & (!halfway));
end

endmodule



`define GPIO_NONE 3'b000
`define GPIO_PEDGE 3'b100
`define GPIO_NEDGE 3'b101
`define GPIO_HIGH 3'b110
`define GPIO_LOW 3'b111
module APB_GPIO
#(parameter GPIO_NUM=32,
  GPIO_INTENABLE = 1'b0 ,
  GPIO_INTCFG={GPIO_NUM{`GPIO_NONE}}
)
(
  input      [5:0]    PADDR,
  input      [0:0]    PSEL,
  input               PENABLE,
  output              PREADY,
  input               PWRITE,
  input      [31:0]   PWDATA,
  output reg [31:0]   PRDATA,
  output              PSLVERROR,
  output              io_interrupt ,
  input      [GPIO_NUM-1:0]   io_gpio_read,
  output     [GPIO_NUM-1:0]   io_gpio_write,
  output     [GPIO_NUM-1:0]   io_gpio_writeEnable,
  output     [GPIO_NUM-1:0]   io_value,
  input               clk,
  input               rst
);
  integer i;genvar j;
  wire       [GPIO_NUM-1:0]   io_gpio_read_buffercc_io_dataOut;
  wire                ctrl_askWrite;
  wire                ctrl_askRead;
  wire                ctrl_doWrite;
  wire                ctrl_doRead;
  reg        [GPIO_NUM-1:0]   io_gpio_write_driver;
  reg        [GPIO_NUM-1:0]   io_gpio_writeEnable_driver;
  reg        [GPIO_NUM-1:0]   gpio_IntMask=0;
  reg        [GPIO_NUM-1:0]   int_edgetest;  
  reg        [GPIO_NUM-1:0]   int_trig;  
  GPIO_Buf #(.WIDTH(GPIO_NUM)) io_gpio_read_buffercc (
    .io_dataIn                (io_gpio_read                      ), //i
    .io_dataOut               (io_gpio_read_buffercc_io_dataOut  ), //o
    .clk               (clk                        ), //i
    .rst               (rst             )  //i
  );

  assign io_value = io_gpio_read_buffercc_io_dataOut;
  assign PREADY = 1'b1;
  always @(*) begin
    PRDATA = 32'h0;
    case(PADDR>>2)
      4'b0000 : begin
        PRDATA[GPIO_NUM-1 : 0] = io_value;
      end
      4'b0001 : begin
        PRDATA[GPIO_NUM-1 : 0] = io_gpio_write_driver;
      end
      4'b0010 : begin
        PRDATA[GPIO_NUM-1 : 0] = io_gpio_writeEnable_driver;
      end
      4'b0011 : begin
        PRDATA[GPIO_NUM-1 : 0] = gpio_IntMask;
      end
      default : begin
      end
    endcase
  end
  assign PSLVERROR = 1'b0;
  assign ctrl_askWrite = ((PSEL[0] && PENABLE) && PWRITE);
  assign ctrl_askRead = ((PSEL[0] && PENABLE) && (! PWRITE));
  assign ctrl_doWrite = (((PSEL[0] && PENABLE) && PREADY) && PWRITE);
  assign ctrl_doRead = (((PSEL[0] && PENABLE) && PREADY) && (! PWRITE));
  assign io_gpio_write = io_gpio_write_driver;
  assign io_gpio_writeEnable = io_gpio_writeEnable_driver;
  
  always @(posedge clk or posedge rst) begin
    if(rst) begin
      io_gpio_writeEnable_driver <= 32'h0;
    end 
    else begin
      case(PADDR>>2)
        4'b0010 : 
        begin
          if(ctrl_doWrite) 
          begin
            io_gpio_writeEnable_driver <= PWDATA[GPIO_NUM-1 : 0];
          end
        end
        4'b0001 : 
        begin
          if(ctrl_doWrite) 
          begin
            io_gpio_write_driver <= PWDATA[GPIO_NUM-1 : 0];
          end
        end
        4'b0011 : 
        begin
          if(ctrl_doWrite & GPIO_INTENABLE) 
          begin
            for (i=0;i<GPIO_NUM;i=i+1)
              if(GPIO_INTCFG[(i+1)*3-1])gpio_IntMask[i] <= PWDATA[i];
            //gpio_IntMask <= PWDATA;
          end
        end
        default : begin
        end
      endcase
    end
  end
assign io_interrupt=((int_trig & gpio_IntMask)!= 0) & GPIO_INTENABLE;
generate //GPIO Int type block
    for (j=0 ;j<GPIO_NUM ;j=j+1 ) //if(GPIO_INTENABLE)
    begin:GPIOINT_BLK
      always@(posedge clk)
        if(GPIO_INTENABLE)
        begin
            if(GPIO_INTCFG[(j+1)*3-1:(j+1)*3-2]==2'b10)
              int_edgetest[j]<=io_value[j];
            case(GPIO_INTCFG[(j+1)*3-1:(j*3)])
              `GPIO_PEDGE:int_trig[j]<=({int_edgetest[j],io_value[j]}==2'b01);
              `GPIO_NEDGE:int_trig[j]<=({int_edgetest[j],io_value[j]}==2'b10);
              `GPIO_HIGH :int_trig[j]<=io_value[j];
              `GPIO_LOW  :int_trig[j]<=!io_value[j];
              default:int_trig[j]<=1'b0;
            endcase
        end
    end
endgenerate

endmodule
module GPIO_Buf #(parameter WIDTH=32)(
  input      [WIDTH-1:0]   io_dataIn,
  output     [WIDTH-1:0]   io_dataOut,
  input               clk,
  input               rst
);
  (* async_reg = "true" *) reg        [WIDTH-1:0]   buffers_0;
  (* async_reg = "true" *) reg        [WIDTH-1:0]   buffers_1;

  assign io_dataOut = buffers_1;
  always @(posedge clk) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end


endmodule



module APB_SPI
#(
    parameter SPI_MODE=5,//if programmable is needed, use >4
    DEVNUM=1
)
(
// APB SLAVE PORT INTERFACE 
    input      [4:0]    APB2SPI_PADDR,
    input      [0:0]    APB2SPI_PSEL,
    input               APB2SPI_PENABLE,
    output              APB2SPI_PREADY,
    input               APB2SPI_PWRITE,
    input      [31:0]   APB2SPI_PWDATA,
    output     [31:0]   APB2SPI_PRDATA,
    output              APB2SPI_INTERRUPT,//Interrupt
    input               APB_CLK,
    input               APB_RST_i,

// SPI signals
    input               MISO_i,
    output              MOSI_o,
    output              SCK_O,
    output [DEVNUM-1:0] SS_O
);

tiny_spi_apb #(
    .SPI_MODE(SPI_MODE),
    .DEVSEL_NUM(DEVNUM)
   ) 
SPI_IP(
    .clk_i(APB_CLK),         // clock
    .rst_i(APB_RST_i),// reset (asynchronous active high)
    .cyc_i(APB2SPI_PSEL),         // cycle
    .stb_i(APB2SPI_PENABLE),         // strobe
    .adr_i(APB2SPI_PADDR[4:2]),         // address
    .we_i(APB2SPI_PWRITE),          // write enable
    .dat_i(APB2SPI_PWDATA),         // data input
    .dat_o(APB2SPI_PRDATA),         // data output
    .ack_o(APB2SPI_PREADY),         // normal bus termination
    .int_o(APB2SPI_INTERRUPT),        // interrupt output
    //SPI INTERFACE
    .SCLK(SCK_O),         // serial clock output
    .MOSI(MOSI_o),        // MasterOut SlaveIN
    .MISO(MISO_i),         // MasterIn SlaveOut
    .MCS(SS_O)
);


endmodule



module tiny_spi_apb(
   // system
   input	  rst_i,
   input	  clk_i,
   // memory mapped
   input	  stb_i,
   input	  we_i,
   output [31:0]  dat_o,
   input [31:0]   dat_i,
   output	  int_o,
   input [2:0]	  adr_i,
   input 	  cyc_i, // comment out for avalon
   output 	  ack_o, // comment out for avalon

   // spi
   output	  MOSI,
   output	  SCLK,
   input	  MISO,
   output reg [DEVSEL_NUM-1:0]MCS
   );

   localparam BAUD_WIDTH = 8;
   //parameter BAUD_DIV = 0;
   parameter SPI_MODE = 0;
   parameter BC_WIDTH = 3;
   parameter DEVSEL_NUM = 1;
   localparam DIV_WIDTH = BAUD_WIDTH;// BAUD_DIV ? $clog2(BAUD_DIV / 2 - 1) :


   reg [7:0]	  sr8, bb8;
   wire [7:0]	  sr8_sf;
   reg [BC_WIDTH - 1:0]		bc, bc_next;
   reg [DIV_WIDTH - 1:0]	ccr;
   reg [DIV_WIDTH - 1:0]	cc, cc_next;
   wire		  misod;
   wire		  cstb, wstb, bstb, istb;
   reg		  sck;
   reg		  sf, ld;
   reg		  bba;   // buffer flag
   reg		  txren, txeen;
   wire 	  txr, txe;
   wire		  cpol, cpha;
   reg		  cpolr, cphar;
   wire 	  wr;
   // wire 	  cyc_i; // comment out for wishbone
   // wire 	  ack_o; // comment out for wishbone
   // assign cyc_i = 1'b1;  // comment out for wishbone
   assign ack_o = 1'b1; // zero wait
   assign wr = stb_i & cyc_i & we_i & ack_o;
   assign wstb = wr & (adr_i == 1);
   assign istb = wr & (adr_i == 0);
   assign cstb = wr & (adr_i == 0);
   assign bstb = wr & (adr_i == 0);
   assign dstb = wr & (adr_i == 3);
   assign sr8_sf = { sr8[6:0],misod };
   assign dat_o =
		      ({txren, txeen,cpolr,cphar,ccr} & {16{(adr_i == 0)}})
		    | (bb8 & {8{(adr_i == 1)}})
		    | ({ txr, txe } & {8{(adr_i == 2)}})
			| ( MCS & {8{(adr_i == 2)}})
		    ;

   parameter
     IDLE = 0,
     PHASE1 = 1,
     PHASE2 = 2
     ;

   reg [1:0] spi_seq, spi_seq_next;
   always @(posedge clk_i or posedge rst_i)
     if (rst_i)
       spi_seq <= IDLE;
     else
       spi_seq <= spi_seq_next;

   always @(posedge clk_i)
     begin
		cc <= cc_next;
		bc <= bc_next;
     end

   always @(bba or bc or cc or ccr or cpha or cpol or spi_seq)
     begin
		sck = cpol;
		cc_next = ccr;//BAUD_DIV ? (BAUD_DIV / 2 - 1) : 
		bc_next = bc;
		ld = 1'b0;
		sf = 1'b0;

		case (spi_seq)
		IDLE:
			begin
				if (bba)
				begin
					bc_next = 7;
					ld = 1'b1;
					spi_seq_next = PHASE2;
				end
				else
					spi_seq_next = IDLE;
			end
		PHASE2:
			begin
				sck = (cpol ^ cpha);
				if (cc == 0)
					spi_seq_next = PHASE1;
				else
				begin
					cc_next = cc - 1;
					spi_seq_next = PHASE2;
				end
			end
		PHASE1:
			begin
			sck = ~(cpol ^ cpha);
			if (cc == 0)
				begin
					bc_next = bc -1;
					sf = 1'b1;
					if (bc == 0)
					begin
						if (bba)
						begin
							bc_next = 7;
							ld = 1'b1;
							spi_seq_next = PHASE2;
						end
						else
							spi_seq_next = IDLE;
					end
					else
						spi_seq_next = PHASE2;
				end
			else
				begin
					cc_next = cc - 1;
					spi_seq_next = PHASE1;
				end
			end
		default:spi_seq_next = IDLE;
		endcase
     end // always @ (...

   always @(posedge clk_i)
     begin
		if (cstb) // control reg
		{ cpolr, cphar } <= dat_i[9:8];
		else
		{ cpolr, cphar } <= { cpolr, cphar };

		if (istb) // irq enable reg
		{ txren, txeen } <= dat_i[11:10];
		else
		{ txren, txeen } <= { txren, txeen };

		if (bstb) // baud reg
		ccr <= dat_i;
		else
		ccr <= ccr;

		if (ld)   // shift reg
		sr8 <= bb8;
		else if (sf)
		sr8 <= sr8_sf;
		else
		sr8 <= sr8;

		if (wstb) // buffer reg
		bb8 <= dat_i;
		else if (ld)
		bb8 <= (spi_seq == IDLE) ? sr8 : sr8_sf;
		else
		bb8 <= bb8;
     end // always @ (posedge clk_i)

   always @(posedge clk_i or posedge rst_i)
     begin
		if (rst_i)
		bba <= 1'b0;
		else if (wstb)
		bba <= 1'b1;
		else if (ld)
		bba <= 1'b0;
		else
		bba <= bba;
     end
	always @(posedge clk_i or posedge rst_i)
     begin
		if (rst_i)
		MCS <= {DEVSEL_NUM{1'b1}};
		else if (dstb)
		MCS <= dat_i;
		else
		MCS <= MCS;
     end
   assign { cpol, cpha } = ((SPI_MODE >= 0) & (SPI_MODE < 4)) ?
			   SPI_MODE : { cpolr, cphar };
   assign txe = (spi_seq == IDLE);
   assign txr = ~bba;
   assign int_o = (txr & txren) | (txe & txeen);
   assign SCLK = sck;
   assign MOSI = sr8[7];
   assign misod = MISO;

endmodule



module APB_IIC_MASTER
#(
    parameter STYLE="CONFIGURABLE",//FIXED
	parameter CLKRATE=50000000
)
(
input             APB_CLK,
input             APB_RST_i,

input [4:0 ]      APB2IIC_PADDR,
input             APB2IIC_PWRITE,
input             APB2IIC_PSEL,
input             APB2IIC_PENABLE,
input  [31:0]     APB2IIC_PWDATA,
output [31:0]     APB2IIC_PRDATA,
output            APB2IIC_PREADY,
//Interrupt
output wire 	  APB2IIC_INTERRUPT,
// I2C signals
// i2c clock line
input  wire SCL_I,       // SCL-line input
output wire SCL_O,       // SCL-line output (always 1'b0)
output wire SCL_T,    // SCL-line output enable (active high)

// i2c data line
input  wire SDA_I,       // SDA-line input
output wire SDA_O,       // SDA-line output (always 1'b0)
output wire SDA_T    // SDA-line output enable (active high)
);
wire  scl_padoen_o,sda_padoen_o;
assign SCL_T=~scl_padoen_o;
assign SDA_T=~sda_padoen_o;
i2c_master_top#(.STYLE(STYLE), .CLKRATE(CLKRATE)) 
i2c_master_top(
.wb_clk_i		(APB_CLK), 
.wb_rst_i		(APB_RST_i), 
.wb_adr_i		(APB2IIC_PADDR[4:2]),
.wb_dat_i		(APB2IIC_PWDATA), 
.wb_dat_o		(APB2IIC_PRDATA),
.wb_we_i		(APB2IIC_PWRITE), 
.wb_stb_i		(APB2IIC_PENABLE), 	//因为ENABLE信号比PSEL信号延后一个周期，故连接STB
.wb_cyc_i		(APB2IIC_PSEL),		//因为PSEL信号在ENABLE信号前一周期出现，所以PSEL信号连接CYC 
.wb_ack_o		(APB2IIC_PREADY), 
.wb_inta_o		(APBS2IIC_INTERRUPT),

//I2C INTERFACE
.scl_pad_i		(SCL_I), 
.scl_pad_o		(SCL_O), 
.scl_padoen_o	(scl_padoen_o), 
.sda_pad_i		(SDA_I), 
.sda_pad_o		(SDA_O), 
.sda_padoen_o	(sda_padoen_o) 
);


endmodule



`define I2C_CMD_NOP   4'b0000
`define I2C_CMD_START 4'b0001
`define I2C_CMD_STOP  4'b0010
`define I2C_CMD_WRITE 4'b0100
`define I2C_CMD_READ  4'b1000


module i2c_master_top(
	wb_clk_i, wb_rst_i, wb_adr_i, wb_dat_i, wb_dat_o,
	wb_we_i, wb_stb_i, wb_cyc_i, wb_ack_o, wb_inta_o,
	scl_pad_i, scl_pad_o, scl_padoen_o, sda_pad_i, sda_pad_o, sda_padoen_o );

	// parameters
	parameter STYLE="CONFIGURABLE";
	parameter CLKRATE=50000000;
	localparam FIXED_PSC=CLKRATE/500000;
	localparam PSC_WIDTH=(STYLE=="CONFIGURABLE")?16:$clog2(FIXED_PSC);
	//
	// inputs & outputs
	//

	// wishbone signals
	input        wb_clk_i;     // master clock input
	input        wb_rst_i;     // synchronous active high reset
	input  [2:0] wb_adr_i;     // lower address bits
	input  [31:0] wb_dat_i;     // databus input
	output [31:0] wb_dat_o;     // databus output
	input        wb_we_i;      // write enable input
	input        wb_stb_i;     // stobe/core select signal
	input        wb_cyc_i;     // valid bus cycle input
	output       wb_ack_o;     // bus cycle acknowledge output
	output       wb_inta_o;    // interrupt request signal output

	reg [31:0] wb_dat_o;
	reg wb_ack_o;
	reg wb_inta_o;

	// I2C signals
	// i2c clock line
	input  scl_pad_i;       // SCL-line input
	output scl_pad_o;       // SCL-line output (always 1'b0)
	output scl_padoen_o;    // SCL-line output enable (active low)

	// i2c data line
	input  sda_pad_i;       // SDA-line input
	output sda_pad_o;       // SDA-line output (always 1'b0)
	output sda_padoen_o;    // SDA-line output enable (active low)


	//
	// variable declarations
	//

	// registers
	wire [15:0] psc;
	reg  [15:0] prer; // clock prescale register
	reg  [ 7:0] ctr;  // control register
	reg  [ 7:0] txr;  // transmit register
	wire [ 7:0] rxr;  // receive register
	reg  [ 7:0] cr;   // command register
	wire [ 7:0] sr;   // status register

	// done signal: command completed, clear command register
	wire done;

	// core enable signal
	wire core_en;
	wire ien;

	// status register signals
	wire irxack;
	reg  rxack;       // received aknowledge from slave
	reg  tip;         // transfer in progress
	reg  irq_flag;    // interrupt pending flag
	wire i2c_busy;    // bus busy (start signal detected)
	wire i2c_al;      // i2c bus arbitration lost
	reg  al;          // status register arbitration lost bit

	wire rst_i = !wb_rst_i;

	// generate wishbone signals
	wire wb_wacc = wb_we_i & wb_stb_i & wb_cyc_i;

	// generate acknowledge output signal
	always @(posedge wb_clk_i)
	  wb_ack_o <= 1'b1; // because timing is always honoredwb_cyc_i & wb_stb_i & ~wb_ack_o

	// assign DAT_O
	always @(posedge wb_clk_i)
	begin
	  case (wb_adr_i) // synopsys parallel_case
	    3'b000: wb_dat_o <= (STYLE=="CONFIGURABLE") ? {ctr,prer[15:8],prer[ 7:0]}:{ctr,16'h0000 | FIXED_PSC};
	    3'b001: wb_dat_o <= {sr,cr};  // write is command register (cr)
	    3'b011: wb_dat_o <= rxr; // write is transmit register (txr)
	    default: wb_dat_o <= 0;   // reserved
	  endcase
	end

	// generate registers
	always @(posedge wb_clk_i or negedge rst_i)
	  if (!rst_i)
	    begin
	        //prer <= 16'hffff;,{prer [15:8],prer [ 7:0]}
	        ctr  <=  8'h0;
	        txr  <=  8'h0;
	    end
	  else if (wb_wacc )
	      case (wb_adr_i) // synopsys parallel_case
	         3'b000 : ctr <= wb_dat_i[23:16];
	         3'b011 : txr <= wb_dat_i;
	         default: ;
	      endcase

	generate 
		if(STYLE=="CONFIGURABLE")
		begin:PRER_GENERATE
			always @(posedge wb_clk_i or negedge rst_i)
			if (!rst_i)
				prer <= 16'hffff;
			else if (wb_wacc & (wb_adr_i==3'b000))
				prer <= wb_dat_i[15:0];
			else
				prer <= prer;
		end
	endgenerate
	// generate command register (special case)
	always @(posedge wb_clk_i or negedge rst_i)
	  if (!rst_i)
	    cr <= 8'h0;
	  else if (wb_wacc)
	    begin
	        if (core_en & (wb_adr_i == 3'b001) )
	          cr <= wb_dat_i[7:0];
	    end
	  else
	    begin
	        if (done | i2c_al)
	          cr[7:4] <= 4'h0;           // clear command bits when done
	                                        // or when aribitration lost
	        cr[2:1] <= 2'b0;             // reserved bits
	        cr[0]   <= 1'b0;             // clear IRQ_ACK bit
	    end


	// decode command register
	wire sta  = cr[7];
	wire sto  = cr[6];
	wire rd   = cr[5];
	wire wr   = cr[4];
	wire ack  = cr[3];
	wire iack = cr[0];

	// decode control register
	assign core_en = ctr[7];
	assign ien = ctr[6];

	// hookup byte controller block
	i2c_master_byte_ctrl byte_controller (
		.clk      ( wb_clk_i     ),
		.nReset   ( rst_i        ),
		.ena      ( core_en      ),
		.clk_cnt  ( (STYLE=="CONFIGURABLE")? prer : FIXED_PSC ),
		.start    ( sta          ),
		.stop     ( sto          ),
		.read     ( rd           ),
		.write    ( wr           ),
		.ack_in   ( ack          ),
		.din      ( txr          ),
		.cmd_ack  ( done         ),
		.ack_out  ( irxack       ),
		.dout     ( rxr          ),
		.i2c_busy ( i2c_busy     ),
		.i2c_al   ( i2c_al       ),
		.scl_i    ( scl_pad_i    ),
		.scl_o    ( scl_pad_o    ),
		.scl_oen  ( scl_padoen_o ),
		.sda_i    ( sda_pad_i    ),
		.sda_o    ( sda_pad_o    ),
		.sda_oen  ( sda_padoen_o )
	);

	// status register block + interrupt request signal
	always @(posedge wb_clk_i or negedge rst_i)
	  if (!rst_i)
	    begin
	        al       <= 1'b0;
	        rxack    <= 1'b0;
	        tip      <= 1'b0;
	        irq_flag <= 1'b0;
	    end
	  else 
	    begin
	        al       <= i2c_al | (al & ~sta);
	        rxack    <= irxack;
	        tip      <= (rd | wr);
	        irq_flag <= (done | i2c_al | irq_flag) & ~iack; // interrupt request flag is always generated
	    end

	// generate interrupt request signals
	always @(posedge wb_clk_i or negedge rst_i)
	  if (!rst_i)
	    wb_inta_o <= 1'b0;
	  else
	    wb_inta_o <= irq_flag && ien; // interrupt signal is only generated when IEN (interrupt enable bit is set)

	// assign status register bits
	assign sr[7]   = rxack;
	assign sr[6]   = i2c_busy;
	assign sr[5]   = al;
	assign sr[4:2] = 3'h0; // reserved
	assign sr[1]   = tip;
	assign sr[0]   = irq_flag;

endmodule



`define I2C_CMD_NOP   4'b0000
`define I2C_CMD_START 4'b0001
`define I2C_CMD_STOP  4'b0010
`define I2C_CMD_WRITE 4'b0100
`define I2C_CMD_READ  4'b1000


module i2c_master_byte_ctrl (
	clk,  nReset, ena, clk_cnt, start, stop, read, write, ack_in, din,
	cmd_ack, ack_out, dout, i2c_busy, i2c_al, scl_i, scl_o, scl_oen, sda_i, sda_o, sda_oen );

	parameter PSC_WIDTH = 16;
	//
	// inputs & outputs
	//
	input clk;     // master clock
	input nReset;  // asynchronous active low reset
	input ena;     // core enable signal
	input [PSC_WIDTH-1:0] clk_cnt; // 4x SCL

	// control inputs
	input       start;
	input       stop;
	input       read;
	input       write;
	input       ack_in;
	input [7:0] din;

	// status outputs
	output       cmd_ack;
	reg cmd_ack;
	output       ack_out;
	reg ack_out;
	output       i2c_busy;
	output       i2c_al;
	output [7:0] dout;

	// I2C signals
	input  scl_i;
	output scl_o;
	output scl_oen;
	input  sda_i;
	output sda_o;
	output sda_oen;


	//
	// Variable declarations
	//

	// statemachine
	parameter [4:0] ST_IDLE  = 5'b0_0000;
	parameter [4:0] ST_START = 5'b0_0001;
	parameter [4:0] ST_READ  = 5'b0_0010;
	parameter [4:0] ST_WRITE = 5'b0_0100;
	parameter [4:0] ST_ACK   = 5'b0_1000;
	parameter [4:0] ST_STOP  = 5'b1_0000;

	// signals for bit_controller
	reg  [3:0] core_cmd;
	reg        core_txd;
	wire       core_ack, core_rxd;

	// signals for shift register
	reg [7:0] sr; //8bit shift register
	reg       shift, ld;

	// signals for state machine
	wire       go;
	reg  [2:0] dcnt;
	wire       cnt_done;

	//
	// Module body
	//

	// hookup bit_controller
	i2c_master_bit_ctrl #(.PSC_WIDTH(PSC_WIDTH))
	bit_controller (
		.clk     ( clk      ),
		.nReset  ( nReset   ),
		.ena     ( ena      ),
		.clk_cnt ( clk_cnt  ),
		.cmd     ( core_cmd ),
		.cmd_ack ( core_ack ),
		.busy    ( i2c_busy ),
		.al      ( i2c_al   ),
		.din     ( core_txd ),
		.dout    ( core_rxd ),
		.scl_i   ( scl_i    ),
		.scl_o   ( scl_o    ),
		.scl_oen ( scl_oen  ),
		.sda_i   ( sda_i    ),
		.sda_o   ( sda_o    ),
		.sda_oen ( sda_oen  )
	);

	// generate go-signal
	assign go = (read | write | stop) & ~cmd_ack;

	// assign dout output to shift-register
	assign dout = sr;

	// generate shift register
	always @(posedge clk or negedge nReset)
	  if (!nReset)
	    sr <= 8'h0;
	  else if (ld)
	    sr <= din;
	  else if (shift)
	    sr <= {sr[6:0], core_rxd};

	// generate counter
	always @(posedge clk or negedge nReset)
	  if (!nReset)
	    dcnt <= 3'h0;
	  else if (ld)
	    dcnt <= 3'h7;
	  else if (shift)
	    dcnt <= dcnt - 3'h1;

	assign cnt_done = ~(|dcnt);

	//
	// state machine
	//
	reg [4:0] c_state; 

	always @(posedge clk or negedge nReset)
	  if (!nReset)
	    begin
	        core_cmd <= `I2C_CMD_NOP;
	        core_txd <= 1'b0;
	        shift    <= 1'b0;
	        ld       <= 1'b0;
	        cmd_ack  <= 1'b0;
	        c_state  <= ST_IDLE;
	        ack_out  <= 1'b0;
	    end
	  else if (i2c_al)
	   begin
	       core_cmd <= `I2C_CMD_NOP;
	       core_txd <= 1'b0;
	       shift    <= 1'b0;
	       ld       <= 1'b0;
	       cmd_ack  <= 1'b0;
	       c_state  <= ST_IDLE;
	       ack_out  <= 1'b0;
	   end
	else
	  begin
	      // initially reset all signals
	      core_txd <= sr[7];
	      shift    <= 1'b0;
	      ld       <= 1'b0;
	      cmd_ack  <= 1'b0;

	      case (c_state) // synopsys full_case parallel_case
	        ST_IDLE:
	          if (go)
	            begin
	                if (start)
	                  begin
	                      c_state  <= ST_START;
	                      core_cmd <= `I2C_CMD_START;
	                  end
	                else if (read)
	                  begin
	                      c_state  <= ST_READ;
	                      core_cmd <= `I2C_CMD_READ;
	                  end
	                else if (write)
	                  begin
	                      c_state  <= ST_WRITE;
	                      core_cmd <= `I2C_CMD_WRITE;
	                  end
	                else // stop
	                  begin
	                      c_state  <= ST_STOP;
	                      core_cmd <= `I2C_CMD_STOP;
	                  end

	                ld <= 1'b1;
	            end

	        ST_START:
	          if (core_ack)
	            begin
	                if (read)
	                  begin
	                      c_state  <= ST_READ;
	                      core_cmd <= `I2C_CMD_READ;
	                  end
	                else
	                  begin
	                      c_state  <= ST_WRITE;
	                      core_cmd <= `I2C_CMD_WRITE;
	                  end

	                ld <= 1'b1;
	            end

	        ST_WRITE:
	          if (core_ack)
	            if (cnt_done)
	              begin
	                  c_state  <= ST_ACK;
	                  core_cmd <= `I2C_CMD_READ;
	              end
	            else
	              begin
	                  c_state  <= ST_WRITE;       // stay in same state
	                  core_cmd <= `I2C_CMD_WRITE; // write next bit
	                  shift    <= 1'b1;
	              end

	        ST_READ:
	          if (core_ack)
	            begin
	                if (cnt_done)
	                  begin
	                      c_state  <= ST_ACK;
	                      core_cmd <= `I2C_CMD_WRITE;
	                  end
	                else
	                  begin
	                      c_state  <= ST_READ;       // stay in same state
	                      core_cmd <= `I2C_CMD_READ; // read next bit
	                  end

	                shift    <= 1'b1;
	                core_txd <= ack_in;
	            end

	        ST_ACK:
	          if (core_ack)
	            begin
	               if (stop)
	                 begin
	                     c_state  <= ST_STOP;
	                     core_cmd <= `I2C_CMD_STOP;
	                 end
	               else
	                 begin
	                     c_state  <= ST_IDLE;
	                     core_cmd <= `I2C_CMD_NOP;

	                     // generate command acknowledge signal
	                     cmd_ack  <= 1'b1;
	                 end

	                 // assign ack_out output to bit_controller_rxd (contains last received bit)
	                 ack_out <= core_rxd;

	                 core_txd <= 1'b1;
	             end
	           else
	             core_txd <= ack_in;

	        ST_STOP:
	          if (core_ack)
	            begin
	                c_state  <= ST_IDLE;
	                core_cmd <= `I2C_CMD_NOP;

	                // generate command acknowledge signal
	                cmd_ack  <= 1'b1;
	            end

	      endcase
	  end
endmodule



`define I2C_CMD_NOP   4'b0000
`define I2C_CMD_START 4'b0001
`define I2C_CMD_STOP  4'b0010
`define I2C_CMD_WRITE 4'b0100
`define I2C_CMD_READ  4'b1000

module i2c_master_bit_ctrl 
#(parameter PSC_WIDTH=16)
(
    input             clk,      // system clock
    input             nReset,   // asynchronous active low reset
    input             ena,      // core enable signal

    input      [PSC_WIDTH-1:0] clk_cnt,  // clock prescale value

    input      [ 3:0] cmd,      // command (from byte controller)
    output reg        cmd_ack,  // command complete acknowledge
    output reg        busy,     // i2c bus busy
    output reg        al,       // i2c bus arbitration lost

    input             din,
    output reg        dout,

    input             scl_i,    // i2c clock line input
    output            scl_o,    // i2c clock line output
    output reg        scl_oen,  // i2c clock line output enable (active low)
    input             sda_i,    // i2c data line input
    output            sda_o,    // i2c data line output
    output reg        sda_oen   // i2c data line output enable (active low)
);


    //
    // variable declarations
    //

    reg [ 1:0] cSCL, cSDA;      // capture SCL and SDA
    reg [ 2:0] fSCL, fSDA;      // SCL and SDA filter inputs
    reg        sSCL, sSDA;      // filtered and synchronized SCL and SDA inputs
    reg        dSCL, dSDA;      // delayed versions of sSCL and sSDA
    reg        dscl_oen;        // delayed scl_oen
    reg        sda_chk;         // check SDA output (Multi-master arbitration)
    reg        clk_en;          // clock generation signals
    reg        slave_wait;      // slave inserts wait states
    reg [PSC_WIDTH-1:0] cnt;             // clock divider counter (synthesis)
    reg [PSC_WIDTH-3:0] filter_cnt;      // clock divider for filter


    // state machine variable
    reg [17:0] c_state; 

    //
    // module body
    //

    // whenever the slave is not ready it can delay the cycle by pulling SCL low
    // delay scl_oen
    always @(posedge clk)
      dscl_oen <= scl_oen;

    // slave_wait is asserted when master wants to drive SCL high, but the slave pulls it low
    // slave_wait remains asserted until the slave releases SCL
    always @(posedge clk or negedge nReset)
      if (!nReset) slave_wait <= 1'b0;
      else         slave_wait <= (scl_oen & ~dscl_oen & ~sSCL) | (slave_wait & ~sSCL);

    // master drives SCL high, but another master pulls it low
    // master start counting down its low cycle now (clock synchronization)
    wire scl_sync   = dSCL & ~sSCL & scl_oen;


    // generate clk enable signal
    always @(posedge clk or negedge nReset)
      if (~nReset)
      begin
          cnt    <= 16'h0;
          clk_en <= 1'b1;
      end
      else if (~|cnt || !ena || scl_sync)
      begin
          cnt    <= clk_cnt;
          clk_en <= 1'b1;
      end
      else if (slave_wait)
      begin
          cnt    <= cnt;
          clk_en <= 1'b0;    
      end
      else
      begin
          cnt    <= cnt - 16'h1;
          clk_en <= 1'b0;
      end


    // generate bus status controller

    // capture SDA and SCL
    // reduce metastability risk
    always @(posedge clk or negedge nReset)
      if (!nReset)
      begin
          cSCL <= 2'b00;
          cSDA <= 2'b00;
      end
      else
      begin
          cSCL <= {cSCL[0],scl_i};
          cSDA <= {cSDA[0],sda_i};
      end


    // filter SCL and SDA signals; (attempt to) remove glitches
    always @(posedge clk or negedge nReset)
      if      (!nReset     ) filter_cnt <= 14'h0;
      else if (!ena )        filter_cnt <= 14'h0;
      else if (~|filter_cnt) filter_cnt <= clk_cnt >> 2; //16x I2C bus frequency
      else                   filter_cnt <= filter_cnt -1;


    always @(posedge clk or negedge nReset)
      if (!nReset)
      begin
          fSCL <= 3'b111;
          fSDA <= 3'b111;
      end
      else if (~|filter_cnt)
      begin
          fSCL <= {fSCL[1:0],cSCL[1]};
          fSDA <= {fSDA[1:0],cSDA[1]};
      end


    // generate filtered SCL and SDA signals
    always @(posedge clk or negedge nReset)
      if (~nReset)
      begin
          sSCL <= 1'b1;
          sSDA <= 1'b1;

          dSCL <= 1'b1;
          dSDA <= 1'b1;
      end
      else
      begin
          sSCL <= &fSCL[2:1] | &fSCL[1:0] | (fSCL[2] & fSCL[0]);
          sSDA <= &fSDA[2:1] | &fSDA[1:0] | (fSDA[2] & fSDA[0]);

          dSCL <= sSCL;
          dSDA <= sSDA;
      end

    // detect start condition => detect falling edge on SDA while SCL is high
    // detect stop condition => detect rising edge on SDA while SCL is high
    reg sta_condition;
    reg sto_condition;
    always @(posedge clk or negedge nReset)
      if (~nReset)
      begin
          sta_condition <= 1'b0;
          sto_condition <= 1'b0;
      end
      else
      begin
          sta_condition <= ~sSDA &  dSDA & sSCL;
          sto_condition <=  sSDA & ~dSDA & sSCL;
      end


    // generate i2c bus busy signal
    always @(posedge clk or negedge nReset)
      if      (!nReset) busy <= 1'b0;
      else              busy <= (sta_condition | busy) & ~sto_condition;


    // generate arbitration lost signal
    // aribitration lost when:
    // 1) master drives SDA high, but the i2c bus is low
    // 2) stop detected while not requested
    reg cmd_stop;
    always @(posedge clk or negedge nReset)
      if (~nReset)
          cmd_stop <= 1'b0;
      else if (clk_en)
          cmd_stop <= cmd == `I2C_CMD_STOP;

    always @(posedge clk or negedge nReset)
      if (~nReset)
          al <= 1'b0;
      else
          al <= (sda_chk & ~sSDA & sda_oen) | (|c_state & sto_condition & ~cmd_stop);


    // generate dout signal (store SDA on rising edge of SCL)
    always @(posedge clk)
      if (sSCL & ~dSCL) dout <= sSDA;


    // generate statemachine

    // nxt_state decoder
    parameter [17:0] idle    = 18'b0_0000_0000_0000_0000;
    parameter [17:0] start_a = 18'b0_0000_0000_0000_0001;
    parameter [17:0] start_b = 18'b0_0000_0000_0000_0010;
    parameter [17:0] start_c = 18'b0_0000_0000_0000_0100;
    parameter [17:0] start_d = 18'b0_0000_0000_0000_1000;
    parameter [17:0] start_e = 18'b0_0000_0000_0001_0000;
    parameter [17:0] stop_a  = 18'b0_0000_0000_0010_0000;
    parameter [17:0] stop_b  = 18'b0_0000_0000_0100_0000;
    parameter [17:0] stop_c  = 18'b0_0000_0000_1000_0000;
    parameter [17:0] stop_d  = 18'b0_0000_0001_0000_0000;
    parameter [17:0] rd_a    = 18'b0_0000_0010_0000_0000;
    parameter [17:0] rd_b    = 18'b0_0000_0100_0000_0000;
    parameter [17:0] rd_c    = 18'b0_0000_1000_0000_0000;
    parameter [17:0] rd_d    = 18'b0_0001_0000_0000_0000;
    parameter [17:0] wr_a    = 18'b0_0010_0000_0000_0000;
    parameter [17:0] wr_b    = 18'b0_0100_0000_0000_0000;
    parameter [17:0] wr_c    = 18'b0_1000_0000_0000_0000;
    parameter [17:0] wr_d    = 18'b1_0000_0000_0000_0000;

    always @(posedge clk or negedge nReset)
      if (!nReset)
      begin
          c_state <= idle;
          cmd_ack <= 1'b0;
          scl_oen <= 1'b1;
          sda_oen <= 1'b1;
          sda_chk <= 1'b0;
      end
      else if ( al)
      begin
          c_state <= idle;
          cmd_ack <= 1'b0;
          scl_oen <= 1'b1;
          sda_oen <= 1'b1;
          sda_chk <= 1'b0;
      end
      else
      begin
          cmd_ack   <= 1'b0; // default no command acknowledge + assert cmd_ack only 1clk cycle

          if (clk_en)
              case (c_state) // synopsys full_case parallel_case
                    // idle state
                    idle:
                    begin
                        case (cmd) // synopsys full_case parallel_case
                             `I2C_CMD_START: c_state <= start_a;
                             `I2C_CMD_STOP:  c_state <= stop_a;
                             `I2C_CMD_WRITE: c_state <= wr_a;
                             `I2C_CMD_READ:  c_state <= rd_a;
                             default:        c_state <= idle;
                        endcase

                        scl_oen <= scl_oen; // keep SCL in same state
                        sda_oen <= sda_oen; // keep SDA in same state
                        sda_chk <= 1'b0;    // don't check SDA output
                    end

                    // start
                    start_a:
                    begin
                        c_state <= start_b;
                        scl_oen <= scl_oen; // keep SCL in same state
                        sda_oen <= 1'b1;    // set SDA high
                        sda_chk <= 1'b0;    // don't check SDA output
                    end

                    start_b:
                    begin
                        c_state <= start_c;
                        scl_oen <= 1'b1; // set SCL high
                        sda_oen <= 1'b1; // keep SDA high
                        sda_chk <= 1'b0; // don't check SDA output
                    end

                    start_c:
                    begin
                        c_state <= start_d;
                        scl_oen <= 1'b1; // keep SCL high
                        sda_oen <= 1'b0; // set SDA low
                        sda_chk <= 1'b0; // don't check SDA output
                    end

                    start_d:
                    begin
                        c_state <= start_e;
                        scl_oen <= 1'b1; // keep SCL high
                        sda_oen <= 1'b0; // keep SDA low
                        sda_chk <= 1'b0; // don't check SDA output
                    end

                    start_e:
                    begin
                        c_state <= idle;
                        cmd_ack <= 1'b1;
                        scl_oen <= 1'b0; // set SCL low
                        sda_oen <= 1'b0; // keep SDA low
                        sda_chk <= 1'b0; // don't check SDA output
                    end

                    // stop
                    stop_a:
                    begin
                        c_state <= stop_b;
                        scl_oen <= 1'b0; // keep SCL low
                        sda_oen <= 1'b0; // set SDA low
                        sda_chk <= 1'b0; // don't check SDA output
                    end

                    stop_b:
                    begin
                        c_state <= stop_c;
                        scl_oen <= 1'b1; // set SCL high
                        sda_oen <= 1'b0; // keep SDA low
                        sda_chk <= 1'b0; // don't check SDA output
                    end

                    stop_c:
                    begin
                        c_state <= stop_d;
                        scl_oen <= 1'b1; // keep SCL high
                        sda_oen <= 1'b0; // keep SDA low
                        sda_chk <= 1'b0; // don't check SDA output
                    end

                    stop_d:
                    begin
                        c_state <= idle;
                        cmd_ack <= 1'b1;
                        scl_oen <= 1'b1; // keep SCL high
                        sda_oen <= 1'b1; // set SDA high
                        sda_chk <= 1'b0; // don't check SDA output
                    end

                    // read
                    rd_a:
                    begin
                        c_state <= rd_b;
                        scl_oen <= 1'b0; // keep SCL low
                        sda_oen <= 1'b1; // tri-state SDA
                        sda_chk <= 1'b0; // don't check SDA output
                    end

                    rd_b:
                    begin
                        c_state <= rd_c;
                        scl_oen <= 1'b1; // set SCL high
                        sda_oen <= 1'b1; // keep SDA tri-stated
                        sda_chk <= 1'b0; // don't check SDA output
                    end

                    rd_c:
                    begin
                        c_state <= rd_d;
                        scl_oen <= 1'b1; // keep SCL high
                        sda_oen <= 1'b1; // keep SDA tri-stated
                        sda_chk <= 1'b0; // don't check SDA output
                    end

                    rd_d:
                    begin
                        c_state <= idle;
                        cmd_ack <= 1'b1;
                        scl_oen <= 1'b0; // set SCL low
                        sda_oen <= 1'b1; // keep SDA tri-stated
                        sda_chk <= 1'b0; // don't check SDA output
                    end

                    // write
                    wr_a:
                    begin
                        c_state <= wr_b;
                        scl_oen <= 1'b0; // keep SCL low
                        sda_oen <= din;  // set SDA
                        sda_chk <= 1'b0; // don't check SDA output (SCL low)
                    end

                    wr_b:
                    begin
                        c_state <= wr_c;
                        scl_oen <= 1'b1; // set SCL high
                        sda_oen <= din;  // keep SDA
                        sda_chk <= 1'b0; // don't check SDA output yet
                                            // allow some time for SDA and SCL to settle
                    end

                    wr_c:
                    begin
                        c_state <= wr_d;
                        scl_oen <= 1'b1; // keep SCL high
                        sda_oen <= din;
                        sda_chk <= 1'b1; // check SDA output
                    end

                    wr_d:
                    begin
                        c_state <= idle;
                        cmd_ack <= 1'b1;
                        scl_oen <= 1'b0; // set SCL low
                        sda_oen <= din;
                        sda_chk <= 1'b0; // don't check SDA output (SCL low)
                    end

              endcase
      end


    // assign scl and sda output (always gnd)
    assign scl_o = 1'b0;
    assign sda_o = 1'b0;

endmodule




























