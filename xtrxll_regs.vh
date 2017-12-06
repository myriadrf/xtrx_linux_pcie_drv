//
// Port constant that used in driver and in FPGA code
//

//WR
localparam GP_PORT_WR_SPI_LMS7_0  = 0;
localparam GP_PORT_WR_INT_PCIE    = 1;
localparam GP_PORT_WR_DAC_SPI     = 2;
localparam GP_PORT_WR_TMP102      = 3;

localparam GP_PORT_WR_UART_TX     = 4;
localparam GP_PORT_WR_SIM_TX      = 5;
localparam GP_PORT_WR_SIM_CTRL    = 6;
localparam GP_PORT_WR_LMS_CTRL    = 7;

localparam GP_PORT_WR_TXDMA_CNF_L = 8;
localparam GP_PORT_WR_TXDMA_CNF_T = 9;
localparam GP_PORT_WR_TCMD_D      = 10;
localparam GP_PORT_WR_TCMD_T      = 11;

localparam GP_PORT_WR_RXDMA_CNFRM = 12;
localparam GP_PORT_WR_RXTXDMA     = 13;
localparam GP_PORT_WR_TXMMCM      = 14;
localparam GP_PORT_WR_RF_SWITCHES = 15;

localparam GP_PORT_WR_QSPI_EXCMD  = 16;
localparam GP_PORT_WR_QSPI_CMD    = 17;
localparam GP_PORT_WR_MEM_CTRL    = 18;
localparam GP_PORT_WR_USB_CTRL    = 19;

localparam GP_PORT_WR_USB_FIFO_CTRL = 20;
localparam GP_PORT_WR_USB_FIFO_PTRS = 21;


//RD
localparam GP_PORT_RD_SPI_LMS7_0  = 0;
localparam GP_PORT_RD_INTERRUPTS  = 1;
localparam GP_PORT_RD_ONEPPS      = 2;
localparam GP_PORT_RD_TMP102      = 3;

localparam GP_PORT_RD_UART_RX     = 4;
localparam GP_PORT_RD_SIM_RX      = 5;
localparam GP_PORT_RD_SIM_STAT    = 6;
localparam GP_PORT_RD_MCU_STAT    = 7;

localparam GP_PORT_RD_TXDMA_STAT  = 8;
localparam GP_PORT_RD_TXDMA_STATM = 9;
localparam GP_PORT_RD_TXDMA_STATTS= 10;
localparam GP_PORT_RD_TXDMA_ST_CPL= 11;

localparam GP_PORT_RD_RXDMA_STAT  = 12;
localparam GP_PORT_RD_RXDMA_STATTS= 13;
localparam GP_PORT_RD_TXMMCM      = 14;
localparam GP_PORT_RD_TCMDSTAT    = 15;

localparam GP_PORT_RD_QSPI_RB     = 16;
localparam GP_PORT_RD_QSPI_STAT   = 17;
localparam GP_PORT_RD_MEM_RB      = 18;
localparam GP_PORT_RD_MCU_DEBUG   = 19;

localparam GP_PORT_RD_REF_OSC     = 20;
localparam GP_PORT_RD_RXIQ_MISS   = 21;
localparam GP_PORT_RD_RXIQ_ODD    = 22;
localparam GP_PORT_RD_RXIQ_BI_BQ  = 23;

localparam GP_PORT_RD_USB_RB      = 24;
localparam GP_PORT_RD_USB_DEBUG   = 25;
localparam GP_PORT_RD_USB_DEBUG1  = 26;
localparam GP_PORT_RD_USB_DEBUG2  = 27;

localparam GP_PORT_RD_USB_DEBUG3  = 28;
localparam GP_PORT_RD_USB_DEBUG4  = 29;
localparam GP_PORT_RD_USB_DEBUG5  = 30;
localparam GP_PORT_RD_USB_DEBUG6  = 31;

localparam GP_PORT_RD_USB_EP0_O   = 32;
localparam GP_PORT_RD_USB_EP1_O   = 32 + 1;
localparam GP_PORT_RD_USB_EP2_O   = 32 + 2;
localparam GP_PORT_RD_USB_EP3_O   = 32 + 3;
localparam GP_PORT_RD_USB_EP4_O   = 32 + 4;
localparam GP_PORT_RD_USB_EP5_O   = 32 + 5;
localparam GP_PORT_RD_USB_EP6_O   = 32 + 6;
localparam GP_PORT_RD_USB_EP7_O   = 32 + 7;
localparam GP_PORT_RD_USB_EP8_O   = 32 + 8;
localparam GP_PORT_RD_USB_EP9_O   = 32 + 9;
localparam GP_PORT_RD_USB_EP10_O  = 32 + 10;
localparam GP_PORT_RD_USB_EP11_O  = 32 + 11;
localparam GP_PORT_RD_USB_EP12_O  = 32 + 12;
localparam GP_PORT_RD_USB_EP13_O  = 32 + 13;
localparam GP_PORT_RD_USB_EP14_O  = 32 + 14;
localparam GP_PORT_RD_USB_EP15_O  = 32 + 15;

localparam GP_PORT_RD_USB_EP0_I   = 48;
localparam GP_PORT_RD_USB_EP1_I   = 48 + 1;
localparam GP_PORT_RD_USB_EP2_I   = 48 + 2;
localparam GP_PORT_RD_USB_EP3_I   = 48 + 3;
localparam GP_PORT_RD_USB_EP4_I   = 48 + 4;
localparam GP_PORT_RD_USB_EP5_I   = 48 + 5;
localparam GP_PORT_RD_USB_EP6_I   = 48 + 6;
localparam GP_PORT_RD_USB_EP7_I   = 48 + 7;
localparam GP_PORT_RD_USB_EP8_I   = 48 + 8;
localparam GP_PORT_RD_USB_EP9_I   = 48 + 9;
localparam GP_PORT_RD_USB_EP10_I  = 48 + 10;
localparam GP_PORT_RD_USB_EP11_I  = 48 + 11;
localparam GP_PORT_RD_USB_EP12_I  = 48 + 12;
localparam GP_PORT_RD_USB_EP13_I  = 48 + 13;
localparam GP_PORT_RD_USB_EP14_I  = 48 + 14;
localparam GP_PORT_RD_USB_EP15_I  = 48 + 15;

// WR bus routing
localparam UL_GP_ADDR    = 0;
localparam UL_MCU_ADDR   = 256;
localparam UL_RXDMA_ADDR = 512;
localparam UL_TXDMA_ADDR = 768;

localparam UL_RD_MEM_ADDR = 512;


// Bits in GP_PORT_LMS_CTRL
localparam GP_PORT_LMS_CTRL_DIGRESET = 0;
localparam GP_PORT_LMS_CTRL_RESET    = 1;
localparam GP_PORT_LMS_CTRL_GPWR     = 2;
localparam GP_PORT_LMS_CTRL_RXEN     = 3;
localparam GP_PORT_LMS_CTRL_TXEN     = 4;
localparam GP_PORT_LMS_RX_TRXIQ      = 5;
localparam GP_PORT_LMS_FCLK_RX_GEN   = 6;

localparam GP_PORT_XTRX_ENBPVIO_N     = 8;
localparam GP_PORT_XTRX_ENBP3V3_N     = 9;
localparam GP_PORT_XTRX_EXT_CLK       = 10;
localparam GP_PORT_XTRX_PD_TCXO       = 11;


// Bits in GP_PORT_RD_SIM_RX / GP_PORT_RD_UART_RX / GP_PORT_RD_SIM_STAT
localparam UART_FIFORX_PARERR    = 14;
localparam UART_FIFORX_EMPTY     = 15;

localparam UART_FIFOTX_USED_OFF  = 16;
localparam UART_FIFOTX_USED_BITS = 5;
localparam UART_FIFOTX_EMPTY     = 21;

// Bits in GP_PORT_WR_SIM_CTRL
localparam WR_SIM_CTRL_RESET  = 0;
localparam WR_SIM_CTRL_ENABLE = 1;
localparam WR_SIM_CTRL_33V    = 2;

// Bits in GP_PORT_WR_TXMMCM
localparam GP_PORT_TXMMCM_REGEN   = 23;
localparam GP_PORT_TXMMCM_REGWR   = 24;
localparam GP_PORT_TXMMCM_CLKSEL  = 25;
localparam GP_PORT_TXMMCM_RESET   = 26;
localparam GP_PORT_TXMMCM_PWRDOWN = 27;
// Bits in GP_PORT_RD_TXMMCM
localparam GP_PORT_TXMMCM_LOCKED  = 28;
localparam GP_PORT_TXMMCM_RDY     = 29;



// Formats for RX/TX DMA
localparam FMT_STOP  = 0;
localparam FMT_8BIT  = 1;
localparam FMT_12BIT = 2;
localparam FMT_16BIT = 3;

localparam WR_RXDMA_FE_FMT_OFF   = 0;
localparam WR_RXDMA_FE_DECIM_OFF = 2;
localparam WR_RXDMA_FE_PAUSED    = 4;
localparam WR_RXDMA_FE_RESET     = 5;
localparam WR_RXDMA_FE_SISO      = 6;


localparam GP_PORT_TXDMA_CTRL_MODE_REP  = 2;
localparam GP_PORT_TXDMA_CTRL_MODE_SISO = 3;
localparam _GP_PORT_TXDMA_CTRL_REP_OFF   = 4;
localparam GP_PORT_TXDMA_CTRL_MODE_INTER_OFF = 4;



localparam GP_PORT_WR_RXTXDMA_RXOFF = 20;
localparam GP_PORT_WR_RXTXDMA_RXV = 30;
localparam GP_PORT_WR_RXTXDMA_TXV = 31;


// Interrupts
localparam INT_L_GPS_UART_TX  = 0;
localparam INT_L_GPS_UART_RX  = 1;
localparam INT_L_SIM_UART_TX  = 2;
localparam INT_L_SIM_UART_RX  = 3;
localparam INT_L_TEMP_ALARM   = 4;
localparam INT_L_1PPS         = 5;

// Direct MSI map
localparam INT_H_LMS7_SPI     = 0;
localparam INT_H_DMA_TX       = 1;
localparam INT_H_DMA_RX       = 2;
localparam INT_H_OTHER        = 3;

localparam INT_COUNT_L = 6;
localparam INT_COUNT_H = 3;

localparam INT_PCIE_I_FLAG = 15;
localparam INT_PCIE_E_FLAG = 31;


//localparam TIMED_COMMANDS
localparam TC_TS_BITS = 30;
localparam TC_ROUTE_BITS = 2;
localparam TC_DATA_BITS = 4;



localparam TC_ROUTE_RX_FRM = 0;


localparam RD_TCMDSTAT_FIFOREADY = 5;


localparam MCU_CTRL_VALID = 8;
localparam MCU_CTRL_RESET = 9;
localparam MCU_CTRL_SUSP = 10;
localparam MCU_CTRL_REGSEL = 11;
localparam MCU_CTRL_PAGESEL = 16;


localparam MCU_STAT_FIFO_EMPTY = 8;
localparam MCU_STAT_CPU_STOP = 9;
localparam MCU_STAT_PC = 10;



