// ============================================================================
// Module: apb_spi_top
// Description: Top-level AMBA APB-to-SPI Bridge connecting the APB Slave
//              register block to the SPI Master serial engine.
// ============================================================================
module apb_spi_top (
    // AMBA APB Bus Ports
    input  wire        pclk,
    input  wire        presetn,
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [7:0]  paddr,
    input  wire [31:0] pwdata,
    output wire [31:0] prdata,
    output wire        pready,
    output wire        pslverr,

    // External SPI Master Ports
    output wire        sclk,
    output wire        mosi,
    input  wire        miso,
    output wire        cs_n
);

    // Internal interconnect wires between APB Slave and SPI Master
    wire       spi_start;
    wire [7:0] spi_tx_data;
    wire [7:0] spi_clk_div;
    wire       spi_cpol;
    wire       spi_cpha;
    wire [7:0] spi_rx_data;
    wire       spi_busy;
    wire       spi_done;

    // Instantiate AMBA APB Slave Register File
    apb_slave u_apb_slave (
        .pclk        (pclk),
        .presetn     (presetn),
        .psel        (psel),
        .penable     (penable),
        .pwrite      (pwrite),
        .paddr       (paddr),
        .pwdata      (pwdata),
        .prdata      (prdata),
        .pready      (pready),
        .pslverr     (pslverr),
        .spi_start   (spi_start),
        .spi_tx_data (spi_tx_data),
        .spi_clk_div (spi_clk_div),
        .spi_cpol    (spi_cpol),
        .spi_cpha    (spi_cpha),
        .spi_rx_data (spi_rx_data),
        .spi_busy    (spi_busy),
        .spi_done    (spi_done)
    );

    // Instantiate SPI Master Engine
    spi_master u_spi_master (
        .clk     (pclk),
        .rst_n   (presetn),
        .start   (spi_start),
        .tx_data (spi_tx_data),
        .clk_div (spi_clk_div),
        .cpol    (spi_cpol),
        .cpha    (spi_cpha),
        .rx_data (spi_rx_data),
        .busy    (spi_busy),
        .done    (spi_done),
        .sclk    (sclk),
        .mosi    (mosi),
        .miso    (miso),
        .cs_n    (cs_n)
    );

endmodule