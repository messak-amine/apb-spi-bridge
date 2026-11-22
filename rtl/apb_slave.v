// ============================================================================
// Module: apb_slave
// Description: AMBA APB3 Slave interface with memory-mapped registers for
//              controlling and monitoring the SPI Master core.
//
// Register Map (Word-aligned offsets):
//   0x00 - CTRL    [R/W] : [0] spi_en, [1] cpol, [2] cpha, [15:8] clk_div
//   0x04 - STATUS  [RO]  : [0] busy, [1] rx_valid (cleared on RX_DATA read)
//   0x08 - TX_DATA [R/W] : [7:0] Byte to send (writing triggers SPI transfer)
//   0x0C - RX_DATA [RO]  : [7:0] Last received byte from MISO
// ============================================================================
module apb_slave (
    // AMBA APB Bus Interface
    input  wire        pclk,      // APB system clock
    input  wire        presetn,   // APB active-low asynchronous reset
    input  wire        psel,      // Peripheral select
    input  wire        penable,   // Enable signal (high during ACCESS phase)
    input  wire        pwrite,    // Direction (1 = Write, 0 = Read)
    input  wire [7:0]  paddr,     // Register byte address
    input  wire [31:0] pwdata,    // Write data bus
    output reg  [31:0] prdata,    // Read data bus
    output wire        pready,    // Slave ready (zero wait-state)
    output wire        pslverr,   // Slave error (tied low)

    // Internal Control/Status Interface to SPI Master
    output reg         spi_start, // 1-cycle start pulse
    output reg  [7:0]  spi_tx_data,
    output reg  [7:0]  spi_clk_div,
    output reg         spi_cpol,
    output reg         spi_cpha,
    input  wire [7:0]  spi_rx_data,
    input  wire        spi_busy,
    input  wire        spi_done
);

    // Register address offsets
    localparam [7:0] ADDR_CTRL    = 8'h00,
                     ADDR_STATUS  = 8'h04,
                     ADDR_TX_DATA = 8'h08,
                     ADDR_RX_DATA = 8'h0C;

    reg       spi_en;
    reg       rx_valid;
    reg [7:0] rx_data_reg;

    // Zero wait-state APB slave with no error responses
    assign pready  = 1'b1;
    assign pslverr = 1'b0;

    // APB ACCESS phase handshake detection
    wire apb_write = psel && penable && pwrite;
    wire apb_read  = psel && penable && !pwrite;
    wire transfer_busy = spi_start || spi_busy;

    // The SPI core presents the completed byte with done, one clock before
    // the APB registers capture it. Reads in that cycle see the new byte too.
    wire rx_available = rx_valid || spi_done;
    wire [7:0] rx_read_data = spi_done ? spi_rx_data : rx_data_reg;

    // Sequential register write & flag management
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            spi_en      <= 1'b0;
            spi_cpol    <= 1'b0;
            spi_cpha    <= 1'b0;
            spi_clk_div <= 8'd4;
            spi_tx_data <= 8'd0;
            spi_start   <= 1'b0;
            rx_valid    <= 1'b0;
            rx_data_reg <= 8'd0;
        end else begin
            // Default single-cycle start pulse
            spi_start <= 1'b0;

            // Latch received SPI byte when transfer finishes
            if (spi_done) begin
                rx_data_reg <= spi_rx_data;
                rx_valid    <= 1'b1;
            end

            // Handle APB register writes
            if (apb_write) begin
                case (paddr)
                    ADDR_CTRL: begin
                        spi_en      <= pwdata[0];
                        spi_cpol    <= pwdata[1];
                        spi_cpha    <= pwdata[2];
                        spi_clk_div <= pwdata[15:8];
                    end

                    ADDR_TX_DATA: begin
                        if (spi_en && !transfer_busy) begin
                            spi_tx_data <= pwdata[7:0];
                            spi_start   <= 1'b1;
                            rx_valid    <= 1'b0;
                        end
                    end

                    default: ; // Ignore writes to read-only or unmapped addresses
                endcase
            end

            // Clear rx_valid flag once CPU reads RX_DATA
            if (apb_read && (paddr == ADDR_RX_DATA)) begin
                rx_valid <= 1'b0;
            end
        end
    end

    // Combinational APB read mux
    always @(*) begin
        case (paddr)
            ADDR_CTRL:    prdata = {16'd0, spi_clk_div, 5'd0, spi_cpha, spi_cpol, spi_en};
            ADDR_STATUS:  prdata = {30'd0, rx_available, transfer_busy};
            ADDR_TX_DATA: prdata = {24'd0, spi_tx_data};
            ADDR_RX_DATA: prdata = {24'd0, rx_read_data};
            default:      prdata = 32'd0;
        endcase
    end

endmodule
