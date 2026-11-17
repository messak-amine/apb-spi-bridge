`timescale 1ns/1ps

// ============================================================================
// Testbench: tb_apb_spi
// Description: Self-checking AMBA APB-to-SPI verification testbench testing
//              full-duplex transfers across all 4 SPI modes (CPOL/CPHA).
// ============================================================================
module tb_apb_spi;

    // APB Bus Signals
    reg         pclk;
    reg         presetn;
    reg         psel;
    reg         penable;
    reg         pwrite;
    reg  [7:0]  paddr;
    reg  [31:0] pwdata;
    wire [31:0] prdata;
    wire        pready;
    wire        pslverr;

    // External SPI Signals
    wire        sclk;
    wire        mosi;
    reg         miso;
    wire        cs_n;

    // Instantiate Top-Level APB-to-SPI Bridge (DUT)
    apb_spi_top dut (
        .pclk    (pclk),
        .presetn (presetn),
        .psel    (psel),
        .penable (penable),
        .pwrite  (pwrite),
        .paddr   (paddr),
        .pwdata  (pwdata),
        .prdata  (prdata),
        .pready  (pready),
        .pslverr (pslverr),
        .sclk    (sclk),
        .mosi    (mosi),
        .miso    (miso),
        .cs_n    (cs_n)
    );

    // 100 MHz APB System Clock (10 ns period)
    initial pclk = 1'b0;
    always #5 pclk = ~pclk;

    // ------------------------------------------------------------------------
    // Behavioral SPI Slave Model (Full-Duplex, supports CPOL/CPHA 0..3)
    // ------------------------------------------------------------------------
    reg [7:0] slave_tx_byte;
    reg [7:0] slave_tx_shift;
    reg [7:0] slave_rx_shift;
    reg       cpol_cfg;
    reg       cpha_cfg;

    wire sclk_leading = (cpol_cfg == 1'b0) ? sclk : ~sclk;

    always @(negedge cs_n) begin
        slave_tx_shift = slave_tx_byte;
        slave_rx_shift = 8'd0;
        if (cpha_cfg == 1'b0)
            miso = slave_tx_byte[7];
    end

    always @(posedge sclk_leading) begin
        if (!cs_n) begin
            if (cpha_cfg == 1'b0) begin
                slave_rx_shift = {slave_rx_shift[6:0], mosi};
            end else begin
                miso = slave_tx_shift[7];
            end
        end
    end

    always @(negedge sclk_leading) begin
        if (!cs_n) begin
            if (cpha_cfg == 1'b0) begin
                slave_tx_shift = {slave_tx_shift[6:0], 1'b0};
                miso           = slave_tx_shift[7];
            end else begin
                slave_rx_shift = {slave_rx_shift[6:0], mosi};
                slave_tx_shift = {slave_tx_shift[6:0], 1'b0};
            end
        end
    end

    // ------------------------------------------------------------------------
    // AMBA APB Master Bus Functional Model (Write & Read Tasks)
    // ------------------------------------------------------------------------
    task apb_write(input [7:0] addr, input [31:0] data);
        begin
            @(posedge pclk);
            psel    <= 1'b1;
            penable <= 1'b0;
            pwrite  <= 1'b1;
            paddr   <= addr;
            pwdata  <= data;

            @(posedge pclk);
            penable <= 1'b1;
            wait (pready == 1'b1);

            @(posedge pclk);
            psel    <= 1'b0;
            penable <= 1'b0;
            pwrite  <= 1'b0;
        end
    endtask

    task apb_read(input [7:0] addr, output [31:0] data);
        begin
            @(posedge pclk);
            psel    <= 1'b1;
            penable <= 1'b0;
            pwrite  <= 1'b0;
            paddr   <= addr;

            @(posedge pclk);
            penable <= 1'b1;
            #1;
            data = prdata;

            @(posedge pclk);
            psel    <= 1'b0;
            penable <= 1'b0;
        end
    endtask

    // ------------------------------------------------------------------------
    // Self-Checking SPI Transfer Test Task
    // ------------------------------------------------------------------------
    integer errors;
    reg [31:0] status_reg;
    reg [31:0] rx_reg;

    task run_spi_test(
        input       cpol,
        input       cpha,
        input [7:0] div,
        input [7:0] master_tx,
        input [7:0] slave_tx
    );
        reg [31:0] ctrl_word;
        begin
            cpol_cfg      = cpol;
            cpha_cfg      = cpha;
            slave_tx_byte = slave_tx;

            // Configure CTRL register: [0]=spi_en, [1]=cpol, [2]=cpha, [15:8]=clk_div
            ctrl_word = {16'd0, div, 5'd0, cpha, cpol, 1'b1};
            apb_write(8'h00, ctrl_word);

            // Write payload to TX_DATA register (triggers SPI transfer)
            apb_write(8'h08, {24'd0, master_tx});

            // Poll STATUS register until busy (bit 0) clears
            status_reg = 32'h1;
            while (status_reg[0] == 1'b1) begin
                apb_read(8'h04, status_reg);
            end

            // Read received byte from RX_DATA register
            apb_read(8'h0C, rx_reg);

            // Verify MOSI (Master -> Slave) and MISO (Slave -> Master)
            if (slave_rx_shift !== master_tx) begin
                $display("[FAIL] Mode (%0d,%0d) MOSI mismatch: Expected 0x%02h, Got 0x%02h",
                         cpol, cpha, master_tx, slave_rx_shift);
                errors = errors + 1;
            end else if (rx_reg[7:0] !== slave_tx) begin
                $display("[FAIL] Mode (%0d,%0d) MISO mismatch: Expected 0x%02h, Got 0x%02h",
                         cpol, cpha, slave_tx, rx_reg[7:0]);
                errors = errors + 1;
            end else begin
                $display("[PASS] Mode (%0d,%0d) | TX=0x%02h -> Slave RX=0x%02h | Slave TX=0x%02h -> APB RX=0x%02h",
                         cpol, cpha, master_tx, slave_rx_shift, slave_tx, rx_reg[7:0]);
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // Main Stimulus Sequence
    // ------------------------------------------------------------------------
    initial begin
        $dumpfile("apb_spi.vcd");
        $dumpvars(0, tb_apb_spi);

        // Initialize signals
        presetn       = 1'b0;
        psel          = 1'b0;
        penable       = 1'b0;
        pwrite        = 1'b0;
        paddr         = 8'd0;
        pwdata        = 32'd0;
        miso          = 1'b0;
        cpol_cfg      = 1'b0;
        cpha_cfg      = 1'b0;
        slave_tx_byte = 8'd0;
        errors        = 0;

        // Release reset
        #25 presetn = 1'b1;
        #20;

        $display("======================================================================");
        $display(" Starting AMBA APB-to-SPI Bridge Functional Tests (Modes 0 to 3)");
        $display("======================================================================");

        run_spi_test(1'b0, 1'b0, 8'd2, 8'hA5, 8'h5A); // SPI Mode 0
        run_spi_test(1'b0, 1'b1, 8'd3, 8'h3C, 8'hC3); // SPI Mode 1
        run_spi_test(1'b1, 1'b0, 8'd2, 8'hF0, 8'h0F); // SPI Mode 2
        run_spi_test(1'b1, 1'b1, 8'd4, 8'h96, 8'h69); // SPI Mode 3

        $display("======================================================================");
        if (errors == 0)
            $display(" RESULT: ALL TESTS PASSED (0 errors)");
        else
            $display(" RESULT: FAILED (%0d errors)", errors);
        $display("======================================================================");

        #50 $finish;
    end

endmodule