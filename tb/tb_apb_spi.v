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
    time      half_period;
    time      last_spi_edge;
    integer   spi_edge_count;

    wire sclk_leading = (cpol_cfg == 1'b0) ? sclk : ~sclk;

    always @(negedge cs_n) begin
        if (sclk !== cpol_cfg)
            $fatal(1, "SCLK was not idle when CS_N asserted");
        last_spi_edge  = $time;
        spi_edge_count = 0;
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

    always @(sclk) begin
        if (!cs_n) begin
            if ($time - last_spi_edge != half_period)
                $fatal(1, "Expected SCLK half-period of %0d ns, got %0d ns",
                       half_period, $time - last_spi_edge);
            last_spi_edge  = $time;
            spi_edge_count = spi_edge_count + 1;
        end
    end

    always @(posedge cs_n) begin
        if (presetn) begin
            if (spi_edge_count != 16 || sclk !== cpol_cfg)
                $fatal(1, "Expected 16 SCLK edges and idle polarity at CS_N release");
            if ($time - last_spi_edge != half_period)
                $fatal(1, "Incorrect CS_N hold time after the last SCLK edge");
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

            @(posedge pclk);
            if ({pready, pslverr} !== 2'b10)
                $fatal(1, "APB write to 0x%02h did not complete without errors or wait states", addr);
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

            @(posedge pclk);
            if ({pready, pslverr} !== 2'b10)
                $fatal(1, "APB read from 0x%02h did not complete without errors or wait states", addr);
            data = prdata;
            psel    <= 1'b0;
            penable <= 1'b0;
        end
    endtask

    task expect_read(input [7:0] addr, input [31:0] expected);
        reg [31:0] actual;
        begin
            apb_read(addr, actual);
            if (actual !== expected)
                $fatal(1, "APB read from 0x%02h: expected 0x%08h, got 0x%08h", addr, expected, actual);
        end
    endtask

    // ------------------------------------------------------------------------
    // Self-Checking SPI Transfer Test Task
    // ------------------------------------------------------------------------
    task automatic run_spi_test(
        input       cpol,
        input       cpha,
        input [7:0] div,
        input [7:0] master_tx,
        input [7:0] slave_tx,
        input       reconfigure,
        input       read_rx_at_completion
    );
        reg [31:0] ctrl_word;
        begin
            cpol_cfg      = cpol;
            cpha_cfg      = cpha;
            slave_tx_byte = slave_tx;
            half_period   = (32'd1 + div) * 10;

            // Configure CTRL register: [0]=spi_en, [1]=cpol, [2]=cpha, [15:8]=clk_div
            ctrl_word = {16'd0, div, 5'd0, cpha, cpol, 1'b1};
            apb_write(8'h00, ctrl_word);
            expect_read(8'h00, ctrl_word);

            fork
                begin
                    apb_write(8'h08, {24'd0, master_tx});
                    expect_read(8'h04, 32'h1);
                    if (reconfigure) begin
                        // Change all SPI settings during the active mode 0 frame.
                        apb_write(8'h00, 32'h0000_0107);
                        expect_read(8'h00, 32'h0000_0107);
                        apb_write(8'h08, {24'd0, ~master_tx});
                        expect_read(8'h08, {24'd0, master_tx});
                    end
                end
                begin
                    @(negedge cs_n);
                    // The frame lasts 17 half-periods. Complete the APB read
                    // on the first PCLK edge after CS_N rises.
                    repeat (17 * (32'd1 + div) - 2) @(posedge pclk);
                    if (read_rx_at_completion)
                        expect_read(8'h0C, {24'd0, slave_tx});
                    else
                        expect_read(8'h04, 32'h2);
                end
            join

            if (!read_rx_at_completion) begin
                expect_read(8'h8C, 32'd0);
                expect_read(8'h04, 32'h2);
                expect_read(8'h0C, {24'd0, slave_tx});
            end
            expect_read(8'h04, 32'd0);

            if (reconfigure && sclk !== 1'b1)
                $fatal(1, "SCLK did not switch to the new idle polarity after the frame");
            if (slave_rx_shift !== master_tx)
                $fatal(1, "SPI mode (%0d,%0d): expected MOSI byte 0x%02h, got 0x%02h",
                       cpol, cpha, master_tx, slave_rx_shift);
            $display("[PASS] SPI mode (%0d,%0d), divider %0d, MOSI 0x%02h, MISO 0x%02h",
                     cpol, cpha, div, master_tx, slave_tx);
        end
    endtask

    // ------------------------------------------------------------------------
    // Main Stimulus Sequence
    // ------------------------------------------------------------------------
    initial #100000 $fatal(1, "Testbench timed out");

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

        // Release reset
        #25 presetn = 1'b1;
        #20;

        expect_read(8'h00, 32'h0000_0400);
        expect_read(8'h04, 32'd0);
        expect_read(8'h0C, 32'd0);
        apb_write(8'h08, 32'hFF); // Disabled TX write
        apb_write(8'h10, 32'h01); // Upper address bits must prevent a CTRL write
        expect_read(8'h08, 32'd0);
        expect_read(8'h00, 32'h0000_0400);
        expect_read(8'h10, 32'd0);
        apb_write(8'h00, 32'h0000_0401);
        apb_write(8'h18, 32'hFF);
        apb_write(8'h09, 32'hFF);
        expect_read(8'h08, 32'd0);
        expect_read(8'h04, 32'd0);
        $display("[PASS] APB reset, disabled TX, and unmapped addresses");

        run_spi_test(1'b0, 1'b0, 8'd2,   8'hA5, 8'h5A, 1'b0, 1'b0);
        run_spi_test(1'b0, 1'b1, 8'd3,   8'h3C, 8'hC3, 1'b0, 1'b0);
        run_spi_test(1'b1, 1'b0, 8'd2,   8'hF0, 8'h0F, 1'b0, 1'b0);
        run_spi_test(1'b1, 1'b1, 8'd4,   8'h96, 8'h69, 1'b0, 1'b0);
        run_spi_test(1'b0, 1'b0, 8'd2,   8'h81, 8'h7E, 1'b1, 1'b0); // Rewrite CTRL while busy
        run_spi_test(1'b1, 1'b1, 8'd0,   8'h57, 8'hE8, 1'b0, 1'b1); // Read RX at completion
        run_spi_test(1'b0, 1'b1, 8'd255, 8'h01, 8'h80, 1'b0, 1'b0);

        $display("RESULT: ALL TESTS PASSED");
        $finish;
    end

endmodule
