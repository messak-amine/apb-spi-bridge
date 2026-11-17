`timescale 1ns/1ps

// ============================================================================
// File: apb_spi_sv_tb.sv
// Description: Layered SystemVerilog OOP verification environment featuring
//              constrained-random stimulus, pin-level SPI monitor,
//              self-checking scoreboard, and functional coverage.
// ============================================================================

// ----------------------------------------------------------------------------
// 1. Transaction Object (Constrained-Random Stimulus Packet)
// ----------------------------------------------------------------------------
class spi_transaction;
    rand bit       cpol;
    rand bit       cpha;
    rand bit [7:0] clk_div;
    rand bit [7:0] master_tx;
    rand bit [7:0] slave_tx;

    // Observed outputs populated by Driver/Monitor
    bit [7:0] observed_mosi;
    bit [7:0] observed_apb_rx;

    // Keep clock divider within fast, realistic simulation bounds
    constraint c_clk_div {
        clk_div inside {[8'd1 : 8'd6]};
    }

    function void display(string tag);
        $display("[%s] Mode=(%0d,%0d) Div=%0d | Master_TX=0x%02h (MOSI=0x%02h) | Slave_TX=0x%02h (APB_RX=0x%02h)",
                 tag, cpol, cpha, clk_div, master_tx, observed_mosi, slave_tx, observed_apb_rx);
    endfunction
endclass

// ----------------------------------------------------------------------------
// 2. Functional Coverage Collector
// ----------------------------------------------------------------------------
class spi_coverage;
    spi_transaction tr;

    covergroup cg_spi;
        cp_cpol: coverpoint tr.cpol;
        cp_cpha: coverpoint tr.cpha;

        // Cross coverage guarantees all 4 SPI modes (0, 1, 2, 3) are exercised
        cx_spi_mode: cross cp_cpol, cp_cpha;

        cp_div: coverpoint tr.clk_div {
            bins fast = {1, 2};
            bins mid  = {3, 4};
            bins slow = {5, 6};
        }

        cp_data: coverpoint tr.master_tx {
            bins all_zeros = {8'h00};
            bins all_ones  = {8'hFF};
            bins alt_bits  = {8'hAA, 8'h55};
            bins others    = default;
        }
    endgroup

    function new();
        cg_spi = new();
    endfunction

    function void sample(spi_transaction t);
        this.tr = t;
        cg_spi.sample();
    endfunction
endclass

// ----------------------------------------------------------------------------
// 3. Generator (Directed Corner Cases + Constrained-Random Transactions)
// ----------------------------------------------------------------------------
class spi_generator;
    mailbox #(spi_transaction) gen2drv;
    int num_random;

    function new(mailbox #(spi_transaction) mbx, int n = 24);
        this.gen2drv    = mbx;
        this.num_random = n;
    endfunction

    task run();
        spi_transaction tr;

        // Directed corner cases: hit 0x00, 0xFF, 0xAA, 0x55 across all 4 modes
        bit [7:0] corner_vals[4] = '{8'h00, 8'hFF, 8'hAA, 8'h55};
        for (int m = 0; m < 4; m++) begin
            tr = new();
            assert(tr.randomize() with {
                cpol      == m[1];
                cpha      == m[0];
                master_tx == corner_vals[m];
            });
            gen2drv.put(tr);
        end

        // Constrained-random transactions
        for (int i = 0; i < num_random; i++) begin
            tr = new();
            assert(tr.randomize());
            gen2drv.put(tr);
        end
    endtask
endclass

// ----------------------------------------------------------------------------
// 4. APB Master Driver + Full-Duplex SPI Slave BFM
// ----------------------------------------------------------------------------
class apb_spi_driver;
    virtual apb_if apb_vif;
    virtual spi_if spi_vif;
    mailbox #(spi_transaction) gen2drv;
    mailbox #(spi_transaction) drv2scb;

    function new(
        virtual apb_if a_vif,
        virtual spi_if s_vif,
        mailbox #(spi_transaction) g2d,
        mailbox #(spi_transaction) d2s
    );
        this.apb_vif = a_vif;
        this.spi_vif = s_vif;
        this.gen2drv = g2d;
        this.drv2scb = d2s;
    endfunction

    task apb_write(input bit [7:0] addr, input bit [31:0] data);
        @(posedge apb_vif.pclk);
        apb_vif.psel    <= 1'b1;
        apb_vif.penable <= 1'b0;
        apb_vif.pwrite  <= 1'b1;
        apb_vif.paddr   <= addr;
        apb_vif.pwdata  <= data;

        @(posedge apb_vif.pclk);
        apb_vif.penable <= 1'b1;
        wait (apb_vif.pready == 1'b1);

        @(posedge apb_vif.pclk);
        apb_vif.psel    <= 1'b0;
        apb_vif.penable <= 1'b0;
        apb_vif.pwrite  <= 1'b0;
    endtask

    task apb_read(input bit [7:0] addr, output bit [31:0] data);
        @(posedge apb_vif.pclk);
        apb_vif.psel    <= 1'b1;
        apb_vif.penable <= 1'b0;
        apb_vif.pwrite  <= 1'b0;
        apb_vif.paddr   <= addr;

        @(posedge apb_vif.pclk);
        apb_vif.penable <= 1'b1;
        #1;
        data = apb_vif.prdata;

        @(posedge apb_vif.pclk);
        apb_vif.psel    <= 1'b0;
        apb_vif.penable <= 1'b0;
    endtask

    // Drives MISO bits serially according to active CPOL/CPHA mode
    task drive_spi_slave(spi_transaction tr);
        bit [7:0] shift_out = tr.slave_tx;
        @(negedge spi_vif.cs_n);

        if (tr.cpha == 1'b0) begin
            spi_vif.miso = shift_out[7];
            for (int i = 7; i >= 0; i--) begin
                if (tr.cpol == 1'b0) @(negedge spi_vif.sclk);
                else                 @(posedge spi_vif.sclk);
                shift_out    = {shift_out[6:0], 1'b0};
                spi_vif.miso = shift_out[7];
            end
        end else begin
            for (int i = 7; i >= 0; i--) begin
                if (tr.cpol == 1'b0) @(posedge spi_vif.sclk);
                else                 @(negedge spi_vif.sclk);
                spi_vif.miso = shift_out[7];
                shift_out    = {shift_out[6:0], 1'b0};
            end
        end
    endtask

    task run();
        spi_transaction tr;
        bit [31:0] ctrl_word;
        bit [31:0] status_word;
        bit [31:0] rx_word;

        forever begin
            gen2drv.get(tr);

            // Launch SPI Slave BFM in parallel with APB register programming
            fork
                drive_spi_slave(tr);
            join_none

            // 1. Write CTRL register
            ctrl_word = {16'd0, tr.clk_div, 5'd0, tr.cpha, tr.cpol, 1'b1};
            apb_write(8'h00, ctrl_word);

            // 2. Write TX_DATA register to trigger SPI transfer
            apb_write(8'h08, {24'd0, tr.master_tx});

            // 3. Poll STATUS[0] (busy) until transfer completes
            status_word = 32'h1;
            while (status_word[0] == 1'b1) begin
                apb_read(8'h04, status_word);
            end

            // 4. Read RX_DATA register
            apb_read(8'h0C, rx_word);
            tr.observed_apb_rx = rx_word[7:0];

            drv2scb.put(tr);
        end
    endtask
endclass

// ----------------------------------------------------------------------------
// 5. Passive Pin-Level SPI Monitor (Samples MOSI directly from physical wires)
// ----------------------------------------------------------------------------
class spi_monitor;
    virtual spi_if spi_vif;
    mailbox #(bit [7:0]) mon2scb;

    function new(virtual spi_if s_vif, mailbox #(bit [7:0]) m2s);
        this.spi_vif = s_vif;
        this.mon2scb = m2s;
    endfunction

    task run();
        bit       idle_clk;
        bit [7:0] sampled_byte;

        forever begin
            @(negedge spi_vif.cs_n);
            idle_clk     = spi_vif.sclk;
            sampled_byte = 8'd0;

            fork
                begin
                    for (int i = 0; i < 8; i++) begin
                        if (idle_clk == 1'b0) @(posedge spi_vif.sclk);
                        else                  @(negedge spi_vif.sclk);
                        sampled_byte = {sampled_byte[6:0], spi_vif.mosi};
                    end
                end
                begin
                    @(posedge spi_vif.cs_n);
                end
            join_any
            disable fork;

            mon2scb.put(sampled_byte);
        end
    endtask
endclass

// ----------------------------------------------------------------------------
// 6. Self-Checking Scoreboard
// ----------------------------------------------------------------------------
class spi_scoreboard;
    mailbox #(spi_transaction) drv2scb;
    mailbox #(bit [7:0])       mon2scb;
    spi_coverage               cov;
    int                        total_tx;
    int                        pass_cnt;
    int                        fail_cnt;

    function new(mailbox #(spi_transaction) d2s, mailbox #(bit [7:0]) m2s);
        this.drv2scb  = d2s;
        this.mon2scb  = m2s;
        this.cov      = new();
        this.total_tx = 0;
        this.pass_cnt = 0;
        this.fail_cnt = 0;
    endfunction

    task run(int expected_count);
        spi_transaction tr;
        bit [7:0]       pin_mosi;

        repeat (expected_count) begin
            drv2scb.get(tr);
            mon2scb.get(pin_mosi);
            tr.observed_mosi = pin_mosi;

            total_tx++;
            cov.sample(tr);

            if (tr.observed_apb_rx !== tr.slave_tx) begin
                $error("[SCB FAIL] MISO mismatch! Expected 0x%02h, Got 0x%02h",
                       tr.slave_tx, tr.observed_apb_rx);
                fail_cnt++;
            end else if ((tr.cpha == 1'b0) && (tr.observed_mosi !== tr.master_tx)) begin
                $error("[SCB FAIL] MOSI mismatch! Expected 0x%02h, Got 0x%02h",
                       tr.master_tx, tr.observed_mosi);
                fail_cnt++;
            end else begin
                pass_cnt++;
                tr.display("SCB PASS");
            end
        end

        $display("======================================================================");
        $display(" SYSTEMVERILOG VERIFICATION SUMMARY");
        $display(" Total Transactions : %0d | Passed: %0d | Failed: %0d",
                 total_tx, pass_cnt, fail_cnt);
        $display(" Functional Coverage: %0.2f%%", cov.cg_spi.get_inst_coverage());
        $display("======================================================================");
    endtask
endclass

// ----------------------------------------------------------------------------
// 7. Top-Level SystemVerilog Testbench Module
// ----------------------------------------------------------------------------
module apb_spi_sv_tb;
    logic pclk;
    logic presetn;

    initial pclk = 1'b0;
    always #5 pclk = ~pclk;

    // Instantiate SystemVerilog Interfaces (includes SVA protocol assertions)
    apb_if u_apb_if (.pclk(pclk), .presetn(presetn));
    spi_if u_spi_if ();

    // Instantiate RTL DUT
    apb_spi_top dut (
        .pclk    (pclk),
        .presetn (presetn),
        .psel    (u_apb_if.psel),
        .penable (u_apb_if.penable),
        .pwrite  (u_apb_if.pwrite),
        .paddr   (u_apb_if.paddr),
        .pwdata  (u_apb_if.pwdata),
        .prdata  (u_apb_if.prdata),
        .pready  (u_apb_if.pready),
        .pslverr (u_apb_if.pslverr),
        .sclk    (u_spi_if.sclk),
        .mosi    (u_spi_if.mosi),
        .miso    (u_spi_if.miso),
        .cs_n    (u_spi_if.cs_n)
    );

    // Verification Components & Mailboxes
    mailbox #(spi_transaction) gen2drv = new();
    mailbox #(spi_transaction) drv2scb = new();
    mailbox #(bit [7:0])       mon2scb = new();

    spi_generator  gen;
    apb_spi_driver drv;
    spi_monitor    mon;
    spi_scoreboard scb;

    localparam int NUM_RANDOM_TX = 24;
    localparam int TOTAL_TX      = 4 + NUM_RANDOM_TX; // 4 corner + 24 random = 28

    initial begin
        // Initialize bus signals cleanly before reset deassertion
        presetn          = 1'b0;
        u_apb_if.psel    = 1'b0;
        u_apb_if.penable = 1'b0;
        u_apb_if.pwrite  = 1'b0;
        u_apb_if.paddr   = 8'd0;
        u_apb_if.pwdata  = 32'd0;
        u_spi_if.miso    = 1'b0;

        #25 presetn = 1'b1;
        #20;

        gen = new(gen2drv, NUM_RANDOM_TX);
        drv = new(u_apb_if, u_spi_if, gen2drv, drv2scb);
        mon = new(u_spi_if, mon2scb);
        scb = new(drv2scb, mon2scb);

        fork
            gen.run();
            drv.run();
            mon.run();
        join_none

        scb.run(TOTAL_TX);
        #50 $finish;
    end

endmodule