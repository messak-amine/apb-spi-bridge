// ============================================================================
// File: apb_spi_if.sv
// Description: SystemVerilog interfaces for AMBA APB3 and SPI buses,
//              including concurrent SystemVerilog Assertions (SVA) for
//              AMBA APB protocol compliance checking.
// ============================================================================

interface apb_if (input logic pclk, input logic presetn);
    logic        psel;
    logic        penable;
    logic        pwrite;
    logic [7:0]  paddr;
    logic [31:0] pwdata;
    logic [31:0] prdata;
    logic        pready;
    logic        pslverr;

    // ------------------------------------------------------------------------
    // Concurrent SystemVerilog Assertions (SVA) - AMBA APB Protocol Rules
    // ------------------------------------------------------------------------

    // Rule 1: SETUP phase (psel && !penable) must transition to ACCESS phase
    //         (psel && penable) on the very next clock cycle.
    property p_setup_to_access;
        @(posedge pclk) disable iff (!presetn)
        (psel && !penable) |=> (psel && penable);
    endproperty
    assert property (p_setup_to_access)
        else $error("[SVA ERROR] APB SETUP phase failed to transition to ACCESS phase.");

    // Rule 2: Address, direction, and write data must remain stable between
    //         the SETUP phase and the ACCESS phase.
    property p_signal_stability;
        @(posedge pclk) disable iff (!presetn)
        (psel && !penable) |=> ($stable(paddr) && $stable(pwrite) && (!pwrite || $stable(pwdata)));
    endproperty
    assert property (p_signal_stability)
        else $error("[SVA ERROR] APB address/control/pwdata changed during ACCESS phase.");

    // Rule 3: Control signals must never be X/Z after reset is deasserted.
    property p_no_unknown_ctrl;
        @(posedge pclk) disable iff (!presetn)
        !$isunknown({psel, penable, pwrite});
    endproperty
    assert property (p_no_unknown_ctrl)
        else $error("[SVA ERROR] Unknown (X/Z) state detected on APB control signals.");

endinterface

interface spi_if;
    logic sclk;
    logic mosi;
    logic miso;
    logic cs_n;
endinterface