// ============================================================================
// Module: spi_master
// Description: Synthesizable 8-bit SPI Master supporting all 4 CPOL/CPHA modes
//              and programmable clock division.
// ============================================================================
module spi_master (
    input  wire       clk,       // System clock
    input  wire       rst_n,     // Active-low asynchronous reset
    input  wire       start,     // 1-cycle pulse to initiate an 8-bit transfer
    input  wire [7:0] tx_data,   // Byte to transmit over MOSI
    input  wire [7:0] clk_div,   // Half-period clock divider value
    input  wire       cpol,      // Clock polarity (0: idle low, 1: idle high)
    input  wire       cpha,      // Clock phase (0: sample 1st edge, 1: sample 2nd edge)
    output reg  [7:0] rx_data,   // Byte received from MISO
    output reg        busy,      // High while transfer is in progress
    output reg        done,      // 1-cycle pulse when transfer completes

    // External SPI physical pins
    output reg        sclk,      // SPI serial clock
    output reg        mosi,      // Master Out Slave In
    input  wire       miso,      // Master In Slave Out
    output reg        cs_n       // Active-low Chip Select
);

    // FSM state encoding
    localparam [1:0] STATE_IDLE     = 2'd0,
                     STATE_LEADING  = 2'd1,
                     STATE_TRAILING = 2'd2,
                     STATE_DONE     = 2'd3;

    reg [1:0] state;
    reg [7:0] shift_tx;
    reg [7:0] shift_rx;
    reg [2:0] bit_cnt;
    reg [7:0] clk_cnt;
    reg [7:0] active_clk_div;
    reg       active_cpol;
    reg       active_cpha;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= STATE_IDLE;
            shift_tx <= 8'd0;
            shift_rx <= 8'd0;
            bit_cnt  <= 3'd0;
            clk_cnt  <= 8'd0;
            rx_data  <= 8'd0;
            busy     <= 1'b0;
            done     <= 1'b0;
            sclk     <= 1'b0;
            mosi     <= 1'b0;
            cs_n     <= 1'b1;

            active_clk_div <= 8'd0;
            active_cpol    <= 1'b0;
            active_cpha    <= 1'b0;
        end else begin
            // Default single-cycle pulse behavior
            done <= 1'b0;

            case (state)
                // Wait for start trigger; keep SCLK at configured idle polarity
                STATE_IDLE: begin
                    sclk <= cpol;
                    cs_n <= 1'b1;
                    busy <= 1'b0;

                    if (start) begin
                        // Keep this frame's timing stable if CTRL is rewritten.
                        active_clk_div <= clk_div;
                        active_cpol    <= cpol;
                        active_cpha    <= cpha;

                        shift_tx <= tx_data;
                        shift_rx <= 8'd0;
                        bit_cnt  <= 3'd7;
                        clk_cnt  <= 8'd0;
                        cs_n     <= 1'b0;
                        busy     <= 1'b1;
                        // For CPHA=0, MSB must be valid on MOSI before the first clock edge
                        if (!cpha)
                            mosi <= tx_data[7];
                        state <= STATE_LEADING;
                    end
                end

                // First half of the SPI bit period (generates the leading SCLK edge)
                STATE_LEADING: begin
                    if (clk_cnt < active_clk_div) begin
                        clk_cnt <= clk_cnt + 8'd1;
                    end else begin
                        clk_cnt <= 8'd0;
                        sclk    <= ~active_cpol;

                        if (!active_cpha) begin
                            // CPHA=0: Sample MISO on the leading edge
                            shift_rx <= {shift_rx[6:0], miso};
                        end else begin
                            // CPHA=1: Drive MOSI on the leading edge
                            mosi <= shift_tx[7];
                        end
                        state <= STATE_TRAILING;
                    end
                end

                // Second half of the SPI bit period (generates the trailing SCLK edge)
                STATE_TRAILING: begin
                    if (clk_cnt < active_clk_div) begin
                        clk_cnt <= clk_cnt + 8'd1;
                    end else begin
                        clk_cnt <= 8'd0;
                        sclk    <= active_cpol;

                        if (!active_cpha) begin
                            // CPHA=0: Shift next bit onto MOSI on the trailing edge
                            mosi     <= shift_tx[6];
                            shift_tx <= {shift_tx[6:0], 1'b0};
                        end else begin
                            // CPHA=1: Sample MISO and shift TX register on the trailing edge
                            shift_rx <= {shift_rx[6:0], miso};
                            shift_tx <= {shift_tx[6:0], 1'b0};
                        end

                        if (bit_cnt == 3'd0) begin
                            state <= STATE_DONE;
                        end else begin
                            bit_cnt <= bit_cnt - 3'd1;
                            state   <= STATE_LEADING;
                        end
                    end
                end

                // Hold CS_N low for one final half-period before completing transfer
                STATE_DONE: begin
                    if (clk_cnt < active_clk_div) begin
                        clk_cnt <= clk_cnt + 8'd1;
                    end else begin
                        cs_n    <= 1'b1;
                        mosi    <= 1'b0;
                        busy    <= 1'b0;
                        done    <= 1'b1;
                        rx_data <= shift_rx;
                        state   <= STATE_IDLE;
                    end
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end

endmodule
