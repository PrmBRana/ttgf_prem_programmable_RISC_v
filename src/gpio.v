`default_nettype none

// ============================================================
// GPIO1 -> LED
// Pure register output
// ============================================================

module gpio1_io (
    input  wire clk,
    input  wire reset,
    input  wire wr_en1,
    input  wire wdata1,
    output wire gpio_out1
);

    reg gpio_out_reg;

    always @(posedge clk) begin
        if (reset)
            gpio_out_reg <= 1'b0;   // LED OFF on reset
        else if (wr_en1)
            gpio_out_reg <= wdata1;
    end

    assign gpio_out1 = gpio_out_reg;

endmodule


// ============================================================
// GPIO2 -> SPI CS_N
// Delays CS deassert until SPI becomes idle
// ============================================================

module gpio2_io (
    input  wire clk,
    input  wire reset,
    input  wire wr_en2,
    input  wire wdata2,
    input  wire spi_busy,
    input  wire spi_pending,
    output wire gpio_out2
);

    reg gpio_out_reg;
    reg deassert_pending;
    reg spi_idle_reg;

    // Register SPI idle status
    always @(posedge clk) begin
        if (reset)
            spi_idle_reg <= 1'b1;
        else
            spi_idle_reg <= ~(spi_busy | spi_pending);
    end

    // CS_N control FSM
    always @(posedge clk) begin
        if (reset) begin
            gpio_out_reg     <= 1'b1; // CS inactive
            deassert_pending <= 1'b0;

        end else begin

            // CPU writes GPIO2
            if (wr_en2) begin

                // Assert CS immediately
                if (wdata2 == 1'b0) begin
                    gpio_out_reg     <= 1'b0;
                    deassert_pending <= 1'b0;

                end else begin

                    // Release CS only when SPI idle
                    if (spi_idle_reg) begin
                        gpio_out_reg     <= 1'b1;
                        deassert_pending <= 1'b0;
                    end else begin
                        deassert_pending <= 1'b1;
                    end
                end

            end else if (deassert_pending && spi_idle_reg) begin

                // Delayed CS release
                gpio_out_reg     <= 1'b1;
                deassert_pending <= 1'b0;

            end
        end
    end

    assign gpio_out2 = gpio_out_reg;

endmodule

`default_nettype wire