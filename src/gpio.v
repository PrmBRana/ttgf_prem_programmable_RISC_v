`default_nettype none

// ============================================================
// GPIO1 -> LED
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
// GPIO2 -> IAM20380 CS_N (Gyroscope)
//
// Fanout fix:
//   spi_busy + spi_pending को fanout धेरै थियो
//   → spi_idle_reg register मा buffer गरियो (already done)
//   → deassert_pending र spi_idle_reg को
//     combined check लाई registered गरियो (नयाँ fix)
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

    // ── Fanout fix: register the idle signal ─────────────────
    // spi_busy र spi_pending दुवै यहाँ buffer हुन्छन्
    // downstream logic मा direct wire जाँदैन
    reg spi_idle_reg;
    always @(posedge clk) begin
        if (reset)
            spi_idle_reg <= 1'b1;
        else
            spi_idle_reg <= ~(spi_busy | spi_pending);
    end

    // ── Fanout fix: pre-register the deassert condition ──────
    // deassert_pending & spi_idle_reg को AND
    // combinational मा गर्दा fanout बढ्छ
    // register मा गरेर 1 cycle early evaluate गरिन्छ
    reg deassert_ok_reg;
    always @(posedge clk) begin
        if (reset)
            deassert_ok_reg <= 1'b0;
        else
            deassert_ok_reg <= deassert_pending & spi_idle_reg;
    end

    // ── CS_N FSM ─────────────────────────────────────────────
    always @(posedge clk) begin
        if (reset) begin
            gpio_out_reg     <= 1'b1;   // CS_N idle HIGH
            deassert_pending <= 1'b0;

        end else begin

            if (wr_en2) begin
                if (wdata2 == 1'b0) begin
                    // Assert CS immediately
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

            end else if (deassert_ok_reg) begin
                // Delayed CS release — uses pre-registered condition
                gpio_out_reg     <= 1'b1;
                deassert_pending <= 1'b0;
            end
        end
    end

    assign gpio_out2 = gpio_out_reg;

endmodule


// ============================================================
// GPIO3 -> MMC5983MA CS_N (Magnetometer)          ← NEW
//
// GPIO2 जस्तै नै — same smart FSM
// Same fanout fix पनि लागू गरिएको छ
// ============================================================

module gpio3_io (
    input  wire clk,
    input  wire reset,
    input  wire wr_en3,
    input  wire wdata3,
    input  wire spi_busy,
    input  wire spi_pending,
    output wire gpio_out3
);

    reg gpio_out_reg;
    reg deassert_pending;

    // ── Fanout fix: register the idle signal ─────────────────
    reg spi_idle_reg;
    always @(posedge clk) begin
        if (reset)
            spi_idle_reg <= 1'b1;
        else
            spi_idle_reg <= ~(spi_busy | spi_pending);
    end

    // ── Fanout fix: pre-register the deassert condition ──────
    reg deassert_ok_reg;
    always @(posedge clk) begin
        if (reset)
            deassert_ok_reg <= 1'b0;
        else
            deassert_ok_reg <= deassert_pending & spi_idle_reg;
    end

    // ── CS_N FSM ─────────────────────────────────────────────
    always @(posedge clk) begin
        if (reset) begin
            gpio_out_reg     <= 1'b1;   // CS_N idle HIGH
            deassert_pending <= 1'b0;

        end else begin

            if (wr_en3) begin
                if (wdata3 == 1'b0) begin
                    // Assert CS immediately
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

            end else if (deassert_ok_reg) begin
                // Delayed CS release — uses pre-registered condition
                gpio_out_reg     <= 1'b1;
                deassert_pending <= 1'b0;
            end
        end
    end

    assign gpio_out3 = gpio_out_reg;

endmodule

`default_nettype wire