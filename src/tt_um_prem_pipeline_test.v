`default_nettype none

module tt_um_prem_pipeline_test (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    // =========================================================
    // Unused signals suppression
    // =========================================================
    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused = &{
        1'b0,
        ui_in[7:2],
        uio_in[7:3],   // only uio[2] used for SPI MISO
        uio_in[1:0],
        ena
    };
    /* verilator lint_on UNUSEDSIGNAL */

    wire reset = ~rst_n;

    // ── UART signals Receiver ─────────────────────────────────────
    wire BOOT_UART1_RX = ui_in[0];
    wire PER_UART2_RX  = ui_in[1];

    //Transmitter
    wire BOOT_UART1_TX;
    wire PER_UART2_TX;

    // ── SPI signals ──────────────────────────────────────
    wire SPI_CS_GPIO2_TOP;
    wire SPI_MOSI_TOP;
    wire SPI_SCLK_TOP;
    wire SPI_MISO_TOP = uio_in[2];

    // ── GPIO ─────────────────────────────────────────────
    wire GPIO1_TOP;

    // ── UART outputs ─────────────────────────────────────
    assign uo_out[0]   = BOOT_UART1_TX;
    assign uo_out[1]   = PER_UART2_TX;
    assign uo_out[7:2] = 6'b000000;

    // =====================================================
    // uio_out mapping
    // =====================================================

    assign uio_out[0] = SPI_CS_GPIO2_TOP;
    assign uio_out[1] = SPI_MOSI_TOP;
    assign uio_out[2] = 1'b0;              // MISO input
    assign uio_out[3] = SPI_SCLK_TOP;
    assign uio_out[4] = GPIO1_TOP;
    assign uio_out[7:5] = 3'b000;

    // =====================================================
    // uio_oe
    // =====================================================

    assign uio_oe[0] = 1'b1; // CS
    assign uio_oe[1] = 1'b1; // MOSI
    assign uio_oe[2] = 1'b0; // MISO input
    assign uio_oe[3] = 1'b1; // SCLK
    assign uio_oe[4] = 1'b1; // GPIO1
    assign uio_oe[7:5] = 3'b000;

    // =====================================================
    // Core
    // =====================================================
    pipeline Top_inst (
        .clk          (clk),
        .reset        (reset),

        .rx           (BOOT_UART1_RX),
        .tx           (BOOT_UART1_TX),

        .UART_tx      (PER_UART2_TX),
        .UART_rx_line (PER_UART2_RX),

        .SPI_SCLK     (SPI_SCLK_TOP),
        .SPI_MOSI     (SPI_MOSI_TOP),
        .SPI_MISO     (SPI_MISO_TOP),
        .SPI_CS_GPIO2 (SPI_CS_GPIO2_TOP),

        .Gpio1        (GPIO1_TOP)
    );

endmodule

`default_nettype wire