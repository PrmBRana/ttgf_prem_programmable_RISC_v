`default_nettype none

module tt_um_prem_pipeline_test (
    input  wire [7:0] ui_in,     // Dedicated inputs
    output wire [7:0] uo_out,    // Dedicated outputs
    input  wire [7:0] uio_in,    // IOs: Input path
    output wire [7:0] uio_out,   // IOs: Output path
    output wire [7:0] uio_oe,    // IOs: Output enable (1=output)
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    // =========================================================
    // Unused signals suppression (Verilator + synthesis clean)
    // =========================================================
    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused = &{
        1'b0,
        ui_in[7:5],     // unused input bits
        ui_in[2:0],     // unused input bits
        uio_in[6:0],    // all except [7] which is SPI2 MISO
        ena             // Tiny Tapeout power enable (always 1)
    };
    /* verilator lint_on UNUSEDSIGNAL */

    wire reset = ~rst_n;

    // ── UART signals ─────────────────────────────────────
    wire BOOT_UART1_RX = ui_in[3];   // Bootloader RX
    wire PER_UART2_RX = ui_in[4];   // Peripheral UART RX
    wire BOOT_UART1_TX;
    wire PER_UART2_TX;

    // ── SPI2 signals ─────────────────────────────────────
    wire SPI_MISO_TOP = uio_in[7];

    // ── GPIO signals ─────────────────────────────────────
    wire GPIO1_TOP;
    wire SPI_CS_GPIO2_TOP;
    wire SPI_MOSI_TOP;
    wire SPI_SCLK_TOP;

    // ── Output assignments ───────────────────────────────
    assign uo_out[0]   = BOOT_UART1_TX;      // Bootloader TX
    assign uo_out[1]   = PER_UART2_TX;      // Peripheral TX
    assign uo_out[7:2] = 6'b000000;     // unused

    // uio_out
    assign uio_out[0] = GPIO1_TOP;    // SPI1 CS_N
    assign uio_out[1] = 1'b0;
    assign uio_out[2] = SPI_MOSI_TOP;    // SPI2 MOSI
    assign uio_out[3] = SPI_SCLK_TOP;    // SPI2 SCLK
    assign uio_out[4] = SPI_CS_GPIO2_TOP;    // SPI2 CS_N
    assign uio_out[7:5] = 3'b000;

    // uio_oe (1 = output)
    assign uio_oe[0] = 1'b1;   // SPI1 CS_N
    assign uio_oe[1] = 1'b0;
    assign uio_oe[2] = 1'b1;   // SPI2 MOSI
    assign uio_oe[3] = 1'b1;   // SPI2 SCLK
    assign uio_oe[4] = 1'b1;   // SPI2 CS_N
    assign uio_oe[7:5] = 3'b000;

    // =========================================================
    // Core instantiation
    // =========================================================
    pipeline Top_inst (
        .clk          (clk),
        .reset        (reset),

        // Bootloader UART
        .rx           (BOOT_UART1_RX),
        .tx           (BOOT_UART1_TX),

        // Peripheral UART
        .UART_tx      (PER_UART2_TX),
        .UART_rx_line (PER_UART2_RX),

        // SPI
        .SPI_SCLK    (SPI_SCLK_TOP),
        .SPI_MOSI    (SPI_MOSI_TOP),
        .SPI_MISO    (SPI_MISO_TOP),
        .SPI_CS_GPIO2 (SPI_CS_GPIO2_TOP),

        // GPIO1
        .Gpio1    (GPIO1_TOP)
    );

endmodule

`default_nettype wire





