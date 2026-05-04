`default_nettype none
`timescale 1ns/1ps

// ============================================================
//  tt_um_prem_pipeline_test — Tiny Tapeout top wrapper
//
//  Fixes vs previous version:
//  ✓ pipeline port names corrected to match pipeline.v:
//      spi_sclk  → spi2_sclk
//      spi_mosi  → spi2_mosi
//      spi_miso  → spi2_miso
//      gpio1_n   → spi1_cs_n
//      gpio2_n   → spi2_cs_n
//  ✓ uio_oe double-assign removed (was assigning 8'b0 then
//    individual bits — the 8'b0 clobbered the bit assigns)
// ============================================================

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

    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused = &{1'b0, ui_in[7:5], ui_in[2:0], uio_in[6:1], ena};
    /* verilator lint_on  UNUSEDSIGNAL */

    wire reset = ~rst_n;

    // ── UART ─────────────────────────────────────────────────
    wire uart1_rx = ui_in[3];   // ui[3]  bootloader RX
    wire uart2_rx = ui_in[4];   // ui[4]  peripheral RX
    wire uart1_tx;              // uo[0]  bootloader TX
    wire uart2_tx;              // uo[1]  peripheral TX

    // ── SPI2 ─────────────────────────────────────────────────
    wire spi2_mosi_w;           // uio[2] output
    wire spi2_sclk_w;           // uio[3] output
    wire spi2_miso_w = uio_in[7]; // uio[7] input

    // ── GPIO chip selects ─────────────────────────────────────
    wire spi1_cs_n_w;           // uio[0] SPI1 CS (gpio1)
    wire spi2_cs_n_w;           // uio[4] SPI2 CS (gpio2)

    // ── Output assignments ────────────────────────────────────
    assign uo_out[0]   = uart1_tx;
    assign uo_out[1]   = uart2_tx;
    assign uo_out[7:2] = 6'b000000;

    // uio_out: driven pins only; unused set to 0
    assign uio_out[0] = spi1_cs_n_w;   // uio[0] gpio1 / SPI1 CS
    assign uio_out[1] = 1'b0;
    assign uio_out[2] = spi2_mosi_w;   // uio[2] SPI2 MOSI
    assign uio_out[3] = spi2_sclk_w;   // uio[3] SPI2 SCLK
    assign uio_out[4] = spi2_cs_n_w;   // uio[4] SPI2 CS
    assign uio_out[5] = 1'b0;
    assign uio_out[6] = 1'b0;
    assign uio_out[7] = 1'b0;          // uio[7] is SPI MISO input

    // uio_oe: 1 = output, 0 = input
    assign uio_oe[0] = 1'b1;   // spi1_cs_n  output
    assign uio_oe[1] = 1'b0;   // unused
    assign uio_oe[2] = 1'b1;   // spi2_mosi  output
    assign uio_oe[3] = 1'b1;   // spi2_sclk  output
    assign uio_oe[4] = 1'b1;   // spi2_cs_n  output
    assign uio_oe[5] = 1'b0;   // unused
    assign uio_oe[6] = 1'b0;   // unused
    assign uio_oe[7] = 1'b0;   // spi2_miso  input

    // ── pipeline instantiation ────────────────────────────────
    pipeline Top_inst (
        .clk          (clk),
        .reset        (reset),

        // Bootloader UART
        .rx           (uart1_rx),
        .tx           (uart1_tx),

        // Peripheral UART
        .UART_tx      (uart2_tx),
        .UART_rx_line (uart2_rx),

        // SPI2 — use pipeline's exact port names
        .spi2_sclk    (spi2_sclk_w),
        .spi2_mosi    (spi2_mosi_w),
        .spi2_miso    (spi2_miso_w),
        .spi2_cs_n    (spi2_cs_n_w),

        // SPI1 CS (GPIO1-driven)
        .spi1_cs_n    (spi1_cs_n_w)
    );

endmodule






