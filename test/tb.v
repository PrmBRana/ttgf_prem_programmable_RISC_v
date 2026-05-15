`default_nettype none
`timescale 1ns/1ps



module tb();

    initial begin
        $dumpfile("tb.vcd");
        $dumpvars(0, tb);
    end

    // ── Clock / Reset ─────────────────────────────────────────
    reg clk;
    reg rst_n;
    reg ena;

    // ── UART ─────────────────────────────────────────────────
    reg  rx;              // ui_in[3]  bootloader RX
    reg  UART_rx_line;   // ui_in[4]  peripheral RX
    wire tx;              // uo_out[0] bootloader TX
    wire UART_tx;        // uo_out[1] peripheral TX

    // ── SPI2 — named to match test.py exactly ────────────────
    reg  spi2_miso;      // uio_in[7]   driven by test.py slave
    wire spi2_mosi;      // uio_out[2]  driven by DUT
    wire spi2_sclk;      // uio_out[3]  driven by DUT
    wire spi2_cs_n;      // uio_out[4]  driven by DUT (gpio2 CS)
    wire spi1_cs_n;      // uio_out[0]  driven by DUT (gpio1 CS)

    // ── IO buses ──────────────────────────────────────────────
    reg  [7:0] uio_in;
    wire [7:0] ui_in;
    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;

    // ── ui_in assembly ────────────────────────────────────────
    assign ui_in[2:0] = 3'b000;
    assign ui_in[3]   = rx;
    assign ui_in[4]   = UART_rx_line;
    assign ui_in[7:5] = 3'b000;

    // ── uio_in: SPI MISO on bit 7 ────────────────────────────
    always @(*) begin
        uio_in    = 8'b0;
        uio_in[7] = spi2_miso;
    end

    // ── uo_out → UART TX ──────────────────────────────────────
    assign tx      = uo_out[0];
    assign UART_tx = uo_out[1];

    // ── uio_out → SPI2 + CS signals ──────────────────────────
    assign spi1_cs_n = uio_out[0];   // gpio1 CS
    assign spi2_mosi = uio_out[2];   // SPI2 MOSI
    assign spi2_sclk = uio_out[3];   // SPI2 SCLK
    assign spi2_cs_n = uio_out[4];   // SPI2 CS  ← matches test.py dut.spi2_cs_n

    // ── Initial values ────────────────────────────────────────
    initial begin
        clk          = 1'b0;
        rst_n        = 1'b0;
        ena          = 1'b1;
        rx           = 1'b1;   // UART idle high
        UART_rx_line = 1'b1;   // UART idle high
        spi2_miso    = 1'b1;   // SPI MISO idle high
    end

    // 50 MHz clock
    always #10 clk = ~clk;

`ifdef GL_TEST
    wire VPWR = 1'b1;
    wire VGND = 1'b0;
`endif

    // ── DUT ───────────────────────────────────────────────────
    tt_um_prem_pipeline_test dut (
`ifdef GL_TEST
        .VPWR  (VPWR),
        .VGND  (VGND),
`endif
        .ui_in  (ui_in),
        .uo_out (uo_out),
        .uio_in (uio_in),
        .uio_out(uio_out),
        .uio_oe (uio_oe),
        .ena    (ena),
        .clk    (clk),
        .rst_n  (rst_n)
    );

endmodule







