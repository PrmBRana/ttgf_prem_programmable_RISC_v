`default_nettype none
`timescale 1ns/1ps

module tb();

    initial begin
        $dumpfile("tb.vcd");
        $dumpvars(0, tb);
    end

    // --------------------------------------------------
    // Clock / Reset
    // --------------------------------------------------
    reg clk;
    reg rst_n;
    reg ena;

    // --------------------------------------------------
    // UART
    // --------------------------------------------------
    reg  rx;             // bootloader RX  → ui_in[3]
    reg  UART_rx_line;   // peripheral RX  → ui_in[4]
    wire tx;             // bootloader TX  ← uo_out[0]
    wire UART_tx;        // peripheral TX  ← uo_out[1]

    // --------------------------------------------------
    // SPI1  (driven/read via uio_*)
    // --------------------------------------------------
    reg  spi1_miso;      // → uio_in[0]
    wire spi1_mosi;      // ← uio_out[1]
    wire spi1_sclk;      // ← uio_out[2]
    wire spi1_cs_n;      // ← uio_out[3]

    // --------------------------------------------------
    // SPI2  (driven/read via uio_*)
    // --------------------------------------------------
    reg  spi2_miso;      // → uio_in[7]
    wire spi2_mosi;      // ← uio_out[4]
    wire spi2_sclk;      // ← uio_out[5]
    wire spi2_cs_n;      // ← uio_out[6]

    // --------------------------------------------------
    // IO buses
    // --------------------------------------------------
    wire [7:0] ui_in;
    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;
    reg  [7:0] uio_in;

    // ── ui_in: only UART RX lines are driven by testbench ──
    // ui_in[3] = uart1_rx (bootloader)
    // ui_in[4] = uart2_rx (peripheral)
    // all other bits are unused inputs → tie low
    assign ui_in[2:0]  = 3'b000;
    assign ui_in[3]    = rx;
    assign ui_in[4]    = UART_rx_line;
    assign ui_in[7:5]  = 3'b000;

    // ── uio_in: SPI MISO lines ──
    // uio_in[0] = spi1_miso
    // uio_in[7] = spi2_miso
    always @(*) begin
        uio_in        = 8'b0;
        uio_in[0]     = spi1_miso;
        uio_in[7]     = spi2_miso;
    end

    // ── uo_out → UART TX ──
    assign tx       = uo_out[0];   // bootloader TX
    assign UART_tx  = uo_out[1];   // peripheral  TX

    // ── uio_out → SPI outputs ──
    assign spi1_mosi = uio_out[1];
    assign spi1_sclk = uio_out[2];
    assign spi1_cs_n = uio_out[3];
    assign spi2_mosi = uio_out[4];
    assign spi2_sclk = uio_out[5];
    assign spi2_cs_n = uio_out[6];

    // --------------------------------------------------
    // Initial values
    // --------------------------------------------------
    initial begin
        clk          = 0;
        rst_n        = 0;
        ena          = 1;
        rx           = 1'b1;   // UART idle high
        UART_rx_line = 1'b1;   // UART idle high
        spi1_miso    = 1'b1;
        spi2_miso    = 1'b1;
    end

    // --------------------------------------------------
    // Clock — 50 MHz  (period = 20 ns)
    // --------------------------------------------------
    always #10 clk = ~clk;

`ifdef GL_TEST
    wire VPWR = 1'b1;
    wire VGND = 1'b0;
`endif

    // --------------------------------------------------
    // DUT
    // --------------------------------------------------
    tt_um_prem_pipeline_test tt_um_prem_pipeline_test (
`ifdef GL_TEST
        .VPWR(VPWR),
        .VGND(VGND),
`endif
        .ui_in   (ui_in),
        .uo_out  (uo_out),
        .uio_in  (uio_in),
        .uio_out (uio_out),
        .uio_oe  (uio_oe),
        .ena     (ena),
        .clk     (clk),
        .rst_n   (rst_n)
    );

endmodule







