`default_nettype none
`timescale 1ns/1ps

module tb();

    initial begin
        $dumpfile("tb.vcd");
        $dumpvars(0, tb);
    end

    // ── Clock / Reset ─────────────────────────────
    reg clk;
    reg rst_n;
    reg ena;

    // ── UART ──────────────────────────────────────
    reg  rx;
    reg  UART_rx_line;

    wire tx;
    wire UART_tx;

    // ── SPI ───────────────────────────────────────
    reg  spi2_miso;

    wire spi2_mosi;
    wire spi2_sclk;
    wire spi2_cs_n;

    wire gpio1;

    // ── IO buses ──────────────────────────────────
    reg  [7:0] uio_in;
    wire [7:0] ui_in;
    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;

    // ── UI input mapping (FIXED) ─────────────────
    assign ui_in[0]   = rx;
    assign ui_in[1]   = UART_rx_line;
    assign ui_in[7:2] = 6'b000000;

    // ── UIO input mapping ────────────────────────
    always @(*) begin
        uio_in    = 8'b0;
        uio_in[2] = spi2_miso;
    end

    // ── UART outputs ─────────────────────────────
    assign tx      = uo_out[0];
    assign UART_tx = uo_out[1];

    // ── SPI outputs ──────────────────────────────
    assign spi2_cs_n = uio_out[0];
    assign spi2_mosi = uio_out[1];
    assign spi2_sclk = uio_out[3];

    // ── GPIO ─────────────────────────────────────
    assign gpio1 = uio_out[4];

    // ── Clock ────────────────────────────────────
    always #10 clk = ~clk;

    // ── Reset + init ─────────────────────────────
    initial begin
        clk   = 0;
        ena   = 1;

        rst_n = 0;
        #100;
        rst_n = 1;

        rx           = 1;
        UART_rx_line = 1;
        spi2_miso    = 1;
    end

`ifdef GL_TEST
    wire VPWR = 1'b1;
    wire VGND = 1'b0;
`endif

    // ── DUT ──────────────────────────────────────
    tt_um_prem_pipeline_test dut (
`ifdef GL_TEST
        .VPWR   (VPWR),
        .VGND   (VGND),
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

`default_nettype wire