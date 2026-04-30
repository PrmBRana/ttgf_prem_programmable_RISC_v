`default_nettype none
`timescale 1ns/1ps

/* verilator lint_off TIMESCALEMOD */
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
/* verilator lint_on TIMESCALEMOD */

    // Silence unused signal warnings
    // ui_in[3] = uart1_rx, ui_in[4] = uart2_rx — rest unused
    // uio_in[0] = spi1_miso, uio_in[7] = spi2_miso — rest unused
    // ena — not used in this design
    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused = &{1'b0, ui_in[7:5], ui_in[2:0], uio_in[6:1], ena};
    /* verilator lint_on  UNUSEDSIGNAL */

    // --------------------------------------------------
    // Internal wires
    // --------------------------------------------------
    wire reset;

    wire uart1_tx, uart1_rx;
    wire uart2_tx, uart2_rx;

    wire spi1_clk, spi1_mosi, spi1_miso, spi1_cs_n;
    wire spi2_clk, spi2_mosi, spi2_miso, spi2_cs_n;

    assign reset = ~rst_n;

    // --------------------------------------------------
    // Input assignments
    // --------------------------------------------------
    assign uart1_rx  = ui_in[3];
    assign uart2_rx  = ui_in[4];
    assign spi1_miso = uio_in[0];
    assign spi2_miso = uio_in[7];

    // --------------------------------------------------
    // Output assignments
    // --------------------------------------------------
    assign uo_out[0]   = uart1_tx;
    assign uo_out[1]   = uart2_tx;
    assign uo_out[7:2] = 6'b000000;

    assign uio_out[0] = 1'b0;
    assign uio_out[1] = spi1_mosi;
    assign uio_out[2] = spi1_clk;
    assign uio_out[3] = spi1_cs_n;
    assign uio_out[4] = spi2_mosi;
    assign uio_out[5] = spi2_clk;
    assign uio_out[6] = spi2_cs_n;
    assign uio_out[7] = 1'b0;

    assign uio_oe[0] = 1'b0;
    assign uio_oe[1] = 1'b1;
    assign uio_oe[2] = 1'b1;
    assign uio_oe[3] = 1'b1;
    assign uio_oe[4] = 1'b1;
    assign uio_oe[5] = 1'b1;
    assign uio_oe[6] = 1'b1;
    assign uio_oe[7] = 1'b0;

    // --------------------------------------------------
    // Instantiate the main pipeline module
    // --------------------------------------------------
    pipeline Top_inst (
        .clk(clk),
        .reset(reset),

        .rx(uart1_rx),
        .tx(uart1_tx),

        .UART_tx(uart2_tx),
        .UART_rx_line(uart2_rx),

        .spi1_sclk(spi1_clk),
        .spi1_mosi(spi1_mosi),
        .spi1_miso(spi1_miso),
        .spi1_cs_n(spi1_cs_n),

        .spi2_cs_n(spi2_cs_n),
        .spi2_sclk(spi2_clk),
        .spi2_mosi(spi2_mosi),
        .spi2_miso(spi2_miso)
    );

endmodule

