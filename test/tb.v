`default_nettype none
`timescale 1ns / 1ps

/* This testbench just instantiates the module and makes some convenient wires
   that can be driven / tested by the cocotb test.py.
*/
module tb ();

  // Dump the signals to a FST file. You can view it with gtkwave or surfer.
  initial begin
    $dumpfile("tb.fst");
    $dumpvars(0, tb);
    #1;
  end

  // Wire up the inputs and outputs:
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

  // --------------------------------------------------
   // UART (ONLY ONE)
   // --------------------------------------------------
   reg  rx;
   wire tx;
   // --------------------------------------------------
   // SPI
   // --------------------------------------------------
   reg  spi2_miso;
   wire spi2_mosi;
   wire spi2_sclk;
   wire spi2_cs_n;


  assign ui_in[3] = rx;   // UART RX
  assign uio_in[7] = spi2_miso;
  // --------------------------------------------------
    // Output mapping
    // --------------------------------------------------
    assign tx = uo_out[0];   // UART TX

    assign spi2_mosi = uio_out[2];
    assign spi2_sclk = uio_out[3];
    assign spi2_cs_n = uio_out[4];

    // --------------------------------------------------
    // Clock (50 MHz)
    // --------------------------------------------------
    always #10 clk = ~clk;

    // --------------------------------------------------
    // Initial
    // --------------------------------------------------
    initial begin
        clk = 0;
        rst_n = 0;
        ena = 1;

        rx = 1'b1;          // UART idle (IMPORTANT)
        spi2_miso = 1'b1;

        #100;
        rst_n = 1;
    end

`ifdef GL_TEST
  wire VPWR = 1'b1;
  wire VGND = 1'b0;
`endif

  // Replace tt_um_example with your module name:
  tt_um_prem_pipeline_test dut (

      // Include power ports for the Gate Level test:
`ifdef GL_TEST
      .VPWR(VPWR),
      .VGND(VGND),
`endif

      .ui_in  (ui_in),    // Dedicated inputs
      .uo_out (uo_out),   // Dedicated outputs
      .uio_in (uio_in),   // IOs: Input path
      .uio_out(uio_out),  // IOs: Output path
      .uio_oe (uio_oe),   // IOs: Enable path (active high: 0=input, 1=output)
      .ena    (ena),      // enable - goes high when design is selected
      .clk    (clk),      // clock
      .rst_n  (rst_n)     // not reset
  );

endmodule
