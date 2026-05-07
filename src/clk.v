`default_nettype none

module pc_register (
    input  wire        clk,
    input  wire        reset,
    input  wire [31:0] PCF_in,
    input  wire        stallF,
    output reg  [31:0] PCF_out
);
    always @(posedge clk) begin
        if (reset)       PCF_out <= 32'd0;
        else if (!stallF) PCF_out <= PCF_in;
    end
endmodule

`default_nettype wire









