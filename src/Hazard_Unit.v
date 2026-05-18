`default_nettype none

module Hazard_Unit (
    input  wire [4:0] Rs1D, Rs2D,
    input  wire [4:0] Rs1E, Rs2E,
    input  wire [4:0] RdE, RdM, RdW,
    input  wire       RegWriteM,
    input  wire       RegWriteW,
    input  wire       PCSRCE,
    input  wire [1:0] ResultSrcE_in,

    output reg  [1:0] Forward_AE,
    output reg  [1:0] Forward_BE,
    output reg        StallF,
    output reg        StallD,
    output reg        FlushD,
    output reg        FlushE
);

    wire use_M_A = RegWriteM && (RdM != 5'd0);
    wire use_W_A = RegWriteW && (RdW != 5'd0);

    wire A_from_M = use_M_A && (Rs1E == RdM);
    wire A_from_W = use_W_A && (Rs1E == RdW);

    wire B_from_M = use_M_A && (Rs2E == RdM);
    wire B_from_W = use_W_A && (Rs2E == RdW);

    always @(*) begin
        Forward_AE = 2'b00;
        if (A_from_M) Forward_AE = 2'b10;
        else if (A_from_W) Forward_AE = 2'b01;

        Forward_BE = 2'b00;
        if (B_from_M) Forward_BE = 2'b10;
        else if (B_from_W) Forward_BE = 2'b01;

        StallF = 1'b0;
        StallD = 1'b0;
        FlushD = 1'b0;
        FlushE = 1'b0;

        // load-use stall
        if ((ResultSrcE_in == 2'b01) &&
            (RdE != 5'd0) &&
            ((Rs1D == RdE) || (Rs2D == RdE))) begin

            StallF = 1'b1;
            StallD = 1'b1;
            FlushE = 1'b1;
        end

        // branch flush priority
        if (PCSRCE) begin
            FlushD = 1'b1;
            FlushE = 1'b1;
            StallF = 1'b0;
            StallD = 1'b0;
        end
    end

endmodule

`default_nettype wire




