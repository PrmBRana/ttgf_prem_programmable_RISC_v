`default_nettype none

module Hazard_Unit (
    input  wire [4:0]  Rs1D,
    input  wire [4:0]  Rs2D,
    input  wire [4:0]  Rs1E,
    input  wire [4:0]  Rs2E,
    input  wire [4:0]  RdE,
    input  wire        RegWriteE,      // kept for future but currently unused
    input  wire [1:0]  ResultSrcE_in,
    input  wire [4:0]  RdM,
    input  wire        RegWriteM,
    input  wire [4:0]  RdW,
    input  wire        RegWriteW,
    input  wire        PCSRCE,

    output reg         StallF,
    output reg         StallD,
    output reg         FlushD,
    output reg         FlushE,
    output reg  [1:0]  Forward_AE,
    output reg  [1:0]  Forward_BE
);

    // =========================================================
    // Forwarding Logic — Early and Simple
    // =========================================================
    wire fwdA_from_M = RegWriteM && (RdM != 5'd0) && (Rs1E == RdM);
    wire fwdA_from_W = RegWriteW && (RdW != 5'd0) && (Rs1E == RdW);

    wire fwdB_from_M = RegWriteM && (RdM != 5'd0) && (Rs2E == RdM);
    wire fwdB_from_W = RegWriteW && (RdW != 5'd0) && (Rs2E == RdW);

    always @(*) begin
        Forward_AE = fwdA_from_M ? 2'b10 :
                     fwdA_from_W ? 2'b01 : 2'b00;

        Forward_BE = fwdB_from_M ? 2'b10 :
                     fwdB_from_W ? 2'b01 : 2'b00;
    end

    // =========================================================
    // Load-Use Hazard
    // =========================================================
    wire rs1_match_E = (Rs1D == RdE) && (RdE != 5'd0);
    wire rs2_match_E = (Rs2D == RdE) && (RdE != 5'd0);

    wire lw_stall = (ResultSrcE_in == 2'b01) && (rs1_match_E || rs2_match_E);

    // =========================================================
    // Stall / Flush — Priority + Reduced Logic Depth
    // =========================================================
    always @(*) begin
        // Default values
        StallF  = 1'b0;
        StallD  = 1'b0;
        FlushD  = 1'b0;
        FlushE  = 1'b0;

        // 1. Highest priority: Branch/Jump taken
        if (PCSRCE) begin
            FlushD = 1'b1;
            FlushE = 1'b1;
            // No stall on taken branch/jump
        end
        // 2. Load-use stall (only if no branch/jump)
        else if (lw_stall) begin
            StallF = 1'b1;
            StallD = 1'b1;
            FlushE = 1'b1;
        end
    end

endmodule

`default_nettype wire