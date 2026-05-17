`default_nettype none

// ============================================================
//  Hazard_Unit — stall, flush, and forwarding control
//
//  Fixed: Branch/Jump flush priority logic (was inverted).
// ============================================================
module Hazard_Unit (
    input  wire [4:0]  Rs1D,
    input  wire [4:0]  Rs2D,
    input  wire [4:0]  Rs1E,
    input  wire [4:0]  Rs2E,
    input  wire [4:0]  RdE,
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

    // ── Forwarding ────────────────────────────────────────────
    always @(*) begin
        Forward_AE = 2'b00;
        if      (RegWriteM && (RdM != 5'b0) && (Rs1E == RdM))
            Forward_AE = 2'b10;
        else if (RegWriteW && (RdW != 5'b0) && (Rs1E == RdW))
            Forward_AE = 2'b01;

        Forward_BE = 2'b00;
        if      (RegWriteM && (RdM != 5'b0) && (Rs2E == RdM))
            Forward_BE = 2'b10;
        else if (RegWriteW && (RdW != 5'b0) && (Rs2E == RdW))
            Forward_BE = 2'b01;
    end

    // ── Load-use stall ────────────────────────────────────────
    wire lw_stall = (ResultSrcE_in == 2'b01) &&
                    (RdE != 5'b0)             &&
                    ((Rs1D == RdE) || (Rs2D == RdE));

    // ── Stall / flush control ─────────────────────────────────
    always @(*) begin
        StallF = 1'b0; 
        StallD = 1'b0;
        FlushD = 1'b0; 
        FlushE = 1'b0;

        if (lw_stall) begin
            StallF = 1'b1;
            StallD = 1'b1;
            FlushE = 1'b1;
        end

        // Branch/jump taken: highest priority (flush older instructions)
        if (PCSRCE) begin
            FlushD = 1'b1;
            FlushE = 1'b1;
            // Do NOT stall when we are taking a branch/jump
            StallF = 1'b0;
            StallD = 1'b0;
        end
    end

endmodule

`default_nettype wire



