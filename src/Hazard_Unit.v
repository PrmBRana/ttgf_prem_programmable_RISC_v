`default_nettype none
`timescale 1ns / 1ps

module Hazard_Unit (
    input  wire [4:0]  Rs1D,
    input  wire [4:0]  Rs2D,
    input  wire [4:0]  Rs1E,
    input  wire [4:0]  Rs2E,
    input  wire [4:0]  RdE,
    input  wire        RegWriteE,
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

    // ── MEM-stage forwarding ──────────────────────────────────
    // Forward from MEM if:
    //   - RegWriteM is asserted (a real write is happening)
    //   - RdM is not x0
    //   - RdM matches the EX-stage source register
    always @(*) begin
        Forward_AE = 2'b00;
        Forward_BE = 2'b00;

        if (RegWriteM && (RdM != 5'd0) && (RdM == Rs1E))
            Forward_AE = 2'b10;
        else if (RegWriteW && (RdW != 5'd0) && (RdW == Rs1E))
            Forward_AE = 2'b01;

        if (RegWriteM && (RdM != 5'd0) && (RdM == Rs2E))
            Forward_BE = 2'b10;
        else if (RegWriteW && (RdW != 5'd0) && (RdW == Rs2E))
            Forward_BE = 2'b01;
    end

    // ── Load-use hazard stall ─────────────────────────────────
    // Must stall when ALL of:
    //   1. ResultSrcE == 2'b01 → EX stage is a LOAD (reads memory)
    //   2. RegWriteE is asserted → result will be written to regfile
    //   3. RdE != x0 → destination is a real register
    //   4. RdE matches Rs1D or Rs2D → decode stage needs that value
    //
    // Original was missing RegWriteE check — could mis-stall on
    // instructions that use ResultSrc=01 without actually writing
    // (e.g. a hypothetical memory-read-discard instruction).
    wire lw_stall = (ResultSrcE_in == 2'b01)
                    && RegWriteE
                    && (RdE != 5'd0)
                    && ((Rs1D == RdE) || (Rs2D == RdE));

    // ── Stall / flush control ─────────────────────────────────
    always @(*) begin
        StallF = 1'b0;
        StallD = 1'b0;
        FlushD = 1'b0;
        FlushE = 1'b0;

        if (PCSRCE) begin
            // Branch/jump taken: flush IF and ID stages
            // Do NOT stall — PC is being redirected
            FlushD = 1'b1;
            FlushE = 1'b1;
        end else if (lw_stall) begin
            // Load-use hazard: freeze IF and ID, bubble into EX
            StallF = 1'b1;
            StallD = 1'b1;
            FlushE = 1'b1;
        end
    end

endmodule









