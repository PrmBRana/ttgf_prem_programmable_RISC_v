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

    // =========================================================
    // 1. REGISTER WRITE VALID FLAGS (reduce repeated logic)
    // =========================================================
    wire valid_M = RegWriteM && (RdM != 5'd0);
    wire valid_W = RegWriteW && (RdW != 5'd0);

    // =========================================================
    // 2. FORWARDING MATCH SIGNALS (separate layer)
    // =========================================================
    wire match_A_M = valid_M && (Rs1E == RdM);
    wire match_A_W = valid_W && (Rs1E == RdW);

    wire match_B_M = valid_M && (Rs2E == RdM);
    wire match_B_W = valid_W && (Rs2E == RdW);

    // =========================================================
    // 3. LOAD-USE DETECTION (simplified cone)
    // =========================================================
    wire load_use_hazard =
        (ResultSrcE_in == 2'b01) &&
        (RdE != 5'd0) &&
        ((Rs1D == RdE) || (Rs2D == RdE));

    // =========================================================
    // 4. BRANCH HAZARD
    // =========================================================
    wire branch_flush = PCSRCE;

    // =========================================================
    // 5. COMBINATIONAL OUTPUT LOGIC (flat, simple muxing)
    // =========================================================
    always @(*) begin

        // -----------------------------
        // Forwarding A
        // -----------------------------
        if (match_A_M)
            Forward_AE = 2'b10;
        else if (match_A_W)
            Forward_AE = 2'b01;
        else
            Forward_AE = 2'b00;

        // -----------------------------
        // Forwarding B
        // -----------------------------
        if (match_B_M)
            Forward_BE = 2'b10;
        else if (match_B_W)
            Forward_BE = 2'b01;
        else
            Forward_BE = 2'b00;

        // -----------------------------
        // Default control
        // -----------------------------
        StallF = 1'b0;
        StallD = 1'b0;
        FlushD = 1'b0;
        FlushE = 1'b0;

        // -----------------------------
        // Load-use hazard (priority 1)
        // -----------------------------
        if (load_use_hazard) begin
            StallF = 1'b1;
            StallD = 1'b1;
            FlushE = 1'b1;
        end

        // -----------------------------
        // Branch / jump flush (priority 2)
        // -----------------------------
        else if (branch_flush) begin
            FlushD = 1'b1;
            FlushE = 1'b1;
            StallF = 1'b0;
            StallD = 1'b0;
        end

    end

endmodule

`default_nettype wire