`default_nettype none
`timescale 1ns / 1ps

module Write_back (
    input  wire [31:0] ALUResultW_in,
    input  wire [31:0] ReadDataW_in,
    input  wire [31:0] PCPlus4W_in,
    input  wire [1:0]  ResultSrcW_in,
    output reg  [31:0] ResultW
);

    // ── Result select mux ─────────────────────────────────────
    // 2'b00 = ALU result   (R-type, I-type, store address)
    // 2'b01 = Memory read  (load instructions)
    // 2'b10 = PC + 4       (JAL, JALR return address)
    // 2'b11 = unused — default to ALU result, not zero.
    //         Returning zero silently on an unexpected encoding
    //         is harder to debug than returning ALU result,
    //         which will at least show up in register traces.
    always @(*) begin
        case (ResultSrcW_in)
            2'b00:   ResultW = ALUResultW_in;
            2'b01:   ResultW = ReadDataW_in;
            2'b10:   ResultW = PCPlus4W_in;
            default: ResultW = ALUResultW_in; // safe fallback
        endcase
    end

endmodule






