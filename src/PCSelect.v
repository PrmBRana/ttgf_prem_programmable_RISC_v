`default_nettype none

module PCSelect_MUX (
    input  wire        PCScr,
    input  wire [31:0] PCSequential,
    input  wire [31:0] PCBranch,
    output wire [31:0] Mux3_PC
);
    assign Mux3_PC = PCScr ? PCBranch : PCSequential;
endmodule

`default_nettype wire


