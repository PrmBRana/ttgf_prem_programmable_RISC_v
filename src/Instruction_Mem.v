`default_nettype none

module instruction_mem #(
    parameter integer DEPTH  = 64,
    parameter integer ADDR_W = $clog2(DEPTH)
)(
    input  wire              clk,
    input  wire              we,
    input  wire [ADDR_W-1:0] addr,
    input  wire [31:0]       wdata,
    input  wire [31:0]       read_Address,
    output reg  [31:0]       Instruction_out
);
    reg [31:0] mem [0:DEPTH-1];

    wire _unused = &{1'b0, read_Address[31:ADDR_W+2], read_Address[1:0]};
    
    // Pre-calculate index to significantly shorten the critical timing path
    wire [ADDR_W-1:0] word_idx = read_Address[ADDR_W+1:2];

    // Write Port (Synchronous)
    always @(posedge clk) begin
        if (we)
            mem[addr] <= wdata;
    end

    // Read Port (Combinational — Optimized for Single-Cycle Simulation)
    always @(*) begin
        if (word_idx < DEPTH)
            Instruction_out = mem[word_idx];
        else
            Instruction_out = 32'h0000_0013; // Fallback to RISC-V NOP (addi x0, x0, 0)
    end

endmodule
`default_nettype wire


