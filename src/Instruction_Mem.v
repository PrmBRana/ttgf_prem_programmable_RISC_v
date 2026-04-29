`default_nettype none
`timescale 1ns/1ps

module instruction_mem #(
    parameter integer DEPTH  = 64,
    parameter integer ADDR_W = 6
)(
    input  wire              clk,
    input  wire              we,
    input  wire [ADDR_W-1:0] addr,
    input  wire [31:0]       wdata,
    input  wire [ADDR_W-1:0] read_word_idx,
    output wire [31:0]       Instruction_out
);

    localparam [31:0] NOP = 32'h0000_0013;

    reg [31:0] mem [0:DEPTH-1];

    integer i;
    initial begin
        for (i = 0; i < DEPTH; i = i + 1)
            mem[i] = NOP;
    end

    // Write port
    always @(posedge clk) begin
        if (we)
            mem[addr] <= wdata;
    end

    // Pure combinational read
    // Simulation: correct, no latency issues
    // ASIC antenna: handled by OpenLane config below
    assign Instruction_out = mem[read_word_idx];

endmodule











