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
    output wire [31:0]       Instruction_out
);
    wire _unused = &{1'b0, read_Address[31:ADDR_W+2], read_Address[1:0]};
    reg [31:0] mem [0:DEPTH-1];

    wire [ADDR_W-1:0] word_idx = read_Address[ADDR_W+1:2];

    // Write port
    always @(posedge clk) begin
        if (we)
            mem[addr] <= wdata;
    end

    assign Instruction_out = mem[word_idx];

endmodule

`default_nettype wire




