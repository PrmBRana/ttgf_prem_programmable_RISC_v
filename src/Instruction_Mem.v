`default_nettype none
module instruction_mem #(
    parameter integer DEPTH  = 64,
    parameter integer ADDR_W = $clog2(DEPTH)
)(
    input  wire              clk,
    input  wire              we,
    input  wire [ADDR_W-1:0] addr,
    input  wire [31:0]       wdata,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [31:0]       read_Address,
    /* verilator lint_on UNUSEDSIGNAL */
    output wire [31:0]       Instruction_out
);
    localparam [31:0] NOP = 32'h00000013;
    reg [31:0] mem [0:DEPTH-1];
    wire [ADDR_W-1:0] word_idx = read_Address[ADDR_W+1:2];
    
    always @(posedge clk) begin
        if (we)
            mem[addr] <= wdata;
    end
    
    assign Instruction_out = (word_idx < DEPTH) ? mem[word_idx] : NOP;
endmodule
`default_nettype wire




