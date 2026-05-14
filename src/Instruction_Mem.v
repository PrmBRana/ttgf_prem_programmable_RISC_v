`default_nettype none

module instruction_mem #(
    parameter integer DEPTH  = 64,
    parameter integer ADDR_W = 6
)(
    input  wire              clk,
    input  wire              we,
    input  wire [ADDR_W-1:0] addr,
    input  wire [31:0]       wdata,

    input  wire [31:0]       read_Address,
    output wire [31:0]       Instruction_out
);

    localparam [31:0] NOP = 32'h00000013;

    reg [31:0] mem [0:DEPTH-1];

    // Correct byte-to-word conversion
    // Note: read_Address[1:0] are always 0 (word-aligned access)
    //       read_Address[31:8] unused in 64-word design (6-bit addressing)
    /* verilator lint_off UNUSEDSIGNAL */
    wire [ADDR_W-1:0] word_idx = read_Address[ADDR_W+1:2];
    /* verilator lint_on UNUSEDSIGNAL */

    // Write
    always @(posedge clk) begin
        if (we)
            mem[addr] <= wdata;
    end

    // Read
    // WIDTHEXPAND: word_idx (6 bits) < DEPTH (64 = 7 bits)
    // Safe because word_idx can only be 0..63, always < DEPTH
    /* verilator lint_off WIDTHEXPAND */
    assign Instruction_out = (word_idx < DEPTH) ? mem[word_idx] : NOP;
    /* verilator lint_on WIDTHEXPAND */

endmodule

`default_nettype wire




