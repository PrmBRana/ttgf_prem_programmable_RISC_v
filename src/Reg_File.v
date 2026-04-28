`default_nettype none
`timescale 1ns / 1ps

module Reg_file (
    input  wire        clk,
    input  wire [4:0]  rs1_addr,
    input  wire [4:0]  rs2_addr,
    input  wire [4:0]  rd_addr,
    input  wire        Regwrite,
    input  wire [31:0] Write_data,
    output wire [31:0] Read_data1,
    output wire [31:0] Read_data2
);

    reg [31:0] rf [0:31];

    // ── Write port — x0 hardwired to zero, always ─────────────
    always @(posedge clk) begin
        if (Regwrite && (rd_addr != 5'd0))
            rf[rd_addr] <= Write_data;
    end

    // ── Raw read — x0 always returns zero ────────────────────
    wire [31:0] raw1 = (rs1_addr == 5'd0) ? 32'd0 : rf[rs1_addr];
    wire [31:0] raw2 = (rs2_addr == 5'd0) ? 32'd0 : rf[rs2_addr];

    // ── Write-first forwarding — explicit all conditions ──────
    // Conditions for forwarding:
    //   1. Regwrite must be asserted (write actually happening)
    //   2. rd_addr must not be x0 (x0 writes are discarded)
    //   3. rd_addr must match the read address
    //   4. rs_addr must not be x0 (x0 reads always return 0)
    // Without condition 4, a write to x1 while reading x0
    // would incorrectly forward Write_data instead of 0.
    wire fwd1 = Regwrite
                && (rd_addr  != 5'd0)
                && (rs1_addr != 5'd0)
                && (rd_addr  == rs1_addr);

    wire fwd2 = Regwrite
                && (rd_addr  != 5'd0)
                && (rs2_addr != 5'd0)
                && (rd_addr  == rs2_addr);

    // ── Outputs — combinational, no register ─────────────────
    // Keeping outputs combinational preserves pipeline timing.
    // Registering here would shift read data one extra cycle and
    // break hazard unit stall/forward decisions.
    assign Read_data1 = fwd1 ? Write_data : raw1;
    assign Read_data2 = fwd2 ? Write_data : raw2;

endmodule






