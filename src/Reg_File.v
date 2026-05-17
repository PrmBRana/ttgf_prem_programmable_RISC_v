`default_nettype none

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

    // Write
    always @(posedge clk) begin
        if (Regwrite && rd_addr != 5'd0) begin
            rf[rd_addr] <= Write_data;
        end
    end

    // Read + Bypass (Forwarding)
    wire [31:0] raw_read1 = rf[rs1_addr];
    wire [31:0] raw_read2 = rf[rs2_addr];

    wire forward_rs1 = Regwrite && (rd_addr == rs1_addr) && (rd_addr != 5'd0);
    wire forward_rs2 = Regwrite && (rd_addr == rs2_addr) && (rd_addr != 5'd0);

    assign Read_data1 = (rs1_addr == 5'd0) ? 32'd0 : (forward_rs1 ? Write_data : raw_read1);
    assign Read_data2 = (rs2_addr == 5'd0) ? 32'd0 : (forward_rs2 ? Write_data : raw_read2);

endmodule

`default_nettype wire