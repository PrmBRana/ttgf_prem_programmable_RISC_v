`default_nettype none

module ALU (
    input  wire [31:0] ScrA,
    input  wire [31:0] ScrB,
    input  wire [3:0]  ALUControl,
    input  wire [1:0]  ALUType,
    output reg  [31:0] ALUResult,
    output reg         Zero
);

    wire is_sub = (ALUType == 2'b00 && ALUControl == 4'b0011);

    wire [31:0] B_eff = is_sub ? (~ScrB + 32'd1) : ScrB;
    wire [31:0] sum   = ScrA + B_eff;

    wire eq = (ScrA == ScrB);
    wire lt_s = ($signed(ScrA) < $signed(ScrB));
    wire lt_u = (ScrA < ScrB);

    wire [4:0] shamt = ScrB[4:0];

    wire [31:0] shl = ScrA << shamt;
    wire [31:0] shr = ScrA >> shamt;
    wire [31:0] sra = $signed(ScrA) >>> shamt;

    always @(*) begin
        ALUResult = 32'd0;
        Zero      = 1'b0;

        case (ALUType)

            2'b01, 2'b11: begin
                ALUResult = sum;
                Zero      = (sum == 32'd0);
            end

            2'b10: begin
                case (ALUControl)
                    4'b0000: Zero = eq;
                    4'b0001: Zero = ~eq;
                    4'b0010: Zero = lt_s;
                    4'b0011: Zero = ~lt_s;
                    4'b0100: Zero = lt_u;
                    4'b0101: Zero = ~lt_u;
                    default: Zero = 1'b0;
                endcase
            end

            2'b00: begin
                case (ALUControl)
                    4'b0000: ALUResult = ScrA & ScrB;
                    4'b0001: ALUResult = ScrA | ScrB;
                    4'b0010: ALUResult = sum;
                    4'b0011: ALUResult = sum;
                    4'b0100: ALUResult = ScrA ^ ScrB;
                    4'b0101: ALUResult = shl;
                    4'b0110: ALUResult = shr;
                    4'b0111: ALUResult = sra;
                    4'b1000: ALUResult = {31'd0, lt_s};
                    4'b1001: ALUResult = {31'd0, lt_u};
                    4'b1010: ALUResult = ScrB;
                    default: ALUResult = 32'd0;
                endcase

                Zero = (ALUResult == 32'd0);
            end

        endcase
    end

endmodule

`default_nettype wire




