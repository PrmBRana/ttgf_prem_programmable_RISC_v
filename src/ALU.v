`default_nettype none

module ALU (
    input  wire [31:0] ScrA,
    input  wire [31:0] ScrB,
    input  wire [3:0]  ALUControl,
    input  wire [1:0]  ALUType,
    output reg  [31:0] ALUResult,
    output reg         Zero
);

    always @(*) begin
        ALUResult = 32'd0;
        Zero      = 1'b0;

        case (ALUType)

            2'b01, 2'b11: begin
                ALUResult = ScrA + ScrB;
            end

            2'b10: begin
                ALUResult = 32'd0;

                case (ALUControl)
                    4'b0000: Zero = (ScrA == ScrB);
                    4'b0001: Zero = (ScrA != ScrB);
                    4'b0010: Zero = ($signed(ScrA) <  $signed(ScrB));
                    4'b0011: Zero = ($signed(ScrA) >= $signed(ScrB));
                    4'b0100: Zero = (ScrA < ScrB);
                    4'b0101: Zero = (ScrA >= ScrB);
                    default: Zero = 1'b0;
                endcase
            end

            2'b00: begin
                case (ALUControl)
                    4'b0000: ALUResult = ScrA & ScrB;
                    4'b0001: ALUResult = ScrA | ScrB;
                    4'b0010: ALUResult = ScrA + ScrB;
                    4'b0011: ALUResult = ScrA - ScrB;
                    4'b0100: ALUResult = ScrA ^ ScrB;
                    4'b0101: ALUResult = ScrA << ScrB[4:0];
                    4'b0110: ALUResult = ScrA >> ScrB[4:0];
                    4'b0111: ALUResult = $signed(ScrA) >>> ScrB[4:0];
                    4'b1000: ALUResult = ($signed(ScrA) < $signed(ScrB)) ? 32'd1 : 32'd0;
                    4'b1001: ALUResult = (ScrA < ScrB) ? 32'd1 : 32'd0;
                    4'b1010: ALUResult = ScrB; // LUI
                    default: ALUResult = 32'd0;
                endcase

                Zero = (ALUResult == 32'd0);
            end

        endcase
    end

endmodule

`default_nettype wire


