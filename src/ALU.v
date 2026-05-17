`default_nettype none

module ALU (
    input  wire [31:0] ScrA,
    input  wire [31:0] ScrB,
    input  wire [3:0]  ALUControl,
    input  wire [1:0]  ALUType,
    output reg  [31:0] ALUResult,
    output reg         Zero
);

    // 1. Unified Adder/Subtractor to minimize 180nm silicon carry chains
    wire is_sub = (ALUType == 2'b00 && ALUControl == 4'b0011);
    wire [31:0] sub_operand = is_sub ? (~ScrB + 32'd1) : ScrB;
    wire [31:0] sum = ScrA + sub_operand;

    // 2. Isolate comparisons to prevent nested multi-bit subtraction
    wire is_equal   = (ScrA == ScrB);
    wire lt_signed  = ($signed(ScrA) < $signed(ScrB));
    wire lt_unsigned = (ScrA < ScrB);

    // 3. Isolate the slow barrel shifters so they do not chain into other logic
    wire [4:0]  shift_amt = ScrB[4:0];
    wire [31:0] sll_res   = ScrA << shift_amt;
    wire [31:0] srl_res   = ScrA >> shift_amt;
    wire [31:0] sra_res   = $signed(ScrA) >>> shift_amt;

    // 4. Flattened Next-State Logic Routing
    always @(*) begin
        ALUResult = 32'd0;
        Zero      = 1'b0;

        case (ALUType)
            2'b01, 2'b11: begin
                ALUResult = sum;
            end

            2'b10: begin
                case (ALUControl)
                    4'b0000: Zero = is_equal;
                    4'b0001: Zero = !is_equal;
                    4'b0010: Zero = lt_signed;
                    4'b0011: Zero = !lt_signed;
                    4'b0100: Zero = lt_unsigned;
                    4'b0101: Zero = !lt_unsigned;
                    default: Zero = 1'b0;
                endcase
            end

            2'b00: begin
                case (ALUControl)
                    4'b0000: ALUResult = ScrA & ScrB;
                    4'b0001: ALUResult = ScrA | ScrB;
                    4'b0010: ALUResult = sum;       // Shared Adder
                    4'b0011: ALUResult = sum;       // Shared Subtractor
                    4'b0100: ALUResult = ScrA ^ ScrB;
                    4'b0101: ALUResult = sll_res;   // Pre-calculated shift
                    4'b0110: ALUResult = srl_res;   // Pre-calculated shift
                    4'b0111: ALUResult = sra_res;   // Pre-calculated shift
                    4'b1000: ALUResult = lt_signed ? 32'd1 : 32'd0;
                    4'b1001: ALUResult = lt_unsigned ? 32'd1 : 32'd0;
                    4'b1010: ALUResult = ScrB;
                    default: ALUResult = 32'd0;
                endcase
                
                // Optimized 32-bit zero check
                Zero = (ALUResult == 32'd0);
            end
            default: ;
        endcase
    end

endmodule
`default_nettype wire



