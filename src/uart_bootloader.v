`default_nettype none

// ============================================================
// UART Bootloader — Clean + Scalable Version
// ============================================================

module uart_bootloader #(
    parameter ADDR_W = 7
)(
    input  wire                 clk,
    input  wire                 reset,
    input  wire [7:0]           rx_data,
    input  wire                 rx_valid,

    output reg  [7:0]           tx_data,
    output reg                  tx_start,

    output reg                  mem_we,
    output reg  [ADDR_W-1:0]    mem_addr,
    output reg  [31:0]          mem_wdata,

    output reg                  stall_pro
);

    // -------------------------
    // Protocol constants
    // -------------------------
    localparam [31:0] SENTINEL       = 32'hBAADF00D;
    localparam [7:0]  HANDSHAKE_BYTE = 8'h25;
    localparam [7:0]  ACK            = 8'h55;
    localparam [7:0]  NACK           = 8'hFF;

    // -------------------------
    // Internal registers
    // -------------------------
    reg handshake_done;
    reg boot_done;

    reg rx_valid_d;

    reg [31:0] asm_word;
    reg [31:0] write_word;

    reg [1:0]  byte_cnt;
    reg        write_pending;
    reg        is_sentinel;

    reg [ADDR_W-1:0] addr_count;

    reg [1:0] done_sr;

    wire rx_edge = rx_valid & ~rx_valid_d;

    // ============================================================
    // Main sequential logic
    // ============================================================
    always @(posedge clk) begin
        if (reset) begin
            rx_valid_d     <= 1'b0;

            tx_data        <= 8'd0;
            tx_start       <= 1'b0;

            mem_we         <= 1'b0;
            mem_addr       <= {ADDR_W{1'b0}};
            mem_wdata      <= 32'd0;

            stall_pro      <= 1'b1;

            handshake_done <= 1'b0;
            boot_done      <= 1'b0;

            asm_word       <= 32'd0;
            write_word     <= 32'd0;

            byte_cnt       <= 2'd0;

            write_pending  <= 1'b0;
            is_sentinel    <= 1'b0;

            addr_count     <= {ADDR_W{1'b0}};

            done_sr        <= 2'b00;
        end else begin

            // -------------------------
            // default pulse clears
            // -------------------------
            rx_valid_d <= rx_valid;
            tx_start   <= 1'b0;
            mem_we     <= 1'b0;

            // -------------------------
            // stall after boot complete
            // -------------------------
            if (boot_done) begin
                done_sr   <= {done_sr[0], 1'b1};
                stall_pro <= ~done_sr[1];
            end

            // -------------------------
            // handshake stage
            // -------------------------
            if (!handshake_done && rx_edge) begin
                if (rx_data == HANDSHAKE_BYTE) begin
                    tx_data        <= ACK;
                    tx_start       <= 1'b1;
                    handshake_done <= 1'b1;
                end else begin
                    tx_data  <= NACK;
                    tx_start <= 1'b1;
                end
            end

            // -------------------------
            // boot data receive stage
            // -------------------------
            else if (handshake_done && rx_edge && !boot_done) begin

                case (byte_cnt)
                    2'd0: asm_word[7:0]   <= rx_data;
                    2'd1: asm_word[15:8]  <= rx_data;
                    2'd2: asm_word[23:16] <= rx_data;
                    2'd3: begin
                        write_word  <= {rx_data, asm_word[23:0]};
                        is_sentinel <= ({rx_data, asm_word[23:0]} == SENTINEL);
                        write_pending <= 1'b1;
                    end
                endcase

                // safe wrap
                byte_cnt <= (byte_cnt == 2'd3) ? 2'd0 : byte_cnt + 1'b1;
            end

            // -------------------------
            // memory write stage
            // -------------------------
            if (write_pending) begin
                write_pending <= 1'b0;

                if (is_sentinel) begin
                    boot_done <= 1'b1;
                end else begin
                    mem_we    <= 1'b1;
                    mem_addr  <= addr_count;
                    mem_wdata <= write_word;

                    addr_count <= addr_count + 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire
