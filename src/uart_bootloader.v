`default_nettype none

// ============================================================
//  uart_bootloader — fixed version
//
//  Bug fixes vs. previous version:
//
//  FIX 1 — Sentinel changed from 0x00000073 (ecall) to
//    0xDEADBEEF.  The sentinel word is NOT written to IMEM —
//    it is purely a stop marker.  Your program must end with
//    a real infinite-loop (JAL x0,0) before the sentinel.
//    Testbench must append 0xDEADBEEF as the last 4 bytes.
//
//  FIX 2 — addr_count widened from 5-bit to 6-bit so
//    addresses 0..63 are reachable (DEPTH=64, ADDR_W=6).
//    Previous 5-bit counter silently wrapped at address 32.
//
//  FIX 3 — boot_done now waits one extra cycle after the
//    last mem_we_reg pulse so the final word is fully written
//    to IMEM before stall_pro drops.  A 2-cycle write-done
//    shift register guarantees the write propagated through
//    the output pipeline register.
//
//  FIX 4 — Dual-buffer race eliminated.  A single 32-bit
//    assembly register with a 2-bit byte counter replaces the
//    two ping-pong buffers.  A 1-bit write_pending flag queues
//    the write; the write happens the cycle after the 4th byte
//    arrives so byte reception and memory writing never
//    compete in the same always block assign.
//
//  FIX 5 — mem_addr port is [7:0] for pipeline.v compat.
//    Internal addr_count is [5:0] (covers 0..63).
//
//  Protocol (unchanged from testbench perspective):
//    1. Host sends 0x25 (handshake).
//    2. Bootloader replies 0x55 (ACK).
//    3. Host sends program words, LSB first, 4 bytes each.
//    4. Host sends sentinel word 0xDEADBEEF (4 bytes).
//    5. Bootloader releases stall, CPU starts running.
// ============================================================

module uart_bootloader (
    input  wire        clk,
    input  wire        reset,
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,
    output reg  [7:0]  tx_data,
    output reg         tx_start,
    output reg         mem_we,
    output reg  [7:0]  mem_addr,
    output reg  [31:0] mem_wdata,
    output reg         stall_pro
);

    // Sentinel is never written to IMEM — it is a stop marker only.
    // Your program must end with JAL x0,0 (0xFFFFF06F) before this.
    localparam [31:0] SENTINEL       = 32'hDEADBEEF;
    localparam [7:0]  HANDSHAKE_BYTE = 8'h25;
    localparam [7:0]  ACK            = 8'h55;
    localparam [7:0]  NACK           = 8'hFF;

    // ── State ─────────────────────────────────────────────────
    reg         handshake_done;
    reg         boot_done;
    reg         rx_valid_d;

    // FIX 4: single assembly register + byte counter
    reg [31:0]  asm_word;           // word being assembled
    reg [1:0]   byte_cnt;           // 0..3 within current word
    reg         write_pending;      // a complete word is waiting
    reg [31:0]  write_word;         // the word to write
    reg         is_sentinel;        // write_word == SENTINEL

    // FIX 2: 6-bit address counter
    reg [5:0]   addr_count;

    // FIX 3: write-done shift register (2 cycles covers the
    // mem_we_reg → mem_we pipeline stage + IMEM write latency)
    reg [1:0]   done_sr;            // shift register for boot_done delay

    wire rx_edge = rx_valid & ~rx_valid_d;

    always @(posedge clk) begin
        if (reset) begin
            rx_valid_d     <= 1'b0;
            tx_data        <= 8'd0;
            tx_start       <= 1'b0;
            mem_we         <= 1'b0;
            mem_addr       <= 8'd0;
            mem_wdata      <= 32'd0;
            handshake_done <= 1'b0;
            boot_done      <= 1'b0;
            stall_pro      <= 1'b1;
            asm_word       <= 32'd0;
            byte_cnt       <= 2'd0;
            write_pending  <= 1'b0;
            write_word     <= 32'd0;
            is_sentinel    <= 1'b0;
            addr_count     <= 6'd0;
            done_sr        <= 2'b00;
        end else begin
            rx_valid_d <= rx_valid;
            tx_start   <= 1'b0;    // default: no TX this cycle
            mem_we     <= 1'b0;    // default: no write this cycle

            // ── stall_pro release (FIX 3) ────────────────────
            // done_sr shifts in a 1 when the sentinel write has
            // propagated.  We release stall two cycles later so
            // the last real instruction is stable in IMEM.
            if (boot_done) begin
                done_sr   <= {done_sr[0], 1'b1};
                stall_pro <= ~done_sr[1];   // drops after 2 cycles
            end

            // ── Handshake ─────────────────────────────────────
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

            // ── Byte assembly (FIX 4) ─────────────────────────
            // Accumulate 4 bytes into asm_word, then raise
            // write_pending.  Never touch mem_we here — that
            // happens the next cycle in the write block below.
            else if (handshake_done && rx_edge && !boot_done) begin
                case (byte_cnt)
                    2'd0: asm_word[7:0]   <= rx_data;
                    2'd1: asm_word[15:8]  <= rx_data;
                    2'd2: asm_word[23:16] <= rx_data;
                    2'd3: begin
                        // Latch full word into write_word so
                        // asm_word is free for the next word
                        // immediately on the next byte.
                        write_word    <= {rx_data, asm_word[23:0]};
                        is_sentinel   <= ({rx_data, asm_word[23:0]} == SENTINEL);
                        write_pending <= 1'b1;
                    end
                    default: ;
                endcase
                byte_cnt <= byte_cnt + 1'b1;  // wraps 3→0 naturally
            end

            // ── Memory write (FIX 3 + FIX 4) ─────────────────
            // Happens the cycle after write_pending is raised,
            // completely separate from byte assembly above.
            // The sentinel word is NOT written to IMEM.
            if (write_pending) begin
                write_pending <= 1'b0;
                if (is_sentinel) begin
                    // Sentinel detected — do NOT write, just stop
                    boot_done <= 1'b1;
                end else begin
                    mem_we    <= 1'b1;
                    mem_addr  <= {2'b00, addr_count};
                    mem_wdata <= write_word;
                    addr_count <= addr_count + 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire


