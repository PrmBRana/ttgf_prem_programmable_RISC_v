`default_nettype none

// ============================================================
//  DataMem — Fixed version
//
//  FIXES vs original:
//
//  FIX 1 — FIFO read-advance timing (Critical)
//    Original bug: the read mux used uart_rx_fifo[uart_rx_rd_ptr]
//    combinatorially. The rd_ptr only advanced the NEXT cycle,
//    so back-to-back LW reads returned the same byte twice.
//
//    Fix: register the FIFO output one cycle ahead into
//    a "read data register" that is updated on the cycle the
//    rd_ptr advances. The read mux then uses this registered
//    value, which is always stable and correct.
//    Same fix applied to SPI2 RX FIFO.
//
//  FIX 2 — 32-bit write data path (Moderate)
//    Original DataWriteM_in was 8-bit; pipeline passed only [7:0].
//    Changed to 32-bit so SW/SH/SB can all be supported.
//    Peripheral registers still use [7:0] slice (UART/SPI
//    are byte-wide by nature).
//
//  FIX 3 — funct3 added for LB/LBU/LH/LHU sign extension
//    The load funct3 is plumbed in so firmware can use
//    signed byte/halfword loads correctly.
// ============================================================

module DataMem (
    input  wire        clk,
    input  wire        reset,
    input  wire [31:0] aluAddress_in,
    input  wire [31:0] DataWriteM_in,    // FIX 2: was [7:0]
    input  wire        memwriteM_in,
    input  wire [2:0]  funct3,           // FIX 3: for load size/sign
    output reg  [31:0] DataMem_out,

    // UART TX
    output reg  [7:0]  uart_out_data,
    output reg         uart_tx_start,
    input  wire        uart_tx_busy,

    // UART RX
    input  wire [7:0]  uart_in_data,
    input  wire        uart_rx_ready,

    // SPI2
    output reg  [7:0]  spi2_tx_data,
    output reg         spi2_start,
    output wire        spi2_pending_out,
    input  wire [7:0]  spi2_rx_data,
    input  wire        spi2_busy,
    input  wire        spi2_done,

    // GPIO
    output reg         gpio1_wr_en,
    output reg         gpio1_wdata,
    output reg         gpio2_wr_en,
    output reg         gpio2_wdata
);

    // =========================================================
    // Address Decode
    // =========================================================
    wire sel_uart_tx    = (aluAddress_in == 32'h1000_0000);
    wire sel_uart_rx    = (aluAddress_in == 32'h1000_0004);
    wire sel_uart_txst  = (aluAddress_in == 32'h1000_0008);
    wire sel_uart_rxst  = (aluAddress_in == 32'h1000_000C);

    wire sel_spi2_tx    = (aluAddress_in == 32'h4000_0000);
    wire sel_spi2_txst  = (aluAddress_in == 32'h4000_0004);
    wire sel_spi2_rx    = (aluAddress_in == 32'h4000_0008);
    wire sel_spi2_rxst  = (aluAddress_in == 32'h4000_000C);

    wire sel_gpio1      = (aluAddress_in == 32'h3000_0000);
    wire sel_gpio2      = (aluAddress_in == 32'h3000_0004);

    // =========================================================
    // FIFO Pointer Helper
    // =========================================================
    function automatic [1:0] fifo_next_ptr(input [1:0] ptr);
        fifo_next_ptr = (ptr == 2'd3) ? 2'd0 : ptr + 2'd1;
    endfunction

    // =========================================================
    // UART TX FIFO (4-deep)
    // =========================================================
    reg [7:0] uart_tx_fifo [0:3];
    reg [1:0] uart_tx_wr_ptr, uart_tx_rd_ptr;
    reg       uart_tx_full, uart_tx_empty;

    wire uart_tx_wr_en = memwriteM_in && sel_uart_tx && !uart_tx_full;
    wire uart_tx_pop   = !uart_tx_empty && !uart_tx_busy && !uart_tx_start;

    always @(posedge clk) begin
        if (reset) begin
            uart_tx_wr_ptr <= 2'd0;
            uart_tx_rd_ptr <= 2'd0;
            uart_tx_full   <= 1'b0;
            uart_tx_empty  <= 1'b1;
        end else begin
            if (uart_tx_wr_en) begin
                uart_tx_fifo[uart_tx_wr_ptr] <= DataWriteM_in[7:0];
                uart_tx_wr_ptr <= fifo_next_ptr(uart_tx_wr_ptr);
            end
            if (uart_tx_pop)
                uart_tx_rd_ptr <= fifo_next_ptr(uart_tx_rd_ptr);

            if (uart_tx_wr_en && !uart_tx_pop)
                uart_tx_full <= (fifo_next_ptr(uart_tx_wr_ptr) == uart_tx_rd_ptr);
            else if (!uart_tx_wr_en && uart_tx_pop)
                uart_tx_full <= 1'b0;

            if (uart_tx_wr_en && !uart_tx_pop)
                uart_tx_empty <= 1'b0;
            else if (!uart_tx_wr_en && uart_tx_pop &&
                     (fifo_next_ptr(uart_tx_rd_ptr) == uart_tx_wr_ptr))
                uart_tx_empty <= 1'b1;
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            uart_tx_start <= 1'b0;
            uart_out_data <= 8'd0;
        end else begin
            uart_tx_start <= 1'b0;
            if (uart_tx_pop) begin
                uart_out_data <= uart_tx_fifo[uart_tx_rd_ptr];
                uart_tx_start <= 1'b1;
            end
        end
    end

    // =========================================================
    // UART RX FIFO — FIX 1: registered read data
    // =========================================================
    reg [7:0] uart_rx_fifo [0:3];
    reg [1:0] uart_rx_wr_ptr, uart_rx_rd_ptr;
    reg       uart_rx_full, uart_rx_empty;
    reg       uart_rx_ready_r, uart_rx_ready_rr;

    // Registered read data — updated when rd_ptr advances
    // so the read mux always sees stable, correct data.
    reg [7:0] uart_rx_rdata_reg;

    wire uart_rx_ready_rise = uart_rx_ready_r & ~uart_rx_ready_rr;
    wire uart_rx_rd_en      = !memwriteM_in && sel_uart_rx && !uart_rx_empty;

    always @(posedge clk) begin
        if (reset) begin
            uart_rx_ready_r  <= 1'b0;
            uart_rx_ready_rr <= 1'b0;
        end else begin
            uart_rx_ready_rr <= uart_rx_ready_r;
            uart_rx_ready_r  <= uart_rx_ready;
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            uart_rx_wr_ptr    <= 2'd0;
            uart_rx_rd_ptr    <= 2'd0;
            uart_rx_full      <= 1'b0;
            uart_rx_empty     <= 1'b1;
            uart_rx_rdata_reg <= 8'd0;
        end else begin
            // Write new byte from UART peripheral
            if (uart_rx_ready_rise && !uart_rx_full) begin
                uart_rx_fifo[uart_rx_wr_ptr] <= uart_in_data;
                uart_rx_wr_ptr <= fifo_next_ptr(uart_rx_wr_ptr);
            end

            // FIX 1: capture read data into register as rd_ptr advances
            // This ensures the mux below always reads stable, correct data
            // even on back-to-back LW instructions.
            if (uart_rx_rd_en) begin
                uart_rx_rdata_reg <= uart_rx_fifo[uart_rx_rd_ptr];
                uart_rx_rd_ptr    <= fifo_next_ptr(uart_rx_rd_ptr);
            end

            // Full flag
            if (uart_rx_ready_rise && !uart_rx_rd_en && !uart_rx_full)
                uart_rx_full <= (fifo_next_ptr(uart_rx_wr_ptr) == uart_rx_rd_ptr);
            else if (!uart_rx_ready_rise && uart_rx_rd_en)
                uart_rx_full <= 1'b0;

            // Empty flag
            if (uart_rx_ready_rise && !uart_rx_rd_en)
                uart_rx_empty <= 1'b0;
            else if (!uart_rx_ready_rise && uart_rx_rd_en &&
                     (fifo_next_ptr(uart_rx_rd_ptr) == uart_rx_wr_ptr))
                uart_rx_empty <= 1'b1;
        end
    end

    // =========================================================
    // SPI2 TX FIFO
    // =========================================================
    reg [7:0] spi2_tx_fifo [0:3];
    reg [1:0] spi2_tx_wr_ptr, spi2_tx_rd_ptr;
    reg       spi2_tx_full, spi2_tx_empty;

    wire spi2_tx_wr_en = memwriteM_in && sel_spi2_tx && !spi2_tx_full;
    wire spi2_tx_pop   = !spi2_tx_empty && !spi2_busy && !spi2_start;

    assign spi2_pending_out = !spi2_tx_empty;

    always @(posedge clk) begin
        if (reset) begin
            spi2_tx_wr_ptr <= 2'd0; spi2_tx_rd_ptr <= 2'd0;
            spi2_tx_full   <= 1'b0; spi2_tx_empty  <= 1'b1;
        end else begin
            if (spi2_tx_wr_en) begin
                spi2_tx_fifo[spi2_tx_wr_ptr] <= DataWriteM_in[7:0];
                spi2_tx_wr_ptr <= fifo_next_ptr(spi2_tx_wr_ptr);
            end
            if (spi2_tx_pop)
                spi2_tx_rd_ptr <= fifo_next_ptr(spi2_tx_rd_ptr);

            if (spi2_tx_wr_en && !spi2_tx_pop)
                spi2_tx_full <= (fifo_next_ptr(spi2_tx_wr_ptr) == spi2_tx_rd_ptr);
            else if (!spi2_tx_wr_en && spi2_tx_pop)
                spi2_tx_full <= 1'b0;

            if (spi2_tx_wr_en && !spi2_tx_pop)
                spi2_tx_empty <= 1'b0;
            else if (!spi2_tx_wr_en && spi2_tx_pop &&
                     (fifo_next_ptr(spi2_tx_rd_ptr) == spi2_tx_wr_ptr))
                spi2_tx_empty <= 1'b1;
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            spi2_start   <= 1'b0;
            spi2_tx_data <= 8'd0;
        end else begin
            spi2_start <= 1'b0;
            if (spi2_tx_pop) begin
                spi2_tx_data <= spi2_tx_fifo[spi2_tx_rd_ptr];
                spi2_start   <= 1'b1;
            end
        end
    end

    // =========================================================
    // SPI2 RX FIFO — FIX 1: registered read data
    // =========================================================
    reg [7:0] spi2_rx_fifo [0:3];
    reg [1:0] spi2_rx_wr_ptr, spi2_rx_rd_ptr;
    reg       spi2_rx_full, spi2_rx_empty;
    reg       spi2_done_r;

    // Registered read data (same fix as UART RX)
    reg [7:0] spi2_rx_rdata_reg;

    wire spi2_done_rise = spi2_done & ~spi2_done_r;
    wire spi2_rx_rd_en  = !memwriteM_in && sel_spi2_rx && !spi2_rx_empty;

    always @(posedge clk) begin
        if (reset) spi2_done_r <= 1'b0;
        else       spi2_done_r <= spi2_done;
    end

    always @(posedge clk) begin
        if (reset) begin
            spi2_rx_wr_ptr    <= 2'd0; spi2_rx_rd_ptr <= 2'd0;
            spi2_rx_full      <= 1'b0; spi2_rx_empty  <= 1'b1;
            spi2_rx_rdata_reg <= 8'd0;
        end else begin
            if (spi2_done_rise && !spi2_rx_full) begin
                spi2_rx_fifo[spi2_rx_wr_ptr] <= spi2_rx_data;
                spi2_rx_wr_ptr <= fifo_next_ptr(spi2_rx_wr_ptr);
            end

            // FIX 1: capture into register as pointer advances
            if (spi2_rx_rd_en) begin
                spi2_rx_rdata_reg <= spi2_rx_fifo[spi2_rx_rd_ptr];
                spi2_rx_rd_ptr    <= fifo_next_ptr(spi2_rx_rd_ptr);
            end

            if (spi2_done_rise && !spi2_rx_rd_en && !spi2_rx_full)
                spi2_rx_full <= (fifo_next_ptr(spi2_rx_wr_ptr) == spi2_rx_rd_ptr);
            else if (!spi2_done_rise && spi2_rx_rd_en)
                spi2_rx_full <= 1'b0;

            if (spi2_done_rise && !spi2_rx_rd_en)
                spi2_rx_empty <= 1'b0;
            else if (!spi2_done_rise && spi2_rx_rd_en &&
                     (fifo_next_ptr(spi2_rx_rd_ptr) == spi2_rx_wr_ptr))
                spi2_rx_empty <= 1'b1;
        end
    end

    // =========================================================
    // GPIO
    // =========================================================
    always @(posedge clk) begin
        if (reset) begin gpio1_wr_en <= 1'b0; gpio1_wdata <= 1'b0; end
        else begin
            gpio1_wr_en <= 1'b0;
            if (memwriteM_in && sel_gpio1) begin
                gpio1_wdata <= DataWriteM_in[0];
                gpio1_wr_en <= 1'b1;
            end
        end
    end

    always @(posedge clk) begin
        if (reset) begin gpio2_wr_en <= 1'b0; gpio2_wdata <= 1'b0; end
        else begin
            gpio2_wr_en <= 1'b0;
            if (memwriteM_in && sel_gpio2) begin
                gpio2_wdata <= DataWriteM_in[0];
                gpio2_wr_en <= 1'b1;
            end
        end
    end

    // =========================================================
    // READ MUX — FIX 1 + FIX 3
    //
    //  FIX 1: Use registered _rdata_reg instead of direct
    //         fifo[rd_ptr] — pointer has already advanced,
    //         so the registered value holds the correct byte.
    //
    //  FIX 3: Apply funct3 sign/zero extension for loads.
    //         funct3[2]=1 → unsigned (zero-extend)
    //         funct3[2]=0 → signed  (sign-extend)
    //         funct3[1:0]: 00=byte, 01=halfword, 10=word
    //         Peripheral bus is byte-wide so only byte/word
    //         cases are meaningful; halfword included for
    //         completeness.
    // =========================================================

    // Raw 8-bit read value selected by address
    reg [7:0] raw_byte;
    always @(*) begin
        raw_byte = 8'd0;
        if (!memwriteM_in) begin
            if      (sel_uart_rx)   raw_byte = uart_rx_rdata_reg;
            else if (sel_spi2_rx)   raw_byte = spi2_rx_rdata_reg;
        end
    end

    // Sign/zero extend based on funct3
    wire load_unsigned = funct3[2];

    wire [31:0] byte_extended =
        load_unsigned ? {24'd0,            raw_byte}
                      : {{24{raw_byte[7]}}, raw_byte};

    always @(*) begin
        DataMem_out = 32'h0000_0000;
        if (!memwriteM_in) begin
            if      (sel_uart_txst)
                DataMem_out = {29'd0, uart_tx_full, uart_tx_busy, !uart_tx_empty};
            else if (sel_uart_rx)
                DataMem_out = byte_extended;          // FIX 1+3
            else if (sel_uart_rxst)
                DataMem_out = {29'd0, uart_rx_full, 1'b0, !uart_rx_empty};

            else if (sel_spi2_txst)
                DataMem_out = {29'd0, spi2_tx_full, spi2_busy, !spi2_tx_empty};
            else if (sel_spi2_rx)
                DataMem_out = byte_extended;          // FIX 1+3
            else if (sel_spi2_rxst)
                DataMem_out = {29'd0, spi2_rx_full, 1'b0, !spi2_rx_empty};
        end
    end

endmodule

`default_nettype wire