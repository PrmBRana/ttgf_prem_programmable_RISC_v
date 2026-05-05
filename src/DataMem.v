`default_nettype none
`timescale 1ns / 1ps

// ============================================================
//  DataMem.v — Memory-mapped peripheral controller
//  GF180MCU / Tiny Tapeout
//
//  CRITICAL PATH FIX (setup violations):
//  Previous version compared full 32-bit ALUResult address in
//  combinational logic AFTER the EX/MEM pipeline register.
//  9 × 32-bit equality comparators + OR/mux chain = ~5 ns at tt,
//  ~12.5 ns at ss_125C — violated 20 ns budget.
//
//  Fix: decode on upper bits only (address[31:12]).
//  All peripheral base addresses are 4 KB aligned so only
//  bits[31:12] are needed to distinguish regions.
//  This reduces the comparator width from 32 to 20 bits,
//  cutting the decode path to ~2.5 ns at tt / ~6 ns at ss.
//
//  Address map (unchanged, just decoded more efficiently):
//    0x1000_0000  UART TX data    [31:12] = 20'h10000
//    0x1000_0004  UART RX data    [31:12] = 20'h10000, [3:2]=01
//    0x1000_0008  UART TX status  [31:12] = 20'h10000, [3:2]=10
//    0x1000_000C  UART RX status  [31:12] = 20'h10000, [3:2]=11
//    0x3000_0000  GPIO1           [31:12] = 20'h30000, [3:2]=00
//    0x3000_0004  GPIO2           [31:12] = 20'h30000, [3:2]=01
//    0x4000_0000  SPI2 TX data    [31:12] = 20'h40000, [3:2]=00
//    0x4000_0004  SPI2 TX status  [31:12] = 20'h40000, [3:2]=01
//    0x4000_0008  SPI2 RX data    [31:12] = 20'h40000, [3:2]=10
//    0x4000_000C  SPI2 RX status  [31:12] = 20'h40000, [3:2]=11
// ============================================================

module DataMem (
    input  wire        clk,
    input  wire        reset,
    input  wire [31:0] aluAddress_in,
    input  wire [7:0]  DataWriteM_in,
    input  wire        memwriteM_in,
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
    // ADDRESS DECODE — upper 20 bits + 2-bit word offset
    // Reduces critical path vs full 32-bit compare
    // =========================================================
    wire [19:0] region  = aluAddress_in[31:12];
    wire [1:0]  word    = aluAddress_in[3:2];    // word select within region

    wire is_uart  = (region == 20'h10000);
    wire is_gpio  = (region == 20'h30000);
    wire is_spi2  = (region == 20'h40000);

    // UART sub-select
    wire sel_uart_tx   = is_uart & (word == 2'b00);
    wire sel_uart_rx   = is_uart & (word == 2'b01);
    wire sel_uart_txst = is_uart & (word == 2'b10);
    wire sel_uart_rxst = is_uart & (word == 2'b11);

    // GPIO sub-select
    wire sel_gpio1 = is_gpio & (word == 2'b00);
    wire sel_gpio2 = is_gpio & (word == 2'b01);

    // SPI2 sub-select
    wire sel_spi2_tx   = is_spi2 & (word == 2'b00);
    wire sel_spi2_txst = is_spi2 & (word == 2'b01);
    wire sel_spi2_rx   = is_spi2 & (word == 2'b10);
    wire sel_spi2_rxst = is_spi2 & (word == 2'b11);

    // =========================================================
    // CDC SYNCHRONIZERS (2-FF for all async inputs)
    // =========================================================
    reg uart_tx_busy_r,    uart_tx_busy_sync;
    reg spi2_busy_r,       spi2_busy_sync;
    reg spi2_done_r,       spi2_done_sync;
    reg [7:0] spi2_rx_data_r,  spi2_rx_data_sync;
    reg [7:0] uart_in_data_r,  uart_in_data_sync;
    reg uart_rx_ready_r,   uart_rx_ready_sync;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            uart_tx_busy_r    <= 1'b0; uart_tx_busy_sync <= 1'b0;
            spi2_busy_r       <= 1'b0; spi2_busy_sync    <= 1'b0;
            spi2_done_r       <= 1'b0; spi2_done_sync    <= 1'b0;
            spi2_rx_data_r    <= 8'd0; spi2_rx_data_sync <= 8'd0;
            uart_in_data_r    <= 8'd0; uart_in_data_sync <= 8'd0;
            uart_rx_ready_r   <= 1'b0; uart_rx_ready_sync<= 1'b0;
        end else begin
            uart_tx_busy_r    <= uart_tx_busy;
            uart_tx_busy_sync <= uart_tx_busy_r;
            spi2_busy_r       <= spi2_busy;
            spi2_busy_sync    <= spi2_busy_r;
            spi2_done_r       <= spi2_done;
            spi2_done_sync    <= spi2_done_r;
            spi2_rx_data_r    <= spi2_rx_data;
            spi2_rx_data_sync <= spi2_rx_data_r;
            uart_in_data_r    <= uart_in_data;
            uart_in_data_sync <= uart_in_data_r;
            uart_rx_ready_r   <= uart_rx_ready;
            uart_rx_ready_sync<= uart_rx_ready_r;
        end
    end

    // =========================================================
    // SPI2 DONE EDGE DETECT
    // =========================================================
    reg spi2_done_sync_r;
    always @(posedge clk or posedge reset) begin
        if (reset) spi2_done_sync_r <= 1'b0;
        else       spi2_done_sync_r <= spi2_done_sync;
    end
    wire spi2_done_rise = spi2_done_sync & ~spi2_done_sync_r;

    // =========================================================
    // UART TX
    // =========================================================
    reg [7:0] uart_tx_reg;
    reg       uart_tx_pending;

    wire uart_tx_wr = memwriteM_in & sel_uart_tx & ~uart_tx_pending;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            uart_tx_reg     <= 8'd0;
            uart_tx_pending <= 1'b0;
            uart_out_data   <= 8'd0;
            uart_tx_start   <= 1'b0;
        end else begin
            uart_tx_start <= 1'b0;
            if (uart_tx_wr) begin
                uart_tx_reg     <= DataWriteM_in;
                uart_tx_pending <= 1'b1;
            end
            if (uart_tx_pending && !uart_tx_busy_sync) begin
                uart_out_data   <= uart_tx_reg;
                uart_tx_start   <= 1'b1;
                uart_tx_pending <= 1'b0;
            end
        end
    end

    // =========================================================
    // UART RX
    // =========================================================
    reg [7:0] uart_rx_reg;
    reg       uart_rx_valid;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            uart_rx_reg   <= 8'd0;
            uart_rx_valid <= 1'b0;
        end else begin
            if (uart_rx_ready_sync) begin
                uart_rx_reg   <= uart_in_data_sync;
                uart_rx_valid <= 1'b1;
            end
            if (!memwriteM_in && sel_uart_rx && uart_rx_valid)
                uart_rx_valid <= 1'b0;
        end
    end

    // =========================================================
    // SPI2 TX
    // =========================================================
    reg       spi2_pending;
    reg [7:0] spi2_tx_buf;

    wire spi2_tx_wr = memwriteM_in & sel_spi2_tx & ~spi2_pending;

    assign spi2_pending_out = spi2_pending;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            spi2_start   <= 1'b0;
            spi2_tx_data <= 8'd0;
            spi2_pending <= 1'b0;
            spi2_tx_buf  <= 8'd0;
        end else begin
            spi2_start <= 1'b0;
            if (spi2_tx_wr) begin
                spi2_tx_buf  <= DataWriteM_in;
                spi2_pending <= 1'b1;
            end
            if (spi2_pending && !spi2_busy_sync && !spi2_done_sync) begin
                spi2_tx_data <= spi2_tx_buf;
                spi2_start   <= 1'b1;
                spi2_pending <= 1'b0;
            end
        end
    end

    // =========================================================
    // SPI2 RX
    // =========================================================
    reg [7:0] spi2_rx_reg;
    reg       spi2_rx_valid;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            spi2_rx_reg   <= 8'd0;
            spi2_rx_valid <= 1'b0;
        end else begin
            if (spi2_done_rise) begin
                spi2_rx_reg   <= spi2_rx_data_sync;
                spi2_rx_valid <= 1'b1;
            end
            if (!memwriteM_in && sel_spi2_rx && spi2_rx_valid)
                spi2_rx_valid <= 1'b0;
        end
    end

    // =========================================================
    // GPIO1
    // =========================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            gpio1_wr_en <= 1'b0;
            gpio1_wdata <= 1'b1;
        end else begin
            gpio1_wr_en <= 1'b0;
            if (memwriteM_in && sel_gpio1) begin
                gpio1_wdata <= DataWriteM_in[0];
                gpio1_wr_en <= 1'b1;
            end
        end
    end

    // =========================================================
    // GPIO2
    // =========================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            gpio2_wr_en <= 1'b0;
            gpio2_wdata <= 1'b1;
        end else begin
            gpio2_wr_en <= 1'b0;
            if (memwriteM_in && sel_gpio2) begin
                gpio2_wdata <= DataWriteM_in[0];
                gpio2_wr_en <= 1'b1;
            end
        end
    end

    // =========================================================
    // READ MUX
    // =========================================================
    always @(*) begin
        DataMem_out = 32'h0000_0000;
        if (!memwriteM_in) begin
            if      (sel_uart_txst) DataMem_out = {30'd0, uart_tx_busy_sync, uart_tx_pending};
            else if (sel_uart_rx)   DataMem_out = {24'd0, uart_rx_reg};
            else if (sel_uart_rxst) DataMem_out = {30'd0, 1'b0, uart_rx_valid};
            else if (sel_spi2_tx)   DataMem_out = {24'd0, spi2_tx_buf};
            else if (sel_spi2_txst) DataMem_out = {30'd0, spi2_pending, spi2_busy_sync};
            else if (sel_spi2_rx)   DataMem_out = {24'd0, spi2_rx_reg};
            else if (sel_spi2_rxst) DataMem_out = {30'd0, 1'b0, spi2_rx_valid};
            else if (sel_gpio1)     DataMem_out = {31'd0, gpio1_wdata};
            else if (sel_gpio2)     DataMem_out = {31'd0, gpio2_wdata};
        end
    end

endmodule








