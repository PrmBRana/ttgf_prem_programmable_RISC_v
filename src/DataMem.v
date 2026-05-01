`default_nettype none
`timescale 1ns / 1ps

// ============================================================
//  DataMem — Memory-mapped peripheral controller
//  OPTIMIZED FOR GF180MCU ASIC SYNTHESIS
//
//  Fixes applied:
//  ✓ gpio1_wr_en, gpio1_wdata, gpio2_wr_en, gpio2_wdata
//    changed from output wire → output reg (driven in always blocks)
//  ✓ Removed duplicate GPIO2 always block
//  ✓ Added UART RX ports (uart_in_data, uart_rx_ready) to match
//    pipeline.v instantiation
//  ✓ Removed non-existent parameters UART_FIFO_DEPTH / SPI_RX_DEPTH
//    (pipeline.v must not pass these — fixed there instead)
// ============================================================

`define UART_TX_ADDR   32'h1000_0000
`define UART_TXST_ADDR 32'h1000_0008
`define UART_RX_ADDR   32'h1000_0004
`define UART_RXST_ADDR 32'h1000_000C
`define SPI2_TX_ADDR   32'h4000_0000
`define SPI2_TXST_ADDR 32'h4000_0004
`define SPI2_RX_ADDR   32'h4000_0008
`define SPI2_RXST_ADDR 32'h4000_000C
`define GPIO1_ADDR     32'h3000_0000
`define GPIO2_ADDR     32'h3000_0004

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

    // UART RX  ← added to match pipeline.v port connections
    input  wire [7:0]  uart_in_data,
    input  wire        uart_rx_ready,

    // SPI2
    output reg  [7:0]  spi2_tx_data,
    output reg         spi2_start,
    output wire        spi2_pending_out,
    input  wire [7:0]  spi2_rx_data,
    input  wire        spi2_busy,
    input  wire        spi2_done,

    // GPIO  ← changed from output wire to output reg (driven in always blocks)
    output reg         gpio1_wr_en,
    output reg         gpio1_wdata,
    output reg         gpio2_wr_en,
    output reg         gpio2_wdata
);

    // =========================================================
    // CDC SYNCHRONIZERS (for cross-domain inputs)
    // =========================================================
    // NOTE: Remove these if uart_tx_busy, spi2_busy, spi2_done,
    //       and spi2_rx_data are guaranteed synchronous with clk.
    //       If they originate from async sources or different clocks,
    //       these 2-FF synchronizers are REQUIRED.

    reg uart_tx_busy_r, uart_tx_busy_sync;
    reg spi2_busy_r,    spi2_busy_sync;
    reg spi2_done_r,    spi2_done_sync;
    reg [7:0] spi2_rx_data_r, spi2_rx_data_sync;

    // CDC for UART RX
    reg [7:0] uart_in_data_r,  uart_in_data_sync;
    reg       uart_rx_ready_r, uart_rx_ready_sync;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            uart_tx_busy_r    <= 1'b0;
            uart_tx_busy_sync <= 1'b0;
            spi2_busy_r       <= 1'b0;
            spi2_busy_sync    <= 1'b0;
            spi2_done_r       <= 1'b0;
            spi2_done_sync    <= 1'b0;
            spi2_rx_data_r    <= 8'd0;
            spi2_rx_data_sync <= 8'd0;
            uart_in_data_r    <= 8'd0;
            uart_in_data_sync <= 8'd0;
            uart_rx_ready_r   <= 1'b0;
            uart_rx_ready_sync<= 1'b0;
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
    // SECONDARY EDGE DETECT (using sync'd spi2_done)
    // =========================================================
    reg spi2_done_sync_r;

    always @(posedge clk or posedge reset) begin
        if (reset) spi2_done_sync_r <= 1'b0;
        else       spi2_done_sync_r <= spi2_done_sync;
    end

    wire spi2_done_rise = spi2_done_sync & ~spi2_done_sync_r;

    // =========================================================
    // ADDRESS DECODE
    // =========================================================
    wire sel_uart_tx   = (aluAddress_in == `UART_TX_ADDR);
    wire sel_uart_txst = (aluAddress_in == `UART_TXST_ADDR);
    wire sel_uart_rx   = (aluAddress_in == `UART_RX_ADDR);
    wire sel_uart_rxst = (aluAddress_in == `UART_RXST_ADDR);

    wire sel_spi2_tx   = (aluAddress_in == `SPI2_TX_ADDR);
    wire sel_spi2_txst = (aluAddress_in == `SPI2_TXST_ADDR);
    wire sel_spi2_rx   = (aluAddress_in == `SPI2_RX_ADDR);
    wire sel_spi2_rxst = (aluAddress_in == `SPI2_RXST_ADDR);

    wire sel_gpio1     = (aluAddress_in == `GPIO1_ADDR);
    wire sel_gpio2     = (aluAddress_in == `GPIO2_ADDR);

    // =========================================================
    // UART TX — single register (no FIFO)
    // =========================================================
    reg [7:0] uart_tx_reg;
    reg       uart_tx_pending;

    wire uart_tx_wr = memwriteM_in && sel_uart_tx && !uart_tx_pending;

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
    // UART RX — single-byte capture
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
            // Clear valid when firmware reads
            if (!memwriteM_in && sel_uart_rx && uart_rx_valid)
                uart_rx_valid <= 1'b0;
        end
    end

    // =========================================================
    // SPI2 TX
    // =========================================================
    reg       spi2_pending;
    reg [7:0] spi2_tx_buf;

    wire spi2_tx_wr = memwriteM_in && sel_spi2_tx && !spi2_pending;

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
    // SPI2 RX — single register (depth=1, no FIFO)
    // =========================================================
    reg [7:0] spi2_rx_reg;
    reg       spi2_rx_valid;

    wire spi2_rx_rd = !memwriteM_in && sel_spi2_rx && spi2_rx_valid;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            spi2_rx_reg   <= 8'd0;
            spi2_rx_valid <= 1'b0;
        end else begin
            if (spi2_done_rise) begin
                spi2_rx_reg   <= spi2_rx_data_sync;
                spi2_rx_valid <= 1'b1;
            end
            if (spi2_rx_rd)
                spi2_rx_valid <= 1'b0;
        end
    end

    // =========================================================
    // GPIO1  ← output reg, duplicate block removed
    // =========================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            gpio1_wr_en <= 1'b0;
            gpio1_wdata <= 1'b1;   // CS_N idle high
        end else begin
            gpio1_wr_en <= 1'b0;
            if (memwriteM_in && sel_gpio1) begin
                gpio1_wdata <= DataWriteM_in[0];
                gpio1_wr_en <= 1'b1;
            end
        end
    end

    // =========================================================
    // GPIO2  ← output reg, duplicate block removed
    // =========================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            gpio2_wr_en <= 1'b0;
            gpio2_wdata <= 1'b1;   // CS_N idle high
        end else begin
            gpio2_wr_en <= 1'b0;
            if (memwriteM_in && sel_gpio2) begin
                gpio2_wdata <= DataWriteM_in[0];
                gpio2_wr_en <= 1'b1;
            end
        end
    end

    // =========================================================
    // READ MUX — combinational
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






