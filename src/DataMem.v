`default_nettype none

module DataMem (
    input  wire        clk,
    input  wire        reset,
    input  wire [31:0] aluAddress_in,
    input  wire [7:0]  DataWriteM_in,
    input  wire        memwriteM_in,
    output reg  [31:0] DataMem_out,

    // ── UART TX ───────────────────────────────────────────────
    output reg  [7:0]  uart_out_data,
    output reg         uart_tx_start,
    input  wire        uart_tx_busy,

    // ── UART RX ───────────────────────────────────────────────
    input  wire [7:0]  uart_in_data,
    input  wire        uart_rx_ready,

    // ── SPI2 ──────────────────────────────────────────────────
    output reg  [7:0]  spi2_tx_data,
    output reg         spi2_start,
    output wire        spi2_pending_out,
    input  wire [7:0]  spi2_rx_data,
    input  wire        spi2_busy,
    input  wire        spi2_done,

    // ── GPIO1 ─────────────────────────────────────────────────
    output reg         gpio1_wr_en,
    output reg         gpio1_wdata,

    // ── GPIO2 → SPI2 CS_N ─────────────────────────────────────
    output reg         gpio2_wr_en,
    output reg         gpio2_wdata
);

    // =========================================================
    // Address Decode
    // =========================================================
    wire sel_uart_tx   = (aluAddress_in == 32'h1000_0000);
    wire sel_uart_rx   = (aluAddress_in == 32'h1000_0004);
    wire sel_uart_txst = (aluAddress_in == 32'h1000_0008);
    wire sel_uart_rxst = (aluAddress_in == 32'h1000_000C);
    wire sel_spi2_tx   = (aluAddress_in == 32'h4000_0000);
    wire sel_spi2_txst = (aluAddress_in == 32'h4000_0004);
    wire sel_spi2_rx   = (aluAddress_in == 32'h4000_0008);
    wire sel_spi2_rxst = (aluAddress_in == 32'h4000_000C);
    wire sel_gpio1     = (aluAddress_in == 32'h3000_0000);
    wire sel_gpio2     = (aluAddress_in == 32'h3000_0004);

    // =========================================================
    // UART TX — Single Register (no FIFO)
    // =========================================================
    reg [7:0] uart_tx_reg;
    reg       uart_tx_pending;

    wire uart_tx_wr = memwriteM_in && sel_uart_tx && !uart_tx_pending;

    always @(posedge clk) begin
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

            if (uart_tx_pending && !uart_tx_busy) begin
                uart_out_data   <= uart_tx_reg;
                uart_tx_start   <= 1'b1;
                uart_tx_pending <= 1'b0;
            end
        end
    end

    // =========================================================
    // UART RX — Single Register (depth=1)
    // =========================================================
    // Optimized for GF180: Registered input removes NOT gate
    // from critical path, improving timing by ~0.6 ns
    
    reg uart_rx_ready_r;    // Pipeline stage: sample ready signal
    reg [7:0] uart_rx_reg;  // Capture incoming data
    reg       uart_rx_valid; // Data valid flag

    wire uart_rx_rd = !memwriteM_in && sel_uart_rx && uart_rx_valid;

    always @(posedge clk) begin
        if (reset) begin
            uart_rx_ready_r <= 1'b0;
            uart_rx_reg     <= 8'd0;
            uart_rx_valid   <= 1'b0;
        end else begin
            // Stage 1: Register the ready signal
            uart_rx_ready_r <= uart_rx_ready;

            // Stage 2: Capture data when ready was seen
            if (uart_rx_ready_r) begin
                uart_rx_reg   <= uart_in_data;
                uart_rx_valid <= 1'b1;
            end

            // Clear valid when CPU reads the data
            if (uart_rx_rd) begin
                uart_rx_valid <= 1'b0;
            end
        end
    end

    // =========================================================
    // SPI2 TX
    // =========================================================
    reg       spi2_pending;
    reg [7:0] spi2_tx_buf;

    wire spi2_tx_wr = memwriteM_in && sel_spi2_tx && !spi2_pending;

    assign spi2_pending_out = spi2_pending;

    always @(posedge clk) begin
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

            if (spi2_pending && !spi2_busy && !spi2_done) begin
                spi2_tx_data <= spi2_tx_buf;
                spi2_start   <= 1'b1;
                spi2_pending <= 1'b0;
            end
        end
    end

    // =========================================================
    // SPI2 RX — Single Register (depth=1)
    // =========================================================
    reg [7:0] spi2_rx_reg;
    reg       spi2_rx_valid;
    reg       spi2_done_r;

    wire spi2_done_rise = spi2_done & ~spi2_done_r;
    wire spi2_rx_rd = !memwriteM_in && sel_spi2_rx && spi2_rx_valid;

    always @(posedge clk) begin
        if (reset) begin
            spi2_done_r   <= 1'b0;
            spi2_rx_reg   <= 8'd0;
            spi2_rx_valid <= 1'b0;
        end else begin
            spi2_done_r <= spi2_done;

            if (spi2_done_rise) begin
                spi2_rx_reg   <= spi2_rx_data;
                spi2_rx_valid <= 1'b1;
            end

            if (spi2_rx_rd)
                spi2_rx_valid <= 1'b0;
        end
    end

    // =========================================================
    // GPIO1
    // =========================================================
    always @(posedge clk) begin
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
    // GPIO2 → SPI2 CS_N
    // =========================================================
    always @(posedge clk) begin
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
    // READ MUX — Combinational
    // =========================================================
    always @(*) begin
        DataMem_out = 32'h0000_0000;
        if (!memwriteM_in) begin
            if      (sel_uart_tx)   DataMem_out = {24'd0, uart_out_data};
            else if (sel_uart_txst) DataMem_out = {30'd0, uart_tx_busy, uart_tx_pending};
            else if (sel_uart_rx)   DataMem_out = {24'd0, uart_rx_reg};
            else if (sel_uart_rxst) DataMem_out = {31'd0, uart_rx_valid};
            else if (sel_spi2_tx)   DataMem_out = {24'd0, spi2_tx_buf};
            else if (sel_spi2_txst) DataMem_out = {30'd0, spi2_pending, spi2_busy};
            else if (sel_spi2_rx)   DataMem_out = {24'd0, spi2_rx_reg};
            else if (sel_spi2_rxst) DataMem_out = {30'd0, 1'b0, spi2_rx_valid};
            else if (sel_gpio1)     DataMem_out = {31'd0, gpio1_wdata};
            else if (sel_gpio2)     DataMem_out = {31'd0, gpio2_wdata};
        end
    end

endmodule

`default_nettype wire