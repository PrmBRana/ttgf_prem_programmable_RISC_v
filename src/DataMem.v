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

    //GPIO1
    output reg         gpio1_wr_en,
    output reg         gpio1_wdata,
    // ── GPIO2 → SPI2 CS_N ─────────────────────────────────────
    output reg         gpio2_wr_en,
    output reg         gpio2_wdata
);

    // ── Address decode ────────────────────────────────────────
    wire sel_uart_tx   = (aluAddress_in == 32'h1000_0000);
    wire sel_uart_txst = (aluAddress_in == 32'h1000_0008);
    wire sel_uart_rx   = (aluAddress_in == 32'h1000_0004);
    wire sel_uart_rxst = (aluAddress_in == 32'h1000_000C);
    wire sel_spi2_tx   = (aluAddress_in == 32'h4000_0000);
    wire sel_spi2_txst = (aluAddress_in == 32'h4000_0004);
    wire sel_spi2_rx   = (aluAddress_in == 32'h4000_0008);
    wire sel_spi2_rxst = (aluAddress_in == 32'h4000_000C);
    wire sel_gpio1     = (aluAddress_in == 32'h3000_0000);
    wire sel_gpio2     = (aluAddress_in == 32'h3000_0004);

    // =========================================================
    // UART TX — improved handshake with proper acknowledge
    // =========================================================
    reg [7:0] uart_tx_reg;
    reg       uart_tx_pending;
    reg       uart_tx_busy_r;      // Sample previous state
    reg       uart_tx_started;     // Track handshake progress

    wire uart_tx_wr = memwriteM_in && sel_uart_tx && !uart_tx_pending;
    wire uart_tx_busy_rising = ~uart_tx_busy_r && uart_tx_busy;

    always @(posedge clk) begin
        if (reset) begin
            uart_tx_reg     <= 8'd0;
            uart_tx_pending <= 1'b0;
            uart_out_data   <= 8'd0;
            uart_tx_start   <= 1'b0;
            uart_tx_busy_r  <= 1'b0;
            uart_tx_started <= 1'b0;
        end else begin
            // Sample uart_tx_busy to detect edges
            uart_tx_busy_r <= uart_tx_busy;

            // Write from firmware: store data and mark pending
            if (uart_tx_wr) begin
                uart_tx_reg     <= DataWriteM_in;
                uart_tx_pending <= 1'b1;
                uart_tx_started <= 1'b0;
                uart_tx_start   <= 1'b0;
            end
            // When data is pending and UART is not busy, assert start
            else if (uart_tx_pending && !uart_tx_started && !uart_tx_busy) begin
                uart_out_data   <= uart_tx_reg;
                uart_tx_start   <= 1'b1;
                uart_tx_started <= 1'b1;
            end
            // When UART becomes busy (acknowledges the transfer), deassert start
            else if (uart_tx_started && uart_tx_busy_rising) begin
                uart_tx_start   <= 1'b0;
            end
            // When UART finishes (goes not busy again), clear pending
            else if (uart_tx_started && !uart_tx_busy && uart_tx_busy_r) begin
                uart_tx_pending <= 1'b0;
                uart_tx_started <= 1'b0;
            end
        end
    end

    // =========================================================
    // SPI2 TX — improved handshake
    // =========================================================
    reg       spi2_pending;
    reg [7:0] spi2_tx_buf;
    reg       spi2_busy_r;
    reg       spi2_done_r;
    reg       spi2_started;

    wire spi2_tx_wr = memwriteM_in && sel_spi2_tx && !spi2_pending;
    wire spi2_busy_rising = ~spi2_busy_r && spi2_busy;
    wire spi2_done_rising = ~spi2_done_r && spi2_done;

    assign spi2_pending_out = spi2_pending;

    always @(posedge clk) begin
        if (reset) begin
            spi2_start   <= 1'b0;
            spi2_tx_data <= 8'd0;
            spi2_pending <= 1'b0;
            spi2_tx_buf  <= 8'd0;
            spi2_busy_r  <= 1'b0;
            spi2_done_r  <= 1'b0;
            spi2_started <= 1'b0;
        end else begin
            // Sample signals to detect edges
            spi2_busy_r <= spi2_busy;
            spi2_done_r <= spi2_done;

            // Write from firmware: store data and mark pending
            if (spi2_tx_wr) begin
                spi2_tx_buf  <= DataWriteM_in;
                spi2_pending <= 1'b1;
                spi2_started <= 1'b0;
                spi2_start   <= 1'b0;
            end
            // When data is pending and SPI is not busy, assert start
            else if (spi2_pending && !spi2_started && !spi2_busy) begin
                spi2_tx_data <= spi2_tx_buf;
                spi2_start   <= 1'b1;
                spi2_started <= 1'b1;
            end
            // When SPI becomes busy (acknowledges), deassert start
            else if (spi2_started && spi2_busy_rising) begin
                spi2_start   <= 1'b0;
            end
            // When SPI finishes (done pulse), clear pending
            else if (spi2_started && spi2_done_rising) begin
                spi2_pending <= 1'b0;
                spi2_started <= 1'b0;
            end
        end
    end

    // =========================================================
    // SPI2 RX — single register (depth=1, no CircularBuffer)
    // =========================================================
    reg [7:0] spi2_rx_reg;
    reg       spi2_rx_valid;   // 1 = unread byte waiting
    reg       spi2_done_rx_r;

    wire spi2_done_rx_rise = spi2_done & ~spi2_done_rx_r;

    // read strobe — one cycle when firmware reads the RX address
    wire spi2_rx_rd = !memwriteM_in && sel_spi2_rx && spi2_rx_valid;

    always @(posedge clk) begin
        if (reset) begin
            spi2_done_rx_r  <= 1'b0;
            spi2_rx_reg     <= 8'd0;
            spi2_rx_valid   <= 1'b0;
        end else begin
            spi2_done_rx_r <= spi2_done;

            // capture incoming byte on rising edge of done
            if (spi2_done_rx_rise) begin
                spi2_rx_reg   <= spi2_rx_data;
                spi2_rx_valid <= 1'b1;
            end

            // clear valid when firmware reads it
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
            if      (sel_uart_tx)   DataMem_out = {24'd0, uart_out_data};
            else if (sel_uart_txst) DataMem_out = {30'd0, uart_tx_busy, uart_tx_pending};
            else if (sel_uart_rx)   DataMem_out = {24'd0, uart_in_data};
            else if (sel_uart_rxst) DataMem_out = {31'd0, uart_rx_ready};
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