`default_nettype none

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

    // GPIO (write-only)
    output reg         gpio1_wr_en,
    output reg         gpio1_wdata,
    output reg         gpio2_wr_en,
    output reg         gpio2_wdata
);

    // ========================================================
    // MINIMAL ADDRESS DECODE - Only used signals
    // ========================================================
    wire [19:0] region = aluAddress_in[31:12];
    wire [1:0]  word   = aluAddress_in[3:2];
    
    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused_addr = &{1'b0, aluAddress_in[11:0]};
    /* verilator lint_on UNUSEDSIGNAL */

    // Only signals that are actually USED:
    wire is_uart  = (region == 20'h10000);
    wire is_spi2  = (region == 20'h40000);
    
    wire sel_uart_tx = is_uart  & (word == 2'b00);
    wire sel_uart_rx = is_uart  & (word == 2'b01);
    
    wire sel_spi2_tx = is_spi2  & (word == 2'b00);
    wire sel_spi2_rx = is_spi2  & (word == 2'b10);
    
    wire sel_gpio1  = (region == 20'h30000) & (word == 2'b00);
    wire sel_gpio2  = (region == 20'h30000) & (word == 2'b01);

    // ========================================================
    // SPI2 done edge detect (minimal)
    // ========================================================
    reg spi2_done_r;
    always @(posedge clk) 
        if (reset) spi2_done_r <= 1'b0;
        else       spi2_done_r <= spi2_done;
    wire spi2_done_rise = spi2_done & ~spi2_done_r;

    // ========================================================
    // UART TX - Minimal state machine
    // ========================================================
    reg [7:0] uart_tx_reg;
    reg       uart_tx_pending;

    wire uart_tx_wr = memwriteM_in & sel_uart_tx & ~uart_tx_pending;

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
            end else if (uart_tx_pending & ~uart_tx_busy) begin
                uart_out_data   <= uart_tx_reg;
                uart_tx_start   <= 1'b1;
                uart_tx_pending <= 1'b0;
            end
        end
    end

    // ========================================================
    // UART RX
    // ========================================================
    reg [7:0] uart_rx_reg;
    reg       uart_rx_valid;

    always @(posedge clk) begin
        if (reset) begin
            uart_rx_reg   <= 8'd0;
            uart_rx_valid <= 1'b0;
        end else begin
            if (uart_rx_ready) begin
                uart_rx_reg   <= uart_in_data;
                uart_rx_valid <= 1'b1;
            end else if (~memwriteM_in & sel_uart_rx & uart_rx_valid) begin
                uart_rx_valid <= 1'b0;
            end
        end
    end

    // ========================================================
    // SPI2 TX
    // ========================================================
    reg [7:0] spi2_tx_buf;
    reg       spi2_pending_reg;

    wire spi2_tx_wr = memwriteM_in & sel_spi2_tx & ~spi2_pending_reg;
    assign spi2_pending_out = spi2_pending_reg;

    always @(posedge clk) begin
        if (reset) begin
            spi2_start      <= 1'b0;
            spi2_tx_data    <= 8'd0;
            spi2_pending_reg<= 1'b0;
            spi2_tx_buf     <= 8'd0;
        end else begin
            spi2_start <= 1'b0;
            if (spi2_tx_wr) begin
                spi2_tx_buf     <= DataWriteM_in;
                spi2_pending_reg<= 1'b1;
            end else if (spi2_pending_reg & ~spi2_busy & ~spi2_done) begin
                spi2_tx_data    <= spi2_tx_buf;
                spi2_start      <= 1'b1;
                spi2_pending_reg<= 1'b0;
            end
        end
    end

    // ========================================================
    // SPI2 RX
    // ========================================================
    reg [7:0] spi2_rx_reg;
    reg       spi2_rx_valid;

    always @(posedge clk) begin
        if (reset) begin
            spi2_rx_reg   <= 8'd0;
            spi2_rx_valid <= 1'b0;
        end else begin
            if (spi2_done_rise) begin
                spi2_rx_reg   <= spi2_rx_data;
                spi2_rx_valid <= 1'b1;
            end else if (~memwriteM_in & sel_spi2_rx & spi2_rx_valid) begin
                spi2_rx_valid <= 1'b0;
            end
        end
    end

    // ========================================================
    // GPIO1 Write-Only
    // ========================================================
    always @(posedge clk) begin
        if (reset) begin
            gpio1_wr_en <= 1'b0;
            gpio1_wdata <= 1'b1;
        end else begin
            gpio1_wr_en <= 1'b0;
            if (memwriteM_in & sel_gpio1) begin
                gpio1_wdata <= DataWriteM_in[0];
                gpio1_wr_en <= 1'b1;
            end
        end
    end

    // ========================================================
    // GPIO2 Write-Only
    // ========================================================
    always @(posedge clk) begin
        if (reset) begin
            gpio2_wr_en <= 1'b0;
            gpio2_wdata <= 1'b1;
        end else begin
            gpio2_wr_en <= 1'b0;
            if (memwriteM_in & sel_gpio2) begin
                gpio2_wdata <= DataWriteM_in[0];
                gpio2_wr_en <= 1'b1;
            end
        end
    end

    // ========================================================
    // READ MUX - Optimized casez (Verilator happy)
    // ========================================================
    always @(*) begin
        DataMem_out = 32'h0000_0000;
        if (~memwriteM_in) begin
            casez ({is_uart, is_spi2})
                2'b10: begin  // UART region
                    casez (word)
                        2'b00: DataMem_out = {30'd0, uart_tx_busy, uart_tx_pending};
                        2'b01: DataMem_out = {24'd0, uart_rx_reg};
                        2'b1?: DataMem_out = {30'd0, 1'b0, uart_rx_valid};
                    endcase
                end
                2'b01: begin  // SPI2 region
                    casez (word)
                        2'b00:    DataMem_out = {24'd0, spi2_tx_buf};
                        2'b01:    DataMem_out = {30'd0, spi2_pending_reg, spi2_busy};
                        2'b10:    DataMem_out = {24'd0, spi2_rx_reg};
                        2'b11:    DataMem_out = {30'd0, 1'b0, spi2_rx_valid};
                    endcase
                end
                default:      DataMem_out = 32'h0000_0000;
            endcase
        end
    end

endmodule

`default_nettype wire