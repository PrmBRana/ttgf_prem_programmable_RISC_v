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
    // FIFO Pointer Next Function
    // =========================================================
    function automatic [1:0] fifo_next_ptr(input [1:0] ptr);
        fifo_next_ptr = (ptr == 2'd3) ? 2'd0 : ptr + 1;
    endfunction

    // =========================================================
    // UART TX FIFO (4-deep Ring Buffer)
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
                uart_tx_fifo[uart_tx_wr_ptr] <= DataWriteM_in;
                uart_tx_wr_ptr <= fifo_next_ptr(uart_tx_wr_ptr);
            end

            if (uart_tx_pop) begin
                uart_tx_rd_ptr <= fifo_next_ptr(uart_tx_rd_ptr);
            end

            // Update flags
            if (uart_tx_wr_en && !uart_tx_pop)
                uart_tx_full  <= (fifo_next_ptr(uart_tx_wr_ptr) == uart_tx_rd_ptr);
            else if (!uart_tx_wr_en && uart_tx_pop)
                uart_tx_full  <= 1'b0;

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
    // UART RX FIFO
    // =========================================================
    reg [7:0] uart_rx_fifo [0:3];
    reg [1:0] uart_rx_wr_ptr, uart_rx_rd_ptr;
    reg       uart_rx_full, uart_rx_empty;
    reg       uart_rx_ready_r, uart_rx_ready_rr;

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
            uart_rx_wr_ptr <= 2'd0;
            uart_rx_rd_ptr <= 2'd0;
            uart_rx_full   <= 1'b0;
            uart_rx_empty  <= 1'b1;
        end else begin
            if (uart_rx_ready_rise && !uart_rx_full) begin
                uart_rx_fifo[uart_rx_wr_ptr] <= uart_in_data;
                uart_rx_wr_ptr <= fifo_next_ptr(uart_rx_wr_ptr);
            end

            if (uart_rx_rd_en) begin
                uart_rx_rd_ptr <= fifo_next_ptr(uart_rx_rd_ptr);
            end

            if (uart_rx_ready_rise && !uart_rx_rd_en && !uart_rx_full)
                uart_rx_full <= (fifo_next_ptr(uart_rx_wr_ptr) == uart_rx_rd_ptr);
            else if (!uart_rx_ready_rise && uart_rx_rd_en)
                uart_rx_full <= 1'b0;

            if (uart_rx_ready_rise && !uart_rx_rd_en)
                uart_rx_empty <= 1'b0;
            else if (!uart_rx_ready_rise && uart_rx_rd_en && 
                     fifo_next_ptr(uart_rx_rd_ptr) == uart_rx_wr_ptr)
                uart_rx_empty <= 1'b1;
        end
    end

    // =========================================================
    // SPI2 TX & RX FIFOs (same pattern)
    // =========================================================
    // SPI2 TX
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
                spi2_tx_fifo[spi2_tx_wr_ptr] <= DataWriteM_in;
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
                     fifo_next_ptr(spi2_tx_rd_ptr) == spi2_tx_wr_ptr)
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

    // SPI2 RX
    reg [7:0] spi2_rx_fifo [0:3];
    reg [1:0] spi2_rx_wr_ptr, spi2_rx_rd_ptr;
    reg       spi2_rx_full, spi2_rx_empty;
    reg       spi2_done_r;

    wire spi2_done_rise = spi2_done & ~spi2_done_r;
    wire spi2_rx_rd_en  = !memwriteM_in && sel_spi2_rx && !spi2_rx_empty;

    always @(posedge clk) begin
        if (reset) spi2_done_r <= 1'b0;
        else       spi2_done_r <= spi2_done;
    end

    always @(posedge clk) begin
        if (reset) begin
            spi2_rx_wr_ptr <= 2'd0; spi2_rx_rd_ptr <= 2'd0;
            spi2_rx_full   <= 1'b0; spi2_rx_empty  <= 1'b1;
        end else begin
            if (spi2_done_rise && !spi2_rx_full) begin
                spi2_rx_fifo[spi2_rx_wr_ptr] <= spi2_rx_data;
                spi2_rx_wr_ptr <= fifo_next_ptr(spi2_rx_wr_ptr);
            end
            if (spi2_rx_rd_en)
                spi2_rx_rd_ptr <= fifo_next_ptr(spi2_rx_rd_ptr);

            if (spi2_done_rise && !spi2_rx_rd_en && !spi2_rx_full)
                spi2_rx_full <= (fifo_next_ptr(spi2_rx_wr_ptr) == spi2_rx_rd_ptr);
            else if (!spi2_done_rise && spi2_rx_rd_en)
                spi2_rx_full <= 1'b0;

            if (spi2_done_rise && !spi2_rx_rd_en)
                spi2_rx_empty <= 1'b0;
            else if (!spi2_done_rise && spi2_rx_rd_en && 
                     fifo_next_ptr(spi2_rx_rd_ptr) == spi2_rx_wr_ptr)
                spi2_rx_empty <= 1'b1;
        end
    end

    // =========================================================
    // GPIO (unchanged)
    // =========================================================
    always @(posedge clk) begin
        if (reset) begin
            gpio1_wr_en <= 1'b0; gpio1_wdata <= 1'b1;
        end else begin
            gpio1_wr_en <= 1'b0;
            if (memwriteM_in && sel_gpio1) begin
                gpio1_wdata <= DataWriteM_in[0];
                gpio1_wr_en <= 1'b1;
            end
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            gpio2_wr_en <= 1'b0; gpio2_wdata <= 1'b1;
        end else begin
            gpio2_wr_en <= 1'b0;
            if (memwriteM_in && sel_gpio2) begin
                gpio2_wdata <= DataWriteM_in[0];
                gpio2_wr_en <= 1'b1;
            end
        end
    end

    // =========================================================
    // READ MUX - Fixed width
    // =========================================================
    always @(*) begin
        DataMem_out = 32'h0000_0000;

        if (!memwriteM_in) begin
            if      (sel_uart_txst)  
                DataMem_out = {29'd0, uart_tx_full, uart_tx_busy, !uart_tx_empty};
            else if (sel_uart_rx)    
                DataMem_out = {24'd0, uart_rx_fifo[uart_rx_rd_ptr]};
            else if (sel_uart_rxst)  
                DataMem_out = {29'd0, uart_rx_full, 1'b0, !uart_rx_empty};   // fixed

            else if (sel_spi2_tx)    
                DataMem_out = {24'd0, spi2_tx_fifo[spi2_tx_rd_ptr]};
            else if (sel_spi2_txst)  
                DataMem_out = {29'd0, spi2_tx_full, spi2_busy, !spi2_tx_empty};
            else if (sel_spi2_rx)    
                DataMem_out = {24'd0, spi2_rx_fifo[spi2_rx_rd_ptr]};
            else if (sel_spi2_rxst)  
                DataMem_out = {29'd0, spi2_rx_full, 1'b0, !spi2_rx_empty};   // fixed
        end
    end

endmodule

`default_nettype wire