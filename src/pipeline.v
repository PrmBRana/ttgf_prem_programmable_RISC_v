`default_nettype none
`timescale 1ns/1ps

// ============================================================
//  pipeline.v — RISC-V 5-stage pipeline with UART bootloader,
//                SPI2, GPIO for GF180MCU tapeout
//  Memory: 32 words (128 bytes) IMEM + DMEM
// ============================================================

module pipeline (
    input  wire clk,
    input  wire reset,          // asynchronous reset
    input  wire rx,
    output wire tx,
    output wire spi2_sclk,
    output wire spi2_mosi,
    input  wire spi2_miso,
    output wire spi2_cs_n
);

    localparam IMEM_DEPTH = 32;
    localparam DMEM_DEPTH = 32;
    localparam ADDR_W     = 5;   // 5 bits → 32 words

    // =========================================================
    // RESET SYNCHRONIZER
    // =========================================================
    reg reset_ff1, reset_ff2;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            reset_ff1 <= 1'b1;
            reset_ff2 <= 1'b1;
        end else begin
            reset_ff1 <= 1'b0;
            reset_ff2 <= reset_ff1;
        end
    end
    wire reset_sync = reset_ff2;

    // =========================================================
    // PIPELINE SIGNALS
    // =========================================================
    wire [31:0] PCPLUS4_top, PC_top, PCF;
    wire [31:0] Instruction1_out, INSTRUCTION;
    wire [31:0] RD1_top, RD2_top, PCD_top, PCPLUS4D_TOP, ImmExtD_top;
    wire [31:0] RD1E_top, RD2E_top, ImmExtE_top, PCE_top, PCPlus4E_top;
    wire [31:0] SrcA_top, outB_top, ScrB_top;
    wire [31:0] ALUResultE_top, PCTarget_top;
    wire [31:0] ALUResultM_top, PCPlus4M_top, WriteDataM_top;
    wire [31:0] Datamem_top;
    wire [31:0] ALUResultW_top, ReadDataW_top, PCPlus4W_top, ResultW_top;

    wire        RegWrite_top, ALUSrcD_top, memWriteD_top;
    wire        jumpD_top, BranchD_top, jumpRD_top;
    wire        JumpE_top, BranchE_top, zero_top, PCSCR_top, JumpRE_top;
    wire        RegWriteE_top, MemWriteE_top, ALUSrcE_top;
    wire        MemWriteM_top, RegWriteM_top, RegWriteW_top;
    wire        StallF_top, StallD_top, FlushD_top, FlushE_top;
    wire        halt_top;

    wire [1:0]  ResultSrcD_top, ALUtyp_top, ALUTypE_top;
    wire [1:0]  ResultSrcE_top, ResultSrcM_top, ResultSrcW_top;
    wire [1:0]  ForwardAE_top, ForwardBE_top;
    wire [3:0]  ALUControlD_top, ALUControlE_top;
    wire [4:0]  RdE_top, RdM_top, Rs1E_top, Rs2E_top, RdW_top;
    wire [2:0]  ImmSrc_top;
    wire [1:0]  ALUSrcAD_top, ALUSrcAE_top;

    wire [2:0]  funct3D_top, funct3E_top, funct3M_top;
    wire [3:0]  byte_en_M;

    // Bootloader & Peripherals
    wire [7:0]  uart_rx_data_shared;
    wire        uart_rx_ready_shared;
    wire        uart_tx_busy_shared;
    wire [7:0]  boot_tx_data;
    wire        boot_tx_start_raw;
    wire [ADDR_W-1:0] mem_addr;
    wire [31:0]       mem_wdata;
    wire              Write_enable;
    wire              stall_Pro;

    wire [7:0]  periph_tx_data_w;
    wire        periph_tx_start_w;

    wire        spi2_start_w, spi2_busy_w, spi2_done_w, spi2_pending_w;
    wire [7:0]  spi2_tx_data_w, spi2_rx_data_w;
    wire        gpio2_wr_en_w, gpio2_wdata_w;

    // =========================================================
    // BOOT CONTROL REGISTERS (improved fanout)
    // =========================================================
    reg stall_pro_r;
    always @(posedge clk or posedge reset_sync) begin
        if (reset_sync) stall_pro_r <= 1'b1;
        else            stall_pro_r <= stall_Pro;
    end

    reg boot_done_r;
    always @(posedge clk or posedge reset_sync) begin
        if (reset_sync) boot_done_r <= 1'b0;
        else            boot_done_r <= ~stall_pro_r;
    end

    // TX MUX
    wire boot_tx_active = boot_tx_start_raw & ~boot_done_r;

    reg        periph_tx_start_r;
    reg [7:0]  periph_tx_data_r;
    always @(posedge clk or posedge reset_sync) begin
        if (reset_sync) begin
            periph_tx_start_r <= 1'b0;
            periph_tx_data_r  <= 8'd0;
        end else begin
            periph_tx_start_r <= periph_tx_start_w;
            periph_tx_data_r  <= periph_tx_data_w;
        end
    end

    wire [7:0] shared_tx_data  = boot_done_r ? periph_tx_data_r  : boot_tx_data;
    wire       shared_tx_start = boot_done_r ? periph_tx_start_r : boot_tx_active;

    wire uart_rx_ready_boot = uart_rx_ready_shared & ~boot_done_r;

    // =========================================================
    // HALT LOGIC (FIXED: combined into single always block)
    // =========================================================
    reg halt_detected_r, halt_reg;
    always @(posedge clk or posedge reset_sync) begin
        if (reset_sync | stall_pro_r) begin
            halt_detected_r <= 1'b0;
            halt_reg        <= 1'b0;
        end else begin
            halt_detected_r <= halt_top & ~FlushD_top;
            if (halt_detected_r)
                halt_reg <= 1'b1;
        end
    end

    wire halt_final = halt_reg | halt_detected_r;

    // STALL / FLUSH
    wire StallF_net = PCSCR_top ? 1'b0 : (stall_pro_r | StallF_top | halt_final);
    wire StallD_net = PCSCR_top ? 1'b0 : (stall_pro_r | StallD_top | halt_final);

    wire MemWriteM_gated = MemWriteM_top & ~stall_pro_r;

    // BYTE ENABLE GENERATION
    assign byte_en_M = (funct3M_top == 3'b010) ? 4'b1111 :  // SW
                       (funct3M_top == 3'b001) ? 4'b0011 :  // SH
                       (funct3M_top == 3'b000) ? 4'b0001 :  // SB
                                                 4'b0000;   // safe default (no write)

    // =========================================================
    // UART
    // =========================================================
    uart_Tx_fixed #(
        .CLK_FREQ(50_000_000),
        .BAUD_RATE(115_200),
        .OVERSAMPLE(16)
    ) uart_shared_inst (
        .clk      (clk),
        .reset    (reset_sync),
        .tx_Start (shared_tx_start),
        .tx_Data  (shared_tx_data),
        .tx       (tx),
        .tx_busy  (uart_tx_busy_shared),
        .rx       (rx),
        .rx_Data  (uart_rx_data_shared),
        .rx_ready (uart_rx_ready_shared)
    );

    // =========================================================
    // BOOTLOADER
    // =========================================================
    uart_bootloader uart_boot_inst (
        .clk      (clk),
        .reset    (reset_sync),
        .rx_data  (uart_rx_data_shared),
        .rx_valid (uart_rx_ready_boot),
        .tx_data  (boot_tx_data),
        .tx_start (boot_tx_start_raw),
        .mem_we   (Write_enable),
        .mem_addr (mem_addr),
        .mem_wdata(mem_wdata),
        .stall_pro(stall_Pro)
    );

    // =========================================================
    // INSTRUCTION MEMORY
    // =========================================================
    wire [ADDR_W-1:0] PC_word_idx = PCF[ADDR_W+1:2];

    instruction_mem #(
        .DEPTH(IMEM_DEPTH),
        .ADDR_W(ADDR_W)
    ) imem_inst (
        .clk            (clk),
        .reset          (reset_sync),
        .we             (Write_enable),
        .addr           (mem_addr),
        .wdata          (mem_wdata),
        .read_word_idx  (PC_word_idx),
        .Instruction_out(Instruction1_out)
    );

    // =========================================================
    // FETCH
    // =========================================================
    PC_incre PC_inc (.pc(PCF), .PCPlus4(PCPLUS4_top));

    PCSelect_MUX PCSelect_top (
        .PCScr(PCSCR_top),
        .PCSequential(PCPLUS4_top),
        .PCBranch(PCTarget_top),
        .Mux3_PC(PC_top)
    );

    pc_register Register_top (
        .clk(clk),
        .reset(reset_sync),
        .PCF_in(PC_top),
        .stallF(StallF_net),
        .PCF_out(PCF)
    );

    // IF/ID Stage
    IF_ID_stage IF_ID_top (
        .clk(clk),
        .reset(reset_sync),
        .stallD(StallD_net),
        .flushD(FlushD_top),
        .PC_in(PCF),
        .PCplus4_in(PCPLUS4_top),
        .instruction_in(Instruction1_out),
        .instruction_out(INSTRUCTION),
        .PCplus4_out(PCPLUS4D_TOP),
        .PC_out(PCD_top)
    );

    wire [6:0] INSTR_op  = INSTRUCTION[6:0];
    wire [2:0] INSTR_f3  = INSTRUCTION[14:12];
    wire [6:0] INSTR_f7  = INSTRUCTION[31:25];
    wire [4:0] INSTR_rs1 = INSTRUCTION[19:15];
    wire [4:0] INSTR_rs2 = INSTRUCTION[24:20];
    wire [4:0] INSTR_rd  = INSTRUCTION[11:7];

    assign funct3D_top = INSTR_f3;

    // Control Unit
    Control control (
        .Opcode(INSTR_op), .funct3(INSTR_f3), .funct7(INSTR_f7),
        .imm(INSTRUCTION[31:20]), .halt(halt_top),
        .RegWriteD(RegWrite_top), .ResultSrcD(ResultSrcD_top),
        .MemWriteD(memWriteD_top), .jumpD(jumpD_top), .jumpR(jumpRD_top),
        .BranchD(BranchD_top), .ALUControlD(ALUControlD_top),
        .ALUSrcD(ALUSrcD_top), .ALUSrcA(ALUSrcAD_top),
        .ImmSrc(ImmSrc_top), .ALUType(ALUtyp_top)
    );

    // Register File
    Reg_file Reg_file_top (
        .clk(clk),
        .rs1_addr(INSTR_rs1), .rs2_addr(INSTR_rs2),
        .rd_addr(RdW_top), .Regwrite(RegWriteW_top),
        .Write_data(ResultW_top),
        .Read_data1(RD1_top), .Read_data2(RD2_top)
    );

    // Immediate Extension
    imm imm_top (
        .ImmSrc(ImmSrc_top),
        .instruction(INSTRUCTION),
        .ImmExt(ImmExtD_top)
    );

    // EX Stage (with funct3 propagation)
    EX_stage ex_stage (
        .clk(clk), .reset(reset_sync), .flushE(FlushE_top),
        .RD1D_in(RD1_top), .RD2D_in(RD2_top),
        .ImmExtD_in(ImmExtD_top), .PCPlus4D_in(PCPLUS4D_TOP),
        .PC_D_in(PCD_top), .Rs1D_in(INSTR_rs1), .Rs2D_in(INSTR_rs2),
        .RdD_in(INSTR_rd), .ALUControlD_in(ALUControlD_top),
        .ALUSrcD_in(ALUSrcD_top), .ALUSrcA_in(ALUSrcAD_top),
        .RegWriteD_in(RegWrite_top), .ResultSrcD_in(ResultSrcD_top),
        .MemWriteD_in(memWriteD_top), .BranchD_in(BranchD_top),
        .JumpD_in(jumpD_top), .JumpR_in(jumpRD_top),
        .ALUType_in(ALUtyp_top), .funct3D_in(funct3D_top),
        .RD1E_out(RD1E_top), .RD2E_out(RD2E_top),
        .ImmExtD_out(ImmExtE_top), .PCPlus4D_out(PCPlus4E_top),
        .PC_D_out(PCE_top), .Rs1D_out(Rs1E_top), .Rs2D_out(Rs2E_top),
        .RdD_out(RdE_top), .ALUControlD_out(ALUControlE_top),
        .ALUSrcD_out(ALUSrcE_top), .ALUSrcA_out(ALUSrcAE_top),
        .RegWriteD_out(RegWriteE_top), .ResultSrcD_out(ResultSrcE_top),
        .MemWriteD_out(MemWriteE_top), .BranchD_out(BranchE_top),
        .JumpD_out(JumpE_top), .JumpR_out(JumpRE_top),
        .ALUType_out(ALUTypE_top), .funct3D_out(funct3E_top)
    );

    // Forwarding
    wire [31:0] SrcA_fwd = (ForwardAE_top == 2'b10) ? ALUResultM_top :
                           (ForwardAE_top == 2'b01) ? ResultW_top    : RD1E_top;

    assign SrcA_top = (ALUSrcAE_top == 2'b10) ? 32'd0 :
                      (ALUSrcAE_top == 2'b01) ? PCE_top : SrcA_fwd;

    assign outB_top = (ForwardBE_top == 2'b10) ? ALUResultM_top :
                      (ForwardBE_top == 2'b01) ? ResultW_top    : RD2E_top;

    assign ScrB_top = ALUSrcE_top ? ImmExtE_top : outB_top;

    // PC Target (JALR fix)
    wire [31:0] base_addr_w = JumpRE_top ? SrcA_fwd : PCE_top;
    assign PCTarget_top = JumpRE_top ?
        ((base_addr_w + ImmExtE_top) & 32'hFFFF_FFFE) :
          (base_addr_w + ImmExtE_top);

    assign PCSCR_top = (zero_top & BranchE_top) | JumpE_top;

    // ALU
    ALU alu (
        .ScrA(SrcA_top), .ScrB(ScrB_top),
        .ALUControl(ALUControlE_top), .ALUType(ALUTypE_top),
        .ALUResult(ALUResultE_top), .Zero(zero_top)
    );

    // MEM Stage
    MEM_stage mem_stage (
        .clk(clk), .reset(reset_sync),
        .ALUResult_in(ALUResultE_top), .WriteData_in(outB_top),
        .RdM_in(RdE_top), .PCPlus4M_in(PCPlus4E_top),
        .RegWriteM_in(RegWriteE_top), .ResultSrcM_in(ResultSrcE_top),
        .MemWriteM_in(MemWriteE_top), .funct3M_in(funct3E_top),
        .ALUResult_out(ALUResultM_top), .WriteData_out(WriteDataM_top),
        .RdM_out(RdM_top), .PCPlus4M_out(PCPlus4M_top),
        .RegWriteM_out(RegWriteM_top), .ResultSrcM_out(ResultSrcM_top),
        .MemWriteM_out(MemWriteM_top), .funct3M_out(funct3M_top)
    );

    // WB Stage
    WriteBack_stage writeback_stage (
        .clk(clk), .reset(reset_sync),
        .ALUResultW_in(ALUResultM_top), .ReadDataW_in(Datamem_top),
        .RdW_in(RdM_top), .PCPlus4W_in(PCPlus4M_top),
        .RegWriteW_in(RegWriteM_top), .ResultSrcW_in(ResultSrcM_top),
        .ALUResultW_out(ALUResultW_top), .ReadDataW_out(ReadDataW_top),
        .RdW_out(RdW_top), .PCPlus4W_out(PCPlus4W_top),
        .RegWriteW_out(RegWriteW_top), .ResultSrcW_out(ResultSrcW_top)
    );

    Write_back write_back (
        .ALUResultW_in(ALUResultW_top),
        .ReadDataW_in(ReadDataW_top),
        .PCPlus4W_in(PCPlus4W_top),
        .ResultSrcW_in(ResultSrcW_top),
        .ResultW(ResultW_top)
    );

    // Hazard Unit
    Hazard_Unit hazard (
        .Rs1D(INSTR_rs1), .Rs2D(INSTR_rs2),
        .Rs1E(Rs1E_top), .Rs2E(Rs2E_top), .RdE(RdE_top),
        .RegWriteE(RegWriteE_top), .PCSRCE(PCSCR_top),
        .ResultSrcE_in(ResultSrcE_top),
        .RdM(RdM_top), .RdW(RdW_top),
        .RegWriteM(RegWriteM_top), .RegWriteW(RegWriteW_top),
        .StallF(StallF_top), .StallD(StallD_top),
        .FlushD(FlushD_top), .FlushE(FlushE_top),
        .Forward_AE(ForwardAE_top), .Forward_BE(ForwardBE_top)
    );

    // =========================================================
    // DATA MEMORY + PERIPHERALS
    // =========================================================
    DataMem #(
        .DMEM_DEPTH(DMEM_DEPTH),
        .DMEM_ADDR_W(ADDR_W)
    ) databus_inst (
        .clk             (clk),
        .reset           (reset_sync),
        .aluAddress_in   (ALUResultM_top),
        .DataWriteM_in   (WriteDataM_top),
        .byte_en         (byte_en_M),
        .memwriteM_in    (MemWriteM_gated),
        .DataMem_out     (Datamem_top),
        .uart_out_data   (periph_tx_data_w),
        .uart_tx_start   (periph_tx_start_w),
        .uart_tx_busy    (uart_tx_busy_shared),
        .spi2_tx_data    (spi2_tx_data_w),
        .spi2_start      (spi2_start_w),
        .spi2_pending_out(spi2_pending_w),
        .spi2_rx_data    (spi2_rx_data_w),
        .spi2_busy       (spi2_busy_w),
        .spi2_done       (spi2_done_w),
        .gpio2_wr_en     (gpio2_wr_en_w),
        .gpio2_wdata     (gpio2_wdata_w)
    );

    // SPI Master
    spi_master #(
        .DATA_WIDTH(8), .CPOL(0), .CPHA(0), .CLK_DIV(8)
    ) spi2_inst (
        .clk(clk), .reset(reset_sync),
        .start(spi2_start_w), .tx_data(spi2_tx_data_w),
        .rx_data(spi2_rx_data_w), .busy(spi2_busy_w),
        .done(spi2_done_w), .sclk(spi2_sclk),
        .mosi(spi2_mosi), .miso(spi2_miso)
    );

    // GPIO for SPI CS
    gpio2_io gpio2 (
        .clk(clk), .reset(reset_sync),
        .wr_en2(gpio2_wr_en_w), .wdata2(gpio2_wdata_w),
        .spi_busy(spi2_busy_w), .spi_pending(spi2_pending_w),
        .gpio_out2(spi2_cs_n)
    );

endmodule














