`default_nettype none
`timescale 1ns / 1ps

// ============================================================
//  pipeline.v  —  ASIC TAPEOUT FIX (Fanout + Antenna)
//
//  VIOLATIONS FIXED IN THIS VERSION:
//
//  1. MAX FANOUT (215 violations on clkbuf_leaf_* cells)
//     ─────────────────────────────────────────────────────
//     ROOT CAUSE: The original code had several wide-fanout
//     combinational signals driving 10+ loads without buffering:
//       • stall_Pro  → drives 5+ logic cones simultaneously
//       • halt_final → drives StallF_net and StallD_net which
//                      each fan out into wide logic trees
//       • FlushD_top → directly used in halt_detected_r AND
//                      passed to IF_ID_stage
//       • PCSCR_top  → combinational, wide fan-out to stall nets
//
//     FIX: Insert explicit registered/buffered intermediate nets
//     for all signals that drive >4 loads.  The synthesis tool
//     and resizer can handle small fanout, but registered staging
//     helps the CTS/resizer not fight the clock tree for resources.
//
//     Additionally, the config now uses:
//       • SYNTH_MAX_FANOUT: 8  (tighter budget)
//       • CTS_SINK_CLUSTERING_SIZE: 16  (fewer FFs per leaf buf)
//       • CTS_CLK_BUFFER_LIST: includes clkbuf_4/8/16  (allows
//         smaller leaves for high-fanout leaf nodes)
//       • GLB_RESIZER_MAX_FANOUT_FIX_PASSES: 10
//
//  2. ANTENNA VIOLATIONS (56 pin / 53 net violations)
//     ─────────────────────────────────────────────────────
//     ROOT CAUSE: Long combinational paths (especially the
//     stall/halt/pcscr chains crossing the full die width)
//     create long unbroken metal runs during routing.  These
//     accumulate plasma-charge damage on gate oxides.
//
//     FIX 1 (RTL): Break long combinational chains at natural
//     pipeline boundaries by registering intermediate values
//     (stall_pro_r, pcscr_r).  Registered signals are driven
//     from FF outputs — short local connections — so the router
//     can place the FF near the loads and avoid long metal runs.
//
//     FIX 2 (Config): DIODE_INSERTION_STRATEGY 4 enables
//     OpenLane's full antenna-diode insertion flow (filler-based
//     and endcap-based) plus ANTENNA_CHECK_MARGIN 2 adds 2×
//     safety margin.  GRT_MAX_DIODE_INS_ITERS 15 gives the
//     router more passes to resolve remaining violations.
//
//  TIMING NOTE:
//     Registering stall_Pro and PCSCR_top adds ONE cycle of
//     latency to stall/branch taken response.
//
//     • stall_Pro:  bootloader stall is already many cycles long
//       (UART byte-by-byte), so one extra cycle is invisible.
//     • PCSCR_top (branch/jump):  the pipeline already inserts
//       two flush bubbles (FlushD + FlushE) for taken branches.
//       Adding one registered-PCSCR stage means the flush now
//       propagates correctly one cycle later — the branch penalty
//       increases from 2 to 3 cycles.  This is architecturally
//       safe but you must ensure your software does not assume
//       a 2-cycle branch shadow.  If 2-cycle is required, remove
//       the pcscr_r stage and accept the antenna risk (mitigated
//       by the diode insertion in the config).
// ============================================================

module pipeline (
    input  wire clk,
    input  wire reset,
    input  wire rx,
    output wire tx,
    output wire spi2_sclk,
    output wire spi2_mosi,
    input  wire spi2_miso,
    output wire spi2_cs_n
);

    localparam IMEM_ADDR_W = 6;

    // =========================================================
    // RESET SYNCHRONISER (unchanged)
    // =========================================================
    reg reset_ff1, reset_ff2;
    always @(posedge clk) begin
        if (reset) begin
            reset_ff1 <= 1'b1;
            reset_ff2 <= 1'b1;
        end else begin
            reset_ff1 <= 1'b0;
            reset_ff2 <= reset_ff1;
        end
    end
    wire reset_sync = reset_ff2;

    // ── Pipeline wires ─────────────────────────────────────────
    wire [31:0] PCPLUS4_top, PC_top, PCF, Instruction1_out, INSTRUCTION;
    wire [31:0] RD1_top, RD2_top, PCD_top, PCE_top, PCPLUS4D_TOP;
    wire [31:0] RD1E_top, RD2E_top;
    wire [31:0] SrcA_top, outB_top, ScrB_top;
    wire [31:0] ALUResultE_top, PCPlus4E_top, ALUResultM_top, PCPlus4M_top;
    wire [31:0] Datamem_top, ALUResultW_top, ReadDataW_top, PCPlus4W_top, ResultW_top;
    wire [31:0] PCTarget_top, ImmExtD_top, ImmExtE_top;

    /* verilator lint_off UNUSEDSIGNAL */
    wire [31:0] WriteDataM_top;
    /* verilator lint_on  UNUSEDSIGNAL */

    wire RegWrite_top, ALUSrcD_top, memWriteD_top, jumpD_top, BranchD_top;
    wire JumpE_top, BranchE_top, zero_top, PCSCR_top;
    wire jumpRD_top, JumpRE_top;
    wire RegWriteE_top, MemWriteE_top, ALUSrcE_top;
    wire MemWriteM_top, RegWriteM_top, RegWriteW_top;
    wire StallF_top, StallD_top, FlushD_top, FlushE_top;

    wire [1:0] ResultSrcD_top, ALUtyp_top, ALUTypE_top;
    wire [1:0] ResultSrcE_top, ResultSrcM_top, ResultSrcW_top;
    wire [1:0] ForwardAE_top, ForwardBE_top;
    wire [3:0] ALUControlD_top, ALUControlE_top;
    wire [4:0] RdE_top, RdM_top, Rs1E_top, Rs2E_top, RdW_top;
    wire [2:0] ImmSrc_top;
    wire [1:0] ALUSrcAD_top, ALUSrcAE_top;

    wire [7:0]  uart_rx_data_shared;
    wire        uart_rx_ready_shared;
    wire        uart_tx_busy_shared;
    wire [7:0]  boot_tx_data;
    wire        boot_tx_start_raw;
    wire [7:0]  periph_tx_data;
    wire        periph_tx_start;

    wire [IMEM_ADDR_W-1:0] mem_addr;
    wire [31:0] mem_wdata;
    wire        Write_enable;
    wire        stall_Pro;
    wire        halt_top;

    // =========================================================
    // FANOUT FIX: Register stall_Pro before it fans out.
    //
    // stall_Pro (from uart_bootloader) is a combinational signal
    // that drives: boot_done_r logic, StallF_net, StallD_net,
    // halt_detected_r clear, halt_latch clear — 5+ cones.
    //
    // Registering it once (stall_pro_r) means all downstream
    // logic sees a flip-flop output that the placer puts near
    // the loads, shortening metal and reducing fanout stress.
    //
    // boot_done_r already acts as a registered version for the
    // UART gating — we reuse that path and add stall_pro_r for
    // the hazard/halt cones.
    // =========================================================
    reg stall_pro_r;
    always @(posedge clk) begin
        if (reset) stall_pro_r <= 1'b1;   // safe: stall during reset
        else       stall_pro_r <= stall_Pro;
    end

    // =========================================================
    // boot_done_r / boot_done_r2 (unchanged functional intent)
    // =========================================================
    wire boot_done_comb = ~stall_Pro;
    reg  boot_done_r;
    always @(posedge clk) begin
        if (reset) boot_done_r <= 1'b0;
        else       boot_done_r <= boot_done_comb;
    end

    reg boot_done_r2;
    always @(posedge clk) begin
        if (reset) boot_done_r2 <= 1'b0;
        else       boot_done_r2 <= boot_done_r;
    end

    wire boot_tx_start_gated = boot_tx_start_raw & ~boot_done_r2;
    wire [7:0] shared_tx_data  = boot_done_r2 ? periph_tx_data : boot_tx_data;
    wire       shared_tx_start = boot_done_r2
                                 ? (periph_tx_start & ~uart_tx_busy_shared)
                                 : boot_tx_start_gated;
    wire uart_rx_ready_boot = uart_rx_ready_shared & ~boot_done_r2;

    // =========================================================
    // HALT LOGIC — two-stage registered (Error #2 fix, unchanged)
    // =========================================================
    reg halt_detected_r;
    always @(posedge clk) begin
        if (reset || stall_pro_r)           // use registered stall
            halt_detected_r <= 1'b0;
        else
            halt_detected_r <= halt_top & ~FlushD_top;
    end

    reg halt_latch;
    always @(posedge clk) begin
        if (reset || stall_pro_r)           // use registered stall
            halt_latch <= 1'b0;
        else if (halt_detected_r)
            halt_latch <= 1'b1;
    end

    wire halt_final = halt_latch | halt_detected_r;

    // =========================================================
    // FANOUT + ANTENNA FIX: Register PCSCR before stall nets.
    //
    // PCSCR_top is combinational (zero_top & BranchE_top | JumpE_top).
    // It fans into both StallF_net and StallD_net and their
    // associated override logic.  Routing this combinationally
    // across the die creates long metal (antenna) and high fanout.
    //
    // pcscr_r: registered copy used by the stall nets below.
    // PCSCR_top: still used by the PC-select MUX (combinational,
    //   local, one load) — timing-critical, must stay fast.
    //
    // Trade-off: stall/flush asserts one cycle after branch.
    // Pipeline effect: 3-cycle branch penalty instead of 2.
    // =========================================================
    reg pcscr_r;
    always @(posedge clk) begin
        if (reset) pcscr_r <= 1'b0;
        else       pcscr_r <= PCSCR_top;
    end

    // ── Stall / flush nets ──────────────────────────────────────
    // Use pcscr_r (registered) to avoid long combinational paths
    // that cause antenna violations on StallF/StallD nets.
    // halt_final is already fully registered — safe to use directly.
    // stall_pro_r is registered — safe to use directly.
    wire StallF_net = pcscr_r ? 1'b0 : (stall_pro_r | StallF_top | halt_final);
    wire StallD_net = pcscr_r ? 1'b0 : (stall_pro_r | StallD_top | halt_final);

    // =========================================================
    // FETCH (unchanged)
    // =========================================================
    PC_incre PC (
        .pc(PCF), .PCPlus4(PCPLUS4_top));

    // PCSelect_MUX: still uses PCSCR_top (combinational) for
    // correct branch target selection this cycle.
    PCSelect_MUX PCSelect_top (
        .PCScr(PCSCR_top), .PCSequential(PCPLUS4_top),
        .PCBranch(PCTarget_top), .Mux3_PC(PC_top));

    pc_register Register_top (
        .clk(clk), .reset(reset_sync),
        .PCF_in(PC_top), .stallF(StallF_net), .PCF_out(PCF));

    // =========================================================
    // SHARED UART (unchanged)
    // =========================================================
    uart_Tx_fixed #(
        .CLK_FREQ(50_000_000), .BAUD_RATE(115_200), .OVERSAMPLE(16)
    ) uart_shared_inst (
        .clk(clk), .reset(reset_sync),
        .tx_Start(shared_tx_start), .tx_Data(shared_tx_data),
        .tx(tx), .tx_busy(uart_tx_busy_shared),
        .rx(rx), .rx_Data(uart_rx_data_shared),
        .rx_ready(uart_rx_ready_shared));

    // =========================================================
    // BOOTLOADER (unchanged)
    // =========================================================
    uart_bootloader uart_bootloader (
        .clk(clk), .reset(reset),
        .rx_data(uart_rx_data_shared),
        .rx_valid(uart_rx_ready_boot),
        .tx_data(boot_tx_data),
        .tx_start(boot_tx_start_raw),
        .mem_we(Write_enable),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .stall_pro(stall_Pro));

    // =========================================================
    // IMEM (Error #1 fix — synchronous read, unchanged)
    // =========================================================
    wire [IMEM_ADDR_W-1:0] PC_word_idx;
    assign PC_word_idx = PCF[IMEM_ADDR_W+1:2];

    instruction_mem #(
        .DEPTH(64), .ADDR_W(IMEM_ADDR_W)
    ) imem_inst (
        .clk(clk),
        .we(Write_enable),
        .addr(mem_addr),
        .wdata(mem_wdata),
        .read_word_idx(PC_word_idx),
        .Instruction_out(Instruction1_out));

    // =========================================================
    // DECODE (unchanged)
    // =========================================================
    IF_ID_stage IF_DF_top (
        .clk(clk), .reset(reset_sync),
        .stallD(StallD_net), .flushD(FlushD_top),
        .PC_in(PCF), .PCplus4_in(PCPLUS4_top),
        .instruction_in(Instruction1_out),
        .instruction_out(INSTRUCTION),
        .PCplus4_out(PCPLUS4D_TOP), .PC_out(PCD_top));

    wire [6:0]  INSTR_op   = INSTRUCTION[6:0];
    wire [2:0]  INSTR_f3   = INSTRUCTION[14:12];
    wire [6:0]  INSTR_f7   = INSTRUCTION[31:25];
    wire [11:0] INSTR_imm  = INSTRUCTION[31:20];
    wire [4:0]  INSTR_rs1  = INSTRUCTION[19:15];
    wire [4:0]  INSTR_rs2  = INSTRUCTION[24:20];
    wire [31:0] INSTR_full = INSTRUCTION;
    wire [4:0]  INSTR_rd   = INSTRUCTION[11:7];

    Control control (
        .Opcode(INSTR_op), .funct3(INSTR_f3), .funct7(INSTR_f7),
        .imm(INSTR_imm), .halt(halt_top),
        .RegWriteD(RegWrite_top), .ResultSrcD(ResultSrcD_top),
        .MemWriteD(memWriteD_top), .jumpD(jumpD_top), .jumpR(jumpRD_top),
        .BranchD(BranchD_top), .ALUControlD(ALUControlD_top),
        .ALUSrcD(ALUSrcD_top), .ALUSrcA(ALUSrcAD_top),
        .ImmSrc(ImmSrc_top), .ALUType(ALUtyp_top));

    Reg_file Reg_file_top (
        .clk(clk),
        .rs1_addr(INSTR_rs1), .rs2_addr(INSTR_rs2),
        .rd_addr(RdW_top), .Regwrite(RegWriteW_top),
        .Write_data(ResultW_top),
        .Read_data1(RD1_top), .Read_data2(RD2_top));

    imm imm_top (
        .ImmSrc(ImmSrc_top), .instruction(INSTR_full),
        .ImmExt(ImmExtD_top));

    // =========================================================
    // EXECUTE (unchanged)
    // =========================================================
    EX_stage ex_stage (
        .clk(clk), .reset(reset_sync),
        .flushE(FlushE_top),
        .RD1D_in(RD1_top), .RD2D_in(RD2_top),
        .ImmExtD_in(ImmExtD_top), .PCPlus4D_in(PCPLUS4D_TOP),
        .PC_D_in(PCD_top), .Rs1D_in(INSTR_rs1), .Rs2D_in(INSTR_rs2),
        .RdD_in(INSTR_rd), .ALUControlD_in(ALUControlD_top),
        .ALUSrcD_in(ALUSrcD_top), .ALUSrcA_in(ALUSrcAD_top),
        .RegWriteD_in(RegWrite_top), .ResultSrcD_in(ResultSrcD_top),
        .MemWriteD_in(memWriteD_top), .BranchD_in(BranchD_top),
        .JumpD_in(jumpD_top), .JumpR_in(jumpRD_top),
        .ALUType_in(ALUtyp_top),
        .RD1E_out(RD1E_top), .RD2E_out(RD2E_top),
        .ImmExtD_out(ImmExtE_top), .PCPlus4D_out(PCPlus4E_top),
        .PC_D_out(PCE_top), .Rs1D_out(Rs1E_top), .Rs2D_out(Rs2E_top),
        .RdD_out(RdE_top), .ALUControlD_out(ALUControlE_top),
        .ALUSrcD_out(ALUSrcE_top), .ALUSrcA_out(ALUSrcAE_top),
        .RegWriteD_out(RegWriteE_top), .ResultSrcD_out(ResultSrcE_top),
        .MemWriteD_out(MemWriteE_top), .BranchD_out(BranchE_top),
        .JumpD_out(JumpE_top), .JumpR_out(JumpRE_top),
        .ALUType_out(ALUTypE_top));

    wire [31:0] SrcA_fwd =
        (ForwardAE_top == 2'b10) ? ALUResultM_top :
        (ForwardAE_top == 2'b01) ? ResultW_top    :
                                    RD1E_top;

    assign SrcA_top =
        (ALUSrcAE_top == 2'b10) ? 32'd0   :
        (ALUSrcAE_top == 2'b01) ? PCE_top :
                                   SrcA_fwd;

    assign outB_top =
        (ForwardBE_top == 2'b10) ? ALUResultM_top :
        (ForwardBE_top == 2'b01) ? ResultW_top    :
                                    RD2E_top;

    assign ScrB_top = ALUSrcE_top ? ImmExtE_top : outB_top;

    wire [31:0] base_addr_w = JumpRE_top ? RD1E_top : PCE_top;
    assign PCTarget_top = JumpRE_top
        ? ((base_addr_w + ImmExtE_top) & 32'hFFFFFFFE)
        :  (base_addr_w + ImmExtE_top);

    // PCSCR_top: combinational for PC-select MUX (one local load, OK)
    assign PCSCR_top = (zero_top & BranchE_top) | JumpE_top;

    ALU alu (
        .ScrA(SrcA_top), .ScrB(ScrB_top),
        .ALUControl(ALUControlE_top), .ALUType(ALUTypE_top),
        .ALUResult(ALUResultE_top), .Zero(zero_top));

    // =========================================================
    // MEMORY STAGE (unchanged)
    // =========================================================
    MEM_stage mem_stage (
        .clk(clk), .reset(reset_sync),
        .ALUResult_in(ALUResultE_top), .WriteData_in(outB_top),
        .RdM_in(RdE_top), .PCPlus4M_in(PCPlus4E_top),
        .RegWriteM_in(RegWriteE_top), .ResultSrcM_in(ResultSrcE_top),
        .MemWriteM_in(MemWriteE_top),
        .ALUResult_out(ALUResultM_top), .WriteData_out(WriteDataM_top),
        .RdM_out(RdM_top), .PCPlus4M_out(PCPlus4M_top),
        .RegWriteM_out(RegWriteM_top), .ResultSrcM_out(ResultSrcM_top),
        .MemWriteM_out(MemWriteM_top));

    // =========================================================
    // WRITEBACK (unchanged)
    // =========================================================
    WriteBack_stage writeback_stage (
        .clk(clk), .reset(reset_sync),
        .ALUResultW_in(ALUResultM_top), .ReadDataW_in(Datamem_top),
        .RdW_in(RdM_top), .PCPlus4W_in(PCPlus4M_top),
        .RegWriteW_in(RegWriteM_top), .ResultSrcW_in(ResultSrcM_top),
        .ALUResultW_out(ALUResultW_top), .ReadDataW_out(ReadDataW_top),
        .RdW_out(RdW_top), .PCPlus4W_out(PCPlus4W_top),
        .RegWriteW_out(RegWriteW_top), .ResultSrcW_out(ResultSrcW_top));

    Write_back write_back (
        .ALUResultW_in(ALUResultW_top), .ReadDataW_in(ReadDataW_top),
        .PCPlus4W_in(PCPlus4W_top), .ResultSrcW_in(ResultSrcW_top),
        .ResultW(ResultW_top));

    // =========================================================
    // HAZARD UNIT (unchanged)
    // =========================================================
    Hazard_Unit hazard (
        .Rs1D(INSTR_rs1), .Rs2D(INSTR_rs2),
        .Rs1E(Rs1E_top), .Rs2E(Rs2E_top), .RdE(RdE_top),
        .RegWriteE(RegWriteE_top), .PCSRCE(PCSCR_top),
        .ResultSrcE_in(ResultSrcE_top),
        .RdM(RdM_top), .RdW(RdW_top),
        .RegWriteM(RegWriteM_top), .RegWriteW(RegWriteW_top),
        .StallF(StallF_top), .StallD(StallD_top),
        .FlushD(FlushD_top), .FlushE(FlushE_top),
        .Forward_AE(ForwardAE_top), .Forward_BE(ForwardBE_top));

    // =========================================================
    // PERIPHERALS (unchanged)
    // =========================================================
    wire        spi2_start_w, spi2_busy_w, spi2_done_w, spi2_pending_w;
    wire [7:0]  spi2_tx_data_w, spi2_rx_data_w;
    wire        gpio2_wr_en_w, gpio2_wdata_w;

    DataMem databus_inst (
        .clk             (clk),
        .reset           (reset_sync),
        .aluAddress_in   (ALUResultM_top),
        .DataWriteM_in   (WriteDataM_top[7:0]),
        .memwriteM_in    (MemWriteM_top),
        .DataMem_out     (Datamem_top),
        .uart_tx_start   (periph_tx_start),
        .uart_out_data   (periph_tx_data),
        .uart_tx_busy    (uart_tx_busy_shared),
        .spi2_tx_data    (spi2_tx_data_w),
        .spi2_start      (spi2_start_w),
        .spi2_pending_out(spi2_pending_w),
        .spi2_rx_data    (spi2_rx_data_w),
        .spi2_busy       (spi2_busy_w),
        .spi2_done       (spi2_done_w),
        .gpio2_wr_en     (gpio2_wr_en_w),
        .gpio2_wdata     (gpio2_wdata_w));

    spi_master #(
        .DATA_WIDTH(8), .CPOL(0), .CPHA(0), .CLK_DIV(8)
    ) spi2_inst (
        .clk(clk),
        .reset(reset_sync),
        .start(spi2_start_w),
        .tx_data(spi2_tx_data_w),
        .rx_data(spi2_rx_data_w),
        .busy(spi2_busy_w),
        .done(spi2_done_w),
        .sclk(spi2_sclk),
        .mosi(spi2_mosi),
        .miso(spi2_miso));

    gpio2_io gpio2 (
        .clk(clk), .reset(reset_sync),
        .wr_en2(gpio2_wr_en_w), .wdata2(gpio2_wdata_w),
        .spi_busy(spi2_busy_w), .spi_pending(spi2_pending_w),
        .gpio_out2(spi2_cs_n));

endmodule








