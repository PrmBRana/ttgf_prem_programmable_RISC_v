<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works
 
 
This project implements a compact 32-bit RISC-V processor with a five-stage pipeline architecture consisting of Instruction Fetch (IF), Decode (ID), Execute (EX), Memory (MEM), and Write-Back (WB) stages. The pipelined design allows multiple instructions to be processed concurrently, improving throughput while maintaining a small hardware footprint.

The system includes two UART, one SPI and two GPIO interfaces:

1. UART1 is used as a bootloader interface for program loading
2. UART2 is used as a general-purpose communication interface during execution
3. SPI is used as a general-purpose communication interface during execution
4. GPIO1 is used as for LED blink or others
5. GPIO2 is used for chip select for SPI

During reset, the processor enters bootloader mode. In this mode, instructions are received serially through the UART1 RX pin and stored into instruction memory. Once the program loading sequence is complete, the processor automatically switches to execution mode and begins fetching and executing instructions through the pipeline.

The system operates with a 25 MHz system clock. Both UART interfaces use a 115200 baud rate with x16 oversampling. The SPI interface operates in Mode 0 (CPOL = 0, CPHA = 0) with a clock frequency of approximately 4.17 MHz (CLK_DIV = 3).

In addition to UART, the design includes an SPI master interface for communication with external peripherals such as sensors or external memory devices. The SPI interface is controlled through dedicated hardware signals (MOSI, MISO, SCLK, and CS), enabling full-duplex data transfer.

The design is optimized for low-area ASIC implementations and is suitable for embedded systems, educational processors, and sensor interfacing applications.

## How to test

The design can be tested using both simulation and hardware.

In simulation, the project is typically run using Icarus Verilog and cocotb. A standard testbench compiles the design, applies clock and reset signals, and executes functional test sequences. Waveform outputs can be analyzed using tools such as GTKWave.

For UART testing, the bootloader interface (UART1) is used to send a sequence of 32-bit instructions serialized as bytes. The processor acknowledges correct reception and stores the instructions in instruction memory. After loading is complete, execution begins automatically, and correctness can be verified by monitoring pipeline activity, register updates, and memory access patterns.

For SPI testing, an external SPI master or testbench drives the SCLK, MOSI, and CS signals while observing the MISO output from the processor. Correct timing (Mode 0) and data integrity are verified by comparing transmitted and received byte streams.

Functional validation includes verifying:

1. Correct instruction execution
2. Proper pipeline operation
3. Memory read/write correctness
4. UART bootloading sequence
5. SPI data transfer timing and integrity

## External hardware

This design requires only standard external signals for operation:

1. Clock input (25 MHz)
2. Active-low reset (rst_n)
3. Enable signal (ena) from system controller
4. UART1 RX/TX for bootloading
5. UART2 RX/TX for communication
6. SPI interface signals (MOSI, MISO, SCLK, CS)
7. GPIO1 and GPIO2(CS)