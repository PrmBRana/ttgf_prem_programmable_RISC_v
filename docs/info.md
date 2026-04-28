<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

This project implements a compact 32-bit RISC-V processor featuring a five-stage pipeline architecture, consisting of instruction fetch, decode, execute, memory, and write-back stages. The pipeline allows multiple instructions to be processed simultaneously, improving overall performance and efficiency within limited hardware resources. The design integrates a single UART interface that serves as both a program loading mechanism and a communication channel. Upon reset, the processor enters a programming mode where instructions are received serially through the UART RX line and stored into instruction memory. After the program is fully loaded, the processor transitions to execution mode and begins fetching and executing instructions sequentially through the pipeline. In addition to UART, the design includes an SPI master interface, enabling communication with external devices through standard signals such as MOSI, MISO, SCLK, and chip select. The SPI interface is controlled via memory-mapped registers, allowing the processor to interact with peripherals such as sensors or external memory devices. The overall system is optimized for environments with limited I/O and silicon area, making it suitable for compact ASIC implementations.

## How to test

The design can be tested through simulation or hardware interaction. In simulation, the project is typically run using tools like Icarus Verilog and cocotb by executing a standard build command, which compiles the design, runs the testbench, and generates waveform files for analysis. During testing, the UART interface is used to send program instructions serially into the processor while it is in programming mode. Each instruction is transmitted in a standard UART frame format and stored sequentially in memory. After loading is complete, the processor automatically begins execution, and its behavior can be verified by observing internal signals such as the program counter, register file updates, and pipeline stage outputs in a waveform viewer. The SPI interface can be tested by providing input data on the MISO line and monitoring the generated MOSI, clock, and chip select signals to confirm correct communication timing and data transfer. Functional correctness is verified by ensuring proper instruction execution, memory access, and peripheral interaction.

## External hardware

Control signals like clock, reset and enable signals, instruction memory