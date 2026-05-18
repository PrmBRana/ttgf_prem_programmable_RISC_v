<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works
 
![Block Diagram](img/block.png)

This project implements a compact 32-bit RISC-V processor with a five-stage pipeline architecture consisting of Instruction Fetch (IF), Decode (ID), Execute (EX), Memory (MEM), and Write-Back (WB) stages. The pipelined design allows multiple instructions to be processed concurrently, improving throughput while maintaining a small hardware footprint.

### The system includes two UART interfaces, one SPI interface, and two GPIOs:

1. UART1 is used as a bootloader interface for program loading  
2. UART2 is used as a general-purpose communication interface during execution  
3. SPI is used as a peripheral communication interface during execution  
4. GPIO1 is used for LED control or general-purpose output  
5. GPIO2 is used as chip select (CS) control for SPI  

During reset, the processor enters bootloader mode. In this mode, instructions are received serially through the UART1 RX pin and stored into instruction memory. Once the program loading sequence is complete, the processor automatically switches to execution mode and begins fetching and executing instructions through the pipeline.

The system operates with a 25 MHz system clock. Both UART interfaces use a 115200 baud rate with x16 oversampling. The SPI interface operates in Mode 0 (CPOL = 0, CPHA = 0) with a clock frequency of approximately 4.17 MHz (CLK_DIV = 3).

In addition to UART, the design includes an SPI master interface for communication with external peripherals such as sensors or external memory devices. The SPI interface is controlled through dedicated hardware signals (MOSI, MISO, SCLK, and CS), enabling full-duplex data transfer.

Peripherals are controlled using a simple polling mechanism instead of interrupts. The CPU continuously reads status registers before performing read/write operations.

### UART Polling Example
1. UART_TX (0x1000_0000) → write transmit byte
2. UART_RX (0x1000_0004) → read received byte
3. UART_TX_STATUS (0x1000_0008) → indicates transmit ready
4. UART_RX_STATUS (0x1000_000C) → indicates data available

The CPU polls the status register before accessing data:

1. Wait until TX ready = 1 before writing new data
2. Wait until RX valid = 1 before reading received data
3. SPI Polling Example
4. SPI_TX (0x4000_0000) → write data to transmit
5. SPI_TX_STATUS (0x4000_0004) → transmission busy/ready flag
6. SPI_RX (0x4000_0008) → read received data
7. SPI_RX_STATUS (0x4000_000C) → data valid flag

The SPI master is also controlled using polling:

CPU waits until SPI is idle before writing new data
CPU reads RX only when valid flag is asserted
GPIO Control
1. GPIO1 (0x3000_0000) → direct output control (e.g., LED)
2. GPIO2 (0x3000_0004) → SPI chip select control

The system uses a memory-mapped I/O architecture where peripherals are accessed through specific 32-bit addresses. The ALU-generated address (aluAddress_in) is decoded to select UART, SPI, and GPIO registers.

The design is optimized for low-area ASIC implementations and is suitable for embedded systems, educational processors, and sensor interfacing applications.

## How to test

The design can be tested using both simulation and hardware.

In simulation, the project is typically run using Icarus Verilog and cocotb. A standard testbench compiles the design, applies clock and reset signals, and executes functional test sequences. Waveform outputs can be analyzed using tools such as GTKWave.

For UART testing, the bootloader interface (UART1) is used to send a sequence of 32-bit instructions serialized as bytes. The processor acknowledges correct reception and stores the instructions in instruction memory. After loading is complete, execution begins automatically, and correctness can be verified by monitoring pipeline activity, register updates, and memory access patterns.

For SPI testing, an external SPI master or testbench drives the SCLK, MOSI, and CS signals while observing the MISO output from the processor. Correct timing (Mode 0) and data integrity are verified by comparing transmitted and received byte streams.

### Functional validation includes verifying:

1. Correct instruction execution
2. Proper pipeline operation
3. Memory read/write correctness
4. UART bootloading sequence
5. SPI data transfer timing and integrity

### Memory setup
The following assembly demonstrates SPI communication using memory-mapped polling, where the CPU continuously checks status registers before reading or writing data.

lui   x10, 0x40000      # SPI base tx
lui   x18, 0x30000      # GPIO base
lui   x19, 0x10000      # UART base

addi  x11, x10, 8       # SPI RX
addi  x12, x10, 4       # SPI TX status
addi  x13, x10, 12      # SPI RX status

addi  x21, x19, 8       # UART TX status

addi  x14, x18, 4       # CS 2 pin

addi  x3, x0, 1         # CS HIGH
addi  x16, x0, 0        # CS LOW
addi  x5, x0, 200        # byte count
addi  x22, x0, 3        # UART mask

sw    x16, 0(x14)       # CS LOW

### send first byte
addi  x7, x0, 0xAA (dummy data)

loop:
    beq   x5, x0, release_cs
    sw    x7, 0(x10)          # trigger SPI

wait_rx:
    lw    x8, 0(x13)
    beq   x8, x0, wait_rx

    lw    x9, 0(x11)          # read received byte

wait_tx:
    lw    x6, 0(x21)
    and   x6, x6, x22         # mask UART ready bits
    bne   x6, x0, wait_tx

    sw    x9, 0(x19)          # send to UART

    addi  x5, x5, -1          # decrement counter
    jal   x0, loop

release_cs:
    sw    x3, 0(x14)          # CS HIGH
    ecall

### Decoded Machine Code

This is the corresponding compiled RISC-V instruction sequence:

           0x40000537,
           0x30000937,
           0x100009b7,
           0x00850593,
           0x00450613,
           0x00c50693,
           0x00898a93,
           0x00490713,
           0x00100193,
           0x00000813,
           0x03200293,
           0x00300b13,
           0x01072023,
           0x0aa00393,
           0x02028663,
           0x00752023,
           0x0006a403,
           0xfe040ee3,
           0x0005a483,
           0x000aa303,
           0x01637333,
           0xfe031ce3,
           0x0099a023,
           0xfff28293,
           0xfd9ff06f,
           0x00372023,
           0x00000073,
           0xBAADF00D // sentinel valuse (last always)

### This sentinel  value detected by the uart bootloader then stop and start execute the processor.


## External hardware

This design requires only standard external signals for operation:

1. Clock input (25 MHz)
2. Active-low reset (rst_n)
3. Enable signal (ena) from system controller
4. UART1 RX/TX for bootloading
5. UART2 RX/TX for communication
6. SPI interface signals (MOSI, MISO, SCLK, CS)
7. GPIO1 (LED or general purpose) and 
8. GPIO2(Chip select control)