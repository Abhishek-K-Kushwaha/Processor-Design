# Verilog-Based 8-bit Microprocessor Design

This project implements an 8-bit microprocessor in Verilog featuring a custom ALU, instruction set, program counter, and register file. It includes a clock divider for timing management and supports basic arithmetic, logic operations, and control instructions. It was implemented on a Basys3 board.

## Processor Overview

The processor supports the following instructions:

| Instruction | Opcode | Operation |
|-------------|--------|-----------|
| NOP         | 0000 0000 | No operation |
| ADD Ri      | 0001 xxxx | Add ACC with Register contents |
| SUB Ri      | 0010 xxxx | Subtract ACC with Register contents |
| MUL Ri      | 0011 xxxx | Multiply ACC with Register contents |
| DIV Ri      | 0100 xxxx | Divide ACC with Register contents |
| LSL ACC     | 0000 0001 | Left shift logical ACC |
| LSR ACC     | 0000 0010 | Right shift logical ACC |
| CIR ACC     | 0000 0011 | Circular right shift ACC |
| CIL ACC     | 0000 0100 | Circular left shift ACC |
| ASR ACC     | 0000 0101 | Arithmetic shift right ACC |
| AND Ri      | 0101 xxxx | Bitwise AND with Register contents |
| XRA Ri      | 0110 xxxx | Bitwise XOR with Register contents |
| CMP Ri      | 0111 xxxx | Compare ACC with Register contents |
| INC ACC     | 0000 0110 | Increment ACC |
| DEC ACC     | 0000 0111 | Decrement ACC |
| Br <4-bit address> | 1000 xxxx | Conditional branch |
| MOV ACC, Ri | 1001 xxxx | Move contents of Ri to ACC |
| MOV Ri, ACC | 1010 xxxx | Move contents of ACC to Ri |
| Ret <4-bit address> | 1011 xxxx | Return from subroutine |
| HLT         | 1111 1111 | Halt execution |

## Implementation Details

The Verilog modules include:
- `processor.v`: Top-level module integrating all components.
- `clock_divide.v`: Clock divider for generating slower clock signals.
- `program_counter.v`: Module for program counter management.
- `alu.v`: Arithmetic Logic Unit supporting various operations.
- `register_file.v`: Register file with sixteen 8-bit registers.
- `instruction_memory.v`: Memory module for storing instructions.
- `divider.v`: Divider module for division operation.

## Demonstration

This processor was implemented on the Basys3 FPGA board. A video of the demonstration can be found [here](https://drive.google.com/file/d/1oN6Vtosld_0KuCGsDvVTTq9ti4P5VHUL/view).

