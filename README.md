# Booth-multiplier
A simple signed booth multiplier that produces accurate results for signed multiplications

Booth Multiplier (Vivado Project)

Overview

I have designed a 4-bit signed Booth multiplier in Verilog. The design supports signed multiplication using Booth's algorithm, efficiently handling positive and negative operands. The repository includes the full RTL, a testbench, and simulation instructions.

Features

  - Booth's Algorithm for signed multiplication

  - Modular design with clear datapath and control separation

  - Parameterized for 4-bit signed inputs (producing 8-bit results)

  - Testbench with multiple signed and unsigned test cases

modules implemented:

  - booth_top.v: Top-level module connecting datapath and control.

    - booth_datapath.v: Implements Booth's algorithm registers and ALU.

    - control_path.v: FSM for Booth control logic.

    - shiftreg_A.v, shiftreg_Q.v: Shift registers for accumulator and multiplier.

    - alu.v: Arithmetic Logic Unit for addition/subtraction.

    - registerM.v: Multiplicand register with sign extension.

    - booth_tb.v: Testbench with sample test cases.

How It Works

Inputs:

  - multiplicand (4-bit signed)

  - multiplier (4-bit signed)

  - start, clk, rst (control signals)

Operation:

  - On start, the control FSM loads inputs and steps through Booth's algorithm.

  - The datapath performs ALU operations and shifts according to Booth's rules.

  - After 4 Booth steps, the product output is valid and done is asserted.

Outputs:

  - product (8-bit signed result)

  - done (operation complete flag)

Testbench

I have provided a testbench (booth_tb.v) that applies the following test cases:
  - 4 X 3 = 12
  - -8 X -7 = 56
  - -4 X -3 = 12

Excluding Unnecessary Files

I have included a.gitignore file to prevent large Vivado-generated directories (such as .Xil/, .runs/, .cache/, etc.)
