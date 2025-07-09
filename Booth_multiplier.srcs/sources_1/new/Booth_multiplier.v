`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 16.06.2025 22:42:35
// Design Name: Booth_multiplier
// Module Name: Booth_multiplier
// Project Name:
// Target Devices:
// Tool Versions:
// Description: Corrected Booth Multiplier implementation.
//              - Corrected shiftreg_Q for incoming bit from A.
//              - Simplified dff for q0 capture.
//              - Fixed multiplicand sign extension for registerM.
//              - Corrected product output concatenation.
//              - Redesigned FSM for a 2-cycle Booth step (ALU op then Shift).
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Revision 0.02 - Corrected issues based on debugging analysis.
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

// Shift register to store Accumulator (generalized to 5-bit for 4-bit signed multiplication)
// This module loads data_in when 'load' is high (for initial A=0 or ALU result).
// It performs an arithmetic right shift when 'shift' is high and 'load' is low.
module shiftreg_A(
  input [4:0] data_in,
  input clk, rst, shift, load,
  output reg [4:0] data_out
);
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      data_out <= 0; // Reset A to 0
    end else if (load) begin // Load has priority (used for initial load and ALU result load)
      data_out <= data_in;
    end else if (shift) begin // Shift only if not loading
      data_out <= {data_out[4], data_out[4:1]}; // Arithmetic right shift
    end
  end
endmodule

// Shift register for multiplier Q
// This module loads data_in when 'load' is high.
// It performs a right shift, taking 'shift_in_from_A' as the MSB.
module shiftreg_Q(
  input [3:0] data_in,
  input clk,rst,load,shift,
  input shift_in_from_A, // Input for the bit shifted out from Accumulator (A_out[0])
  output reg [3:0] data_out
);
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      data_out <= 0; // Reset Q to 0
    end else if(load) begin // Load has priority (for initial multiplier load)
      data_out <= data_in;
    end else if(shift) begin // Shift only if not loading
      data_out <= {shift_in_from_A, data_out[3:1]}; // Shift right, MSB comes from A's LSB
    end
  end
endmodule

// D Flip-Flop for storage of q0 bit (Q[-1] in Booth's algorithm)
// This DFF simply captures its input 'd' on the positive clock edge.
// It effectively holds the Q_out[0] value from the *previous* clock cycle.
module dff(
  input clk,rst,
  input d,
  output reg q
);
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      q <= 0; // Reset q0 to 0
    end else begin
      q <= d; // Capture the input 'd'
    end
  end
endmodule

// Register for storage of multiplicand M
// This module simply loads data_in when 'load' is high.
module registerM(
  input [4:0] data_in, // Multiplicand (with sign extension)
  input clk,rst,load,
  output reg [4:0] data_out
);
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      data_out <= 0; // Reset M to 0
    end else if (load) begin // Load has priority (for initial multiplicand load)
      data_out <= data_in;
    end
  end
endmodule

// ALU unit (Arithmetic Logic Unit)
// Performs addition (A+B) or subtraction (A-B) in two's complement.
module alu(
  input [4:0] A,B,
  input sel, // 0: subtraction (A-B), 1: addition (A+B)
  output [4:0] result
);
  assign result = sel ? (A+B) : (A+(~B+1)); // A-B is A + (~B + 1) for two's complement
endmodule

// Datapath for the Booth Multiplier
// Instantiates all registers and ALU, and handles data flow.
module booth_datapath(
  input clk,rst,shift,load,alu_en,sel,
  input [3:0] multiplier_in, // Multiplier input
  input [3:0] multiplicand_in, // Multiplicand input
  output [7:0] product,       // Final 8-bit product
  output q0_out,              // Q[-1] (output from dff)
  output q1_out               // Q[0] (current LSB of Q_out)
);
  wire [4:0] A_out;          // Output of Accumulator A register
  wire [4:0] M_out;          // Output of Multiplicand M register
  wire [4:0] alu_result;     // Combinational output of ALU
  wire [4:0] A_in;           // Input to Accumulator A register (either alu_result or A_out)
  wire [3:0] Q_out;          // Output of Multiplier Q register

  // q1_out represents the current Q[0] bit
  assign q1_out = Q_out[0];

  // A_Lsb is the bit shifted out of A_out (LSB of A_out) which goes into Q_out's MSB during shift
  wire A_Lsb;
  assign A_Lsb = A_out[0];

  // A_in logic: If alu_en is high, load alu_result into A; otherwise, load A_out (for no change or shift).
  // This A_in feeds into the 'data_in' of shiftreg_A.
  assign A_in = alu_en ? alu_result : A_out;

  // Instantiate Accumulator (A) register
  // 'load' or 'alu_en' activates the load function in shiftreg_A.
  // 'shift' activates the shift function in shiftreg_A.
  shiftreg_A A_reg(
    .clk(clk),
    .rst(rst),
    .shift(shift),
    .load(load || alu_en), // Initial load OR load ALU result
    .data_in(A_in),
    .data_out(A_out)
  );

  // Instantiate Multiplier (Q) register
  // 'load' activates initial load. 'shift' activates shifting.
  // 'shift_in_from_A' connects to A_Lsb.
  shiftreg_Q Q_reg(
    .clk(clk),
    .rst(rst),
    .load(load),
    .shift(shift),
    .data_in(multiplier_in),
    .shift_in_from_A(A_Lsb), // Pass the LSB of A into Q's MSB
    .data_out(Q_out)
  );

  // Instantiate q0 DFF (stores previous Q[0] for Booth's algorithm)
  // q0_out will hold the value of Q_out[0] from the *previous* clock cycle.
  dff q0_dff(
    .clk(clk),
    .rst(rst),
    .d(Q_out[0]), // Input is the current Q_out[0]
    .q(q0_out)    // Output is the previous Q_out[0]
  );

  // Instantiate ALU
  // Operates on current A_out and M_out. Result is combinational.
  alu ALU_unit(
    .A(A_out),
    .B(M_out),
    .sel(sel),
    .result(alu_result)
  );

  // Instantiate Multiplicand (M) register
  // 'load' activates initial load.
  // Multiplicand_in (4-bit) is sign-extended to 5 bits for M_out.
  registerM M_reg(
    .clk(clk),
    .rst(rst),
    .load(load),
    .data_in({multiplicand_in[3], multiplicand_in}), // Sign-extend 4-bit multiplicand to 5 bits
    .data_out(M_out)
  );

  // Form the 8-bit product from A_out (upper 4 bits, discarding sign extension) and Q_out (lower 4 bits).
  // A_out is 5-bit where A_out[4] is the sign bit. A_out[3:0] are the magnitude bits that form product.
  assign product = {A_out[3:0], Q_out};
endmodule

// FSM for the Control Path
// Implements a 2-cycle per Booth step: ALU operation then Shift.
module control_path(
  input q0_prev, // Represents Q[-1] (from q0_out of datapath)
  input q1_curr, // Represents Q[0] (from q1_out of datapath)
  input clk,rst,start,
  output reg load,shift,alu_en,sel,done
);
  reg [2:0] COUNT;    // Counter for the 4 Booth steps (for 4-bit multiplication)
  reg [1:0] state;    // FSM state register
  parameter IDLE = 2'b00, ALU_OP = 2'b01, SHIFT_STEP = 2'b10, DONE = 2'b11;

  always @(posedge clk or posedge rst) begin
    if (rst) begin
      // Reset all control signals and state
      state <= IDLE;
      COUNT <= 0;
      done <= 0;
      load <= 0;
      shift <= 0;
      alu_en <= 0;
      sel <= 0;
    end else begin
      case (state)
        IDLE : begin
          done <= 0;
          if (start) begin
            load <= 1;        // Assert load for initial values into A, Q, M
            COUNT <= 4;       // Initialize counter for 4 Booth steps
            state <= ALU_OP;  // Transition to the first ALU operation phase
          end else begin
            load <= 0;        // De-assert load if start is not active
          end
        end

        ALU_OP : begin // Cycle 1: Determine and perform ALU operation if needed
          load <= 0;      // De-assert initial load
          shift <= 0;     // No shift in this cycle
          alu_en <= 0;    // Reset alu_en/sel
          sel <= 0;

          // Booth's encoding logic based on current Q[0] (q1_curr) and previous Q[0] (q0_prev)
          case ({q1_curr, q0_prev})
            2'b01 : begin alu_en <= 1; sel <= 1; end // Q[0]Q[-1] = 01 -> A = A + M
            2'b10 : begin alu_en <= 1; sel <= 0; end // Q[0]Q[-1] = 10 -> A = A - M
            default : begin alu_en <= 0; sel <= 0; end // Q[0]Q[-1] = 00 or 11 -> No operation (A = A)
          endcase
          state <= SHIFT_STEP; // Always transition to the shift step after ALU_OP
        end

        SHIFT_STEP : begin // Cycle 2: Perform the combined shift
          load <= 0;      // De-assert load
          alu_en <= 0;    // De-assert ALU enable (operation already propagated)
          sel <= 0;       // De-assert ALU select
          shift <= 1;     // Assert shift for A and Q registers

          COUNT <= COUNT - 1; // Decrement step counter
          if (COUNT == 0) begin // All 4 Booth steps completed
            state <= DONE;      // Transition to DONE state
          end else begin
            state <= ALU_OP;    // Go back to ALU operation for the next Booth step
          end
        end

        DONE : begin
          done <= 1;          // Assert done signal
          // De-assert all control signals for next operation
          load <= 0;
          shift <= 0;
          alu_en <= 0;
          sel <= 0;
          state <= IDLE;      // Go back to IDLE state, waiting for next start
        end
      endcase
    end
  end
endmodule

// Top-level Booth Multiplier module
// Connects the datapath and control path.
module booth_top(
  input clk,rst,start,
  input [3:0] multiplicand, multiplier,
  output [7:0] product,
  output done
);
  // Wires to connect control path outputs to datapath inputs
  wire load, alu_en, shift, sel;
  // Wires to connect datapath outputs (Booth's bits) to control path inputs
  wire q0_from_dff; // Output from the dff (Q[-1])
  wire q1_from_Q;   // LSB of Q_out (Q[0])

  // Instantiate Datapath
  booth_datapath dp(
    .clk(clk),
    .rst(rst),
    .alu_en(alu_en),
    .load(load),
    .multiplicand_in(multiplicand),
    .multiplier_in(multiplier),
    .shift(shift),
    .sel(sel),
    .q0_out(q0_from_dff),
    .q1_out(q1_from_Q),
    .product(product)
  );

  // Instantiate Control Path
  control_path cp(
    .clk(clk),
    .rst(rst),
    .start(start),
    .q0_prev(q0_from_dff), // Pass q0 from DFF to control
    .q1_curr(q1_from_Q),   // Pass current Q[0] to control
    .load(load),
    .shift(shift),
    .alu_en(alu_en),
    .sel(sel),
    .done(done)
  );
endmodule
