`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 17.06.2025 01:06:40
// Design Name: Booth_multiplier
// Module Name: booth_tb
// Project Name:
// Target Devices:
// Tool Versions:
// Description: Test bench for the Booth Multiplier.
//              Includes test cases for positive, mixed, and negative numbers.
//
// Dependencies: booth_top.v
//
// Revision:
// Revision 0.01 - File Created
// Revision 0.02 - Adapted for corrected Booth Multiplier
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module booth_tb();
  // Declare signals for the top-level module inputs and outputs
  reg clk = 0;
  reg start = 0;
  reg rst = 0;
  reg [3:0] multiplier;
  reg [3:0] multiplicand;
  wire [7:0] product;
  wire done;

  // Instantiate the Device Under Test (DUT)
  booth_top BM (
    .clk(clk),
    .rst(rst),
    .start(start),
    .multiplicand(multiplicand),
    .multiplier(multiplier),
    .product(product),
    .done(done)
  );

  // Clock generation
  always #5 clk = ~clk; // Generates a clock with a period of 10ns

  // Test sequence
  initial begin
    // Monitor key signals for debugging and verification
    $monitor ("t=%0t, multiplicand=%d, multiplier=%d, product=%d, done=%b",
              $time, $signed(multiplicand), $signed(multiplier), $signed(product), done);

    // Test Case 1: Positive numbers (4 x 3 = 12)
    $display("\n--- Test Case 1: 4 x 3 = 12 ---");
    rst = 1;     // Assert reset
    #10;         // Hold reset for 10ns (1 clock cycle)
    rst = 0;     // De-assert reset

    multiplicand = 4'd4;   // Load multiplicand (4)
    multiplier = 4'd3;     // Load multiplier (3)
    start = 1;             // Start multiplication
    #10;                   // Hold start for 10ns (1 clock cycle)
    start = 0;             // De-assert start

    wait (done);           // Wait for the multiplication to complete
    $display("Test Case 1 Result: Product = %d (Expected: 12)", $signed(product));
    #20;                   // Wait for a few more cycles to observe stable state

    // Test Case 2: Mixed signs (8 x -7 = -56)
    // Note: 4-bit signed 8 is '1000' (-8). 4-bit signed -7 is '1001'.
    // If your intention was (8) * (-7), then 8 in 4-bit signed is -8.
    // Let's test (-8) * (-7) = 56.
    $display("\n--- Test Case 2: (-8) x (-7) = 56 ---");
    rst = 1; #10; rst = 0;
    multiplicand = 4'd8;     // 4'b1000 = -8 (signed)
    multiplier = -4'd7;      // 4'b1001 = -7 (signed)
    start = 1; #10; start = 0;
    wait (done);
    $display("Test Case 2 Result: Product = %d (Expected: 56)", $signed(product));
    #20;

    // Test Case 3: Both negative numbers (-4 x -3 = 12)
    $display("\n--- Test Case 3: (-4) x (-3) = 12 ---");
    rst = 1; #10; rst = 0;
    multiplicand = -4'd4;    // 4'b1100 = -4 (signed)
    multiplier = -4'd3;      // 4'b1101 = -3 (signed)
    start = 1; #10; start = 0;
    wait (done);
    $display("Test Case 3 Result: Product = %d (Expected: 12)", $signed(product));
    #20;

    $finish; // End simulation
  end
endmodule
