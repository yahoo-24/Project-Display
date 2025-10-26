`timescale 1 ns / 100 ps
module SpecialDFlipFlop8Bit_tb;

	reg clk;
	reg rst;
	reg [7:0] ResetVal;
	reg [7:0] in;
	wire [7:0] Q;
	
	SpecialDFlipFlop8Bit dut (
		clk,
		rst,
		ResetVal,
		in,
		Q
	);
	
	always begin
	
		#5;
		clk <= ~clk;
	
	end
	
	always @ (posedge clk or posedge rst) begin
	
		#0.1; // Give the output time to adjust to the new value
		if (rst & Q != ResetVal) begin
		
			$display("Q did not reset to ResetVal"); // Checking that it resets to ResetVal when rst is 1
		
		end
		else if (clk & in != Q & !rst) begin
		
			$display("Q is not following input"); // Checking that when the clk is HIGH Q follows in
		
		end
	
	end
	
	initial begin
	
		// Initialise the values and set the reset and ResetVal
		clk <= 1'b0;
		ResetVal <= 8'b10010011;
		rst <= 1'b1;
		in <= 8'b0;
		#5;
		
		rst <= 1'b0; // Remove the reset
		ResetVal <= 8'b11100010; // Change the Val
		#10;
		
		// Test with different inputs
		in <= 8'b00110011;
		#15;
		
		in <= 8'b00111100;
		#10;
		
		in <= 8'b01010000;
		#10;
		
		rst <= 1'b1; // Apply the reset
		#15;
		
		$stop;
	
	end

endmodule 