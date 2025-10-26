`timescale 1 ns / 100 ps
module SpecialDFlipFlopSingle_tb;

	reg clk;
	reg rst;
	reg ResetVal;
	reg in;
	wire Q;
	
	SpecialDFlipFlopSingle dut (
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
	
		// Initialise values
		clk <= 1'b0;
		ResetVal <= 1'b0; // Set the reset value
		rst <= 1'b1; // Reset is on
		in <= 1'b0;
		#5;
		
		rst <= 1'b0; // Remove reset
		ResetVal <= 1'b1; // Change the reset value
		#10;
		
		// Test with different input values
		in <= 1'b0;
		#15;
		
		in <= 1'b1;
		#10;
		
		in <= 1'b0;
		#10;
		
		rst <= 1'b1; // End it with a reset.
		#15;
		
		$stop;
	
	end

endmodule 