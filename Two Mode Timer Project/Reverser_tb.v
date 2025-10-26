`timescale 1 ns / 100 ps 		//modify the parameters accordingly

module Reverser_tb;

	reg [7:0] RevIn;
	reg ModeSel;
	wire [7:0] RevOut;
	reg [3:0] temp1, temp10;
	
	Reverser dut (
		RevIn,
		ModeSel,
		RevOut
	);
	
	integer i, expected_tens, expected_ones, actual_tens, actual_ones;
	
	initial begin
	
		ModeSel <= 1'b1; // Mode B
	
		for (i = 0; i < 60; i = i + 1) begin // Mode B inputs: 0 to 59
		
			temp10 = i / 10;
			temp1 = i % 10;
			RevIn = {temp10, temp1};
			
			expected_tens = 5 - i / 10; // The expected output is 59 - input
			expected_ones = 9 - i % 10;
			
			#0.1; // Delay to update values above
			
			actual_tens = RevOut[7:4]; // The actual output from the tens (4 MSB)
			actual_ones = RevOut[3:0]; // The actual output from the ones (4 LSB)
			
			#10; // Delay to update values above
			
			if (expected_tens != RevOut[7:4] | expected_ones != RevOut[3:0]) begin
				$display("Failure at %d in Mode B", i); // Check that output == expected
			end
		
		end
		
		ModeSel <= 1'b0; // Mode A
		// Reeat procedure above
		
		for (i = 0; i < 100; i = i + 1) begin // Mode A inputs: 0 to 99
		
			temp10 = i / 10;
			temp1 = i % 10;
			RevIn = {temp10, temp1};
			
			expected_tens = i / 10; // For Mode A: Out = In
			expected_ones = i % 10;
			
			#0.1;
			
			actual_tens = RevOut[7:4];
			actual_ones = RevOut[3:0];
			
			#10;
			
			if (expected_tens != RevOut[7:4] | expected_ones != RevOut[3:0]) begin
				$display("Failure at %d in Mode A", i);
			end
		
		end
		
		$stop;
		
	end

endmodule 