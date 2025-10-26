`timescale 1 ns/ 100 ps
module Incrementor8Bit_tb;

	reg [7:0] in;
	wire [7:0] out;
	
	Incrementor8Bit dut (
		in,
		out
	);


	integer i;
	reg [7:0] expected, actual;
	
	initial begin
		
		for (i = 0; i < 256; i = i + 1) // Perform 500,000 tests
		begin
			in = i; // Generate a 25-bit random value
			
			#0.1; // Let the input update to the new value
			
			expected = in + 1; // This is what is expected
			actual = out; // This would be the actual value
			
			#8.9; // Let the expected and actual update to the new value
			
			if (expected != actual) // Check if the expected is the same as the actual
			begin
			
				$display("incorrect value (%d) at %d: %d + 1", out, i, in);
			
			end
			#1;
		end
		
		$stop;
	
	end

endmodule 