`timescale 1 ns / 100 ps
module DoubleDabble8Bit_tb;

	reg [7:0] Binary;
	wire [7:0] BCD;
	
	DoubleDabble8Bit dut (
		Binary,
		BCD
	);
	
	integer i, expected_tens, expected_ones, actual_tens, actual_ones;
	
	initial begin
		
		// All the possible input go from 0 to 99
		for (i = 0; i < 100; i = i + 1) begin
			
			Binary = i;
			#1; // Let the Binary value update
			
			// These are the actual and expected values
			expected_tens = i / 10;
			expected_ones = i % 10;
			actual_tens = BCD[7:4]; // This is used for viewing
			actual_ones = BCD[3:0]; // This is used for viewing
			#5; // The delay lets the 4 values above update
			
			if (BCD[7:4] != expected_tens | BCD[3:0] != expected_ones) begin
				$display("Failure at i = %d", i); // If actual != expected then print this message
			end
		
		end
		
		$stop;
		
	end

endmodule 