`timescale 1 ns / 100 ps
module Subtractor4Bit_tb;

	reg [3:0] subtrahend, minuend;
	wire [3:0] difference;
	wire carry;
	
	Subtractor4Bit dut (
		minuend,
		subtrahend,
		difference,
		carry
	);
	
	integer i;
	reg [3:0] expected;
	
	initial begin
	
		for (i = 0; i < 256; i = i + 1)
		begin
			{minuend, subtrahend} = i;
			#0.1;
			expected = minuend - subtrahend; // For viewing only
			#10;
			if (minuend - subtrahend != difference)
			begin
			
				$display("incorrect value at %d + %d", minuend, subtrahend);
			
			end
		end
		
		$stop;
	
	end

endmodule
