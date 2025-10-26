module Subtractor4Bit (
	input [3:0] minuend,
	input [3:0] subtrahend,
	output [3:0] difference,
	output carry
);

	wire [3:0] subtrahend_temp;
	
	// x - y => x + not(y) + 1 since not(y) + 1 is the 2's complement of y
	
	not(subtrahend_temp[0], subtrahend[0]);
	not(subtrahend_temp[1], subtrahend[1]);
	not(subtrahend_temp[2], subtrahend[2]);
	not(subtrahend_temp[3], subtrahend[3]);

	FourBitCarryLookAheadAdder Adder (
		minuend,
		subtrahend_temp,
		1'b1,
		difference,
		carry
	);

endmodule
