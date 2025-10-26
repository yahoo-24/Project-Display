module Incrementor8Bit (
	input [7:0] in,
	output [7:0] out
);

	wire temp1, temp2, temp3, temp4, temp5, temp6;
	
	// When adding by 1, all the bits from RHS to LHS toggle up until the first 0.
	// Therefore we can tell if we need to toggle by 'anding' all the previous bits.
	// The value would be 1 if all the bits are 1 hence we need to toggle otherwise it would be 0 and no toggle.
	// Finally, the toggle is achieved using the xor gate.
	
	assign temp1 = (in[1] & in[0]);
	assign temp2 = (in[2] & temp1);
	assign temp3 = (in[3] & temp2);
	assign temp4 = (in[4] & temp3);
	assign temp5 = (in[5] & temp4);
	assign temp6 = (in[6] & temp5);

	assign out[0] = in[0] ^ 1'b1;
	assign out[1] = in[1] ^ in[0];
	assign out[2] = in[2] ^ temp1;
	assign out[3] = in[3] ^ temp2;
	assign out[4] = in[4] ^ temp3;
	assign out[5] = in[5] ^ temp4;
	assign out[6] = in[6] ^ temp5;
	assign out[7] = in[7] ^ temp6;
	
endmodule 