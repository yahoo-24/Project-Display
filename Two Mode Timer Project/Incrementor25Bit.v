module Incrementor25Bit (
	input [24:0] in,
	output [24:0] out
);

	wire temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8, temp9;
	wire temp10, temp11, temp12, temp13, temp14, temp15, temp16, temp17, temp18, temp19;
	wire temp20, temp21, temp22, temp23;
	
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
	assign temp7 = (in[7] & temp6);
	assign temp8 = (in[8] & temp7);
	assign temp9 = (in[9] & temp8);
	assign temp10 = (in[10] & temp9);
	assign temp11 = (in[11] & temp10);
	assign temp12 = (in[12] & temp11);
	assign temp13 = (in[13] & temp12);
	assign temp14 = (in[14] & temp13);
	assign temp15 = (in[15] & temp14);
	assign temp16 = (in[16] & temp15);
	assign temp17 = (in[17] & temp16);
	assign temp18 = (in[18] & temp17);
	assign temp19 = (in[19] & temp18);
	assign temp20 = (in[20] & temp19);
	assign temp21 = (in[21] & temp20);
	assign temp22 = (in[22] & temp21);
	assign temp23 = (in[23] & temp22);

	assign out[0] = in[0] ^ 1'b1;
	assign out[1] = in[1] ^ in[0];
	assign out[2] = in[2] ^ temp1;
	assign out[3] = in[3] ^ temp2;
	assign out[4] = in[4] ^ temp3;
	assign out[5] = in[5] ^ temp4;
	assign out[6] = in[6] ^ temp5;
	assign out[7] = in[7] ^ temp6;
	assign out[8] = in[8] ^ temp7;
	assign out[9] = in[9] ^ temp8;
	assign out[10] = in[10] ^ temp9;
	assign out[11] = in[11] ^ temp10;
	assign out[12] = in[12] ^ temp11;
	assign out[13] = in[13] ^ temp12;
	assign out[14] = in[14] ^ temp13;
	assign out[15] = in[15] ^ temp14;
	assign out[16] = in[16] ^ temp15;
	assign out[17] = in[17] ^ temp16;
	assign out[18] = in[18] ^ temp17;
	assign out[19] = in[19] ^ temp18;
	assign out[20] = in[20] ^ temp19;
	assign out[21] = in[21] ^ temp20;
	assign out[22] = in[22] ^ temp21;
	assign out[23] = in[23] ^ temp22;
	assign out[24] = in[24] ^ temp23;

endmodule 