module Reverser (
	
	input [7:0] RevIn,	//The 8-bit BCD value input. Tips that is two 4-bit BCDs
	input ModeSel,			//Control signal to make the counting upwards RevIn to Counting downwards RevOut (HIGH) or pass the RevIn to the RevOut (LOW).
	
	output [7:0] RevOut	//the 8-bit BCD value out
	
);

	wire unused1, unused2;
	wire [3:0] result1, result2;

	// The reverser subtracts 59 by the BCD value. After 10s from 59 it will read 59 - 10 = 49s.
	// For the MSB, the 7-seg Encoder conditions the signal for the reverser.

	Subtractor4Bit Subtractor1(
		4'b1001,
		RevIn[3:0],
		result1,
		unused1
	); // Subtracting 9 by the 4 LSB
	
	Subtractor4Bit Subtractor2(
		4'b0101,
		RevIn[7:4],
		result2,
		unused2
	); // Subtracting 5 by the 4 MSB

	/*
	always @ (RevIn) begin
		if (ModeSel == 1'b0) begin
			RevOut = RevIn;
		end
		else begin
			RevOut = {result2, result1};
		end
	end
	*/
	
	// Selects between the reversed input and the normal input. Controlled by ModeSel
	mux8bit2way Mux (
		RevIn,
		{result2, result1},
		ModeSel,
		RevOut
	);
	


endmodule
