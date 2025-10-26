module SevenSegEncoder (
	
	input [7:0] LSBBinary,	//The LSB binary value input
	input [7:0] MSBBinary,	//The MSB bianry value input
	input ModeSel,			//Control signal for the internal Reverser
	input [2:0] TimeControl,//Manipulates the adder for the MSB in mode B
	
	output [6:0] HexMSBH,	//The 7-Seg display Signal for higer-digit in MSB
	output [6:0] HexMSBL,	//The 7-Seg display Signal for lower-digit in MSB
	output [6:0] HexLSBH,	//The 7-Seg display Signal for higer-digit in LSB
	output [6:0] HexLSBL	//The 7-Seg display Signal for lower-digit in LSB

);

	wire [3:0] BCD0, BCD1, BCD2, BCD3, ModifiedBCD2, ModifiedBCD3, add_value;
	wire [7:0] signal_1_seven_seg, signal_2_seven_seg, mux_out;
	wire unused1, unused2, unused3;

	//Double Dabble Output
	DoubleDabble8Bit DD8BitLSB (
		LSBBinary,
		{BCD1, BCD0}
	);
	
	DoubleDabble8Bit DD8BitMSB (
		MSBBinary,
		{BCD3, BCD2}
	);
	
	/*
	Mux4Bit8Way (
		4'b1000,
		4'b0111,
		4'b0110,
		4'b0101,
		4'b0100,
		4'b0011,
		4'b0010,
		4'b0001,
		TimeControl,
		add_value
	); I have not made this module so the other approach reuses pre-existing modules
	
	OR
	
	Subtractor4Bit (
		4'1000,
		{1'b0, TimeControl},
		add_value,
		unused3
	);
	*/
	// For the case of 2 (TimeControl = 3'b001) we need to add 7 as explained below.
	// For 3 (TimeControl = 3'b010) it would 6. Therefore, for TimeControl = X, we add 8 - X.
	Subtractor4Bit ValueToAdd (
		4'b1000,
		{1'b0, TimeControl[2:0]},
		add_value,
		unused3
	);
	
	// The point of the adder is that the reverser does 59 - input for Mode B
	// Therefore if MSB is 0 (i.e, output should be 2), then 59 - 0 = 59 which is wrong
	// The solution is to add 57 and hence 59 - 57 - 0 = 2 (assuming that we are going from 02.00 to 00.00)
	FourBitCarryLookAheadAdder BCD2Adder(
		BCD2,
		add_value, // 4'b0111,
		1'b0,
		ModifiedBCD2,
		unused1
	);
	
	// This is where we add the 5 from 57 and above we add the 7
	FourBitCarryLookAheadAdder BCD3Adder(
		BCD3,
		4'b0101,
		1'b0,
		ModifiedBCD3,
		unused2
	);
	
	// Based on the Mode we either pass the added version or the non-added version into the reverser
	mux8bit2way MuxMSB (
		{BCD3, BCD2},
		{ModifiedBCD3, ModifiedBCD2},
		ModeSel,
		mux_out
	);
	
	Reverser LSBReverser (
		{BCD1, BCD0},
		ModeSel,
		signal_1_seven_seg
	);
	
	Reverser MSBReverser (
		mux_out,
		ModeSel,
		signal_2_seven_seg
	);
	
	SevenSegSignalGen Hex1 (
		signal_1_seven_seg[3:0],
		HexLSBL
	);
	
	SevenSegSignalGen Hex2 (
		signal_1_seven_seg[7:4],
		HexLSBH
	);
	
	SevenSegSignalGen Hex3 (
		signal_2_seven_seg[3:0],
		HexMSBL
	);
	
	SevenSegSignalGen Hex4 (
		signal_2_seven_seg[7:4],
		HexMSBH
	);


endmodule 