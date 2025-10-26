// Reference: https://en.wikipedia.org/wiki/Double_dabble
// The circuit is build based on the schematics from the link above
module DoubleDabble8Bit (
	input [7:0] Binary,
	output [7:0] BCD
);

	wire [3:0] adder1_out, adder2_out, adder3_out, adder4_out, adder5_out;
	wire unused;

	buf(BCD[0], Binary[0]);

	ConditionalAdder Adder1 (
		{1'b0, Binary[7:5]},
		adder1_out
	);
	
	ConditionalAdder Adder2 (
		{adder1_out[2:0], Binary[4]},
		adder2_out
	);
	
	ConditionalAdder Adder3 (
		{adder2_out[2:0], Binary[3]},
		adder3_out
	);
	
	ConditionalAdder Adder4 (
		{adder3_out[2:0], Binary[2]},
		adder4_out
	);
	
	ConditionalAdder Adder5 (
		{adder4_out[2:0], Binary[1]},
		BCD[4:1]
	);
	
	ConditionalAdder Adder6 (
		{1'b0, adder1_out[3], adder2_out[3], adder3_out[3]},
		adder5_out
	);
	
	ConditionalAdder Adder7 (
		{adder5_out[2:0], adder4_out[3]},
		{unused, BCD[7:5]}
	);

endmodule 