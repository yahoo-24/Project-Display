module mux25bit2way (
	input [24:0] in1,
	input [24:0] in2,
	input sel,
	output [24:0] out
);
	
	mux8bit2way MuxLSB (
		in1[7:0],
		in2[7:0],
		sel,
		out[7:0]
	);
	
	mux8bit2way MuxMiddle8 (
		in1[15:8],
		in2[15:8],
		sel,
		out[15:8]
	);
	
	mux8bit2way MuxMSB (
		in1[23:16],
		in2[23:16],
		sel,
		out[23:16]
	);
	
	mux2way MuxBit24 (
		in1[24],
		in2[24],
		sel,
		out[24]
	);

endmodule
