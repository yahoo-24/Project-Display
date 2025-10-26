module mux8bit2way (
	input [7:0] in1,
	input [7:0] in2,
	input sel,
	output [7:0] out
);
	
	mux2way MuxBit0 (
		in1[0],
		in2[0],
		sel,
		out[0]
	);
	
	mux2way MuxBit1 (
		in1[1],
		in2[1],
		sel,
		out[1]
	);
	
	mux2way MuxBit2 (
		in1[2],
		in2[2],
		sel,
		out[2]
	);
	
	mux2way MuxBit3 (
		in1[3],
		in2[3],
		sel,
		out[3]
	);
	
	mux2way MuxBit4 (
		in1[4],
		in2[4],
		sel,
		out[4]
	);
	
	mux2way MuxBit5 (
		in1[5],
		in2[5],
		sel,
		out[5]
	);
	
	mux2way MuxBit6 (
		in1[6],
		in2[6],
		sel,
		out[6]
	);
	
	mux2way MuxBit7 (
		in1[7],
		in2[7],
		sel,
		out[7]
	);
	
endmodule
