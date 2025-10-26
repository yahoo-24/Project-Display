module mux2way (
	input in1,
	input in2,
	input sel,
	output out
);

	wire not_sel;
	wire out1, out2;
	
	// The circuit schematic of a mux is well known out = (in1 & ~sel) | (in2 & sel)

	not(not_sel, sel);
	
	and(out1, in1, not_sel);
	
	and(out2, in2, sel);
	
	or(out, out1, out2);

endmodule
