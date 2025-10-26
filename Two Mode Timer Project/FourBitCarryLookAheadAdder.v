module FourBitCarryLookAheadAdder (
	input [3:0] in1,
	input [3:0] in2,
	input cin,
	output [3:0] sum,
	output cout
);

	// The circuit schematic is from the Semester 1 presentation.

	wire [3:0] P, G; // Propagate and generate
	wire [3:0] C; // Carry
	wire [3:0] PC; // Propagate & Carry
	
	and(G[0], in1[0], in2[0]);
	xor(P[0], in1[0], in2[0]);
	and(G[1], in1[1], in2[1]);
	xor(P[1], in1[1], in2[1]);
	and(G[2], in1[2], in2[2]);
	xor(P[2], in1[2], in2[2]);
	and(G[3], in1[3], in2[3]);
	xor(P[3], in1[3], in2[3]);
	
	and(PC[0], P[0], cin);
	or(C[0], G[0], PC[0]);

	and(PC[1], P[1], C[0]);
	or(C[1], G[1], PC[1]);
	
	and(PC[2], P[2], C[1]);
	or(C[2], G[2], PC[2]);
	
	and(PC[3], P[3], C[2]);
	or(C[3], G[3], PC[3]);
	
	
	xor(sum[0], P[0], cin);
	xor(sum[1], P[1], C[0]);
	xor(sum[2], P[2], C[1]);
	xor(sum[3], P[3], C[2]);
	buf(cout, C[3]);
	

endmodule
