module SpecialDFlipFlop8Bit (
    input clk,
    input rst,
	input [7:0] Reset_Val,
    input [7:0] in,
    output [7:0] Q
);

	/* always @ (posedge clk or posedge rst) begin
		if (rst == 1'b1) begin
			Q <= Reset_Val;
		end
		else begin
			Q <= in;
		end
	end */
	
	SpecialDFlipFlopSingle DFFBit0 (
		clk,
		rst,
		Reset_Val[0],
		in[0],
		Q[0]
	);
	
	SpecialDFlipFlopSingle DFFBit1 (
		clk,
		rst,
		Reset_Val[1],
		in[1],
		Q[1]
	);
	
	SpecialDFlipFlopSingle DFFBit2 (
		clk,
		rst,
		Reset_Val[2],
		in[2],
		Q[2]
	);
	
	SpecialDFlipFlopSingle DFFBit3 (
		clk,
		rst,
		Reset_Val[3],
		in[3],
		Q[3]
	);
	
	SpecialDFlipFlopSingle DFFBit4 (
		clk,
		rst,
		Reset_Val[4],
		in[4],
		Q[4]
	);
	
	SpecialDFlipFlopSingle DFFBit5 (
		clk,
		rst,
		Reset_Val[5],
		in[5],
		Q[5]
	);
	
	SpecialDFlipFlopSingle DFFBit6 (
		clk,
		rst,
		Reset_Val[6],
		in[6],
		Q[6]
	);
	
	SpecialDFlipFlopSingle DFFBit7 (
		clk,
		rst,
		Reset_Val[7],
		in[7],
		Q[7]
	);

endmodule 