module DFlipFlop25 (
    input clk,
    input rst,
    input [24:0] in,
    output [24:0] Q
);
	
	/* always @ (posedge clk or posedge rst) begin
		if (rst == 1'b1) begin
			Q <= 25'b0;
		end
		else begin
			Q <= in;
		end
	end */
	
	SpecialDFlipFlop8Bit DFFLSB (
		clk,
		rst,
		8'b0,
		in[7:0],
		Q[7:0]
	);
	
	SpecialDFlipFlop8Bit DFFMiddle (
		clk,
		rst,
		8'b0,
		in[15:8],
		Q[15:8]
	);
	
	SpecialDFlipFlop8Bit DFFMSB (
		clk,
		rst,
		8'b0,
		in[23:16],
		Q[23:16]
	);
	
	SpecialDFlipFlopSingle DFFBit24 (
		clk,
		rst,
		1'b0,
		in[24],
		Q[24]
	);

endmodule 