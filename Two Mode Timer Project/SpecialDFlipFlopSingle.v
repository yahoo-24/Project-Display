module SpecialDFlipFlopSingle (
    input clk,
    input rst,
	input Reset_Val,
    input in,
    output reg Q
);

	always @ (posedge clk or posedge rst) begin
		if (rst == 1'b1) begin
			Q <= Reset_Val;
		end
		else begin
			Q <= in;
		end
	end

endmodule 