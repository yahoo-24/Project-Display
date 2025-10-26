module DLatchSingle (
    input clk,
    input rst,
    input in,
    output Q
);

	/*

	always @ (*) begin
		if (rst == 1'b1) begin
			Q <= 1'b0;
		end
		else if (clk == 1'b1) begin
			Q <= in;
		end
	end
	
	*/
	
	wire not_in, not_in_nand, in_nand;
	wire Q_and, Q_temp, not_Q;
	
	not(not_in, in); // Invert input
	
	nand(not_in_nand, clk, not_in); // The nand for the inverted input
	nand(in_nand, clk, in); // The nand for the normal input
	
	nand(not_Q, not_in_nand, Q_and); // The nand for Q bar
	nand(Q_temp, in_nand, not_Q); // The nand for Q which goes into the and gate
	
	and(Q_and, ~rst, Q_temp); // When rst is 1, ~rst = 0 hence the reset is forced on Q
	
	assign Q = Q_and;

endmodule 