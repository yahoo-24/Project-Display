`timescale 1 ns / 100 ps
module mux25bit2way_tb;

	reg [24:0] in1;
	reg [24:0] in2;
	reg sel;
	wire [24:0] out;
	
	mux25bit2way dut (
		in1,
		in2,
		sel,
		out
	);
	
	always begin
	
		#1; // Checking every 1 ns to see if the output follows in1 during sel = 0 and in2 during sel = 1
		if (sel & (out != in2) | ~sel & (out != in1))
		begin
		
			$display("Incorrect Output!");
		
		end
	
	end

	initial begin
		
		// Initialise the values and test with sel = 0
		in1 <= 25'd180;
		in2 <= 25'd500;
		sel <= 1'b0;
		#5;
		
		in1 <= 25'd900;
		in2 <= 25'd100000;
		sel <= 1'b0;
		#5;
		
		in1 <= 25'd2500000;
		in2 <= 25'd300000;
		sel <= 1'b0;
		#5;
		
		// Switch sel to 1
		in1 <= 25'd400000;
		in2 <= 25'd8000000;
		sel <= 1'b1;
		#5;
		
		in1 <= 25'd231234;
		in2 <= 25'd9902193;
		sel <= 1'b1;
		#5;
		
		in1 <= 25'd7958392;
		in2 <= 25'd1203993;
		sel <= 1'b1;
		#5;
		
		$stop;
	
	end

endmodule 