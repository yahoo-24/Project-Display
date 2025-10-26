`timescale 1 ns / 100 ps
module DLatchSingle_tb;

	reg clk;
	reg rst;
	reg in;
	wire Q;
	
	DLatchSingle dut (
		clk,
		rst,
		in,
		Q
	);
	
	always begin
	
		#5;
		clk <= ~clk; // Toggle the clk every 5ns
	
	end
	
	always @ (*) begin
	
		#0.1; // Give the output time to adjust to the new value
		if (rst & Q != 0) begin
		
			$display("Q did not reset to 0"); // Checking that it resets to 0 when rst is 1
		
		end
		else if (clk & in != Q & !rst) begin
		
			$display("Q is not following input"); // Checking that when the clk is HIGH Q follows in
		
		end
	
	end
	
	initial begin
	
		// Initialise the values
		clk <= 1'b0;
		rst <= 1'b1; // Reset on
		in <= 1'b0;
		#5;
		
		// Remove the reset and change the input values throughout
		rst <= 1'b0;
		in <= 1'b1;
		#3;
		in <= 1'b0;
		#7;
		
		in <= 1'b1;
		#15;
		
		in <= 1'b0;
		#10;
		
		in <= 1'b1;
		#10;
		
		// End it with a reset
		rst <= 1'b1;
		#15;
		
		$stop;
	
	end

endmodule 