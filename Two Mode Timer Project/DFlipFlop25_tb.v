`timescale 1 ns / 100 ps
module DFlipFlop25_tb;

	reg clk;
   reg rst;
   reg [24:0] in;
   wire [24:0] Q;
	
	DFlipFlop25 dut (
		 clk,
		 rst,
		 in,
		 Q
	);
	
	always begin
	
		#5;
		clk <= ~clk; // Toggle the clock every 5ns (10ns time period)
		
	end
	
	always @ (posedge rst) begin
	
		#0.1; // A tiny delay gives Q time to adjust to the new value
		// If the reset is on and Q is not 0 then output an error message
		if (rst & Q != 0) begin
			$display("Q did not reset to 0");
		end
		
	end
	
	always @ (posedge clk) begin
	
		#0.1; // A tiny delay gives Q time to adjust to the new value
		// At the rising edge 'Q' should be equal to 'in' unless there is a reset. Output an error if that is not the case
		if (!rst & Q != in) begin
			$display("Q is not following input");
		end
		
	end
	
	initial 
	begin
	
		// Initialise values
		clk <= 1'b0;
		rst <= 1'b1; // Reset on
		in <= 25'd600;
		#5;
		
		// Remove the reset
		rst <= 1'b0;
		#12;
		
		// Changing the values of the input
		in <= 25'd30;
		#24;
		
		in <= 25'd70;
		#1;
		
		in <= 25'd900;
		#3;
		
		in <= 25'd800;
		#15;
		
		// Lastly, reseting
		rst <= 1'b1;
		#10;
		
		$stop;
	
	end
	
endmodule 