`timescale 1 ns / 100 ps
module Resetter_tb;

	reg clk, rst, ModeSel;
	reg [2:0] TimeControl;
	wire resetter;

	Resetter dut (
		clk,
		rst,
		ModeSel,
		TimeControl,
		resetter
	);
	
	integer i;
	
	always begin
		#5;
		clk <= ~clk;
	end
	
	initial begin
	
		clk <= 1'b0;
		rst <= 1'b1;
		ModeSel <= 1'b0;
		TimeControl <= 3'b000;
		#1;
		if (!resetter) begin
			$display("Reset was not set when rst is set!");
		end
		#9;
		
		// Remove rst
		rst <= 1'b0;
		#1;
		if (resetter) begin
			$display("Reset was not removed when rst is removed!");
		end
		#9;
		
		// Change ModeSel
		ModeSel <= 1'b1;
		#1;
		if (!resetter) begin
			$display("Reset was not set when ModeSel changed!");
		end
		#9;
		
		// ModeSel should have updated so output should go LOW again
		if (resetter) begin
			$display("Reset is still 1!");
		end
		
		// Change TimeControl during Mode B
		for (i = 0; i < 8; i = i + 1) begin
		
			TimeControl <= i;
			#1;
			if (!resetter) begin
				$display("Reset was not set when TimeControl changed!");
			end
			#9;
			
			// TimeControl should have updated so output should go LOW again
			if (resetter) begin
				$display("Reset is still 1: %d!", i);
			end
		
		end
		
		// Change TimeControl during Mode A. No reset is expected
		ModeSel <= 1'b1;
		TimeControl <= 3'b001;
		#1;
		if (resetter) begin
			$display("Should not reset during mode A");
		end
		#9;
		
		// Test 1 more time
		ModeSel <= 1'b1;
		TimeControl <= 3'b010;
		#1;
		if (resetter) begin
			$display("Should not reset during mode A");
		end
		#9;
		
		$stop;
	
	end

endmodule
