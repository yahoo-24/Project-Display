`timescale 1 ns / 100 ps
module ActiveHandler_tb;

	reg clk, rst, resetter, StartStop, StopCondition;
	wire Active;

	ActiveHandler dut (
		clk,
		rst,
		resetter,
		StartStop,
		StopCondition,
		Active
	);
	
	always begin
		#5;
		clk <= ~clk;
	end
	
	initial begin
	
		clk <= 1'b0;
		rst <= 1'b1;
		resetter <= 1'b1; // If the rst is 1 then resetter has to always be 1 as it relies on rst
		StartStop <= 1'b0;
		StopCondition <= 1'b0;
		#10;
		
		// Remove rst
		rst <= 1'b0;
		resetter <= 1'b0;
		#10;
		
		// Long StartStop
		StartStop <= 1'b0;
		#20;
		
		StartStop <= 1'b1;
		#10;
		
		// Short StartStop
		StartStop <= 1'b0;
		#1;
		
		StartStop <= 1'b1;
		#10;
		
		// Turn active to true for next test
		StartStop <= 1'b0;
		#10;
		StartStop <= 1'b1;
		#10;
		
		// Turn resetter (due to ModeSel or TimeControl change)
		resetter <= 1'b1;
		#10;
		
		// Turn active to true for next test
		resetter <= 1'b0;
		StartStop <= 1'b0;
		#10;
		StartStop <= 1'b1;
		#10;
		
		// StopCondition
		StopCondition <= 1'b1;
		#15;
		
		$stop;
	
	end

endmodule
