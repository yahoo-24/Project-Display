`timescale 1 ns / 100 ps
module EdgeDetectorHolder_tb;

	reg not_clk, rst, StartStop;
	wire ProcessedStartStop;

	EdgeDetectorHolder dut (
		not_clk,
		rst,
		StartStop,
		ProcessedStartStop
	);
	
	always begin
		#5;
		not_clk <= ~not_clk;
	end
	
	initial begin
	
		not_clk <= 1'b0;
		rst <= 1'b1;
		StartStop <= 1'b1;
		#10;
		
		// Long StartStop press
		rst <= 1'b0; // Remove reset
		StartStop <= 1'b0;
		#20;
		
		StartStop <= 1'b1;
		#11;
		
		// Short StartStop press on clock LOW
		StartStop <= 1'b0;
		#1;
		
		StartStop <= 1'b1;
		#25;
		
		// Short StartStop press on clock HIGH
		StartStop <= 1'b0;
		#1;
		
		StartStop <= 1'b1;
		#10;
		
		// 2 quick presses
		StartStop <= 1'b0;
		#1;
		StartStop <= 1'b1;
		#3;
		StartStop <= 1'b0;
		#1;
		StartStop <= 1'b1;
		#14;
		
		$stop;
	
	end
	
endmodule 