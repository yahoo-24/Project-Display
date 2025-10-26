`timescale 1 ns / 100 ps 		//modify the parameters accordingly
module ClockDivider_tb;

	reg CLK_50MHz;
	reg rst_n;
	
	wire CLK_100Hz;
	wire CLK_1Hz;
	
	ClockDivider dut (
		CLK_50MHz,
		rst_n,
		CLK_100Hz,
		CLK_1Hz
	);
	
	initial begin
	
		CLK_50MHz <= 1'b0;
		rst_n <= 1'b0;
		#1;
		rst_n <= 1'b1;
		
		#500000060;
		
		rst_n <= 1'b0;
		#1;
		rst_n <= 1'b1;
		
		$stop;
	
	end
	
	always begin
		
		#5;
		CLK_50MHz <= ~CLK_50MHz;
		
	end

endmodule 