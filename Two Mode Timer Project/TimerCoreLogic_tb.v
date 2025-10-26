`timescale 1 ns / 100 ps 		//modify the parameters accordingly

module TimerCoreLogic_tb;

	reg clk;
	reg rst_n;
	reg StartStop;
	reg ModeSel;
	reg [2:0] TimeControl;

	wire [7:0] LSBbinaryout;
	wire [7:0] MSBbinaryout;
	wire StopLED;
	
	TimerCoreLogic dut (
		clk,
		rst_n,
		StartStop,
		ModeSel,
		TimeControl,
		LSBbinaryout,
		MSBbinaryout,
		StopLED
	);
	
	always begin
	
		#5;
		clk <= ~clk;
	
	end
	
	initial begin
	
		TimeControl <= 3'b001; // Set time to 2 minutes
		rst_n <= 1'b0;
		clk <= 1'b0;
		StartStop <= 1'b1;
		ModeSel <= 1'b0;
		#1;
		// Start and remove reset
		StartStop <= 1'b0;
		rst_n <= 1'b1;
		#10;
		// Remove the start
		StartStop <= 1'b1;
		#5000;
		// Stop
		StartStop <= 1'b0;
		#2;
		// Remove stop
		StartStop <= 1'b1;
		#200;
		// Continue
		StartStop <= 1'b0;
		#100;
		StartStop <= 1'b1;
		#100000;
		
		// Reset and change mode
		rst_n <= 1'b0;
		ModeSel <= 1'b1;
		#1;
		// Start and remove reset
		StartStop <= 1'b0;
		rst_n <= 1'b1;
		#10;
		// Remove start
		StartStop <= 1'b1;
		#2000;
		
		// Change to different time - 3 minutes
		rst_n <= 1'b0; // Reset
		TimeControl <= 3'b010;
		#1;
		rst_n <= 1'b1;
		StartStop <= 1'b0; // Start
		
		#10;
		StartStop <= 1'b1; // Unclick button
		
		#3000;
		
		// Change to different time - 1 minutes
		rst_n <= 1'b0; // Reset
		TimeControl <= 3'b000;
		#1;
		rst_n <= 1'b1;
		StartStop <= 1'b0; // Start
		
		#10;
		StartStop <= 1'b1; // Unclick button
		
		#1000;
		
		
		$stop;
	
	end

endmodule 