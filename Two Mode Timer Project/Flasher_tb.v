`timescale 1 ns / 100 ps
module Flasher_tb;

	wire FlashingLED;
	reg CLK_1Hz;
	reg [2:0] TimeControl;
	reg Stopped;
	reg ModeSel;
	reg [7:0] MSB;
	reg [7:0] LSB;
	
	Flasher dut (
		CLK_1Hz,
		TimeControl,
		Stopped,
		ModeSel,
		MSB,
		LSB,
		FlashingLED
	);
	
	always begin
		#2;
		CLK_1Hz <= ~CLK_1Hz;
	end
	
	initial begin
	
		CLK_1Hz <=  1'b0;
		TimeControl <= 3'b000;
		Stopped <= 1'b0;
		ModeSel <= 1'b0;
		MSB <= 8'b00001111;
		LSB <= 8'b00000000;
		
		#10;
		
		// Meeting Condition B although we are in A. No Flash expected
		LSB <= 8'd49;
		MSB <= 8'd1;
		#20;
		
		// Switching to Mode B with the condition met previously. Flash Expected
		ModeSel <= 1'b1;
		#20;
		
		// Changing the value of LSB but still condition is met. Still Flashing
		LSB <= 8'd50;
		#10;
		
		// Force Stop it using Stopped. No Flash
		Stopped <= 1'b1;
		#20;
		
		// Condition is not met now for mode B and stopped removed. Still No Flash
		LSB <= 8'd0;
		MSB <= 8'd80;
		Stopped <= 1'b0;
		#20;
		
		// Mode A condition is met. Swithcing to Mode A. Flash Expected
		MSB <= 8'd90;
		ModeSel <= 1'b0;
		#20;
		
		// Value changes but condition still met. Still Flashing
		MSB <= 8'd95;
		#20;
		
		// Wrong mode now. No Flash
		ModeSel <= 1'b1;
		#10;
		
		// Stop it for the next test.
		Stopped <= 1'b1;
		#30;
		
		// Try with different TimeControl
		TimeControl <= 3'b100;
		MSB <= 8'd5;
		LSB <= 8'd49;
		Stopped <= 1'b0;
		ModeSel <= 1'b1;
		#30;
		
		$stop;
	
	end

endmodule 