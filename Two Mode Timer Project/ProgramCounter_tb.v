`timescale 1 ns / 100 ps 		//modify the parameters accordingly

module ProgramCounter_tb;

	reg clk;
	reg [7:0] ResetVal;
	reg [7:0] LoadVal;
	reg reset;
	reg load;
	reg inc;
	
	wire [7:0] PCoutput;
	
	ProgramCounter dut (
		clk,
		ResetVal,
		LoadVal,
		reset,
		load,
		inc,
		PCoutput
	);
	
	integer i;
	
	always begin
	
		#5;
		clk <= ~clk;
	
	end
	
	initial begin
	
		// Initialise values. Reset on
		reset <= 1'b1;
		load <= 1'b1;
		inc <= 1'b1;
		ResetVal <= 8'b0;
		LoadVal <= 8'b10010001;
		clk <= 1'b0;
		#10;
		
		ResetVal <= 8'b1; // Change reset value to see if it applies
		#10;
		
		// Set Load and remove reset. Inc set but load > inc
		load <= 1'b1;
		reset <= 1'b0;
		inc <= 1'b1;
		#20;
		
		LoadVal <= 8'b10011101; // Check that it loads new value and that no inc occurs
		#10;
		
		load <= 1'b0; // Remove load. It should inc now	
		#100;
		
		load <= 1'b1; // Put load again
		LoadVal <= 8'b00000001; // Change load value
		#30;
		
		load <= 1'b0; // Remove load. It should inc
		#40;
		
		inc <= 1'b0; // Remove inc. The PC value should stay as is
		#40;
		
		reset <= 1'b1; // Reset and test with 8 different ResetVals
		for (i = 0; i < 8; i = i + 1) begin
			ResetVal = $urandom() % 256;
			#10;
		end
		
		$stop;
		
	end


endmodule 