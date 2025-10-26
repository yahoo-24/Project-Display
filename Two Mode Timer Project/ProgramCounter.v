module ProgramCounter (
	
	input clk,					//Clock Signal (100Hz or 1Hz)
	input [7:0] ResetVal,	//input value to update the PC wehn load is active
	input [7:0] LoadVal,		//input value to update the PC when reset is active
	input reset,					//An active-high singal to rest the PC (exectute without clk)
	input load,					//An active-high signal to load value in PC (function activated with clk)
	input inc,					//An active-high singal to start the counting
	
	output [7:0] PCoutput	//PC output signal in binary

);

	wire [7:0] sum;
	wire [7:0] load_out;
	wire [7:0] inc_out;
	wire [7:0] dff_out;
	
	// This is the adder that increments the PC.
	Incrementor8Bit Adder (
		PCoutput,
		sum
	);
	
	// A mux that chooses between the previous value and the inc'ed value. Controlled by inc flag
	mux8bit2way IncMux (
		PCoutput,
		sum,
		inc,
		inc_out
	);
	
	// Selects between the previous Mux output and load. Controlled by load flag
	mux8bit2way LoadMux (
		inc_out,
		LoadVal,
		load,
		load_out
	);
	
	// The DFF that stores the PC value and at the same time can be reset to ResetVal
	SpecialDFlipFlop8Bit FlipFlop (
		clk,
		reset,
		ResetVal,
		load_out,
		PCoutput
	);
		
endmodule
