module TimerCoreLogic (

								//Edit your own input configuration parameters here
	
	input clk,				//Clock Signal (100Hz or 1Hz)
	input rst_n,			//Active-LOW reset signal
	input StartStop,		//A control signal to start (active) and stop (pause) the module
	input ModeSel,
	input [2:0] TimeControl, // Sets the time for Mode B

	output [7:0] LSBbinaryout,	//The Least Signaficant Bit (LSB) output signal in binary
	output [7:0] MSBbinaryout,		//The Most Signaficant Bit (MSB) output signal in binary
	output StopLED // When the timer reaches its limit the LED turns on


);

	wire active, inc, load, load_msb, stop_cond, rst, not_clk, resetter, negative_resetter, unused;
	wire lsb_go_to_zero_cond, msb_load_cond, inc_rst_mux_in;
	wire [7:0] reset_val;
	wire [7:0] LSB, MSB;
	wire [3:0] msb_stop_value, msb_equal_stop_value;
	wire lsb_is_99, lsb_is_59, msb_is_99, msb_is_zero, msb_is_stop_value;
	
	not(rst, rst_n);
	not(not_clk, clk);
	
	// The Resetter will reset the PC and a few other components if it detects a change in ModeSel or TimeControl
	Resetter ConfigurationChange (
		clk,
		rst,
		ModeSel,
		TimeControl,
		resetter
	);
	
	not(negative_resetter, resetter); // Used for modules that reset on a falling edge
	
	// MSB PC
	ProgramCounter PC1 (
		clk,
		8'b0,
		8'b00000001,
		resetter,
		load_msb,
		inc & active,
		MSB
	);
	
	// LSB PC
	ProgramCounter PC2 (
		clk,
		reset_val,
		8'b0,
		resetter,
		load & active,
		active,
		LSB
	);
	
	assign LSBbinaryout = LSB;
	assign MSBbinaryout = MSB;
	
	// The code below are condition checks and mode configurations that determine whether to load or inc
	
	// lsb_is_99 checks that the bits make up 99 (01100011)
	assign lsb_is_99 = ~LSBbinaryout[7] & LSBbinaryout[6] & LSBbinaryout[5] & ~LSBbinaryout[4] & ~LSBbinaryout[3] & ~LSBbinaryout[2] & LSBbinaryout[1] & LSBbinaryout[0];
	
	// lsb_is_59 checks that the bits make up 59 (00111011)
	assign lsb_is_59 = ~LSBbinaryout[7] & ~LSBbinaryout[6] & LSBbinaryout[5] & LSBbinaryout[4] & LSBbinaryout[3] & ~LSBbinaryout[2] & LSBbinaryout[1] & LSBbinaryout[0];
	
	// msb_is_zero checks that the bits make up 0
	nor(msb_is_zero, MSBbinaryout[0], MSBbinaryout[1], MSBbinaryout[2], MSBbinaryout[3], MSBbinaryout[4], MSBbinaryout[5], MSBbinaryout[6], MSBbinaryout[7]);
	
	assign lsb_go_to_zero_cond = (lsb_is_99 & ~ModeSel | lsb_is_59 & ModeSel); // Tells the PC to increment MSB and return LSB to 0
	assign msb_load_cond = (msb_is_zero & active & ModeSel); // This allows us to start at 2.00 and then jump directly to 1.59 avoiding 2.59
	
	// msb_is_99 checks that the bits make up 99 (01100011)
	assign msb_is_99 = ~MSBbinaryout[7] & MSBbinaryout[6] & MSBbinaryout[5] & ~MSBbinaryout[4] & ~MSBbinaryout[3] & ~MSBbinaryout[2] & MSBbinaryout[1] & MSBbinaryout[0];
	
	// We have to add 1 since 000 is used to set the timer to 1 and 111 to 8
	FourBitCarryLookAheadAdder TimeControlAdder (
		{1'b0, TimeControl[2:0]},
		4'b0001,
		1'b0,
		msb_stop_value,
		unused
	);
	
	// XOR gates output 0 if 2 bits are the same. The 4 most MSB will always be 0 so they are ignored
	assign msb_equal_stop_value = MSBbinaryout[3:0] ^ msb_stop_value;
	// The NOR gate checks that all bits are 0 i.e. MSBbinaryout == msb_stop_value
	assign msb_is_stop_value = ~(msb_equal_stop_value[0] | msb_equal_stop_value[1] | msb_equal_stop_value[2] | msb_equal_stop_value[3]);
	
	assign stop_cond = (msb_is_99 & lsb_is_99 & ~ModeSel | msb_is_stop_value & lsb_is_59 & ModeSel);
	buf(StopLED, stop_cond);
	
	// If Mode A then reset is 0 but if Mode B then reset is 59
	// This 59 is needed as reverser does 59 - 59 = 0 so we see 2.00
	// After that lsb_go_to_zero_cond immediately brings it down to 0
	mux8bit2way RstVal (
		8'b00000000,
		8'b00111011,
		ModeSel,
		reset_val
	);
	
	// As explained above msb_load_cond allows us to go from 2.00 to 1.59 instead of 2.59
	// MSB starts as 0 (reverser 2 - 0 = 2) which causes msb_load_cond = 1 hence 1 would be loaded as MSB load flag is HIGH
	mux2way MSBLoad (
		1'b0,
		1'b1,
		msb_load_cond,
		load_msb
	);
	
	// Once LSB reaches 99 (A) or 59 (B) load is set to load 0 in LSB PC
	mux2way LSBLoad (
		1'b0,
		1'b1,
		lsb_go_to_zero_cond,
		load
	);
	
	// Tell the MSB to inc when LSB is 99 or 59
	mux2way MSBInc (
		1'b0,
		1'b1,
		lsb_go_to_zero_cond,
		inc_rst_mux_in
	);
	
	// We have to make sure that the reset is not clicked
	mux2way IncMux (
		1'b0,
		inc_rst_mux_in,
		negative_resetter,
		inc
	);
	
	// The ActiveHandler is a register that stores and updates the timer's active status
	ActiveHandler TimerActive (
		clk,
		rst, // rst will set the active status to false
		resetter, // Like the rst but some modules reset based on the resetter while others on the rst
		StartStop, // Toggles the active status
		stop_cond, // Sets the active status to false
		active // The timer's active status
	);

endmodule 