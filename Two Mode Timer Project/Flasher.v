module Flasher (
	input CLK_1Hz,
	input [2:0] TimeControl,
	input Stopped,
	input ModeSel,
	input [7:0] MSB,
	input [7:0] LSB,
	output FlashingLED
);

	wire lsb_is_49, msb_is_90, msb_is_stop_value, mode_a_flashing_cond, mode_b_flashing_cond, toggled_flash, unused;
	wire condition_checker_mux_out;
	wire [3:0] msb_stop_value, msb_equal_stop_value;

	// LSB flashes when it is at least 0011 0001 and MSB is at TimeControl + 1 (Mode B). This happens when LSB 5 and 4 are set and any of 0 to 3 are also set.
	assign lsb_is_49 = LSB[5] & LSB[4] & (LSB[3] | LSB[2] | LSB[1] | LSB[0]);	
	// Flashes when MSB is at least 90 = 0101 1010 in mode A. Happens when MSB 6 is is on and either one of 5 or 4.
	// If bit 4 is the one set then bit 3 has to be set and either one of 2 or 1.
	assign msb_is_90 = MSB[6] & (MSB[5] | (MSB[4] & MSB[3] & (MSB[2] | MSB[1])));

	FourBitCarryLookAheadAdder TimeControlAdder (
		{1'b0, TimeControl},
		4'b0001,
		1'b0,
		msb_stop_value,
		unused
	);
	
	// XOR gates output 0 if 2 bits are the same. The 4 most MSB will always be 0 so they are ignored
	assign msb_equal_stop_value = MSB[3:0] ^ msb_stop_value;
	// The NOR gate checks that all bits are 0 i.e. MSB == msb_stop_value
	assign msb_is_stop_value = ~(msb_equal_stop_value[0] | msb_equal_stop_value[1] | msb_equal_stop_value[2] | msb_equal_stop_value[3]);
	
	assign mode_a_flashing_cond = msb_is_90 & ~ModeSel; // MSB has to be 90 for mode A to flash
	assign mode_b_flashing_cond = msb_is_stop_value & lsb_is_49 & ModeSel; // MSB has to be TimeControl+1 and LSB=49 to flash in Mode B
	
	SpecialDFlipFlopSingle Toggler (
		CLK_1Hz,
		Stopped,
		1'b0,
		~FlashingLED,
		toggled_flash
	);
	
	// The condition checker will output the toggled flash if either condition A or B is met
	mux2way ConditionChecker (
		1'b0,
		toggled_flash,
		mode_a_flashing_cond | mode_b_flashing_cond,
		condition_checker_mux_out
	);
	
	// If the stop is high then the timer finished so stop flashing.
	mux2way FlashStopper (
		condition_checker_mux_out,
		1'b0,
		Stopped,
		FlashingLED
	);

endmodule 