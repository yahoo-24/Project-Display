// The module resets the PC and TimeCore when it detects a change in ModeSel or TimeControl
module Resetter (
	input clk,
	input rst,
	input ModeSel,
	input [2:0] TimeControl,
	output resetter
);

	wire [2:0] stored_TimeControl, TimeControl_change_detector;
	wire stored_ModeSel, ModeSel_change_detector;

	// When the mode is changed, the timer resets
	// Therefore, we use a register to store the previous value of ModeSel
	// A special reset is used as it needs to reset to the current ModeSel not to 0 (Mode A)
	SpecialDFlipFlopSingle ModeSelChangeReset (
		clk,
		rst,
		ModeSel,
		ModeSel,
		stored_ModeSel
	);

	// Similar to the method above, the 3 FlipFlops below detect any changes in the 3 TimeControl bits
	SpecialDFlipFlopSingle TimeControlBit2(
		clk,
		rst,
		TimeControl[2],
		TimeControl[2],
		stored_TimeControl[2]
	);
	
	SpecialDFlipFlopSingle TimeControlBit1 (
		clk,
		rst,
		TimeControl[1],
		TimeControl[1],
		stored_TimeControl[1]
	);
	
	SpecialDFlipFlopSingle TimeControlBit0 (
		clk,
		rst,
		TimeControl[0],
		TimeControl[0],
		stored_TimeControl[0]
	);
	
	// A change in mode, control outputs 1. XOR gate outputs 1 if the inputs are different
	assign ModeSel_change_detector = ModeSel ^ stored_ModeSel;
	// A change in any one of the control bits will give an output of 1 on that bit
	assign TimeControl_change_detector = TimeControl ^ stored_TimeControl;
	
	// Resets either using the reset, mode or control change. The control only resets if we are at mode B hence the and gate
	assign resetter = rst | ModeSel_change_detector | ((TimeControl_change_detector[0] | TimeControl_change_detector[1] | TimeControl_change_detector[2]) & ModeSel);

endmodule 