module EdgeDetectorHolder (
	input not_clk,
	input rst,
	input StartStop,
	output ProcessedStartStop
);

	wire not_StartStop, detected_edge, reg_stored_StartStop;

	// Falling Edge Detector 
	// Tackles the problem where the SS is clicked and removed faster than time period or pressed for long time
	not(not_StartStop, StartStop);
	// The Flip Flop is used as a register.
	// The Flip Flop causes a delay up to the next clk falling edge which means the output is HIGH for a short duration
	// The StartStop is used as part of the reset because the clock may take time to reload the 1 by which time a click might occur. 
	SpecialDFlipFlopSingle RegIn (
		not_clk,
		rst | StartStop,
		1'b1,
		StartStop,
		reg_stored_StartStop
	);
	
	and(detected_edge, not_StartStop, reg_stored_StartStop);
	
	// The holder is a D latch that not only accepts clock, but also the StartStop
	// This is because if the click is too fast, then the clock does not have enough time to load the latch so not_StartStop is used to forcefully load the latch
	// The latch holds the StartStop output during the positive clock cycle before ~clk removes it. Rhis gives enough time for the latches
	// in the Active Handler to update. See the ActiveHandler Latches for more information
	DLatchSingle EdgeHolder (
		not_clk | not_StartStop,
		rst,
		detected_edge,
		ProcessedStartStop
	);

endmodule 