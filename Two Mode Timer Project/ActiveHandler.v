module ActiveHandler (
	input clk,
	input rst, // Set the active status to false and resets some of the latches
	input resetter, // Some latches use this reset instead of rst
	input StartStop, // Toggles the active status between start and stop (true and false)
	input StopCondition, // Set active to false (stopped) since timer finished
	output Active // Active status
);

	wire processed_StartStop, active_latch1, active_latch2, active_toggler, not_clk, rst_n;
	
	assign not_clk = ~clk;
	assign rst_n = ~rst;

	// Process the Start Stop button
	EdgeDetectorHolder StartStopProcessor (
		not_clk,
		rst,
		StartStop,
		processed_StartStop
	);
	
	// 2 Latches make up a register that store the status of active
	// The processed_StartStop from the edge detector and holder above is used here
	// It is used with the clock for the reason that the click may be faster than the clock. Therefore, both the StartStop and the clk can load the latch
	DLatchSingle ActiveIn (
		clk | processed_StartStop,
		resetter,
		Active,
		active_latch1
	);
	
	// The processed_StartStop is also used here to prevent both latches opening simultaneously
	/*
	The delayer is controlled by the inverted clock. If the StartStop click is done and removed during the HIGH
	clock cycle, the delayer will not be updated with the new value of active since it is open in the LOW cycle. The previous latch 
	would still be open since it is open in the HIGH clock; hence it reverts to the old value.
	The StartStopControlledMux will load it with new value and when the StartStop is removed it reverts to the old value as it remains open.
	That is why the edge holder keeps holding the StartStop during the clk HIGH cycle up to the falling edge because after the hold is removed
	the previous latch running on clk HIGH is now locked so it cannot be updated again and the latch below is updated with
	the new value and hence the Mux that follows this latch will be using the updated value and not the old one as well.
	*/
	DLatchSingle Delayer (
		not_clk & ~processed_StartStop,
		resetter,
		active_latch1,
		active_latch2
	);
	
	// Using the StartStop flag processed by the edge detector lets us toggle the active state
	mux2way StartStopControlledMux (
		active_latch2,
		~active_latch2,
		processed_StartStop,
		active_toggler
	);
	
	// This is the mux that forces active to 0 either because reset is clicked or because the time finished
	mux2way ActiveReseter (
		1'b0,
		active_toggler,
		rst_n & ~StopCondition,
		Active
	);

endmodule 