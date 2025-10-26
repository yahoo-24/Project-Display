module MainCode (

	input CLK_50MHz,			//50MHz clock input
	input CLK_100Hz, //<--you can use this for your module ModelSim validation
	input CLK_1Hz,	 //<--you can use this for your module ModelSim validation
	input rst_n,				//An active-low sginal to reset the module
	input StartStop,			//A control signal to start (active) and stop (pause) the module
	input ModeSel,				//A control signal to swich between the two modes (10s stopwatch and 2 mins timer)
	input [2:0] TimeControl, // TimeControl sets the time countdown for mode B
	
	output [6:0] HexMSBH,	//The 7-Seg display Signal for higer-digit in MSB
	output [6:0] HexMSBL,	//The 7-Seg display Signal for lower-digit in MSB
	output [6:0] HexLSBH,	//The 7-Seg display Signal for higer-digit in LSB
	output [6:0] HexLSBL,	//The 7-Seg display Signal for lower-digit in LSB
	output DOT,					//The flashing decimal signal at the lower-digit MSB
	output StopLED, // An LED that turns on when the Timer reaches its limit
	output FlashingLED // Flashed an LED when the timer is about to finish

);

	wire CLK_100Hz1, CLK_1Hz1, clk;
	wire [7:0] LSB, MSB;

	ClockDivider ClkDivider (
		CLK_50MHz,
		rst_n,
		CLK_100Hz1,
		CLK_1Hz1
	);
	
	mux2way ClkMux (
		CLK_100Hz,
		CLK_1Hz,
		ModeSel,
		clk
	);
	
	TimerCoreLogic TimerCore (
		clk,
		rst_n,
		StartStop,
		ModeSel,
		TimeControl,
		LSB,
		MSB,
		StopLED
	);
	
	Flasher LEDFlasher (
		CLK_1Hz,
		TimeControl,
		StopLED,
		ModeSel,
		MSB,
		LSB,
		FlashingLED
	);
	
	SevenSegEncoder Encoder (
		LSB,
		MSB,
		ModeSel,
		TimeControl,
		HexMSBH,
		HexMSBL,
		HexLSBH,
		HexLSBL
	);
	
	buf(DOT, clk);

endmodule 