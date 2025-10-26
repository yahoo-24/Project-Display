module ClockDivider (

	input CLK_50MHz, 		//50MHz clock input
	input rst_n, 			//Active-LOW reset signal
	
	output CLK_100Hz,	 	//100Hz clock; You can tweak the output type accordingly.
	output CLK_1Hz			//1Hz clock; You can tweak the output type accordingly.

);

	wire [24:0] sum100, sum1, sum1_out, sum100_out;
	wire [24:0] count100, count1;
	wire not_clk1, not_clk100, toggled1, toggled100, rst_count1, rst_count100;
	wire Hz100, Hz1, rst, condition100, condition1, not_condition1, not_condition100;

	not(rst, rst_n);
	
	Incrementor25Bit Adder100 (
		count100,
		sum100
	); // Increments the 100Hz counter
	
	Incrementor25Bit Adder1 (
		count1,
		sum1
	); // Increments the 1Hz counter
	
	DFlipFlop25 Sum1DFF (
		CLK_50MHz,
		rst,
		sum1,
		sum1_out
	); // Stores the output of the 1Hz Counter
	
	DFlipFlop25 Sum100DFF (
		CLK_50MHz,
		rst,
		sum100,
		sum100_out
	); // Stores the output of the 100Hz Counter
	
	not(not_condition1, condition1); // Condition 1 is where the counter has counted to 25M
	not(not_condition100, condition100); // Condition 1 is where the counter has counted to 250k
	// rst_count resets at LOW not HIGH; therefore if either the condition is true or rst_n is LOW it resets
	and(rst_count1, rst_n, not_condition1);
	and(rst_count100, rst_n, not_condition100);
	
	// The 2 Muxs below are the ones used to reset the counters
	mux25bit2way Mux100 (
		25'b0,
		sum100_out,
		rst_count100,
		count100
	);
	
	mux25bit2way Mux1 (
		25'b0,
		sum1_out,
		rst_count1,
		count1
	);
	
	// Checking that the 100 counter holds those bits 11 1101 0000 1001 0000 for 250,000
	// I am using this way instead of == 250,000 so everything is structural logic only
	//assign condition100 = (sum100_out == 25'd250000);
	assign condition100 = ~sum100_out[0] & ~sum100_out[1] & ~sum100_out[2] & ~sum100_out[3] &
	sum100_out[4] & ~sum100_out[5] & ~sum100_out[6] & sum100_out[7] &
	~sum100_out[8] & ~sum100_out[9] & ~sum100_out[10] & ~sum100_out[11] &
	sum100_out[12] & ~sum100_out[13] & sum100_out[14] & sum100_out[15] &
	sum100_out[16] & sum100_out[17];
	
	// Checking that the 1 counter holds those bits 1 0111 1101 0111 1000 0100 0000 for 25,000,000
	//assign condition1 = (sum1_out == 25'd25000000);
	assign condition1 = ~sum1_out[0] & ~sum1_out[1] & ~sum1_out[2] & ~sum1_out[3] &
	~sum1_out[4] & ~sum1_out[5] & sum1_out[6] & ~sum1_out[7] &
	~sum1_out[8] & ~sum1_out[9] & ~sum1_out[10] & sum1_out[11] &
	sum1_out[12] & sum1_out[13] & sum1_out[14] & ~sum1_out[15] &
	sum1_out[16] & ~sum1_out[17] & sum1_out[18] & sum1_out[19] &
	sum1_out[20] & sum1_out[21] & sum1_out[22] & ~sum1_out[23] & sum1_out[24];
	
	not(not_clk1, CLK_1Hz);
	not(not_clk100, CLK_100Hz);
	
	// When the condition is met, it toggles the clock through this multiplexer.
	mux2way Toggler1 (
		CLK_1Hz,
		not_clk1,
		condition1,
		toggled1
	);
	
	mux2way Toggler100 (
		CLK_100Hz,
		not_clk100,
		condition100,
		toggled100
	);
	
	// To avoid a loop a DFF stores the clk value
	SpecialDFlipFlopSingle Hz100Out (
		CLK_50MHz,
		rst,
		1'b0,
		toggled100,
		CLK_100Hz
	);
	
	SpecialDFlipFlopSingle Hz1Out (
		CLK_50MHz,
		rst,
		1'b0,
		toggled1,
		CLK_1Hz
	);

endmodule 