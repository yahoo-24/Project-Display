module SevenSegSignalGen(
	input [3:0] BCD,
	output [6:0] HEX
);
	
	wire NotBCD0, NotBCD1, NotBCD2, NotBCD3, NotHEX4;
	wire T1H0, T2H0, T1H1, T2H1, T1H3, T2H3, T3H3, T1H4, T2H4, T3H4, T4H4;
	wire T1H5, T2H5, T3H5, T4H5, T1H6, T2H6, T3H6;
	
	// The logic here is based on the truth table from Lab 4.2
	
	/*
	assign HEX[0] = (BCD[0] & ~BCD[1] & ~BCD[2] & ~BCD[3] | ~BCD[3] & BCD[2] & ~BCD[1] & ~BCD[0]);
	assign HEX[1] = (BCD[0] & ~BCD[1] & BCD[2] & ~BCD[3] | ~BCD[3] & BCD[2] & BCD[1] & ~BCD[0]);
	assign HEX[2] = (~BCD[0] & BCD[1] & ~BCD[2] & ~BCD[3]);
	assign HEX[3] = (BCD[0] & ~BCD[1] & ~BCD[2] & ~BCD[3] | ~BCD[3] & BCD[2] & ~BCD[1] & ~BCD[0] | ~BCD[3] & BCD[2] & BCD[1] & BCD[0]);
	assign HEX[4] = ~(~BCD[0] & ~BCD[1] & ~BCD[2] & ~BCD[3] | ~BCD[3] & ~BCD[2] & BCD[1] & ~BCD[0] | ~BCD[3] & BCD[2] & BCD[1] & ~BCD[0] | ~BCD[0] & ~BCD[1] & ~BCD[2] & BCD[3]);
	assign HEX[5] = (BCD[0] & ~BCD[1] & ~BCD[2] & ~BCD[3] | ~BCD[3] & ~BCD[2] & BCD[1] & ~BCD[0] | ~BCD[3] & ~BCD[2] & BCD[1] & BCD[0] | BCD[0] & BCD[1] & BCD[2] & ~BCD[3]);
	assign HEX[6] = (~BCD[0] & ~BCD[1] & ~BCD[2] & ~BCD[3] | ~BCD[3] & ~BCD[2] & ~BCD[1] & BCD[0] | ~BCD[3] & BCD[2] & BCD[1] & BCD[0]);
	*/

	not(NotBCD0, BCD[0]);
	not(NotBCD1, BCD[1]);
	not(NotBCD2, BCD[2]);
	not(NotBCD3, BCD[3]);
	
	and(T1H0, BCD[0], NotBCD1, NotBCD2, NotBCD3);
	and(T2H0, NotBCD0, NotBCD1, BCD[2], NotBCD3);
	or(HEX[0], T2H0, T1H0);
	
	and(T1H1, BCD[0], NotBCD1, BCD[2], NotBCD3);
	and(T2H1, NotBCD0, BCD[1], BCD[2], NotBCD3);
	or(HEX[1], T2H1, T1H1);
	
	and(HEX[2], NotBCD0, BCD[1], NotBCD2, NotBCD3);
	
	and(T1H3, BCD[0], NotBCD1, NotBCD2, NotBCD3);
	and(T2H3, NotBCD0, NotBCD1, BCD[2], NotBCD3);
	and(T3H3, BCD[0], BCD[1], BCD[2], NotBCD3);
	or(HEX[3], T3H3, T2H3, T1H3);
	
	and(T1H4, NotBCD0, NotBCD1, NotBCD2, NotBCD3);
	and(T2H4, NotBCD0, BCD[1], NotBCD2, NotBCD3);
	and(T3H4, NotBCD0, BCD[1], BCD[2], NotBCD3);
	and(T4H4, NotBCD0, NotBCD1, NotBCD2, BCD[3]);
	or(NotHEX4, T4H4, T3H4, T2H4, T1H4);
	not(HEX[4], NotHEX4);
	
	and(T1H5, BCD[0], NotBCD1, NotBCD2, NotBCD3);
	and(T2H5, NotBCD0, BCD[1], NotBCD2, NotBCD3);
	and(T3H5, BCD[0], BCD[1], NotBCD2, NotBCD3);
	and(T4H5, BCD[0], BCD[1], BCD[2], NotBCD3);
	or(HEX[5], T4H5, T3H5, T2H5, T1H5);
	
	and(T1H6, NotBCD0, NotBCD1, NotBCD2, NotBCD3);
	and(T2H6, BCD[0], NotBCD1, NotBCD2, NotBCD3);
	and(T3H6, BCD[0], BCD[1], BCD[2], NotBCD3);
	or(HEX[6], T3H6, T2H6, T1H6);

endmodule 