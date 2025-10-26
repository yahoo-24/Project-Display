module ConditionalAdder (
	input [3:0] in,
	output [3:0] out
);

	wire check_2lsb_bits, check_3lsb_bits, is_greater_4, unused;
	wire [3:0] sum;

	// The 3 gates check if MSB (bit 3) is high or bit 2 and either one of bit 1 and 0 is High.
	// Bit 3 == High -> our number is at least 8.
	// Bit 2 == High -> our number is 4 so we need either Bit 1 OR 0 set to exceed 4.
	// This is to check that the number is greater than 4.
	or(check_2lsb_bits, in[0], in[1]);
	and(check_3lsb_bits, check_2lsb_bits, in[2]);
	or(is_greater_4, in[3], check_3lsb_bits);
	
	// Adding by 3.
	FourBitCarryLookAheadAdder Adder (
		4'b0011,
		in,
		1'b0,
		sum,
		unused
	);
	
	// There is no 4-bit mux so 4 x 1-bit mux are used.
	mux2way Bit0Mux (
		in[0],
		sum[0],
		is_greater_4,
		out[0]
	);
	
	mux2way Bit1Mux (
		in[1],
		sum[1],
		is_greater_4,
		out[1]
	);
	
	mux2way Bit2Mux (
		in[2],
		sum[2],
		is_greater_4,
		out[2]
	);
	
	mux2way Bit3Mux (
		in[3],
		sum[3],
		is_greater_4,
		out[3]
	);

	/*
	The functionality that we are trying to acheive with structured logic:
	
	always @(in) begin
	
		if (in > 4) begin
			out = in + 3;
		end
		else begin
			out = in;
		end
	
	end
	*/

endmodule 