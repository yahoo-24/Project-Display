`timescale 1 ns/ 100 ps
module FourBitCarryLookAheadAdder_tb;

	wire [3:0] sum;
	wire cout;
	reg [3:0] in1, in2;
	reg cin;
	
	FourBitCarryLookAheadAdder dut (
		in1,
		in2,
		cin,
		sum,
		cout
	);
	
	integer i, expected, actual;
	
	initial begin
	
		cin = 1'b0; // Set carry to 0
	
		for (i = 0; i < 256; i = i + 1) // Testing all possible values
		begin
			{in1, in2} = i;
			#0.1; // Give the inputs time to update
			
			expected = in1 + in2; // Expected output
			actual = {cout, sum}; // Actual output
			#10; // Delay gives time for expected and actual to update
			
			if (expected != actual)
			begin
			
				$display("incorrect value at %d + %d", in1, in2); // Check if output is correct
			
			end
		end
		
		// Do the same procedure above but with the carry set to 1
		cin = 1'b1;
	
		for (i = 0; i < 256; i = i + 1)
		begin
			{in1, in2} = i;
			#0.1;
			expected = in1 + in2 + cin;
			actual = {cout, sum};
			#10;
			if (expected != actual)
			begin
			
				$display("incorrect value at %d + %d", in1, in2);
			
			end
		end
		
		$stop;
	
	end
	
endmodule
