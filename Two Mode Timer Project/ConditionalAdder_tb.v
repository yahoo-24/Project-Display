`timescale 1 ns / 100 ps
module ConditionalAdder_tb;

	reg [3:0] in;
	wire [3:0] out;
	
	ConditionalAdder dut (
		in,
		out
	);
	
	integer i;
	reg [3:0] temp;
	
	initial begin
	
		for (i = 0; i < 16; i = i + 1) begin // Testing all possible inputs
			
			in = i;
			temp = i + 3; // Used in the condition checking: This is what is expected when in > 4
			#5;
			
			if (i <= 4 & out != in | i > 4 & out != temp) begin
				$display("Failure at i = %d", i); // Check that it adds 3 when greater than 4 otherwise in = out
			end
		
		end
		
		$stop;
		
	end

endmodule 