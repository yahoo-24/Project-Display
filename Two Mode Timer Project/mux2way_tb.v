`timescale 1 ns / 100 ps
module mux2way_tb;

	reg in1, in2, sel;
	wire out;
	
	mux2way dut (
		in1,
		in2,
		sel,
		out
	);
	
	always begin
	
		#1; // Checking every 1 ns to see if the output follows in1 during sel = 0 and in2 during sel = 1
		if (sel & (out != in2) | ~sel & (out != in1))
		begin
		
			$display("Incorrect Output!");
		
		end
	
	end
	
	initial begin
		
		in1 <= 1'b0;
		in2 <= 1'b0;
		sel <= 1'b0;
		#5;
		
		in1 <= 1'b1;
		in2 <= 1'b0;
		sel <= 1'b0;
		#5;
		
		in1 <= 1'b0;
		in2 <= 1'b1;
		sel <= 1'b0;
		#5;
		
		in1 <= 1'b1;
		in2 <= 1'b1;
		sel <= 1'b0;
		#5;
		
		in1 <= 1'b0;
		in2 <= 1'b0;
		sel <= 1'b1;
		#5;
		
		in1 <= 1'b1;
		in2 <= 1'b0;
		sel <= 1'b1;
		#5;
		
		in1 <= 1'b0;
		in2 <= 1'b1;
		sel <= 1'b1;
		#5;
		
		in1 <= 1'b1;
		in2 <= 1'b1;
		sel <= 1'b1;
		#5;
		
		$stop;
	
	end

endmodule 