`timescale 1 ns / 100 ps
module mux8bit2way_tb;

	reg [7:0] in1;
	reg [7:0] in2;
	reg sel;
	wire [7:0] out;
	
	mux8bit2way dut (
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
	
		in1 <= 8'b10001000;
		in2 <= 8'b11110000;
		sel <= 1'b0;
		#5;
		
		in1 <= 8'b00000001;
		in2 <= 8'b11110000;
		sel <= 1'b0;
		#5;
		
		in1 <= 8'b10000000;
		in2 <= 8'b11110000;
		sel <= 1'b0;
		#5;
		
		in1 <= 8'b10001000;
		in2 <= 8'b11110000;
		sel <= 1'b1;
		#5;
		
		in1 <= 8'b00000001;
		in2 <= 8'b11001100;
		sel <= 1'b1;
		#5;
		
		in1 <= 8'b00000001;
		in2 <= 8'b11101100;
		sel <= 1'b1;
		#5;
		
		$stop;
	
	end

endmodule 