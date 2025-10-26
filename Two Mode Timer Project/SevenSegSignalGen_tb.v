`timescale 1 ns / 100 ps
module SevenSegSignalGen_tb;

	reg [3:0] BCD;
	wire [6:0] HEX;
	
	SevenSegSignalGen dut (
		BCD,
		HEX
	);
	
	// The function converts a BCD value into the output of a seven segment used for comparison
	function [6:0] encoder;
        input [3:0] digit;
        begin
            case (digit)
                4'b0000: encoder = 7'b1000000;
                4'b0001: encoder = 7'b1111001;
                4'b0010: encoder = 7'b0100100;
                4'b0011: encoder = 7'b0110000;
                4'b0100: encoder = 7'b0011001;
                4'b0101: encoder = 7'b0010010;
                4'b0110: encoder = 7'b0000010;
                4'b0111: encoder = 7'b1111000;
                4'b1000: encoder = 7'b0000000;
                4'b1001: encoder = 7'b0010000;
            endcase
        end
    endfunction
	
	integer i, expected;
	
	initial begin
	
		for (i = 0; i < 10; i = i + 1) begin
		
				BCD = i;
				expected = encoder(i); // Uses the encoder to determine what the expected value is.
				#5;
				
				// expected is only used for viewing. The following checks are used to determine exactly which Hex bit is wrong
				if (HEX[0] == 0 && (i == 4 || i == 1) || HEX[0] == 1 && ~(i == 4 || i == 1))
				begin
				
					$display("HEX0 is incorrect at i = %d", i);
				
				end
				
				if (HEX[1] == 0 && (i == 5 || i == 6) || HEX[1] == 1 && ~(i == 5 || i == 6))
				begin
				
					$display("HEX1 is incorrect at i = %d", i);
				
				end
				
				if (HEX[2] == 0 && (i == 2) || HEX[2] == 1 && ~(i == 2))
				begin
				
					$display("HEX2 is incorrect at i = %d", i);
				
				end
				
				if (HEX[3] == 0 && (i == 4 || i == 1 || i == 7) || HEX[3] == 1 && ~(i == 4 || i == 1 || i == 7))
				begin
				
					$display("HEX3 is incorrect at i = %d", i);
				
				end
			
				if (HEX[4] == 0 && ~(i == 0 || i == 2 || i == 6 || i == 8) || HEX[4] == 1 && (i == 0 || i == 2 || i == 6 || i == 8))
				begin
				
					$display("HEX4 is incorrect at i = %d", i);
				
				end
				
				if (HEX[5] == 0 && (i == 1 || i == 2 || i == 3 || i == 7) || HEX[5] == 1 && ~(i == 1 || i == 2 || i == 3 || i == 7))
				begin
				
					$display("HEX5 is incorrect at i = %d", i);
				
				end
				
				if (HEX[6] == 0 && (i == 0 || i == 1 || i == 7) || HEX[6] == 1 && ~(i == 0 || i == 1 || i == 7))
				begin
				
					$display("HEX6 is incorrect at i = %d", i);
			
			end
			
		end
		
		$stop;
	
	end

endmodule 