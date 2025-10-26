`timescale 1 ns / 100 ps 		//modify the parameters accordingly
// For function -> Reference: https://www.chipverify.com/verilog/verilog-functions
module SevenSegEncoder_tb;

	reg [7:0] LSBBinary;
	reg [7:0] MSBBinary;
	reg ModeSel;
	reg [2:0] TimeControl;
	
	wire [6:0] HexMSBH;
	wire [6:0] HexMSBL;
	wire [6:0] HexLSBH;
	wire [6:0] HexLSBL;
	
	SevenSegEncoder dut (
		LSBBinary,
		MSBBinary,
		ModeSel,
		TimeControl,
		HexMSBH,
		HexMSBL,
		HexLSBH,
		HexLSBL
	);
	
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
	
	function [3:0] BCDencoder;
        input [6:0] digit;
        begin
            case (digit)
                7'b1000000: BCDencoder = 4'd0;
                7'b1111001: BCDencoder = 4'd1;
                7'b0100100: BCDencoder = 4'd2;
                7'b0110000: BCDencoder = 4'd3;
                7'b0011001: BCDencoder = 4'd4;
                7'b0010010: BCDencoder = 4'd5;
                7'b0000010: BCDencoder = 4'd6;
                7'b1111000: BCDencoder = 4'd7;
                7'b0000000: BCDencoder = 4'd8;
                7'b0010000: BCDencoder = 4'd9;
				default: BCDencoder = 4'd0;
            endcase
        end
    endfunction
	
	integer i, j;
	reg [6:0] expected_MSBH, expected_MSBL, expected_LSBH, expected_LSBL;
	reg [8:0] MSB, LSB;
	
	initial begin
	
		TimeControl <= 3'b010;
		
		// For Mode A
		ModeSel = 1'b0;
		for (i = 0; i < 100; i = i + 1) begin
			MSBBinary = i;
			expected_MSBH = encoder(i / 10);
			expected_MSBL = encoder(i % 10);
			for (j = 0; j < 100; j = j + 1) begin
				expected_LSBH = encoder(j / 10);
				expected_LSBL = encoder(j % 10);
				LSBBinary = j;
				
				MSB = BCDencoder(HexMSBH) * 10 + BCDencoder(HexMSBL);
				LSB = BCDencoder(HexLSBH) * 10 + BCDencoder(HexLSBL);
				
				#5;
				
				if (expected_MSBH != HexMSBH | expected_MSBL != HexMSBL | expected_LSBH != HexLSBH | expected_LSBL != HexLSBL)
				begin
				
					$display("Failed (Mode A) at i = %d, j = %d", i, j);
				
				end
				
			end
		end
		
		// For Mode B
		ModeSel = 1'b1;
		for (i = 1; i < 4; i = i + 1) begin
			MSBBinary = i;
			expected_MSBH = encoder(0);
			expected_MSBL = encoder(3 - i % 10);
			for (j = 0; j < 60; j = j + 1) begin
				expected_LSBH = encoder(5 - j / 10);
				expected_LSBL = encoder(9 - j % 10);
				LSBBinary = j;
				
				MSB = BCDencoder(HexMSBH) * 10 + BCDencoder(HexMSBL);
				LSB = BCDencoder(HexLSBH) * 10 + BCDencoder(HexLSBL);
				
				#5;
				
				if (expected_MSBH != HexMSBH | expected_MSBL != HexMSBL | expected_LSBH != HexLSBH | expected_LSBL != HexLSBL)
				begin
				
					$display("Failed (Mode B) at i = %d, j = %d", i, j);
				
				end
				
			end
		end
		
		// The last case of 02:00 which is not tested above.
		MSBBinary = 0;
		LSBBinary = 0;
		expected_MSBH = encoder(0);
		expected_MSBL = encoder(0);
		expected_LSBH = encoder(0);
		expected_LSBL = encoder(0);
		MSB = BCDencoder(HexMSBH) * 10 + BCDencoder(HexMSBL);
		LSB = BCDencoder(HexLSBH) * 10 + BCDencoder(HexLSBL);
		#10;
		if (expected_MSBH != HexMSBH | expected_MSBL != HexMSBL | expected_LSBH != HexLSBH | expected_LSBL != HexLSBL)
		begin
				
			$display("Failed the last case of 02:00");
				
		end
		
		$stop;
	end

endmodule 