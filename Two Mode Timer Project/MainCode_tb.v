`timescale 1 ns / 100 ps 		//modify the parameters accordingly

module MainCode_tb;

	reg CLK_50MHz;
	reg CLK_100Hz;
	reg CLK_1Hz;
	reg rst_n;
	reg StartStop;
	reg ModeSel;
	reg [2:0] TimeControl;
	
	wire [6:0] HexMSBH;
	wire [6:0] HexMSBL;
	wire [6:0] HexLSBH;
	wire [6:0] HexLSBL;
	wire DOT;
	wire StopLED;
	wire FlashingLED;
	
	MainCode dut (
		CLK_50MHz,
		CLK_100Hz,
		CLK_1Hz,
		rst_n,
		StartStop,
		ModeSel,
		TimeControl,
		HexMSBH,
		HexMSBL,
		HexLSBH,
		HexLSBL,
		DOT,
		StopLED,
		FlashingLED
	);
	
	function [3:0] encoder;
        input [6:0] digit;
        begin
            case (digit)
                7'b1000000: encoder = 4'd0;
                7'b1111001: encoder = 4'd1;
                7'b0100100: encoder = 4'd2;
                7'b0110000: encoder = 4'd3;
                7'b0011001: encoder = 4'd4;
                7'b0010010: encoder = 4'd5;
                7'b0000010: encoder = 4'd6;
                7'b1111000: encoder = 4'd7;
                7'b0000000: encoder = 4'd8;
                7'b0010000: encoder = 4'd9;
				default: encoder = 4'd0;
            endcase
        end
    endfunction
	
	reg [7:0] MSB, LSB;
	
	always begin
		#10;
		//CLK_50MHz <= ~CLK_50MHz;
		CLK_100Hz <= ~CLK_100Hz;
	end
	
	always begin
	
		#1000;
		CLK_1Hz <= ~CLK_1Hz;
	
	end
	
	always @(posedge CLK_100Hz or negedge CLK_100Hz) begin
	
		MSB <= encoder(HexMSBH) * 10 + encoder(HexMSBL);
		LSB <= encoder(HexLSBH) * 10 + encoder(HexLSBL);
		
	end
	
	initial begin
	
		// Initialise values
		TimeControl <= 3'b001; // 2 minutes
		CLK_50MHz <= 1'b0;
		CLK_100Hz <= 1'b0;
		CLK_1Hz <= 1'b0;
		rst_n <= 1'b0; // Reset on
		StartStop <= 1'b1;
		ModeSel <= 1'b0; // Mode A
		#50;
		
		rst_n <= 1'b1;
		StartStop <= 1'b0;
		// Start and Remove Reset
		#8000;
		// Remove Start after long press
		StartStop <= 1'b1;
		#3;
		// Change Mode to see if it automatically resets
		ModeSel <= 1'b1;
		#4000;
		
		// Press Start
		StartStop <= 1'b0;
		#2;
		// Remove Start after short press
		StartStop <= 1'b1;
		#10000;
		// Press Stop
		StartStop <= 1'b0;
		#4000;
		// Remove long press
		StartStop <= 1'b1;
		// The above short and long presses test the edge holder and detector
		#4000;
		
		// Resume timer after 4 idle cycles
		StartStop <= 1'b0;
		#1000;
		StartStop <= 1'b1; // Remove the press on the Start button
		#240000; // Count till end
		
		rst_n <= 1'b0; // Reset
		TimeControl <= 3'b010; // 3 minutes
		ModeSel <= 1'b1; // Still Mode B
		#50;
		
		rst_n <= 1'b1; // Remove Reset
		StartStop <= 1'b0; // Start
		
		#10;
		StartStop <= 1'b1; // Remove press
		#4000; // 2 cycles
		
		TimeControl <= 3'b000; // Switch to 1 minute and see if it has reset
		#8000;
		
		StartStop <= 1'b0; // If it has reset as intended then this should start the timer.
		#100;
		
		StartStop <= 1'b1; // Remove press.
		#130000; // Continue till end
		
		ModeSel <= 1'b0; // Back to Mode A
		#100;
		
		StartStop <= 1'b0; // Start Mode A
		#10;
		
		StartStop <= 1'b1; // Remove press.
		#210000; // Continue till end
	
		$stop;
	end

endmodule 