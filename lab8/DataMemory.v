`timescale 1ns / 1ps

module DataMemory(ReadData, Address, WriteData, MemoryRead, MemoryWrite, Clock);
input [31:0] Address, WriteData;
input MemoryRead, MemoryWrite, Clock;
output reg [31:0] ReadData;
reg [31:0] tmp;
reg [31:0] Mem [63:0]; //Require size of 64 words 

always @ (negedge Clock) //At the negative edge  of the clock, check if we need to write data into memory
begin 
	if (MemoryWrite == 1 && MemoryRead == 0)
	begin 
		 Mem[(Address>>2)] <= #20 WriteData;
	$display("The data written into Memory is %d\n", Mem[Address]); 
 	end
end
always @ (posedge Clock) //At the positive edge of the clock, check if we need to read data into memory
begin
        if (MemoryWrite == 0 && MemoryRead == 1)
	begin 
	
	ReadData <= #20 Mem[(Address>>2)]; 
	$display("The data read from Memory is %d\n", Mem[Address]); 

	end
end
endmodule
		

