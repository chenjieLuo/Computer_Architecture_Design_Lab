module RegisterFile(BusA, BusB, BusW, RW, RegWr,Clk, RA, RB);
   //  REG INPUT AND OUTPUT
    output  [31:0] BusA;
    output  [31:0] BusB;
    input [31:0] BusW;
    input [4:0] RW;
    input RegWr;
    input Clk;
    input [4:0] RA, RB;
    reg [31:0] registers [31:0];
	 
	 initial begin
	 // initial the register
	 registers[0] = 0;
         registers[1] = 1;
	 registers[2] = 32'h2;
	 registers[3] = 32'h3;
 	 registers[4] = 32'h4;
	 registers[5] = 32'h5;
	 registers[6] = 32'h6;
	 registers[7] = 32'h7;
	 registers[8] = 32'h8;
	 registers[9] = 32'h9;
	 registers[10] = 32'h10;
	 registers[11] = 32'h11;
	 registers[12] = 32'h12;
	 registers[13] = 32'h13;
	 registers[14] = 32'h14;
	 registers[15] = 32'h15;
	 registers[16] = 0;
	 registers[17] = 0;
	 registers[18] = 0;
	 registers[19] = 0;
	 registers[20] = 0;
	 registers[21] = 0;	
         registers[22] = 0;
	 registers[23] = 0;
	 registers[24] = 0;
	 registers[25] = 0;	 
	 registers[26] = 0;
	 registers[27] = 0;
	 registers[28] = 0;
	 registers[29] = 0;
	 registers[30] = 0;
	 registers[31] = 0; 
end
	 assign #2 BusA = registers[RA];        //busA was set by regisrerRA
    	 assign #2 BusB = registers[RB];        //BusB was set by registerRB
always @ (negedge Clk) begin           // negative edge begin
    if(RegWr)                    
	begin
		if (RW!=0)               
                registers[RW] <= BusW;      // when RW!=0 register which is set by RW become BusW
		end
	end
endmodule
