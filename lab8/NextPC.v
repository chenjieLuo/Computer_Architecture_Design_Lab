module NextPClogic(NextPC, CurrentPC, JumpField, SignExtImm32, Branch, ALUZero, Jump);

input [31:0] CurrentPC, SignExtImm32;
input [25:0] JumpField;
input Branch, ALUZero, Jump;
output reg [31:0] NextPC; 
reg [31:0] PC;
reg [31:0] SignExt;
reg [31:0] JumpExt;

always @(*)
begin
  PC = #1 CurrentPC + 4; //CurrentPC will increment by 4 each circle
	if (Branch&&ALUZero) //If it is a branch instruction, firstly shift left immediate field by 2 bits and add it to CurrentPC
	begin
		SignExt = #2 SignExtImm32 << 2; //SignExt is the relative address after extending the upper 16 bits. 
		NextPC = #2 SignExt + PC;
	end
	else if (Jump)//If it is a Jump instruction, the immediate field will also shift left by 2 and then the [31-28] bit should be the CurrentPC first 4 bits. 
	begin 
		JumpExt = JumpField << 2; 
		NextPC = #2 JumpExt + (32'hf0000000 & PC); 
	end  
	else 
	NextPC = PC; 
end
endmodule

