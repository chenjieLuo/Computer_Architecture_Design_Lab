module ALUControl(ALUCtrl, ALUop, FuncCode);

input [3:0] ALUop;
input [5:0] FuncCode;
output reg [3:0] ALUCtrl;
always @(*)
begin
if (ALUop != 4'b1111) //If it is a I-type instruction, ALUCtrl = ALUop
	ALUCtrl = #2 ALUop;
else if (ALUop == 4'b1111) //It is a R-type instruction, check the FuncCode
	begin
	case(FuncCode)
	6'b000000: ALUCtrl = #2 4'b0011; //The instruction is sll
	6'b000010: ALUCtrl = #2 4'b0100; //The instruction is srl
	6'b100000: ALUCtrl = #2 4'b0010; //The instruction is add
	6'b100010: ALUCtrl = #2 4'b0110; //The instruction is sub	
	6'b100100: ALUCtrl = #2 4'b0000; //The instruction is AND
	6'b100101: ALUCtrl = #2 4'b0001; //The instruction is OR
	6'b101010: ALUCtrl = #2 4'b0111; //The instruction is slt
	6'b111000: ALUCtrl = #2 4'b0101; //The instruction is MULA
	6'b101011: ALUCtrl = #2 4'b1011; //The instruction is SLTU
	6'b100111: ALUCtrl = #2 4'b1100; //The instruction is NOR
	6'b100110: ALUCtrl = #2 4'b1010; //THe instruction is XOR
	6'b100011: ALUCtrl = #2 4'b1001; //The instruction is SUBU
	6'b100001: ALUCtrl = #2 4'b1000; //The instruction is ADDU
	6'b000011: ALUCtrl = #2 4'b1101; //The instruction is SRA
	endcase
	end
end
endmodule
