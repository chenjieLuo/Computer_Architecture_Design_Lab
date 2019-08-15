module SignExtender(BusImm, Imm16, Ctrl);
output [31:0] BusImm;
input [15:0] Imm16;
input Ctrl;

wire extBit; 

assign #1	extBit = (Ctrl ? Imm16[15]: 1'b0);//extBit will be Imm16[15] when Ctrl is 1
assign 		BusImm = {{16{extBit}}, Imm16};

endmodule
