
module SignExtenderTest();
//reg [31:0] BusImm; 
reg [15:0]  Imm16;
reg Ctrl; 
wire [31:0] BusImm;

SignExtender DUT (.BusImm(BusImm), .Imm16(Imm16), .Ctrl(Ctrl));

initial
begin
#0	Ctrl = 1;  //set the flip time as 10 unit time
	Imm16 = 16'b0000000000000010;
#10  	Imm16 = 16'b1111111111111100;
	Ctrl = 1;
#10
$finish;
end
endmodule
