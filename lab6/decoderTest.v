module decoderTest();
reg [0:1] in;
reg CLK;
wire [0:3] out;

decoder DUT (.in(in), .out(out),.CLK(CLK));

initial
begin
#0	in=0;CLK=1;	//set flip time interval as 10 unit time
#10	CLK=0;
#10	in=2'b01;CLK=1;
#10	CLK=0;
#10	in=2'b10;CLK=1;
#10	CLK=0;
#10	in=2'b11;CLK=1;
#10	CLK=0;
#10	in=0;
$finish;//exit the simulation
end
endmodule
