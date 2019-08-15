module decoder(in,out,CLK); //name the decoder, which contains two inputs: in and CLK and one output: out
input [0:1] in;  	//define the two-bit input
input CLK; 		//define the CLK as another input
output reg [0:3] out;	//define the 4-bit output
always @(posedge CLK)
begin
	if (in==2'b00)	//If the input is 2'b00, the output will be 4'b0001
	out=4'b0001;	
	if (in==2'b01) 	//If the input is 2'b01, the output will be 4'b0010
	out=4'b0010;	
	if (in==2'b10)	//If the input is 2'b10, the output will be 4'b0100
	out=4'b0100;
	if (in==2'b11)	//If the input is 2.b11, the output will be 4'b1000	
	out=4'b1000;	
end
endmodule
