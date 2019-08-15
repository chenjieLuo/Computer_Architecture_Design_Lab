module half_adderTest();
reg [0:1] in;
wire Sum, Cout;

half_adder DUT (.in(in), .Sum(Sum), .Cout(Cout));

initial
begin
#0	in=0;  //set the flip time as 10 unit time
#10	in=2'b01;
#10	in=2'b10;
#10 	in=2'b11;
#10	in=0; //set the input back to 0
$finish;	//exit the simulation
end
endmodule
