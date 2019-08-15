module MUXTest();
reg [0:1] in;
reg sel;
wire out;

MUX DUT (.in(in), .out(out), .sel(sel));

initial
begin
#0	in=0; sel=0;  //set the flip time interval as 10 unit time
#10	in=2'b00;sel=1'b1;
#10	in=2'b01;sel=1'b0;
#10	in=2'b01;sel=1'b1;
#10	in=2'b10;sel=1'b0;
#10	in=2'b10;sel=1'b1;
#10	in=2'b11;sel=1'b0;
#10	in=2'b11;sel=1'b1;
#10	in=0;sel=0;
$finish;  //exit the simulation
end
endmodule
