module MUX(in,sel,out);
input [0:1] in;  //set the input as 2-bit
input sel;      
output out;
wire a1, a2, a3;
not inv1(a1,sel); //inverse function: a1 = sel'
and and1(a2, a1, in[0]);//and function: a2 = a1 * in[0]
and and2(a3, in[1], sel);//The same as above
or or1(out, a2, a3); //or function: out = a2 + a3
endmodule
