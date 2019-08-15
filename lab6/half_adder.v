module half_adder(in,Sum,Cout);
input [0:1] in;  //in is defined as 2-bit input
output Sum, Cout;//out is defined as Sum and Cout
wire a1, a2, a3;
nand g1(a1, in[0], in[1]);//nand gate function works as: a1 = (in[0] * in[1])'
nand g2(a2, in[1], a1);   //gate works the same way as above
nand g3(a3, in[0], a1);
nand g4(Sum, a2, a3);//Sum represents the sum and  Cout represent the Carry
nand g5(Cout, a1, a1);
endmodule
