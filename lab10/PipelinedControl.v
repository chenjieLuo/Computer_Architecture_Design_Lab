`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    12:23:34 03/10/2009 
// Design Name: 
// Module Name:    PipelinedControl 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
`define RTYPEOPCODE 6'b000000
`define LWOPCODE        6'b100011
`define SWOPCODE        6'b101011
`define BEQOPCODE       6'b000100
`define JOPCODE     6'b000010
`define ORIOPCODE       6'b001101
`define ADDIOPCODE  6'b001000
`define ADDIUOPCODE 6'b001001
`define ANDIOPCODE  6'b001100
`define LUIOPCODE       6'b001111
`define SLTIOPCODE  6'b001010
`define SLTIUOPCODE 6'b001011
`define XORIOPCODE  6'b001110

`define AND     4'b0000
`define OR      4'b0001
`define ADD     4'b0010
`define SLL     4'b0011
`define SRL     4'b0100
`define SUB     4'b0110
`define SLT     4'b0111
`define ADDU    4'b1000
`define SUBU    4'b1001
`define XOR     4'b1010
`define SLTU    4'b1011
`define NOR     4'b1100
`define SRA     4'b1101
`define LUI     4'b1110
`define FUNC    4'b1111


module PipelinedControl(RegDst, ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, Jump, SignExtend, ALUOp, Opcode, Bubble);
   input [5:0] Opcode;
   input Bubble;
   output RegDst;
   output ALUSrc;
   output MemToReg;
   output RegWrite;
   output MemRead;
   output MemWrite;
   output Branch;
   output Jump;
   output SignExtend;
   output [3:0] ALUOp;
	 
	reg	RegDst, ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, Jump, SignExtend;
	reg  [3:0] ALUOp;
	always @ (Opcode or Bubble) begin
		if(Bubble) begin
			//Put your code here!
			RegDst <= #2 1'b0;
			ALUSrc <= #2 1'b0;
             		MemToReg <= #2 1'b0;
             		RegWrite <= #2 0;
             		MemRead <= #2 0;
             		MemWrite <= #2 0;
             		Branch <= #2 0;
             		Jump <= #2 0;
             		SignExtend <= #2 1'b0;
			ALUOp <= #2 4'bxxxx;
		end
		else begin
		case(Opcode)
		`RTYPEOPCODE: begin
                RegDst <= #2 1;
                ALUSrc <= #2 0;
                MemToReg <= #2 0;
                RegWrite <= #2 1;
                MemRead <= #2 0;
                MemWrite <= #2 0;
                Branch <= #2 0;
                Jump <= #2 0;
                SignExtend <= #2 1'b0;
                ALUOp <= #2 `FUNC;
            end
            `LWOPCODE:  	//lw instruction
	    begin
                RegDst <= #2 0;
                ALUSrc <= #2 1;
                MemToReg <= #2 1;
                RegWrite <= #2 1;
                MemRead <= #2 1;
                MemWrite <= #2 0;
                Branch <= #2 0;
                Jump <= #2 0;
                SignExtend <= #2 1'b0;
                ALUOp <= #2 `ADD;
            end

            `SWOPCODE:		//sw instruction
	    begin
                RegDst <= #2 1'bx;
                ALUSrc <= #2 1;
                MemToReg <= #2 1'bx;
                RegWrite <= #2 0;
                MemRead <= #2 0;
                MemWrite <= #2 1;
                Branch <= #2 0;
                Jump <= #2 0;
                SignExtend <= #2 1'b0;
                ALUOp <= #2 `ADD;
            end

            `BEQOPCODE:		//beq instruction
	    begin
                RegDst <= #2 1'bx;
                ALUSrc <= #2 0;
                MemToReg <= #2 1'bx;
                RegWrite <= #2 0;
                MemRead <= #2 0;
                MemWrite <= #2 0;
                Branch <= #2 1;
                Jump <= #2 0;
                SignExtend <= #2 1'b1;
                ALUOp <= #2 `SUB;
            end

            `JOPCODE:		//jump instruction
	    begin
                RegDst <= #2 1'bx;
                ALUSrc <= #2 1'bx;
                MemToReg <= #2 1'bx;
                RegWrite <= #2 0;
                MemRead <= #2 0;
                MemWrite <= #2 0;
                Branch <= #2 1'bx;
                Jump <= #2 1;
                SignExtend <= #2 1'b1;
                ALUOp <= #2 4'bxxxx;
            end

            `ORIOPCODE: 	//ori instruction
	    begin
                RegDst <= #2 0;
                ALUSrc <= #2 1;
                MemToReg <= #2 0;
                RegWrite <= #2 1;
                MemRead <= #2 0;
                MemWrite <= #2 0;
                Branch <= #2 0;
                Jump <= #2 0;
                SignExtend <= #2 1'b1;
                ALUOp <= #2 `OR;
            end

            `ADDIOPCODE:	//addi instruction
	    begin
                RegDst <= #2 0;
                ALUSrc <= #2 1;
                MemToReg <= #2 0;
                RegWrite <= #2 1;
                MemRead <= #2 0;
                MemWrite <= #2 0;
                Branch <= #2 0;
                Jump <= #2 0;
                SignExtend <= #2 1'b0;
                ALUOp <= #2 `ADD;
            end

            `ADDIUOPCODE:	//addiu instruction
	    begin
                RegDst <= #2 0;
                ALUSrc <= #2 1;
                MemToReg <= #2 0;
                RegWrite <= #2 1;
                MemRead <= #2 0;
                MemWrite <= #2 0;
                Branch <= #2 0;
                Jump <= #2 0;
                SignExtend <= #2 1'b1;
                ALUOp <= #2 `ADDU;
            end

            `ANDIOPCODE:	//andi instruction 
	    begin
                RegDst <= #2 0;
                ALUSrc <= #2 1;
                MemToReg <= #2 0;
                RegWrite <= #2 1;
                MemRead <= #2 0;
                MemWrite <= #2 0;
                Branch <= #2 0;
                Jump <= #2 0;
                SignExtend <= #2 1'b1;
                ALUOp <= #2 `AND;
            end

            `LUIOPCODE: 	//lui instruction
	    begin
                RegDst <= #2 0;
                ALUSrc <= #2 1;
                MemToReg <= #2 0;
                RegWrite <= #2 1;
                MemRead <= #2 0;
                MemWrite <= #2 0;
                Branch <= #2 0;
                Jump <= #2 0;
                SignExtend <= #2 1'b1;
                ALUOp <= #2 `LUI;
            end

            `SLTIOPCODE:	//slti instruction
	    begin
                RegDst <= #2 0;
                ALUSrc <= #2 1;
                MemToReg <= #2 0;
                RegWrite <= #2 1;
                MemRead <= #2 0;
                MemWrite <= #2 0;
                Branch <= #2 0;
                Jump <= #2 0;
                SignExtend <= #2 1'b0;
                ALUOp <= #2 `SLT;
            end

            `SLTIUOPCODE:	//sltiu instruction
	    begin
                RegDst <= #2 0;
                ALUSrc <= #2 1;
                MemToReg <= #2 0;
                RegWrite <= #2 1;
                MemRead <= #2 0;
                MemWrite <= #2 0;
                Branch <= #2 0;
                Jump <= #2 0;
                SignExtend <= #2 1'b0;
                ALUOp <= #2 `SLTU;
            end

            `XORIOPCODE:	//xor instruction
	    begin 
                RegDst <= #2 0;
                ALUSrc <= #2 1;
                MemToReg <= #2 0;
                RegWrite <= #2 1;
                MemRead <= #2 0;
                MemWrite <= #2 0;
                Branch <= #2 0;
                Jump <= #2 0;
                SignExtend <= #2 1'b1;
                ALUOp <= #2 `XOR;
            end

            default: begin	//default instruction
                RegDst <= #2 1'bx;
                ALUSrc <= #2 1'bx;
                MemToReg <= #2 1'bx;
                RegWrite <= #2 1'bx;
                MemRead <= #2 1'bx;
                MemWrite <= #2 1'bx;
                Branch <= #2 1'bx;
                Jump <= #2 1'bx;
                SignExtend <= #2 1'bx;
                ALUOp <= #2 4'bxxxx;

            end
        endcase
	end
    end
endmodule
