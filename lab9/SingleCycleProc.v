`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    12:16:03 03/10/2009 
// Design Name: 
// Module Name:    SingleCycleProc 
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
module SingleCycleProc(CLK, Reset_L, startPC, dMemOut);
   input CLK;
   input Reset_L;
   input [31:0]	startPC;
   output [31:0]	dMemOut;
	
	//PC Logic
	wire	[31:0]	nextPC;
	reg	[31:0]	currentPC;
	
	//Instruction Decode
	wire	[31:0]	currentInstruction;
	wire	[5:0]		opcode;
	wire	[4:0]		rs,rt,rd;
	wire	[15:0]	imm16;
	wire	[4:0]		shamt;
	wire	[5:0]		func;
	wire	[25:0]	jumpImmediate;
	
	assign {opcode, rs, rt, rd, shamt, func} = currentInstruction;
	assign imm16 = currentInstruction[15:0];
	assign jumpImmediate = currentInstruction[25:0];
	
	//Register wires
	wire	[31:0]	busA, busB, busW;
	wire	[4:0]		rw;
	
	//Control Logic Wires
	wire	RegDst, ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, Jump, SignExtend;
	wire	[3:0]		ALUOp;
	wire	[3:0]		ALUCtrl;
	wire	UseShiftField;
	
	//ALU Wires
	wire	[31:0]	ALUAIn, signExtImm32, ALUImmRegChoice, ALUBIn;
	wire	[31:0]	ALUResult;
	wire	ALUZero;
	
	//Data Memory Wires
	wire	[31:0]	dMemOut;

	//Instruction Memory
	InstructionMemory instrMem(currentInstruction, currentPC);	
	
	//PC Logic
	NextPClogic next(nextPC, currentPC, jumpImmediate, signExtImm32, Branch, ALUZero, Jump);
	always @ (negedge CLK, negedge Reset_L) begin
		if(~Reset_L)
			currentPC = startPC;
		else
			currentPC = nextPC;
	end
	
	//Control
	SingleCycleControl control(RegDst, ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, Jump, SignExtend, ALUOp, opcode);
	
	//Register file
	assign rw = (RegDst ? rd : rt);                                               // assign the rw depends on the registers destination 
    RegisterFile registers(busA, busB, busW, rs, rt, rw, RegWrite, CLK);          //instantiate the RegisterFile
	
	//Sign Extender
	/*instantiate the sign extender*/
	SignExtender imm16to32(signExtImm32, imm16, SignExtend);
	//ALU
	ALUControl ALUCont(ALUCtrl, ALUOp, func);
	assign #2 UseShiftField = ((ALUOp == 4'b1111) && ((func == 6'b000000) || (func == 6'b000010) || (func == 6'b000011)));
	assign #2 ALUImmRegChoice = ALUSrc ? signExtImm32 : busB;
	assign #2 ALUBIn = UseShiftField ? {27'b0, shamt} : ALUImmRegChoice;
	assign #2 ALUAIn = UseShiftField ? busB : busA;
	ALU mainALU(ALUResult, ALUZero, ALUAIn, ALUBIn, ALUCtrl);
	
	//Data Memory
	DataMemory data(dMemOut, ALUResult, busB, MemRead, MemWrite, CLK);           //instantiate the DataMemory  
	assign  busW = (MemToReg ? dMemOut : ALUResult);                             //assign the busW depends on the MemToReg
	
endmodule

`timescale 1ns / 1ps
`define AND 4'b0000
`define OR 4'b0001
`define ADD 4'b0010
`define SLL 4'b0011
`define SRL 4'b0100
`define SUB 4'b0110
`define SLT 4'b0111
`define ADDU 4'b1000
`define SUBU 4'b1001
`define XOR 4'b1010
`define SLTU 4'b1011
`define NOR 4'b1100
`define SRA 4'b1101
`define LUI 4'b1110

module ALU(BusW, Zero, BusA, BusB, ALUCtrl);
// set input and out put
output reg [31:0] BusW;
output Zero;
input [31:0] BusA;
input [31:0] BusB;
input [3:0] ALUCtrl;
  always @(ALUCtrl or BusA or BusB) begin 	
  case (ALUCtrl)                          	
  `AND:begin 
  BusW <= #20 BusA&BusB;                  	//Bitwise AND comparison
  end
  `OR:begin
  BusW <= #20 BusA|BusB;                 	//Bitwise OR comparison
  end 
  `ADD:begin 
  BusW <= #20 $signed(BusA)+$signed(BusB); 	//Add value from BusA with BusB
  end
  `SLL:begin
  BusW <= #20 BusA<<BusB;                 	//shift left for certain bits saved in BusB
  end
  `SRL:begin
  BusW <= #20 BusA>>BusB;                	//shift right for certain bits saved BusB
  end
  `SUB:begin
  BusW <= #20 $signed(BusA)-$signed(BusB); 	//Substraction
  end
  `SLT:begin					//Set less than instruction, if BusA is smaller than BusB, output will be 1
  if({~BusA[31],BusA[30:0]}<{~BusB[31],BusB[30:0]})  
  BusW <= #20 32'b1;
  else 
  BusW <= #20 32'b0; 
  end
  `ADDU:begin 
  BusW <= #20 BusA+BusB;               		//Addition unsigned
  end
  `SUBU:begin
  BusW <= #20 BusA-BusB;               		//subu
  end
  `XOR:begin
  BusW <= #20 BusA^BusB;                 	//xor
  end
  `SLTU:begin
  if(BusA < BusB)                       	//sltu
  BusW <= #20 1;
  else
  BusW <= #20 0;
  end
  `NOR:begin
  BusW <= #20 ~(BusA|BusB);              	//nor
  end
  `SRA:begin
  BusW <= #20 $signed(BusA)>>>BusB;       	//sra
  end
  `LUI:begin
  BusW <= #20 BusB<<16;                   	//lui
  end
  endcase
  end
  assign #1 Zero =(BusW==32'b0)?1:0;        //to set the right value of zero
endmodule

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

module DataMemory(ReadData, Address, WriteData, MemoryRead, MemoryWrite, Clock);
input [31:0] Address, WriteData;
input MemoryRead, MemoryWrite, Clock;
output [31:0] ReadData;
reg[31:0] ReadData;
reg[31:0] register[63:0] ;  // reserve memory to hold data

always @(negedge Clock)begin
 if(MemoryWrite && ~(MemoryRead)) begin
   register[(Address>>2)] <= #20 WriteData; // write the data to the memory
 end
end
always @(posedge Clock) begin
 if(MemoryRead && ~(MemoryWrite)) begin
    ReadData <= #20 register[(Address>>2)]; // read the data from the memory 
 end 
end

endmodule

module InstructionMemory(Data, Address);
	parameter T_rd = 20;
	parameter MemSize = 40;
	
	output [31:0] Data;
	input [31:0] Address;
	reg [31:0] Data;
	
	/*
	 * ECEN 350 Processor Test Functions
	 * Texas A&M University
	 */

	always @ (Address) begin
		case(Address)
		/*
		 * Test Program 1:
		 * Sums $a0 words starting at $a1.  Stores the sum at the end of the array
		 * Tests add, addi, lw, sw, beq
		 */

		/*
		main:	
				li $t0, 50					# Initialize the array to (50, 40, 30)
				sw $t0, 0($0)				# Store first value
				li $t0, 40          	
				sw $t0, 4($0)				# Store Second Value
				li $t0, 30          	
				sw $t0, 8($0)				# Store Third Value
				li $a0, 0					# address of array
				li $a1, 3					# 3 values to sum
		TestProg1:                  	
				add $t0, $0, $0			# This is the sum
				add $t1, $0, $a0			# This is our array pointer
				add $t2, $0, $0			# This is our index counter
		P1Loop:	beq $t2, $a1, P1Done	# Our loop
				lw	$t3, 0($t1)				# Load Array[i]
				add $t0, $t0, $t3			# Add it into the sum
				add $t1, $t1, 4			# Next address
				add $t2, $t2, 1			# Next index
				j P1Loop						# Jump to loop
		P1Done:	sw $t0, 0($t1)			# Store the sum at end of array
				lw $t0, 12($0)      		# Load Final Value
				nop							# Complete
		*/
			32'h00: Data = 32'h34080032;
			32'h04: Data = 32'hac080000;
			32'h08: Data = 32'h34080028;
			32'h0C: Data = 32'hac080004;
			32'h10: Data = 32'h3408001e;
			32'h14: Data = 32'hac080008;
			32'h18: Data = 32'h34040000;
			32'h1C: Data = 32'h34050003;
			32'h20: Data = 32'h00004020;
			32'h24: Data = 32'h00044820;
			32'h28: Data = 32'h00005020;
			32'h2C: Data = 32'h11450005;
			32'h30: Data = 32'h8d2b0000;
			32'h34: Data = 32'h010b4020;
			32'h38: Data = 32'h21290004;
			32'h3C: Data = 32'h214a0001;
			32'h40: Data = 32'h0800000b;
			32'h44: Data = 32'had280000;
			32'h48: Data = 32'h8c08000c;
			32'h4C: Data = 32'h00000000;

		/*
		 * Test Program 2:
		 * Does some arithmetic computations and stores result in memory
		 */

		/*
		main2:
				li	$a0, 32					# Address of memory to store result
		TestProg2:
				add $2, $0, 1				# $2 = 1
				sub	$3, $0, $2			# $3 = -1
				slt	$5, $3, $0			# $5 = 1
				add	$6, $2, $5      	# $6 = 2
				or	$7, $5, $6				# $7 = 3
				sub	$8, $5, $7			# $8 = -2
				and	$9, $8, $7			# $9 = 2
				sw	$9, 0($a0)				# Store $9 in DMem[8]
				lw  $9, 32($0)      		# Load Final Value
				nop							# Complete
		*/
			32'h60: Data = 32'h34040020;
			32'h64: Data = 32'h20020001;
			32'h68: Data = 32'h00021822;
			32'h6C: Data = 32'h0060282a;
			32'h70: Data = 32'h00453020;
			32'h74: Data = 32'h00a63825;
			32'h78: Data = 32'h00a74022;
			32'h7C: Data = 32'h01074824;
			32'h80: Data = 32'hac890000;
			32'h84: Data = 32'h8c090020;
			32'h88: Data = 32'h00000000;
		
		/*
		 * Test Program 3
		 * Test Immediate Function
		 */
		
		/*
				TestProg3:
				li $a0, 0xfeedbeef		# $a0 = 0xfeedbeef
				sw $a0, 36($0)				# Store $a0 in DMem[9]
				addi $a1, $a0, -2656		# $a1 = 0xfeedb48f
				sw $a1, 40($0)				# Store $a1 in DMem[10]
				addiu $a1, $a0, -2656	# $a1 = 0xfeeeb48f
				sw $a1, 44($0)				# Store $a1 in DMem[11]
				andi $a1, $a0, 0xf5a0	# $a1 = 0xb4a0
				sw $a1, 48($0)				# Store $a1 in DMem[12]
				sll $a1, $a0, 5			# $a1 = 0xddb7dde0
				sw $a1, 52($0)				# Store $a1 in DMem[13]
				srl $a1, $a0, 5			# $a1 = 0x07f76df7
				sw $a1, 56($0)				# Store $a1 in DMem[14]
				sra $a1, $a0, 5			# $a1 = 0xfff76df7
				sw $a1, 60($0)				# Store $a1 in DMem[15]
				slti $a1, $a0, 1			# $a1 = 1
				sw $a1, 64($0)				# Store $a1 in DMem[16]
				slti $a1, $a1, -1			# $a1 = 0
				sw $a1, 68($0)				# Store $a1 in DMem[17]
				sltiu $a1, $a0, 1			# $a1 = 0
				sw $a1, 72($0)				# Store $a1 in DMem[18]
				sltiu $a1, $a1, -1		# $a1 = 1
				sw $a1, 76($0)				# Store $a1 in DMem[19]
				xori $a1, $a0, 0xf5a0	# $a1 = 0xfeed4b4f
				sw $a1, 80($0)				# Store $a1 in DMem[20]
				lw $a0, 36($0)				# Load Value to test
				lw $a1, 40($0)				# Load Value to test
				lw $a1, 44($0)				# Load Value to test
				lw $a1, 48($0)				# Load Value to test
				lw $a1, 52($0)				# Load Value to test
				lw $a1, 56($0)				# Load Value to test
				lw $a1, 60($0)				# Load Value to test
				lw $a1, 64($0)				# Load Value to test
				lw $a1, 68($0)				# Load Value to test
				lw $a1, 72($0)				# Load Value to test
				lw $a1, 76($0)				# Load Value to test
				lw $a1, 80($0)				# Load Value to test
				nop							# Complete
		*/
			32'hA0: Data = 32'h3c01feed;
			32'hA4: Data = 32'h3424beef;
			32'hA8: Data = 32'hac040024;
			32'hAC: Data = 32'h2085f5a0;
			32'hB0: Data = 32'hac050028;
			32'hB4: Data = 32'h2485f5a0;
			32'hB8: Data = 32'hac05002c;
			32'hBC: Data = 32'h3085f5a0;
			32'hC0: Data = 32'hac050030;
			32'hC4: Data = 32'h00042940;
			32'hC8: Data = 32'hac050034;
			32'hCC: Data = 32'h00042942;
			32'hD0: Data = 32'hac050038;
			32'hD4: Data = 32'h00042943;
			32'hD8: Data = 32'hac05003c;
			32'hDC: Data = 32'h28850001;
			32'hE0: Data = 32'hac050040;
			32'hE4: Data = 32'h28a5ffff;
			32'hE8: Data = 32'hac050044;
			32'hEC: Data = 32'h2c850001;
			32'hF0: Data = 32'hac050048;
			32'hF4: Data = 32'h2ca5ffff;
			32'hF8: Data = 32'hac05004c;
			32'hFC: Data = 32'h3885f5a0;
			32'h100: Data = 32'hac050050;
			32'h104: Data = 32'h8c040024;
			32'h108: Data = 32'h8c050028;
			32'h10C: Data = 32'h8c05002c;
			32'h110: Data = 32'h8c050030;
			32'h114: Data = 32'h8c050034;
			32'h118: Data = 32'h8c050038;
			32'h11C: Data = 32'h8c05003c;
			32'h120: Data = 32'h8c050040;
			32'h124: Data = 32'h8c050044;
			32'h128: Data = 32'h8c050048;
			32'h12C: Data = 32'h8c05004c;
			32'h130: Data = 32'h8c050050;
			32'h134: Data = 32'h00000000;		
			
		/*
		 * Test Program 4
		 * Test jal and jr
		 */
		/*
		TestProg4:
				li $t1, 0xfeed				# $t1 = 0xfeed
				li $t0, 0x190				# Load address of P4jr
				jr $t0						# Jump to P4jr
				li $t1, 0					# Check for failure to jump
		P4jr:	sw $t1, 84($0)				# $t1 should be 0xfeed if successful
				li $t0, 0xcafe				# $t1 = 0xcafe
				jal P4Jal					# Jump to P4Jal
				li $t0, 0xbabe				# Check for failure to jump
		P4Jal:	sw $t0, 88($0)			# $t0 should be 0xcafe if successful
				li $t2, 0xface				# $t2 = 0xface
				j P4Skip						# Jump to P4Skip
				li $t2, 0           	
		P4Skip:	sw $t2, 92($0)			# $t2 should be 0xface if successful
				sw $ra, 96($0)				# Store $ra
				lw $t0, 84($0)				# Load value for check
				lw $t1, 88($0)				# Load value for check
				lw $t2, 92($0)				# Load value for check
				lw $ra, 96($0)				# Load value for check

		*/
			32'h180: Data = 32'h3409feed;
			32'h184: Data = 32'h34080190;
			32'h188: Data = 32'h01000008;
			32'h18C: Data = 32'h34090000;
			32'h190: Data = 32'hac090054;
			32'h194: Data = 32'h3408cafe;
			32'h198: Data = 32'h0c000068;
			32'h19C: Data = 32'h3408babe;
			32'h1A0: Data = 32'hac080058;
			32'h1A4: Data = 32'h340aface;
			32'h1A8: Data = 32'h0800006c;
			32'h1AC: Data = 32'h340a0000;
			32'h1B0: Data = 32'hac0a005c;
			32'h1B4: Data = 32'hac1f0060;
			32'h1B8: Data = 32'h8c080054;
			32'h1BC: Data = 32'h8c090058;
			32'h1C0: Data = 32'h8c0a005c;
			32'h1C4: Data = 32'h8c1f0060;
			32'h1C8: Data = 32'h00000000;
		/*
		 * Test Program 5
		 * Tests mula using wavelet transform
		 */
		
		/*
		TestProg5:
				li $2, 1					# $1 = 1
				li $3, 0					# $2 = -1
				li $20, 0				# $20 = 0 (result)
				li $4, 5					# Load wavelet part 1
				li $5, 7					# Load wavelet part 2
				li $6, 2					# Load wavelet part 3
				li $7, 9					# Load wavelet part 4
		
				mula $20, $4, $2		# Perform convolution
				mula $20, $5, $2
				mula $20, $6, $0
				mula $20, $7, $0				
				sw $20, 104($0)		# Save result
		
				li $20, 0				# Reset result
				mula $20, $4, $0		# Perform convolution
				mula $20, $5, $0
				mula $20, $6, $2
				mula $20, $7, $2
				sw $20, 108($0)		# Save result
		
				li $20, 0				# Reset result
				mula $20, $4, $2		# Perform convolution
				mula $20, $5, $0
				mula $20, $6, $2
				mula $20, $7, $0
				sw $20, 112($0)		# Save result
		
				li $20, 0				# Reset result
				mula $20, $4, $0		# Perform convolution
				mula $20, $5, $2
				mula $20, $6, $0
				mula $20, $7, $2
				sw $20, 116($0)		# Save result
		
				lw $t0, 104($0)		# Load value for check
				lw $t0, 108($0)		# Load value for check
				lw $t0, 112($0)		# Load value for check
				lw $t0, 116($0)		# Load value for check
		*/
			
			32'h200: Data = 32'h34020001;
			32'h204: Data = 32'h34030000;
			32'h208: Data = 32'h34140000;
			32'h20C: Data = 32'h34040005;
			32'h210: Data = 32'h34050007;
			32'h214: Data = 32'h34060002;
			32'h218: Data = 32'h34070009;
			32'h21C: Data = 32'h0082a038;
			32'h220: Data = 32'h00a2a038;
			32'h224: Data = 32'h00c0a038;
			32'h228: Data = 32'h00e0a038;
			32'h22C: Data = 32'hac140068;
			32'h230: Data = 32'h34140000;
			32'h234: Data = 32'h0080a038;
			32'h238: Data = 32'h00a0a038;
			32'h23C: Data = 32'h00c2a038;
			32'h240: Data = 32'h00e2a038;
			32'h244: Data = 32'hac14006c;
			32'h248: Data = 32'h34140000;
			32'h24C: Data = 32'h0082a038;
			32'h250: Data = 32'h00a0a038;
			32'h254: Data = 32'h00c2a038;
			32'h258: Data = 32'h00e0a038;
			32'h25C: Data = 32'hac140070;
			32'h260: Data = 32'h34140000;
			32'h264: Data = 32'h0080a038;
			32'h268: Data = 32'h00a2a038;
			32'h26C: Data = 32'h00c0a038;
			32'h270: Data = 32'h00e2a038;
			32'h274: Data = 32'hac140074;
			32'h278: Data = 32'h8c080068;
			32'h27C: Data = 32'h8c08006c;
			32'h280: Data = 32'h8c080070;
			32'h284: Data = 32'h8c080074;
			
		/*
		 * Test Program 6
		 * Tests Overflow Exceptions
		 */
		
		/*
		Test4-1:
				li $t0, -2147450880
				add $t0, $t0, $t0
				lw $t0, 4($0)
				
		Test4-2:
				li $t0, 2147450879
				add $t0, $t0, $t0
				lw $t0, 4($0)
		
		Test 4-3:
				lw $t0, 4($0)
				li $t0, -2147483648
				li $t1, 1
				sub $t0, $t0, $t1
				lw $t0, 4($0)
		
		Test 4-4:
				li $t0, 2147483647
				mula $t0, $t0, $t0
				lw $t0, 4($0)
		*/
			32'h300: Data = 32'h3c018000;
			32'h304: Data = 32'h34288000;
			32'h308: Data = 32'h01084020;
			32'h30C: Data = 32'h8c080004;
			
			32'h310: Data = 32'h3c017fff;
			32'h314: Data = 32'h34287fff;
			32'h318: Data = 32'h01084020;
			32'h31C: Data = 32'h8c080004;
			
			32'h320: Data = 32'h8c080004;
			32'h324: Data = 32'h3c088000;
			32'h328: Data = 32'h34090001;
			32'h32C: Data = 32'h01094022;
			32'h330: Data = 32'h8c080004;
			
			32'h334: Data = 32'h3c017FFF;
			32'h338: Data = 32'h3428FFFF;
			32'h33C: Data = 32'h01084038;
			32'h340: Data = 32'h8c080004;

		/*
		 * Overflow Exception
		 */
		/*
				lw $t0, 0($0)
		*/
			32'hF0000000: Data = 32'h8c080000;
			
			default: Data = 32'hXXXXXXXX;
		endcase
	end
endmodule

module NextPClogic(NextPC,CurrentPC,JumpField,SignExtImm32,Branch,ALUZero,Jump);
input [31:0] CurrentPC,SignExtImm32;
input [25:0] JumpField;
input Branch,ALUZero,Jump;
output [31:0] NextPC;

reg [31:0]PC4;                                          // to reserve a place to store CurrentPC +4
reg [31:0]NextPC;

always@(*)begin 
PC4=#1 CurrentPC+4;                                     //   PC4 == the current PCaddress +4
if(Jump)begin                                           // if jump=1  NextPc= the combine of PC4 and junpfield and shift of junpfield
NextPC=#3 {PC4[31:28],JumpField[25:24],JumpField<<2};
end
else if(Branch&ALUZero)begin                            // if jump=0 and branch=1 and ALUzero=1  NextPC= PC4 + shift of the signextend immidiate
NextPC=#3 PC4+(SignExtImm32<<2);
end
else begin                                               // defalut 
NextPC=#1PC4;
end
end
endmodule

module RegisterFile(BusA, BusB, BusW,RA,RB, RW, RegWr, Clk);
   //  REG INPUT AND OUTPUT
    	output  [31:0] BusA;
    	output  [31:0] BusB;
   	input [31:0] BusW;
  	input [4:0] RW;
    	input RegWr;
    	input Clk;
	input [4:0] RA;
	input [4:0] RB;
    	reg [31:0] registers [31:0];
	 
	 initial begin
	 // initial the register
	 registers[0] = 0;
         registers[1] = 1;
	 registers[2] = 32'h2;
	 registers[3] = 32'h3;
 	 registers[4] = 32'h4;
	 registers[5] = 32'h5;
	 registers[6] = 32'h6;
	 registers[7] = 32'h7;
	 registers[8] = 32'h8;
	 registers[9] = 32'h9;
	 registers[10] = 32'ha;
	 registers[11] = 32'hb;
	 registers[12] = 32'hc;
	 registers[13] = 32'hd;
	 registers[14] = 32'he;
	 registers[15] = 32'hf;
	 registers[16] = 0;
	 registers[17] = 0;
	 registers[18] = 0;
	 registers[19] = 0;
	 registers[20] = 0;
	 registers[21] = 0;	
         registers[22] = 0;
	 registers[23] = 0;
	 registers[24] = 0;
	 registers[25] = 0;	 
	 registers[26] = 0;
	 registers[27] = 0;
	 registers[28] = 0;
	 registers[29] = 0;
	 registers[30] = 0;
	 registers[31] = 0; 

	 end
	assign #2 BusA = registers[RA];       	//busA was set by regisrerRA
    	assign #2 BusB = registers[RB];        	//BusB was set by registerRB
    always @ (negedge Clk) begin           	// negative edge begin
        if(RegWr)                    
		  begin
		  if (RW!=0)               
            registers[RW] <= #3 BusW;      	//when RW!=0 register which is specified by RW become BusW
		  end
	 end
endmodule

module SignExtender(BusImm, Imm16, Ctrl);
output [31:0] BusImm;
input [15:0] Imm16;
input Ctrl;

wire extBit; 

assign #1	extBit = (Ctrl ? Imm16[15]: 1'b0);//extBit will be Imm16[15] when Ctrl is 1
assign 		BusImm = {{16{extBit}}, Imm16};

endmodule

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

module SingleCycleControl(RegDst, ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, Jump, SignExtend, ALUOp, Opcode);
   input [5:0] Opcode;
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
     
    reg RegDst, ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, Jump, SignExtend;
    reg  [3:0] ALUOp;
    always @ (Opcode) begin
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
                Branch <= #2 0;
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
                SignExtend <= #2 1'b0;
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
                SignExtend <= #2 1'b1;
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
                SignExtend <= #2 1'b0;
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
                SignExtend <= #2 1'b1;
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
                SignExtend <= #2 1'b0;
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
endmodule
