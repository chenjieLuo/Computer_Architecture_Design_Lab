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
    
    parameter n = 32;

    output  [n-1:0] BusW;
    input   [n-1:0] BusA, BusB;
    input   [3:0] ALUCtrl;
    output  Zero;
	 wire [31:0] BusA_cmp,BusB_cmp;
	 
    reg     [n-1:0] BusW;
    
    always @(ALUCtrl or BusA or BusB) begin
        case(ALUCtrl)
         `AND: begin			//Bitwise AND comparison
            BusW <= #20 BusA & BusB;
         end
	`OR: begin			//Bitwise OR comparison
	 	BusW <= #20 BusA | BusB;
	end
	`ADD: begin			//Add value from BusA with BusB
	 	BusW <= #20 BusA + BusB;
	end
	`SLL: begin			//shift left for certain bits saved in BusB
	 	BusW <= #20 BusA << BusB;
	end	
	`SRL: begin			//shift right for certain bits saved BusB
	 	BusW <= #20 BusA >> BusB;
	end	
	`SUB: begin			//Substraction
	 	BusW <= #20 BusA - BusB;
	end	
	`SLT: begin			//Set less than instruction, if BusA is smaller than BusB, output will be 1
		if (BusA <= BusB)
			BusW <= #20 1;
		else BusW <= #20 0;
	end	
	`ADDU: begin			//Addtion unsigned
	 	BusW <= #20 BusA + BusB;
	end
	`SUBU: begin			//Substraction unsigned
	 	BusW <= #20 BusA - BusB;
	end	
	`XOR: begin			//Exclusive or
	 	BusW <= #20 BusA ^ BusB;
	end	
	`SLTU: begin			//set less than unsigned
	 	if (BusA <= BusB)
			BusW <= #20 1;
		else BusW <= #20 0;
	end
	`NOR: begin			//not or instruction
		BusW <= #20 !(BusA | BusB);
	end	
	`SRA: begin
	 	BusW <= #20 $signed(BusA)>>>BusB;
	end
	`LUI: begin
	 	BusW <= #20 (BusA << BusB); //Load upper immediate
	end
	
        endcase
    end
	assign #1 Zero = (BusW==32'b0) ? 1: 0;
endmodule
