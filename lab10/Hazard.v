`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    18:22:52 04/11/2008 
// Design Name: 
// Module Name:    Hazard 
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
module Hazard(PCWrite, IFWrite, Bubble, Branch, ALUZero4, Jump, rw, rs, rt, Reset_L, CLK);
	input			Branch;
	input			ALUZero4;
	input			Jump;
	input	[4:0]	rw;
	input	[4:0]	rs;
	input	[4:0]	rt;
	input			Reset_L;
	input			CLK;
	output reg	PCWrite;
	output reg	IFWrite;
	output reg	Bubble;
		
	/*state definition for FSM*/
		parameter NoH_state = 3'b000,        /*Define a name for each of the states so it is easier to debug and follow*/ 
		Bubble0 = 3'b001, 
		Bubble1 = 3'b010, 
		Branch0 = 3'b011, 
		Branch1 = 3'b100, 
		Branch2 = 3'b101, 
		J = 3'b110;

	/*internal signals*/
		wire cmp1, cmp2, cmp3;

	/*internal state*/
		reg [2:0] FSM_state, FSM_nxt_state;
		reg [4:0] rw1, rw2, rw3; 	//rw history registers

		assign  cmp1 = (((rs== rw1)||(rt== rw1))&&(rw1!= 0)) ? 1:0;  //if (m+1)th instruction will use the output from mth instrucion
		assign  cmp2 = (((rs== rw2)||(rt== rw2))&&(rw2!= 0)) ? 1:0; //if (m+2)th instuction will use the output	
		 assign  cmp3 = (((rs== rw3)||(rt== rw3))&&(rw3!= 0)) ? 1:0;

		always @(negedge CLK) begin  
		case(Reset_L)               
		 0:begin                  
		     FSM_state <= 0;
		     rw1 <=  0;
			 rw2 <=  0;
			 rw3 <=  0;
			 end
	         1: begin                  
		      FSM_state <= FSM_nxt_state;
		      if(Bubble)               
			  rw1 <= 0;
			  else                     
			  rw1 <= rw;
			  rw2 <= rw1;
			  rw3 <= rw2;
			  end
			  endcase
			  end
			  always @(*) begin 
			  case(FSM_state)
			  NoH_state: begin                       // No hazard state
			     if(Jump== 1'b1) begin   		//  When jump comes, next state will be J 
			     	FSM_nxt_state <= #2 J; 
                 	     	PCWrite <= #2 1'b1;
			     	IFWrite <= #2 1'b0;
			     	Bubble  <= #2 1'bx; 
			     end

				 else if(cmp1== 1'b1) begin   //  Machine need to wait two cycles
			    	 	FSM_nxt_state <= #2 Bubble0;
			     		PCWrite <= #2 1'b0;
			     		IFWrite <= #2 1'b0;
				 	Bubble  <= #2 1'b1;
			     end
			     else if(cmp2== 1'b1) begin   // (m+2)th instructio nwill use output from mth instruction
			     		FSM_nxt_state <= #2 Bubble1;
			     		PCWrite <= #2 1'b0;
			     		IFWrite <= #2 1'b0;
			     		Bubble  <= #2 1'b1;
			     end
			     else if(cmp3 == 1'b1) begin    // The next stage will directly be NoH_stage
			     		FSM_nxt_state <= #2 NoH_state;
			     		PCWrite <= #2 1'b0;
			     		IFWrite <= #2 1'b0;
			     		Bubble  <= #2 1'b1;
			     end
				else if(Branch== 1'b1) begin  // When branch comes, we need 3 cycles noop, so next state is branch0
			     		FSM_nxt_state <= #2 Branch0;
			     		PCWrite <= #2 1'b0;
			     		IFWrite <= #2 1'b0;
			     		Bubble  <= #2 1'b0;
				end
			     else begin 
			     		FSM_nxt_state <= #2 NoH_state;                //Return to the No Hazard State
			     		PCWrite <= #2 1'b1;
			     		IFWrite <= #2 1'b1;
			     		Bubble  <= #2 1'b0; 
			     end
			     end
			     Bubble0: begin                   	//Bubble0 will actually take 2 cycles to return No Hazard
			     FSM_nxt_state <= #2 Bubble1;             
			     PCWrite <= #2 1'b0;
			     IFWrite <= #2 1'b0;
			     Bubble  <= #2 1'b1;
			     end
			     Bubble1: begin                      // Bubble1 will take 1 cycle to return NO Hazard 
			     FSM_nxt_state <= #2 NoH_state;                
			     PCWrite <= #2 1'b0;
			     IFWrite <= #2 1'b0;
			     Bubble  <= #2 1'b1;
			     end
			     Branch0: begin                      // Branch0 takes 3 cycles to return No Hazard
			     FSM_nxt_state <= #2 Branch1;
			     PCWrite <= #2 1'b0;
			     IFWrite <= #2 1'b0;
			     Bubble  <= #2 1'b1;
			     end
			     Branch1: begin                         //Branch1 takes 2 cycles to return No Hazard
			     if(ALUZero4) begin
			     	FSM_nxt_state <= #2 Branch2;
			     	PCWrite <= #2 1'b1;
			     	IFWrite <= #2 1'b0;
			     	Bubble  <= #2 1'b1;
			     end
			     else begin                                  //finish cycle stalling
                 	     FSM_nxt_state <= #2 NoH_state;
			     PCWrite <= #2 1'b1;
			     IFWrite <= #2 1'b1;
			     Bubble  <= #2 1'b1;
			     end
			     end
			     Branch2: begin                               // Only take 1 cycle to return
			     FSM_nxt_state <= #2 NoH_state;
			     PCWrite <= #2 1'b1;
			     IFWrite <= #2 1'b1;
			     Bubble  <= #2 1'b1;
			     end
			     J: begin                                  // finish stalling
			     FSM_nxt_state <= #2 NoH_state; 
			     PCWrite <= #2 1'b1;
			     IFWrite <= #2 1'b1;
			     Bubble  <= #2 1'bX; 
			     end
			     default: begin                            //default
			     FSM_nxt_state <= #2 NoH_state;
			     PCWrite <= #2 1'bx;
			     IFWrite <= #2 1'bx;
			     Bubble  <= #2 1'bx;
			     end
			  endcase
		end
    endmodule
