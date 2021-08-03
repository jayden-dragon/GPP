module GPP (CLOCK_50,SW,HEX0,HEX1,HEX2,HEX7,rst_);

   input CLOCK_50,rst_;
   input [0:0]SW;
   output [0:6] HEX0,HEX1,HEX2,HEX7; //OutData,n or m or o 

   reg [25:0]count; 
	
	wire Start;
	
   wire [7:0] ALUresult; 
   wire [7:0] RFr1, RFr2, RFr3;
   wire [3:0] pc, state; 
   

   assign clk = count[25]; 
   assign clk1 = CLOCK_50; 
   assign Start = SW[0];
	

  
	
   always @(posedge clk1)
      count = count+1; 
		
  
 
   // display the data input, data output, and address on the 7-segs

   hex7seg digit0 (RFr3[3:0], HEX0);
   hex7seg digit1 (RFr2[3:0], HEX1);
   hex7seg digit2 (RFr1[3:0], HEX2);
   hex7seg digit7 (state[3:0], HEX7);

   Control_GPP(pc, dir, clr_pc, incr_pc, ld_pc, sel_ma, Mwe, RFr1a, RFr1e, RFr2a, RFr2e, RFr3a, RFr3e, set_RF3, clr_RF3, sel_rf, sel_ALU, Start, rst_, clk, Mre, MAresult, ALUresult,rfn_eq_rfm,rfn_lt_rfm,r_data1);
	Datapath_GPP(ld_IR, clr_pc, incr_pc, ld_pc, Mwe, RFr1e, RFr2e, RFr3e, set_RF3, clr_RF3, sel_rf, sel_ALU, Start, rst_, clk, rfn_eq_rfm, rfn_lt_rfm);
	memory_GPP(clk, Mwe,pc, w_addr, r_addr,w_data,r_data1);
endmodule


module hex7seg (hex, display);
   input [3:0] hex;
   output [0:6] display;

   reg [0:6] display;

   /*
    *       0  
    *      ---  
    *     |   |
    *    5|   |1
    *     | 6 |
    *      ---  
    *     |   |
    *    4|   |2
    *     |   |
    *      ---  
    *       3  
    */
   always @ (hex)
      case (hex)
         4'h0: display = 7'b0000001;
         4'h1: display = 7'b1001111;
         4'h2: display = 7'b0010010;
         4'h3: display = 7'b0000110;
         4'h4: display = 7'b1001100;
         4'h5: display = 7'b0100100;
         4'h6: display = 7'b0100000;
         4'h7: display = 7'b0001111;
         4'h8: display = 7'b0000000;
         4'h9: display = 7'b0000100;
         4'hA: display = 7'b0001000;
         4'hb: display = 7'b1100000;
         4'hC: display = 7'b0110001;
         4'hd: display = 7'b1000010;
         4'hE: display = 7'b0110000;
         4'hF: display = 7'b0111000;
      endcase
endmodule


module Control_GPP(pc, addr, clr_pc, incr_pc, ld_pc, sel_ma, Mwe, RFr1a, RFr1e, RFr2a, RFr2e, RFr3a, RFr3e, set_RF3, clr_RF3, sel_rf, sel_ALU, Start, rst_, clk, Mre, MAresult, ALUresult,rfn_eq_rfm,rfn_lt_rfm,r_data1);

	output reg   clr_pc, incr_pc, ld_pc;
	output reg   Mre, Mwe, RFr1a, RFr1e, RFr2a, RFr2e, RFr3e, RFr3a, set_RF3, clr_RF3;
	output reg   [15:0] MAresult;
	output reg	[1:0] sel_rf, sel_ALU;
   input   Start, rst_, clk;
   input  [15:0] ALUresult;
   input rfn_eq_rfm, rfn_lt_rfm;
	input [1:0] sel_ma;
	input [7:0] pc, addr;
   
	input [15:0] r_data1;
   reg ld_IR;
   reg [3:0] state, next_state;
   reg [1:0] IR;
   parameter    S_0 = 4'b0000, S_1 = 4'b0001, S_2 = 4'b0010, S_3 = 4'b0011,
                S_4 = 4'b0100, S_5 = 4'b0101, S_6 = 4'b0110;   // State codes

   always @ (posedge clk, negedge rst_)   
   if(rst_==0) state<=S_0; else state <= next_state;  

	always@(posedge clk)
			
	if (ld_IR) IR <= r_data1[15:12];
		

   
   always @ (state, Start, IR)    
    begin
   next_state=S_0;
   case(state)
      S_0: if(Start) next_state=S_1; else next_state=S_0;
      S_1: next_state=S_2;
      S_2: next_state=S_3;
      S_3: if(IR==0) next_state=S_4; else if(IR==1) next_state=S_5; else if(IR==2) next_state=S_6; 
      default: next_state=S_1; 
   endcase
   end
 
   always @ (state, Start, IR, rfn_lt_rfm, rfn_eq_rfm) 
      begin
         clr_pc=0;
         ld_IR=0;
         incr_pc=0;
         RFr1a=0;
         RFr1e=0;
         RFr2a=0;
         RFr2e=0;
         RFr3a=0;
         RFr3e=0;
         set_RF3=0;
         clr_RF3=0;
         Mwe=0;
			Mre=0;
         ld_pc=0;
         incr_pc=0;
         sel_rf=00;
         sel_ALU=00;

   case(state)
      S_0: if(Start) clr_pc=1; 
      S_1: begin Mre=1; incr_pc=1; end
      S_2: ld_IR=1;
      S_3: if(IR==0) begin Mwe=1; sel_rf=00; end 
            else if(IR==1) begin RFr1e=1; end 
            else if(IR==2) begin RFr1e=1; end
            else if(IR==3) begin Mwe=1; sel_rf=10; end
            else if(IR==4) begin Mwe=1; sel_rf=01; RFr1e=1; RFr2e=1; RFr3e=1; sel_ALU=00; end
            else if(IR==5) begin Mwe=1; sel_rf=01; RFr1e=1; RFr2e=1; RFr3e=1; sel_ALU=01; end
            else if(IR==6) begin Mwe=1; sel_rf=01; RFr1e=1; RFr2e=1; RFr3e=1; sel_ALU=10; end
            else if(IR==7) begin Mwe=1; sel_rf=01; RFr1e=1; RFr2e=1; RFr3e=1; sel_ALU=11; end
            else if(IR==8) begin RFr1e=1; ld_pc=1; end
            else if(IR==9) if (rfn_eq_rfm)  clr_RF3=1; else set_RF3=1;
            else if(IR==10) if (!rfn_lt_rfm) clr_RF3=1; else set_RF3=1;
      S_4: Mre=1;
      S_5: Mwe=1;
      S_6: Mwe=1;
   endcase
  end

  always @ (sel_ma,pc,r_data1,addr) begin 
case (sel_ma)
2'b00 : MAresult = pc;
2'b01 : MAresult = r_data1[11:8];
2'b10 : MAresult = addr[7:0];
endcase
end
  
  endmodule

module Datapath_GPP(r_data1, ld_IR, clr_pc, incr_pc, ld_pc,w_addr, r_addr, Mwe, RFwe, RFr1e, RFr2e, RFr3e, set_RF3, clr_RF3, sel_rf, sel_ALU, Start, rst_, clk, rfn_eq_rfm, rfn_lt_rfm);
   input clk, Start, rst_;
   input RFr1e, RFr2e, RFr3e, clr_RF3, set_RF3;   
   input sel_ALU, sel_rf, Mwe,RFwe;
   input ld_IR, clr_pc, incr_pc, ld_pc;
	
	input [15:0] r_data1;
   
   output wire rfn_eq_rfm, rfn_lt_rfm;
	output wire [7:0] w_addr, r_addr;
 
   reg [3:0] ALUresult, IR;
   reg [3:0] RFr1a, RFr2a, RFr3a;
   reg [7:0] pc;
   reg [7:0] addr, RFresult;
   
	reg R[9];	
		
	assign rn = r_data1[11:8], rm = r_data1[7:4], ro = r_data1[3:0];
   
always @ (posedge clk) begin
         if (ld_pc) pc <= addr[7:0];
         if (clr_pc) pc <= 0;
         if (incr_pc) pc <= pc + 1;
			
			if(RFr1e) RFr1a = R[rn];
			if(RFr2e) RFr2a = R[rm];
			if(RFr3e) RFr3a = R[ro];


			if(clr_RF3) R[ro] <= 0; 
			if(set_RF3) R[ro] <= 1; 
			end
			
	assign rfn_eq_rfm = (R[rn] == R[rm])? 1'b1:1'b0,			
		rfn_lt_rfm = (R[rn] < R[rm])? 1'b1:1'b0; 
	
	assign w_addr = r_data1[7:0],
		    r_addr = r_data1[7:0];




     
   always @ (sel_ALU,R[rn],R[r_data1[7:4]],ALUresult) 
      begin
         case(sel_ALU)
            2'b00 : ALUresult = R[rn] + R[rm];
            2'b01 : ALUresult = R[rn] - R[rm];
            2'b10 : ALUresult = R[rn] / R[rm];
            2'b11 : ALUresult = R[rn] % R[rm];
         endcase
      end
      
		
      always @ (sel_rf,addr[7:0],ALUresult) begin 
     case(sel_rf) 
      2'b00 : RFresult = addr[7:0];
      2'b01 : RFresult = ALUresult;
      2'b10 : RFresult = addr[7:0];
      endcase
      end
		
		always@(posedge clk)
		
		if(RFwe) R[ro] <= ALUresult;
		

		
		
always@(IR, rn, rm, ro)	
			begin
				if((IR == 2)|(IR == 4)|(IR == 5)|(IR == 6)|(IR == 7)|(IR == 9)|(IR == 10))
				begin
					R[rn] <= rn;
					R[rm] <= rm;
					R[ro] <= ro;
				end
				else if((IR == 0)|(IR == 1)|(IR == 3)|(IR == 8))
				begin
					R[rn] <= rn;
					addr[7:0] <= r_data1[7:0];
				end
			end
	
		
endmodule

module memory_GPP
  (
    input wire clk,
    input wire Mwe,
	 input wire [7:0] pc,
    input wire [7:0] w_addr, r_addr,
    input wire [15:0] w_data,
    output wire [15:0] r_data1
  );
  
  (* ram_init_file = "Fibo.mif" *) reg [15:0] memory_array [127:0];
  reg [3:0] Address_reg;
  
  
  // Write(store
   always @(posedge clk) 
      if(Mwe)
         begin
            memory_array[w_addr] <= w_data;              
         end
		
               
            
   // Read(load
   assign   r_data = memory_array[r_addr];
            
  
  
  /*always @(posedge clk) begin
      if (Mwe)
         memory_array[wm_addr]<=wm_data;
      Address_reg <= rm_addr;
   end
  
   assign rm_data = memory_array[Address_reg];*/

endmodule
