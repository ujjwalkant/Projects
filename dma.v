`timescale 1ns/10ps
//command wont get high when request is zero
module dmac(clk,rst,req,cmdout,addrout,lenout,resp,resptype,respend,
  respaddr,respdatain,respdataout,resphold,
  LSBreq,LSBaddr,LSBwrite,LSBdataOut,LSBdataIn,LSBatten);

input clk,rst;
output reg LSBreq,LSBwrite;
output reg req;
output resphold;
output reg[2:0] cmdout;
output reg[31:0] addrout;//made changes
output reg[31:0] respdataout;
output reg [3:0] lenout;
input resp,respend;
input[2:0] resptype;
input[31:0]respaddr,respdatain;
output reg [15:0]LSBaddr;
reg [15:0] LSBaddr1;
output reg [7:0]LSBdataOut;
input[7:0] LSBdataIn;
input[3:0] LSBatten;
reg[3:0] valid;
reg[31:0] pc_jmp,reg0;
reg[26:0] valid_IB;// change it from 24-->27
reg[7:0] reg_pc;
reg[7:0] register[8'h00+63:8'h00];
reg[31:0] pc;
reg[4:0] pc_IB;//pointer of instruction buffer to see if instruction exist or not
reg[4:0] IB_read;//used for reading instruction buffer by decoder
reg[31:0] run;
reg[7:0] IB[(8'h00+8'h1F):8'h00];
reg[31:0] instruction;
wire[31:0] rd1,rd2;
reg[15:0] rd1_q, constant_q;
reg[4:0] opcode;
reg[3:0] jump_op, Reg1;
reg[5:0] Reg, constant_w, constant_w1, Reg2;//used to select register
reg[8:0] constant;
reg[9:0] jump_offset;
//reg[2:0] save;
reg[3:0] state,save;
reg R, F, D, E, J, NJ, STP,resume,W,P; // Representing states or state machine
wire U,G,L,jump_signal;	 
reg N, z; // flags			    	
reg [31:0] temp, temp1;
reg [7:0] slr_reg;
reg [31:0] aluout, aluout1;
wire [5:0] reg_a, reg_b; 
wire [4:0] offset;
wire [31:0] constant_sext, pc_jump1, pc_jump2;

parameter rw= 4'd0, request= 4'd1, fetch= 4'd2, decode= 4'd3, execute= 4'd4, writeb= 4'd5, jump= 4'd6, stop= 4'd7, LSB = 4'd8; 
//-----
assign jump_signal = (!jump_op[2] & jump_op[1] & N) | (!jump_op[2] & jump_op[0] & z) | (jump_op[2] & !z & !N) | (jump_op[2] & !jump_op[1] &    !jump_op[0]) |  (jump_op[2] & !jump_op[0] & !N) | (jump_op[2] & !jump_op[1] & !z);
assign pc_jump1 = pc + {27'b0,IB_read} + ({{22{jump_offset[9]}}, jump_offset}<<2);////////jump_logic1
assign pc_jump2 = {register[reg_a+3],register[reg_a+2],register[reg_a+1],register[reg_a]} + {register[reg_b+3],register[reg_b+2],register[reg_b+1],register[reg_b]};////////jump_logic2

//----
//assign reg0 = {register[3],register[2],register[1],register[0]};
assign constant_sext = {{23{constant[8]}},constant[8:0]};
assign rd1 = {register[Reg+3],register[Reg+2],register[Reg+1],register[Reg]};
assign rd2 = {register[constant_w+3],register[constant_w+2],register[constant_w+1],register[constant_w]};

assign resphold = 1'b0;
assign offset = pc[4:0];
assign reg_a = (jump_offset[7:4] << 2);
assign reg_b = (jump_offset[3:0] << 2);

assign G = ( !opcode[4] & !opcode[2]) | (!opcode[4] & opcode[1] & opcode[0]) | (!opcode[3] & !opcode[1] & !opcode[0]) | (!opcode[3] & !opcode[2]) | (!opcode[2] & !opcode[1] & !opcode[0]) | (!opcode[4] & !opcode[1] & !opcode[0] );
/*assign G = (opcode== 5'b00000|| opcode== 5'b00001 || opcode== 5'b00010  || opcode== 5'b00011 || opcode== 5'b00100 || opcode== 5'b00111 || opcode== 5'b01000 || opcode== 5'b01001 || opcode== 5'b01010 || opcode== 5'b01011 || opcode== 5'b01100 || opcode== 5'b01111 || opcode== 5'b10000 || opcode== 5'b10001 || opcode== 5'b10010 || opcode== 5'b10011 || opcode== 5'b10100 || opcode== 5'b11000);*/

assign L = (opcode == 5'b10101 || opcode == 5'b10110 );
assign U = (G || opcode == 5'b11010 || 5'b11001);
//////////////////////////////////////////////////////////////////////////////////////////////////////
always@(posedge clk)
	begin
		if(rst) begin
			state <= #0.2 rw;
			resume <= #0.2 1'b0; end
		else begin

		case(state)
			rw:
				if(R)begin
					state <= #0.2 request;end
				else if(resume)begin
					state <= #0.2 save;
					resume <= #0.2 1'b0; end
				else if(resptype == 3'b111 && !resp && run)begin
					state <= #0.2 request; end
					//save  <= #0.2 rw;end					
				else begin
					state <= #0.2 rw;
					$display ("request");end
			request:
				if(resptype == 3'b101)begin
					state <= #0.2 rw;
					save  <= #0.2 request;end
				else if(R)begin
					state <= #0.2 fetch;end
					//save  <= #0.2 request;end
				else begin
					state <= #0.2 request;
					//$display ("else request %b %d",state, $time); 
					end
			fetch:
				if(F)begin
					state <= #0.2 decode;
					save  <= #0.2 fetch;end
				else begin
					state <= #0.2 fetch;
					//$display ("else fetch %b %d",state, $time); 
					end
			decode: 
				if(D && instruction[31:27] == 5'b11011)begin
					state <= #0.2 stop;end
				else if(resptype == 3'b101)begin
					state <= #0.2 rw;
					resume <= #0.2 1'b1; 
					save  <= #0.2 decode; end
				else if(D)begin
					state <= #0.2 execute; end
					//save  <= #0.2 decode; end
				else begin
					state <= #0.2 decode;
					//$display ("else decode %b %d",state, $time); 
					end
			execute:
				if (resptype == 3'b101)begin
					state <= #0.2 rw;
					resume <= #0.2 1'b1;
					save  <= #0.2 execute; end
				else if(L)begin
					state <= #0.2 LSB; end
				else if(U)begin
					state <= #0.2 writeb; end
				else if(E)begin
					state <= #0.2 jump; end					
				else begin
					state <= #0.2 execute;
					//$display ("else execute %b %d",state, $time); 
					end
			writeb:
				if (resptype == 3'b101)begin
					state <= #0.2 rw;
					resume <= #0.2 1'b1;
					save  <= #0.2 writeb; end
				else if (W)begin
					state <= #0.2 jump;
					end
				else
					state<= #0.2 writeb;
			LSB:
				if (resptype == 3'b101)begin
					state <= #0.2 rw;
					resume <= #0.2 1'b1;
					save  <= #0.2 LSB; end
				else if (P)begin
					state <= #0.2 jump;
					end
				else
					state<= #0.2 LSB;
			jump:
				if (resptype == 3'b101)begin
					state <= #0.2 rw;
					resume <= #0.2 1'b1;
					save  <= #0.2 jump; end
				else if(J)begin 
					state <= #0.2 decode;
					//$display ("jump %b %d",state, $time); 
					end
				else if(NJ)begin
					state <= #0.2 request; end	
				else begin
					state <= #0.2 jump;
					//$display ("else jump %b %d",state, $time); 
					end
			stop:
				if(STP)begin
					state <= #0.2 rw;end
				else
					state <= #0.2 state;	
		
			endcase	
			end	
	end

///////////////////////////////////////////////////////////////////////////////////////////////
always @(*)begin
	if (rst) ///state or rst or respdatain or respaddr
	  begin	
		R = 1'b0;		
		F = 1'b0;
		D = 1'b0;
		E = 1'b0;	    	
	    	J = 1'b0;
		W = 1'b0;
		P = 1'b0;
		req = 1'b0;
		LSBreq = 1'b0;
		NJ = 1'b0;
		STP = 1'b0;
		instruction = 32'd0;
		lenout = 4'd0;
		Reg1 = 4'b0;
		temp = 32'd0;
		cmdout = 3'b111;
		slr_reg = 8'b0;
		pc_IB = 5'b0;
		aluout = 32'b0;
		addrout = 32'b0;
		LSBwrite = 1'b0;
		pc_jmp = 32'b0;
		respdataout = 32'b0;
		LSBdataOut = 8'b0;
		LSBaddr = 16'b0;
		reg_pc = 8'b0;
	  end
 else 
     begin
          case (state)
		rw:
			begin	R = 1'b0;
				F = 1'b0;
				D = 1'b0;
				E = 1'b0;
				J = 1'b0;
				W = 1'b0;
				NJ = 1'b0;
				STP = 1'b0;
				req = 1'b0;
				LSBreq = 1'b0;
				LSBwrite = 1'b0;
				instruction = 32'd0;
				lenout = 4'd0;
				Reg1 = 4'b0;
				pc_IB = 5'b0;
				temp = 32'd0;
				cmdout = 3'b111;
				slr_reg = 8'b0;
				aluout = 32'b0;
				addrout = 32'b0;
				pc_jmp = 32'b0;
				LSBdataOut = 8'b0;
				LSBaddr = 16'b0;
				P = 1'b0;
				if(resptype == 3'b110 && respaddr[11:8] == 4'hD && resp)//HSB writing DMA register
					begin
						reg_pc = respaddr[7:0];
						//valid = respaddr[11:8];
						respdataout = run;
					end
				else if(resptype == 3'b101 && respaddr[11:8] == 4'hD && resp)//HSB reading DMA register
					begin
						reg_pc = respaddr[7:0];
						respdataout = {register[reg_pc+3],register[reg_pc+2],register[reg_pc+1],register[reg_pc]};
					end
				else if(resptype == 3'b101 && respaddr[11:4] == 8'hC1 && resp)//Read run
					begin
						reg_pc = 8'b0;
						respdataout = run;
					end
				else if(resptype == 3'b101 && respaddr[11:4] == 8'hC0 && resp)//Read Program Counter
					begin	
						reg_pc = 8'b0;
						respdataout = pc;
					end
				
				else 
					begin
						reg_pc = 8'b0;
						respdataout = run;
					end
			end
		request://filling instruction inside buffer
			begin
				req = (!resp) ? 1'b1 : 1'b0;
				cmdout = 3'b001;
				addrout = (!resp) ? pc : 32'b0;
				lenout = 4'b0111;
				R = 1'b1;
				NJ = 1'b0;
				STP = 1'b0;
				F = 1'b0;
				P = 1'b0;
				D = 1'b0;
				E = 1'b0;
				temp = 32'd0;
				J = 1'b0;
				W = 1'b0;
				Reg1 = 4'b0;
				LSBreq = 1'b0;
				LSBwrite = 1'b0;
				instruction = 32'd0;
				pc_IB = 5'b0;
				slr_reg = 8'b0;
				aluout = 32'b0;
				pc_jmp = 32'b0;
				respdataout = 32'b0;
				LSBdataOut = 8'b0;
				LSBaddr = 16'b0;
				reg_pc = 8'b0;
			end
		fetch:		
			begin	R = 1'b0;
				D = 1'b0;
				E = 1'b0;
				J = 1'b0;
				W = 1'b0;
				NJ = 1'b0;
				STP = 1'b0;
				req = 1'b0;
				P = 1'b0;
				lenout = 4'd0;
				Reg1 = 4'b0;
				temp = 32'd0;
				slr_reg = 8'b0;
				instruction = 32'd0;
				aluout = 32'b0;
				addrout = 32'b0;
				cmdout = 3'b111;
				LSBreq = 1'b0;
				LSBwrite = 1'b0;
				respdataout = 32'b0;
				LSBdataOut = 8'b0;
				LSBaddr = 16'b0;
				pc_jmp = 32'b0;
				pc_IB = respaddr[7:0] - pc[7:0];
				reg_pc = 8'b0;
				F = (pc_IB == (8'd28) && resp) ? 1'b1 : 1'b0;///want resptype == 001
			end
		decode://decode
			begin
				R = 1'b0;
				F = 1'b0;
				E = 1'b0;
				W = 1'b0;
				NJ = 1'b0;
				STP = 1'b0;
				lenout = 4'd0;
				pc_IB = 5'b0;
				temp = 32'd0;
				slr_reg = 8'b0;
				aluout = 32'b0;
				respdataout = 32'b0;
				pc_jmp = 32'b0;
				LSBdataOut = 8'b0;
				LSBaddr = 16'b0;
				instruction = (resptype == 3'b111) ? {IB[IB_read+3],IB[IB_read+2],IB[IB_read+1],IB[IB_read]} : 32'b0;
				reg_pc = 8'b0;
				Reg1 = instruction[26:23];
				//Reg2 = Reg1<< 2;//Reg2 eq to Reg, added for addrout
				req = 1'b0;
				P = 1'b0;
				//constant_w1 = (instruction[17:14] << 2);//constant_w1 eq to constant_w, added for addrout
				addrout = 32'b0;
				D = 1'b1;
				J = 1'b0;
				LSBreq = 1'b0;
				LSBwrite = 1'b0;
				cmdout = 3'b111;
			end

		execute://execute
                	begin	
				F = 1'b0;
				D = 1'b0;
				J = 1'b0;
				W = 1'b0;
				NJ = 1'b0;
				STP = 1'b0;
				lenout = 4'd0;
				Reg1 = 4'b0;
				pc_IB = 5'b0;
				addrout = 32'b0;
				instruction = 32'd0;
				pc_jmp = 32'b0;
				reg_pc = 8'b0;
				R = 1'b0;
				P = 1'b0;
				case (opcode[4:3])
					2'b00: //only Reg operation
						begin	
							case(opcode[2:0])
								3'b000:                                //load
									if(resptype[1])
									  begin
									    	aluout = constant_sext;
					 					E = 1'b1;
										req = 1'b0;
										temp = 32'd0;
										slr_reg = 8'b0;
										LSBreq = 1'b0;
										LSBwrite = 1'b0;
										respdataout = 32'b0;
										LSBdataOut = 8'b0;
										LSBaddr = 16'b0;
										cmdout = 3'b111;
									  end
									else 
									  begin
										aluout = 32'b0;
										E = 1'b1;
										req = 1'b0;
										temp = 32'd0;
										slr_reg = 8'b0;
										LSBreq = 1'b0;
										LSBwrite = 1'b0;
										respdataout = 32'b0;
										LSBdataOut = 8'b0;
										LSBaddr = 16'b0;
										cmdout = 3'b111;
									  end
								3'b001:                                //add
				 					if(resptype[1])
									  begin
										aluout = rd1 + constant_sext ;
										slr_reg = 8'b0;
										E = 1'b1;
										req = 1'b0;
										temp = 32'd0;
										LSBreq = 1'b0;
										LSBwrite = 1'b0;
										respdataout = 32'b0;
										LSBdataOut = 8'b0;
										LSBaddr = 16'b0;
										cmdout = 3'b111;

									end
									else begin
										aluout = 32'b0;
										slr_reg = 8'b0;
										E = 1'b1;
										req = 1'b0;
										temp = 32'd0;
										LSBreq = 1'b0;
										LSBwrite = 1'b0;
										respdataout = 32'b0;
										LSBdataOut = 8'b0;
										LSBaddr = 16'b0;
										cmdout = 3'b111;
									end
								3'b010:                                   //and
									begin
										aluout = rd1 & constant_sext ;
									   	E = 1'b1;
										slr_reg = 8'b0;
										req = 1'b0;
										temp = 32'd0;
										LSBreq = 1'b0;
										LSBwrite = 1'b0;
										respdataout = 32'b0;
										LSBdataOut = 8'b0;
										LSBaddr = 16'b0;
										cmdout = 3'b111;
										
									end

								3'b011:                                          //xor
									begin
										aluout = rd1 ^ constant_sext;
										E = 1'b1;
										slr_reg = 8'b0;
										temp = 32'd0;
										req = 1'b0;
										LSBreq = 1'b0;
										LSBwrite = 1'b0;
										respdataout = 32'b0;
										LSBdataOut = 8'b0;
										LSBaddr = 16'b0;
										cmdout = 3'b111;

									end
	
								3'b100:                                            //sub
									begin
										aluout = rd1 - constant_sext;
										E = 1'b1;
										slr_reg = 8'b0;
										temp = 32'd0;
										req = 1'b0;
										LSBreq = 1'b0;
										LSBwrite = 1'b0;
										respdataout = 32'b0;
										LSBdataOut = 8'b0;
										LSBaddr = 16'b0;
										cmdout = 3'b111;
									end

								3'b101:                                  //cmp & not storing
									begin
										temp = rd1 -  constant_sext;
										E = 1'b1;
										slr_reg = 8'b0;
										aluout = 32'b0;
										LSBreq = 1'b0;
										req = 1'b0;
										LSBwrite = 1'b0;
										respdataout = 32'b0;
										LSBdataOut = 8'b0;
										LSBaddr = 16'b0;
										cmdout = 3'b111;
									end

								3'b110:                                          //tst
									begin
										temp = rd1 &  constant_sext;
										E = 1'b1;
										slr_reg = 8'b0;
										aluout = 32'b0;
										LSBreq = 1'b0;
										req = 1'b0;
										LSBwrite = 1'b0;
										respdataout = 32'b0;
										LSBdataOut = 8'b0;
										LSBaddr = 16'b0;
										cmdout = 3'b111;
									end

								3'b111:                                                   //or		
									begin
										aluout = rd1 | constant_sext;
										E = 1'b1;
										req = 1'b0;
										temp = 32'd0;
										slr_reg = 8'b0;
										LSBreq = 1'b0;
										LSBwrite = 1'b0;
										respdataout = 32'b0;
										LSBdataOut = 8'b0;
										LSBaddr = 16'b0;
										cmdout = 3'b111;
									end
							endcase
						end	 
					2'b01:               //constant & Reg both
						begin
								
				
							case(opcode[2:0])
							
								3'b000: //Loadr	
									begin
										aluout = rd2 ;
										E = 1'b1;
										temp = 32'd0;
										slr_reg = 8'b0;
										LSBreq = 1'b0;
										req = 1'b0;
										LSBwrite = 1'b0;
										respdataout = 32'b0;
										LSBdataOut = 8'b0;
										LSBaddr = 16'b0;
										cmdout = 3'b111;
									end
								3'b001: //Addr
									begin
										aluout = rd1 + rd2 ;
										E = 1'b1;
										temp = 32'b0;
										slr_reg = 8'b0;
										LSBreq = 1'b0;
										req = 1'b0;
										LSBwrite = 1'b0;
										respdataout = 32'b0;
										LSBdataOut = 8'b0;
										LSBaddr = 16'b0;
										cmdout = 3'b111;

									end	
								3'b010: //Andr
									begin
										aluout = rd1 & rd2 ;
										E = 1'b1;
										temp = 32'd0;
										slr_reg = 8'b0;
										LSBreq = 1'b0;
										req = 1'b0;
										LSBwrite = 1'b0;
										respdataout = 32'b0;
										LSBdataOut = 8'b0;
										LSBaddr = 16'b0;
										cmdout = 3'b111;
									end	
								3'b011: //Xorr
									begin
										aluout = rd1 ^ rd2 ;
										E = 1'b1;
										temp = 32'd0;
										slr_reg = 8'b0;
										LSBreq = 1'b0;
										req = 1'b0;
										LSBwrite = 1'b0;
										respdataout = 32'b0;
										LSBdataOut = 8'b0;
										LSBaddr = 16'b0;
										cmdout = 3'b111;
									end	
								3'b100: //Subr
									begin
										aluout = rd1 - rd2 ;
										E = 1'b1;
										temp = 32'd0;
										slr_reg = 8'b0;
										LSBreq = 1'b0;
										req = 1'b0;
										LSBwrite = 1'b0;
										respdataout = 32'b0;
										LSBdataOut = 8'b0;
										LSBaddr = 16'b0;
										cmdout = 3'b111;
									end	
								3'b101: //Cmpr
									begin
										temp = rd1 - rd2 ;
										E = 1'b1;
										slr_reg = 8'b0;
										aluout = 32'b0;
										LSBreq = 1'b0;
										req = 1'b0;
										LSBwrite = 1'b0;
										respdataout = 32'b0;
										LSBdataOut = 8'b0;
										LSBaddr = 16'b0;
										cmdout = 3'b111;
									end
								3'b110: //tstrr
									begin
										temp = rd1 & rd2 ;
										E = 1'b1;
										slr_reg = 8'b0;
										aluout = 32'b0;
										LSBreq = 1'b0;
										req = 1'b0;
										LSBwrite = 1'b0;
										respdataout = 32'b0;
										LSBdataOut = 8'b0;
										LSBaddr = 16'b0;
										cmdout = 3'b111;
									end
								3'b111: //Orr
									begin
										aluout = rd1 | rd2 ;
										E = 1'b1;
										temp = 32'd0;
										slr_reg = 8'b0;
										LSBreq = 1'b0;
										req = 1'b0;
										LSBwrite = 1'b0;
										respdataout = 32'b0;
										LSBdataOut = 8'b0;
										LSBaddr = 16'b0;
										cmdout = 3'b111;
									end
							endcase
						end

					2'b10://////
						begin
								

							case (opcode[2:0])
							
								3'b000://signExt
								begin
													
									aluout = {{24{rd1[7]}},rd1[7:0]}; 
									cmdout = 3'b111;
									E = 1'b1;
									temp = 32'd0;
									slr_reg = 8'b0;
									LSBreq = 1'b0;
									req = 1'b0;
									LSBwrite = 1'b0;
									respdataout = 32'b0;
									LSBdataOut = 8'b0;
									LSBaddr = 16'b0;
								end
								3'b001://SI
								begin
									aluout = rd1 << constant [4:0];
									cmdout = 3'b111;
									E = 1'b1;
									slr_reg = 8'b0;
									temp = 32'd0;
									LSBreq = 1'b0;
									req = 1'b0;
									LSBwrite = 1'b0;
									respdataout = 32'b0;
									LSBdataOut = 8'b0;
									LSBaddr = 16'b0;
								end
								3'b010://Sr
								begin
									aluout = rd1 >> constant [4:0];
									cmdout = 3'b111;
									E = 1'b1;
									slr_reg = 8'b0;
									temp = 32'd0;
									LSBreq = 1'b0;
									req = 1'b0;
									LSBwrite = 1'b0;
									respdataout = 32'b0;
									LSBdataOut = 8'b0;
									LSBaddr = 16'b0;
								end
								3'b011://LoadPC
								begin
									LSBreq = 1'b0;
									LSBwrite = 1'b0;
									req = 1'b0;
									aluout = pc + {27'b0,IB_read} +({{23{constant[8]}}, constant}<<2);
									E = 1'b1;
									temp = 32'd0;
									slr_reg = 8'b0;
									respdataout = 32'b0;
									LSBdataOut = 8'b0;
									LSBaddr = 16'b0;
									cmdout = 3'b111;
								end
								3'b100://Slr
								begin
									cmdout = 3'b111;
									LSBreq = 1'b0;
									LSBwrite = 1'b0;
									slr_reg = register [constant_w]; 
									aluout = rd1 << slr_reg[4:0];
									E = 1'b1;
									temp = 32'd0;
									respdataout = 32'b0;
									LSBdataOut = 8'b0;
									LSBaddr = 16'b0;
									req = 1'b0;
								end/*
								3'b101://Writelsb
								begin
									if (resptype[1])begin
										LSBreq= 1'b1;
										LSBwrite =  1'b1;
										LSBdataOut = register[0];
										LSBaddr =  (rd1[15:0] + {{7{constant[8]}}, constant});  
										///////////////////////////////////////////////////////
										E = 1'b1;
										req = 1'b0;
										cmdout = 3'b111;
										slr_reg = 8'b0;
										temp = 32'd0;
										aluout = 32'b0;
										respdataout = 32'b0;end
									else	begin
										LSBreq= 1'b0;
										LSBwrite = 1'b0;
										LSBdataOut = 8'b0;
										LSBaddr =  16'b0;  
										///////////////////////////////////////////////////////
										E = 1'b1;
										req = 1'b0;
										cmdout = 3'b111;
										slr_reg = 8'b0;
										temp = 32'd0;
										aluout = 32'b0;
										respdataout = 32'b0;end
								end	
								3'b110://Readlsb
								begin
									if (resptype[1])begin
										LSBreq= 1'b1;
										LSBwrite = 1'b0;
										LSBdataOut =8'b0;
										LSBaddr =  (rd1[15:0] + {{7{constant[8]}}, constant});  
										///////////////////////////////////////////////////////
										E = 1'b1;
										req = 1'b0;
										cmdout = 3'b111;
										slr_reg = 8'b0;
										temp = 32'd0;
										aluout = 32'b0;
										respdataout = 32'b0;end
									else begin
										LSBreq= 1'b0;
										LSBwrite =  1'b0;
										LSBdataOut =  8'b0;
										LSBaddr =  16'b0;  
										///////////////////////////////////////////////////////
										E = 1'b1;
										req = 1'b0;
										cmdout = 3'b111;
										slr_reg = 8'b0;
										temp = 32'd0;
										aluout = 32'b0;
										respdataout = 32'b0;end
								end*/	
		
								3'b111://Readtst
								begin
									LSBreq =1'b1;
									req = 1'b0;
									LSBwrite = 1'b0;
									LSBaddr =  rd1[15:0];
									temp[7:0] = LSBdataIn & constant[7:0];
									E = 1'b1;
									cmdout = 3'b111;
									temp[31:8] = 24'd0;
									slr_reg = 8'b0;
									aluout = 32'b0;
									respdataout = 32'b0;
									LSBdataOut = 8'b0;
								end
								default:
								begin
									LSBreq = 1'b0;
									LSBwrite = 1'b0;
									E = 1'b1;
									req = 1'b0;
									temp = 32'd0;
									slr_reg = 8'b0;
									aluout = 32'b0;
									respdataout = 32'b0;
									LSBdataOut = 8'b0;
									LSBaddr = 16'b0;
									cmdout = 3'b111;
																	
								end
							endcase
						end
		
					2'b11:	
						begin
							case(opcode[2:0])
								3'b000://Read atten
								begin
									LSBreq = 1'b0;
									LSBwrite = 1'b0;
									aluout = {28'b0, LSBatten} & {{7{constant[8]}}, constant};
									E = 1'b1;
									req = 1'b0;
									cmdout = 3'b111;
									temp = 32'd0;
									slr_reg = 8'b0;
									respdataout = 32'b0;
									LSBdataOut = 8'b0;
									LSBaddr = 16'b0;
								end
							
								3'b001://Read HSB
								begin
					LSBreq = 1'b0; LSBwrite = 1'b0; E = 1'b1; temp = 32'd0; slr_reg = 8'b0; aluout = 32'b0;
					respdataout = 32'b0; LSBdataOut = 8'b0; LSBaddr = 16'b0;
					req= 1'b1;
					cmdout = 3'b000;
					addrout = rd1 + rd2;
								end
								3'b010://writehsb
								begin
					LSBreq = 1'b0; LSBwrite = 1'b0; E = 1'b1; temp = 32'd0; slr_reg = 8'b0; aluout = 32'b0;
					respdataout = 32'b0; LSBdataOut = 8'b0; LSBaddr = 16'b0;
					req= 1'b1;
					cmdout =  3'b010 ;
					addrout = rd1 + rd2;
								end
								
								default:
								begin
									LSBreq = 1'b0;
									LSBwrite = 1'b0;
									E = 1'b1;
									req = 1'b0;
									temp = 32'd0;
									slr_reg = 8'b0;
									aluout = 32'b0;
									respdataout = 32'b0;
									LSBdataOut = 8'b0;
									LSBaddr = 16'b0;
									cmdout = 3'b111;
								end
								
							endcase 
					
						end
						
				endcase
			end
		LSB:
			begin	
				R = 1'b0; F = 1'b0; D = 1'b0; E = 1'b0; W = 1'b0; STP = 1'b0; lenout = 4'd0; Reg1 = 4'b0; temp = 32'd0;lenout=  4'd0; pc_IB = 5'b0;P = 1'b1;pc_jmp = 32'b0;reg_pc = 8'b0;LSBdataOut = 8'b0;aluout =32'b0;NJ = 1'b0;Reg1 = 4'b0;addrout = 32'b0;
respdataout = 32'b0; slr_reg = 8'b0; temp = 32'd0; cmdout = 3'b111; J = 1'b0;  instruction = 32'd0; req = 1'b0;
					if(resptype == 3'b111)begin
					LSBreq= 1'b1;
					LSBwrite = opcode[0] ? 1'b1 : 1'b0;
					LSBdataOut = opcode[0] ? register[0] : 8'b0;
					//LSBaddr =  (rd1[15:0] + {{7{constant[8]}}, constant});
					LSBaddr = rd1_q + constant_q;
					//P = 1'b1;
					end  
					else begin
					E = 1'b0; LSBreq= 1'b0; LSBwrite= 1'b0;
					LSBdataOut = 8'b0; LSBaddr = 16'b0;
					LSBaddr = 16'b0;
					end
			end
		writeb: 
			begin
				W = 1'b1; R = 1'b0; F = 1'b0; D = 1'b0; E = 1'b0; STP = 1'b0; J = 1'b0; NJ = 1'b0; temp = 32'd0; 
				reg_pc = 8'b0; slr_reg = 8'b0; lenout = 4'd0; Reg1 = 4'b0; pc_IB = 5'b0; aluout = 32'b0;
				cmdout = 3'b111; LSBdataOut = 8'b0; req = 1'b0; LSBreq = 1'b0; LSBwrite = 1'b0;
				addrout = 32'b0; instruction = 32'd0; pc_jmp = 32'b0; LSBaddr = 16'b0; P = 1'b0;
				respdataout = (opcode == 5'b11010) ? {register[3],register[2],register[1],register[0]} : 32'b0;//HSB Write
				
			end

		jump://Jump--for incrementing pointer put here
		   begin R = 1'b0;
			 F = 1'b0;
			 D = 1'b0;
			 E = 1'b0;
			 W = 1'b0;
			 P = 1'b0;
			 STP = 1'b0;
			 lenout = 4'd0;
			 Reg1 = 4'b0;
			 temp = 32'd0;
			 slr_reg = 8'b0;
			 cmdout = 3'b111;
			 aluout = 32'b0;
			 LSBdataOut = 8'b0;
			 reg_pc = 8'b0;
			 LSBreq = 1'b0;
			 req = 1'b0;
			 LSBwrite = 1'b0;
			 pc_IB = 5'b0;
			 addrout = 32'b0;	
			 instruction = 32'd0;
			 respdataout={register[3],register[2],register[1],register[0]};
			 LSBaddr = 16'b0;


				if( !jump_op[3] && jump_signal && resptype[1])
					begin
					pc_jmp = pc_jump1;
					
						if(valid_IB == pc_jmp[31:5])
							begin
							J = 1'b1;
							NJ = 1'b0;
							end
						else 
							begin
							J = 1'b0;
							NJ = 1'b1;
							end
					end

				else if(jump_op[3] && jump_signal && resptype[1])
					begin	
					pc_jmp = pc_jump2;
					
						if(valid_IB == pc_jmp[31:5])
							begin
							J = 1;
							NJ = 1'b0;
							end
						else 
							begin
							J = 1'b0;
							NJ = 1'b1;
							end
				 	end
				else
					begin
						pc_jmp = 32'b0;
						if(IB_read != 5'd28)
							begin
							J = (resptype[1] == 1'b1) ? 1'b1 : 1'b0;
							NJ = 1'b0;
							end
						else
							begin
							NJ = (resptype[1] == 1'b1) ? 1'b1 : 1'b0;
							J = 1'b0;
							end
					end

			end

			

		stop:
			begin	P = 1'b0;	
				R = 1'b0;
				F = 1'b0;
				D = 1'b0;
				E = 1'b0;
				W = 1'b0;
				STP = 1'b1;
				J = 1'b0;
				NJ = 1'b0;
				temp = 32'd0;
			 	slr_reg = 8'b0;
				lenout = 4'd0;
				pc_IB = 5'b0;
				aluout = 32'b0;
				cmdout = 3'b111;
				LSBdataOut = 8'b0;
			 	LSBreq = 1'b0;
				req = 1'b0;
			 	LSBwrite = 1'b0;
				addrout = 32'b0;
				instruction = 32'd0;
				respdataout = 32'b0;
				pc_jmp = 32'b0;
				LSBaddr = 16'b0;
				reg_pc = 8'b0;
				Reg1 = 4'b0;			
			end
		default:
			begin
				P = 1'b0;
				R = 1'b0;
				F = 1'b0;
				D = 1'b0;
				E = 1'b0;
				W = 1'b0;
				J = 1'b0;
				temp = 32'd0;
			 	slr_reg = 8'b0;
				STP = 1'b0;
				NJ = 1'b0;
				lenout = 4'd0;
				pc_IB = 5'b0;
				cmdout = 3'b111;
				LSBdataOut = 8'b0;
			 	LSBreq = 1'b0;
				req = 1'b0;
			 	LSBwrite = 1'b0;
				aluout = 32'b0;
				addrout = 32'b0;
				instruction = 32'd0;
				respdataout = 32'b0;
				LSBaddr = 16'b0;
				pc_jmp = 32'b0;
				Reg1 = 4'b0;
				reg_pc = 8'b0;
			end
		endcase
	end end
//////////////////////////////////////////////////////////////////////////////
always @(posedge clk) begin
	if (rst)
	  begin	
		IB_read <= #0.2 5'b0;
		//z<= #0.2 1'b0;
		//N<= #0.2 1'b0;
		//aluout1 <= #0.2 32'b0;
		//respaddr_q<= respaddr;
		//temp1 <= #0.2 1'b0;
		register[32] <= #0.2 8'd0;register[33] <= #0.2 8'd0;register[34] <= #0.2 8'd0;register[35] <= #0.2 8'd0;
		register[52] <= #0.2 8'd0;register[53] <= #0.2 8'd0;register[54] <= #0.2 8'd0;register[55] <= #0.2 8'd0;
		register[56] <= #0.2 8'd0;register[57] <= #0.2 8'd0;register[58] <= #0.2 8'd0;register[59] <= #0.2 8'd0;
	  end
 else 
     begin
          case (state)
		rw:
			begin
				if(resptype == 3'b110 && respaddr[11:8] == 4'hD && resp)//HSB writing DMA register
					begin
						{register[reg_pc+3],register[reg_pc+2],register[reg_pc+1],register[reg_pc]} <= #0.2 respdatain;
					end
				else if(resptype == 3'b110 && respaddr[11:4] == 8'hC1 && resp)//Write run		
					begin
						run <= #0.2 respdatain;
					end
				else if(resptype == 3'b110 && respaddr[11:4] == 8'hC0 && resp)//Write Program Counter
					begin
						pc <= #0.2 respdatain;
					end
				else 
					begin
						run <= #0.2 run;
					end
			end
		fetch:		
			begin
				valid_IB <= #0.2 pc[31:5];
				{IB[pc_IB+3],IB[pc_IB+2],IB[pc_IB+1],IB[pc_IB]} <= #0.2 respdatain;
			end

		decode://decode
			begin
				opcode <= #0.2 instruction[31:27];
				Reg <= #0.2 Reg1<< 2;
				constant <= #0.2 instruction[22:14];
				constant_w <= #0.2 (instruction[17:14] << 2);
				jump_op <= #0.2 instruction[13:10];
				jump_offset <= #0.2 instruction[9:0];
			end

		execute://execute
                	begin   
				if(opcode == 5'b00000 && resptype[1] == 1'b1)//load
					begin
					aluout1 <= #0.2 {{23{constant[8]}},constant[8:0]};  
										
					end
				else if(opcode == 5'b00001 && resptype[1] == 1'b1)//add
					begin
					aluout1 <= #0.2 rd1 + {{23{constant[8]}},constant[8:0]};
					N <= #0.2 (aluout[31]== 1'b1)? 1:0 ;
					z <= #0.2 (aluout == 32'b0)? 1:0 ;
					
					end
				else if (opcode == 5'b00010)//and
					begin
					aluout1 <= #0.2 rd1 & {{23{constant[8]}},constant[8:0]}; 
			           	N <= #0.2 (aluout[31]== 1'b1)? 1:0 ;
					z <= #0.2 (aluout == 32'b0)? 1'b1:1'b0 ;
					
					end
				else if (opcode == 5'b00011)//xor
					begin
					aluout1 <= #0.2 rd1 ^ {{23{constant[8]}},constant[8:0]}; 
					N <= #0.2 (aluout[31]== 1'b1)? 1:0 ;
					z <= #0.2 (aluout == 32'b0)? 1'b1:1'b0 ;
					
					end
				else if (opcode == 5'b00100)//sub
					begin
					aluout1 <= #0.2 rd1 - {{23{constant[8]}},constant[8:0]}; 
					N <= #0.2 (aluout[31]== 1'b1)? 1:0 ;
					z <= #0.2 (aluout == 32'b0)? 1'b1:1'b0 ;
					
					end
				else if (opcode == 5'b00101)// cmp & not storing
					begin
				
					N <= #0.2 (temp[31]== 1'b1)? 1:0 ;
					z <= #0.2 (temp == 32'b0)? 1:0 ;
					
					end
				else if (opcode == 5'b00110)// tst// make temp as 32 bit register in main module
					begin
					
					N <= #0.2 (temp[31]== 1'b1)? 1:0 ;
					z <= #0.2 (temp == 32'b0)? 1:0 ;
					
					end		 
				else if (opcode == 5'b00111)// or
					begin
					aluout1 <= #0.2 rd1 | {{23{constant[8]}},constant[8:0]}; 
					N <= #0.2 (aluout[31]== 1'b1)? 1:0 ;
					z <= #0.2 (aluout == 32'b0)? 1'b1:1'b0 ;

					end
				else if (opcode == 5'b01000)// Loadr
					begin
					aluout1 <= #0.2 rd2;
									
					end	
				else if (opcode == 5'b01001)// Addr
					begin
					aluout1 <= #0.2 rd2 + rd1;
					N <= #0.2 (aluout[31]== 1'b1)? 1:0 ;
					z <= #0.2 (aluout == 32'b0)? 1'b1:1'b0 ;
					
					end
				else if (opcode == 5'b01010)// Andr
					begin
					aluout1 <= #0.2 rd2 & rd1;
					N <= #0.2 (aluout[31]== 1'b1)? 1:0 ;
					z <= #0.2 (aluout == 32'b0)? 1'b1:1'b0 ;
					
					end	
				else if (opcode == 5'b01011)// Xorr
					begin
					aluout1 <= #0.2 rd2  ^ rd1;
					N <= #0.2 (aluout[31]== 1'b1)? 1:0 ;
					z <= #0.2 (aluout == 32'b0)? 1'b1:1'b0 ;
					
					end
				else if (opcode == 5'b01100)// Subr
					begin
					aluout1 <= #0.2 rd1 - rd2;
					N <= #0.2 (aluout[31]== 1'b1)? 1:0 ;
					z <= #0.2 (aluout == 32'b0)? 1'b1:1'b0 ;
					
					end
				else if (opcode == 5'b01101)// Cmpr
					begin
					
					N <= #0.2 (temp[31]== 1'b1)? 1:0 ;
					z <= #0.2 (temp == 32'b0)? 1:0 ;	
								
					end
				else if (opcode == 5'b01110 && resptype[1] == 1'b1)// tstr
					begin 
					
					N <= #0.2 (temp[31]== 1'b1)? 1:0 ;
					z <= #0.2 (temp == 32'b0)? 1:0 ;
					
					end
				else if (opcode == 5'b01111)// Orr
					begin
					aluout1 <= #0.2 rd1 | rd2;
					N <= #0.2 (aluout[31]== 1'b1)? 1:0 ;
					z <= #0.2 (aluout == 32'b0)? 1'b1:1'b0 ;
					
					end
				else if (opcode == 5'b10000)// SignExt
					begin
					aluout1 <= #0.2 {{24{rd1[7]}},rd1[7:0]}; 
					N <= #0.2 (aluout[31]== 1'b1)? 1:0 ;
					z <= #0.2 (aluout == 32'b0)? 1'b1:1'b0 ;
					end
				else if (opcode == 5'b10001)// SI
					begin
					aluout1 <= #0.2 rd1 << constant [4:0];
					N <= #0.2 (aluout[31]== 1'b1)? 1:0 ;
					z <= #0.2 (aluout == 32'b0)? 1'b1:1'b0 ;
					
 					end
				else if (opcode == 5'b10010)// Sr
					begin
					aluout1 <= #0.2 rd1 >> constant [4:0];
					N <= #0.2 (aluout[31]== 1'b1)? 1:0 ;
					z <= #0.2 (aluout == 32'b0)? 1'b1:1'b0 ;
					
 					end
				else if (opcode == 5'b10011 && resptype[1] == 1'b1)// LoadPC
					begin
					aluout1 <= #0.2 pc + {27'b0,IB_read} + ({{23{constant[8]}}, constant}<<2);
					N <= #0.2 (aluout[31]== 1'b1)? 1:0 ;
					z <= #0.2 (aluout == 32'b0)? 1'b1:1'b0 ;
					
 					end
				else if (opcode == 5'b10100)//Slr
					begin
					aluout1 <= #0.2 rd1 << slr_reg[4:0]; 
					N <= #0.2 (aluout[31]== 1'b1)? 1:0 ;
					z <= #0.2 (aluout == 32'b0)? 1'b1:1'b0 ;
					
 					end
				else if ((opcode == 5'b10110 || opcode == 5'b10101) && resptype[1] == 1'b1)//Read lsb
					begin
					
					rd1_q <= #0.2 rd1[15:0];
					constant_q <= #0.2 ({{7{constant[8]}}, constant}); 
				
					end
				else if (opcode == 5'b10111 && resptype[1] == 1'b1)//Read tst
					begin
					
					N <= #0.2 (temp[7]== 1'b1)? 1:0 ;
					z <= #0.2 (temp[7:0] == 8'b0)? 1:0 ;
					
					end
				else if (opcode == 5'b11000)//Read atten
					begin
					
					aluout1 <= #0.2 {28'b0, LSBatten} & {{7{constant[8]}}, constant};
					N <= #0.2 (aluout[31]== 1'b1)? 1:0 ;
					z <= #0.2 (aluout == 32'b0)? 1'b1:1'b0 ;
					
					end
				else if (opcode == 5'b11001)//Read HSB
					begin
		
					N <= #0.2 (respdatain[31]== 1'b1)? 1:0 ;
					z <= #0.2 (respdatain == 32'b0)? 1:0 ;
					end
				else 	
					begin
					z <= #0.2 z;
					N <= #0.2 N;
					end
		
		end
		writeb:
			begin
				if(G )
				{register[Reg+3],register[Reg+2],register[Reg+1],register[Reg]} <= #0.2 aluout1;
 				else if (opcode == 5'b11001)
				{register[3],register[2],register[1],register[0]} <= #0.2 respdatain;
				else
				register[Reg] <= #0.2 register[Reg];
			end
		LSB:
			begin
				if(opcode == 5'b10110)begin
				register [0] <= #0.2 LSBdataIn;							 
				N <= #0.2 (register[0][7]== 1'b1)? 1:0;
				z <= #0.2 (register[0]== 8'b0)? 1:0;end
				else
				register [0] <= #0.2 register[0];	
				
			end
	
		jump://Jump--for incrementing pointer put here
		   begin
			if (jump_op == 4'b0001 && z== 1 && resptype[1] == 1'b1)
				begin
				 	if(valid_IB == pc_jmp[31:5])
							begin
								IB_read <= #0.2 pc_jmp [4:0] - offset;
								
							end
						else begin
								pc <= #0.2 pc_jmp;
								IB_read <= #0.2 5'b0;
							end
				end
			else if (jump_op ==4'b0010 && N ==1 && resptype[1] == 1'b1)
				begin
				 	
					if(valid_IB == pc_jmp[31:5])
							begin
								IB_read <= #0.2 pc_jmp [4:0] - offset;
								
							end
						else begin
								pc <= #0.2 pc_jmp;
								IB_read <= #0.2 5'b0;
							end
				end
			else if (jump_op == 4'b0011 && (N | z ) && resptype[1] == 1'b1)
				begin
				 	if(valid_IB == pc_jmp[31:5])
							begin
								IB_read <= #0.2 pc_jmp [4:0] - offset;
								
							end
						else begin
							//resphold = 1'b1;
							end
				end
			else if (jump_op == 4'b0100 && resptype[1] == 1'b1)
				begin
					if(valid_IB == pc_jmp[31:5])
							begin
								IB_read <= #0.2 pc_jmp [4:0] - offset;
								
							end
						else begin
							//resphold = 1'b1;
							end
				end
			else if (jump_op == 4'b0101 && !z && resptype[1] == 1'b1)
				begin
					if(valid_IB == pc_jmp[31:5])
							begin
								IB_read <= #0.2 (resptype == 3'b111) ? (pc_jmp [4:0] - offset) : IB_read;
								
							end
						else begin
							  	pc <= #0.2 pc_jmp;
								IB_read <= #0.2 5'b0;
							end
				
				end	
			else if (jump_op == 4'b0110 && !N && resptype[1] == 1'b1)
				begin
					if(valid_IB == pc_jmp[31:5])
							begin
								IB_read <= #0.2 pc_jmp [4:0] - offset;
								
							end
						else begin
							  //resphold = 1'b1;
							end
				end
			else if (jump_op == 4'b0111 && !z && !N && resptype[1] == 1'b1)
				begin
					if(valid_IB == pc_jmp[31:5])
							begin
								IB_read <= #0.2 pc_jmp [4:0]- offset;
								
							end
						else begin
							  	pc <= #0.2 pc_jmp;
								IB_read <= #0.2 5'b0;
							end
				end
			else if (jump_op == 4'b1001 && z && resptype[1] == 1'b1)
				begin
					if(valid_IB == pc_jmp[31:5])
							begin
								IB_read <= #0.2 pc_jmp [4:0];
								
							end
						else begin
							  	pc <= #0.2 pc_jmp;
								IB_read <= #0.2 5'b0;
							end
				end
			else if (jump_op == 4'b1010 && N && resptype[1] == 1'b1)
				begin
					if(valid_IB == pc_jmp[31:5])
							begin
								IB_read <= #0.2 pc_jmp [4:0];
								
							end
						else begin
							  	pc <= #0.2 pc_jmp;
								IB_read <= #0.2 5'b0;
							end
				end
			else if (jump_op == 4'b1011 && (z | N) && resptype[1] == 1'b1)
				begin
					if(valid_IB == pc_jmp[31:5])
							begin
								IB_read <= #0.2 pc_jmp [4:0];
								
							end
					else begin
							  //resphold = 1'b1;
							end
				end
			else if (jump_op == 4'b1100 && resptype[1] == 1'b1)
				begin
					if(valid_IB == pc_jmp[31:5])
							begin
								IB_read <= #0.2 pc_jmp [4:0];
								
							end
						else begin
							  	pc <= #0.2 pc_jmp;
								IB_read <= #0.2 5'b0;
							end
				end		
			else if (jump_op == 4'b1101 && !z && resptype[1] == 1'b1)
				begin
					if(valid_IB == pc_jmp[31:5])
							begin
								IB_read <= #0.2 pc_jmp [4:0];
								
							end
						else begin
							  	pc <= #0.2 pc_jmp;
								IB_read <= #0.2 5'b0;
							end
				end
			else if (jump_op == 4'b1110 && !N && resptype[1] == 1'b1)
				begin
					if(valid_IB == pc_jmp[31:5])
							begin
								IB_read <= #0.2 pc_jmp [4:0];
								
							end
						else begin
							  //resphold = 1'b1;
							end				
				end
			else if (jump_op == 4'b1111 && !z && !N && resptype[1] == 1'b1)
				begin
					if(valid_IB == pc_jmp[31:5])
							begin
								IB_read <= #0.2 pc_jmp [4:0];
								
							end
						else begin
							  //resphold = 1'b1;
							end
				end
			else
				begin
					if(IB_read != 5'd28)
						begin
						IB_read <= #0.2 (resptype[1] == 1'b1) ? (IB_read + 5'd4) : IB_read;
						end
					else
						begin
						pc <= #0.2 (resptype[1] == 1'b1) ? pc + {27'b0,IB_read} + 32'd4 : pc;
						IB_read <= #0.2 (resptype[1] == 1'b1) ? IB_read + 5'd4 : IB_read;
						end
				end
 	  	end
		stop:
			begin	
				run <= # 0.2 32'b0;
				IB_read <= #0.2 5'b0;
			end
		endcase
	end end
endmodule
