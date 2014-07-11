`timescale 1ns / 1ps

module hazard_detection(opcode,pc_en,IF,ID,EX,Mem,rs,rt,rdef_final,rst,clk);
input rst,clk;
input[4:0] rs,rt,rdef_final;
input[5:0] opcode;
reg [1:0] count;
output reg pc_en,IF,ID,EX,Mem;

always@(*)
	begin
		if(rst)
			begin
				IF = 1'b1;
				ID = 1'b1;
				EX = 1'b1;
				Mem = 1'b1;
				pc_en = 1'b1;
			end
		else if((count == 2'b00) && (opcode[5:2] == 4'b1000) && ((rdef_final === rs) || (rdef_final === rt)))
			begin
				pc_en = 1'b0;
				IF = 1'b0;
				ID = 1'b0;
				EX = 1'b1;
				Mem = 1'b1;
			end
		else
			begin
				IF = 1'b1;
				ID = 1'b1;
				EX = 1'b1;
				Mem = 1'b1;
				pc_en = 1'b1;
			end
	end
	
always@(posedge clk or posedge rst)
	begin
		if(rst)
			begin
				count <= 0;
			end
		else if(!pc_en)
			begin
				count <= count + 2'b01;
			end
		else
			begin
				count <= 2'b00;
			end
	end
endmodule
