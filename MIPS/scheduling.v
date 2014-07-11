`timescale 1ns / 1ps

module scheduling(rd1_q,rd2_q,rs_next_q,rt_next_q,rdef,opcode,sel_inst,reset,read);
input reset;
input[2:0] read;
input[5:0] opcode;
input[4:0] rd1_q,rd2_q,rs_next_q,rt_next_q,rdef;
output reg sel_inst;
reg[2:0] temp;

always @(*)
	begin
		if(reset)
			begin
				sel_inst = 1'b0;
				temp = 3'b0;
			end
		else if(!(rd1_q == rd2_q) && !sel_inst && opcode[5:2] == 4'b1000 && ((rs_next_q == rdef) || (rt_next_q === rdef)))
			begin
				sel_inst = 1'b1;
				temp = read;
			end
		else if(temp == read)
			begin
				sel_inst = 1'b0;
			end
		else
			begin
				//sel_inst = 1'b0;
				sel_inst = sel_inst;
			end
	end
endmodule
