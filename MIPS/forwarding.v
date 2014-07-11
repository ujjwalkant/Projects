`timescale 1ns / 1ps

module forwarding(rdef_final_q,rdef_final_q1,rs_q,rt_q,selA,selB,reg_write_q1,reg_write_q2,f);
input reg_write_q1,reg_write_q2,f;
input[4:0] rdef_final_q, rs_q,rt_q, rdef_final_q1;
output reg[1:0] selA, selB;

always@(*)///for Rs handling Mux A
	begin
		if((rs_q == rdef_final_q) && reg_write_q1 && f)
		selA = 2'b11;
		else if((rs_q == rdef_final_q1) && reg_write_q2 && f)
		selA = 2'b10;
		else
		selA = 2'b00;
	end

always@(*)///for Rt handling Mux B
	begin
		if((rt_q == rdef_final_q) && reg_write_q1 && f)
		selB = 2'b11;
		else if((rt_q == rdef_final_q1) && reg_write_q2 && f)
		selB = 2'b10;
		else
		selB = 2'b00;
	end
endmodule
