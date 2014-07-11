`timescale 1ns / 1ps

module i_mem2(data1,data2,rd1,rd2,rs_next,rt_next,pc_out,data,en2,read);
input[31:0] data1,data2;
input[31:0] pc_out;
input en2;
output reg [31:0] data;
output reg [4:0] rd1,rd2,rs_next,rt_next;
reg[31:0] pc4,pc8;
reg[7:0] IMEM[7:0];
//wire[2:0] read;
output[2:0] read;

assign read = pc_out[2:0];

always@(*)
	begin
		if(en2)
			begin
				//{IMEM[3],IMEM[2],IMEM[1],IMEM[0]} = data2;
				//{IMEM[7],IMEM[6],IMEM[5],IMEM[4]} = data1;
				{IMEM[3],IMEM[2],IMEM[1],IMEM[0]} = data1;
				{IMEM[7],IMEM[6],IMEM[5],IMEM[4]} = data2;
				pc4 = data2;//storing in reverse order
				pc8 = data1;//storing in reverse order
			end
		else
			begin
				{IMEM[3],IMEM[2],IMEM[1],IMEM[0]} = {IMEM[3],IMEM[2],IMEM[1],IMEM[0]};
				{IMEM[7],IMEM[6],IMEM[5],IMEM[4]} = {IMEM[7],IMEM[6],IMEM[5],IMEM[4]};
				pc4 = pc4;
				pc8 = pc8;
			end
	end

always@(*)
	begin
		data =  {IMEM[read+3],IMEM[read+2],IMEM[read+1],IMEM[read]};
	end

always@(*)//for pc8
begin
if(|(pc8[31:26])== 1'b0)//register
		begin
			rs_next = pc8[25:21];
			rt_next = pc8[20:16];
			rd1 = pc8[15:11];
		end
else if (|(pc8[31:29])== 1'b1)//Immediate
		begin
			rs_next = pc8[25:21];
			rt_next = 5'bz;
			rd1 = pc8[20:16];
		end
else
		begin
			rs_next = 5'bz; 
			rt_next = 5'bz;
			rd1 = 5'bz;
		end
end

always@(*)//for pc4
begin
if(|(pc4[31:26])== 1'b0)//register
		begin
			rd2 = pc8[15:11];
		end
else if (|(pc4[31:29])== 1'b1)//Immediate
		begin
			rd2 = pc8[20:16];
		end
else
		begin
			rd2 = 5'bz;
		end
end
			
endmodule
