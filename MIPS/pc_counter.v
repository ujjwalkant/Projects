`timescale 1ns/10ps

module pc_counter(clk, rst, branch_cntrl, zero, jump , pc_out,jr, operand1, branch1, instruction,pc_en,IF,ID,EX);

input zero, jr;
input jump;
input [31:0] operand1;
input [25:0] instruction;
input branch_cntrl;
input clk;
input rst;
input pc_en,IF,ID,EX;
//output reg IMEM_en;//changed it back to reg from wire
//wire IMEM_en_q, s;
output reg [31:0] pc_out;
reg [31:0] pc_out_q, pc_out_q1;
reg   branch_select;
wire [31:0] pc_add;
reg  [31:0] pc_add_q, pc_add_q1;
wire [31:0] branch_add;
reg  [31:0] branch_add_q, operand1_q1;
reg  [31:0] mux1_out;
reg  [31:0] pc;
wire [15:0] address;
input[31:0] branch1 ;
wire [31:0] jump_add;
reg [31:0] jump_add_q;

always @(*)
	begin
		if(rst)
			begin
				branch_select = 0;
			end
		else
			begin
				branch_select = branch_cntrl & zero; // branch_control signal will come from control unit. Made it under always because to intialize it
			end
	end

assign branch_add = pc_add_q1+(branch1<<2);//shifting by 2 is like multiplying by 4 because JTA & BTA requires word boundries
assign pc_add = pc_out + 32'd4; // program counter is incremented by 4

always @(branch_select, branch_add, pc_add)// branch instruction implemented
	begin
		if (branch_select==1)
			mux1_out = branch_add_q; 
		else
			mux1_out = pc_add;
	end

assign jump_add =  {pc_out_q1[31:28],instruction[25:0] << 2} ;

always@ (jump or jump_add or mux1_out)//jump implementation//force branch operation
	begin
    if ( jump == 1 ) 
        pc = jump_add_q;
    else
        pc = mux1_out;
	end

always@(posedge clk or posedge rst)//fetch stage
	begin
		if(rst)
			begin
				pc_add_q <= 0;
				pc_out_q <= 0;
			end
		else if (IF && !jr)
			begin
				pc_add_q <= pc_add;
				pc_out_q <= pc_out;
			end
		else
			begin
				pc_add_q <= pc_add_q;
				pc_out_q <= pc_out_q;
			end
	end
	
always@(posedge clk or posedge rst)//id stage
	begin
		if(rst)
			begin
				pc_out_q1 <= 0;
				pc_add_q1 <= 0;
			end
		else if (ID && !jr)
			begin
				pc_out_q1 <= pc_out_q;
				pc_add_q1 <= pc_add_q;
			end
		else
			begin
				pc_out_q1 <= pc_out_q1;
				pc_add_q1 <= pc_add_q1;
			end
	end

always@(posedge clk or posedge rst)//ex stage
	begin
		if(rst)
			begin
				jump_add_q <= 0;
				branch_add_q <= 0;
			end
		else if (EX && !jr)
			begin
				jump_add_q <= jump_add;
				branch_add_q <= branch_add;
			end
		else
			begin
				jump_add_q <= jump_add_q;
				branch_add_q <= branch_add_q;
			end
	end


always@(posedge clk or posedge rst)//pc stage
	begin
		if(rst)
			begin
				pc_out <= 0;
			end
		else if (jr && pc_en)
			begin
				pc_out <= operand1;
			end
		else if (pc_en)
			begin
				pc_out <= pc;
			end	
	end

endmodule
			
