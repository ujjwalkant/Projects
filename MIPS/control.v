`timescale 1ns / 10ps

module control(instruction, rs,rt,rd,opcode,funct, shamt,hint,zeroth,immediate,instr_index, reg_des, branch, mem2reg, mem_en 	
	      ,reg_write, jump, jr, byte_en,IMEM_EN);

input 	  [31:0] instruction;
output reg [5:0] opcode, funct;
output reg [4:0] rs;
output reg [4:0] rt;
output reg [4:0] rd;
output reg [4:0] shamt;
output reg [4:0] hint;
output reg [9:0] zeroth;
output reg [15:0] immediate; 
output reg [25:0] instr_index;
output reg reg_des,branch,mem2reg,mem_en,reg_write,jump,jr,byte_en,IMEM_EN;

always @(*)
begin
	if((instruction[29:26] == 4'b0010) || ((instruction[29:26] == 4'b0011) && (instruction[31] == 1'b0)))//jump & JAL without jr
		begin
			opcode = instruction[31:26];
			instr_index =instruction[25:0];
			jump = 1'b1;
			branch = 0;
			//IMEM_EN = 1;
			reg_des = 1'b0;//JAL
			reg_write = instruction[26] ? 1'b1 : 1'b0;//JAL
			rt = instruction[26] ? 5'b11111 : 5'b00000;//JAL
			mem2reg = 1'bz;
			jr = 0;
			mem_en = 1'bz;
			byte_en = 1'bz;
			
		end
	else if ((instruction[5:0]== 6'b001000) && (instruction[31:26]== 6'b000000))// jump with jr
		begin
			opcode= instruction[31:26];
			rs = instruction[25:21];
			zeroth = instruction[20:11];
			hint = instruction[10:6];
			funct=instruction[5:0];
			jump = 1'b0;//changing to 0 from z because of stall
			branch = 1'b0;//changing to 0 from z because of stall
			//IMEM_EN = 1;
			reg_des = 1'bz;
			reg_write = 0;
			mem2reg = 1'bz;
			jr = 1;
			mem_en = 1'bz;
			byte_en = 1'bz;
			
		end
		
	else if(|(instruction[31:26])== 1'b0)//register
		begin
			opcode= instruction[31:26];
			rs = instruction[25:21];
			rt = instruction[20:16];
			rd = instruction[15:11];
			shamt = instruction[10:6];
			funct=instruction[5:0];
			jump = 0;
			branch = 0;
			//IMEM_EN = 1;
			reg_des = 1;
			reg_write = 1;
			mem2reg = 1;
			jr = 0;
			mem_en = 1'bz;
			byte_en = 1'bz;
			
		end
		
	else if(|(instruction[31:26])== 1'b1 && instruction[31] == 1'b1 && instruction[29] == 1'b0 && instruction[27:26]== 2'b00)//load byte
			begin
				opcode= instruction[31:26];
				rs = instruction[25:21];
				rt = instruction[20:16];
				immediate =instruction[15:0];
				jump = 0;
				branch = 0;
				//IMEM_EN = 1;
				reg_des = 0;
				reg_write = 1;
				mem2reg = 0;
				jr = 0;
				mem_en = 0;
				byte_en = 1;
			
			end	
	else if (|(instruction[31:26])== 1'b1 && instruction[31] == 1'b1 && instruction[29] == 1'b0 && instruction[27:26]== 2'b11)// load word		
			begin
				opcode= instruction[31:26];
				rs = instruction[25:21];
				rt = instruction[20:16];
				immediate =instruction[15:0];
				jump = 0;
				branch = 0;
				//IMEM_EN = 1;
				reg_des = 0;
				reg_write = 1;
				mem2reg = 0;
				jr = 0;
				mem_en = 0;
				byte_en = 0;
									
			end
	else if (|(instruction[31:26])== 1'b1 && instruction[31] == 1'b1 && instruction[29] == 1'b1 && instruction[27:26]== 2'b00)//store byte
			begin
				opcode= instruction[31:26];
				rs = instruction[25:21];
				rt = instruction[20:16];
				immediate =instruction[15:0];
				jump = 0;
				branch = 0;
				//IMEM_EN = 1;
				reg_des = 0;
				reg_write = 0;
				mem2reg = 1'bz;
				jr = 0;
				mem_en = 1;
				byte_en = 1;
				
			end
	else if (|(instruction[31:26])== 1'b1 && instruction[31] == 1'b1 && instruction[29] == 1'b1 && instruction[27:26]== 2'b11)//store word
			begin
				opcode= instruction[31:26];
				rs = instruction[25:21];
				rt = instruction[20:16];
				immediate =instruction[15:0];
				jump = 0;
				branch = 0;
				//IMEM_EN = 1;
				reg_des = 0;
				reg_write = 0;
				mem2reg = 1'bz;
				jr = 0;
				mem_en = 1;
				byte_en = 0;
				
			end
	else if (|(instruction[31:29])== 1'b0 && instruction[31] == 1'b0 && instruction[27:26]== 2'b00)//BEQ
          begin
				opcode= instruction[31:26];
				rs = instruction[25:21];
				rt = instruction[20:16];
				immediate =instruction[15:0];
				jump = 0;
				branch = 1;
				//IMEM_EN = 1;
				//IMEM_EN = 0;//added later
				reg_des = 1'bz;
				reg_write = 0;
				mem2reg = 1'bz;
				jr = 0;
				mem_en = 1'bz;
				byte_en = 1'bz;
				
			end
	else if (|(instruction[31:29])== 1'b0 && instruction[31] == 1'b0 && instruction[27:26]== 2'b01)//BNE
	       begin
				opcode= instruction[31:26];
				rs = instruction[25:21];
				rt = instruction[20:16];
				immediate =instruction[15:0];
				jump = 0;
				branch = 1;
				//IMEM_EN = 1;
				reg_des = 1'bz;
				reg_write = 0;
				mem2reg = 1'bz;
				jr = 0;
				mem_en = 1'bz;
				byte_en = 1'bz;
				
			end
				
	else if (|(instruction[31:29])== 1'b1)//Immediate
			begin
				opcode= instruction[31:26];
				rs = instruction[25:21];
				rt = instruction[20:16];
				immediate =instruction[15:0];
				jump = 0;
				branch = 0;
				//IMEM_EN = 1;
				reg_des = 0;
				reg_write = 1;
				mem2reg = 1;
				jr = 0;
				mem_en = 1'bz;
				byte_en = 1'bz;
				
			end
	else
			begin//No operation
				opcode= instruction[31:26];
				rs = instruction[25:21];
				rt = instruction[20:16];
				rd = instruction[15:11];
				shamt = instruction[10:6];
				funct=instruction[5:0];
			   jump = 0;
				branch = 0;
				//IMEM_EN = 1;
				reg_des = 1'bz;
				reg_write = 0;
				mem2reg = 1'bz;
				jr = 0;
				mem_en = 1'bz;
				byte_en = 1'bz;
				
			end 
end
endmodule
