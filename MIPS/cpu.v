`timescale 1ns / 10ps

module cpu(clk, reset, overflow);

input clk, reset;
wire [31:0]pc_out;
reg [31:0] pc_out_q,pc_out_q1,pc_out_q2,pc_out_q3;
//wire [31:0]data, 
wire [31:0]data_out, alu_op, mem_out, data_out1;
reg [31:0] alu_op_q, alu_op_q1, mem_out_q;
wire [5:0] opcode, funct;
reg  [5:0] opcode_q, funct_q;
wire zero;// it will come from alu.
wire [31:0] instruct,data1,data2,instruct1,instruct2;
reg  [31:0] instruct_q;
wire [4:0] read1, read2, shamt, hint;//rs = read1 & rt = read2
wire [4:0] wreg;
reg  [4:0] wreg_q, read2_q, read1_q;
wire [31:0] operand1, operand2;
reg  [31:0] operand1_q, operand2_q, operand2_q1, operand1_q1;
wire [9:0] zeroth; //jr inst
wire [31:0] immediate1;
reg  [31:0] immediate1_q;
wire [15:0] immediate;
wire [25:0] instr_index;
reg  [25:0] instr_index_q;
wire	[4:0] rdef_final,rdef,rd1,rd2,rs_next,rt_next;
reg   [4:0] rdef_final_q, rdef_final_q1,rd1_q,rd2_q,rs_next_q,rt_next_q;
output overflow;
wire  reg_des, branch, mem2reg, reg_write, mem_en, byte_en, jump, jr, jal;
reg jump_q, jr_q, branch_q, mem_en_q, byte_en_q, reg_des_q, mem2reg_q, reg_write_q,zero_q,jal_q,jal_q1;
reg jump_q1, branch_q1, byte_en_q1, mem_en_q1, mem2reg_q1,reg_write_q1, jr_q1, reg_write_q2, mem2reg_q2;
wire s, IMEM_en, sel_inst,en2;
wire  [1:0] selA, selB;
wire  [2:0] read;
wire pc_en,IF,ID,EX,Mem,f;

assign f = (pc_en & IF & ID & EX & Mem);
assign rdef = (reg_des)? wreg : read2;
assign rdef_final = (reg_des_q)? wreg_q : read2_q;
assign immediate1 ={{16{immediate[15]}}, immediate[15:0]}; // sign extension
assign data_out1 = mem2reg_q2 ? alu_op_q1 : mem_out_q;
assign data_out = jal_q1 ? pc_out_q3 : data_out1;
assign jal = opcode_q[0] & jump_q;
assign instruct =  sel_inst  ? instruct2 : instruct1; //handling logic to avoid stall after load

forwarding i0(.rdef_final_q(rdef_final_q),.rdef_final_q1(rdef_final_q1),.rs_q(read1_q),.rt_q(read2_q),.selA(selA),.selB(selB),.reg_write_q1(reg_write_q1),.reg_write_q2(reg_write_q2),.f(f));

pc_counter i1 (.clk(clk), .rst(reset), .branch_cntrl(branch_q1), .zero(zero_q),.instruction(instr_index_q), .jump(jump_q1) , .pc_out(pc_out),.jr(jr_q1), .operand1(operand1_q1),.branch1(immediate1_q),.pc_en(pc_en),.IF(IF),.ID(ID),.EX(EX));

I_Memory   i2 (.read(pc_out),.data(instruct1),.en(IMEM_en),.data1(data1),.data2(data2),.sel_inst(sel_inst),.en2(en2));//Instruction Memory

control i3(.instruction(instruct_q),.rs(read1),.rt(read2),.rd(wreg),.opcode(opcode),.funct(funct), .shamt(shamt),.hint(hint),.zeroth(),.immediate(immediate),.instr_index(instr_index),.reg_des(reg_des), .branch(branch), .mem2reg(mem2reg), .mem_en(mem_en), .reg_write(reg_write), .jump(jump), .jr(jr), .byte_en(byte_en), .IMEM_EN());
									
register_file i4 (.read1(read1),.read2(read2),.data1(operand1),.data2(operand2),.wreg(rdef_final_q1),.wdata(data_out),.wr_en(reg_write_q2),.rst(reset));

ALU i5 (.Opcode(opcode_q),.Operand1(operand1_q),.Operand(operand2_q),.Immediate(immediate1_q),.Output(alu_op),.Zero(zero),.Overflow(overflow),.Funct(funct_q),.selB(selB),.selA(selA),.data_out1(data_out1),.alu_op_q(alu_op_q));

D_Memory i6 (.read(alu_op_q),.write(alu_op_q),.wdata(operand2_q1),.data(mem_out),.en(mem_en_q1),.byte_en(byte_en_q1));

i_mem2 i7 (.data1(data1),.data2(data2),.rd1(rd1),.rd2(rd2),.rs_next(rs_next),.rt_next(rt_next),.pc_out(pc_out),.data(instruct2),.en2(en2),.read(read));

scheduling i8 (.rd1_q(rd1_q),.rd2_q(rd2_q),.rs_next_q(rs_next_q),.rt_next_q(rt_next_q),.rdef(rdef),.opcode(opcode),.sel_inst(sel_inst),.reset(reset),.read(read));

hazard_detection i9 (.opcode(opcode_q),.pc_en(pc_en),.IF(IF),.ID(ID),.EX(EX),.Mem(Mem),.rs(read1),.rt(read2),.rdef_final(rdef_final),.rst(reset),.clk(clk));

always@(posedge clk or posedge reset)//fetch stage
	begin
		if(reset)
			begin
				instruct_q <= 0;
				pc_out_q <= 0;
			end	
		else if (IF)
			begin
				instruct_q <= instruct;
				pc_out_q <= pc_out;
			end
		else
			begin
				instruct_q <= instruct_q;
				pc_out_q <= pc_out_q;
			end
	end
				
always@(posedge clk or posedge reset)//id stage				
	begin
		if(reset)
			begin
				pc_out_q1 <= 0;
				read1_q <= 0;
				read2_q <= 0;
				wreg_q <= 0;
				immediate1_q <= 0;
				operand1_q <= 0;
				operand2_q <= 0;
				opcode_q <= 0;
				funct_q <= 0;
				branch_q <= 0;
				byte_en_q <= 0;
				mem_en_q <= 0;
				mem2reg_q <= 0;
				jump_q <= 0;
				jr_q <= 0;
				reg_des_q <= 0;
				reg_write_q <= 0;
				instr_index_q <= 0;
				///////////////////
				rd1_q <= 0;
				rd2_q <= 0;
				rs_next_q <= 0;
				rt_next_q <= 0;
			end
		else if (ID)
			begin
				pc_out_q1 <= pc_out_q;
				read1_q <= read1;
				read2_q <= read2;
				wreg_q <= wreg;
				immediate1_q <= immediate1;
				operand1_q <= operand1;
				operand2_q <= operand2;
				opcode_q <= opcode;
				funct_q <= funct;
				branch_q <= branch;
				byte_en_q <= byte_en;
				mem_en_q <= mem_en;
				mem2reg_q <= mem2reg;
				jump_q <= jump;
				jr_q <= jr;
				reg_des_q <= reg_des;
				reg_write_q <= reg_write;
				instr_index_q <= instr_index;
				/////////////////////////////handling logic to avoid stall after load
				rd1_q <= rd1;
				rd2_q <= rd2;
				rs_next_q <= rs_next;
				rt_next_q <= rt_next;
			end
		else
			begin
				pc_out_q1 <= pc_out_q1;
				//read1_q <= read1_q;
				read1_q <= 5'bz;
				read2_q <= 5'bz;
				//read2_q <= read2_q;
				wreg_q <= wreg_q;
				immediate1_q <= immediate1_q;
				operand1_q <= operand1_q;
				operand2_q <= operand2_q;
				opcode_q <= opcode_q;
				funct_q <= funct_q;
				branch_q <= branch_q;
				byte_en_q <= byte_en_q;
				mem_en_q <= mem_en_q;
				mem2reg_q <= mem2reg_q;
				jump_q <= jump_q;
				jr_q <= jr_q;
				reg_des_q <= reg_des_q;
				reg_write_q <= reg_write_q;
				instr_index_q <= instr_index_q;
				//////////////////////////////
				rd1_q <= rd1_q;
				rd2_q <= rd2_q;
				rs_next_q <= rs_next_q;
				rt_next_q <= rt_next_q;
			end
	end
	
always@(posedge clk or posedge reset)//ex stage
	begin
		if(reset)
			begin
				pc_out_q2 <= 0;
				alu_op_q <= 0;
				zero_q <= 0;
				rdef_final_q <= 0;
				operand1_q1 <= 0;
				operand2_q1 <= 0;
				jump_q1 <= 0;
				branch_q1 <= 0;
				byte_en_q1 <=  0;
				mem_en_q1 <= 0;
				mem2reg_q1 <= 0;
				reg_write_q1 <= 0;
				jr_q1 <= 0;
				jal_q <= 0;
			end	
		else if (EX)
			begin
				pc_out_q2 <= pc_out_q1;
				alu_op_q <= alu_op;
				zero_q <= zero;
				rdef_final_q <= rdef_final;
				operand1_q1 <= operand1_q;
				operand2_q1 <= operand2_q;
				jump_q1 <= jump_q;
				branch_q1 <= branch_q;
				byte_en_q1 <=  byte_en_q;
				mem_en_q1 <= mem_en_q;
				mem2reg_q1 <= mem2reg_q;
				reg_write_q1 <= reg_write_q;
				jr_q1 <= jr_q;
				jal_q <= jal;
			end
		else
			begin
				pc_out_q2 <= pc_out_q2;
				alu_op_q <= alu_op_q;
				zero_q <= zero_q;
				rdef_final_q <= rdef_final_q;
				operand1_q1 <= operand1_q1;
				operand2_q1 <= operand2_q1;
				jump_q1 <= jump_q1;
				branch_q1 <= branch_q1;
				byte_en_q1 <=  byte_en_q1;
				mem_en_q1 <= mem_en_q1;
				mem2reg_q1 <= mem2reg_q1;
				reg_write_q1 <= reg_write_q1;
				jr_q1 <= jr_q1;
				jal_q <= jal_q;
			end
	end

always@(posedge clk or posedge reset)//mem stage
	begin
		if(reset)
			begin
				mem_out_q <= 0;
				alu_op_q1 <= 0;
				rdef_final_q1 <= 0;
				reg_write_q2 <= 0;
				mem2reg_q2 <= 0;
				jal_q1 <= 0;
				pc_out_q3 <= 0;
			end	
		else if (Mem)
			begin
				mem_out_q <= mem_out;
				alu_op_q1 <= alu_op_q;
				rdef_final_q1 <= rdef_final_q;
				reg_write_q2 <= reg_write_q1;
				mem2reg_q2 <= mem2reg_q1;
				jal_q1 <= jal_q;
				pc_out_q3 <= pc_out_q2;
			end
		else
			begin
				mem_out_q <= mem_out_q;
				alu_op_q1 <= alu_op_q1;
				rdef_final_q1 <= rdef_final_q1;
				reg_write_q2 <= reg_write_q2;
				mem2reg_q2 <= mem2reg_q2;
				jal_q1 <= jal_q1;
				pc_out_q3 <= pc_out_q3;
			end
	end

assign s = reset | !(branch^branch_q^branch_q1^jump^jump_q^jump_q1^jr^jr_q^jr_q1);//Solution for stall
assign IMEM_en = s;

endmodule 
