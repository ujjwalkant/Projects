
`timescale 1ns / 10ps

module ALU(Opcode,Operand1,Operand,Immediate,Output,Zero,Overflow,Funct,selB,selA,data_out1,alu_op_q);
input [31:0] Operand1,Operand,Immediate;
input [5:0] Opcode,Funct;
input [1:0] selA,selB;
input [31:0] data_out1,alu_op_q;
output [31:0] Output;
output Zero,Overflow;
wire IM;
wire [31:0] Operand2;
wire [5:0] Select;
wire [3:0] Control;
wire [31:0] Output1,operand2_f,operand1_f;
wire Zero1,Overflow1,Op;

assign operand1_f = (selA == 2'b10) ? data_out1 : ((selA == 2'b11) ? alu_op_q : Operand1);
assign operand2_f = (selB == 2'b10) ? data_out1 : ((selB == 2'b11) ? alu_op_q : Operand2);

assign Op = (|Opcode);

assign Select = (|Opcode) ? Opcode : Funct;

ControlALU N1(.Select(Select),.Control(Control),.IM(IM),.Op(Op));

assign Operand2 = IM ? Immediate : Operand;

SubALU N2(.Control(Control),.Operand1(operand1_f),.Operand2(operand2_f),.Output(Output1),.Zero(Zero1),.Overflow(Overflow1));

assign Output = Output1;
assign Zero = Zero1;
assign Overflow = Overflow1;

endmodule
