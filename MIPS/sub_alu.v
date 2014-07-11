`timescale 1ns / 10ps

module SubALU(Control,Operand1,Operand2,Output,Zero,Overflow);
input [31:0] Operand1,Operand2;
output reg[31:0] Output;
output Zero,Overflow;//status flags
reg a;
input [3:0] Control;

always @(*)begin
case (Control)

0: {a,Output} = Operand1 + Operand2; //ADD
1: {a,Output} = Operand1 - Operand2; //SUB
2: {a,Output} = Operand1 & Operand2; //AND
3: {a,Output} = Operand1 | Operand2; //OR
4: {a,Output} = Operand1 ^ Operand2; //XOR
5: {a,Output} = Operand1 < Operand2 ? 1: 0;//Set on less than 
6: {a,Output} = {1 'b0,Output}; //NOP
7: {a,Output} = Operand1 == Operand2;
default: {a,Output} = 0;

endcase
end
assign Zero = (Output == 0);
assign Overflow = (a == 1);

endmodule
