`timescale 1ns / 10ps

module ControlALU(Select,Control,IM,Op);
input [5:0] Select;
input Op;
output reg [3:0]Control;
output reg IM;

always @(*)begin
case({Select,Op})

64: {Control,IM} = 5'b00000;//ADD(control=0)
68: {Control,IM} = 5'b00010;//SUB(control=1)
72: {Control,IM} = 5'b00100;//AND(control=2)
74: {Control,IM} = 5'b00110;//OR(control=3)
76: {Control,IM} = 5'b01000;//XOR(control=4)
84: {Control,IM} = 5'b01010;//SLT(control=5)
17: {Control,IM} = 5'b00001;//ADDI(control=0,IM=1)
25: {Control,IM} = 5'b00101;//ANDI(control)
27: {Control,IM} = 5'b00111;//ORI
29: {Control,IM} = 5'b01001;//XORI
21: {Control,IM} = 5'b01011;//SLTI
1:  {Control,IM} = 5'b01100;//NOP
9:  {Control,IM} = 5'b00010;//BEQ
11: {Control,IM} = 5'b01110;//BNE
71: {Control,IM} = 5'b00001;//Load Word
65: {Control,IM} = 5'b00001;//Load Byte
81: {Control,IM} = 5'b00001;//Store Byte
87: {Control,IM} = 5'b00001;//Store Word
default: Control = 15;
endcase
end

endmodule
