`timescale 1ns / 10ps

module D_Memory(read,write,wdata,data,en,byte_en);//Data Memory byte addressable
input[31:0] read,write;//address ports
input byte_en,en;//if en = 0 then read else write
input[31:0] wdata;
output reg[31:0] data;
reg[7:0] DMEM[511:0];//size of data memory is 512 * 8

always @(*)begin 
if(en && byte_en)//store byte
DMEM[write] = wdata[7:0];
else if(en && !byte_en)//store word

{DMEM[write+3],DMEM[write+2],DMEM[write+1],DMEM[write]} = wdata;
else if(!en && byte_en)//load byte

data = {24'b0,DMEM[read]} ;
else if(!en && !byte_en)//load word
data = {DMEM[read+3],DMEM[read+2],DMEM[read+1],DMEM[read]};
else
data = 32'bz;
end

integer i;//writing values to memory for testing purpose

initial
begin
for (i = 0; i < 512; i = i +1) begin
DMEM[i] = i;
end end

always @(*)begin
$display("value in memory %b",DMEM[write]);
$display("value in memory %b",DMEM[write+1]);
$display("value in memory %b",DMEM[write+2]);
$display("value in memory %b",DMEM[write+3]);end

endmodule
