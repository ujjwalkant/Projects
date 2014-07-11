`timescale 1ns / 10ps

//rs = read1 & rt = read2
module register_file(read1,read2,data1,data2,wreg,wdata,wr_en,rst);
input [4:0] read1,read2,wreg; //read1, read2 & wreg will carry address
input wr_en,rst; //write enble for write operation
input[31:0] wdata; //data to be written into register
output reg[31:0] data1,data2;//dataout acc to input read1 and read2, it is asynchronous
reg [31:0] REGF [31:0]; //32 register each of 32bits

always @(*)begin
if(rst)begin
data1 = 0;
REGF[0] = 1'b 0;//taking care of zero register
end
else
data1 = REGF[read1];
end

always @(*)begin
if(rst)
data2 = 0;
else
data2 = REGF[read2];
end

always @(*) begin //synchronous write
if(wr_en)begin
REGF[wreg] = wdata;
end

else
begin
REGF[wreg] = REGF[wreg];end
end

//inserting values into registers for testing purpose
initial
begin

REGF[0] = 0; REGF[1] = 1; REGF[2] = 2; REGF[3] = 42; REGF[4] = 4; REGF[5] = 5; REGF[6] = 6;
REGF[7] = 7; REGF[8] = 8; REGF[9] = 9; REGF[10] = 10; REGF[11] = 11; REGF[12] = 12; REGF[13] = 13; 
REGF[14] = 14; REGF[15] = 15; REGF[16] = 16; REGF[17] = 17; REGF[18] = 18; REGF[19] = 19; REGF[20] = 20;
REGF[21] = 21; REGF[22] = 22; REGF[23] = 23; REGF[24] = 24; REGF[25] = 25; REGF[26] = 26; REGF[27] = 27;
REGF[28] = 28; REGF[29] = 29; REGF[30] = 30; REGF[31] = 31;

end
endmodule
