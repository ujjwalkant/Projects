`timescale 1ns / 10ps

module I_Memory(read,data,data1,data2,en,sel_inst,en2);//Instruction Memory
input[31:0] read;//address port--only read port as write port not required
input en,sel_inst;//if en = 1 then read
output reg en2;
output reg[31:0] data,data1,data2;
reg[7:0] IMEM[511:0];//size of data memory is 512 * 8

always @(*)
	begin 
		if(en)
			begin
				data = {IMEM[read+3],IMEM[read+2],IMEM[read+1],IMEM[read]};
			end
		else
			begin
				data = 32'b0;
			end
	end

always @(*)
	begin
		if((!sel_inst) && en && (IMEM[read+3][7:4] == 4'b1000))
			begin
				en2 = 1'b1;
				data1 = {IMEM[read+7],IMEM[read+6],IMEM[read+5],IMEM[read+4]};
				data2 = {IMEM[read+11],IMEM[read+10],IMEM[read+9],IMEM[read+8]};
			end
		else
			begin
				en2 = 1'b0;
				//data1 = 32'b0;
				//data2 = 32'b0;
			end
	end

initial 
begin
//-------------------  Random Test
{IMEM [3],IMEM [2],IMEM [1],IMEM [0]} = 32'b 100011_00001_00011_0000000000000100;// load word operation
{IMEM [7],IMEM [6],IMEM [5],IMEM [4]} = 32'b 000000_00011_00110_01100_00000_100000;//normal add operation
//{IMEM [11],IMEM [10],IMEM [9],IMEM [8]} = 32'b 000000_00000_10110_11100_00000_100000;//normal add operation
{IMEM [11],IMEM [10],IMEM [9],IMEM [8]} = 32'b 000000_00000_10110_11100_00000_100000;//normal add operation
//{IMEM [15],IMEM [14],IMEM [13],IMEM [12]} = 32'b 001000_00001_00101_0000000000000100;// add imm operation
/*
{IMEM [3],IMEM [2],IMEM [1],IMEM [0]} = 32'b 000000_00001_00110_01100_00000_100000;//normal add operation
{IMEM [7],IMEM [6],IMEM [5],IMEM [4]} = 32'b 001000_00001_00101_0000000000000100;// add imm operation
{IMEM [11],IMEM [10],IMEM [9],IMEM [8]} = 32'b 100011_00001_00011_0000000000000100;// load word operation
{IMEM [15],IMEM [14],IMEM [13],IMEM [12]} = 32'b 000100_00011_00011_0000000000000100;//BEQ
{IMEM [35],IMEM [34],IMEM [33],IMEM [32]} = 32'b 000101_00010_00011_0000000000000100;//BNE
{IMEM [55],IMEM [54],IMEM [53],IMEM [52]} = 32'b 100000_00001_00011_0000000000000100;// load byte operation
{IMEM [59],IMEM [58],IMEM [57],IMEM [56]} = 32'b 101011_00001_00011_0000000000001100;// store word operation
{IMEM [63],IMEM [62],IMEM [61],IMEM [60]} = 32'b 001010_00001_00101_0000000000000100;// SLTI
{IMEM [67],IMEM [66],IMEM [65],IMEM [64]} = 32'b 000000_00001_00110_01100_00000_101010;//SLT
{IMEM [71],IMEM [70],IMEM [69],IMEM [68]} = 32'b 000010_00000000000000000000000100;//Jump without jr
{IMEM [19],IMEM [18],IMEM [17],IMEM [16]} = 32'b 101000_00001_00011_0000000000001100;// store byte operation
{IMEM [23],IMEM [22],IMEM [21],IMEM [20]} = 32'b 000011_00000000000000000000010010;//JAL
{IMEM [75],IMEM [74],IMEM [73],IMEM [72]} = 32'b 000000_11111_0000000000_00000_001000;//Jump with jr*/


/*---------------- Test Bench type 1
{IMEM [3],IMEM [2],IMEM [1],IMEM [0]} =32'b000000_01000_01001_10000_00000_100000;//add
{IMEM [7],IMEM [6],IMEM [5],IMEM [4]} =32'b001000_01010_10001_0000000000001111;//addi
{IMEM [11],IMEM [10],IMEM [9],IMEM [8]} =32'b001100_01010_10010_0000000000001010;//andi
{IMEM [15],IMEM [14],IMEM [13],IMEM [12]} =32'b000000_01000_01011_10010_00000_100010;//sub
{IMEM [19],IMEM [18],IMEM [17],IMEM [16]} =32'b100011_00000_10100_0000000000000000;//lw
{IMEM [23],IMEM [22],IMEM [21],IMEM [20]} =32'b101011_00000_01000_0000000000001010;//sw RAW HAZAR
{IMEM [27],IMEM [26],IMEM [25],IMEM [24]} =32'b000010_00000000000000000000001010;//jump
{IMEM [31],IMEM [30],IMEM [29],IMEM [28]} =32'b000000_01001_01010_01000_00000_100000;//add
{IMEM [35],IMEM [34],IMEM [33],IMEM [32]} =32'b000000_01001_01000_01010_00000_100010;//sub
{IMEM [39],IMEM [38],IMEM [37],IMEM [36]} =32'b000101_01000_01001_0000000000000001;//bne
{IMEM [43],IMEM [42],IMEM [41],IMEM [40]} =32'b000000_01100_01101_01011_00000_100010;//sub
{IMEM [47],IMEM [46],IMEM [45],IMEM [44]} =32'b000000_01010_01110_10011_00000_100000;//add
{IMEM [51],IMEM [50],IMEM [49],IMEM [48]} =32'b000000_01011_01000_10100_00000_100000;//add
{IMEM [55],IMEM [54],IMEM [53],IMEM [52]} =32'b000000_10100_01000_10101_00000_100000;//add*/


/*------ test bench 
{IMEM [3],IMEM [2],IMEM [1],IMEM [0]} =	  32'b000000_01000_01001_10000_00000_100000;
{IMEM [7],IMEM [6],IMEM [5],IMEM [4]} =     32'b001000_01010_10001_0000000000001111;  
{IMEM [11],IMEM [10],IMEM [9],IMEM [8]} =   32'b001100_01010_10010_0000000000001010;  
{IMEM [15],IMEM [14],IMEM [13],IMEM [12]} = 32'b000000_01000_01011_1001000000100010;  
{IMEM [19],IMEM [18],IMEM [17],IMEM [16]} = 32'b100011_00000_10100_0000000000000001;  
{IMEM [23],IMEM [22],IMEM [21],IMEM [20]} = 32'b101011_01010_01000_0000000000000000;  
{IMEM [27],IMEM [26],IMEM [25],IMEM [24]} = 32'b000010_00000_00000_0000000000001001; 
{IMEM [31],IMEM [30],IMEM [29],IMEM [28]} = 32'b000000_01001_01010_01000_00000_100000;
{IMEM [35],IMEM [34],IMEM [33],IMEM [32]} = 32'b000000_01001_01000_01010_00000_100010;
{IMEM [39],IMEM [38],IMEM [37],IMEM [36]} = 32'b000101_01000_01001_000000000000000001;
{IMEM [43],IMEM [42],IMEM [41],IMEM [40]} = 32'b000000_01100_01101_01011_00000_100010;
{IMEM [47],IMEM [46],IMEM [45],IMEM [44]} = 32'b000000_01010_01110_10011_00000_100000;
{IMEM [51],IMEM [50],IMEM [49],IMEM [48]} = 32'b000000_10011_01000_10100_00000_100000;
{IMEM [55],IMEM [54],IMEM [53],IMEM [52]} = 32'b000000_10100_01000_10101_00000_100000;*/


//------------Fibonacci Series
/*
{IMEM [3],IMEM [2],IMEM [1],IMEM [0]} =	  32'h8c030000; 
{IMEM [7],IMEM [6],IMEM [5],IMEM [4]} =     32'h8c040001; 	
{IMEM [11],IMEM [10],IMEM [9],IMEM [8]} =   32'h8c050002; 	
{IMEM [15],IMEM [14],IMEM [13],IMEM [12]} = 32'h8c010002; 	
{IMEM [19],IMEM [18],IMEM [17],IMEM [16]} = 32'h10600004; 	
{IMEM [23],IMEM [22],IMEM [21],IMEM [20]} = 32'h00852020; 	
{IMEM [27],IMEM [26],IMEM [25],IMEM [24]} = 32'h00852822; 	
{IMEM [31],IMEM [30],IMEM [29],IMEM [28]} = 32'h00611820; 	
{IMEM [35],IMEM [34],IMEM [33],IMEM [32]} = 32'h1000fffb; 	
{IMEM [39],IMEM [38],IMEM [37],IMEM [36]} = 32'hac040006;
*/

end
endmodule
