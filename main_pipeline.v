//NIHAL SINGH and VISHNU VENKATESH

/*

[25-21] : rs
[20-16] : rt
[15-11] : rd


IF_ID register
IF_ID.IR


ID_EX register
id_ex__WB
id_ex__M
id_ex__EX
ID_EX.A
ID_EX.B
ID_EX.IR
ID_EX.IMM
ID_EX.RT
ID_EX.RD

EX_MEM register
ex_mem__WB
ex_mem__M
EX_MEM.IR
EX_MEM.ALUOutput
EX_MEM.B
EX_MEM.RegDst

MEM_WB register
mem_wb__WB
MEM_WB.ReadData
MEM_WB.Address
MEM_WB.RegDst

*/

`include "pc.v"
`include "memory.v"
`include "muxes.v"
`include "register_file.v"
`include "adder.v"
`include "alu.v"


module shift(output reg [31:0] out, input [31:0] in);
	always@(*)
		out = in<<2;
endmodule

module instr_dec (out, in);

	input [5:0]  in;
	output [8:0] out;
	
	assign out = (in == 6'b000000) ? 9'b100001100 : 9'b001000010;
endmodule

module sign_extend (output [31:0] out, input [15:0] in);	
	assign out = in[15] ? {16'hFFFF, in} : {16'h0000, in};
endmodule


//IF/ID stage
module if_id (output reg [31:0] next_stageA, output reg [31:0] next_stageB, input [31:0] current_stageA, input [31:0] current_stageB, input clk, input clear_regs, input reset, pipeline_flag);

always @(posedge clk)
begin
if(pipeline_flag)
begin
	next_stageB <= #1 current_stageA;
	next_stageA <= #1 current_stageB;
end
if (clear_regs)
begin
	next_stageA <= #1 32'b0;
	next_stageB <= #1 32'b0;
end
else 
begin 
	next_stageA <= #1 current_stageA;
	next_stageB <= #1 current_stageB;
end
end

always @(posedge reset)
begin 
	next_stageA <= #1 32'b0;
	next_stageB <= #1 32'b0;
end

endmodule 


// ID/EX stage

module id_ex (output reg [1:0] next_stageA, output reg [2:0] next_stageB, output reg [3:0] next_stageC, output reg [31:0] next_stageD, next_stageE, next_stageF, next_stageG, output reg [4:0] next_stageH, 
				next_stageI, next_stageJ, input [8:0] inDec, input [31:0] current_stageD, current_stageE, current_stageF, current_stageG, input [4:0] current_stageH, current_stageI, current_stageJ, input clear_regs, clk, reset, pipeline_flag);
	
always @(posedge clk)
begin
	
if (clear_regs) 
begin
	next_stageA <= #1 0;
	next_stageB <= #1 0;
	next_stageC <= #1 0;
	next_stageD <= #1 0;
	next_stageE <= #1 0;
	next_stageF <= #1 0;
	next_stageG <= #1 0;
	next_stageH <= #1 0;
	next_stageI <= #1 0;
	next_stageJ <= #1 0;
end
else
begin
	{next_stageA, next_stageB, next_stageC} = #1 inDec;
	next_stageD <= #1 current_stageD;
	next_stageE <= #1 current_stageE;
	next_stageF <= #1 current_stageF;
	next_stageG <= #1 current_stageG;
	next_stageH <= #1 current_stageH;
	next_stageI <= #1 current_stageI;
	next_stageJ <= #1 current_stageJ;
end
//debug
if(pipeline_flag)
begin
	next_stageA <= #1 1;
	next_stageB <= #1 1;
	next_stageC <= #1 1;
	next_stageD <= #1 1;
	next_stageE <= #1 1;
	next_stageF <= #1 1;
	next_stageG <= #1 1;
	next_stageH <= #1 1;
	next_stageI <= #1 1;
	next_stageJ <= #1 1;
end

end

always @(posedge reset)
begin
	next_stageA <= #1 0;
	next_stageB <= #1 0;
	next_stageC <= #1 0;
	next_stageD <= #1 0;
	next_stageE <= #1 0;
	next_stageF <= #1 0;
	next_stageG <= #1 0;
	next_stageH <= #1 0;
	next_stageI <= #1 0;
end

endmodule

//EX/MEM stage
module ex_mem (output reg [1:0] next_stageA, output reg [2:0] next_stageB, output reg [31:0] next_stageC, output reg next_stageD, output reg [31:0] next_stageE, output reg [31:0] next_stageF, 
				output reg [4:0] next_stageG, input [1:0] current_stageA, input [2:0] current_stageB, input [31:0] current_stageC, input current_stageD, 
				input [31:0] current_stageE, input [31:0] current_stageF, input [4:0] current_stageG, input clk, input clear_regs, input reset);
		
always @(posedge clk)
begin
if (clear_regs)
begin
	next_stageA <= #1 0;
	next_stageB <= #1 0;
	next_stageC <= #1 0;
	next_stageD <= #1 0;
	next_stageE <= #1 0;
	next_stageF <= #1 0;
	next_stageG <= #1 0;
end
else
begin
	next_stageA <= #1 current_stageA;
	next_stageB <= #1 current_stageB;
	next_stageC <= #1 current_stageC;
	next_stageD <= #1 current_stageD;
	next_stageE <= #1 current_stageE;
	next_stageF <= #1 current_stageF;
	next_stageG <= #1 current_stageG;
end
end

always @(posedge reset)
begin
	next_stageA <= #1 0;
	next_stageB <= #1 0;
	next_stageC <= #1 0;
	next_stageD <= #1 0;
	next_stageE <= #1 0;
	next_stageF <= #1 0;
	next_stageG <= #1 0;			
end
	
		
endmodule 


//MEM/WB stage

module mem_wb (output reg [1:0] next_stageA, output reg [31:0] next_stageB, next_stageC, output reg [4:0] next_stageD, input [1:0] current_stageA, input [31:0] current_stageB, current_stageC, input [4:0] current_stageD, 
				input clk, reset);
	
always @(posedge clk)
begin
	next_stageA <= #1 current_stageA;
	next_stageB <= #1 current_stageB;
	next_stageC <= #1 current_stageC;
	next_stageD <= #1 current_stageD;
end

always @(posedge reset)
begin
	next_stageA <= #1 0;
	next_stageB <= #1 0;
	next_stageC <= #1 0;
	next_stageD <= #1 0;
end
	
endmodule

//Forwarding Unit for Data Hazard Handling

module f_unit (output reg [1:0] next_stageA, next_stageB, input [4:0] in1, in2, input regwr1, regwr2, input [4:0] rs, rt, input clk, pipeline_flag);
	
always @(negedge clk)
begin
if (regwr1 == 1 && regwr2 == 1)
begin
if(in1 == in2)
begin
if (in1 == rs && in1 == rt) 
begin 
	next_stageA = 2'b10;					
	next_stageB = 2'b01;
end
else if (in1 == rs && in1 != rt)
begin
	next_stageA = 2'b10;					
	next_stageB = 2'b00;
end
else if (in1 != rs && in1 == rt)
begin
	next_stageA = 2'b00;					
	next_stageB = 2'b01;
end
else if (pipeline_flag)
begin
	next_stageA = 2'b11;					
	next_stageB = 2'b11;
end
else
begin
	next_stageA = 2'b00;					
	next_stageB = 2'b00;
end
end
else 
begin
if ({rs, rt} == {in1, in1})
begin
	next_stageA = 2'b10;					
	next_stageB = 2'b01;
end
else if ({rs,rt} == {in2, in2})
begin
	next_stageA = 2'b01;					
	next_stageB = 2'b10;
end
else if ({rs,rt} == {in1, in2})
begin
	next_stageA = 2'b10;					
	next_stageB = 2'b10;
end	
else if ({rs,rt} == {in2, in1})
begin
	next_stageA = 2'b01;					
	next_stageB = 2'b01;
end
else if (rs == in1)
begin
	next_stageA = 2'b10;					
	next_stageB = 2'b00;
end
else if (rs == in2)
begin
	next_stageA = 2'b01;					
	next_stageB = 2'b00;
end
else if (rt == in1)
begin
	next_stageA = 2'b00;					
	next_stageB = 2'b01;
end
else if (rt == in2)
begin
	next_stageA = 2'b00;					
	next_stageB = 2'b10;
end
else if (pipeline_flag == 1)
begin
	next_stageA = 2'b11;					
	next_stageB = 2'b11;
end
else 
begin
	next_stageA = 2'b00;					
	next_stageB = 2'b00;
end
end					
end


else if (regwr1 == 0 && regwr2 == 1)
begin
if (in2 == rs && in2 == rt) 
begin 
	next_stageA = 2'b01;					
	next_stageB = 2'b10;
end
else if (in2 == rs && in2 != rt)
begin
	next_stageA = 2'b01;					
	next_stageB = 2'b00;
end
else if (in2 != rs && in2 == rt)
begin
	next_stageA = 2'b00;					
	next_stageB = 2'b10;
end
else
begin
	next_stageA = 2'b00;					
	next_stageB = 2'b00;
end
end


else if (regwr1 == 1 && regwr2 == 0)
begin
if (in1 == rs && in1 == rt) 
begin 
	next_stageA = 2'b10;					
	next_stageB = 2'b01;
end
else if (in1 == rs && in1 != rt)
begin
	next_stageA = 2'b10;					
	next_stageB = 2'b00;
end
else if (in1 != rs && in1 == rt)
begin
	next_stageA = 2'b00;					
	next_stageB = 2'b01;
end
else
begin
	next_stageA = 2'b00;					
	next_stageB = 2'b00;
end
end
else	
begin
	next_stageA = 2'b00;					
	next_stageB = 2'b00;
end

end

endmodule



module main_pipeline (reset, clk);

	input reset, clk;
	wire [1:0] fB, wb1, wb, fA, wb2;
	wire [2:0] mem, alu_sel, mem1;
	wire [3:0] ex;
	reg pipeline_flag;
	wire [31:0] PC_relative_ex, write_data, instr_id_sign_extended, PC_plus_4_id, PC_in, PC_out, instr_if, PC_plus_4_if, instr_id, regfile_readreg1, regfile_readreg2, w1, w2, w3, instr_ex_sign_extended, w4, PC_relative_mem, w5, w6, g1, g2, g3, g4, g5, g6, g7;
	wire [8:0] dec_instr;
	wire [4:0] dstReg, y1, y2, y3, y4, y5;
	initial pipeline_flag = 0;
	wire branch_decision, zero, zero1;
	
	//IF
	mux2_32bit mux2b1 (PC_in, PC_plus_4_if, PC_relative_ex, branch_decision);
	PC pc_instance (PC_out, PC_in, clk, reset);
	instruction_memory imem (instr_if, PC_out, reset);
	PC_add add4forPC (PC_plus_4_if, PC_out);
	
	//IF-ID
	if_id IFID(PC_plus_4_id, instr_id, PC_plus_4_if, instr_if, clk, branch_decision, reset, pipeline_flag);
	
	//ID
	register_file regfile(regfile_readreg1, regfile_readreg2, instr_id[25:21], instr_id[20:16], dstReg, write_data, wb2[1], clk, reset);
	sign_extend sign_ext(instr_id_sign_extended, instr_id[15:0]);
	instr_dec dec(dec_instr, instr_id[31:26]);
	
	//ID-EX
	id_ex IDEX(wb, mem, ex, w1, w2, w3, instr_ex_sign_extended, y1, y2, y5, dec_instr, PC_plus_4_id, regfile_readreg1, regfile_readreg2, instr_id_sign_extended, instr_id[20:16], instr_id[15:11], instr_id[25:21], branch_decision, clk, reset, pipeline_flag);
	
	//EX
	shift shift_unit(w4, instr_ex_sign_extended);
	basic_adder BA(PC_relative_mem, w1, w4);
	mux_2_5bit mux5b1(y3, y1, y2, ex[3]);
	mux2_32bit mux2b2(w5, w3, instr_ex_sign_extended, ex[0]);
	cntrl_alu Alucntrl (alu_sel, ex[2:1], instr_id_sign_extended[5:0]);
	mux_3d_32bit mux3b1(w6, w2, write_data, g2, fA);  
	mux_3d_32bit mux3b2(g1, w5, g2, write_data, fB);
	alu_main ALU(g4, w6, g1, alu_sel, zero);
	
	//EX-MEM
	ex_mem EXMEM(wb1, mem1, PC_relative_ex, zero1, g2, g5, y4, wb, mem, PC_relative_mem, zero, g4, w3, y3, clk, branch_decision, reset);
	
	//MEM
	and branch_check (branch_decision, zero1, mem1[2]);
	data_memory data_mem(g3, g5, g2, mem1[0], mem1[1], clk);
	
	//MEM-WB
	mem_wb MEMWB(wb2, g6, g7, dstReg, wb1, g3, g2, y4, clk, reset);
	
	//WB
	mux2_32bit m2b3(write_data, g7, g6, wb2[0]);
	
	//Forwarding
	f_unit forward_unit(fA, fB, y4, dstReg, wb1[1], wb2[1], y5, y1, clk, pipeline_flag);
endmodule

module main_pipeline_tb;

	reg clk, reset;
	
	main_pipeline processor(reset, clk);
	
	initial 
	begin
		$monitor($time, "clk = %b", clk);
		$dumpfile("main_pipeline.vcd");
		$dumpvars(0, main_pipeline_tb);
		#0 clk = 1'b0; reset = 1'b0;
		#10 reset = 1'b1;
		#2 reset = 1'b0;
		#6 clk = 1'b1;
		forever #5 clk = ~clk;
	end
	
	initial #150 $finish;
		

endmodule