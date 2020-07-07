// Module for ALU control

module cntrl_alu(output [2:0] operation, input [1:0] ALUop, input [5:0] funct);
	
	assign operation[2] = (~ALUop[1] & ALUop[0]) | (ALUop[1] & ~funct[3] & ~funct[2] & funct[1] & ~funct[0]) | (ALUop[1] & funct[3] & ~funct[2] & funct[1] & ~funct[0]) ? 1 : 0;
	assign operation[1] = (~ALUop[1] & ALUop[0]) | (~ALUop[1] & ~ALUop[0]) | (ALUop[1] & ~funct[3] & ~funct[2] & ~funct[1] & ~funct[0]) | (ALUop[1] & ~funct[3] & ~funct[2] & funct[1] & ~funct[0]| (ALUop[1] & funct[3] & ~funct[2] & funct[1] & ~funct[0])) ? 1 : 0;
	assign operation[0] = (ALUop[1] & ~funct[3] & funct[2] & ~funct[1] & funct[0]) | (ALUop[1] & funct[3] & ~funct[2] & funct[1] & ~funct[0]) ? 1 : 0;
	
endmodule

/* module tbALU();

reg Binvert, Carryin; 
reg [2:0] Operation; 
reg [31:0] a,b;
wire [31:0] Result;
wire CarryOut;

ALU alu1(a,b,Operation,Result,CarryOut);

initial begin 


$monitor($time," a=%b, b=%b, Result=%b", a, b, Result);

a=32'h0000FFFF; 
b=32'h00000001; 

Operation=3'b000; 
Carryin=1'b0; //must perform AND resulting in zero
#100 Operation=3'b001; //OR
#100 Operation=3'b010; //ADD
#100 Operation=3'b110; //SUB
#200 $finish;

end 

initial begin
 $dumpfile("ALU.vcd"); 
 $dumpvars;           
end

endmodule */

module alu_main (output reg [31:0] result, input [31:0] inpA, input [31:0] inpB, input [2:0] select_sig, output reg zero);
	always @(*)
	begin
		case (select_sig)
		3'b110: result= (inpA-inpB);
		3'b000: result= (inpA&inpB);
		3'b010: result= (inpA+inpB);
		3'b001: result= (inpA|inpB);
		
		default: result = 32'b0;		
		endcase
		if(result == 32'b0) 
			zero = 1;
		else 
			zero = 0;
	end
endmodule

