module basic_adder(output [31:0] out, input [31:0] A, input [31:0] B);
	assign out= A+B;
endmodule

// PC+4
module PC_add (output [31:0] out, input [31:0] in);
	assign out=in +4;
endmodule 