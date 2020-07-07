module mux_3d_32bit (output [31:0] out,input [31:0] A, B, C, input [1:0] sel);
	assign out = sel[1] ? C : (sel[0] ? B : A) ;
endmodule

module mux_2_5bit (output [4:0] out, input [4:0] A, B, input sel);
	assign out = sel ? B : A;
endmodule

module mux2to1(output out,input sel,in1,in2);
	wire not_sel,a1,a2;
	not (not_sel,sel);
	and (a1,sel,in2);
	and (a2,not_sel,in1);
	or(out,a1,a2);
endmodule

module mux2_32bit(output [31:0] b32out, input [31:0] b32in1,b32in2, input b32sel);
	genvar j;
	generate 
		for(j=0;j<32;j=j+1) 
		begin
			mux2to1 m0(b32out[j],b32sel,b32in1[j],b32in2[j]);
		end
	endgenerate
endmodule
