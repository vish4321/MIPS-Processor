//Program Counter

module PC(output reg [31:0] PCout,input [31:0] PCin, input clk, rst);
	
	initial PCout = 32'b0;
	
	always @(posedge clk)
		begin
			PCout = PCin;
		end
	
endmodule 