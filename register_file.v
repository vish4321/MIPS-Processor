module register_file (output reg [31:0] readreg1, readreg2, input [4:0]  rs, rt, rd,input [31:0] write_data, input write_enable, clk, rst);

	always @ (posedge rst)
		$readmemh("register_file.txt", reg_file);
	
	reg [31:0] reg_file [0:31];
	
	always @(posedge clk)
		begin
			if(write_enable) reg_file[rd] = write_data;
			readreg1 = reg_file[rs];
			readreg2 = reg_file[rt];
		end
endmodule 