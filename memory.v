// Data Memory

module data_memory (output [31:0] out,input [31:0] data, addr, input write_enable, read_enable, clk);
	
	reg [7:0] mem [0:24];
	
	assign out = read_enable ? {mem[addr+3], mem[addr+2], mem[addr+1], mem[addr]} : 32'b0;
	
	always @(posedge clk)
		begin
			
			if (write_enable) {mem[addr+3], mem[addr+2], mem[addr+1], mem[addr]} = data;
			
		end

endmodule

//Instruction Memory

module instruction_memory (output [31:0] instr, input [31:0] addr, input rst);

	reg [7:0] mem [0:23];
	
	assign #1 instr = {mem[addr+3], mem[addr+2], mem[addr+1], mem[addr]};
	
	always @ (posedge rst)
		begin
			$readmemh("instructions_hex.txt", mem);
		end
		
endmodule