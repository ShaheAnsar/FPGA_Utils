module register_file(
	input clk, // CPU clock
	input n_rst, // Active low reset
	input[4:0] reg_addr_w, // Register address for writes
	input [31:0] reg_data_w, // Input data (Write to reg file)
	input[4:0] reg_addr_r[1:0], // Register address for reads (2 ports)
	output reg [31:0] reg_data_r[1:0], // Output data (Read from reg file)
);
	reg [31:0] registers [31:0]; // There are 32 registers

	assign odata = registers[reg_addr];

	always @(posedge clk) begin
		if(!n_rst) begin
			registers[0] <= 32'd0;
			registers[1] <= 32'd0;
			registers[2] <= 32'd0;
			registers[3] <= 32'd0;
			registers[4] <= 32'd0;
			registers[5] <= 32'd0;
			registers[6] <= 32'd0;
			registers[7] <= 32'd0;
			registers[8] <= 32'd0;
			registers[9] <= 32'd0;
			registers[10] <= 32'd0;
			registers[11] <= 32'd0;
			registers[12] <= 32'd0;
			registers[13] <= 32'd0;
			registers[14] <= 32'd0;
			registers[15] <= 32'd0;
			registers[16] <= 32'd0;
			registers[17] <= 32'd0;
			registers[18] <= 32'd0;
			registers[19] <= 32'd0;
			registers[20] <= 32'd0;
			registers[21] <= 32'd0;
			registers[22] <= 32'd0;
			registers[23] <= 32'd0;
			registers[24] <= 32'd0;
			registers[25] <= 32'd0;
			registers[26] <= 32'd0;
			registers[27] <= 32'd0;
			registers[28] <= 32'd0;
			registers[29] <= 32'd0;
			registers[30] <= 32'd0;
			registers[31] <= 32'd0;
		end

		else begin
			reg_data_r[0] <= registers[reg_addr_r[0]];
			reg_data_r[1] <= registers[reg_addr_r[1]];
			registers[reg_addr_w] <= reg_data_w;
		end
	end
endmodule

module program_counter(
	input clk,
	input n_rst,
	input[31:0] add_offset, // Offset added to the program counter every clock
	output [31:0] value
);
	reg[31:0] pc_reg;
	assign value = pc_reg;
	always @(posedge clk) begin
		if(!n_rst) begin
			pc_reg <= 0;
		end else begin
			pc_reg <= pc_reg + add_offset;
		end
	end
endmodule

module alu(
	input[3:0] op, // Actual operation, funct3 of opcode (Bits 12:14) and an additional bit (the MSB) for diff between SRL and SRA and SUM/SUB
	input[31:0] operand1,
	input[31:0] operand2,
	output[31:0] result,
	output zero,
	output negative,
);
	localparam OP_SUM = 4'h0;
	localparam OP_DIFF = 4'h8;
	localparam OP_SLT = 4'h2;
	localparam OP_SLTU = 4'h3;
	localparam OP_AND = 4'h7;
	localparam OP_OR = 4'h6;
	localparam OP_XOR = 4'h4;
	localparam OP_SLL = 4'h1;
	localparam OP_SRL = 4'h5;
	localparam OP_SRA = 4'd14;
	wire[31:0] _sum;
	wire[31:0] _diff;
	//wire[31:0] neg;
	wire[31:0] _and;
	wire[31:0] _or;
	wire[31:0] _xor;
	wire[31:0] _slt; // Set less than
	wire[31:0] _sltu; // Set less than unsigned
	wire[31:0] _sll; // Shift left logical
	wire[31:0] _srl; // Shift right logical
	wire[31:0] _sra; // Shift right arithmetic
	assign _sum = operand1 + operand2;
	assign _diff = operand1 - operand2;
	assign _and = operand1 & operand2;
	assign _or = operand1 | operand2;
	assign _xor = operand1 ^ operand2;
	assign _slt = ( operand1 > $signed(operand2) )? 1:0;
	assign _sltu = ( operand1 > operand2 ) ? 1 : 0;
	assign _srl = operand1 >> operand2;
	assign _sll = operand1 << operand2;
	assign _sra = operand1 >>> operand2;
	assign zero = result == 0;
	assign negative = result[31];
	always @(*) begin
		case(op)
			OP_SUM:
				result = _sum;
			OP_DIFF:
				result = _diff;
			OP_SLT:
				result = _slt;
			OP_SLTU:
				result = _sltu;
			OP_AND:
				result = _and;
			OP_OR:
				result = _or;
			OP_XOR:
				result = _xor;
			OP_SLL:
				result = _sll;
			OP_SRL:
				result = _srl;
			OP_SRA:
				result = _sra;
		endcase
	end
endmodule 

module cu();
endmodule

module instr_decode(
	input[6:0] opcode,
	input[2:0] funct3,
	input[6:0] funct7,
	output reg_read,
	output reg_write,
	
);

endmodule

module cpu(
	input clk, 
	input n_rst,
	output[31:0] iram_addr_bus,
	input[31:0] iram_data,
	output[31:0] dram_addr_bus,
	inout[31:0] dram_data,
	output dram_write, // 1 if write, 0 if read
	input int, // Interrupt pin, unused for now
);

	// Required components
	wire[31:0] pc_add_offset;
	wire[31:0] curr_pc;
	program_counter pc(
		.clk(clk), .n_rst(n_rst),
		.add_offset(pc_add_offset),
		.value(curr_pc)
	);
	
	wire[4:0] reg_addr_w;
	wire[31:0] reg_write_data;
	
	wire[4:0] reg_addr_r[1:0];
	wire[4:0] reg_data_r[1:0];
	register_file rf0(
		.clk(clk), .n_rst(n_rst),
		.reg_addr_w(reg_addr_w),
		.reg_write_data(reg_write_data),
		.reg_addr_r(reg_addr_r),
		.reg_data_r(reg_data_r)
	);
	
	wire[4:0] alu_operation;
	wire[31:0] alu_operand1;
	wire[31:0] alu_operand2;
	wire[31:0] alu_result;
	alu alu0(
		.clk(clk), .n_rst(n_rst),
		.op(alu_operation),
		.operand1(alu_operand1),
		.operand2(alu_operand2),
		.result(alu_result) //Ignoring the zero and negative flag
	);

	localparam INSTR_ARITH_IMM = 7'b0010011;	
	localparam INSTR_ARITH_REG = 7'b0110011;
	localparam INSTR_LOAD = 7'b0110111;
	localparam INSTR_ADDR = 7'b0010111;
	localparam INSTR_JAL = 7'b1101111;
	localparam INSTR_JALR = 7'b1100111;
	reg[31:0] instr;
	wire[6:0] instr_opcode;
	wire[2:0] instr_funct3;
	wire[5:0] instr_rs1;
	wire[5:0] instr_rs2;
	wire[5:0] instr_rd;
	wire[12:0] instr_imm12;
	wire[4:0] instr_uimm5;
	wire[6:0] instr_funct7;
	wire[19:0] instr_imm20;

	assign instr_opcode = instr[6:0];
	assign instr_funct3 = instr[14:12];
	assign instr_rs1 = instr[19:15];
	assign instr_rd = instr[11:7];
	assign instr_imm12 = instr[31:20];
	assign instr_funct7 = instr[31:25];
	assign instr_uimm5 = instr[24:20];
	assign instr_imm20 = instr[31:12];
	assign instr_rs2 = instr[24:20];
	//assign instr_
	reg instr_valid;
	reg instr_consumed;
	assign iram_addr_bus = curr_pc; // Assuming a simple SRAM style RAM
	always @(posedge clk) begin
		if(!n_rst) begin
			instr <= 0;
			instr_valid <= 0;
		end else begin 
			// Instruction fetch
			instr <= iram_data;
			// Instruction decode

			// Instruction exec
		end
	end
endmodule
//module instr_fetch(
//	input clk,
//	input n_rst,
//	input[31:0] pc_value,
//	output[31:0] instr,
//	output[31:0] iram_addr_bus, // IRAM address bus
//	input[31:0] iram_data, // IRAM data bus
//);
//	assign iram_addr_bus = pc_value;
//	always @(posedge clk) begin
//		if(!n_rst) begin
//			
//		end
//	end
//endmodule



