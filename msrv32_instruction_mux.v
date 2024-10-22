module msrv32_instruction_mux (
    input flush_in,                   // Signal to flush the instruction
    input [31:0] instr_in,           // Input instruction
              
    output [6:0] opcode_out,         // Output opcode
    output [6:0] funct7_out,         // Output funct7
    output [2:0] funct3_out,         // Output funct3
    output [4:0] rs1_addr_out,       // Output rs1 address
    output [4:0] rs2_addr_out,       // Output rs2 address
    output [4:0] rd_addr_out,        // Output rd address
    output [11:0] csr_addr_out,      // Output CSR address
    output [24:0] instr_31_7_out     // Output bits 31 to 7 of instruction
);

wire [31:0] instr_mux;

// Select between the input instruction and a default instruction (0x13) based on flush_in
assign instr_mux = flush_in ? 32'h00000013 : instr_in;

// Assign output signals based on the selected instruction
assign opcode_out = instr_mux[6:0];
assign funct3_out = instr_mux[14:12];
assign funct7_out = instr_mux[31:25];
assign csr_addr_out = instr_mux[31:20];
assign rs1_addr_out = instr_mux[19:15];
assign rs2_addr_out = instr_mux[24:20];
assign rd_addr_out = instr_mux[11:7];
assign instr_31_7_out = instr_mux[31:7];

endmodule
