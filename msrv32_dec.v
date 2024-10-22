module msrv32_dec (
    input [6:0] opcode_in,          // Opcode input
    input funct7_5_in,              // 5th bit of funct7
    input [2:0] funct3_in,          // Function code (funct3)
    input [1:0] iadder_1_to_0_in,   // Address input for alignment checks
    input trap_taken_in,             // Trap signal input
    
    output [3:0] alu_opcode_out,     // ALU operation code output
    output mem_wr_req_out,           // Memory write request output
    output [1:0] load_size_out,      // Load size output
    output load_unsigned_out,         // Load unsigned output
    output alu_src_out,              // ALU source selection output
    output iadder_src_out,           // I-adder source selection output
    output csr_wr_en_out,            // CSR write enable output
    output rf_wr_en_out,             // Register file write enable output
    output [2:0] wb_mux_sel_out,     // Write-back MUX selection output
    output [2:0] imm_type_out,       // Immediate type output
    output [2:0] csr_op_out,         // CSR operation output
    output illegal_instr_out,        // Illegal instruction output
    output misaligned_load_out,      // Misaligned load output
    output misaligned_store_out       // Misaligned store output
);

// Opcode parameters for different instruction types
parameter OPCODE_OP =       5'b01100;
parameter OPCODE_OP_IMM =   5'b00100;
parameter OPCODE_LOAD =     5'b00000;
parameter OPCODE_STORE =    5'b01000;
parameter OPCODE_BRANCH =   5'b11000;
parameter OPCODE_JAL =      5'b11011;
parameter OPCODE_JALR =     5'b11001;
parameter OPCODE_LUI =      5'b01101;
parameter OPCODE_AUIPC =    5'b00101;
parameter OPCODE_MISC_MEM = 5'b00011;
parameter OPCODE_SYSTEM =    5'b11100;

// Function code parameters
parameter FUNCT3_ADD =  3'b000;
parameter FUNCT3_SUB =  3'b000; // NOTE: SUB not used directly in decoding
parameter FUNCT3_SLT =  3'b010;
parameter FUNCT3_SLTU = 3'b011;
parameter FUNCT3_AND =  3'b111;
parameter FUNCT3_OR =   3'b110;
parameter FUNCT3_XOR =  3'b100;
parameter FUNCT3_SLL =  3'b001;
parameter FUNCT3_SRL =  3'b101;
parameter FUNCT3_SRA =  3'b101; // NOTE: SRA is covered by SRL logic

// Internal flags for instruction types
reg is_branch, is_jal, is_jalr, is_auipc, is_lui;
reg is_load, is_store, is_system;
reg is_op, is_op_imm, is_misc_mem;
reg is_addi, is_slti, is_sltiu, is_andi, is_ori, is_xori;
wire is_csr;
wire is_implemented_instr;
wire mal_word, mal_half, misaligned;

// Decoding the opcode
always @* begin
    case(opcode_in[6:2])
        OPCODE_OP       : {is_op, is_op_imm, is_load, is_store, is_branch, is_jal, is_jalr, is_lui, is_auipc, is_misc_mem, is_system} = 11'b10000000000;
        OPCODE_OP_IMM   : {is_op, is_op_imm, is_load, is_store, is_branch, is_jal, is_jalr, is_lui, is_auipc, is_misc_mem, is_system} = 11'b01000000000;
        OPCODE_LOAD     : {is_op, is_op_imm, is_load, is_store, is_branch, is_jal, is_jalr, is_lui, is_auipc, is_misc_mem, is_system} = 11'b00100000000;
        OPCODE_STORE    : {is_op, is_op_imm, is_load, is_store, is_branch, is_jal, is_jalr, is_lui, is_auipc, is_misc_mem, is_system} = 11'b00010000000;
        OPCODE_BRANCH   : {is_op, is_op_imm, is_load, is_store, is_branch, is_jal, is_jalr, is_lui, is_auipc, is_misc_mem, is_system} = 11'b00001000000;
        OPCODE_JAL      : {is_op, is_op_imm, is_load, is_store, is_branch, is_jal, is_jalr, is_lui, is_auipc, is_misc_mem, is_system} = 11'b00000100000;
        OPCODE_JALR     : {is_op, is_op_imm, is_load, is_store, is_branch, is_jal, is_jalr, is_lui, is_auipc, is_misc_mem, is_system} = 11'b00000010000;
        OPCODE_LUI      : {is_op, is_op_imm, is_load, is_store, is_branch, is_jal, is_jalr, is_lui, is_auipc, is_misc_mem, is_system} = 11'b00000001000;
        OPCODE_AUIPC    : {is_op, is_op_imm, is_load, is_store, is_branch, is_jal, is_jalr, is_lui, is_auipc, is_misc_mem, is_system} = 11'b00000000100;
        OPCODE_MISC_MEM : {is_op, is_op_imm, is_load, is_store, is_branch, is_jal, is_jalr, is_lui, is_auipc, is_misc_mem, is_system} = 11'b00000000010;
        OPCODE_SYSTEM    : {is_op, is_op_imm, is_load, is_store, is_branch, is_jal, is_jalr, is_lui, is_auipc, is_misc_mem, is_system} = 11'b00000000001;
        default         : {is_op, is_op_imm, is_load, is_store, is_branch, is_jal, is_jalr, is_lui, is_auipc, is_misc_mem, is_system} = 11'b00000000000;
    endcase
end

// Decoding the funct3 for specific operations
always @* begin
    case(funct3_in)
        FUNCT3_ADD  : {is_addi, is_slti, is_sltiu, is_andi, is_ori, is_xori} = {is_op_imm, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0}; // ADDI
        FUNCT3_SLT  : {is_addi, is_slti, is_sltiu, is_andi, is_ori, is_xori} = {1'b0, is_op_imm, 1'b0, 1'b0, 1'b0, 1'b0}; // SLTI
        FUNCT3_SLTU : {is_addi, is_slti, is_sltiu, is_andi, is_ori, is_xori} = {1'b0, 1'b0, is_op_imm, 1'b0, 1'b0, 1'b0}; // SLTIU
        FUNCT3_AND  : {is_addi, is_slti, is_sltiu, is_andi, is_ori, is_xori} = {1'b0, 1'b0, 1'b0, is_op_imm, 1'b0, 1'b0}; // ANDI
        FUNCT3_OR   : {is_addi, is_slti, is_sltiu, is_andi, is_ori, is_xori} = {1'b0, 1'b0, 1'b0, 1'b0, is_op_imm, 1'b0}; // ORI
        FUNCT3_XOR  : {is_addi, is_slti, is_sltiu, is_andi, is_ori, is_xori} = {1'b0, 1'b0, 1'b0, 1'b0, 1'b0, is_op_imm}; // XORI
        default     : {is_addi, is_slti, is_sltiu, is_andi, is_ori, is_xori} = 6'b000000; // Default case
    endcase
end

// Output assignments based on decoded signals
assign load_size_out = funct3_in[1:0]; // Load size is determined by the least significant bits of funct3
assign load_unsigned_out = funct3_in[2]; // Load unsigned if bit 2 of funct3 is set
assign alu_src_out = opcode_in[5]; // ALU source selection based on opcode
assign is_csr = is_system & (funct3_in[2] | funct3_in[1] | funct3_in[0]); // Check for CSR operations
assign csr_wr_en_out = is_csr; // CSR write enable based on is_csr
assign csr_op_out = funct3_in; // CSR operation code is directly from funct3

// I-adder source selection
assign iadder_src_out = is_load | is_store | is_jalr; 
assign rf_wr_en_out = is_lui | is_auipc | is_jalr | is_jal | is_op | is_load | is_csr | is_op_imm; // Register file write enable
assign alu_opcode_out[2:0] = funct3_in; // ALU opcode from funct3
assign alu_opcode_out[3] = funct7_5_in & ~(is_addi | is_slti | is_sltiu | is_andi | is_ori | is_xori); // Determine ALU opcode based on funct7_5

// Write-back multiplexer selection
assign wb_mux_sel_out[0] = is_load | is_auipc | is_jal | is_jalr; // First bit for load, AUIPC, JAL, JALR
assign wb_mux_sel_out[1] = is_csr | is_jal | is_jalr; // Second bit for CSR, JAL, JALR

// Immediate type signals
assign imm_type_out[0] = is_op_imm | is_load | is_jalr | is_branch | is_jal; // Type 0 immediate signals
assign imm_type_out[1] = is_store | is_branch | is_csr; // Type 1 immediate signals
assign imm_type_out[2] = is_lui | is_auipc | is_jal | is_csr; // Type 2 immediate signals

// Implemented instruction check
assign is_implemented_instr = is_op | is_op_imm | is_branch | is_jal | is_jalr | is_auipc | is_lui | is_system;

// Illegal instruction flag
assign illegal_instr_out = ~opcode_in[1] | ~opcode_in[0] | ~is_implemented_instr;

// Misalignment checks
assign mal_word = funct3_in[1] & ~funct3_in[0] & (iadder_1_to_0_in[1] | iadder_1_to_0_in[0]); // Word misalignment check
assign mal_half = ~funct3_in[1] & funct3_in[0] & iadder_1_to_0_in[0]; // Half-word misalignment check

endmodule
