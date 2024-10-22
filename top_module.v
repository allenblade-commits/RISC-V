module msrv32_top #(
    parameter BOOT_ADDRESS = 32'h00000000
)(
    input ms_riscv32_mp_clk_in,                // Clock input for RISC-V core
    input ms_riscv32_mp_rst_in,                // Reset input for RISC-V core

    // Connection with Real Time Counter
    input [63:0] ms_riscv32_mp_rc_in,          // Real time counter input

    // Connections with Instruction Memory
    output [31:0] ms_riscv32_mp_imaddr_out,    // Instruction memory address output
    input [31:0] ms_riscv32_mp_instr_in,       // Instruction memory data input
    input ms_riscv32_mp_instr_hready_in,       // Instruction memory ready signal

    // Connections with Data Memory
    output [31:0] ms_riscv32_mp_dmaddr_out,    // Data memory address output
    output [31:0] ms_riscv32_mp_dmdata_out,    // Data memory data output
    output ms_riscv32_mp_dmwr_req_out,         // Data memory write request output
    output [3:0] ms_riscv32_mp_dmwr_mask_out,  // Data memory write mask output
    input [31:0] ms_riscv32_mp_data_in,        // Data memory data input
    input ms_riscv32_mp_data_hready_in,        // Data memory ready signal
    input ms_riscv32_mp_hresp_in,              // Data memory response signal
    output [1:0] ms_riscv32_mp_data_htrans_out, // Data memory transaction type output

    // Connections with Interrupt Controller
    input ms_riscv32_mp_eirq_in,                // External interrupt input
    input ms_riscv32_mp_tirq_in,                // Timer interrupt input
    input ms_riscv32_mp_sirq_in                 // Software interrupt input
);

// Parameter Definitions for Write Back Multiplexer Selection
parameter WB_ALU = 3'b000;
parameter WB_LU = 3'b001;
parameter WB_IMM = 3'b010;
parameter WB_IADDER_OUT = 3'b011;
parameter WB_CSR = 3'b100;
parameter WB_PC_PLUS = 3'b101;

// Internal Wire Declarations
wire [31:0] iaddr;                            // Instruction address wire
wire [31:0] pc;                               // Program counter wire
wire [31:0] pc_plus_4;                        // PC + 4 wire
wire misaligned_instr;                         // Misaligned instruction flag
wire [31:0] pc_mux;                           // PC multiplexer output
wire [31:0] rs2;                              // Second source register wire
wire mem_wr_req;                              // Memory write request wire
wire flush;                                   // Flush signal wire
wire [6:0] opcode;                            // Opcode wire
wire [6:0] funct7;                            // Function code (7 bits) wire
wire [2:0] funct3;                            // Function code (3 bits) wire
wire [4:0] rs1_addr;                          // First source register address wire
wire [4:0] rs2_addr;                          // Second source register address wire
wire [4:0] rd_addr;                           // Destination register address wire
wire [11:0] csr_addr;                         // Control and Status Register address wire
wire [31:7] instr_31_to_7;                   // Instruction bits 31 to 7
wire [31:0] rs1;                              // First source register wire
wire [31:0] imm;                              // Immediate value wire
wire iadder_src;                              // Immediate adder source selection
wire wr_en_csr_file;                          // Write enable for CSR file
wire wr_en_integer_file;                      // Write enable for integer register file
wire [11:0] csr_addr_reg;                     // CSR address register
wire [2:0] csr_op_reg;                        // CSR operation register
wire [31:0] imm_reg;                          // Immediate value register
wire [31:0] rs1_reg;                          // First source register value
wire [31:0] pc_reg2;                          // PC register output
wire i_or_e;                                  // Instruction or exception flag
wire set_cause;                               // Set cause flag
wire [3:0] cause;                             // Cause register
wire set_epc;                                 // Set exception program counter flag
wire instret_inc;                             // Increment instruction count flag
wire mie_clear;                               // Machine interrupt enable clear flag
wire mie_set;                                 // Machine interrupt enable set flag
wire misaligned_exception;                    // Misaligned exception flag
wire mie;                                     // Machine interrupt enable flag
wire meie_out;                                // Machine external interrupt enable output
wire mtie_out;                                // Machine timer interrupt enable output
wire msie_out;                                // Machine software interrupt enable output
wire meip_out;                                // Machine external interrupt pending output
wire mtip_out;                                // Machine timer interrupt pending output
wire msip_out;                                // Machine software interrupt pending output
wire rf_wr_en_reg;                            // Register file write enable register
wire csr_wr_en_reg;                           // CSR write enable register
wire csr_wr_en_reg_file;                      // CSR write enable for file
wire integer_wr_en_reg_file;                  // Integer register write enable for file
wire [4:0] rd_addr_reg;                       // Destination register address for write back
wire [2:0] wb_mux_sel;                        // Write back multiplexer selection
wire [2:0] wb_mux_sel_reg;                    // Registered write back multiplexer selection
wire [31:0] lu_output;                        // Load unit output
wire [31:0] alu_result;                       // ALU result output
wire [31:0] csr_data;                         // CSR data output
wire [31:0] pc_plus_4_reg;                   // Registered PC + 4 output
wire [31:0] iadder_out_reg;                   // Immediate adder output register
wire [31:0] rs2_reg;                          // Second source register value
wire alu_src_reg;                             // ALU source register flag
wire [31:0] wb_mux_out;                       // Write back multiplexer output
wire [31:0] alu_2nd_src_mux;                  // ALU second source multiplexer output
wire illegal_instr;                           // Illegal instruction flag
wire branch_taken;                            // Branch taken flag
wire [31:0] next_pc;                          // Next program counter value
reg [31:0] pc_reg;                            // PC register
wire misaligned_load;                         // Misaligned load flag
wire misaligned_store;                        // Misaligned store flag
wire [3:0] cause_in;                          // Input for cause register
wire [1:0] pc_src;                            // PC source selection
wire trap_taken;                              // Trap taken flag
wire [1:0] load_size_reg;                     // Load size register
wire [3:0] alu_opcode_reg;                    // ALU opcode register
wire load_unsigned_reg;                       // Load unsigned flag

// PC_MUX Module Instance
msrv32_pc_mux PC (
    .branch_taken_in(branch_taken),
    .rst_in(ms_riscv32_mp_rst_in),
    .ahb_ready_in(ms_riscv32_mp_instr_hready_in),
    .pc_src_in(pc_src),
    .epc_in(epc),
    .trap_address_in(trap_address),
    .pc_in(pc),
    .iaddr_in(iaddr[31:1]),
    .pc_plus_4_out(pc_plus_4),
    .i_addr_out(ms_riscv32_mp_imaddr_out),
    .misaligned_instr_out(misaligned_instr),
    .pc_mux_out(pc_mux)
);

// REG BLOCK 1 Module Instance
msrv32_reg_block_1 REG1 (
    .clk_in(ms_riscv32_mp_clk_in),
    .rst_in(ms_riscv32_mp_rst_in),
    .pc_mux_in(pc_mux),
    .pc_out(pc)
);

// INSTRUCTION MUX Module Instance
msrv32_instruction_mux ID (
    .flush_in(flush),
    .instr_in(ms_riscv32_mp_instr_in),
    .opcode_out(opcode),
    .funct7_out(funct7),
    .funct3_out(funct3),
    .rs1_addr_out(rs1_addr),
    .rs2_addr_out(rs2_addr),
    .rd_addr_out(rd_addr),
    .csr_addr_out(csr_addr),
    .instr_31_7_out(instr_31_to_7)
);

// STORE UNIT Module Instance
msrv32_store_unit SU (
    .funct3_in(funct3),
    .iadder_in(iaddr),
    .rs2_in(rs2),
    .mem_wr_req_in(mem_wr_req),
    .ahb_ready_in(ms_riscv32_mp_data_hready_in),
    .d_addr_out(ms_riscv32_mp_dmaddr_out),
    .data_out(ms_riscv32_mp_dmdata_out),
    .wr_mask_out(ms_riscv32_mp_dmwr_mask_out),
    .ahb_htrans_out(ms_riscv32_mp_data_htrans_out),
    .wr_req_out(ms_riscv32_mp_dmwr_req_out)
);

// DECODER Module Instance
msrv32_dec DEC (
    .opcode_in(opcode),
    .funct7_5_in(funct7[5]),
    .funct3_in(funct3),
    .alu_opcode_out(alu_opcode_reg),
    .alu_src_out(alu_src_reg),
    .mem_wr_req_out(mem_wr_req),
    .trap_taken_out(trap_taken),
    .branch_taken_out(branch_taken),
    .illegal_instr_out(illegal_instr),
    .load_size_out(load_size_reg)
);

// DATA MEMORY MUX Module Instance
msrv32_data_memory_mux DMM (
    .alu_result_in(alu_result),
    .lu_output_in(lu_output),
    .csr_data_in(csr_data),
    .load_size_in(load_size_reg),
    .wb_mux_sel_in(wb_mux_sel),
    .wb_mux_out(wb_mux_out)
);

// ALU Module Instance
msrv32_alu ALU (
    .alu_opcode_in(alu_opcode_reg),
    .alu_src_in(alu_src_reg),
    .rs1_in(rs1),
    .rs2_in(rs2),
    .alu_result_out(alu_result)
);

// CSR FILE Module Instance
msrv32_csr_file CSR (
    .wr_en_in(wr_en_csr_file),
    .rd_addr_in(csr_addr),
    .wr_data_in(wb_mux_out),
    .csr_data_out(csr_data),
    .csr_addr_out(csr_addr_reg)
);

// WRITE BACK CONTROL Module Instance
msrv32_write_back_control WBC (
    .alu_result_in(alu_result),
    .lu_output_in(lu_output),
    .csr_data_in(csr_data),
    .wb_mux_sel_in(wb_mux_sel),
    .wb_mux_out(wb_mux_out)
);

endmodule
