module msrv32_top #(
    parameter BOOT_ADDRESS = 32'h00000000
)(
    input ms_riscv32_mp_clk_in,
    input ms_riscv32_mp_rst_in,

    // Connection with Real Time Counter
    input [63:0] ms_riscv32_mp_rc_in,

    // Connections with Instruction Memory
    output [31:0] ms_riscv32_mp_imaddr_out,
    input  [31:0] ms_riscv32_mp_instr_in,
    input         ms_riscv32_mp_instr_hready_in,

    // Connections with Data Memory
    output [31:0] ms_riscv32_mp_dmaddr_out,
    output [31:0] ms_riscv32_mp_dmdata_out,
    output        ms_riscv32_mp_dmwr_req_out,
    output [3:0]  ms_riscv32_mp_dmwr_mask_out,
    input [31:0]  ms_riscv32_mp_data_in,
    input         ms_riscv32_mp_data_hready_in,
    input         ms_riscv32_mp_hresp_in,
    output [1:0]  ms_riscv32_mp_data_htrans_out,

    // Connections with Interrupt controller
    input ms_riscv32_mp_eirq_in,
    input ms_riscv32_mp_tirq_in,
    input ms_riscv32_mp_sirq_in
);

// Internal parameters for Writeback multiplexer selection
parameter WB_ALU = 3'b000;
parameter WB_LU = 3'b001;
parameter WB_IMM = 3'b010;
parameter WB_IADDER_OUT = 3'b011;
parameter WB_CSR = 3'b100;
parameter WB_PC_PLUS = 3'b101;

// Internal wire declarations
wire [31:0] iaddr;
wire [31:0] pc;
wire [31:0] pc_plus_4;
wire        misaligned_instr;
wire [31:0] pc_mux;
wire [31:0] rs2;
wire        mem_wr_req;
wire        flush;
wire [6:0]  opcode;
wire [6:0]  funct7;
wire [2:0]  funct3;
wire [4:0]  rs1_addr;
wire [4:0]  rs2_addr;
wire [4:0]  rd_addr;
wire [11:0] csr_addr;
wire [31:7] instr_31_to_7;
wire [31:0] rs1;
wire [31:0] imm;
wire        iadder_src;
wire        wr_en_csr_file;
wire        wr_en_integer_file;
wire [11:0] csr_addr_reg;
wire [2:0]  csr_op_reg;
wire [31:0] imm_reg;
wire [31:0] rs1_reg;
wire [31:0] pc_reg2;
wire        i_or_e;
wire        set_cause;
wire [3:0]  cause;
wire        set_epc;
wire        instret_inc;
wire        mie_clear;
wire        mie_set;
wire        misaligned_exception;
wire        mie;
wire        meget_out;
wire        mtie_out;
wire        msie_out;
wire        meip_out;
wire        mtip_out;
wire        msip_out;
wire        rf_wr_en_reg;
wire        csr_wr_en_reg;
wire        csr_wr_en_reg_file;
wire        integer_wr_en_reg_file;
wire [4:0]  rd_addr_reg;
wire [2:0]  wb_mux_sel;
wire [2:0]  wb_mux_sel_reg;
wire [31:0] lu_output; 
wire [31:0] alu_result;
wire [31:0] csr_data;
wire [31:0] pc_plus_4_reg;
wire [31:0] iadder_out_reg;
wire [31:0] rs2_reg;
wire        alu_src_reg;
wire [31:0] wb_mux_out;
wire [31:0] alu_2nd_src_mux;
wire        illegal_instr; 
wire        branch_taken;
wire [31:0] next_pc;
reg  [31:0] pc_reg;
wire        misaligned_load; 
wire        misaligned_store;
wire [3:0]  cause_in;
wire [1:0]  pc_src;
wire        trap_taken;
wire [1:0]  load_size_reg;
wire [3:0]  alu_opcode_reg; 
wire        load_unsigned_reg;

// Internal wire declarations for ALU and control signals
wire [31:0] iadder_out;
wire [31:0] epc; 
wire [31:0] trap_address;
wire [3:0]  alu_opcode;
wire [3:0]  mem_wr_mask; 
wire [1:0]  load_size;
wire        load_unsigned;
wire        alu_src;
wire        csr_wr_en;
wire        rf_wr_en;
wire [2:0]  imm_type;
wire [2:0]  csr_op;
wire [31:0] su_data_out;
wire [31:0] su_d_addr;
wire [3:0]  su_wr_mask;
wire        su_wr_req;

// PC MUX for selecting the next PC value
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

// Register Block 1 for Program Counter
msrv32_reg_block_1 REG1 (
    .clk_in(ms_riscv32_mp_clk_in),
    .rst_in(ms_riscv32_mp_rst_in),
    .pc_mux_in(pc_mux),
    .pc_out(pc)
);

// Instruction MUX to decode the instruction fields
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

// Store Unit for handling store operations
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

// Decoder for instruction control signals
msrv32_dec DEC (
    .opcode_in(opcode),
    .funct7_5_in(funct7[5]),
    .funct3_in(funct3),
    .iadder_1_to_0_in(iaddr[1:0]),
    .trap_taken_in(trap_taken),
    .alu_opcode_out(alu_opcode),
    .mem_wr_req_out(mem_wr_req),
    .load_size_out(load_size),
    .load_unsigned_out(load_unsigned),
    .alu_src_out(alu_src),
    .iadder_src_out(iadder_src),
    .csr_wr_en_out(csr_wr_en),
    .rf_wr_en_out(rf_wr_en),
    .wb_mux_sel_out(wb_mux_sel),
    .imm_type_out(imm_type),
    .csr_op_out(csr_op),
    .illegal_instr_out(illegal_instr),
    .misaligned_load_out(misaligned_load),
    .misaligned_store_out(misaligned_store)
);

// Immediate Generator to compute immediate values
msrv32_img IMG (
    .instr_in(instr_31_to_7),
    .imm_out(imm),
    .imm_type_in(imm_type)
);

// Immediate Adder for calculating addresses
msrv32_immediate_adder imm_adder (
    .pc_in(pc),
    .imm_in(imm),
    .rs1_in(rs1),
    .iadder_src_in(iadder_src),
    .iadder_out(iaddr)
);

// Branch Unit for handling branch instructions
msrv32_bu BU (
    .opcode_6_to_2_in(opcode[6:2]),
    .funct3_in(funct3),
    .rs1_in(rs1),
    .rs2_in(rs2),
    .branch_taken_out(branch_taken)
);

// Integer Register File
msrv32_integer_file IRF (
    .clk_in(ms_riscv32_mp_clk_in),
    .reset_in(ms_riscv32_mp_rst_in),
    .rs_1_addr_in(rs1_addr),
    .rs_2_addr_in(rs2_addr),
    .rs_1_out(rs1),
    .rs_2_out(rs2),
    .wr_en_in(wr_en_integer_file),
    .rd_addr_in(rd_addr),
    .wr_data_in(wb_mux_out)
);

// Load Unit for handling load instructions
msrv32_lu LU (
    .funct3_in(funct3),
    .data_in(ms_riscv32_mp_data_in),
    .load_size_in(load_size),
    .load_unsigned_in(load_unsigned),
    .alu_2nd_src_in(alu_2nd_src_mux),
    .alu_result_in(alu_result),
    .lu_out(lu_output),
    .load_unsigned_out(load_unsigned_reg)
);

// ALU for arithmetic operations
msrv32_ALU ALU (
    .alu_opcode_in(alu_opcode),
    .rs1_in(rs1),
    .rs2_in(alu_2nd_src_mux),
    .alu_result_out(alu_result)
);

// Control and Status Register File
msrv32_csr_file CSR (
    .clk_in(ms_riscv32_mp_clk_in),
    .reset_in(ms_riscv32_mp_rst_in),
    .csr_addr_in(csr_addr),
    .csr_op_in(csr_op),
    .wr_en_in(wr_en_csr_file),
    .rs1_in(rs1),
    .wr_data_in(wb_mux_out),
    .csr_data_out(csr_data),
    .epc_out(epc),
    .trap_address_out(trap_address)
);

// Control logic for handling exceptions and interrupts
msrv32_machine_control MC (
    .clk_in(ms_riscv32_mp_clk_in),
    .reset_in(ms_riscv32_mp_rst_in),
    .misaligned_load_in(misaligned_load),
    .misaligned_store_in(misaligned_store),
    .illegal_instr_in(illegal_instr),
    .exception_in(set_cause),
    .trap_taken_in(trap_taken),
    .epc_in(epc),
    .interrupts_in({ms_riscv32_mp_eirq_in, ms_riscv32_mp_tirq_in, ms_riscv32_mp_sirq_in}),
    .cause_out(cause),
    .set_epc_out(set_epc),
    .instret_inc_out(instret_inc),
    .mie_out(mie),
    .mip_out({meip_out, mtip_out, msip_out}),
    .interrupt_mask_out({mtie_out, msie_out})
);

// Pipeline Register Block for stage separation
msrv32_reg_block_2 REG2 (
    .clk_in(ms_riscv32_mp_clk_in),
    .rst_in(ms_riscv32_mp_rst_in),
    .alu_result_in(alu_result),
    .wb_mux_in(wb_mux_out),
    .rs2_in(rs2),
    .imm_in(imm),
    .pc_plus_4_in(pc_plus_4),
    .pc_in(pc),
    .load_size_in(load_size),
    .wb_mux_out(wb_mux_out),
    .alu_result_out(alu_result),
    .rs2_out(rs2_reg),
    .imm_out(imm_reg),
    .pc_plus_4_out(pc_plus_4_reg)
);

// Writeback multiplexer for final result selection
msrv32_wb_mux_sel_unit WB_MUX (
    .alu_result_in(alu_result),
    .lu_result_in(lu_output),
    .imm_in(imm_reg),
    .pc_plus_4_in(pc_plus_4_reg),
    .csr_data_in(csr_data),
    .wb_mux_sel_in(wb_mux_sel_reg),
    .wb_mux_out(wb_mux_out)
);

// Control and Status Register Write Enable
assign wr_en_integer_file = rf_wr_en_reg_file;
assign wr_en_csr_file = csr_wr_en_reg_file;

endmodule
