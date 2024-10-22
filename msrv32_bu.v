module msrv32_bu (
    input [6:2] opcode_6_to_2_in, // Opcode bits [6:2]
    input [2:0] funct3_in,         // Function code (funct3)
    input [31:0] rs1_in,           // First source register input
    input [31:0] rs2_in,           // Second source register input
    output reg branch_taken_out     // Output signal indicating if branch is taken
);

// Opcode parameters for different instruction types
parameter OPCODE_BRANCH = 5'b11000; // Opcode for branch instructions
parameter OPCODE_JAL = 5'b11011;     // Opcode for JAL (Jump and Link)
parameter OPCODE_JALR = 5'b11001;    // Opcode for JALR (Jump and Link Register)

reg take; // Internal signal to determine if branch is taken

// Determine if the branch should be taken based on funct3
always @(*) begin
    case (funct3_in)
        3'b000: take = (rs1_in == rs2_in);                  // BEQ: Branch if equal
        3'b001: take = !(rs1_in == rs2_in);                 // BNE: Branch if not equal
        3'b100: take = rs1_in[31] ^ rs2_in[31] ? rs1_in[31] : (rs1_in < rs2_in); // BLT
        3'b101: take = rs1_in[31] ^ rs2_in[31] ? ~rs1_in[31] : !(rs1_in < rs2_in); // BGE
        3'b110: take = (rs1_in < rs2_in);                    // BLTU
        3'b111: take = !(rs1_in < rs2_in);                   // BGEU
        default: take = 1'b0;                                 // Default case
    endcase
end

// Determine if the branch is taken based on opcode and internal take signal
always @(*) begin
    case (opcode_6_to_2_in[6:2])
        OPCODE_JAL: branch_taken_out = 1'b1; // JAL instruction takes branch
        OPCODE_JALR: branch_taken_out = 1'b1; // JALR instruction takes branch
        OPCODE_BRANCH: branch_taken_out = take; // Branch instruction based on 'take'
        default: branch_taken_out = 1'b0; // No branch taken for other opcodes
    endcase
end

endmodule
