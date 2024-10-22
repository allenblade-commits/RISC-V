module msrv32_img (
    input [2:0] imm_type_in,          // Immediate type input
    input [31:7] instr_in,            // Instruction input (bits 31 to 7)
    output reg [31:0] imm_out          // Immediate output
);

    always @(imm_type_in, instr_in) begin
        case (imm_type_in)
            // I-type and other immediate types
            3'b000: imm_out = {{20{instr_in[31]}}, instr_in[31:20]}; // I-type immediate
            3'b001: imm_out = {{20{instr_in[31]}}, instr_in[31:20]}; // Similar to I-type (could be for another format)
            3'b010: imm_out = {{20{instr_in[31]}}, instr_in[31:25], instr_in[11:7]}; // S-type immediate
            3'b011: imm_out = {{20{instr_in[31]}}, instr_in[7], instr_in[30:25], instr_in[11:8], 1'b0}; // B-type immediate
            3'b100: imm_out = {instr_in[31:12], 12'h000}; // U-type immediate (LUI)
            3'b101: imm_out = {{12{instr_in[31]}}, instr_in[19:12], instr_in[20], instr_in[30:21], 1'b0}; // J-type immediate
            3'b110: imm_out = {27'b0, instr_in[19:15]}; // Another specific immediate type
            3'b111: imm_out = {{20{instr_in[31]}}, instr_in[31:20]}; // Similar to I-type
            default: imm_out = 32'h0; // Default case for unhandled types
        endcase
    end
endmodule
