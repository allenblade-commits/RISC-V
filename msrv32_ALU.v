module msrv32_ALU (
                    input  [31:0] op_1_in,   // Operand 1 input
                    input  [31:0] op_2_in,   // Operand 2 input
                    input  [3:0]  opcode_in, // 4-bit ALU opcode input
                    output reg [31:0] result_out // ALU result output
                   );

// Always block triggered for any change in inputs
always @(*)
begin 
    result_out = 32'b0; // Default result is set to 0
    
    // ALU operation based on the opcode_in
    case(opcode_in)
        4'b0000 : result_out = op_1_in + op_2_in;  // Addition
        4'b1000 : result_out = op_1_in - op_2_in;  // Subtraction
        4'b0010 : result_out = (op_1_in[31] ^ op_2_in[31]) ? op_1_in[31] : (op_1_in < op_2_in); // Signed less than
        4'b0011 : result_out = op_1_in < op_2_in;  // Unsigned less than
        4'b0111 : result_out = op_1_in & op_2_in;  // AND operation
        4'b0110 : result_out = op_1_in | op_2_in;  // OR operation
        4'b0100 : result_out = op_1_in ^ op_2_in;  // XOR operation
        4'b0001 : result_out = op_1_in << op_2_in; // Logical left shift
        4'b0101 : result_out = op_1_in >> op_2_in; // Logical right shift
        4'b1101 : result_out = op_1_in >>> op_2_in;// Arithmetic right shift

        // Default case can be added to handle unsupported opcodes
    endcase
end

endmodule
