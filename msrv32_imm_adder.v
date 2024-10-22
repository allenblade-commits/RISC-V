module msrv32_immediate_adder (
    input [31:0] pc_in,          // Program Counter input
    input [31:0] rs1_in,        // Register source 1 input
    input [31:0] imm_in,        // Immediate value input
    input iadder_src_in,        // Source select for adder (1: rs1, 0: pc)
    output reg [31:0] iadder_out // Immediate adder output
);

    always @(*) begin
        // Select between pc + imm or rs1 + imm based on iadder_src_in
        iadder_out = (iadder_src_in) ? (rs1_in + imm_in) : (pc_in + imm_in);
    end
endmodule
