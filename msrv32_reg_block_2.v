module msrv32_reg_block_2 (
    input [4:0] rd_addr_in,          // Destination register address
    input [11:0] csr_addr_in,        // CSR address input
    input [31:0] rs1_in, rs2_in,     // Source register inputs
    input [31:0] pc_in,              // Program Counter (PC) input
    input [31:0] pc_plus_4_in,       // PC + 4 input (next sequential instruction)
    input [31:0] iadder_in,          // Immediate adder result input
    input [31:0] imm_in,             // Immediate value input
    input [3:0] alu_opcode_in,       // ALU opcode input
    input [1:0] load_size_in,        // Load size input
    input [2:0] wb_mux_sel_in,       // Writeback mux select input
    input [2:0] csr_op_in,           // CSR operation input
    input load_unsigned_in,          // Load unsigned input
    input alu_src_in,                // ALU source select input
    input csr_wr_en_in,              // CSR write enable input
    input rf_wr_en_in,               // Register file write enable input
    input branch_taken_in,           // Branch taken signal
    input clk_in,                    // Clock input
    input reset_in,                  // Reset input

    output reg [4:0] rd_addr_reg_out,      // Register output for destination address
    output reg [11:0] csr_addr_reg_out,    // Register output for CSR address
    output reg [31:0] rs1_reg_out,         // Register output for source register 1
    output reg [31:0] rs2_reg_out,         // Register output for source register 2
    output reg [31:0] pc_reg_out,          // Register output for program counter
    output reg [31:0] pc_plus_4_reg_out,   // Register output for PC + 4
    output reg [31:0] iadder_out_reg_out,  // Register output for immediate adder result
    output reg [31:0] imm_reg_out,         // Register output for immediate value
    output reg [3:0] alu_opcode_reg_out,   // Register output for ALU opcode
    output reg [1:0] load_size_reg_out,    // Register output for load size
    output reg [2:0] wb_mux_sel_reg_out,   // Register output for writeback mux select
    output reg [2:0] csr_op_reg_out,       // Register output for CSR operation
    output reg load_unsigned_reg_out,      // Register output for load unsigned
    output reg alu_src_reg_out,            // Register output for ALU source select
    output reg csr_wr_en_reg_out,          // Register output for CSR write enable
    output reg rf_wr_en_reg_out            // Register output for register file write enable
);

    parameter BOOT_ADDRESS = 32'h00000000;  // Boot address (initial PC value)
    parameter WB_ALU = 3'b000;              // Default writeback selection for ALU results

    // Sequential logic for register operations
    always @(posedge clk_in or posedge reset_in)
    begin 
        if (reset_in) 
        begin 
            // Reset all registers to their default values
            rd_addr_reg_out <= 5'b00000;
            csr_addr_reg_out <= 12'b000000000000;
            rs1_reg_out <= 32'h00000000;
            rs2_reg_out <= 32'h00000000;
            pc_reg_out <= BOOT_ADDRESS;
            pc_plus_4_reg_out <= 32'h00000000;
            iadder_out_reg_out <= 32'h00000000;
            alu_opcode_reg_out <= 4'b0000;
            load_size_reg_out <= 2'b00;
            load_unsigned_reg_out <= 1'b0;
            alu_src_reg_out <= 1'b0;
            csr_wr_en_reg_out <= 1'b0;
            rf_wr_en_reg_out <= 1'b0;
            wb_mux_sel_reg_out <= WB_ALU;
            csr_op_reg_out <= 3'b000;
            imm_reg_out <= 32'h00000000;
        end
        else 
        begin 
            // Update the registers with input values on clock edge
            rd_addr_reg_out <= rd_addr_in;
            csr_addr_reg_out <= csr_addr_in;
            rs1_reg_out <= rs1_in;
            rs2_reg_out <= rs2_in;
            pc_reg_out <= pc_in;
            pc_plus_4_reg_out <= pc_plus_4_in;
            iadder_out_reg_out[31:1] <= iadder_in[31:1];
            iadder_out_reg_out[0] <= branch_taken_in ? 1'b0 : iadder_in[0];  // Align instruction address if branch taken
            alu_opcode_reg_out <= alu_opcode_in;
            load_size_reg_out <= load_size_in;
            load_unsigned_reg_out <= load_unsigned_in;
            alu_src_reg_out <= alu_src_in;
            csr_wr_en_reg_out <= csr_wr_en_in;
            rf_wr_en_reg_out <= rf_wr_en_in;
            wb_mux_sel_reg_out <= wb_mux_sel_in;
            csr_op_reg_out <= csr_op_in;
            imm_reg_out <= imm_in;
        end
    end

endmodule
