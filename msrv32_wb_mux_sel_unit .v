module msrv32_wb_mux_sel_unit (
    input [2:0] wb_mux_sel_reg_in,        // Selection signal for the multiplexer
    input [31:0] alu_result_in,           // ALU result input
    input [31:0] lu_output_in,            // Load upper immediate output
    input [31:0] imm_reg_in,              // Immediate value input
    input [31:0] iadder_out_reg_in,       // IAdder output input
    input [31:0] csr_data_in,             // CSR data input
    input [31:0] pc_plus_4_reg_in,       // PC + 4 value input
    input [31:0] rs2_reg_in,              // RS2 register input
    input alu_source_reg_in,              // ALU source select signal
    output reg [31:0] wb_mux_out,         // Output from the multiplexer
    output [31:0] alu_2nd_src_mux_out     // Second source for ALU
);

    // Parameter definitions for multiplexer selection
    parameter WB_ALU        = 3'b000;
    parameter WB_LU         = 3'b001;
    parameter WB_IMM        = 3'b010;
    parameter WB_IADDER_OUT = 3'b011;
    parameter WB_CSR        = 3'b100;
    parameter WB_PC_PLUS    = 3'b101;

    // Second source selection for ALU based on alu_source_reg_in
    assign alu_2nd_src_mux_out = alu_source_reg_in ? rs2_reg_in : imm_reg_in;

    // Multiplexer logic to select the appropriate output based on wb_mux_sel_reg_in
    always @* begin
        case (wb_mux_sel_reg_in)
            WB_ALU:        wb_mux_out = alu_result_in;
            WB_LU:         wb_mux_out = lu_output_in;
            WB_IMM:        wb_mux_out = imm_reg_in;
            WB_IADDER_OUT: wb_mux_out = iadder_out_reg_in;
            WB_CSR:        wb_mux_out = csr_data_in;
            WB_PC_PLUS:    wb_mux_out = pc_plus_4_reg_in;
            default:       wb_mux_out = alu_result_in; // Default case
        endcase
    end

endmodule
