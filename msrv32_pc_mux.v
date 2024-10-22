module msrv32_pc_mux (
    input        branch_taken_in,       // Input indicating if a branch is taken
    input        rst_in,                // Reset input
    input        ahb_ready_in,          // AHB ready signal
    input  [1:0] pc_src_in,             // PC source control input
    input  [31:0] epc_in,               // Exception Program Counter (EPC) input
    input  [31:0] trap_address_in,      // Trap address input
    input  [31:0] pc_in,                // Current Program Counter (PC) input
    input  [31:1] iaddr_in,             // Instruction address input (aligned)
    output [31:0] pc_plus_4_out,        // Output for PC + 4 (next sequential instruction)
    output [31:0] i_addr_out,           // Output for the current instruction address
    output       misaligned_instr_out,  // Output indicating misaligned instruction
    output reg [31:0] pc_mux_out        // Output for the selected PC (next PC)
);

    reg [31:0] i_addr;                  // Internal register to store instruction address
    parameter boot_address = 32'h00000000; // Boot address (initial PC)

    wire [31:0] next_pc;                // Wire for the next PC value

    // Detect misaligned instruction if branch is taken and the next PC is not aligned
    assign misaligned_instr_out = next_pc[1] & branch_taken_in;

    // Calculate PC + 4 (next sequential instruction address)
    assign pc_plus_4_out = pc_in + 32'h00000004;

    // Calculate the next PC (branch or PC + 4 based on branch_taken_in)
    assign next_pc = branch_taken_in ? {iaddr_in, 1'b0} : pc_plus_4_out;

    // Output the current instruction address
    assign i_addr_out = i_addr;

    // Mux to select the next PC value based on the pc_src_in control signal
    always @(*)
    begin
        case (pc_src_in)
            2'b00: pc_mux_out = boot_address;     // Boot address
            2'b01: pc_mux_out = epc_in;           // EPC (Exception Program Counter)
            2'b10: pc_mux_out = trap_address_in;  // Trap address
            2'b11: pc_mux_out = next_pc;          // Next PC (branch or PC + 4)
            default: pc_mux_out = next_pc;        // Default case is the next PC
        endcase
    end

    // Update the instruction address based on reset and AHB ready signals
    always @(*)
    begin
        if (rst_in)
            i_addr = boot_address;    // Reset to boot address
        else if (ahb_ready_in)
            i_addr = pc_mux_out;      // Update address when AHB is ready
    end

endmodule
