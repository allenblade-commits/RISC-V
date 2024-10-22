module msrv32_integer_file (
    input  clk_in,                  // Clock input
    input  reset_in,                // Reset input
    input  [4:0] rs_1_addr_in,      // Address for source register 1
    input  [4:0] rs_2_addr_in,      // Address for source register 2
    output [31:0] rs_1_out,         // Data output for source register 1
    output [31:0] rs_2_out,         // Data output for source register 2
    input  [4:0] rd_addr_in,        // Address for destination register
    input  wr_en_in,                // Write enable for register write-back
    input  [31:0] rd_in             // Data input for destination register
);

    reg [31:0] reg_file [31:0];     // Register file array (32 registers, 32-bit wide)
    wire fwd_op1_enable, fwd_op2_enable; // Forwarding control signals
    integer i;

    // Forwarding enable conditions for operands
    assign fwd_op1_enable = (rs_1_addr_in == rd_addr_in && wr_en_in == 1'b1) ? 1'b1 : 1'b0; 
    assign fwd_op2_enable = (rs_2_addr_in == rd_addr_in && wr_en_in == 1'b1) ? 1'b1 : 1'b0;

    // Sequential block for resetting and writing to the register file
    always @(posedge clk_in or posedge reset_in) 
    begin
        if (reset_in) 
        begin
            // Reset all registers to 0
            for (i = 0; i < 32; i = i+1)
                reg_file[i] = 32'b0;
        end 
        else if (wr_en_in && rd_addr_in) 
        begin
            // Write to the destination register
            reg_file[rd_addr_in] <= rd_in;
        end
    end

    // Forwarding or reading from register file for source 1
    assign rs_1_out = fwd_op1_enable == 1'b1 ? rd_in : reg_file[rs_1_addr_in];
    
    // Forwarding or reading from register file for source 2
    assign rs_2_out = fwd_op2_enable == 1'b1 ? rd_in : reg_file[rs_2_addr_in];

endmodule
