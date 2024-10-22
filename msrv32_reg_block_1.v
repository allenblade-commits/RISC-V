module msrv32_reg_block_1 (
    input        clk_in,       // Clock input
    input        rst_in,       // Reset input
    input  [31:0] pc_mux_in,   // Input from the PC mux (next PC value)
    output reg [31:0] pc_out   // Program Counter output (current PC)
);

    parameter boot_address = 32'h00000000;  // Boot address (initial PC)

    // Sequential logic for updating the Program Counter (PC)
    always @(posedge clk_in)
    begin
        if (rst_in)
            pc_out <= boot_address;      // Set PC to boot address on reset
        else
            pc_out <= pc_mux_in;         // Update PC with the value from the mux
    end

endmodule
