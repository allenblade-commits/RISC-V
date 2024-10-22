module msrv32_wr_en_generator (
    input flush_in,                // Input signal to control flushing
    input rf_wr_en_reg_in,        // Register file write enable input
    input csr_wr_en_reg_in,       // CSR write enable input
    
    output wr_en_int_file_out,    // Output for register file write enable
    output wr_en_csr_file_out      // Output for CSR write enable
);

    // Generate write enable for integer register file based on flush signal
    assign wr_en_int_file_out = flush_in ? 1'b0 : rf_wr_en_reg_in;

    // Generate write enable for CSR file based on flush signal
    assign wr_en_csr_file_out = flush_in ? 1'b0 : csr_wr_en_reg_in;

endmodule
