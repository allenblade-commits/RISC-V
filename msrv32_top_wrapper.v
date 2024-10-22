module msrv32_top_wrapper (
    msrv32_ahb_instr_if instr_if,
    msrv32_ahb_data_if data_if,
    msrv32_rst_if rst_if,
    msrv32_irq_if irq_if
);

    // Instantiate the DUT (Device Under Test) - msrv32_top
    msrv32_top DUT (
        // Connect the input clock and reset signals
        .ms_riscv32_mp_clk_in(instr_if.ms_riscv32_mp_clk_in),
        .ms_riscv32_mp_rst_in(rst_if.ms_riscv32_mp_rst_in),
        .ms_riscv32_mp_rc_in(rst_if.ms_riscv32_mp_rc_in),

        // Connect Instruction Memory Interface
        .ms_riscv32_mp_imaddr_out(instr_if.ms_riscv32_mp_imaddr_out),
        .ms_riscv32_mp_instr_in(instr_if.ms_riscv32_mp_instr_in),
        .ms_riscv32_mp_instr_hready_in(instr_if.ms_riscv32_mp_instr_hready_in),

        // Connect Data Memory Interface
        .ms_riscv32_mp_dmaddr_out(data_if.ms_riscv32_mp_dmaddr_out),
        .ms_riscv32_mp_dmdata_out(data_if.ms_riscv32_mp_dmdata_out),
        .ms_riscv32_mp_dmwr_req_out(data_if.ms_riscv32_mp_dmwr_req_out),
        .ms_riscv32_mp_dmwr_mask_out(data_if.ms_riscv32_mp_dmwr_mask_out),
        .ms_riscv32_mp_data_in(data_if.ms_riscv32_mp_data_in),
        .ms_riscv32_mp_data_hready_in(data_if.ms_riscv32_mp_data_hready_in),
        .ms_riscv32_mp_hresp_in(data_if.ms_riscv32_mp_hresp_in),
        .ms_riscv32_mp_data_htrans_out(data_if.ms_riscv32_mp_data_htrans_out),

        // Connect Interrupt Request Interface
        .ms_riscv32_mp_eirq_in(irq_if.ms_riscv32_mp_eirq_in),
        .ms_riscv32_mp_tirq_in(irq_if.ms_riscv32_mp_tirq_in),
        .ms_riscv32_mp_sirq_in(irq_if.ms_riscv32_mp_sirq_in)
    );

endmodule
