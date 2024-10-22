module msrv32_store_unit (
    input [1:0] funct3_in,         // Function to determine byte/halfword/full-word operation
    input [31:0] iadder_in,        // Immediate address input (memory address)
    input [31:0] rs2_in,           // Data to be stored
    input mem_wr_req_in,           // Memory write request signal
    input ahb_ready_in,            // AHB bus ready signal

    output [31:0] d_addr_out,      // Data address output (aligned)
    output reg [31:0] data_out,    // Data to be written to memory
    output reg [3:0] wr_mask_out,  // Write mask (byte enable signals)
    output reg [1:0] ahb_htrans_out,  // AHB transaction type (non-sequential or idle)
    output wr_req_out              // Write request output
);

    reg [31:0] byte_dout, halfword_dout;  // Data output for byte and halfword stores
    reg [3:0] byte_wr_mask, halfword_wr_mask;  // Write masks for byte and halfword stores

    // Assigning the aligned data address (word aligned)
    assign d_addr_out = {iadder_in[31:2], 2'b00};

    // Assign write request output to be the memory write request signal
    assign wr_req_out = mem_wr_req_in;

    // Logic to determine the byte data output based on the 2 LSBs of the address
    always @(*) 
    begin
        case(iadder_in[1:0])
            2'b00: byte_dout = {24'b0, rs2_in[7:0]};    // Store byte in lowest byte position
            2'b01: byte_dout = {16'b0, rs2_in[15:8], 8'b0}; // Store byte in second byte position
            2'b10: byte_dout = {8'b0, rs2_in[23:16], 16'b0}; // Store byte in third byte position
            2'b11: byte_dout = {rs2_in[31:24], 24'b0};    // Store byte in highest byte position
            default: byte_dout = 32'b0;
        endcase
    end

    // Logic to determine the halfword data output based on the 1st LSB of the address
    always @(*) 
    begin
        case(iadder_in[1])
            1'b0: halfword_dout = {16'b0, rs2_in[15:0]}; // Store halfword in the lower half
            1'b1: halfword_dout = {rs2_in[31:16], 16'b0}; // Store halfword in the upper half
            default: halfword_dout = 32'b0;
        endcase
    end

    // AHB transaction type determination based on readiness of the bus
    always @(*) 
    begin
        if (ahb_ready_in) 
        begin
            case(funct3_in)
                2'b00: data_out = byte_dout;          // Store byte
                2'b01: data_out = halfword_dout;      // Store halfword
                default: data_out = rs2_in;           // Store full word
            endcase
            ahb_htrans_out = 2'b10;  // Indicate a non-sequential transaction
        end 
        else 
        begin
            ahb_htrans_out = 2'b00;  // Idle state
        end
    end

    // Write mask generation for byte, halfword, or full word
    always @(*) 
    begin
        case(funct3_in)
            2'b00: wr_mask_out = byte_wr_mask;        // Write mask for byte store
            2'b01: wr_mask_out = halfword_wr_mask;    // Write mask for halfword store
            default: wr_mask_out = {4{mem_wr_req_in}}; // Full word write
        endcase
    end

    // Write mask generation for byte stores based on the 2 LSBs of the address
    always @(*) 
    begin
        case(iadder_in[1:0])
            2'b00: byte_wr_mask = {3'b0, mem_wr_req_in}; // Enable only the lowest byte
            2'b01: byte_wr_mask = {2'b0, mem_wr_req_in, 1'b0}; // Enable the second byte
            2'b10: byte_wr_mask = {1'b0, mem_wr_req_in, 2'b0}; // Enable the third byte
            2'b11: byte_wr_mask = {mem_wr_req_in, 3'b0}; // Enable the highest byte
            default: byte_wr_mask = {4{mem_wr_req_in}};
        endcase
    end

    // Write mask generation for halfword stores based on the 1st LSB of the address
    always @(*) 
    begin
        case(iadder_in[1])
            1'b0: halfword_wr_mask = {2'b0, {2{mem_wr_req_in}}}; // Enable lower halfword
            1'b1: halfword_wr_mask = {{2{mem_wr_req_in}}, 2'b0}; // Enable upper halfword
            default: halfword_wr_mask = {4{mem_wr_req_in}};
        endcase
    end

endmodule
