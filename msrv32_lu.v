module msrv32_lu (
    input  [1:0] load_size_in,         // Specifies the size of the load (byte, half, word)
    input        clk_in,               // Clock input
    input        load_unsigned_in,     // Specifies if the load is unsigned
    input  [31:0] data_in,             // Input data from memory
    input  [1:0] iadder_1_to_0_in,     // Address offset (least significant 2 bits)
    input        ahb_resp_in,          // AHB response (0 for valid, 1 for error)
    output reg [31:0] lu_output        // Load unit output
);

    reg [7:0]  data_byte;              // To hold the selected byte of data
    reg [15:0] data_half;              // To hold the selected half-word of data
    wire [23:0] byte_ext;              // Sign or zero extension for byte
    wire [15:0] half_ext;              // Sign or zero extension for half-word

    // Load unit output logic based on load size and AHB response
    always @(*)
    begin
        if (!ahb_resp_in) 
        begin
            case (load_size_in)
                2'b00: lu_output = {byte_ext, data_byte};    // Byte load
                2'b01: lu_output = {half_ext, data_half};    // Half-word load
                2'b10: lu_output = data_in;                  // Word load (32 bits)
                2'b11: lu_output = data_in;                  // Word load (32 bits)
            endcase
        end
        else
        begin
            lu_output = 32'dZ;   // High impedance in case of an AHB error
        end
    end

    // Select the appropriate byte based on the least significant 2 bits of the address
    always @(*)
    begin
        case (iadder_1_to_0_in)
            2'b00: data_byte = data_in[7:0];     // Byte 0
            2'b01: data_byte = data_in[15:8];    // Byte 1
            2'b10: data_byte = data_in[23:16];   // Byte 2
            2'b11: data_byte = data_in[31:24];   // Byte 3
        endcase
    end

    // Select the appropriate half-word based on the address offset (bit 1)
    always @(*)
    begin
        case (iadder_1_to_0_in[1])
            1'b0: data_half = data_in[15:0];     // Lower 16 bits
            1'b1: data_half = data_in[31:16];    // Upper 16 bits
        endcase
    end

    // Byte extension logic: zero-extension for unsigned, sign-extension for signed
    assign byte_ext = (load_unsigned_in == 1'b1) ? 24'b0 : {24{data_byte[7]}};

    // Half-word extension logic: zero-extension for unsigned, sign-extension for signed
    assign half_ext = (load_unsigned_in == 1'b1) ? 16'b0 : {16{data_half[15]}};

endmodule
