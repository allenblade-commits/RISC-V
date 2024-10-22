# msrv32 - RISC-V Processor Implementation

This repository contains Verilog modules for a RISC-V processor architecture, specifically designed around the msrv32 instruction set. The modules cover a wide range of functionalities, including arithmetic operations, instruction decoding, control logic, and data handling.

## Table of Contents

- [Modules](#modules)
- [Module Descriptions](#module-descriptions)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Modules

1. `msrv32_ALU`
2. `msrv32_bu`
3. `msrv32_dec`
4. `msrv32_img`
5. `msrv32_imm_adder`
6. `msrv32_instruction_mux`
7. `msrv32_integer_file`
8. `msrv32_lu`
9. `msrv32_pc_mux`
10. `msrv32_reg_block_1`
11. `msrv32_reg_block_2`
12. `msrv32_store_unit`
13. `msrv32_top`
14. `msrv32_top_wrapper`
15. `msrv32_wb_mux_sel_unit`
16. `msrv32_wr_en_generator`
17. `msrv32_immediate_adder`
18. `top_module`

## Module Descriptions

### 1. msrv32_ALU

**Description:**  
Implements the arithmetic logic unit (ALU) operations for the RISC-V architecture, handling various arithmetic and logical functions based on control signals.

**Inputs:**  
- `op_1_in`: Operand 1 input (32 bits).
- `op_2_in`: Operand 2 input (32 bits).
- `opcode_in`: 4-bit ALU opcode input, determining the operation to perform.

**Outputs:**  
- `result_out`: ALU result output (32 bits).

**Supported Operations:**  
- `0000`: Addition
- `1000`: Subtraction
- `0010`: Signed less than
- `0011`: Unsigned less than
- `0111`: AND operation
- `0110`: OR operation
- `0100`: XOR operation
- `0001`: Logical left shift
- `0101`: Logical right shift
- `1101`: Arithmetic right shift

---

### 2. msrv32_bu

**Description:**  
Branch unit that evaluates branch conditions and determines if a branch instruction should be taken based on the inputs.

**Inputs:**  
- `opcode_6_to_2_in`: Opcode bits [6:2].
- `funct3_in`: Function bits [2:0].
- `rs1_in`: First source register input (32 bits).
- `rs2_in`: Second source register input (32 bits).

**Outputs:**  
- `branch_taken_out`: Indicates if the branch is taken (1 bit).

**Branch Conditions Supported:**  
- `BEQ`: Branch if equal
- `BNE`: Branch if not equal
- `BLT`: Branch if less than
- `BGE`: Branch if greater than or equal
- `BLTU`: Branch if less than (unsigned)
- `BGEU`: Branch if greater than or equal (unsigned)

---

### 3. msrv32_dec

**Description:**  
Instruction decoder module that decodes the opcode and `funct3` signals to generate control signals for the processor.

**Inputs:**  
- `opcode_in`, `funct7_5_in`, `funct3_in`, `iadder_1_to_0_in`, `trap_taken_in`.

**Outputs:**  
- Control signals for ALU, memory, and register operations.

---

### 4. msrv32_img

**Description:**  
Generates immediate values based on the instruction and the specified immediate type.

**Inputs:**  
- `imm_type_in`: Immediate type input (3 bits).
- `instr_in`: Instruction input (bits 31 to 7).

**Outputs:**  
- `imm_out`: Generated immediate value (32 bits).

**Immediate Value Generation Logic:**  
- `3'b000`: I-type immediate: `{{20{instr_in[31]}}, instr_in[31:20]}`
- `3'b001`: Similar to I-type (could be for another format).
- `3'b010`: S-type immediate: `{{20{instr_in[31]}}, instr_in[31:25], instr_in[11:7]}`
- `3'b011`: B-type immediate: `{{20{instr_in[31]}}, instr_in[7], instr_in[30:25], instr_in[11:8], 1'b0}`
- `3'b100`: U-type immediate (LUI): `{instr_in[31:12], 12'h000}`
- `3'b101`: J-type immediate: `{{12{instr_in[31]}}, instr_in[19:12], instr_in[20], instr_in[30:21], 1'b0}`
- `3'b110`: Another specific immediate type: `{27'b0, instr_in[19:15]}`
- `3'b111`: Similar to I-type: `{{20{instr_in[31]}}, instr_in[31:20]}`
- Default case for unhandled types: `imm_out = 32'h0`.

---

### 5. msrv32_imm_adder

**Description:**  
Computes the effective address for immediate values by adding the program counter or a base register with an immediate value.

**Inputs:**  
- `pc_in`: Program Counter input (32 bits).
- `rs1_in`: Register source 1 input (32 bits).
- `imm_in`: Immediate value input (32 bits).
- `iadder_src_in`: Source select for adder (1: use `rs1`, 0: use `pc`).

**Outputs:**  
- `iadder_out`: The computed address.

**Operation:**  
- When `iadder_src_in` is `1`, it computes `rs1_in + imm_in`.
- When `iadder_src_in` is `0`, it computes `pc_in + imm_in`.

---

### 6. msrv32_instruction_mux

**Description:**  
Multiplexes between the incoming instruction and a default instruction based on the flush signal.

**Inputs:**  
- `flush_in`: Signal to flush the instruction.
- `instr_in`: Input instruction (32 bits).

**Outputs:**  
- `opcode_out`: Output opcode (7 bits).
- `funct3_out`: Output funct3 (3 bits).
- `funct7_out`: Output funct7 (7 bits).
- `csr_addr_out`: Output CSR address (12 bits).
- `rs1_addr_out`: Output rs1 address (5 bits).
- `rs2_addr_out`: Output rs2 address (5 bits).
- `rd_addr_out`: Output rd address (5 bits).
- `instr_31_7_out`: Output bits 31 to 7 of instruction (25 bits).

**Operation:**  
- Selects between the input instruction and a default instruction (`0x13`) based on the `flush_in` signal.
- Outputs various fields from the selected instruction.

---

### 7. msrv32_integer_file

**Description:**  
Manages the integer register file, providing read and write functionalities for the processor's registers. It supports register forwarding to handle data hazards.

**Inputs:**  
- `clk_in`: Clock input.
- `reset_in`: Reset input.
- `rs_1_addr_in`: Address for source register 1 (5 bits).
- `rs_2_addr_in`: Address for source register 2 (5 bits).
- `rd_addr_in`: Address for destination register (5 bits).
- `wr_en_in`: Write enable for register write-back.
- `rd_in`: Data input for destination register (32 bits).

**Outputs:**  
- `rs_1_out`: Data output for source register 1 (32 bits).
- `rs_2_out`: Data output for source register 2 (32 bits).

**Operation:**  
- On reset, all registers are initialized to `0`.
- If `wr_en_in` is asserted, the data in `rd_in` is written to the register specified by `rd_addr_in`.
- The outputs `rs_1_out` and `rs_2_out` provide the values of the registers specified by `rs_1_addr_in` and `rs_2_addr_in`, respectively. If the destination register is also one of these source registers and a write operation is happening, it forwards the written data.

---

### 8. msrv32_lu

**Description:**  
Handles load operations for the processor, managing the loading of data from memory with support for different data sizes (byte, half-word, word) and sign/zero extension based on the load type.

**Inputs:**  
- `load_size_in`: Specifies the size of the load (2 bits: `00` for byte, `01` for half-word, `10` for word).
- `clk_in`: Clock input.
- `load_unsigned_in`: Indicates if the load is unsigned (1 bit).
- `data_in`: Input data from memory (32 bits).
- `iadder_1_to_0_in`: Address offset (2 bits, least significant bits).
- `ahb_resp_in`: AHB response (0 for valid, 1 for error).

**Outputs:**  
- `lu_output`: Load unit output (32 bits).

**Operation:**  
- If `ahb_resp_in` is `0` (valid response), it loads data based on `load_size_in`:
  - **Byte Load**: Uses the least significant 2 bits of `iadder_1_to_0_in` to determine which byte to load.
  - **Half-word Load**: Uses the least significant 1 bit to determine the half-word offset.
  - **Word Load**: Loads the entire 32 bits.
- If `load_unsigned_in` is `1`, it ensures that the loaded data is zero-extended.

---

### 9. msrv32_pc_mux

**Description:**  
Selects the next program counter value based on control signals, such as branch taken or jump instructions.

**Inputs:**  
- `next_pc_in`: Next PC input (32 bits).
- `branch_pc_in`: PC input for branch target (32 bits).
- `jump_pc_in`: PC input for jump target (32 bits).
- `branch_taken_in`: Indicates if a branch is taken (1 bit).
- `jump_taken_in`: Indicates if a jump is taken (1 bit).

**Outputs:**  
- `pc_out`: Selected program counter output (32 bits).

**Operation:**  
- If `jump_taken_in` is `1`, it selects `jump_pc_in`.
- If `branch_taken_in` is `1`, it selects `branch_pc_in`.
- Otherwise, it selects `next_pc_in`.

---

### 10. msrv32_reg_block_1

**Description:**  
First register block for the RISC-V processor, responsible for managing the primary set of registers and their functionalities.

**Inputs:**  
- `rd_addr_in`: Destination register address (5 bits).
- `csr_addr_in`: CSR address input (12 bits).
- `rs1_in`: Source register 1 input (32 bits).
- `rs2_in`: Source register 2 input (32 bits).
- `pc_in`: Program Counter input (32 bits).
- `pc_plus_4_in`: PC + 4 input (next sequential instruction) (32 bits).
- `iadder_in`: Immediate adder result input (32 bits).
- `imm_in`: Immediate value input (32 bits).
- `alu_opcode_in`: ALU opcode input (4 bits).
- `load_size_in`: Load size input (2 bits).
- `wb_mux_sel_in`: Writeback mux select input (3 bits).
- `csr_op_in`: CSR operation input (3 bits).
- `load_unsigned_in`: Load unsigned input (1 bit).
- `alu_src_in`: ALU source select input (1 bit).
- `csr_wr_en_in`: CSR write enable input (1 bit).
- `rf_wr_en_in`: Register file write enable input (1 bit).
- `branch_taken_in`: Branch taken signal (1 bit).
- `clk_in`: Clock input.
- `reset_in`: Reset input.

**Outputs:**  
- `rd_addr_reg_out`: Register output for destination address (5 bits).
- `csr_addr_reg_out`: Register output for CSR address (12 bits).
- `rs1_reg_out`: Register output for source register 1 (32 bits).
- `rs2_reg_out`: Register output for source register 2 (32 bits).
- `pc_reg_out`: Register output for program counter (32 bits).
- `pc_plus_4_reg_out`: Register output for PC + 4 (32 bits).
- `iadder_out_reg_out`: Register output for immediate adder result (32 bits).
- `imm_reg_out`: Register output for immediate value (32 bits).
- `alu_opcode_reg_out`: Register output for ALU opcode (4 bits).
- `load_size_reg_out`: Register output for load size (2 bits).
- `wb_mux_sel_reg_out`: Register output for writeback mux select (3 bits).
- `csr_op_reg_out`: Register output for CSR operation (3 bits).
- `load_unsigned_reg_out`: Register output for load unsigned (1 bit).
- `alu_src_reg_out`: Register output for ALU source select (1 bit).
- `csr_wr_en_reg_out`: Register output for CSR write enable (1 bit).
- `rf_wr_en_reg_out`: Register output for register file write enable (1 bit).

**Operation:**  
- On reset, all registers are initialized to their default values.
- On the rising edge of the clock, the module updates the register values based on the provided inputs. If the branch is taken, the immediate adder output is aligned accordingly.

---

### 11. msrv32_reg_block_2

**Description:**  
Second register block for the RISC-V processor, responsible for managing additional registers and their functionalities.

**Inputs:**  
- `rd_addr_in`: Destination register address (5 bits).
- `csr_addr_in`: CSR address input (12 bits).
- `rs1_in`: Source register 1 input (32 bits).
- `rs2_in`: Source register 2 input (32 bits).
- `pc_in`: Program Counter input (32 bits).
- `pc_plus_4_in`: PC + 4 input (next sequential instruction) (32 bits).
- `iadder_in`: Immediate adder result input (32 bits).
- `imm_in`: Immediate value input (32 bits).
- `alu_opcode_in`: ALU opcode input (4 bits).
- `load_size_in`: Load size input (2 bits).
- `wb_mux_sel_in`: Writeback mux select input (3 bits).
- `csr_op_in`: CSR operation input (3 bits).
- `load_unsigned_in`: Load unsigned input (1 bit).
- `alu_src_in`: ALU source select input (1 bit).
- `csr_wr_en_in`: CSR write enable input (1 bit).
- `rf_wr_en_in`: Register file write enable input (1 bit).
- `branch_taken_in`: Branch taken signal (1 bit).
- `clk_in`: Clock input.
- `reset_in`: Reset input.

**Outputs:**  
- `rd_addr_reg_out`: Register output for destination address (5 bits).
- `csr_addr_reg_out`: Register output for CSR address (12 bits).
- `rs1_reg_out`: Register output for source register 1 (32 bits).
- `rs2_reg_out`: Register output for source register 2 (32 bits).
- `pc_reg_out`: Register output for program counter (32 bits).
- `pc_plus_4_reg_out`: Register output for PC + 4 (32 bits).
- `iadder_out_reg_out`: Register output for immediate adder result (32 bits).
- `imm_reg_out`: Register output for immediate value (32 bits).
- `alu_opcode_reg_out`: Register output for ALU opcode (4 bits).
- `load_size_reg_out`: Register output for load size (2 bits).
- `wb_mux_sel_reg_out`: Register output for writeback mux select (3 bits).
- `csr_op_reg_out`: Register output for CSR operation (3 bits).
- `load_unsigned_reg_out`: Register output for load unsigned (1 bit).
- `alu_src_reg_out`: Register output for ALU source select (1 bit).
- `csr_wr_en_reg_out`: Register output for CSR write enable (1 bit).
- `rf_wr_en_reg_out`: Register output for register file write enable (1 bit).

**Operation:**  
- On reset, all registers are initialized to their default values.
- On the rising edge of the clock, the module updates the register values based on the provided inputs. If the branch is taken, the immediate adder output is aligned accordingly.

---

### 12. msrv32_store_unit

**Description:**  
Handles store operations, managing the writing of data from the processor to memory based on store instructions.

**Inputs:**  
- `data_in`: Data input to store (32 bits).
- `address_in`: Memory address input for the store operation (32 bits).
- `store_size_in`: Specifies the size of the store (2 bits).
- `clk_in`: Clock input.
- `reset_in`: Reset input.
- `store_enable_in`: Enable signal for the store operation (1 bit).

**Outputs:**  
- `store_response_out`: Response from the store operation (1 bit).

**Operation:**  
- If `reset_in` is asserted, the module resets its internal state.
- On the rising edge of the clock, if `store_enable_in` is asserted, the module stores `data_in` to the specified `address_in` based on `store_size_in`.

---

### 13. msrv32_top

**Description:**  
Top-level module for the RISC-V processor, integrating all components, including the ALU, register blocks, instruction decoder, and memory units.

**Inputs:**  
- `clk`: System clock input.
- `reset`: System reset input.

**Outputs:**  
- `result`: Processor output (32 bits).

**Operation:**  
- Initializes all sub-modules and manages the flow of data and control signals across the processor components.

---

### 14. msrv32_top_wrapper

**Description:**  
Wrapper for the top-level module, designed to handle testbench connections and provide a test environment for the RISC-V processor.

**Inputs:**  
- `clk`: Clock input.
- `reset`: Reset input.

**Outputs:**  
- `result`: Result output from the processor (32 bits).

**Operation:**  
- Instantiates the `msrv32_top` module and connects it to the testbench environment, allowing for simulation and verification.

---

### 15. msrv32_wb_mux_sel_unit

**Description:**  
Multiplexer unit for selecting the write-back data to be written back to the register file.

**Inputs:**  
- `alu_result_in`: ALU result input (32 bits).
- `load_data_in`: Data loaded from memory (32 bits).
- `wb_mux_sel_in`: Write-back select signal (2 bits).

**Outputs:**  
- `wb_data_out`: Selected write-back data output (32 bits).

**Operation:**  
- Selects between the ALU result and the load data based on the `wb_mux_sel_in` signal.

---

### 16. msrv32_wr_en_generator

**Description:**  
Generates the write enable signal for the register file based on the instruction type and control signals.

**Inputs:**  
- `opcode_in`: Opcode input (7 bits).
- `funct3_in`: Function bits (3 bits).
- `rs1_in`: Source register 1 (5 bits).
- `rs2_in`: Source register 2 (5 bits).
- `rd_in`: Destination register (5 bits).

**Outputs:**  
- `wr_en_out`: Write enable output (1 bit).

**Operation:**  
- Determines if the write operation should be enabled based on the instruction type and if the destination register is valid.

---

### 17. msrv32_immediate_adder

**Description:**  
Adds an immediate value to the program counter or to a base register for addressing modes.

**Inputs:**  
- `pc_in`: Program Counter input (32 bits).
- `rs1_in`: Source register input (32 bits).
- `imm_in`: Immediate value input (32 bits).
- `alu_src_in`: ALU source select input (1 bit).

**Outputs:**  
- `iadder_out`: Result output (32 bits).

**Operation:**  
- If `alu_src_in` is `1`, it adds `rs1_in` and `imm_in`. Otherwise, it adds `pc_in` and `imm_in`.

---

### 18. top_module

**Description:**  
Highest level module for testing purposes, allowing for integration and simulation of the RISC-V processor.

**Inputs:**  
- `clk`: System clock input.
- `reset`: System reset input.

**Outputs:**  
- `output_data`: Output data from the processor (32 bits).

**Operation:**  
- Instantiates `msrv32_top` and manages the testbench environment for simulation and verification.

---

## Usage

To simulate the RISC-V processor design, follow these steps:

1. Clone the repository:
   ```bash
   git clone <repository-url>
