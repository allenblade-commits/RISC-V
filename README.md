# msrv32 - RISC-V Processor Modules

This repository contains a collection of Verilog modules that implement various components of a RISC-V processor architecture, specifically focusing on the msrv32 instruction set. These modules facilitate instruction decoding, immediate handling, and arithmetic operations, among other functionalities.

## Table of Contents

- [Modules](#modules)
- [Module Descriptions](#module-descriptions)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Modules

1. `msrv32_bu`
2. `msrv32_dec`
3. `msrv32_img`
4. `msrv32_immediate_adder`
5. `msrv32_instruction_mux`

## Module Descriptions

### 1. msrv32_bu

**Description:**  
This module implements the branch unit logic for the RISC-V architecture. It evaluates branch conditions based on the opcode and `funct3` input signals to determine if a branch should be taken.

**Inputs:**
- `opcode_6_to_2_in`: Opcode bits [6:2].
- `funct3_in`: Function bits [2:0].
- `rs1_in`: Source register 1.
- `rs2_in`: Source register 2.

**Outputs:**
- `branch_taken_out`: Indicates if the branch is taken.

---

### 2. msrv32_dec

**Description:**  
The instruction decoder module decodes the opcode and `funct3` signals to determine the appropriate control signals for the processor's execution stage.

**Inputs:**
- `opcode_in`: The instruction opcode.
- `funct7_5_in`: Function bits [6].
- `funct3_in`: Function bits [2:0].
- `iadder_1_to_0_in`: Address bits for immediate handling.
- `trap_taken_in`: Indicates if a trap has been taken.

**Outputs:**
- `alu_opcode_out`: ALU opcode.
- `mem_wr_req_out`: Memory write request signal.
- `load_size_out`: Size of the load operation.
- Other control signals for various operations.

---

### 3. msrv32_img

**Description:**  
This module generates the immediate value based on the instruction and the type of immediate value required.

**Inputs:**
- `imm_type_in`: Specifies the type of immediate to generate.
- `instr_in`: The instruction from which to extract the immediate.

**Outputs:**
- `imm_out`: The generated immediate value.

---

### 4. msrv32_immediate_adder

**Description:**  
This module computes the address for the next instruction based on either the program counter (PC) or a base register and an immediate value.

**Inputs:**
- `pc_in`: Current program counter.
- `rs1_in`: Base register value.
- `imm_in`: Immediate value.
- `iadder_src_in`: Selects the source for the addition.

**Outputs:**
- `iadder_out`: The computed address.

---

### 5. msrv32_instruction_mux

**Description:**  
This module multiplexes between the incoming instruction and a default instruction based on the flush signal.

**Inputs:**
- `flush_in`: Signal to flush the instruction.
- `instr_in`: The input instruction.

**Outputs:**
- Various output signals representing parts of the instruction (opcode, funct3, etc.).

---

## Usage

To use these modules in your RISC-V processor design, instantiate them in your top-level module and connect the inputs and outputs appropriately according to your architecture. Ensure that you have a proper testbench to validate the functionality of each module.

### Example Instantiation

```verilog
msrv32_bu branch_unit (
    .opcode_6_to_2_in(opcode),
    .funct3_in(funct3),
    .rs1_in(rs1),
    .rs2_in(rs2),
    .branch_taken_out(branch_taken)
);
