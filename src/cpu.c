#include "cpu.h"

void cpu_init(cpu_t *cpu)
{
  init_instruction_set(cpu);
  cpu->registers.A = 0;
  cpu->registers.X = 0;
  cpu->registers.Y = 0;
  cpu->registers.SP = 0xFD;
  cpu->registers.P = 0;
  cpu->registers.PC = 0xFFFC;
}

instruction_t *get_instruction_from_op(instruction_t *instruction_set, const uint8_t op_code)
{
  for (int i = 0; i < INSTRUCTION_SET_SIZE; i++)
  {
    if (instruction_set[i].op_code == op_code)
    {
      return &instruction_set[i];
    }
  }
}

uint8_t get_operand_count(address_mode_t address_mode)
{
  switch (address_mode)
  {
  case IMP:
  case ACC:
    return 1;
  case IMM:
  case ZP:
  case ZPX:
  case ZPY:
  case INDX:
  case INDY:
  case REL:
    return 2;
  case IND:
  case ABS:
  case ABSX:
  case ABSY:
    return 3;
  }
}

void cpu_loop(cpu_t *cpu)
{
  // Get start from reset vector (0xFFFC)
  uint16_t reset_vector = read_address(cpu->registers.PC + 1);
  reset_vector = (reset_vector << 8) | read_address(cpu->registers.PC);
  cpu->registers.PC = reset_vector;

  uint16_t operands = 0x0000;

  while (TRUE)
  {
    operands = 0;

    uint8_t op_code = read_address(cpu->registers.PC);
    instruction_t *instruction = get_instruction_from_op(cpu->instruction_set, op_code);
    uint8_t operand_count = get_operand_count(instruction->address_mode); // Includes the opcode in the count

    if (operand_count == 2)
    {
      operands |= read_address(cpu->registers.PC + 1);
    }

    if (operand_count == 3)
    {
      operands |= read_address(cpu->registers.PC + 2);                  // Upper byte
      operands = (operands << 8) | read_address(cpu->registers.PC + 1); // Lower byte
    }

    cpu->registers.PC += operand_count;
  }
}

uint8_t read_address_mode(cpu_t *cpu, instruction_t *instruction, uint16_t operands)
{
  address_mode_t am = instruction->address_mode;
  if (am == ACC)
  {
    return cpu->registers.A;
  }

  if (am == IMM)
  {
    return (uint8_t)(operands & 0x00FF);
  }

  if (am == ZP)
  {
    return (uint8_t)read_address(operands & 0x00FF);
  }

  if (am == ABS)
  {
    return read_address(operands);
  }

  if (am == ABSX)
  {
    return read_address(operands + cpu->registers.X);
  }

  if (am == ABSY)
  {
    return read_address(operands + cpu->registers.Y);
  }

  if (am == INDX)
  {
    uint8_t addr = (uint8_t)(operands & 0x00FF);
    addr += cpu->registers.X;
    uint16_t lower_pointer = read_address(addr);
    uint16_t high_pointer = read_address(addr + 1);
    uint16_t final_address = (high_pointer << 8) | lower_pointer;

    return read_address(final_address);
  }

  if (am == INDY)
  {
    uint8_t addr = (uint8_t)(operands & 0x00FF);
    uint16_t pointer_low = read_address(addr);
    uint16_t pointer_high = read_address(addr + 1);
    uint16_t base_address = (pointer_high << 8) | pointer_low;

    uint16_t final_address = base_address + cpu->registers.Y;
    return read_address(final_address);
  }
}

void write_address_mode(cpu_t *cpu, instruction_t *instruction, uint16_t operands, uint8_t value)
{
  address_mode_t am = instruction->address_mode;

  if (am == ACC)
  {
    cpu->registers.A = value;
  }

  if (am == IMM)
  {
    write_address((uint8_t)(operands & 0x00FF), value);
  }

  if (am == ZP)
  {
    write_address((uint8_t)(operands & 0x00FF), value);
  }

  if (am == ABS)
  {
    write_address(operands, value);
  }

  if (am == ABSX)
  {
    write_address(operands + cpu->registers.X, value);
  }

  if (am == ABSY)
  {
    write_address(operands + cpu->registers.Y, value);
  }

  if (am == INDX)
  {
    uint8_t addr = (uint8_t)(operands & 0x00FF);
    addr += cpu->registers.X;
    uint16_t lower_pointer = read_address(addr);
    uint16_t high_pointer = read_address(addr + 1);
    uint16_t final_address = (high_pointer << 8) | lower_pointer;

    write_address(final_address, value);
  }

  if (am == INDY)
  {
    uint8_t addr = (uint8_t)(operands & 0x00FF);
    uint16_t pointer_low = read_address(addr);
    uint16_t pointer_high = read_address(addr + 1);
    uint16_t base_address = (pointer_high << 8) | pointer_low;

    uint16_t final_address = base_address + cpu->registers.Y;
    write_address(final_address, value);
  }
}

void set_flags(cpu_t *cpu, uint8_t operands, uint8_t result)
{
}

// Start CPU instructions

void adc(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  uint8_t value = read_address_mode(cpu, instruction, operands);
  uint8_t result = cpu->registers.A + value + (cpu->registers.flags & FLAG_CARRY ? 1 : 0);

  if (result == 0)
  {
    cpu->registers.flags |= FLAG_ZERO;
  }

  if (result & 0x80 != 0)
  {
    cpu->registers.flags |= FLAG_NEGATIVE;
  }

  if (result < cpu->registers.A)
  {
    cpu->registers.flags |= FLAG_CARRY;
  }

  bool signA = (cpu->registers.A & 0x80) != 0;
  bool signOperands = (value & 0x80) != 0;
  bool signResult = (result & 0x80) != 0;

  if ((signA == signOperands) && (signA != signResult))
  {
    cpu->registers.flags |= FLAG_OVERFLOW;
  }

  cpu->registers.A += result;
}

void and (instruction_t * instruction, uint16_t operands, cpu_t *cpu)
{
  uint8_t value = read_address_mode(cpu, instruction, operands);
  uint8_t result = cpu->registers.A & value;

  if (result == 0)
  {
    cpu->registers.flags |= FLAG_ZERO;
  }

  if (result & 0x80 != 0)
  {
    cpu->registers.flags |= FLAG_NEGATIVE;
  }

  cpu->registers.A = result;
}

void asl(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  uint8_t value = read_address_mode(cpu, instruction, operands);

  bool isA = (instruction->instruction_type == ACC);

  uint8_t result = (isA ? (value >> 1) : (cpu->registers.A >> 1));

  if (result == 0 && isA)
  {
    cpu->registers.flags |= FLAG_ZERO;
  }

  if (result & 0x80 != 0)
  {
    cpu->registers.flags |= FLAG_NEGATIVE;
  }

  if (value & BIT_7 == 0x1)
  {
    cpu->registers.flags |= FLAG_CARRY;
  }

  write_address_mode(cpu, instruction, operands, result);
}

void bit(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  uint8_t value = read_address_mode(cpu, instruction, operands);
  uint8_t result = cpu->registers.A & value;

  if (value & BIT_6 == 0x20)
  {
    cpu->registers.flags |= FLAG_OVERFLOW;
  }

  if (value & BIT_7 == 0x40)
  {
    cpu->registers.flags |= FLAG_NEGATIVE;
  }

  if (result == 0)
  {
    cpu->registers.flags |= FLAG_ZERO;
  }
}

void dec(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
}

void eor(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
}

void inc(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
}

void jsr(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
}

void lsh(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
}

void ora(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
}

void rol(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
}

void ror(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
}

void sbc(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
}

void load(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
}

void store(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
}

void compare(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
}

void branch(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  instruction_type_t b = instruction->instruction_type;
  if (b == BCC && cpu->registers.flags & FLAG_CARRY != 0x1)
  {
    cpu->registers.PC = operands;
  }
  else if (b == BCS && cpu->registers.flags & FLAG_CARRY == 0x1)
  {
    cpu->registers.PC = operands;
  }
  else if (b == BEQ && cpu->registers.flags & FLAG_ZERO == 0x2)
  {
    cpu->registers.PC = operands;
  }
  else if (b == BMI && cpu->registers.flags & FLAG_NEGATIVE == 0x80)
  {
    cpu->registers.PC = operands;
  }
  else if (b == BNE && cpu->registers.flags & FLAG_ZERO != 0x2)
  {
    cpu->registers.PC = operands;
  }
  else if (b == BPL && cpu->registers.flags & FLAG_NEGATIVE != 0x80)
  {
    cpu->registers.PC = operands;
  }
}

void flags(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
}

void stack(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
}

void general(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  instruction_type_t b = instruction->instruction_type;
  if (b == BRK)
  {
    cpu->registers.flags |= FLAG_BREAK;
    cpu->registers.flags |= FLAG_INTERRUPT;
    push_stack(cpu->registers.flags, cpu->registers.SP);
    push_stack((uint8_t)(cpu->registers.PC & 0x00FF), cpu->registers.SP);
    push_stack((uint8_t)((cpu->registers.PC >> 8) & 0x00FF), cpu->registers.SP);

    uint8_t irq_vec_low = read_address(0xFFFE);
    uint8_t irq_vec_high = read_address(0xFFFF);
    cpu->registers.PC = (irq_vec_high << 8) | irq_vec_low;
  }
}

// End of CPU instructions

void init_instruction_set(cpu_t *cpu)
{
  cpu->instruction_set[0] = (const instruction_t){0x69, ADC, IMM, 2};
  cpu->instruction_set[1] = (const instruction_t){0x65, ADC, ZP, 3};
  cpu->instruction_set[2] = (const instruction_t){0x75, ADC, ZPX, 4};
  cpu->instruction_set[3] = (const instruction_t){0x6d, ADC, ABS, 4};
  cpu->instruction_set[4] = (const instruction_t){0x7d, ADC, ABSX, 4};
  cpu->instruction_set[5] = (const instruction_t){0x79, ADC, ABSY, 4};
  cpu->instruction_set[6] = (const instruction_t){0x61, ADC, INDX, 6};
  cpu->instruction_set[7] = (const instruction_t){0x71, ADC, INDY, 5};
  cpu->instruction_set[8] = (const instruction_t){0x29, AND, IMM, 2};
  cpu->instruction_set[9] = (const instruction_t){0x25, AND, ZP, 3};
  cpu->instruction_set[10] = (const instruction_t){0x35, AND, ZPX, 4};
  cpu->instruction_set[11] = (const instruction_t){0x2d, AND, ABS, 4};
  cpu->instruction_set[12] = (const instruction_t){0x3d, AND, ABSX, 4};
  cpu->instruction_set[13] = (const instruction_t){0x39, AND, ABSY, 4};
  cpu->instruction_set[14] = (const instruction_t){0x21, AND, INDX, 6};
  cpu->instruction_set[15] = (const instruction_t){0x31, AND, INDY, 5};
  cpu->instruction_set[16] = (const instruction_t){0x0a, ASL, ACC, 2};
  cpu->instruction_set[17] = (const instruction_t){0x06, ASL, ZP, 5};
  cpu->instruction_set[18] = (const instruction_t){0x16, ASL, ZPX, 6};
  cpu->instruction_set[19] = (const instruction_t){0x0e, ASL, ABS, 6};
  cpu->instruction_set[20] = (const instruction_t){0x1e, ASL, ABSX, 7};
  cpu->instruction_set[21] = (const instruction_t){0x24, BIT, ZP, 3};
  cpu->instruction_set[22] = (const instruction_t){0x2c, BIT, ABS, 4};
  cpu->instruction_set[23] = (const instruction_t){0x10, BPL, REL, 2};
  cpu->instruction_set[24] = (const instruction_t){0x30, BMI, REL, 2};
  cpu->instruction_set[25] = (const instruction_t){0x50, BVC, REL, 2};
  cpu->instruction_set[26] = (const instruction_t){0x70, BVS, REL, 2};
  cpu->instruction_set[27] = (const instruction_t){0x90, BCC, REL, 2};
  cpu->instruction_set[28] = (const instruction_t){0xb0, BCS, REL, 2};
  cpu->instruction_set[29] = (const instruction_t){0xd0, BNE, REL, 2};
  cpu->instruction_set[30] = (const instruction_t){0xf0, BEQ, REL, 2};
  cpu->instruction_set[31] = (const instruction_t){0x18, CLC, IMP, 2};
  cpu->instruction_set[32] = (const instruction_t){0x38, SEC, IMP, 2};
  cpu->instruction_set[33] = (const instruction_t){0x58, CLI, IMP, 2};
  cpu->instruction_set[34] = (const instruction_t){0x78, SEI, IMP, 2};
  cpu->instruction_set[35] = (const instruction_t){0xb8, CLV, IMP, 2};
  cpu->instruction_set[36] = (const instruction_t){0xd8, CLD, IMP, 2};
  cpu->instruction_set[37] = (const instruction_t){0xf8, SED, IMP, 2};
  cpu->instruction_set[38] = (const instruction_t){0xaa, TAX, IMP, 2};
  cpu->instruction_set[39] = (const instruction_t){0x8a, TXA, IMP, 2};
  cpu->instruction_set[40] = (const instruction_t){0xca, DEX, IMP, 2};
  cpu->instruction_set[41] = (const instruction_t){0xe8, INX, IMP, 2};
  cpu->instruction_set[42] = (const instruction_t){0xa8, TAY, IMP, 2};
  cpu->instruction_set[43] = (const instruction_t){0x98, TYA, IMP, 2};
  cpu->instruction_set[44] = (const instruction_t){0x88, DEY, IMP, 2};
  cpu->instruction_set[45] = (const instruction_t){0xc8, INY, IMP, 2};
  cpu->instruction_set[46] = (const instruction_t){0x9a, TXS, IMP, 2};
  cpu->instruction_set[47] = (const instruction_t){0xba, TSX, IMP, 2};
  cpu->instruction_set[48] = (const instruction_t){0x48, PHA, IMP, 3};
  cpu->instruction_set[49] = (const instruction_t){0x68, PLA, IMP, 4};
  cpu->instruction_set[50] = (const instruction_t){0x08, PHP, IMP, 3};
  cpu->instruction_set[51] = (const instruction_t){0x28, PLP, IMP, 4};
  cpu->instruction_set[52] = (const instruction_t){0x00, BRK, IMP, 7};
  cpu->instruction_set[53] = (const instruction_t){0x40, RTI, IMP, 6};
  cpu->instruction_set[54] = (const instruction_t){0x60, RTS, IMP, 6};
  cpu->instruction_set[55] = (const instruction_t){0xea, NOP, IMP, 2};
  cpu->instruction_set[56] = (const instruction_t){0xc9, CMP, IMM, 2};
  cpu->instruction_set[57] = (const instruction_t){0xc5, CMP, ZP, 3};
  cpu->instruction_set[58] = (const instruction_t){0xd5, CMP, ZPX, 4};
  cpu->instruction_set[59] = (const instruction_t){0xcd, CMP, ABS, 4};
  cpu->instruction_set[60] = (const instruction_t){0xdd, CMP, ABSX, 4};
  cpu->instruction_set[61] = (const instruction_t){0xd9, CMP, ABSY, 4};
  cpu->instruction_set[62] = (const instruction_t){0xc1, CMP, INDX, 6};
  cpu->instruction_set[63] = (const instruction_t){0xd1, CMP, INDY, 5};
  cpu->instruction_set[64] = (const instruction_t){0xe0, CPX, IMM, 2};
  cpu->instruction_set[65] = (const instruction_t){0xe4, CPX, ZP, 3};
  cpu->instruction_set[66] = (const instruction_t){0xec, CPX, ABS, 4};
  cpu->instruction_set[67] = (const instruction_t){0xc0, CPY, IMM, 2};
  cpu->instruction_set[68] = (const instruction_t){0xc4, CPY, ZP, 3};
  cpu->instruction_set[69] = (const instruction_t){0xcc, CPY, ABS, 4};
  cpu->instruction_set[70] = (const instruction_t){0xc6, DEC, ZP, 5};
  cpu->instruction_set[71] = (const instruction_t){0xd6, DEC, ZPX, 6};
  cpu->instruction_set[72] = (const instruction_t){0xce, DEC, ABS, 6};
  cpu->instruction_set[73] = (const instruction_t){0xde, DEC, ABSX, 7};
  cpu->instruction_set[74] = (const instruction_t){0x49, EOR, IMM, 2};
  cpu->instruction_set[75] = (const instruction_t){0x45, EOR, ZP, 3};
  cpu->instruction_set[76] = (const instruction_t){0x55, EOR, ZPX, 4};
  cpu->instruction_set[77] = (const instruction_t){0x4d, EOR, ABS, 4};
  cpu->instruction_set[78] = (const instruction_t){0x5d, EOR, ABSX, 4};
  cpu->instruction_set[79] = (const instruction_t){0x59, EOR, ABSY, 4};
  cpu->instruction_set[80] = (const instruction_t){0x41, EOR, INDX, 6};
  cpu->instruction_set[81] = (const instruction_t){0x51, EOR, INDY, 5};
  cpu->instruction_set[82] = (const instruction_t){0xe6, INC, ZP, 5};
  cpu->instruction_set[83] = (const instruction_t){0xf6, INC, ZPX, 6};
  cpu->instruction_set[84] = (const instruction_t){0xee, INC, ABS, 6};
  cpu->instruction_set[85] = (const instruction_t){0xfe, INC, ABSX, 7};
  cpu->instruction_set[86] = (const instruction_t){0x4c, JMP, ABS, 3};
  cpu->instruction_set[87] = (const instruction_t){0x6c, JMP, IND, 5};
  cpu->instruction_set[88] = (const instruction_t){0x20, JSR, ABS, 6};
  cpu->instruction_set[89] = (const instruction_t){0xa9, LDA, IMM, 2};
  cpu->instruction_set[90] = (const instruction_t){0xa5, LDA, ZP, 3};
  cpu->instruction_set[91] = (const instruction_t){0xb5, LDA, ZPX, 4};
  cpu->instruction_set[92] = (const instruction_t){0xad, LDA, ABS, 4};
  cpu->instruction_set[93] = (const instruction_t){0xbd, LDA, ABSX, 4};
  cpu->instruction_set[94] = (const instruction_t){0xb9, LDA, ABSY, 4};
  cpu->instruction_set[95] = (const instruction_t){0xa1, LDA, INDX, 6};
  cpu->instruction_set[96] = (const instruction_t){0xb1, LDA, INDY, 5};
  cpu->instruction_set[97] = (const instruction_t){0xa2, LDX, IMM, 2};
  cpu->instruction_set[98] = (const instruction_t){0xa6, LDX, ZP, 3};
  cpu->instruction_set[99] = (const instruction_t){0xb6, LDX, ZPY, 4};
  cpu->instruction_set[100] = (const instruction_t){0xae, LDX, ABS, 4};
  cpu->instruction_set[101] = (const instruction_t){0xbe, LDX, ABSY, 4};
  cpu->instruction_set[102] = (const instruction_t){0xa0, LDY, IMM, 2};
  cpu->instruction_set[103] = (const instruction_t){0xa4, LDY, ZP, 3};
  cpu->instruction_set[104] = (const instruction_t){0xb4, LDY, ZPX, 4};
  cpu->instruction_set[105] = (const instruction_t){0xac, LDY, ABS, 4};
  cpu->instruction_set[106] = (const instruction_t){0xbc, LDY, ABSX, 4};
  cpu->instruction_set[107] = (const instruction_t){0x4a, LSR, ACC, 2};
  cpu->instruction_set[108] = (const instruction_t){0x46, LSR, ZP, 5};
  cpu->instruction_set[109] = (const instruction_t){0x56, LSR, ZPX, 6};
  cpu->instruction_set[110] = (const instruction_t){0x4e, LSR, ABS, 6};
  cpu->instruction_set[111] = (const instruction_t){0x5e, LSR, ABSX, 7};
  cpu->instruction_set[112] = (const instruction_t){0x09, ORA, IMM, 2};
  cpu->instruction_set[113] = (const instruction_t){0x05, ORA, ZP, 3};
  cpu->instruction_set[114] = (const instruction_t){0x15, ORA, ZPX, 4};
  cpu->instruction_set[115] = (const instruction_t){0x0d, ORA, ABS, 4};
  cpu->instruction_set[116] = (const instruction_t){0x1d, ORA, ABSX, 4};
  cpu->instruction_set[117] = (const instruction_t){0x19, ORA, ABSY, 4};
  cpu->instruction_set[118] = (const instruction_t){0x01, ORA, INDX, 6};
  cpu->instruction_set[119] = (const instruction_t){0x11, ORA, INDY, 5};
  cpu->instruction_set[120] = (const instruction_t){0x2a, ROL, ACC, 2};
  cpu->instruction_set[121] = (const instruction_t){0x26, ROL, ZP, 5};
  cpu->instruction_set[122] = (const instruction_t){0x36, ROL, ZPX, 6};
  cpu->instruction_set[123] = (const instruction_t){0x2e, ROL, ABS, 6};
  cpu->instruction_set[124] = (const instruction_t){0x3e, ROL, ABSX, 7};
  cpu->instruction_set[125] = (const instruction_t){0x6a, ROR, ACC, 2};
  cpu->instruction_set[126] = (const instruction_t){0x66, ROR, ZP, 5};
  cpu->instruction_set[127] = (const instruction_t){0x76, ROR, ZPX, 6};
  cpu->instruction_set[128] = (const instruction_t){0x6e, ROR, ABS, 6};
  cpu->instruction_set[129] = (const instruction_t){0x7e, ROR, ABSX, 7};
  cpu->instruction_set[130] = (const instruction_t){0xe9, SBC, IMM, 2};
  cpu->instruction_set[131] = (const instruction_t){0xe5, SBC, ZP, 3};
  cpu->instruction_set[132] = (const instruction_t){0xf5, SBC, ZPX, 4};
  cpu->instruction_set[133] = (const instruction_t){0xed, SBC, ABS, 4};
  cpu->instruction_set[134] = (const instruction_t){0xfd, SBC, ABSX, 4};
  cpu->instruction_set[135] = (const instruction_t){0xf9, SBC, ABSY, 4};
  cpu->instruction_set[136] = (const instruction_t){0xe1, SBC, INDX, 6};
  cpu->instruction_set[137] = (const instruction_t){0xf1, SBC, INDY, 5};
  cpu->instruction_set[138] = (const instruction_t){0x85, STA, ZP, 3};
  cpu->instruction_set[139] = (const instruction_t){0x95, STA, ZPX, 4};
  cpu->instruction_set[140] = (const instruction_t){0x8d, STA, ABS, 4};
  cpu->instruction_set[141] = (const instruction_t){0x9d, STA, ABSX, 5};
  cpu->instruction_set[142] = (const instruction_t){0x99, STA, ABSY, 5};
  cpu->instruction_set[143] = (const instruction_t){0x81, STA, INDX, 6};
  cpu->instruction_set[144] = (const instruction_t){0x91, STA, INDY, 6};
  cpu->instruction_set[145] = (const instruction_t){0x86, STX, ZP, 3};
  cpu->instruction_set[146] = (const instruction_t){0x96, STX, ZPY, 4};
  cpu->instruction_set[147] = (const instruction_t){0x8e, STX, ABS, 4};
  cpu->instruction_set[148] = (const instruction_t){0x84, STY, ZP, 3};
  cpu->instruction_set[149] = (const instruction_t){0x94, STY, ZPX, 4};
  cpu->instruction_set[150] = (const instruction_t){0x8c, STY, ABS, 4};
}