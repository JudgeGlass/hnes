#include "cpu.h"

static instruction_t *get_instruction_from_op(instruction_t *instruction_set, const uint8_t op_code);
static uint8_t get_operand_count(address_mode_t address_mode);
static uint8_t read_address_mode(cpu_t *cpu, instruction_t *instruction, uint16_t operands);
static void write_address_mode(cpu_t *cpu, instruction_t *instruction, uint16_t operands, uint8_t value);
static void init_instruction_set(cpu_t *cpu);
static void set_flag(cpu_t *cpu, bool should_set, uint8_t flag);
static void exec(cpu_t *cpu, instruction_t *instruction, uint16_t operands);

static void adc(instruction_t *instruction, uint16_t operands, cpu_t *cpu);
static void and(instruction_t *instruction, uint16_t operands, cpu_t *cpu);
static void asl(instruction_t *instruction, uint16_t operands, cpu_t *cpu);
static void bit(instruction_t *instruction, uint16_t operands, cpu_t *cpu);
static void dec(instruction_t *instruction, uint16_t operands, cpu_t *cpu);
static void eor(instruction_t *instruction, uint16_t operands, cpu_t *cpu);
static void inc(instruction_t *instruction, uint16_t operands, cpu_t *cpu);
static void jsr(instruction_t *instruction, uint16_t operands, cpu_t *cpu);
static void ora(instruction_t *instruction, uint16_t operands, cpu_t *cpu);
static void rol(instruction_t *instruction, uint16_t operands, cpu_t *cpu);
static void ror(instruction_t *instruction, uint16_t operands, cpu_t *cpu);
static void sbc(instruction_t *instruction, uint16_t operands, cpu_t *cpu);

static void shift(instruction_t *instruction, uint16_t operands, cpu_t *cpu);
static void load(instruction_t *instruction, uint16_t operands, cpu_t *cpu);
static void store(instruction_t *instruction, uint16_t operands, cpu_t *cpu);
static void compare(instruction_t *instruction, uint16_t operands, cpu_t *cpu);
static void branch(instruction_t *instruction, uint16_t operands, cpu_t *cpu);
static void flags(instruction_t *instruction, uint16_t operands, cpu_t *cpu);
static void stack(instruction_t *instruction, uint16_t operands, cpu_t *cpu);
static void general(instruction_t *instruction, uint16_t operands, cpu_t *cpu);

static bool g_skip_address_cycle = FALSE;

void cpu_init(cpu_t *cpu)
{
  init_instruction_set(cpu);
  cpu->registers.A = 0;
  cpu->registers.X = 0;
  cpu->registers.Y = 0;
  cpu->registers.SP = 0xFD;
  cpu->registers.PC = 0xFFFC;
  cpu->registers.flags = 0;
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

  return NULL;
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

  return 0;
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
    g_skip_address_cycle = FALSE;

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

    printf("CPU: A: 0x%X  X: 0x%X  Y: 0x%X  FLAGS: 0x%X  SP: 0x%X  PC: 0x%X  INST: %s  OPERANDS: 0x%X\n", cpu->registers.A,
           cpu->registers.X, cpu->registers.Y, cpu->registers.flags, cpu->registers.SP, cpu->registers.PC, g_instruction_names[instruction->instruction_type], operands);
    exec(cpu, instruction, operands);

    DELAY(CPU_CYCLE_TIME * instruction->cycles);

    cpu->registers.PC += ((g_skip_address_cycle) ? 0 : operand_count);
    cpu->total_cpu_cycles += instruction->cycles;
  }
}

static void exec(cpu_t *cpu, instruction_t *instruction, uint16_t operands)
{
  instruction_type_t b = instruction->instruction_type;
  switch (b)
  {
  case ADC:
    adc(instruction, operands, cpu);
    break;
  case AND:
    and(instruction, operands, cpu);
    break;
  case ASL:
    asl(instruction, operands, cpu);
    break;
  case BIT:
    bit(instruction, operands, cpu);
    break;
  case CMP:
  case CPX:
  case CPY:
    compare(instruction, operands, cpu);
    break;
  case DEC:
    dec(instruction, operands, cpu);
    break;
  case EOR:
    eor(instruction, operands, cpu);
    break;
  case INC:
  case INX:
  case INY:
    inc(instruction, operands, cpu);
    break;
  case JSR:
    jsr(instruction, operands, cpu);
    break;
  case LDA:
  case LDX:
  case LDY:
    load(instruction, operands, cpu);
    break;
  case LSR:
    shift(instruction, operands, cpu);
    break;
  case ORA:
    ora(instruction, operands, cpu);
    break;
  case ROL:
    rol(instruction, operands, cpu);
    break;
  case ROR:
    ror(instruction, operands, cpu);
    break;
  case SBC:
    sbc(instruction, operands, cpu);
    break;
  case STA:
  case STX:
  case STY:
    store(instruction, operands, cpu);
    break;
  case BPL: // Branch inst.
  case BMI:
  case BVC:
  case BVS:
  case BCC:
  case BCS:
  case BNE:
  case BEQ:
    branch(instruction, operands, cpu);
    break;
  case JMP:
  case CLC: // Flag inst.
  case CLI:
  case CLV:
  case CLD:
  case TAX: // Register inst.
  case TXA:
  case TAY:
  case TYA:
  case TXS: // Stack inst.
  case TSX:
  case RTI:
  case RTS:
  case BRK: // General inst.
    general(instruction, operands, cpu);
    break;
  case SEC:
  case SEI:
  case SED:
    flags(instruction, operands, cpu);
    break;

  case DEX:
  case DEY:
    dec(instruction, operands, cpu);
    break;
  case PHA:
  case PLA:
  case PHP:
  case PLP:
    stack(instruction, operands, cpu);
    break;
  case NOP:
    break;
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

  DEBUG("WARN: INVALID ADDRESS MODE");
  return 0;
}

static void write_address_mode(cpu_t *cpu, instruction_t *instruction, uint16_t operands, uint8_t value)
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

static void set_flag(cpu_t *cpu, bool should_set, uint8_t flag)
{
  if (should_set)
  {
    switch (flag)
    {
    case FLAG_NEGATIVE:
      cpu->registers.flags |= FLAG_NEGATIVE;
      break;
    case FLAG_ZERO:
      cpu->registers.flags |= FLAG_ZERO;
      break;
    case FLAG_CARRY:
      cpu->registers.flags |= FLAG_CARRY;
      break;
    case FLAG_OVERFLOW:
      cpu->registers.flags |= FLAG_OVERFLOW;
      break;

    default:
      printf("WARN: Invalid flag %d\n", flag);
    }
  }
}

// Start CPU instructions

static void adc(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  uint8_t value = read_address_mode(cpu, instruction, operands);
  uint8_t result = cpu->registers.A + value + (cpu->registers.flags & FLAG_CARRY ? 1 : 0);

  set_flag(cpu, result == 0, FLAG_ZERO);
  set_flag(cpu, (result & 0x80) != 0, FLAG_NEGATIVE);
  set_flag(cpu, result < cpu->registers.A, FLAG_CARRY);

  bool signA = (cpu->registers.A & 0x40) != 0;
  bool signOperands = (value & 0x40) != 0;
  bool signResult = (result & 0x40) != 0;

  set_flag(cpu, ((signA == signOperands) && (signA != signResult)), FLAG_OVERFLOW);

  cpu->registers.A = result;
}

static void and(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  uint8_t value = read_address_mode(cpu, instruction, operands);
  uint8_t result = cpu->registers.A & value;

  set_flag(cpu, result == 0, FLAG_ZERO);
  set_flag(cpu, (result & 0x80) != 0, FLAG_NEGATIVE);

  cpu->registers.A = result;
}

static void asl(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  uint8_t value = read_address_mode(cpu, instruction, operands);

  bool isA = (instruction->address_mode == ACC);

  uint8_t result = (isA ? (value >> 1) : (cpu->registers.A >> 1));

  set_flag(cpu, result == 0 && isA, FLAG_ZERO);
  set_flag(cpu, (result & 0x80) != 0, FLAG_NEGATIVE);
  set_flag(cpu, (value & BIT_8) == 0x80, FLAG_CARRY);

  write_address_mode(cpu, instruction, operands, result);
}

static void bit(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  uint8_t value = read_address_mode(cpu, instruction, operands);
  uint8_t result = cpu->registers.A & value;

  set_flag(cpu, (value & BIT_7) == 0x40, FLAG_OVERFLOW);
  set_flag(cpu, (value & BIT_8) == 0x80, FLAG_NEGATIVE);
  set_flag(cpu, result == 0, FLAG_ZERO);
}

static void dec(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  instruction_type_t b = instruction->instruction_type;

  if (b == DEC)
  {
    uint8_t value = read_address_mode(cpu, instruction, operands);
    set_flag(cpu, (value - 1) == 0, FLAG_ZERO);
    set_flag(cpu, ((value - 1) & BIT_8) == 0x80, FLAG_NEGATIVE);

    write_address_mode(cpu, instruction, operands, value - 1);
  }
  else if (b == DEX)
  {
    set_flag(cpu, (cpu->registers.X - 1) == 0, FLAG_ZERO);
    set_flag(cpu, ((cpu->registers.X - 1) & BIT_8) == 0x80, FLAG_NEGATIVE);

    cpu->registers.X--;
  }
  else if (b == DEY)
  {
    set_flag(cpu, (cpu->registers.Y - 1) == 0, FLAG_ZERO);
    set_flag(cpu, ((cpu->registers.Y - 1) & BIT_8) == 0x80, FLAG_NEGATIVE);

    cpu->registers.Y--;
  }
}

static void eor(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  uint8_t value = read_address_mode(cpu, instruction, operands);
  uint8_t result = cpu->registers.A ^ value;

  set_flag(cpu, result == 0, FLAG_ZERO);
  set_flag(cpu, (result & 0x80) != 0, FLAG_NEGATIVE);

  cpu->registers.A = result;
}

static void inc(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  instruction_type_t b = instruction->instruction_type;

  if (b == INC)
  {
    uint8_t value = read_address_mode(cpu, instruction, operands);
    set_flag(cpu, (value + 1) == 0, FLAG_ZERO);
    set_flag(cpu, ((value + 1) & BIT_8) == 0x80, FLAG_NEGATIVE);

    write_address_mode(cpu, instruction, operands, value + 1);
  }
  else if (b == INX)
  {
    set_flag(cpu, (cpu->registers.X + 1) == 0, FLAG_ZERO);
    set_flag(cpu, ((cpu->registers.X + 1) & BIT_8) == 0x80, FLAG_NEGATIVE);

    cpu->registers.X++;
  }
  else if (b == INY)
  {
    set_flag(cpu, (cpu->registers.Y + 1) == 0, FLAG_ZERO);
    set_flag(cpu, ((cpu->registers.Y + 1) & BIT_8) == 0x80, FLAG_NEGATIVE);

    cpu->registers.Y++;
  }
}

static void jsr(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  instruction_type_t b = instruction->instruction_type;
  if (b == JSR)
  {
    push_stack((uint8_t)(cpu->registers.PC & 0x00FF), &cpu->registers.SP);
    push_stack((uint8_t)((cpu->registers.PC >> 8) & 0x00FF), &cpu->registers.SP);
    cpu->registers.PC = operands;
    g_skip_address_cycle = TRUE;
  }
}

static void shift(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  instruction_type_t b = instruction->instruction_type;
  if (b == LSR)
  {
    if (instruction->address_mode == ACC)
    {
      set_flag(cpu, (cpu->registers.A & BIT_1) == 0x1, FLAG_CARRY);

      cpu->registers.A >>= 1;
      set_flag(cpu, cpu->registers.A == 0, FLAG_ZERO);
    }
    else
    {
      uint8_t result = read_address_mode(cpu, instruction, operands);
      set_flag(cpu, (result & BIT_1) == 0x1, FLAG_CARRY);

      result >>= 1;
      set_flag(cpu, result == 0, FLAG_ZERO);

      write_address_mode(cpu, instruction, operands, result);
    }
  }
}

static void ora(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  uint8_t result = read_address_mode(cpu, instruction, operands);

  cpu->registers.A |= result;

  set_flag(cpu, cpu->registers.A == 0, FLAG_ZERO);
  set_flag(cpu, (cpu->registers.A & BIT_8) == 0x80, FLAG_NEGATIVE);
}

static void rol(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  instruction_type_t b = instruction->instruction_type;
  if (b == ROL)
  {
    if (instruction->address_mode == ACC)
    {
      set_flag(cpu, (cpu->registers.A & BIT_8) == 0x80, FLAG_CARRY);

      cpu->registers.A <<= 1;
      set_flag(cpu, cpu->registers.A == 0, FLAG_ZERO);
      set_flag(cpu, (cpu->registers.A & BIT_8) == 0x80, FLAG_NEGATIVE);
    }
    else
    {
      uint8_t result = read_address_mode(cpu, instruction, operands);
      set_flag(cpu, (result & BIT_8) == 0x80, FLAG_CARRY);

      result <<= 1;
      set_flag(cpu, result == 0, FLAG_ZERO);
      set_flag(cpu, (result & BIT_8) == 0x80, FLAG_NEGATIVE);

      write_address_mode(cpu, instruction, operands, result);
    }
  }
}

static void ror(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  instruction_type_t b = instruction->instruction_type;
  if (b == ROR)
  {
    if (instruction->address_mode == ACC)
    {
      set_flag(cpu, (cpu->registers.A & BIT_1) == 0x1, FLAG_CARRY);

      cpu->registers.A >>= 1;
      set_flag(cpu, cpu->registers.A == 0, FLAG_ZERO);
      set_flag(cpu, (cpu->registers.A & BIT_8) == 0x80, FLAG_NEGATIVE);
    }
    else
    {
      uint8_t result = read_address_mode(cpu, instruction, operands);
      set_flag(cpu, (result & BIT_1) == 0x1, FLAG_CARRY);

      result >>= 1;
      set_flag(cpu, result == 0, FLAG_ZERO);
      set_flag(cpu, (result & BIT_8) == 0x80, FLAG_NEGATIVE);

      write_address_mode(cpu, instruction, operands, result);
    }
  }
}

static void sbc(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  uint8_t value = read_address_mode(cpu, instruction, operands);
  uint8_t result = cpu->registers.A - value - (1 - (cpu->registers.flags & FLAG_CARRY) ? 1 : 0);

  set_flag(cpu, result == 0, FLAG_ZERO);
  set_flag(cpu, (result & BIT_8) != 0, FLAG_NEGATIVE);
  set_flag(cpu, result < cpu->registers.A, FLAG_CARRY);

  bool signA = (cpu->registers.A & 0x40) != 0;
  bool signOperands = (value & 0x40) != 0;
  bool signResult = (result & 0x40) != 0;

  set_flag(cpu, (signA == signOperands) && (signA != signResult), FLAG_OVERFLOW);

  cpu->registers.A = result;
}

static void load(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  instruction_type_t b = instruction->instruction_type;
  uint8_t result = read_address_mode(cpu, instruction, operands);
  if (b == LDA)
  {
    cpu->registers.A = result;
    set_flag(cpu, cpu->registers.A == 0, FLAG_ZERO);
    set_flag(cpu, (cpu->registers.A & BIT_8) == 0x80, FLAG_NEGATIVE);
  }
  else if (b == LDX)
  {
    cpu->registers.X = result;
    set_flag(cpu, cpu->registers.X == 0, FLAG_ZERO);
    set_flag(cpu, (cpu->registers.X & BIT_8) == 0x80, FLAG_NEGATIVE);
  }
  else if (b == LDY)
  {
    cpu->registers.Y = result;
    set_flag(cpu, cpu->registers.Y == 0, FLAG_ZERO);
    set_flag(cpu, (cpu->registers.Y & BIT_8) == 0x80, FLAG_NEGATIVE);
  }
}

static void store(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  instruction_type_t b = instruction->instruction_type;
  if (b == STA)
  {
    write_address_mode(cpu, instruction, operands, cpu->registers.A);
  }
  else if (b == STX)
  {
    write_address_mode(cpu, instruction, operands, cpu->registers.X);
  }
  else if (b == STY)
  {
    write_address_mode(cpu, instruction, operands, cpu->registers.Y);
  }
}

static void compare(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  instruction_type_t b = instruction->instruction_type;
  uint8_t value = read_address_mode(cpu, instruction, operands);
  if (b == CMP)
  {
    set_flag(cpu, (cpu->registers.A - value) == 0, FLAG_ZERO);
    set_flag(cpu, (cpu->registers.A - value) >= 0, FLAG_CARRY);
  }
  else if (b == CPX)
  {
    set_flag(cpu, (cpu->registers.X - value) == 0, FLAG_ZERO);
    set_flag(cpu, (cpu->registers.X - value) >= 0, FLAG_CARRY);
  }
  else if (b == CPY)
  {
    set_flag(cpu, (cpu->registers.Y - value) == 0, FLAG_ZERO);
    set_flag(cpu, (cpu->registers.Y - value) >= 0, FLAG_CARRY);
  }
}

static void branch(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  instruction_type_t b = instruction->instruction_type;
  if (b == BCC && (cpu->registers.flags & FLAG_CARRY) != 0x1)
  {
    cpu->registers.PC = operands;
  }
  else if (b == BCS && (cpu->registers.flags & FLAG_CARRY) == 0x1)
  {
    cpu->registers.PC = operands;
  }
  else if (b == BEQ && (cpu->registers.flags & FLAG_ZERO) == 0x2)
  {
    cpu->registers.PC = operands;
  }
  else if (b == BMI && (cpu->registers.flags & FLAG_NEGATIVE) == 0x80)
  {
    cpu->registers.PC = operands;
  }
  else if (b == BNE && (cpu->registers.flags & FLAG_ZERO) != 0x2)
  {
    cpu->registers.PC = operands;
  }
  else if (b == BPL && (cpu->registers.flags & FLAG_NEGATIVE) != 0x80)
  {
    cpu->registers.PC = operands;
  }
  else if (b == BVC && (cpu->registers.flags & FLAG_OVERFLOW) != 0x40)
  {
    cpu->registers.PC = operands;
  }
  else if (b == BVS && (cpu->registers.flags & FLAG_OVERFLOW) == 0x40)
  {
    cpu->registers.PC = operands;
  }
}

static void flags(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  instruction_type_t b = instruction->instruction_type;
  if (b == SEC)
  {
    cpu->registers.flags |= FLAG_CARRY;
  }
  else if (b == SED)
  {
    cpu->registers.flags |= FLAG_DECIMAL;
  }
  else if (b == SEI)
  {
    cpu->registers.flags |= FLAG_INTERRUPT;
  }
}

static void stack(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  instruction_type_t b = instruction->instruction_type;
  if (b == PHA)
  {
    push_stack(cpu->registers.A, &cpu->registers.SP);
  }
  else if (b == PHP)
  {
    push_stack(cpu->registers.flags, &cpu->registers.SP);
  }
  else if (b == PLA)
  {
    cpu->registers.A = pop_stack(&cpu->registers.SP);
    set_flag(cpu, cpu->registers.A == 0, FLAG_ZERO);
    set_flag(cpu, (cpu->registers.A & BIT_8) == 0x80, FLAG_NEGATIVE);
  }
  else if (b == PLP)
  {
    cpu->registers.flags = pop_stack(&cpu->registers.SP);
  }
}

static void general(instruction_t *instruction, uint16_t operands, cpu_t *cpu)
{
  instruction_type_t b = instruction->instruction_type;
  if (b == BRK)
  {
    cpu->registers.flags |= FLAG_BREAK;
    cpu->registers.flags |= FLAG_INTERRUPT;
    push_stack(cpu->registers.flags, &cpu->registers.SP);
    push_stack((uint8_t)(cpu->registers.PC & 0x00FF), &cpu->registers.SP);
    push_stack((uint8_t)((cpu->registers.PC >> 8) & 0x00FF), &cpu->registers.SP);

    uint8_t irq_vec_low = read_address(0xFFFE);
    uint8_t irq_vec_high = read_address(0xFFFF);
    cpu->registers.PC = (irq_vec_high << 8) | irq_vec_low;
  }
  else if (b == CLC)
  {
    cpu->registers.flags &= ~FLAG_CARRY;
  }
  else if (b == CLD)
  {
    cpu->registers.flags &= ~FLAG_DECIMAL;
  }
  else if (b == CLI)
  {
    cpu->registers.flags &= ~FLAG_INTERRUPT;
  }
  else if (b == CLV)
  {
    cpu->registers.flags &= ~FLAG_OVERFLOW;
  }
  else if (b == JMP)
  {
    cpu->registers.PC = operands;
    g_skip_address_cycle = TRUE;
  }
  else if (b == RTI)
  {
    cpu->registers.flags = pop_stack(&cpu->registers.SP);
    cpu->registers.PC = pop_stack(&cpu->registers.SP);
  }
  else if (b == RTS)
  {
    uint8_t pc1 = pop_stack(&cpu->registers.SP);
    uint8_t pc2 = pop_stack(&cpu->registers.SP);
    cpu->registers.PC = ((pc1 << 8) | pc2) + 3;
    g_skip_address_cycle = TRUE;
  }
  else if (b == TAX)
  {
    cpu->registers.X = cpu->registers.A;
    set_flag(cpu, cpu->registers.X == 0, FLAG_ZERO);
    set_flag(cpu, (cpu->registers.X & FLAG_NEGATIVE) == 0x80, FLAG_NEGATIVE);
  }
  else if (b == TAY)
  {
    cpu->registers.Y = cpu->registers.A;
    set_flag(cpu, cpu->registers.Y == 0, FLAG_ZERO);
    set_flag(cpu, (cpu->registers.Y & FLAG_NEGATIVE) == 0x80, FLAG_NEGATIVE);
  }
  else if (b == TSX)
  {
    cpu->registers.X = cpu->registers.SP;
    set_flag(cpu, cpu->registers.X == 0, FLAG_ZERO);
    set_flag(cpu, (cpu->registers.X & FLAG_NEGATIVE) == 0x80, FLAG_NEGATIVE);
  }
  else if (b == TXA)
  {
    cpu->registers.A = cpu->registers.X;
    set_flag(cpu, cpu->registers.A == 0, FLAG_ZERO);
    set_flag(cpu, (cpu->registers.A & FLAG_NEGATIVE) == 0x80, FLAG_NEGATIVE);
  }
  else if (b == TYA)
  {
    cpu->registers.A = cpu->registers.Y;
    set_flag(cpu, cpu->registers.A == 0, FLAG_ZERO);
    set_flag(cpu, (cpu->registers.A & FLAG_NEGATIVE) == 0x80, FLAG_NEGATIVE);
  }
  else if (b == TXS)
  {
    cpu->registers.SP = cpu->registers.X;
  }
}

// End of CPU instructions

static void init_instruction_set(cpu_t *cpu)
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

void t_exec(cpu_t *cpu, instruction_t *instruction, uint16_t operands)
{
  exec(cpu, instruction, operands);
}
