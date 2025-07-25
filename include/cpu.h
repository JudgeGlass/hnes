#ifndef __CPU_H__
#define __CPU_H__

#include "macros.h"
#include "global.h"

#include "system.h"

#define INSTRUCTION_SET_SIZE 151

#define FLAG_NEGATIVE 0b10000000
#define FLAG_OVERFLOW 0b01000000
#define FLAG_BREAK 0b00010000
#define FLAG_DECIMAL 0b00001000
#define FLAG_INTERRUPT 0b00000100
#define FLAG_ZERO 0b00000010
#define FLAG_CARRY 0b00000001

typedef enum
{       // OP code + operand(s)
  ZPX,  // 2 bytes zero page (x)
  ZPY,  // 2 bytes zero page (y)
  ABSX, // 3 bytes absolute (x)
  ABSY, // 3 bytes absolute (y)
  INDX, // 2 bytes indirect (x) (Indexed indirect)
  INDY, // 2 bytes indirect (y) (also indirect index)
  IMP,  // 1 byte  implicit
  ACC,  // 1 byte  accumulator
  IMM,  // 2 bytes immediate
  ZP,   // 2 bytes zero page
  ABS,  // 3 bytes absolute
  REL,  // 2 bytes relative
  IND,  // 3 bytes indirect
} address_mode_t;

typedef enum
{
  ADC,
  AND,
  ASL,
  BIT,
  CMP,
  CPX,
  CPY,
  DEC,
  EOR,
  INC,
  JSR,
  LDA,
  LDX,
  LDY,
  LSR,
  ORA,
  ROL,
  ROR,
  SBC,
  STA,
  STX,
  STY,
  BPL, // Branch inst.
  BMI,
  BVC,
  BVS,
  BCC,
  BCS,
  BNE,
  BEQ,
  JMP,
  CLC, // Flag inst.
  SEC,
  CLI,
  SEI,
  CLV,
  CLD,
  SED,
  TAX, // Register inst.
  TXA,
  DEX,
  INX,
  TAY,
  TYA,
  DEY,
  INY,
  TXS, // Stack inst.
  TSX,
  PHA,
  PLA,
  PHP,
  PLP,
  BRK, // General inst.
  RTI,
  RTS,
  NOP,
} instruction_type_t;

const static char *g_instruction_names[] = {
    "ADC", "AND", "ASL", "BIT", "CMP", "CPX", "CPY", "DEC", "EOR", "INC",
    "JSR", "LDA", "LDX", "LDY", "LSR", "ORA", "ROL", "ROR", "SBC", "STA",
    "STX", "STY", "BPL", "BMI", "BVC", "BVS", "BCC", "BCS", "BNE", "BEQ",
    "JMP", "CLC", "SEC", "CLI", "SEI", "CLV", "CLD", "SED", "TAX", "TXA",
    "DEX", "INX", "TAY", "TYA", "DEY", "INY", "TXS", "TSX", "PHA", "PLA",
    "PHP", "PLP", "BRK", "RTI", "RTS", "NOP"};

typedef struct
{
  uint8_t op_code;
  instruction_type_t instruction_type;
  address_mode_t address_mode;
  uint8_t cycles;
} instruction_t;

typedef struct
{
  uint8_t A;   // Accumulator
  uint8_t X;   // General purpose X
  uint8_t Y;   // General purpose Y
  uint16_t PC; // Program counter
  uint8_t SP;  // Stack pointer
  uint8_t flags;
} registers_t;

typedef struct
{
  registers_t registers;
  instruction_t instruction_set[INSTRUCTION_SET_SIZE];
  size_t total_cpu_cycles;
} cpu_t;

void cpu_init(cpu_t *cpu);
void cpu_loop(cpu_t *cpu);

#ifdef TEST
void t_exec(cpu_t *cpu, instruction_t *instruction, uint16_t operands);
#endif

#endif