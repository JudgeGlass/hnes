#ifndef __CPU_H__
#define __CPU_H__

#include "macros.h"
#include "global.h"

#include "system.h"

#define INSTRUCTION_SET_SIZE 151

typedef enum
{       // OP code + operand(s)
  ZPX,  // 2 bytes
  ZPY,  // 2 bytes
  ABSX, // 3 bytes
  ABSY, // 3 bytes
  INDX, // 2 bytes
  INDY, // 2 bytes
  IMP,  // 1 byte
  ACC,  // 1 byte
  IMM,  // 2 bytes
  ZP,   // 2 bytes
  ABS,  // 3 bytes
  REL,  // 2 bytes
  IND,  // 3 bytes
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
  uint32_t SP; // Stack pointer
  uint8_t P;   // Status (6 bits used for ALU)
} registers_t;

typedef struct
{
  registers_t registers;
  instruction_t instruction_set[INSTRUCTION_SET_SIZE];
} cpu_t;

void cpu_init(cpu_t *cpu);
void cpu_loop(cpu_t *cpu);

static instruction_t *get_instruction_from_op(instruction_t *instruction_set, const uint8_t op_code);
static uint8_t get_operand_count(address_mode_t address_mode);
static void init_instruction_set(cpu_t *cpu);

#endif