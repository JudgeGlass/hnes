#ifndef __CPU_H__
#define __CPU_H__

#include "macros.h"
#include "global.h"

#include "system.h"

#define INSTRUCTION_SET_SIZE 151

#define FLAG_NEGATIVE 0b10000000
#define FLAG_OVERFLOW 0b01000000
#define FLAG_DECIMAL 0b00001000
#define FLAG_INTERRUPT 0b00000100
#define FLAG_ZERO 0b00000010
#define FLAG_CARRY 0b00000001

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
  uint16_t SP; // Stack pointer
  uint8_t P;   // Status (6 bits used for ALU)
  uint8_t flags;
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

static void adc(instruction_t *instruction, uint16_t operands);
static void and (instruction_t * instruction, uint16_t operands);
static void asl(instruction_t *instruction, uint16_t operands);
static void bit(instruction_t *instruction, uint16_t operands);
static void dec(instruction_t *instruction, uint16_t operands);
static void eor(instruction_t *instruction, uint16_t operands);
static void inc(instruction_t *instruction, uint16_t operands);
static void jsr(instruction_t *instruction, uint16_t operands);
static void lsh(instruction_t *instruction, uint16_t operands);
static void ora(instruction_t *instruction, uint16_t operands);
static void rol(instruction_t *instruction, uint16_t operands);
static void ror(instruction_t *instruction, uint16_t operands);
static void sbc(instruction_t *instruction, uint16_t operands);

static void load(instruction_t *instruction, uint16_t operands);
static void store(instruction_t *instruction, uint16_t operands);
static void compare(instruction_t *instruction, uint16_t operands);
static void branch(instruction_t *instruction, uint16_t operands);
static void flags(instruction_t *instruction, uint16_t operands);
static void stack(instruction_t *instruction, uint16_t operands);
static void general(instruction_t *instruction, uint16_t operands);

#endif