#ifndef __TEST_CPU__
#define __TEST_CPU__

#include "cpu.h"
#include "system.h"

static cpu_t g_cpu;

static void execute_instruction(instruction_type_t inst_type, address_mode_t addr_mode, uint16_t operands)
{
  instruction_t inst = (instruction_t){0, inst_type, addr_mode, 1};
  t_exec(&g_cpu, &inst, operands);
}

void reset()
{
  cpu_init(&g_cpu);
  init_system();
}

uint8_t reg_a()
{
  return g_cpu.registers.A;
}

uint8_t reg_x()
{
  return g_cpu.registers.X;
}

uint8_t reg_y()
{
  return g_cpu.registers.Y;
}

bool flag_set(uint8_t flag)
{
  return (g_cpu.registers.flags & flag) == flag;
}

static void test_adc()
{
  TEST_MARKER("test_adc")

  execute_instruction(ADC, IMM, 1);
  assert(reg_a() == 1);
  reset();

  for (int i = 0; i < 16; i++)
  {
    execute_instruction(ADC, IMM, 2);
  }
  assert(reg_a() == 16 * 2);
  reset();

  execute_instruction(LDA, IMM, 0xFF);
  execute_instruction(ADC, IMM, 1);
  assert(reg_a() == 0);
  assert(flag_set(FLAG_CARRY | FLAG_ZERO));
  reset();
}

static void test_load()
{
  TEST_MARKER("test_load");

  execute_instruction(LDA, IMM, 0xFF);
  assert(reg_a() == 0xFF);
  reset();

  execute_instruction(LDX, IMM, 0xFE);
  assert(reg_x() == 0xFE);
  reset();

  execute_instruction(LDY, IMM, 0xFD);
  assert(reg_y() == 0xFD);
  reset();

  write_address(SYSTEM_RAM_START + 17, 45);
  execute_instruction(LDA, ABS, SYSTEM_RAM_START + 17);
  assert(reg_a() == 45);
  reset();

  write_address(SYSTEM_RAM_START + 97, 5);
  execute_instruction(LDX, IMM, 97);
  execute_instruction(LDA, ABSX, SYSTEM_RAM_START);
  assert(reg_a() == 5);
  reset();

  write_address(SYSTEM_RAM_START + 57, 65);
  execute_instruction(LDY, IMM, 57);
  execute_instruction(LDA, ABSY, SYSTEM_RAM_START);
  assert(reg_a() == 65);
  reset();

  execute_instruction(LDY, IMM, 0xFF);
  assert(reg_y() == 0xFF);
  assert(flag_set(FLAG_NEGATIVE));
  reset();
}

void test_cpu(void)
{
  reset();
  RUN_TEST(test_load);
  RUN_TEST(test_adc);
}
#endif