#ifndef __TEST_SYSTEM__
#define __TEST_SYSTEM__

#include "system.h"

void test_read_write_address()
{
  TEST_MARKER("test_read_write_address");

  write_address(SYSTEM_RAM_START, 0xDA);
  assert(read_address(SYSTEM_RAM_START) == 0xDA);
  init_system();

  for(int i = SYSTEM_RAM_START; i < 64; i++)
  {
    write_address(i, i - SYSTEM_RAM_START);
    assert(read_address(i) == (i - SYSTEM_RAM_START));
  }
  init_system();

  write_address(CART_RAM_START, 0xAD);
  assert(read_address(CART_RAM_START) == 0xAD);
  init_system();

  for(int i = CART_RAM_START; i < 64; i++)
  {
    write_address(i, i - CART_RAM_START);
    assert(read_address(i) == (i - CART_RAM_START));
  }
  init_system();
}

void test_stack()
{
  TEST_MARKER("test_stack");
  uint8_t sp = 0xFF;

  for(int i = 0; i < 64; i++)
  {
    push_stack(i, &sp);
  }

  assert(sp == (0xFF - 64));

  for(int i = 63; i >= 0; i--)
  {
    assert(pop_stack(&sp) == i);
  }

  assert(sp == 0xFF);
}

void test_system()
{
  init_system();
  RUN_TEST(test_read_write_address);
  RUN_TEST(test_stack);
}

#endif