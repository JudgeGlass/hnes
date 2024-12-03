#include "system.h"

void init_system()
{
  load_rom("/home/hwilcox/hnes/mario.nes", &g_rom);

  init_ram(&g_sys_ram);
  init_cart_ram(&g_cart_ram);
}

uint8_t read_address(uint16_t address)
{
  if (address >= SYSTEM_RAM_START && address <= SYSTEM_RAM_END)
  {
    return g_sys_ram.buffer[address - SYSTEM_RAM_START];
  }
  else if (address >= PPU_START && address <= PPU_END)
  {
    // Get PPU
  }
  else if (address >= APU_IO_START && address <= APU_IO_END)
  {
    // Get APU or IO
  }
  else if (address >= APU_IO_TEST_START && address <= APU_IO_TEST_END)
  {
    // Get APU or IO test
  }
  else if (address >= UNMAPPED_START && address <= UNMAPPED_END)
  {
    // Get unmapped
  }
  else if (address >= CART_RAM_START && address <= CART_RAM_END)
  {
    return g_cart_ram.buffer[address - CART_RAM_START];
  }
  else if (address >= CART_ROM_START && address <= CART_ROM_END)
  {
    return read_rom(&g_rom, address - CART_ROM_START);
  }

  return 0x0000;
}

void write_address(uint16_t address, uint8_t value)
{
  if (address >= SYSTEM_RAM_START && address <= SYSTEM_RAM_END)
  {
    g_sys_ram.buffer[address - SYSTEM_RAM_START] = value;
  }
  else if (address >= PPU_START && address <= PPU_END)
  {
    // Get PPU
  }
  else if (address >= APU_IO_START && address <= APU_IO_END)
  {
    // Get APU or IO
  }
  else if (address >= APU_IO_TEST_START && address <= APU_IO_TEST_END)
  {
    // Get APU or IO test
  }
  else if (address >= UNMAPPED_START && address <= UNMAPPED_END)
  {
    // Get unmapped
  }
  else if (address >= CART_RAM_START && address <= CART_RAM_END)
  {
    g_cart_ram.buffer[address - CART_RAM_START] = value;
  }
  else if (address >= CART_ROM_START && address <= CART_ROM_END)
  {
  }
}

void push_stack(uint8_t value, uint8_t *sp)
{
  if ((*sp) + 0x100 <= STACK_START)
  {
    log("WARN: Stack pointer went at or below start address! (overflow)");
  }
  write_address((*sp) + 0x100, value);
  (*sp)--;
}

uint8_t pop_stack(uint8_t *sp)
{
  uint8_t value = read_address((*sp) + 0x100 + 1);
  (*sp)++;
  return value;
}
