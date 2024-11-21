#include "system.h"

void init_system()
{
  load_rom("/home/hwilcox/Documents/hnes/mario.nes", &g_rom);
}

uint8_t read_address(uint16_t address)
{
  if (address >= SYSTEM_RAM_START && address <= SYSTEM_RAM_END)
  {
    // Get ram
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
    // Get CART RAM
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
    // Get ram
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
    // Get CART RAM
  }
  else if (address >= CART_ROM_START && address <= CART_ROM_END)
  {
    //  Get CART ROM
  }
}
