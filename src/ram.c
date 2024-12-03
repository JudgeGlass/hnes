#include "ram.h"

void init_ram(sys_ram_t *sys_ram)
{
  DEBUG("Init system ram...");
  sys_ram->start = SYSTEM_RAM_START;
  sys_ram->end = SYSTEM_RAM_END;
  sys_ram->size = SYSTEM_RAM_END - SYSTEM_RAM_START + 1;
  sys_ram->buffer = (uint8_t *)malloc(sizeof(uint8_t) * sys_ram->size);
}

void init_cart_ram(cart_ram_t *cart_ram)
{
  DEBUG("Init cart ram...");
  cart_ram->start = CART_RAM_START;
  cart_ram->end = CART_RAM_END;
  cart_ram->size = CART_RAM_END - CART_RAM_START + 1;
  cart_ram->buffer = (uint8_t *)malloc(sizeof(uint8_t) * cart_ram->size);
}