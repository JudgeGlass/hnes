#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include "global.h"
#include "macros.h"

#include "rom.h"
#include "ram.h"

static ines_t g_rom;
static sys_ram_t g_sys_ram;
static cart_ram_t g_cart_ram;

uint8_t read_address(uint16_t address);
uint8_t pop_stack(uint8_t *sp);

void init_system();
void write_address(uint16_t address, uint8_t value);
void push_stack(uint8_t value, uint8_t *sp);

#endif