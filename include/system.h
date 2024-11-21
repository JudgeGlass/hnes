#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include "global.h"
#include "macros.h"

#include "rom.h"

static ines_t g_rom;

uint8_t read_address(uint16_t address);

void init_system();
void write_address(uint16_t address, uint8_t value);

#endif