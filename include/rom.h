#ifndef __ROM_H__
#define __ROM_H__

#include "global.h"
#include "macros.h"

/***
 * Header (16 bytes)
 * Trainer, (0 or 512 bytes)
 * PRG ROM (16384 * x bytes)
 * CHR ROM (0 or 8192 * y bytes)
 * PlayChoice INST-ROM (0 or 8192 bytes)
 */

typedef struct
{
  uint8_t header[16];
  bool has_trainer;
  bool chr_ram;
  uint8_t *trainer;
  uint8_t *prg_rom;
  uint8_t *chr_rom;
  uint32_t prg_rom_size;
  uint32_t chr_rom_size;

  uint8_t mapper_number;
  bool alternative_nametable;
  bool battery_backup;
  bool nametable_arrangement;
} ines_t;

void load_rom(const char *file, ines_t *rom_meta);

#endif