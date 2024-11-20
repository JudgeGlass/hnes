#include "rom.h"

#define PROGRAM_ROM_START 16

void load_rom(const char *file, ines_t *rom_meta)
{
  log("Loading ROM");
  FILE *f = fopen(file, "rb");
  fseek(f, 0, SEEK_END);
  uint64_t file_size = ftell(f);
  rewind(f);

  uint8_t *file_buffer = (uint8_t *)malloc(file_size * sizeof(uint8_t));
  fread(file_buffer, file_size, 1, f);
  fclose(f);

  assert(file_buffer[0] == 'N');
  assert(file_buffer[2] == 'S');

  rom_meta->prg_rom_size = file_buffer[4] * 1024 * 16;
  rom_meta->chr_rom_size = file_buffer[5] * 1024 * 8;
  if (rom_meta->chr_rom_size == 0)
  {
    rom_meta->chr_ram = TRUE;
  }

  uint8_t flag6 = file_buffer[6];
  rom_meta->mapper_number = (flag6 >> 4);
  rom_meta->alternative_nametable = ((flag6 & 0x8) >> 3);
  rom_meta->has_trainer = ((flag6 & 0x4) >> 2);
  rom_meta->battery_backup = ((flag6 & 0x2) >> 1);
  rom_meta->nametable_arrangement = flag6 & 0x1;

  rom_meta->prg_rom = (uint8_t *)malloc(sizeof(uint8_t) * rom_meta->prg_rom_size);
  rom_meta->chr_rom = (uint8_t *)malloc(sizeof(uint8_t) * rom_meta->chr_rom_size);

  uint32_t prg_rom_start = PROGRAM_ROM_START;
  if (rom_meta->has_trainer)
  {
    prg_rom_start += 512;
  }

  for (int i = prg_rom_start; i < rom_meta->prg_rom_size + prg_rom_start; i++)
  {
    rom_meta->prg_rom[i - prg_rom_start] = file_buffer[i];
  }

  for (int i = rom_meta->prg_rom_size + prg_rom_start; i < rom_meta->prg_rom_size + prg_rom_start + rom_meta->chr_rom_size; i++)
  {
    rom_meta->chr_rom[i - prg_rom_start - rom_meta->prg_rom_size] = file_buffer[i];
  }

#ifdef DEBUG
  // Save program and character from to file
  FILE *ff = fopen("/home/hwilcox/hnes/prg.bin", "wb");
  fwrite(rom_meta->prg_rom, rom_meta->prg_rom_size, 1, ff);
  fclose(ff);
  ff = fopen("/home/hwilcox/hnes/chr.bin", "wb");
  fwrite(rom_meta->chr_rom, rom_meta->chr_rom_size, 1, ff);
  fclose(ff);
#endif
}