#include "rom.h"

#define PROGRAM_ROM_START 16

uint8_t *read_file(const char *file, uint64_t *file_size)
{
  FILE *f = fopen(file, "rb");
  if (f == NULL)
  {
    printf("Fatal error: Could not open %s\n", file);
    exit(-1);
  }
  fseek(f, 0, SEEK_END);
  uint64_t fs = ftell(f);
  rewind(f);

  uint8_t *file_buffer = (uint8_t *)malloc(fs * sizeof(uint8_t));
  fread(file_buffer, fs, 1, f);
  fclose(f);

  *file_size = fs;
  return file_buffer;
}

void load_rom(const char *file, ines_t *rom_meta)
{
  DEBUG("Loading ROM");
  uint64_t file_size = 0;
  uint8_t *file_buffer = read_file(file, &file_size);

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

  // TEST REMOVE LATER
  uint64_t fs = 0;
  uint8_t *fb = read_file("/home/hwilcox/hnes/EEPROM.bin", &fs);
  rom_meta->prg_rom = fb;
  rom_meta->prg_rom_size = 1024 * 32;
  //

#ifdef IS_DEBUG
  // Save program and character roms to file
  FILE *ff = fopen("/home/hwilcox/hnes/prg.bin", "wb");
  fwrite(rom_meta->prg_rom, rom_meta->prg_rom_size, 1, ff);
  fclose(ff);
  ff = fopen("/home/hwilcox/hnes/chr.bin", "wb");
  fwrite(rom_meta->chr_rom, rom_meta->chr_rom_size, 1, ff);
  fclose(ff);
#endif
}

uint8_t read_rom(ines_t *rom_meta, const uint16_t address)
{
  uint32_t rom_size = rom_meta->prg_rom_size;
  if (address > rom_size)
  {
    DEBUG("Warning, invalid PRG ROM address!");
    return 0x00;
  }
  return rom_meta->prg_rom[address];
}