#ifndef __PPU_H__
#define __PPU_H__

#include "global.h"
#include "macros.h"

// PPUCTRL
#define VBLANK_NMI 0b10000000 // V
#define PPU_SLAVE_SELECT 0b01000000 // P
#define SPRITE_SIZE 0b00100000 // H
#define BACKGROUND_PATTERN_ADDRESS 0b00010000 // B
#define SPRITE_PATTERN_ADDRESS 0b00001000 // S
#define VRAM_ADDRESS_INC 0b00000100 // S
#define BASE_NAMETABLE_ADDRESS 0b00000011 // N

typedef struct
{
  uint8_t PPUCTRL; // W
  uint8_t PPUMASK; // W
  uint8_t PPUSTATUS; // R
  uint8_t OAMADDR; // W
  uint8_t OAMDATA; // RW
  uint8_t PPUSCROLL; // Two writes: X scroll, then Y scroll  W
  uint8_t PPUADDR; // VRAM, two writes: upper bits + lower bits  w
  uint8_t PPUDATA; // RW
  uint8_t OAMDMA; // W
} ppu_registers_t;


#endif