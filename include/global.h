#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#define IS_DEBUG

typedef uint8_t bool;

#define TRUE 1
#define FALSE 0;

#define BIT_1 0b00000001
#define BIT_2 0b00000010
#define BIT_3 0b00000100
#define BIT_4 0b00001000
#define BIT_5 0b00010000
#define BIT_6 0b00100000
#define BIT_7 0b01000000
#define BIT_8 0b10000000

#define SYSTEM_RAM_START 0x0000
#define SYSTEM_RAM_END 0x07FF
#define PPU_START 0x2000
#define PPU_END 0x2007
#define APU_IO_START 0x4000
#define APU_IO_END 0x4017
#define APU_IO_TEST_START 0x4018
#define APU_IO_TEST_END 0x401F
#define UNMAPPED_START 0x4020
#define UNMAPPED_END 0x6000 - UNMAPPED_START
#define CART_RAM_START 0x6000
#define CART_RAM_END 0x7FFF
#define CART_ROM_START 0x8000
#define CART_ROM_END 0xFFFF

#define STACK_START 0x0100

#endif