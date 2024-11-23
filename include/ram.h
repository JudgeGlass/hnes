#ifndef __RAM_H__
#define __RAM_H__

#include "macros.h"
#include "global.h"

typedef struct
{
  uint16_t start;
  uint16_t end;
  uint16_t size;
  uint8_t *buffer;
} sys_ram_t;

typedef struct
{
  uint16_t start;
  uint16_t end;
  uint16_t size;
  uint8_t *buffer;
} cart_ram_t;

void init_ram(sys_ram_t *sys_ram);
void init_cart_ram(cart_ram_t *cart_ram);

#endif