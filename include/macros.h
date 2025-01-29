#ifndef __MACROS_H__
#define __MACROS_H__

#include "global.h"

#define DEBUG(string) \
  printf("DEBUG: %s\t - LINE: %d FILE: %s\n", string, __LINE__, __FILE__);

#define DELAY(seconds)                                                      \
  clock_t start_t = clock();                                                \
  while ((clock() / CLOCKS_PER_SEC) < (start_t / CLOCKS_PER_SEC) + seconds) \
    ;

#endif