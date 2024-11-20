#ifndef __MACROS_H__
#define __MACROS_H__

#include "global.h"

#define log(string) \
  printf("%s - LINE: %d FILE: %s\n", string, __LINE__, __FILE__);

#endif