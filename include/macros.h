#ifndef __MACROS_H__
#define __MACROS_H__

#define DISABLE_DEBUG 0

#include "global.h"

#define DEBUG(string) \
  if (!DISABLE_DEBUG) \
    printf("DEBUG: %s\t - LINE: %d FILE: %s\n", string, __LINE__, __FILE__);

#define DELAY(seconds)                                                      \
  clock_t start_t = clock();                                                \
  while ((clock() / CLOCKS_PER_SEC) < (start_t / CLOCKS_PER_SEC) + seconds) \
    ;

#define TEST_MARKER(test) \
  printf("Running test: %s", test);

#define RUN_TEST(test_function) \
  test_function();              \
  printf(" -- PASSED\n");

#endif