#ifndef TEST
#include <stdio.h>

#include "macros.h"

#include "system.h"
#include "cpu.h"

int main()
{
    printf("hnes - Copyright (c) 2024 Hunter Wilcox\n");
    printf("Starting hnes...\n");

    init_system();

    cpu_t cpu;
    cpu_init(&cpu);
    cpu_loop(&cpu);

    return 0;
}
#endif