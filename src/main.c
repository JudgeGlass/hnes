#include <stdio.h>

#include "macros.h"

#include "system.h"
#include "cpu.h"

int main()
{
    log("hnes - Copyright (c) 2024 Hunter Wilcox");
    log("Starting hnes...");

    init_system();

    cpu_t cpu;
    cpu_init(&cpu);
    cpu_loop(&cpu);

    return 0;
}