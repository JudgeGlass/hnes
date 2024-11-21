#include <stdio.h>

#include "macros.h"

#include "rom.h"
#include "cpu.h"

int main()
{
    log("hnes - Copyright (c) 2024 Hunter Wilcox");
    log("Starting hnes...");

    ines_t rom_meta;
    load_rom("/home/hwilcox/Documents/hnes/mario.nes", &rom_meta);

    cpu_t cpu;
    cpu_init(&cpu);

    return 0;
}