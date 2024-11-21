#include <stdio.h>

#include "macros.h"

#include "rom.h"

int main()
{
    log("hnes - Copyright (c) 2024 Hunter Wilcox");
    log("Starting hnes...");

    ines_t rom_meta;
    load_rom("/home/hwilcox/Documents/hnes/mario.nes", &rom_meta);

    return 0;
}