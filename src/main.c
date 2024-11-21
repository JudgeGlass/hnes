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

    instruction_t *instruction = get_instruction_from_op(cpu.instruction_set, 0x69);
    printf("OP: 0x%X\tINS: %d\tAM: %d\tCC: %d\n", instruction->op_code, instruction->instruction_type, instruction->address_mode, instruction->cycles);

    return 0;
}