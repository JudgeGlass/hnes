#include "cpu.h"

void cpu_init(cpu_t *cpu)
{
  init_instruction_set(cpu);
}

instruction_t *get_instruction_from_op(instruction_t *instruction_set, const uint8_t op_code)
{
  for (int i = 0; i < INSTRUCTION_SET_SIZE; i++)
  {
    if (instruction_set[i].op_code == op_code)
    {
      return &instruction_set[i];
    }
  }
}

void init_instruction_set(cpu_t *cpu)
{
  cpu->instruction_set[0] = (const instruction_t){0x69, ADC, IMM, 2};
  cpu->instruction_set[1] = (const instruction_t){0x65, ADC, ZP, 3};
  cpu->instruction_set[2] = (const instruction_t){0x75, ADC, ZPX, 4};
  cpu->instruction_set[3] = (const instruction_t){0x6d, ADC, ABS, 4};
  cpu->instruction_set[4] = (const instruction_t){0x7d, ADC, ABSX, 4};
  cpu->instruction_set[5] = (const instruction_t){0x79, ADC, ABSY, 4};
  cpu->instruction_set[6] = (const instruction_t){0x61, ADC, INDX, 6};
  cpu->instruction_set[7] = (const instruction_t){0x71, ADC, INDY, 5};
  cpu->instruction_set[8] = (const instruction_t){0x29, AND, IMM, 2};
  cpu->instruction_set[9] = (const instruction_t){0x25, AND, ZP, 3};
  cpu->instruction_set[10] = (const instruction_t){0x35, AND, ZPX, 4};
  cpu->instruction_set[11] = (const instruction_t){0x2d, AND, ABS, 4};
  cpu->instruction_set[12] = (const instruction_t){0x3d, AND, ABSX, 4};
  cpu->instruction_set[13] = (const instruction_t){0x39, AND, ABSY, 4};
  cpu->instruction_set[14] = (const instruction_t){0x21, AND, INDX, 6};
  cpu->instruction_set[15] = (const instruction_t){0x31, AND, INDY, 5};
  cpu->instruction_set[16] = (const instruction_t){0x0a, ASL, ACC, 2};
  cpu->instruction_set[17] = (const instruction_t){0x06, ASL, ZP, 5};
  cpu->instruction_set[18] = (const instruction_t){0x16, ASL, ZPX, 6};
  cpu->instruction_set[19] = (const instruction_t){0x0e, ASL, ABS, 6};
  cpu->instruction_set[20] = (const instruction_t){0x1e, ASL, ABSX, 7};
  cpu->instruction_set[21] = (const instruction_t){0x24, BIT, ZP, 3};
  cpu->instruction_set[22] = (const instruction_t){0x2c, BIT, ABS, 4};
  cpu->instruction_set[23] = (const instruction_t){0x10, BPL, IMM, 2};
  cpu->instruction_set[24] = (const instruction_t){0x30, BMI, IMM, 2};
  cpu->instruction_set[25] = (const instruction_t){0x50, BVC, IMM, 2};
  cpu->instruction_set[26] = (const instruction_t){0x70, BVS, IMM, 2};
  cpu->instruction_set[27] = (const instruction_t){0x90, BCC, IMM, 2};
  cpu->instruction_set[28] = (const instruction_t){0xb0, BCS, IMM, 2};
  cpu->instruction_set[29] = (const instruction_t){0xd0, BNE, IMM, 2};
  cpu->instruction_set[30] = (const instruction_t){0xf0, BEQ, IMM, 2};
  cpu->instruction_set[31] = (const instruction_t){0x18, CLC, IMM, 2};
  cpu->instruction_set[32] = (const instruction_t){0x38, SEC, IMM, 2};
  cpu->instruction_set[33] = (const instruction_t){0x58, CLI, IMM, 2};
  cpu->instruction_set[34] = (const instruction_t){0x78, SEI, IMM, 2};
  cpu->instruction_set[35] = (const instruction_t){0xb8, CLV, IMM, 2};
  cpu->instruction_set[36] = (const instruction_t){0xd8, CLD, IMM, 2};
  cpu->instruction_set[37] = (const instruction_t){0xf8, SED, IMM, 2};
  cpu->instruction_set[38] = (const instruction_t){0xaa, TAX, IMM, 2};
  cpu->instruction_set[39] = (const instruction_t){0x8a, TXA, IMM, 2};
  cpu->instruction_set[40] = (const instruction_t){0xca, DEX, IMM, 2};
  cpu->instruction_set[41] = (const instruction_t){0xe8, INX, IMM, 2};
  cpu->instruction_set[42] = (const instruction_t){0xa8, TAY, IMM, 2};
  cpu->instruction_set[43] = (const instruction_t){0x98, TYA, IMM, 2};
  cpu->instruction_set[44] = (const instruction_t){0x88, DEY, IMM, 2};
  cpu->instruction_set[45] = (const instruction_t){0xc8, INY, IMM, 2};
  cpu->instruction_set[46] = (const instruction_t){0x9a, TXS, IMM, 2};
  cpu->instruction_set[47] = (const instruction_t){0xba, TSX, IMM, 2};
  cpu->instruction_set[48] = (const instruction_t){0x48, PHA, IMM, 3};
  cpu->instruction_set[49] = (const instruction_t){0x68, PLA, IMM, 4};
  cpu->instruction_set[50] = (const instruction_t){0x08, PHP, IMM, 3};
  cpu->instruction_set[51] = (const instruction_t){0x28, PLP, IMM, 4};
  cpu->instruction_set[52] = (const instruction_t){0x00, BRK, IMP, 7};
  cpu->instruction_set[53] = (const instruction_t){0x40, RTI, IMP, 6};
  cpu->instruction_set[54] = (const instruction_t){0x60, RTS, IMP, 6};
  cpu->instruction_set[55] = (const instruction_t){0xea, NOP, IMM, 2};
  cpu->instruction_set[56] = (const instruction_t){0xc9, CMP, IMM, 2};
  cpu->instruction_set[57] = (const instruction_t){0xc5, CMP, ZP, 3};
  cpu->instruction_set[58] = (const instruction_t){0xd5, CMP, ZPX, 4};
  cpu->instruction_set[59] = (const instruction_t){0xcd, CMP, ABS, 4};
  cpu->instruction_set[60] = (const instruction_t){0xdd, CMP, ABSX, 4};
  cpu->instruction_set[61] = (const instruction_t){0xd9, CMP, ABSY, 4};
  cpu->instruction_set[62] = (const instruction_t){0xc1, CMP, INDX, 6};
  cpu->instruction_set[63] = (const instruction_t){0xd1, CMP, INDY, 5};
  cpu->instruction_set[64] = (const instruction_t){0xe0, CPX, IMM, 2};
  cpu->instruction_set[65] = (const instruction_t){0xe4, CPX, ZP, 3};
  cpu->instruction_set[66] = (const instruction_t){0xec, CPX, ABS, 4};
  cpu->instruction_set[67] = (const instruction_t){0xc0, CPY, IMM, 2};
  cpu->instruction_set[68] = (const instruction_t){0xc4, CPY, ZP, 3};
  cpu->instruction_set[69] = (const instruction_t){0xcc, CPY, ABS, 4};
  cpu->instruction_set[70] = (const instruction_t){0xc6, DEC, ZP, 5};
  cpu->instruction_set[71] = (const instruction_t){0xd6, DEC, ZPX, 6};
  cpu->instruction_set[72] = (const instruction_t){0xce, DEC, ABS, 6};
  cpu->instruction_set[73] = (const instruction_t){0xde, DEC, ABSX, 7};
  cpu->instruction_set[74] = (const instruction_t){0x49, EOR, IMM, 2};
  cpu->instruction_set[75] = (const instruction_t){0x45, EOR, ZP, 3};
  cpu->instruction_set[76] = (const instruction_t){0x55, EOR, ZPX, 4};
  cpu->instruction_set[77] = (const instruction_t){0x4d, EOR, ABS, 4};
  cpu->instruction_set[78] = (const instruction_t){0x5d, EOR, ABSX, 4};
  cpu->instruction_set[79] = (const instruction_t){0x59, EOR, ABSY, 4};
  cpu->instruction_set[80] = (const instruction_t){0x41, EOR, INDX, 6};
  cpu->instruction_set[81] = (const instruction_t){0x51, EOR, INDY, 5};
  cpu->instruction_set[82] = (const instruction_t){0xe6, INC, ZP, 5};
  cpu->instruction_set[83] = (const instruction_t){0xf6, INC, ZPX, 6};
  cpu->instruction_set[84] = (const instruction_t){0xee, INC, ABS, 6};
  cpu->instruction_set[85] = (const instruction_t){0xfe, INC, ABSX, 7};
  cpu->instruction_set[86] = (const instruction_t){0x4c, JMP, ABS, 3};
  cpu->instruction_set[87] = (const instruction_t){0x6c, JMP, IND, 5};
  cpu->instruction_set[88] = (const instruction_t){0x20, JSR, ABS, 6};
  cpu->instruction_set[89] = (const instruction_t){0xa9, LDA, IMM, 2};
  cpu->instruction_set[90] = (const instruction_t){0xa5, LDA, ZP, 3};
  cpu->instruction_set[91] = (const instruction_t){0xb5, LDA, ZPX, 4};
  cpu->instruction_set[92] = (const instruction_t){0xad, LDA, ABS, 4};
  cpu->instruction_set[93] = (const instruction_t){0xbd, LDA, ABSX, 4};
  cpu->instruction_set[94] = (const instruction_t){0xb9, LDA, ABSY, 4};
  cpu->instruction_set[95] = (const instruction_t){0xa1, LDA, INDX, 6};
  cpu->instruction_set[96] = (const instruction_t){0xb1, LDA, INDY, 5};
  cpu->instruction_set[97] = (const instruction_t){0xa2, LDX, IMM, 2};
  cpu->instruction_set[98] = (const instruction_t){0xa6, LDX, ZP, 3};
  cpu->instruction_set[99] = (const instruction_t){0xb6, LDX, ZPY, 4};
  cpu->instruction_set[100] = (const instruction_t){0xae, LDX, ABS, 4};
  cpu->instruction_set[101] = (const instruction_t){0xbe, LDX, ABSY, 4};
  cpu->instruction_set[102] = (const instruction_t){0xa0, LDY, IMM, 2};
  cpu->instruction_set[103] = (const instruction_t){0xa4, LDY, ZP, 3};
  cpu->instruction_set[104] = (const instruction_t){0xb4, LDY, ZPX, 4};
  cpu->instruction_set[105] = (const instruction_t){0xac, LDY, ABS, 4};
  cpu->instruction_set[106] = (const instruction_t){0xbc, LDY, ABSX, 4};
  cpu->instruction_set[107] = (const instruction_t){0x4a, LSR, ACC, 2};
  cpu->instruction_set[108] = (const instruction_t){0x46, LSR, ZP, 5};
  cpu->instruction_set[109] = (const instruction_t){0x56, LSR, ZPX, 6};
  cpu->instruction_set[110] = (const instruction_t){0x4e, LSR, ABS, 6};
  cpu->instruction_set[111] = (const instruction_t){0x5e, LSR, ABSX, 7};
  cpu->instruction_set[112] = (const instruction_t){0x09, ORA, IMM, 2};
  cpu->instruction_set[113] = (const instruction_t){0x05, ORA, ZP, 3};
  cpu->instruction_set[114] = (const instruction_t){0x15, ORA, ZPX, 4};
  cpu->instruction_set[115] = (const instruction_t){0x0d, ORA, ABS, 4};
  cpu->instruction_set[116] = (const instruction_t){0x1d, ORA, ABSX, 4};
  cpu->instruction_set[117] = (const instruction_t){0x19, ORA, ABSY, 4};
  cpu->instruction_set[118] = (const instruction_t){0x01, ORA, INDX, 6};
  cpu->instruction_set[119] = (const instruction_t){0x11, ORA, INDY, 5};
  cpu->instruction_set[120] = (const instruction_t){0x2a, ROL, ACC, 2};
  cpu->instruction_set[121] = (const instruction_t){0x26, ROL, ZP, 5};
  cpu->instruction_set[122] = (const instruction_t){0x36, ROL, ZPX, 6};
  cpu->instruction_set[123] = (const instruction_t){0x2e, ROL, ABS, 6};
  cpu->instruction_set[124] = (const instruction_t){0x3e, ROL, ABSX, 7};
  cpu->instruction_set[125] = (const instruction_t){0x6a, ROR, ACC, 2};
  cpu->instruction_set[126] = (const instruction_t){0x66, ROR, ZP, 5};
  cpu->instruction_set[127] = (const instruction_t){0x76, ROR, ZPX, 6};
  cpu->instruction_set[128] = (const instruction_t){0x6e, ROR, ABS, 6};
  cpu->instruction_set[129] = (const instruction_t){0x7e, ROR, ABSX, 7};
  cpu->instruction_set[130] = (const instruction_t){0xe9, SBC, IMM, 2};
  cpu->instruction_set[131] = (const instruction_t){0xe5, SBC, ZP, 3};
  cpu->instruction_set[132] = (const instruction_t){0xf5, SBC, ZPX, 4};
  cpu->instruction_set[133] = (const instruction_t){0xed, SBC, ABS, 4};
  cpu->instruction_set[134] = (const instruction_t){0xfd, SBC, ABSX, 4};
  cpu->instruction_set[135] = (const instruction_t){0xf9, SBC, ABSY, 4};
  cpu->instruction_set[136] = (const instruction_t){0xe1, SBC, INDX, 6};
  cpu->instruction_set[137] = (const instruction_t){0xf1, SBC, INDY, 5};
  cpu->instruction_set[138] = (const instruction_t){0x85, STA, ZP, 3};
  cpu->instruction_set[139] = (const instruction_t){0x95, STA, ZPX, 4};
  cpu->instruction_set[140] = (const instruction_t){0x8d, STA, ABS, 4};
  cpu->instruction_set[141] = (const instruction_t){0x9d, STA, ABSX, 5};
  cpu->instruction_set[142] = (const instruction_t){0x99, STA, ABSY, 5};
  cpu->instruction_set[143] = (const instruction_t){0x81, STA, INDX, 6};
  cpu->instruction_set[144] = (const instruction_t){0x91, STA, INDY, 6};
  cpu->instruction_set[145] = (const instruction_t){0x86, STX, ZP, 3};
  cpu->instruction_set[146] = (const instruction_t){0x96, STX, ZPY, 4};
  cpu->instruction_set[147] = (const instruction_t){0x8e, STX, ABS, 4};
  cpu->instruction_set[148] = (const instruction_t){0x84, STY, ZP, 3};
  cpu->instruction_set[149] = (const instruction_t){0x94, STY, ZPX, 4};
  cpu->instruction_set[150] = (const instruction_t){0x8c, STY, ABS, 4};
}