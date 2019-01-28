#pragma once
#include "utility.h"
#include <Zydis/Zydis.h>

struct CodeDecoder {
    ZydisDecoder decoder;
    uchar *code;
    uint length;
    bool succeeded;
    ZydisDecodedInstruction instruction;
};

/*
    Returns the number of processor ticks needed for the given instruction.
*/
uint get_instruction_ticks( ZydisDecodedInstruction &instruction );