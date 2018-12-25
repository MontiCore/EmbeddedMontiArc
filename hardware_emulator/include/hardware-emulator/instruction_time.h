#pragma once
#include "utility.h"
#include <Zydis/Zydis.h>



uint get_instruction_ticks( ZydisDecodedInstruction &instruction );