#include "instruction_time.h"

uint get_instruction_ticks( ZydisDecodedInstruction &instruction ) {
    switch ( instruction.mnemonic ) {
        default: return 1;
    }
    return 0;
}
