#include "instruction_time.h"

#include "computer/memory.h"
#include "computer/computer.h"

uint get_instruction_ticks( ZydisDecodedInstruction &instruction ) {
    switch ( instruction.mnemonic ) {
        case ZYDIS_MNEMONIC_MOV: return 1;
        case ZYDIS_MNEMONIC_MOVNTI: return 2;
        case ZYDIS_MNEMONIC_MOVZX:
        case ZYDIS_MNEMONIC_MOVSXD:
        case ZYDIS_MNEMONIC_MOVSX:
            return 1;
        case ZYDIS_MNEMONIC_MOVSD:
            return 5;
        case ZYDIS_MNEMONIC_PUSH: return 2;
        case ZYDIS_MNEMONIC_SUB: return 1;
        case ZYDIS_MNEMONIC_ADD: return 1;
        case ZYDIS_MNEMONIC_CMP: return 1;
        case ZYDIS_MNEMONIC_JB:
        case ZYDIS_MNEMONIC_JBE:
        case ZYDIS_MNEMONIC_JCXZ:
        case ZYDIS_MNEMONIC_JECXZ:
        case ZYDIS_MNEMONIC_JKNZD:
        case ZYDIS_MNEMONIC_JKZD:
        case ZYDIS_MNEMONIC_JL:
        case ZYDIS_MNEMONIC_JLE:
        case ZYDIS_MNEMONIC_JMP:
        case ZYDIS_MNEMONIC_JNB:
        case ZYDIS_MNEMONIC_JNBE:
        case ZYDIS_MNEMONIC_JNL:
        case ZYDIS_MNEMONIC_JNLE:
        case ZYDIS_MNEMONIC_JNO:
        case ZYDIS_MNEMONIC_JNP:
        case ZYDIS_MNEMONIC_JNS:
        case ZYDIS_MNEMONIC_JNZ:
        case ZYDIS_MNEMONIC_JO:
        case ZYDIS_MNEMONIC_JP:
        case ZYDIS_MNEMONIC_JRCXZ:
        case ZYDIS_MNEMONIC_JS:
        case ZYDIS_MNEMONIC_JZ:
            return 1;
        case ZYDIS_MNEMONIC_CALL: return 2;
        case ZYDIS_MNEMONIC_XOR: return 1;
        case ZYDIS_MNEMONIC_LEA: return 1;
        case ZYDIS_MNEMONIC_AND: return 1;
        case ZYDIS_MNEMONIC_OR: return 1;
        case ZYDIS_MNEMONIC_SHR:
        case ZYDIS_MNEMONIC_SHL:
            return 1;
        case ZYDIS_MNEMONIC_NOT: return 1;
        case ZYDIS_MNEMONIC_CMOVB:
        case ZYDIS_MNEMONIC_CMOVBE:
        case ZYDIS_MNEMONIC_CMOVL:
        case ZYDIS_MNEMONIC_CMOVLE:
        case ZYDIS_MNEMONIC_CMOVNB:
        case ZYDIS_MNEMONIC_CMOVNBE:
        case ZYDIS_MNEMONIC_CMOVNL:
        case ZYDIS_MNEMONIC_CMOVNLE:
        case ZYDIS_MNEMONIC_CMOVNO:
        case ZYDIS_MNEMONIC_CMOVNP:
        case ZYDIS_MNEMONIC_CMOVNS:
        case ZYDIS_MNEMONIC_CMOVNZ:
        case ZYDIS_MNEMONIC_CMOVO:
        case ZYDIS_MNEMONIC_CMOVP:
        case ZYDIS_MNEMONIC_CMOVS:
        case ZYDIS_MNEMONIC_CMOVZ:
            return 1;
        case ZYDIS_MNEMONIC_POP:
            return 1;
        case ZYDIS_MNEMONIC_RET:
            return 1;
        case ZYDIS_MNEMONIC_TEST:
            return 1;
        case ZYDIS_MNEMONIC_CPUID:
            return 118;
        case ZYDIS_MNEMONIC_BT:
            return 1;
        case ZYDIS_MNEMONIC_STOSB:
        case ZYDIS_MNEMONIC_STOSD:
        case ZYDIS_MNEMONIC_STOSQ:
        case ZYDIS_MNEMONIC_STOSW:
            return 3;
        case ZYDIS_MNEMONIC_XCHG:
            return 8;
        case ZYDIS_MNEMONIC_IMUL:
            return 2;
        case ZYDIS_MNEMONIC_ROL:
        case ZYDIS_MNEMONIC_ROR:
            return 5;
        case ZYDIS_MNEMONIC_SBB:
            return 4;
        case ZYDIS_MNEMONIC_NOP:
            return 1;
        case ZYDIS_MNEMONIC_INC:
            return 3;
        case ZYDIS_MNEMONIC_NEG:
            return 2;
        case ZYDIS_MNEMONIC_SETB:
        case ZYDIS_MNEMONIC_SETBE:
        case ZYDIS_MNEMONIC_SETL:
        case ZYDIS_MNEMONIC_SETLE:
        case ZYDIS_MNEMONIC_SETNB:
        case ZYDIS_MNEMONIC_SETNBE:
        case ZYDIS_MNEMONIC_SETNL:
        case ZYDIS_MNEMONIC_SETNLE:
        case ZYDIS_MNEMONIC_SETNO:
        case ZYDIS_MNEMONIC_SETNP:
        case ZYDIS_MNEMONIC_SETNS:
        case ZYDIS_MNEMONIC_SETNZ:
        case ZYDIS_MNEMONIC_SETO:
        case ZYDIS_MNEMONIC_SETP:
        case ZYDIS_MNEMONIC_SETS:
        case ZYDIS_MNEMONIC_SETSSBSY:
        case ZYDIS_MNEMONIC_SETZ:
            return 1;
        case ZYDIS_MNEMONIC_MOVQ:
            return 1;
        case ZYDIS_MNEMONIC_CDQE:
            return 1;
        case ZYDIS_MNEMONIC_MOVAPS:
            return 1;
        case ZYDIS_MNEMONIC_CMPXCHG:
            return 10;
        case ZYDIS_MNEMONIC_DIVSD:
            return 1;
        case ZYDIS_MNEMONIC_FLD:
            return 1;
        case ZYDIS_MNEMONIC_FSTP:
            return 1;
        case ZYDIS_MNEMONIC_FLDLN2:
            return 2;
        case ZYDIS_MNEMONIC_FSUB:
        case ZYDIS_MNEMONIC_FADD:
            return 1;
        case ZYDIS_MNEMONIC_FABS:
            return 1;
        case ZYDIS_MNEMONIC_FCOMP:
            return 1;
        default: return 1000;
    }
    return 0;
}

void CodeDecoder::init( Memory &memory, ComputerTime &computer_time ) {
    this->computer_time = &computer_time;
    this->memory = &memory;
    ZydisDecoderInit( &decoder, ZYDIS_MACHINE_MODE_LONG_64, ZYDIS_ADDRESS_WIDTH_64 );
}

ulong CodeDecoder::handle_instruction( ulong addr, uint size ) {
    length = size;
    code = ( uchar * )memory->read_memory( addr, size );
    succeeded = ZYAN_SUCCESS( ZydisDecoderDecodeBuffer( &decoder, code, length, &instruction ) );
    ulong ticks = 1000;
    if ( succeeded )
        ticks = get_instruction_ticks( instruction );
    computer_time->add_ticks( ticks );
    return ticks;
}

void MemoryModel::init( Memory &memory, ComputerTime &computer_time ) {
    this->computer_time = &computer_time;
    this->memory = &memory;
    
    memory_layers.init( 5 );
    instruction_memory = &base_time;
    data_memory = &base_time;
    mem_layer_count = 0;
}

ulong MemoryModel::handle_access( MemAccess type, ulong addr ) {
    uint sec_id = memory->section_lookup[MemoryRange( addr, 1 )];
    ulong time = 0;
    switch ( type ) {
        case MemAccess::READ:
            time = data_memory->read( addr, sec_id );
            break;
        case MemAccess::WRITE:
            time = data_memory->write( addr, sec_id );
            break;
        case MemAccess::FETCH:
            time = instruction_memory->read( addr, sec_id );
            break;
    }
    computer_time->add_pico_time( time );
    return time;
}
