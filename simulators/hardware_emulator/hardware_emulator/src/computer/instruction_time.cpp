/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "instruction_time.h"

#include "computer/memory.h"
#include "computer/computer.h"
#include <iostream>

int time_table[] = {
#include "timetable/amd_full_table_with_hardcoded_values.table"
};

// stores all the zydis instructions
// this table can be used to retreive instruction mnemonics as std::string
std::string zydis_instructions[] = {
#include "timetable/zydis_instructions.table"
};

// agner tables: https://www.agner.org/optimize/instruction_tables.pdf
// the latency in this table could be more realistic
// the latency of listed instructions were measured by conducting experiments
std::string instruction_table[] = {
#include "timetable/agner_zen2_instructions.table"
};

std::string operands_table[509][4] = {
#include "timetable/agner_zen2_operands.table"
};

double latency_table[509][2] = {
#include "timetable/agner_zen2_latency.table"
};


void CodeDecoder::init( Memory &memory, ComputerTime &computer_time ) {
    this->computer_time = &computer_time;
    this->memory = &memory;
    ZydisDecoderInit( &decoder, ZYDIS_MACHINE_MODE_LONG_64, ZYDIS_ADDRESS_WIDTH_64 );
    ZydisFormatterInit( &formatter, ZYDIS_FORMATTER_STYLE_ATT );
}

bool CodeDecoder::handle_instruction( ulong addr, uint size, ulong &tick_time ) {
    length = size;
    // reads(fetches) instruction from memory
    code = ( uchar * )memory->read_memory( addr, size );
    succeeded = ZYAN_SUCCESS( ZydisDecoderDecodeBuffer( &decoder, code, length, &instruction ) );
    ZydisFormatterFormatInstruction(&formatter, &instruction, buffer.data(), buffer.size(), addr);

    int ticks = 0;
    bool no_val = false;
    if ( succeeded ) {
        ticks = resolve_latency(addr);
        if ( ticks == -1 ) {
            no_val = true;
            time_table[instruction.mnemonic] = 50;
            ticks = 50;
        }
    }
    else {
        ticks = 200;
    }

    computer_time->add_cpu_ticks( ticks );
    tick_time = ticks;
    return no_val;
}

int CodeDecoder::resolve_latency(ulong addr) {
    // count number of operands that are acutally used
    uint operands_count = count_operands();

    // resolve operands of the instruction
    resolve_operands(operands_count);

    //get instruction mnemonic as string with instructions.mnemonic using zydis_instructions.table
    std::string mnemonic = zydis_instructions[instruction.mnemonic];
    // check if the mnemonic is in agner's table
    std::vector<uint> indexes = get_agner_indexes(mnemonic);

    // check is there are matching operands
    bool matching_operands = false;
    int matching_idx;
    for(uint& idx : indexes) {
        // count the number of operands listed in agner's table
        uint agner_operands_count = 0;
        for ( int i = 0 ; i < 4 ; ++i ) {
            if (operands_table[idx][i] == "") break;
            agner_operands_count += 1;
        }

        if ( agner_operands_count == operands_count) {
            bool all_op_match = true;
            for (int i = 0 ; i < operands_count ; ++i) {
                if (operands_table[idx][i] != operands[i]) {
                    // not a perfect matching. check for loose-matching
                    if (operands_table[idx][i] != operands_types[i]) {
                        all_op_match = false;
                    }
                }
            }
            if (all_op_match) {
                matching_operands = true;
                matching_idx = idx;
                break;
            }
        }
    }

    // if there was a  matching operands, return latency measured by agner
    if (matching_operands) {
        int latency = get_latency(matching_idx);
        // if the latency is not defined by agner, return the latency defined by vendor
        if ( latency == -1 ) return time_table[instruction.mnemonic];
        else return latency;
    }

    // return latency value defined by vendors
    return time_table[instruction.mnemonic];
}

// retrieve latency from agner's table
int CodeDecoder::get_latency(int idx) {
    if ( latency_table[idx][0] == -1 ) return -1;

    // check if the latency is defined as a range
    bool is_range = latency_table[idx][1] == 0 ? false : true;

    if ( is_range ) {
        uint min = latency_table[idx][0];
        uint max = latency_table[idx][1];
        // return random number in the range
        return min + ( std::rand() % (max - min + 1) );
    }
    else{
        return latency_table[idx][0];
    }

}

uint CodeDecoder::count_operands() {
    uint operands_count = 0;
    // counting number of operands based on the occurences of ','
    // this will result in ( actual number of operands - 1) operands
    for (char& c : buffer) {
        if (c == ',') {
            operands_count += 1;
        }
        // stop counting when null-terminated
        if (c == '\000') {
            break;
        }
    }

    if ( operands_count == 0 ) {
        // check if the instruction really required no operands
        if ( instruction.operand_count != 0 ) operands_count = 1;
    }
    else {
        operands_count += 1;
    }

    return operands_count;
}

void CodeDecoder::resolve_operands(uint operands_count) {
    // check the agner table first
    operands_types.clear();
    operands.clear();

    for (int i = 0 ; i < operands_count ; ++i) {
        std::string operand_type = ComputerDebug::resolve_operand_type(this, nullptr, instruction.operands[i].type);
        std::string operand_size = ComputerDebug::resolve_operand_size(this, instruction.operands[i].size);
        operands_types.push_back(operand_type);
        operands.push_back(operand_type + operand_size);
    }
}

std::vector<uint> CodeDecoder::get_agner_indexes(std::string &mnemonic) {
    bool in_agner = false;
    std::vector<uint> indexes;
    for (int i = 0 ; i < 509 ; i++) {
        if (mnemonic == instruction_table[i]) {
            in_agner = true;
            indexes.push_back(i);
        }
    }

    return indexes;
}

void MemoryModel::init( Memory &memory, ComputerTime &computer_time ) {
    this->computer_time = &computer_time;
    this->memory = &memory;

    // size is hard coded
    memory_layers.resize( 5 );
    instruction_memory = &base_time;
    data_memory = &base_time;
    mem_layer_count = 0;
}

ulong MemoryModel::handle_access( MemAccess type, ulong addr ) {
    //uint sec_id = memory->section_lookup[MemoryRange( addr, 1 )];
    ulong time = 0;
    switch ( type ) {
        case MemAccess::READ:
            time = data_memory->read( addr );
            break;
        case MemAccess::WRITE:
            time = data_memory->write( addr );
            break;
        case MemAccess::FETCH:
            time = instruction_memory->read( addr );
            break;
    }
    computer_time->add_pico_time( time );
    return time;
}
