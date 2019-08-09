/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
#include "instruction_time.h"

#include "computer/memory.h"
#include "computer/computer.h"

uint time_table[] = {
#include "timetable/skylake.table"
};


void CodeDecoder::init( Memory &memory, ComputerTime &computer_time ) {
    this->computer_time = &computer_time;
    this->memory = &memory;
    ZydisDecoderInit( &decoder, ZYDIS_MACHINE_MODE_LONG_64, ZYDIS_ADDRESS_WIDTH_64 );
}

bool CodeDecoder::handle_instruction( ulong addr, uint size, ulong &tick_time ) {
    length = size;
    code = ( uchar * )memory->read_memory( addr, size );
    succeeded = ZYAN_SUCCESS( ZydisDecoderDecodeBuffer( &decoder, code, length, &instruction ) );
    long ticks = 0;
    bool no_val = false;
    if ( succeeded ) {
        ticks = time_table[instruction.mnemonic];
        if ( ticks <= 0 ) {
            if ( ticks == 0 ) {
                no_val = true;
                time_table[instruction.mnemonic] = -1; //Mark the first time
            }
            ticks = 200;
        }
        
    }
    else
        ticks = 200;
        
    computer_time->add_cpu_ticks( ticks );
    tick_time = ticks;
    return no_val;
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
