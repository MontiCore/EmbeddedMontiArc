#pragma once
#include "utility.h"
#include <Zydis/Zydis.h>
#include "memory.h"

struct ComputerTime {
    bool use_time;
    ulong pico_time;
    ulong micro_time;
    ulong cpu_frequency;
    ulong tick_time_pico;
    
    ComputerTime() : micro_time( 0 ), cpu_frequency( 1000000 ), pico_time( 0 ), tick_time_pico( 0 ), use_time( false ) {}
    
    void set_frequency( ulong cpu_frequency ) {
        this->cpu_frequency = cpu_frequency;
        this->tick_time_pico = 1000000000000UL / cpu_frequency;
    }
    
    void add_ticks( ulong tick_count ) {
        add_pico_time( tick_time_pico * tick_count );
    }
    
    void add_pico_time( ulong delta ) {
        pico_time += delta;
        if ( pico_time >= 1000000 ) {
            micro_time += pico_time / 1000000;
            pico_time %= 1000000;
        }
    }
    
    void reset() {
        pico_time = 0;
        micro_time = 0;
    }
};



struct MemoryAccessInterface {
    virtual ulong read( ulong address, uint sec_id ) = 0;
    virtual ulong write( ulong address, uint sec_id ) = 0;
    MemoryAccessInterface *underlying_memory;
    MemoryAccessInterface() : underlying_memory( nullptr ) {}
    virtual ~MemoryAccessInterface() {}
};

struct MemoryTime : public MemoryAccessInterface {
    ulong read_time;
    ulong write_time;
    ulong read( ulong address, uint sec_id ) {
        return read_time;
    }
    ulong write( ulong address, uint sec_id ) {
        return write_time;
    }
    MemoryTime() : read_time( 1000000 ), write_time( 2000000 ) {}
    void init( ulong read_time, ulong write_time ) {
        this->read_time = read_time;
        this->write_time = write_time;
    }
};

struct MemoryModel {
    ComputerTime *computer_time;
    Memory *memory;
    
    MemoryTime base_time;
    MemoryAccessInterface *instruction_memory;
    MemoryAccessInterface *data_memory;
    Array<std::unique_ptr< MemoryAccessInterface>> memory_layers;
    uint mem_layer_count;
    void register_memory_layer( MemoryAccessInterface *layer ) {
        memory_layers[mem_layer_count++] = std::unique_ptr<MemoryAccessInterface>( layer );
    }
    
    //Use these functions to link memory access interfaces from furthest to nearest (L3 -> L1 cache)
    void add_common_interface( MemoryAccessInterface *mem_access ) {
        register_memory_layer( mem_access );
        mem_access->underlying_memory = data_memory;
        data_memory = mem_access;
        instruction_memory = mem_access;
    }
    void add_instruction_interface( MemoryAccessInterface *mem_access ) {
        register_memory_layer( mem_access );
        mem_access->underlying_memory = instruction_memory;
        instruction_memory = mem_access;
    }
    void add_data_interface( MemoryAccessInterface *mem_access ) {
        register_memory_layer( mem_access );
        mem_access->underlying_memory = data_memory;
        data_memory = mem_access;
    }
    
    void init( Memory &memory, ComputerTime &computer_time );
    
    ulong handle_access( MemAccess type, ulong addr );
    
    MemoryModel() : instruction_memory( nullptr ), data_memory( nullptr ), mem_layer_count( 0 ) {}
};

struct Memory;
struct ComputerTime;
struct CodeDecoder {
    Memory *memory;
    ComputerTime *computer_time;
    ZydisDecoder decoder;
    uchar *code;
    uint length;
    bool succeeded;
    ZydisDecodedInstruction instruction;
    
    CodeDecoder() : computer_time( nullptr ), memory( nullptr ), code( nullptr ) {}
    
    void init( Memory &memory, ComputerTime &computer_time );
    
    ulong handle_instruction( ulong addr, uint size );
};

/*
    Returns the number of processor ticks needed for the given instruction.
*/
uint get_instruction_ticks( ZydisDecodedInstruction &instruction );