/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include "utility/utility.h"
#include <Zydis/Zydis.h>
#include "memory.h"



/*
    This structure presents the evaluated computer time.

    It can be configured with CPU and Memory frequencies in order to directly convert tick (cycle) counts
    to time when adding time to the counter.

    It uses a picosecond time to keep precision with high frequencies, but converts the accumulated time
    to microseconds as soon as the picosecond counter is > 1us and keeps the picosecond rest time for precison.
*/
struct ComputerTime {
    bool use_time;
    
    ulong pico_time;
    ulong micro_time;
    ulong cpu_frequency;
    ulong cpu_tick_time_pico;
    ulong memory_frequency;
    ulong memory_tick_time_pico;

    ComputerTime() : micro_time( 0 ), pico_time( 0 ),
        use_time( false ) {
        set_cpu_frequency( 1000000 );
        set_memory_frequency( 100000 );
    }
    
    void set_cpu_frequency( ulong cpu_frequency ) {
        this->cpu_frequency = cpu_frequency;
        this->cpu_tick_time_pico = 1000000000000UL / cpu_frequency;
    }
    
    void set_memory_frequency( ulong memory_frequency ) {
        this->memory_frequency = memory_frequency;
        this->memory_tick_time_pico = 1000000000000UL / memory_frequency;
    }
    
    void add_cpu_ticks( ulong tick_count ) {
        add_pico_time( cpu_tick_time_pico * tick_count );
    }
    
    void add_memory_ticks( ulong tick_count ) {
        add_pico_time( cpu_tick_time_pico * tick_count );
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


/*
    This interface should be implemented by objects that represent intermediates between the processor
    and the main memory. This interface is used to create a chain of layers that can then model timing properties
    of memory read and write operations.

    Implementations must define the read() and write() functions, which must return a delay time in picoseconds.
    The MemoryAccessInterface are linked to another MemoryAccessInterface object that is the next in the
    MemoryAccessInterface list. Implementations must propagate the calls to write().
    In the case of cache hits, the specific cache layer will ignore the time of the next layers but must still call it
    in case of write() operations, to register the address in the next cache levels.
*/
struct MemoryAccessInterface {
        virtual ulong read( ulong address ) = 0;
        virtual ulong write( ulong address ) = 0;
        MemoryAccessInterface() : underlying_memory( nullptr ) {}
        virtual ~MemoryAccessInterface() {}
    protected:
        friend struct MemoryModel;
        MemoryAccessInterface *underlying_memory;
};


/*
    This represents the last element of the MemoryAccessInterface chain that starts
    from the processor. It yields a constant time delay for read and write operations.
*/
struct MemoryTime : public MemoryAccessInterface {
    private:
        ulong read_time;
        ulong write_time;
    public:
        ulong read( ulong address ) {
            return read_time;
        }
        ulong write( ulong address ) {
            return write_time;
        }
        MemoryTime() : read_time( 1000000 ), write_time( 2000000 ) {}
        MemoryTime(ulong read_time, ulong write_time): read_time(read_time), write_time(write_time) {}
        void init( ComputerTime &time, uint read_cas, uint write_cas ) {
            this->read_time = read_cas * time.memory_tick_time_pico;
            this->write_time = write_cas * time.memory_tick_time_pico;
        }
};


/*
    This structure holds the MemoryAccessInterface stack setup and uses it
    when recieving triggers from the code and memory emulation callbacks to evaluate the time
    delay caused by memory access.

    The add_common_interface(), add_instruction_interface() and add_data_interface() are used to build the stack
    of MemoryAccessInterface layers. This stack must be build from the highest (furthest from processor) layer to the nearest.
    add_instruction_interface() and add_data_interface() will register a layer only for memory accesses
    caused specifically by instruction reading or data reading.

    The MemoryModel object takes ownership of the layers added to the stack.
    (Provide allocated objects, not a static ones)

    handle_access() gets called by the callbacks for code execution and memory access.
*/
struct MemoryModel {
    private:
        ComputerTime *computer_time;
        Memory *memory;
        
        MemoryAccessInterface *instruction_memory;
        MemoryAccessInterface *data_memory;
		std::vector<std::unique_ptr< MemoryAccessInterface>> memory_layers;
        uint mem_layer_count;
        
    public:
        MemoryTime base_time;

        uint get_layer_count() {
            return mem_layer_count;
        }

        void register_memory_layer( MemoryAccessInterface *layer ) {
            memory_layers[mem_layer_count++] = std::unique_ptr<MemoryAccessInterface>( layer );
        }

        MemoryAccessInterface* get_memory_layer(int idx) {
            if (idx < mem_layer_count) return memory_layers[idx].get();
            else return nullptr;
        }
        
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
        
        MemoryModel() : instruction_memory( nullptr ), data_memory( nullptr ), mem_layer_count( 0 ), computer_time(nullptr), memory(nullptr) {}
};

struct Memory;
struct ComputerTime;

/*
    The CodeDecoder component evaluates the time delay caused by executing instructions.
    It uses ZyDis to get the mnemonic of the executed instruction. This enum value of the mnemonic is used to
    look up a tick (cycle) count that is added to the ComputerTime component using the CPU-tick=>time conversion.
*/
struct CodeDecoder {
    static constexpr uint BUFFER_SIZE = 0x400;

    Memory *memory;
    ComputerTime *computer_time;
    ZydisDecoder decoder;
    uchar *code;
    uint length;
    bool succeeded;
    ZydisDecodedInstruction instruction;
    ZydisFormatter formatter;

    std::vector<char> buffer;
    std::vector<std::string> operands_types;
    std::vector<std::string> operands;

    CodeDecoder() : computer_time( nullptr ), memory( nullptr ), code( nullptr ), length(0), instruction(), succeeded(false), decoder(),
                    formatter(), buffer(BUFFER_SIZE){}
    
    void init( Memory &memory, ComputerTime &computer_time);
    
    /*
        tick_time must be set to the evaluated tick count.
        returns true if the instruction had no entry in the used time table.
    */
    bool handle_instruction( ulong addr, uint size, ulong &tick_count );

private:
    int resolve_latency(ulong addr);
    int get_latency(int idx);
    uint count_operands();
    void resolve_operands(uint operands_count);
    std::vector<uint> get_agner_indexes(std::string& mnemonic);
};
