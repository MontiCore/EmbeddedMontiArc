#pragma once
#include "dll_loader.h"
#include "computer/computer.h"

namespace OS {
    extern MemoryRange io_slot;
    struct Windows {
        Computer *computer;
        MemorySection *section;
        SectionStack section_stack;
        
        DLLLoader dll;
        
        MemoryRange cmd_line_wstr;
        MemoryRange cmd_line_str;
        
        MemoryRange io_stdin;
        MemoryRange io_stdout;
        MemoryRange io_stderr;
        
        Array<char> name_buffer;
        
        Windows() : computer( nullptr ), name_buffer( 1024 ) {}
        
        void init( Computer &computer );
        
        bool loaded() {
            return computer != nullptr;
        }
        
        bool load_dll( const char *file );
        
        ulong add_symbol( const std::string &mod, const std::string &name, uint64_t size,
                          Annotation::Type type = Annotation::SYMBOL );
    };
    
    
    
    
    
}