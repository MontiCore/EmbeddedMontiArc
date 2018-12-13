#pragma once
#include "dll_loader.h"
#include "computer/computer.h"

namespace OS {

    struct Windows {
        Computer *computer;
        MemorySection *section;
        SectionStack section_stack;
        
        DLLLoader dll;
        
        MemoryRange cmd_line_wstr;
        MemoryRange cmd_line_str;
        
        Windows() : computer( nullptr ) {}
        
        void init( Computer &computer );
        
        bool loaded() {
            return computer != nullptr;
        }
        
        bool load_dll( const char *file );
        
    };
    
    
    
    
    
}