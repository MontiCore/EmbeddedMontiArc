#pragma once
#include <string>
struct Computer;
struct MemorySection;

namespace OS {

    struct SectionInfo {
        MemorySection *mem;
    };
    
    struct OS {
        Computer *computer;
        
        OS() : computer( nullptr ) {}
        
        virtual void init( Computer &computer ) = 0;
        
        
        bool loaded() {
            return computer != nullptr;
        }
        
        //File without extension
        virtual bool load_file( const char *file ) = 0;
        virtual ~OS() {}
    };
}