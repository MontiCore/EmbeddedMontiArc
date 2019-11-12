/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#pragma once
#include <string>
#include "utility/utility.h"
struct Computer;
struct MemorySection;

/*
    This interface must be implemented by operating system emulations.
    The init() function must register an implementation of the FunctionCalling interface.
*/
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
        virtual void load_file(const FS::File& file) = 0;
        virtual ~OS() {}
    };
}
