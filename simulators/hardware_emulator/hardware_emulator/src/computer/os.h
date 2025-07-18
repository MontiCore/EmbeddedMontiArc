/**
 * (c) https://github.com/MontiCore/monticore
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
        virtual void load_file(const fs::path& file) = 0;
        virtual ~OS() {}

        // Os-independent function calling methods => map to the corresponding function-calling standard (LinuxFastCall or WindowsFastCall)
        virtual ulong get_return_64() = 0;
        virtual void set_param1_32(uint p) = 0;
        virtual void set_param1_64(ulong p) = 0;
        virtual void set_param2_32(uint p) = 0;
        virtual void set_param2_64(ulong p) = 0;
        virtual void set_param3_32(uint p) = 0;
        virtual void set_param3_64(ulong p) = 0;
        virtual void set_param1_double(double p) = 0;

        virtual void set_return_64(ulong r) = 0;
        virtual ulong get_param1_64() = 0;
        virtual ulong get_param2_64() = 0;


        virtual bool uses_shadow_space() = 0;
    };
}
