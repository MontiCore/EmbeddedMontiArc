#pragma once
#include "computer/os.h"
#include "computer/function_calling.h"
#include "elf_loader.h"

namespace OS {

    struct LinuxFastCall : public FunctionCalling {
        Registers &registers;
        
        void set_params( ulong p1 );
        void set_params( ulong p1, ulong p2 );
        void set_params( ulong p1, ulong p2, ulong p3 );
        void set_params( ulong p1, ulong p2, ulong p3, ulong p4 );
        
        ulong get_param1();
        ulong get_param2();
        ulong get_param3();
        ulong get_param4();
        
        ulong get_return();
        void set_return( ulong r );
        
        LinuxFastCall( Registers &registers ) : registers( registers ) {}
    };
    
    struct Linux : public OS {
        MemorySection *section;
        SectionStack *section_stack;
        
        
        ElfLoader elf;
        
        Array<char> name_buffer;
        
        Linux() : name_buffer( 1024 ) {}
        
        void init( Computer &computer );
        
        //File without extension
        bool load_file( const char *file );
        
    };
}