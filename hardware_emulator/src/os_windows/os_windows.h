#pragma once
#include "dll_loader.h"
#include "computer/os.h"
#include "computer/function_calling.h"

namespace OS {

    /*
    A Simple wrapper for the Fast Call standard, putting the parameters of a function call in the
    correct registers.
    Also give the return value from function calls.
    All the parameters and return values are 64bit unsigned longs, which are read/copied as-is
    in the registers.
    */
    struct WindowsFastCall : public FunctionCalling {
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
        
        WindowsFastCall( Registers &registers ) : registers( registers ) {}
    };
    
    
    struct Windows : public OS {
        MemorySection *section;
        SectionStack *section_stack;
        
        DLLLoader dll;
        
        MemoryRange cmd_line_wstr;
        MemoryRange cmd_line_str;
        
        MemoryRange io_stdin;
        MemoryRange io_stdout;
        MemoryRange io_stderr;
        
        Array<char> name_buffer;
        
        Windows() : name_buffer( 1024 ) {}
        
        void init( Computer &computer );
        
        //File without extension
        bool load_file( const char *file );
        
    };
    
}