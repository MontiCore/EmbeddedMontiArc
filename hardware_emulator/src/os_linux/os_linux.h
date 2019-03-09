#pragma once
#include "computer/os.h"
#include "computer/function_calling.h"
#include "elf_loader.h"

namespace OS {

    struct LinuxFastCall : public FunctionCalling {
        Registers &registers;
        
        //Caller
        void set_params_64( ulong p1 );
        void set_params_64( ulong p1, ulong p2 );
        void set_params_64( ulong p1, ulong p2, ulong p3 );
        void set_params_64( ulong p1, ulong p2, ulong p3, ulong p4 );
        
        void set_params_32( uint p1 );
        void set_params_32( uint p1, uint p2 );
        void set_params_32( uint p1, uint p2, uint p3 );
        void set_params_32( uint p1, uint p2, uint p3, uint p4 );
        
        void set_param1_64( ulong p );
        void set_param2_64( ulong p );
        void set_param3_64( ulong p );
        void set_param4_64( ulong p );
        
        void set_param1_32( uint p );
        void set_param2_32( uint p );
        void set_param3_32( uint p );
        void set_param4_32( uint p );
        
        void set_param1_double( double p );
        void set_param2_double( double p );
        void set_param3_double( double p );
        void set_param4_double( double p );
        
        ulong get_return_64();
        uint get_return_32();
        double get_return_double();
        
        
        //Callee
        ulong get_param1_64();
        ulong get_param2_64();
        ulong get_param3_64();
        ulong get_param4_64();
        
        uint get_param1_32();
        uint get_param2_32();
        uint get_param3_32();
        uint get_param4_32();
        
        double get_param1_double();
        double get_param2_double();
        double get_param3_double();
        double get_param4_double();
        
        void set_return_64( ulong r );
        void set_return_32( uint r );
        void set_return_double( double r );
        
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