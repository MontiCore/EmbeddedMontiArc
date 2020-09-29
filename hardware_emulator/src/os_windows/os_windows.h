/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include "dll_loader.h"
#include "computer/os.h"
#include "computer/function_calling.h"

namespace OS {

    /*
        Implementation of the FunctionCalling interface that represents the Windows FastCall convention,
        putting the parameters of a function call in the correct registers.
    
        The integer and pointer arguments are passed from left to right into the RCX, RDX, R8, R9 registers
        The return value for integers and pointers is passed in the RAX register.
    
        For floating-point arguments, they are placed inside the XMM0, XMM1, XMM2, ... registers.
        The return value is in the XMM0 register.
    */
    struct WindowsFastCall : public FunctionCalling {
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
        
        void set_params_double( double p1 );
        void set_params_double( double p1, double p2 );
        void set_params_double( double p1, double p2, double p3 );
        void set_params_double( double p1, double p2, double p3, double p4 );
        
        void set_params_float( float p1 );
        void set_params_float( float p1, float p2 );
        void set_params_float( float p1, float p2, float p3 );
        void set_params_float( float p1, float p2, float p3, float p4 );
        
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
        
        void set_param1_float( float p );
        void set_param2_float( float p );
        void set_param3_float( float p );
        void set_param4_float( float p );
        
        ulong get_return_64();
        uint get_return_32();
        double get_return_double();
        float get_return_float();
        char get_return_char();
        
        
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
        
        float get_param1_float();
        float get_param2_float();
        float get_param3_float();
        float get_param4_float();
        
        void set_return_64( ulong r );
        void set_return_32( uint r );
        void set_return_double( double r );
        
        WindowsFastCall( Registers &registers ) : registers( registers ) {}
    };
    
    /*
        The Windows component sets up the Windows system function, the WindowsFastCall implementation and various
        objects needed by autopilots running under Windows.
    */
    struct Windows : public OS {
        MemorySection *section = nullptr;
        SectionStack *section_stack = nullptr;
        
        DllLoader dll;
        
        MemoryRange cmd_line_wstr;
        MemoryRange cmd_line_str;
        
        MemoryRange io_stdin;
        MemoryRange io_stdout;
        MemoryRange io_stderr;

        MemoryRange lconv_slot;
        
		std::vector<char> name_buffer;
        
        Windows() : name_buffer( 1024 ) {}
        
        void init( Computer &computer );
        void setup_tib_gdt();
        void setup_command_line_args();
        void setup_cout();
        void setup_io();
        void setup_locale();
        
        //File without extension
        void load_file(const FS::File& file);
        

        ulong upload_system_string(const std::string& str, const char* description);
    };
    
}
