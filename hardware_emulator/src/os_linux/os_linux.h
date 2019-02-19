#pragma once
#include "computer/os.h"

namespace OS {

    struct Linux : public OS {
        MemorySection *section;
        SectionStack *section_stack;
        
        Array<char> name_buffer;
        
        Linux() : name_buffer( 1024 ) {}
        
        void init( Computer &computer );
        
        bool load_file( const char *file );
        
        ulong add_symbol( const std::string &mod, const std::string &name, uint size,
                          Annotation::Type type = Annotation::SYMBOL );
    };
}