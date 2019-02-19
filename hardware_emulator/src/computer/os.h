#pragma once
#include "computer/computer.h"

namespace OS {
    struct OS {
        Computer *computer;
        
        OS() : computer( nullptr ) {}
        
        virtual void init( Computer &computer ) = 0;
        
        
        bool loaded() {
            return computer != nullptr;
        }
        
        virtual bool load_file( const char *file ) = 0;
    };
}