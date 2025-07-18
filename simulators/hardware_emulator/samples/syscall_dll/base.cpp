/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "base.h"
#include <Windows.h>
#include <iostream>

typedef int( *add )( int a, int b );

void test_method() {
    auto lib = LoadLibraryExW( L"sample_simple", NULL, 0 );
    std::cout << "Lib handle: " << lib << std::endl;
    auto method = GetProcAddress( lib, "add" );
    std::cout << "Method address: " << method << std::endl;
    if ( method != NULL ) {
        auto res = ( ( add )method )( 1, 2 );
        std::cout << "Add(1, 2)= " << res << std::endl;
    }
    auto heap_handle = GetProcessHeap();
    auto mem = HeapAlloc( heap_handle, HEAP_ZERO_MEMORY, 0x80 );
    ( ( uint64_t * )mem )[0] = 0x123456;
    HeapFree( heap_handle, NULL, mem );
}
