/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
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