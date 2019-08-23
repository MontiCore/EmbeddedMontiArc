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
#include <dlfcn.h>
#include <iostream>
#include <stdlib.h>

typedef int( *add )( int a, int b );

void test_method() {
    auto lib = dlopen("sample_simple.so", RTLD_LAZY);
    printf("Lib handle: %p\n", lib);
    if (lib){
        auto method = dlsym(lib, "add");
        char *error;
        if ((error = dlerror()) == NULL){
            printf("Method address: %p\n", method);
            auto res = ( ( add )method )( 1, 2 );
            printf("add(1, 2)= %d\n", res);
        } else {
            fprintf(stderr, "%s\n", error);
        }
    } else {
        fprintf(stderr, "%s\n", dlerror());
    }
    
    auto mem = malloc( 0x80 );
    ( ( uint64_t * )mem )[0] = 0x123456;
    free(mem);
}