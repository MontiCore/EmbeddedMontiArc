/**
 * (c) https://github.com/MontiCore/monticore
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
