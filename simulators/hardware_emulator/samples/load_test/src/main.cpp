/**
 * (c) https://github.com/MontiCore/monticore
 */
#include <iostream>
#include <dlfcn.h>

using namespace std;

int main(int argc, char** argv) {

    //auto handle = dlopen( "lib_test.so", RTLD_NOW );
    auto handle = dlopen( "/home/jean/dev/build_environment/load_test/build_debug/lib_test.so", RTLD_NOW );
    if (handle == nullptr) {
        cerr << "Could not open lib: "<< dlerror() << endl;
        return -1;
    }

    auto func_ptr = dlsym( handle, "lib_bar_function" );
    if (func_ptr == nullptr) {
        cerr << "Could not load 'lib_bar_function': " << dlerror() << endl;
        return -1;
    }

    using FuncSign = int(*)();
    auto cast_func_ptr = (FuncSign)func_ptr;

    auto res = cast_func_ptr();

    cout << "Got res=" << res << endl;

    return 0;
}