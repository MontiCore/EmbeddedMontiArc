/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "lib.h"
#include <iostream>

using namespace std;



Foo foo_ns::global_foo;

int lib_bar_function() {
    return foo_ns::global_foo.a;
}

Foo::Foo() {
    a = 42;
    //cout << "Running Constructor" << endl;
}