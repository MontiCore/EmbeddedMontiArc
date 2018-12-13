#include "tests.h"

bool test_simple_dll() {
    ADD_DLL::DllInterface interf;
    return interf.test_main();
}
