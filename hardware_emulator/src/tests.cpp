#include "tests.h"

bool test_simple_dll() {
    ADD_DLL::DllInterface interf;
    return interf.test_main();
}

bool test_syscall_dll() {
    LOADED_DLL::DllInterface interf;
    return interf.test_main();
}

bool test_autopilot_dll() {
    AutopilotInterface interf;
    return interf.test_main();
}