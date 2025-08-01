/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include "utility/utility.h"

/*
    Contains the virtual memory layout for the computer.
*/
namespace ComputerLayout {
    constexpr ulong BASE_ADDRESS = 0x01000000;
    //A section to allocate system owned data passed to the emulated code (can be read/written to).
    constexpr ulong SYSPAGE_ADDRESS = BASE_ADDRESS;
    constexpr ulong SYSPAGE_RANGE = 0x2000;
    //A section for allocatable external handles.
    constexpr ulong HANDLES_ADDRESS = SYSPAGE_ADDRESS + SYSPAGE_RANGE;
    constexpr ulong HANDLES_RANGE = 0x1000;
    constexpr ulong HANDLE_COUNT = 0x100;
    //A section for registering system calls.
    constexpr ulong SYSCALLS_ADDRESS = HANDLES_ADDRESS + HANDLES_RANGE;
    constexpr ulong SYSCALLS_RANGE = 0x1000;
    //A section to exchange data with the program. (Read onyl)
    constexpr ulong EXCHANGE_ADDRESS = SYSCALLS_ADDRESS + SYSCALLS_RANGE;
    constexpr ulong EXCHANGE_RANGE = 0x10000;
    //Virtual stack position and max size.
    constexpr ulong STACK_BOUNDARY = 0x1000;
    constexpr ulong STACK_ADDRESS = EXCHANGE_ADDRESS + EXCHANGE_RANGE + STACK_BOUNDARY;
    constexpr ulong STACK_MAX_SIZE = 0x100000;
    //Virtual heap section.
    constexpr ulong HEAP_ADDRESS = STACK_ADDRESS + STACK_MAX_SIZE + STACK_BOUNDARY;
    constexpr ulong HEAP_SIZE = 0x100000;
    constexpr ulong HEAP_MAX_SIZE = 0x100000;
}
