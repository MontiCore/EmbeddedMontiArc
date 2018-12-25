#pragma once
#include "utility.h"


namespace ComputerLayout {
    constexpr ulong SYSPAGE_ADDRESS = 0x1000;
    constexpr ulong SYSPAGE_RANGE = 0x4000;
    constexpr ulong HANDLES_ADDRESS = SYSPAGE_ADDRESS + SYSPAGE_RANGE;
    constexpr ulong HANDLES_RANGE = 0x1000;
    constexpr ulong HANDLE_COUNT = 0x100;
    constexpr ulong SYSCALLS_ADDRESS = HANDLES_ADDRESS + HANDLES_RANGE;
    constexpr ulong SYSCALLS_RANGE = 0x1000;
    constexpr ulong SYMBOLS_ADDRESS = SYSCALLS_ADDRESS + SYSCALLS_RANGE;
    constexpr ulong SYMBOLS_RANGE = 0x1000;
    constexpr ulong STACK_BOUNDARY = 0x1000;
    constexpr ulong STACK_ADDRESS = SYMBOLS_ADDRESS + SYMBOLS_RANGE + STACK_BOUNDARY;
    constexpr ulong STACK_MAX_SIZE = 0x100000;
    constexpr ulong HEAP_ADDRESS = STACK_ADDRESS + STACK_MAX_SIZE + STACK_BOUNDARY;
    constexpr ulong HEAP_SIZE = 0x10000;
    constexpr ulong HEAP_MAX_SIZE = 0x100000;
}