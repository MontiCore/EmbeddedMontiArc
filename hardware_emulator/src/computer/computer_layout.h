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
#pragma once
#include "utility.h"

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
    //Virtual stack position and max size.
    constexpr ulong STACK_BOUNDARY = 0x1000;
    constexpr ulong STACK_ADDRESS = SYSCALLS_ADDRESS + SYSCALLS_RANGE + STACK_BOUNDARY;
    constexpr ulong STACK_MAX_SIZE = 0x100000;
    //Virtual heap section.
    constexpr ulong HEAP_ADDRESS = STACK_ADDRESS + STACK_MAX_SIZE + STACK_BOUNDARY;
    constexpr ulong HEAP_SIZE = 0x10000;
    constexpr ulong HEAP_MAX_SIZE = 0x100000;
}