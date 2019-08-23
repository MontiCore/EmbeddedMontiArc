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
#include "computer/computer.h"

/*
    The structures implementing ProgramInterface provide a simple interface to
    an program by resolving and storing the addresses of the functions
    and providing proxy functions that handle the argument passing and calling of the
    emulated function.
*/
struct ProgramInterface {
    Computer *computer;
    void init( Computer &computer ) {
        this->computer = &computer;
    }
    Array<uint64_t> addresses;
    bool call_success;
};

namespace ADD_DLL {

    struct Interface : public ProgramInterface {
        enum Functions {
            ADD,
            FUNCTION_COUNT
        };
        bool init( Computer &computer, bool windows );
        int add( int a, int b );
    };
}

namespace LOADED_DLL {

    struct Interface : public ProgramInterface {
        enum Functions {
            TEST_METHOD,
            FUNCTION_COUNT
        };
        bool init( Computer &computer );
        void test_method( void );
    };
}


