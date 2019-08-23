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
#include "os_windows/os_windows.h"
#include "computer/computer.h"
#include "emulation/hardware_emulator.h"


struct MessageParser {
    const char *msg;
    uint pos;
    uint cmd_size;
    const char *rest;
    bool has_cmd;
    
    MessageParser( const char *msg );
    bool is_cmd( const char *cmd );
    bool get_long( slong &target );
    void to_non_ws();
    void to_comma();
};

struct AutopilotInterface {
    static AutopilotInterface instance;
    bool loaded;
    AutopilotInterface() : loaded( false ) {}
    OS::Windows os_windows;
    Computer computer;
    HardwareEmulator emulator;
    
    bool test_main();
    
    bool init();
    std::string message( const char *msg );
};

