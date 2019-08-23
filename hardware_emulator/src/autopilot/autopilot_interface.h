/* (c) https://github.com/MontiCore/monticore */
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

