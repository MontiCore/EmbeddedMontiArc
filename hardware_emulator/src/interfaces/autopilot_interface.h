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
    
    enum Functions {
        SET_TIME_INC,
        SET_VELOCITY,
        SET_X,
        SET_Y,
        SET_COMPASS,
        SET_ENGINE,
        SET_STEERING,
        SET_BRAKES,
        SET_TRAJECTORY_LENGTH,
        SET_TRAJECTORY_X,
        SET_TRAJECTORY_Y,
        INPUT_FUNC_COUNT,
        
        OUTPUT_FUNC_START = INPUT_FUNC_COUNT,
        GET_ENGINE = INPUT_FUNC_COUNT,
        GET_STEERING,
        GET_BRAKES,
        OUTPUT_FUNC_END,
        OUTPUT_FUNC_COUNT = OUTPUT_FUNC_END - INPUT_FUNC_COUNT,
        
        EXEC = OUTPUT_FUNC_END,
        INIT,
        MESSAGE,
        
        FUNCTION_COUNT
    };
    bool test_main();
    
    bool init();
    void exec();
    std::string message( const char *msg );
    
    
    
};

