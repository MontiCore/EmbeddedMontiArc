#pragma once
#include "os_windows/os_windows.h"
#include "computer/computer.h"
#include "jni/jni_emulator.h"



struct AutopilotInterface {
    static AutopilotInterface instance;
    bool loaded;
    AutopilotInterface() : loaded( false ) {}
    Computer computer;
    OS::Windows os_windows;
    Array<uint64_t> addresses;
    JNIEmulator jni_emu;
    
    uint simulation_time;
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
        GET_ENGINE,
        GET_STEERING,
        GET_BRAKES,
        EXEC,
        INIT,
        MESSAGE,
        
        FUNCTION_COUNT
    };
    bool test_main();
    bool init();
    
    bool running() {
        return simulation_time < computer.computing_time;
    }
    
    void jni_set_timeIncrement( double timeIncrement );
    void jni_set_currentVelocity( double currentVelocity );
    void jni_set_x( double currentGpsLat );
    void jni_set_y( double currentGpsLon );
    void jni_set_compass( double compass );
    void jni_set_currentEngine( double currentEngine );
    void jni_set_currentSteering( double currentSteering );
    void jni_set_currentBrakes( double currentBrakes );
    void jni_set_trajectory_length( int length );
    void jni_set_trajectory_x( double *x, uint count );
    void jni_set_trajectory_y( double *y, uint count );
    double jni_get_engine();
    double jni_get_steering();
    double jni_get_brakes();
    
    double engine_value;
    double steering_value;
    double brake_value;
    
    void jni_exec();
    void jni_init();
    
    char *jni_message( char *msg );
    
    
    bool call_success;
};

