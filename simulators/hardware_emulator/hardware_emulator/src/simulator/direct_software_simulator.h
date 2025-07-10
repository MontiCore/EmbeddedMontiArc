/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once

#include "err_out.h"
#include "software_simulator.h"
#include "utility/utility.h"

struct DirectProgramFunctions : public ProgramFunctions {
    using GetInterfaceFunc = const char*(*)(void);
    using SetPortFunc = void(*)(int,const char*,int);
    using GetPortFunc = const char*(*)(int,int);
    using InitFunc = void(*)();
    using ExecFunc = void(*)(double);
    GetInterfaceFunc real_get_interface = nullptr;
    SetPortFunc real_set_port = nullptr;
    GetPortFunc real_get_port = nullptr;
    InitFunc real_init = nullptr;
    ExecFunc real_exec = nullptr;
    set_functions_func real_set_functions;

    void load(Library &software);
    
    const char* get_interface() {
        return real_get_interface();
    }
    void set_port(int i, const char* data, int is_json) {
        real_set_port(i, data, is_json);
    }
    const char* get_port(int i, int is_json) {
        return real_get_port(i, is_json);
    }
    
    void init() {
        real_init();
    }
    void execute(double delta_sec) {
        real_exec(delta_sec);
    }
};

struct DirectSoftwareSimulator : public SoftwareSimulator {
    Library software;

    MeanAvgCollector avg_runtime;
    PrecisionTimer timer;

    json query_simulator(const json& query);
    void init_simulator(const json& config, const fs::path& software_folder);

    void start_timer();
    ulong get_timer_micro();
};