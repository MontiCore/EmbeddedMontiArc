#pragma once

#include "software_simulator.h"
#include "utility/utility.h"

struct DirectProgramInterface : public ProgramInterface {
    using GetInterfaceFunc = const char*(*)(void);
    using SetPortFunc = void(*)(int,const char*);
    using GetPortFunc = const char*(*)(int);
    using InitFunc = void(*)();
    using ExecFunc = void(*)(double);
    GetInterfaceFunc real_get_interface = nullptr;
    SetPortFunc real_set_port = nullptr;
    GetPortFunc real_get_port = nullptr;
    InitFunc real_init = nullptr;
    ExecFunc real_exec = nullptr;

    void load(Library &software);
    
    const char* get_interface() {
        return real_get_interface();
    }
    void set_port(int i, const char* data) {
        real_set_port(i, data);
    }
    const char* get_port(int i) {
        return real_get_port(i);
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
    void init_simulator(const json& config, const FS::Directory& software_folder);

    void start_timer();
    ulong get_timer_micro();
};