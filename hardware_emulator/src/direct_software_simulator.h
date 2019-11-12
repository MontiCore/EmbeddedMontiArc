/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#pragma once

#include "software_simulator.h"


struct DirectSoftwareSimulator : public SoftwareSimulator {
    using ExecFunc = void(*)();
    using InitFunc = void(*)();
    Library software;

    void* real_exec = nullptr;
    void* real_init = nullptr;
    long real_time_accumulator = 0;

    std::string query_simulator(const char* msg);
    void init_simulator(SoftwareSimulatorManager& manager, const char* config);
    void exec();
    Port* new_port_by_type(const PortInformation& info);
    const char* get_string_by_id(const char* name, int id);
    int get_int(const char* name);
    void resolve_real(const std::string& name, void*& target);
};