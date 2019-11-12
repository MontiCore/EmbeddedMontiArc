/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#pragma once
#include <vector>
#include <memory>
#include <unordered_map>
#include "port/port.h"
#include "dynamic_interface/resolver.h"

struct SoftwareSimulatorManager;


enum class TimeModel {
    INSTANT,
    CONSTANT,
    TIME_MODELS
};

struct SoftwareSimulator : public DynamicSoftwareInterface {
    void init(SoftwareSimulatorManager& manager, const char* config);
    ulong run_cycle_get_time();
    virtual void run_cycle();
    virtual ulong get_cycle_time();
    void add_one_input(JNIEnv* jni, jstring key, jobject value);
    void query_outputs(JNIEnv* jni, jclass cls, jobject opaque_hashmap);

    std::string query(const char* msg);
    virtual ~SoftwareSimulator() {}

    Port* get_port(const char* port_name);
protected:
    TimeModel time_model = TimeModel::INSTANT;
    ulong const_time = 0;
    std::vector<std::unique_ptr<Port>> input_ports;
    std::vector<std::unique_ptr<Port>> output_ports;
    using PortMap = std::unordered_map<std::string, Port*>;
    PortMap port_map;
    std::string software_name;
    FS::File software_path;

    void process_inputs();
    void process_outputs();

    //Methods that must be implemented by the Simulator instance
    virtual std::string query_simulator(const char* msg) = 0;
    virtual void init_simulator(SoftwareSimulatorManager& manager, const char* config) = 0;
    virtual void exec() = 0;
    virtual Port* new_port_by_type(const PortInformation& info) = 0;
};