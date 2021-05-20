/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include <vector>
#include <memory>
#include <unordered_map>
#include "utility/utility.h"
#include "json.hpp"
using json = nlohmann::json;

struct SoftwareSimulatorManager;


enum class TimeModel {
    MEASURED,
    CONSTANT,
    TIME_MODELS
};

struct ProgramInterface {
    static constexpr const char *FUNC_NAME_GET_INTERFACE = "DI__get_interface";
    static constexpr const char *FUNC_NAME_SET_PORT = "DI__set_port";
    static constexpr const char *FUNC_NAME_GET_PORT = "DI__get_port";
    static constexpr const char *FUNC_NAME_INIT = "DI__init";
    static constexpr const char *FUNC_NAME_EXECUTE = "DI__execute";
    virtual const char* get_interface() = 0;
    virtual void set_port(int i, const char* data, int is_json) = 0;
    virtual const char* get_port(int i, int is_json) = 0;
    
    virtual void init() = 0;
    virtual void execute(double delta_sec) = 0;
    virtual ~ProgramInterface() {}
};

struct SoftwareSimulator {
    std::unique_ptr<ProgramInterface> program_interface;

    void init(const json& config, const fs::path& software_folder);

    virtual void start_timer() = 0;
    virtual ulong get_timer_micro() = 0;
    json query(const json& query);
    
    virtual ~SoftwareSimulator() {}
protected:
    TimeModel time_model = TimeModel::CONSTANT;
    ulong const_time = 0;
    
    fs::path software_path;
    std::string program_name;

    //Methods that must be implemented by the Simulator instance
    virtual json query_simulator(const json& query) = 0;
    virtual void init_simulator(const json& config, const fs::path& software_folder) = 0;
};