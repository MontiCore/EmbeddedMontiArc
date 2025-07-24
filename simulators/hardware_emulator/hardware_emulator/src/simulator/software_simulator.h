/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include <vector>
#include <memory>
#include <unordered_map>
#include "utility/utility.h"
#include "json.hpp"
#include "program_interface.h"
using json = nlohmann::json;



struct SoftwareSimulatorManager;


enum class TimeModel {
    MEASURED,
    CONSTANT,
    TIME_MODELS
};

struct ProgramFunctions {
    static constexpr const char* ERR_OUT_MISSING_WARNING = "[Warning] Cannot find the ERR_OUT_set_functions() function in the program. Programs should use 'err_out.h' and 'err_out.cpp' for logging and errors.";
    static constexpr const char* FUNC_NAME_ERR_OUT_SET_FUNCTIONS = "ERR_OUT_set_functions";
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
    virtual ~ProgramFunctions() {}
};

struct SoftwareSimulator {
    std::unique_ptr<ProgramFunctions> program_functions;
    std::string interface_string;
    ProgramInterface program_interface;

    static SoftwareSimulator* alloc(const json& config, fs::path& softwares_folder);
    static void ERR_OUT_throw_error(const char* type, const char* message);
    static void ERR_OUT_print_cout(const char* message);
    static void ERR_OUT_print_cerr(const char* message);



    void init(const json& config, const fs::path& software_folder);

    virtual void start_timer() = 0;
    virtual ulong get_timer_micro() = 0;
    json query(const json& query);
    // Returns '-1' if not found
    int get_port_id(const std::string& name) const;
    int get_port_id_expected(const std::string& name) const;
    
    virtual ~SoftwareSimulator() {}
protected:
    TimeModel time_model = TimeModel::CONSTANT;
    ulong const_time = 0;
    
    fs::path software_path;
    std::string program_name;

    std::unordered_map<std::string, int> port_id;

    //Methods that must be implemented by the Simulator instance
    virtual json query_simulator(const json& query) = 0;
    virtual void init_simulator(const json& config, const fs::path& software_folder) = 0;
};