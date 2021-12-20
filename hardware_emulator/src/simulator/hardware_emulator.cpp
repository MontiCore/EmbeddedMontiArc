/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "hardware_emulator.h"
#include "os_windows/os_windows.h"
#include "os_linux/os_linux.h"
#include "buffer.h"

using namespace std;


uint64_t EmulatedProgramFunctions::resolve( const char *name ) {
    auto sym = computer.symbols.get_symbol( name );
    if ( sym.type != Symbols::Symbol::Type::EXPORT )
        throw_error(Error::hardware_emu_software_load_error("Could not resolve function " + std::string(name)));
    return sym.addr;
}


void EmulatedProgramFunctions::load() {
    auto sym = computer.symbols.get_symbol( FUNC_NAME_ERR_OUT_SET_FUNCTIONS );
    if ( sym.type == Symbols::Symbol::Type::EXPORT ) {
        addr_get_functions = sym.addr;
    } else {
        Log::err.log(ERR_OUT_MISSING_WARNING);
    }
    addr_get_interface = resolve(FUNC_NAME_GET_INTERFACE);
    addr_set_port = resolve(FUNC_NAME_SET_PORT);
    addr_get_port = resolve(FUNC_NAME_GET_PORT);
    addr_init = resolve(FUNC_NAME_INIT);
    addr_exec = resolve(FUNC_NAME_EXECUTE);
}

void EmulatedProgramFunctions::set_functions(uint64_t throw_error_ptr, uint64_t print_cout_ptr, uint64_t print_cerr_ptr)
{
    computer.os->set_param1_64(throw_error_ptr);
    computer.os->set_param2_64(print_cout_ptr);
    computer.os->set_param3_64(print_cerr_ptr);
    computer.call(addr_get_functions, FUNC_NAME_ERR_OUT_SET_FUNCTIONS);
}

const char* EmulatedProgramFunctions::get_interface() {
    computer.call(addr_get_interface, FUNC_NAME_GET_INTERFACE);
    return computer.memory.read_str( computer.os->get_return_64() );
}
void EmulatedProgramFunctions::set_port(int i, const char* data, int is_json) {
    auto addr = computer.memory.exchange_section->address_range.start_address;
    if (is_json) {
        computer.memory.write_str(addr, data);
    }
    else {
        BinaryReader br(data, 4);
        auto size = br.read_u32();
        computer.memory.write_memory(addr, size + 4, (void*)data);
    }

    computer.os->set_param1_32(i);
    computer.os->set_param2_64(addr);
    computer.os->set_param3_32(is_json);
    computer.call(addr_set_port, FUNC_NAME_SET_PORT);
}
const char* EmulatedProgramFunctions::get_port(int i, int is_json) {
    computer.os->set_param1_32(i);
    computer.os->set_param2_32(is_json);
    computer.call(addr_get_port, FUNC_NAME_GET_PORT);
    auto ret = computer.os->get_return_64();
    if (is_json) {
        return computer.memory.read_str(ret);
    } else {

        auto buff = computer.memory.read_memory(ret, 4);
        BinaryReader br((const char*)buff, 4);
        auto size = br.read_u32();
        return (char*)computer.memory.read_memory(ret, size + 4);
    }
}

void EmulatedProgramFunctions::init() {
    computer.call(addr_init, FUNC_NAME_INIT);
}
void EmulatedProgramFunctions::execute(double delta_sec) {
    computer.os->set_param1_double(delta_sec);
    computer.call(addr_exec, FUNC_NAME_EXECUTE);
}


json HardwareEmulator::query_simulator(const json& query) {
    json output;

    for (auto& e : query.items()) {
        auto& key = e.key();
        //if (key.compare("get_avg_runtime") == 0) output["avg_runtime"] = avg_runtime.mean_avg();
        //else 
            if (key.compare("get_is_computing") == 0) output["is_computing"] = computing();
        else if (key.compare("get_computer_time") == 0) output["computer_time"] = computer.time.micro_time;
        else output["unknown_query"].push_back(key);
    }

    return output;
}

void HardwareEmulator::init_simulator(const json& config, const fs::path& software_folder) {
    //Setup default configuration values.
    debug_time = false;
    os_name = "";
    computer.time.use_time = true;
    computer.debug.debug = false;

    // Load the configuration (With silent failing, assumes config is validated before by the "ComputerProperties" Java class)
    if (config.contains("time_model")) {
        auto time_model_config = config["time_model"];
        if (time_model_config.is_object()) {
            std::string type;
            if (json_get(time_model_config, "type", type)) {
                if (type.compare("measured") == 0) time_model = TimeModel::MEASURED;
                else if (type.compare("constant") == 0) time_model = TimeModel::CONSTANT;
                else if (type.compare("models") == 0) {
                    time_model = TimeModel::TIME_MODELS;
                    ulong cpu_f, mem_f;
                    if (json_get(time_model_config, "cpu_frequency", cpu_f)) computer.time.set_cpu_frequency(cpu_f);
                    if (json_get(time_model_config, "memory_frequency", mem_f)) computer.time.set_memory_frequency(mem_f);
                    cache_settings.handle_config(time_model_config);
                }
                else Log::err.log_tag("Unknown Time Model");
            }
        }
    }

    auto backend = config["backend"];
    json_get(backend, "os", os_name);
    
    //Validate configuration and set up the computer.
    
	computer.init();
    parse_flags(config);
    cache_settings.setup_computer(computer);

    resolve_autopilot_os(software_folder);
    computer.os->load_file(software_path);

    // Load the Program interface
    
    EmulatedProgramFunctions* prog_functions = new EmulatedProgramFunctions(computer);
    prog_functions->load();
    if (prog_functions->addr_get_functions != 0)
        prog_functions->set_functions(computer.throw_error_addr, computer.print_cout_addr, computer.print_cerr_addr);
    prog_functions->init();
    computer.time.reset();

    program_functions = std::unique_ptr<ProgramFunctions>(prog_functions);

    Log::info.log_tag("Initiated software \"%s\" in emulated mode, os: %s", program_name.c_str(), os_name.c_str());
}

void HardwareEmulator::resolve_autopilot_os( const fs::path& software_folder ) {
    auto ext = software_path.extension().string();
    // If an extension is specified for the Software -> get corresponding OS
    if (ext.size() > 0){
        if ( ext.compare( ".dll" ) == 0 )
            os_name = "windows";
        else if ( ext.compare( ".so" ) == 0 )
            os_name = "linux";
        else throw_error(Error::hardware_emu_software_load_error("Unknown Software extension: " + ext));
    }

    if ( os_name.size() == 0 || os_name.compare("auto") == 0) {
        bool found = false;
        //Check existing autopilots with this name
        for (const auto& subpath : fs::recursive_directory_iterator(software_folder)) {
            if (subpath.is_directory()) continue;
            fs::path file = subpath;
            if (file.stem().compare(program_name) == 0) {
                found = true;
                if (file.extension().compare(".dll") == 0)
                    os_name = "windows";
                else if (file.extension().compare(".so") == 0)
                    os_name = "linux";
                break;
            }
        }
        if ( !found ) {
            throw_error(Error::hardware_emu_software_load_error("Could not find Software program file: " + program_name));
        }
    }
    else {
        if ( os_name.compare( "windows" ) != 0 && os_name.compare( "linux" ) != 0 ) {
            throw_error(Error::hardware_emu_init_error("Unsupported OS: " + os_name));
        }
    }

    bool os_mistmatch = false;
    if (os_name.compare("windows") == 0) {
        computer.set_os(new OS::Windows(computer.func_call_windows));
        if (Library::type != Library::OsType::WINDOWS) {
            os_mistmatch = true;
        }
    }
    else if (os_name.compare("linux") == 0) {
        computer.set_os(new OS::Linux(computer.func_call_linux));
        if (Library::type != Library::OsType::LINUX) {
            os_mistmatch = true;
        }
    }
}


void HardwareEmulator::start_timer()
{
    if (time_model == TimeModel::MEASURED) {
        timer.start();
    }
    else {
        timer_start = computer.time.micro_time;
    }
}

ulong HardwareEmulator::get_timer_micro()
{
    if (time_model == TimeModel::MEASURED) {
        timer.end();
        return timer.getDelta();
    }
    else {
        return computer.time.micro_time - timer_start;
    }
}

void HardwareEmulator::parse_flags(const json& config) {
    computer.debug.debug = false;
    computer.debug.d_mem = false;
    computer.debug.d_code = false;
    computer.debug.d_regs = false;
    computer.debug.d_unsupported_syscalls = false;
    computer.debug.d_reg_update = false;
    computer.debug.d_syscalls = false;
    computer.debug.d_call = false;
    debug_time = false;

    if (!config.contains("debug_flags")) return;
    computer.debug.debug = true;
    auto& flags = config["debug_flags"];
    if (!flags.is_array()) return;
    for (auto& e : flags) {
        if (!e.is_string()) {
            Log::err.log_tag("Invalid flag: %s", e.dump().c_str());
        }
        auto flag = e.get<std::string>();
        if (flag.compare("p_mem") == 0) computer.debug.d_mem = true;
        else if (flag.compare("p_regs") == 0) computer.debug.d_regs = true;
        else if (flag.compare("p_reg_update") == 0) computer.debug.d_reg_update = true;
        else if (flag.compare("p_syscalls") == 0) computer.debug.d_syscalls = true;
        else if (flag.compare("p_unsupported_syscalls") == 0) computer.debug.d_unsupported_syscalls = true;
        else if (flag.compare("p_code") == 0) computer.debug.d_code = true;
        else if (flag.compare("p_call") == 0) computer.debug.d_call = true;
        else if (flag.compare("p_time") == 0) debug_time = true;
        else if (flag.compare("p_instruction_operands") == 0) computer.debug.d_instruction_operands = true;
        else if (flag.compare("p_operands_details") == 0) computer.debug.d_operands_details = true;
        else if (flag.compare("p_cache_hit_ratio") == 0) computer.debug.d_cache_hit_ratio = true;
        else if (flag.compare("p_mem_access") == 0) computer.debug.d_mem_access = true;
        else Log::err.log_tag("Unknown flag: %s", flag.c_str());
    }
}
//TODO: test the debug flag with java scenario

