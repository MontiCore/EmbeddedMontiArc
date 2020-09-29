#include "hardware_emulator.h"
#include "os_windows/os_windows.h"
#include "os_linux/os_linux.h"
#include "software_simulator_manager.h"

using namespace std;


uint64_t EmulatedProgramInterface::resolve( const char *name ) {
    auto sym = computer.symbols.get_symbol( name );
    if ( sym.type != Symbols::Symbol::Type::EXPORT )
        throw_error(Error::hardware_emu_software_load_error("Could not resolve function " + std::string(name)));
    return sym.addr;
}


void EmulatedProgramInterface::load() {
    addr_get_interface = resolve(FUNC_NAME_GET_INTERFACE);
    addr_set_port = resolve(FUNC_NAME_SET_PORT);
    addr_get_port = resolve(FUNC_NAME_GET_PORT);
    addr_init = resolve(FUNC_NAME_INIT);
    addr_exec = resolve(FUNC_NAME_EXECUTE);
}

const char* EmulatedProgramInterface::get_interface() {
    computer.call(addr_get_interface, FUNC_NAME_GET_INTERFACE);
    return computer.memory.read_str( computer.func_call->get_return_64() );
}
void EmulatedProgramInterface::set_port(int i, const char* data) {
    auto addr = computer.memory.exchange_section->address_range.start_address;
    computer.memory.write_str(addr, data);
    computer.func_call->set_param1_32(i);
    computer.func_call->set_param2_64(addr);
    computer.call(addr_set_port, FUNC_NAME_SET_PORT);
}
const char* EmulatedProgramInterface::get_port(int i) {
    computer.func_call->set_param1_32(i);
    computer.call(addr_get_port, FUNC_NAME_GET_PORT);
    return computer.memory.read_str( computer.func_call->get_return_64() );
}

void EmulatedProgramInterface::init() {
    computer.call(addr_init, FUNC_NAME_INIT);
}
void EmulatedProgramInterface::execute(double delta_sec) {
    computer.func_call->set_param1_double(delta_sec);
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

void HardwareEmulator::init_simulator(const json& config, const FS::Directory& software_folder) {
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
                if (type.compare("instant") == 0) time_model = TimeModel::INSTANT;
                else if (type.compare("constant") == 0) {
                    time_model = TimeModel::CONSTANT;
                    json_get(time_model_config, "execution_time", const_time);
                }
                else if (type.compare("models") == 0) {
                    time_model = TimeModel::TIME_MODELS;
                    ulong cpu_f, mem_f;
                    if (json_get(time_model_config, "cpu_frequency", cpu_f)) computer.time.set_cpu_frequency(cpu_f);
                    if (json_get(time_model_config, "memory_frequency", mem_f)) computer.time.set_memory_frequency(mem_f);
                    cache_settings.handle_config(time_model_config);
                }
                else Log::err << "Unknown Time Model\n";
            }
        }
    }
    
    json_get(config, "os", os_name);
    
    //Validate configuration and set up the computer.
    
	computer.init();
    parse_flags(config);
    cache_settings.setup_computer(computer);

    resolve_autopilot_os(software_folder);
    computer.os->load_file(software_path);

    // Load the Program interface
    
    EmulatedProgramInterface* prog_interface = new EmulatedProgramInterface(computer);
    prog_interface->load();
    prog_interface->init();
    computer.time.reset();

    program_interface = std::unique_ptr<ProgramInterface>(prog_interface);

    Log::info << Log::tag << "Initiated software \"" << program_name <<  "\" in emulated mode, os: " << os_name << "\n";
}

void HardwareEmulator::resolve_autopilot_os( const FS::Directory& software_folder ) {
    auto ext = software_path.get_extension();
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
        for ( const auto &file : software_folder.get_files()) {
            if ( file.get_name().compare( program_name ) == 0 ) {
                found = true;
                if ( file.get_extension().compare( ".dll" ) == 0 )
                    os_name = "windows";
                else if ( file.get_extension().compare( ".so" ) == 0 )
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
        computer.set_os(new OS::Windows());
        if (Library::type != Library::OsType::WINDOWS) {
            os_mistmatch = true;
        }
    }
    else if (os_name.compare("linux") == 0) {
        computer.set_os(new OS::Linux());
        if (Library::type != Library::OsType::LINUX) {
            os_mistmatch = true;
        }
    }
}


void HardwareEmulator::start_timer()
{
    timer_start = computer.time.micro_time;
}

ulong HardwareEmulator::get_timer_micro()
{
    return computer.time.micro_time - timer_start;
}

void HardwareEmulator::parse_flags(const json& config) {
    computer.debug.debug = true;
    computer.debug.d_mem = false;
    computer.debug.d_code = false;
    computer.debug.d_regs = false;
    computer.debug.d_unsupported_syscalls = false;
    computer.debug.d_reg_update = false;
    computer.debug.d_syscalls = false;
    computer.debug.d_call = false;
    debug_time = false;

    if (!config.contains("debug_flags")) return;
    auto& flags = config["debug_flags"];
    if (!flags.is_array()) return;
    for (auto& e : flags) {
        if (!e.is_string()) {
            Log::err << Log::tag << "Invalid flag: " << e.dump() << "\n";
        }
        auto& flag = e.get<std::string>();
        if (flag.compare("p_mem") == 0) computer.debug.d_mem = true;
        else if (flag.compare("p_regs") == 0) computer.debug.d_regs = true;
        else if (flag.compare("p_reg_update") == 0) computer.debug.d_reg_update = true;
        else if (flag.compare("p_syscalls") == 0) computer.debug.d_syscalls = true;
        else if (flag.compare("p_unsupported_syscalls") == 0) computer.debug.d_unsupported_syscalls = true;
        else if (flag.compare("p_code") == 0) computer.debug.d_code = true;
        else if (flag.compare("p_call") == 0) computer.debug.d_call = true;
        else if (flag.compare("p_time") == 0) debug_time = true;
        else Log::err << Log::tag << "Unknown flag: " << flag << "\n";
    }
}

