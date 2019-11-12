/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#include "hardware_emulator.h"
#include "os_windows/os_windows.h"
#include "os_linux/os_linux.h"
#include "software_simulator_manager.h"
#include "direct_software_simulator.h"
#include "port/port_simple.h"
#include "port/port_array.h"

using namespace std;



std::string HardwareEmulator::query_simulator( const char *msg ) {
    MessageParser parser( msg );
    MessageBuilder builder;
    while ( parser.has_next() ) {
        if ( parser.is_cmd( "get_avg_runtime" ) )
            builder.add( "avg_runtime", std::to_string( avg_runtime.mean_avg() ) );
        else if ( parser.is_cmd( "is_computing" ) )
            builder.add( "computing", std::to_string( computing() ? 1 : 0 ) );
        else if ( parser.is_cmd( "get_computer_time" ) )
            builder.add( "computer_time", std::to_string( computer.time.micro_time ) );
        else
            parser.unknown();
    }
    return builder.res;
}

void HardwareEmulator::init_simulator( SoftwareSimulatorManager &manager, const char *config ) {
    //Setup default configuration values.
    simulation_time = 0;
    debug_time = false;
    test_real = false;
    os_name = "";
    computer.time.use_time = true;
    computer.debug.debug = false;
    
    //Read configuration
    //Log::info << Log::tag << "HardwareEmulator::init() with config:\n" << config << "\n";
    MessageParser parser( config );
    while ( parser.has_next() ) {
        if (parser.is_cmd("os"))
            os_name = parser.get_string();
        else if (parser.is_cmd("debug"))
            setup_debug(parser);
        else if (parser.is_cmd("cpu_frequency")) {
            slong cpu_frequency;
            if (parser.get_long(cpu_frequency))
                computer.time.set_cpu_frequency(cpu_frequency);
            else
                Log::err << "Could not read cpu_frequency config value\n";
        }
        else if (parser.is_cmd("memory_frequency")) {
            slong memory_frequency;
            if (parser.get_long(memory_frequency))
                computer.time.set_memory_frequency(memory_frequency);
            else
                Log::err << "Could not read cpu_frequency config value\n";
        }
        else if (parser.cmd_starts_with("cache_"))
            cache_settings.handle_config(parser);
        else if (parser.is_cmd("test_real"))
            test_real_opt = parser.get_string();
        //else parser.unknown();
    }
    
    //Validate configuration and set up the computer.
    
	computer.init();
    resolve_autopilot_os(manager);
    computer.os->load_file(software_path);
    cache_settings.setup_computer(computer);
    resolve("init", init_address);
    resolve("execute", execute_address);
    call_init();
    computer.time.reset();
    
    if ( test_real) {
        direct_simulator = std::unique_ptr<SoftwareSimulator>(new DirectSoftwareSimulator());
        direct_simulator->init(manager, config);
    }
    
       
	Log::info << Log::tag << "Initiated emulator with autopilot: " << software_name << ",\n"
		<< "os: " << os_name << ",\n";
	if (test_real)
		Log::info << "real autopilot compared,\n";
}

void HardwareEmulator::resolve_autopilot_os( SoftwareSimulatorManager &manager ) {

    if ( os_name.size() == 0 || os_name.compare("auto") == 0) {
        bool found = false;
        //Check existing autopilots with this name
        for ( const auto &file : manager.available_softwares) {
            if ( file.get_name().compare( software_name ) == 0 ) {
                found = true;
                if ( file.get_extension().compare( ".dll" ) == 0 )
                    os_name = "windows";
                else if ( file.get_extension().compare( ".so" ) == 0 )
                    os_name = "linux";
                break;
            }
        }
        if ( !found ) {
            throw_error(Error::hardware_emu_software_load_error("Could not find Software program file: " + software_name));
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

    bool test_real_required = test_real_opt.compare("required") == 0;
    test_real = test_real_required || test_real_opt.compare("try") == 0;

    if (os_mistmatch && test_real_required) 
        throw_error(Error::hardware_emu_init_error("With test_real required, the emulated software OS must match the OS running the HardwareEmulator."));
    if (os_mistmatch) 
        test_real = false;
}

void HardwareEmulator::run_cycle()
{
    computer.time.reset();
    process_inputs();
    exec();
    process_outputs();

    if (test_real) {
        direct_simulator->run_cycle();
        //TODO
        //compare();
    }
}

ulong HardwareEmulator::get_cycle_time()
{
    return computer.time.micro_time;
}

void HardwareEmulator::exec()
{
    computer.call(execute_address, "execute");
}

void compare_results( double real_value, double emulated_value, const char *port_name ) {
    if ( abs_t( real_value - emulated_value ) > 0.0001 )
        Log::err << Log::tag << "Desyncronisation detected on port " << port_name << ". "
                 "Real: " << real_value << " Emulated: " << emulated_value << "\n";
}

void HardwareEmulator::call_init() {
    computer.call( init_address, "init" );
}

void HardwareEmulator::resolve( const std::string &name, uint64_t &target ) {
    auto sym = computer.symbols.get_symbol( name );
    if ( sym.type != Symbols::Symbol::Type::EXPORT )
        throw_error(Error::hardware_emu_software_load_error("Could not resolve function " + name + " of software " + software_name));
    target = sym.addr;
}




void HardwareEmulator::setup_debug( MessageParser &parser ) {
    computer.debug.debug = true;
    computer.debug.d_mem = false;
    computer.debug.d_code = false;
    computer.debug.d_regs = false;
    computer.debug.d_unsupported_syscalls = false;
    computer.debug.d_reg_update = false;
    computer.debug.d_syscalls = false;
    computer.debug.d_call = false;
    debug_time = false;
    
    do {
        auto next = parser.get_string();
        if ( next.size() == 0 )
            return;
        if ( next.compare( "mem" ) == 0 )
            computer.debug.d_mem = true;
        else if ( next.compare( "regs" ) == 0 )
            computer.debug.d_regs = true;
        else if ( next.compare( "reg_update" ) == 0 )
            computer.debug.d_reg_update = true;
        else if ( next.compare( "syscalls" ) == 0 )
            computer.debug.d_syscalls = true;
        else if ( next.compare( "unsupported_syscalls" ) == 0 )
            computer.debug.d_unsupported_syscalls = true;
        else if ( next.compare( "code" ) == 0 )
            computer.debug.d_code = true;
        else if ( next.compare( "call" ) == 0 )
            computer.debug.d_call = true;
        else if ( next.compare( "time" ) == 0 )
            debug_time = true;
        else
            Log::err << Log::tag << "Unknown debug flag\n";
    } while ( true );
}

const char* HardwareEmulator::get_string_by_id(const char* name, int id)
{
    computer.func_call->set_param1_32(id);
    ulong function_address;
    resolve(name, function_address);
    computer.call(function_address, name);
    return ( char * )computer.memory.read_str( computer.func_call->get_return_64() );
}

int HardwareEmulator::get_int(const char* name)
{
    uint64_t function_address;
    resolve(name, function_address);
    computer.call(function_address, name);
    return computer.func_call->get_return_32();
}

Port* HardwareEmulator::new_port_by_type(const PortInformation& info)
{
    if (info.dimension.dimension == PortDimension::Dimension::SINGLE) {
        switch (info.type.type) {
        case PortType::Type::INT:
            return new PortIntEmu(info, *this);
        case PortType::Type::DOUBLE:
            return new PortDoubleEmu(info, *this);
        }
    }
    else if (info.dimension.dimension == PortDimension::Dimension::ARRAY) {
        switch (info.type.type) {
        case PortType::Type::INT:
            return new PortIntArrayEmu(info, *this);
        case PortType::Type::DOUBLE:
            return new PortDoubleArrayEmu(info, *this);
        }
    }
    else if (info.dimension.dimension == PortDimension::Dimension::DYNAMIC) {

    }
    return nullptr;
}





