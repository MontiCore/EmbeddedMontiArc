/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#include "software_simulator.h"

#include "utility/config.h"
#include "software_simulator_manager.h"

void SoftwareSimulator::init(SoftwareSimulatorManager& manager, const char* config)
{
    software_name = "";
    std::string time_model_config;
    MessageParser parser(config);
    while (parser.has_next()) {
        if (parser.is_cmd("software"))
            software_name = parser.get_string();
        else if (parser.is_cmd("time_model")) {
            time_model_config  = parser.get_string();
        }
        else if (parser.is_cmd("const_execution_time")) {
            slong t;
            if (parser.get_long(t))
                const_time = (ulong)t;
            else
                Log::err << "Could not read const_execution_time config value\n";
        }
    }

    if (time_model_config.compare("instant") == 0) time_model = TimeModel::INSTANT;
    else if (time_model_config.compare("constant") == 0) time_model = TimeModel::CONSTANT;
    else if (time_model_config.compare("models") == 0) time_model = TimeModel::TIME_MODELS;
    else {
        Log::err << "Unkown Time Model\n";
    }

    //TODO time model

    if (software_name.size() == 0)
        throw_error(Error::hardware_emu_software_load_error("Missing the name of the Software program to load."));

    software_path = FS::File(manager.softwares_folder, software_name);

    init_simulator(manager, config);

    //Discover Software
    DynamicInterfaceResolver resolver;
    resolver.discover_interface(*this);

    input_ports.resize(resolver.input_ports.size());
    output_ports.resize(resolver.output_ports.size());

    for (auto i : ulrange(input_ports.size())) {
        auto& port_info = resolver.input_ports[i];
        auto& port = input_ports[i];
        port = std::unique_ptr<Port>(new_port_by_type(port_info));
        if (!port) throw_error("Error creating new port: " + port_info.name);
        port_map.insert(std::pair<std::string, Port*>(port_info.name, port.get()));
    }

    for (auto i : ulrange(output_ports.size())) {
        auto& port_info = resolver.output_ports[i];
        auto& port = output_ports[i];
        port = std::unique_ptr<Port>(new_port_by_type(port_info));
        if (!port) throw_error("Error creating new port: " + port_info.name);
        port_map.insert(std::pair<std::string, Port*>(port_info.name, port.get()));
    }
}

ulong SoftwareSimulator::run_cycle_get_time()
{
    run_cycle();
    switch (time_model) {
    case TimeModel::INSTANT: return 0;
    case TimeModel::CONSTANT: return const_time;
    case TimeModel::TIME_MODELS: return get_cycle_time();
    }
    return 0;
}

void SoftwareSimulator::run_cycle()
{
    process_inputs();
    exec();
    process_outputs();
}

ulong SoftwareSimulator::get_cycle_time()
{
    return 0;
}

void SoftwareSimulator::add_one_input(JNIEnv* jni, jstring key, jobject value)
{
    auto port_name = jni->GetStringUTFChars(key, 0);
    //Log::info << Log::tag << "Received input for port " << port_name << ": ";
    auto port = get_port( port_name );
    jni->ReleaseStringUTFChars( key, port_name );
    if ( port == nullptr )
        return; //TODO Warning ?

    port->receive(jni, value);
}

void SoftwareSimulator::query_outputs(JNIEnv* jni, jclass cls, jobject opaque_hashmap)
{
    for (auto& output_port : output_ports) {
        jstring key = jni->NewStringUTF( output_port->name.c_str() );
        auto new_object = output_port->send(jni);
        jni->CallStaticVoidMethod(cls, JNIEnvironment::instance.CppBridge_addOneOutput_method, opaque_hashmap, key, new_object);
    }
}

std::string SoftwareSimulator::query(const char* msg)
{
    //TODO
    return query_simulator(msg);
}

Port* SoftwareSimulator::get_port(const char* port_name)
{
    auto res = port_map.find(port_name);
    if (res == port_map.end())
        return nullptr;
    return (*res).second;
}

void SoftwareSimulator::process_inputs()
{
    for (auto& port : input_ports) {
        port->process_input();
    }
}

void SoftwareSimulator::process_outputs()
{
    for (auto& port : output_ports) {
        port->process_output();
    }
}
