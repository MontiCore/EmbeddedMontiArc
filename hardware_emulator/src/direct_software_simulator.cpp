/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#include "direct_software_simulator.h"
#include "port/port_simple.h"
#include "port/port_array.h"

std::string DirectSoftwareSimulator::query_simulator(const char* msg)
{
    return std::string();
}

void DirectSoftwareSimulator::init_simulator(SoftwareSimulatorManager& manager, const char* config)
{
    software.init(software_path);

    resolve_real("init", real_init);
    resolve_real("execute", real_exec);

    ((InitFunc)real_init)();

    Log::info << Log::tag << "Initiated software in direct mode: " << software_name << "\n";
}

void DirectSoftwareSimulator::exec()
{
    ((ExecFunc)real_exec)();
}

Port* DirectSoftwareSimulator::new_port_by_type(const PortInformation& info)
{
    if (info.dimension.dimension == PortDimension::Dimension::SINGLE) {
        switch (info.type.type) {
        case PortType::Type::INT:
            return new PortIntDirect(info, *this);
        case PortType::Type::DOUBLE:
            return new PortDoubleDirect(info, *this);
        }
    } else if (info.dimension.dimension == PortDimension::Dimension::ARRAY) {
        switch (info.type.type) {
        case PortType::Type::INT:
            return new PortIntArrayDirect(info, *this);
        case PortType::Type::DOUBLE:
            return new PortDoubleArrayDirect(info, *this);
        }
    }
    else if (info.dimension.dimension == PortDimension::Dimension::DYNAMIC) {

    }
    return nullptr;
}

const char* DirectSoftwareSimulator::get_string_by_id(const char* name, int id)
{
    void* function_address;
    resolve_real(name, function_address);
    using GetStringFunc = const char*(*)(int);
    return ((GetStringFunc)function_address)(id);
}

int DirectSoftwareSimulator::get_int(const char* name)
{
    void* function_address;
    resolve_real(name, function_address);
    using GetCountFunc = int(*)();
    return ((GetCountFunc)function_address)();
}

void DirectSoftwareSimulator::resolve_real( const std::string &name, void *&target ) {
    target = software.get_function( name.c_str() );
}
