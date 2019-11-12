/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#include "resolver.h"
#include "utility/utility.h"

using namespace std;

void DynamicInterfaceResolver::discover_interface(DynamicSoftwareInterface& interface) {
    discover_ports(interface, PortDirection::INPUT);
    discover_ports(interface, PortDirection::OUTPUT);
}

//TODO put strings in Header as constants
void DynamicInterfaceResolver::discover_ports(DynamicSoftwareInterface& interface, PortDirection dir)
{
    std::string target = dir == PortDirection::OUTPUT ? "output" : "input";
    auto& ports = dir == PortDirection::OUTPUT ? output_ports : input_ports;
    auto get_count_function_name = GET_COUNT_PREFIX + target;
    auto get_name_function_name = GET_NAME_PREFIX + target;
    auto get_type_function_name = GET_TYPE_PREFIX + target;

    auto accessor_prefix = dir == PortDirection::OUTPUT ? GETTER_PREFIX : SETTER_PREFIX;

    int port_count = interface.get_int(get_count_function_name.c_str());
    

    ports.resize(port_count);
    for (auto i : urange(port_count)) {
        auto& port_info = ports[i];
        port_info.name = interface.get_string_by_id(get_name_function_name.c_str(), i);
        port_info.dimension.dimension = PortDimension::Dimension::SINGLE;
        port_info.main_accessor_function_name = accessor_prefix + port_info.name;
        auto type = interface.get_string_by_id(get_type_function_name.c_str(), i);

        auto base_type = type; //TODO check Array or Dynamic array (and extract base type)

        if (strcmp(base_type, TYPE_DOUBLE_NAME) == 0) {
            port_info.type.type = PortType::Type::DOUBLE;
        }
        else if (strcmp(base_type, TYPE_INT_NAME) == 0)
            port_info.type.type = PortType::Type::INT;
        else if (strcmp(base_type, "CommonMatrixType") == 0) { //Should be of type: Q{SIZE} (array)
            port_info.type.type = PortType::Type::DOUBLE;
            port_info.dimension.dimension = PortDimension::Dimension::ARRAY;
        }
        else {
            //TODO handle struct type && eventual unsupported types
            //For structs, check struct map, if nothing try to resolve the struct discovery functions
            //int get_struct_count__StructType()
            //char* get_struct_name__StructType()
            //char* get_struct_type__StructType()
            //Store struct type in map.
        }
    }
}
