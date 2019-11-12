/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#pragma once
#include <vector>
#include <string>
#include "port_info.h"

/*
    A SoftwareSimulator must implement this interface to allow the DynamicInterfaceResolver to call the discovery functions of the Software.
    get_int(name) must call the function "name" of the software (no parameters) and return an integer.
    get_string_by_id(name, id) must call the function "name" of the software (with parameter id) and return a null-terminated string.
*/
struct DynamicSoftwareInterface {
    virtual const char* get_string_by_id(const char *name, int id) = 0;
    virtual int get_int(const char *name) = 0;
    virtual ~DynamicSoftwareInterface() {}
};

/*
    The DynamicInterfaceResolver calls the discovery functions of the Dynamic Interface and creates a representation
    of its input and output ports.
    An overview of the PortInformation structure and how it is used to create the Ports by the SoftwareSimulator can be 
    found in docs/PortStructure.pdf
    
    TODO document the DynamicInterface
*/
struct DynamicInterfaceResolver {
    static constexpr auto GET_COUNT_PREFIX = "get_count_";
    static constexpr auto GET_NAME_PREFIX = "get_name_";
    static constexpr auto GET_TYPE_PREFIX = "get_type_";
    static constexpr auto GETTER_PREFIX = "get_output__";
    static constexpr auto SETTER_PREFIX = "set_input__";


    static constexpr auto TYPE_DOUBLE_NAME = "Q";
    static constexpr auto TYPE_INT_NAME = "Z";

    std::vector<PortInformation> input_ports;
    std::vector<PortInformation> output_ports;
    //TODO Map of StructInfo for already discovered structs

    void discover_interface(DynamicSoftwareInterface &interface);

private:
    void discover_ports(DynamicSoftwareInterface& interface, PortDirection dir);
};