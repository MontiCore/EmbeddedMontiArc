/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#pragma once
#include <string>

/*
    An overview of the port and port information structure can be found at docs/PortStructure.pdf
*/

enum class PortDirection {
    INPUT,
    OUTPUT
};

struct PortType {
    enum class Type {
        DOUBLE,
        INT,
        STRUCT //TODO HANDLE
    } type = Type::INT;
    std::string struct_name; //If type is STRUCT
};

struct StructInfo {
    //vector of PortTypes
};

struct PortDimension {
    enum class Dimension {
        SINGLE,
        ARRAY,
        DYNAMIC
    } dimension = Dimension::SINGLE;
    unsigned int size; //If array or max size if dynamic
    std::string size_accessor_function_name; //TODO
};

struct PortInformation {
    PortType type;
    PortDimension dimension;
    std::string name;
    std::string main_accessor_function_name;
};