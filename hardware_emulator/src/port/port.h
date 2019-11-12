/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#pragma once
#include "utility/jni_interface.h"
#include "utility/utility.h"
#include <string>
#include "dynamic_interface/port_info.h"
#include "computer/computer.h"

struct Computer;
struct Library;

struct Port {
    //Send/receive from java hashmap (<~> BUS)
    virtual void receive(JNIEnv* jni, jobject value) = 0;
    virtual jobject send(JNIEnv* jni) = 0;

    virtual void receive(Port *other) = 0;

    //Interaction with Software Ports
    virtual void process_input() = 0;
    virtual void process_output() = 0;
    virtual PortType::Type get_port_type() = 0;
    virtual PortDimension::Dimension get_port_dimension() = 0;

    Port(const std::string &name) : name(name) {}

    std::string name;

    virtual ~Port() {}
};

/*
    This is used as template parameter for specialization of specific types (double, int)
    (PortSimple, PortArray)
*/
struct PortTypeInfoDouble {
    // Signature of the native functions
    using InputFunc = void(*)(double);
    using OutputFunc = double(*)();
    using ArrayInputFunc = void(*)(double*, int);
    //Reference to the JNI type specifics
    using JNIType = JNITypeDouble;
    //C data type
    using CType = JNIType::CType;
    //PortInformation Type
    static constexpr auto port_type = PortType::Type::DOUBLE;
    // How to get a double value from the Computer
    static CType get_return(Computer& computer) {
        return computer.func_call->get_return_double();
    }
    // How to pass a double value to the Computer
    static void set_param(Computer& computer, CType param) {
        computer.func_call->set_param1_double(param);
    }
};

struct PortTypeInfoInt {
    // Signature of the native functions
    using InputFunc = void(*)(int);
    using OutputFunc = int(*)();
    using ArrayInputFunc = void(*)(jint*, int);
    //Reference to the JNI type specifics
    using JNIType = JNITypeInt;
    //C data type
    using CType = JNIType::CType;
    //PortInformation Type
    static constexpr auto port_type = PortType::Type::INT;
    // How to get an int value from the Computer
    static inline CType get_return(Computer& computer) {
        return computer.func_call->get_return_32();
    }
    // How to pass an int value to the Computer
    static inline void set_param(Computer& computer, CType param) {
        computer.func_call->set_param1_32(param);
    }
};