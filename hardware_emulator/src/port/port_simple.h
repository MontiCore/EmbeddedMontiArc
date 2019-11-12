/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#pragma once
#include "port.h"
#include "dynamic_interface/port_info.h"
#include "computer/computer.h"
#include "hardware_emulator.h"
#include "direct_software_simulator.h"



 /*
     PortSimple specifies how to pass/read basic type to/from Java (reveice() and send() ).
     It also specifies the type, dimension and internal data buffer.
 */
template<typename PortTypeInfo>
struct PortSimple : public Port {
    typename PortTypeInfo::CType data;
    //Send/receive from java hashmap (<~> BUS)
    void receive(JNIEnv* jni, jobject value)
    {
        data = PortTypeInfo::JNIType::call_type_method(jni, value);
        //Log::debug << Log::tag << "Port " << this->name << " received: " << data << '\n';
    }
    jobject send(JNIEnv* jni) {
        return PortTypeInfo::JNIType::new_object(jni, this->data);
    }

    void receive(Port* other) {
        this->data = ((PortSimple*)other)->data;
    }

    PortType::Type get_port_type() {
        return PortTypeInfo::port_type;
    }

    PortDimension::Dimension get_port_dimension() {
        return PortDimension::Dimension::SINGLE;
    }

    PortSimple(const std::string& name) : Port(name), data(0) {}

};

/*
    Specifies how to exchange basic types with Emulated Software
*/
template<typename PortTypeInfo>
struct PortSimpleEmu : public PortSimple<PortTypeInfo> {
    //TODO function name from PortTypeInfo
    PortSimpleEmu(const PortInformation& info, HardwareEmulator& emu) : PortSimple<PortTypeInfo>(info.name), computer(emu.computer)
    {
        emu.resolve(info.main_accessor_function_name, this->emu_function);
    }

    //Interaction with Software Ports
    void process_input() {
        PortTypeInfo::set_param(this->computer, this->data);
        computer.call(this->emu_function, this->name.c_str());
    }
    void process_output() {
        computer.call(this->emu_function, this->name.c_str());
        this->data = PortTypeInfo::get_return(this->computer);
    }

private:
    Computer& computer;
    //Getter/Setter function of the DynamicSoftware
    uint64_t emu_function;
};

/*
    Specifies how to exchange basic types with Native Software
*/
template<typename PortTypeInfo>
struct PortSimpleDirect : public PortSimple<PortTypeInfo> {
    PortSimpleDirect(const PortInformation& info, DirectSoftwareSimulator &sim) : PortSimple<PortTypeInfo>(info.name)
    {
        sim.resolve_real(info.main_accessor_function_name, this->direct_function);
    }

    //Interaction with Software Ports
    void process_input() {
        ((typename PortTypeInfo::InputFunc)(this->direct_function))(this->data);
    }
    void process_output() {
        this->data = ((typename PortTypeInfo::OutputFunc)(this->direct_function))();
    }
private:
    //Getter/Setter function of the DynamicSoftware
    void* direct_function;
};

/*
    Emulator/Native PortSimples for different types
*/
using PortIntEmu = PortSimpleEmu<PortTypeInfoInt>;
using PortIntDirect = PortSimpleDirect<PortTypeInfoInt>;
using PortDoubleEmu = PortSimpleEmu<PortTypeInfoDouble>;
using PortDoubleDirect = PortSimpleDirect<PortTypeInfoDouble>;