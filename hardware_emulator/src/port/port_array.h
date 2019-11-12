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
    PortArray specifies how to pass/read arrays of a base type to/from Java (reveice() and send() ).
    It also specifies the type, dimension and internal data buffer.
*/
template<typename PortTypeInfo>
struct PortArray : public Port {
    //Send/receive from java hashmap (<~> BUS)
    void receive(JNIEnv* jni, jobject value)
    {
        //TODO update with standarized array size/model
        this->size = jni->GetArrayLength((jarray)value);
        if (this->size > this->data.size()) {
            this->data.resize(this->size);
        }
        typename PortTypeInfo::CType* arr = PortTypeInfo::JNIType::get_array(jni, value);
        for (auto i : urange(this->size))
            this->data[i] = arr[i];
        PortTypeInfo::JNIType::free_array(jni,value, arr);

        // Log::debug << Log::tag << "Port " << this->name << " received: [";
        // for (auto i : urange(this->size)) {
        //     if (i > 0) Log::debug << ", ";
        //     Log::debug << this->data[i];
        // }
        // Log::debug << "]\n";
    }
    jobject send(JNIEnv* jni) {
        throw_error("TODO implement.");
    }

    void receive(Port* other) {
        auto& o = *((PortArray*)other);
        this->size = o.size;
        if (this->size > data.size()) {
            this->data.resize(this->size);
        }
        for (auto i : urange(o.size)) {
            this->data[i] = o.data[i];
        }
    }

    PortType::Type get_port_type() {
        return PortTypeInfo::port_type;
    }

    PortDimension::Dimension get_port_dimension() {
        return PortDimension::Dimension::ARRAY;
    }
    
    PortArray(const std::string& name) : Port(name), size(0) {}

    std::vector<typename PortTypeInfo::CType> data;
    uint size;
};

/*
    Specifies how to exchange arrays with Emulated Software
*/
template<typename PortTypeInfo>
struct PortArrayEmu : public PortArray<PortTypeInfo> {
    PortArrayEmu(const PortInformation& info, HardwareEmulator& emu) : PortArray<PortTypeInfo>(info.name), computer(emu.computer)
    {
        emu.resolve(info.main_accessor_function_name, this->emu_function);

        //TODO standarize
        auto& section = this->computer.memory.sys_section;
        auto& section_stack = this->computer.memory.sys_section_stack;
        this->buffer_slot = section_stack.get_annotated(1024, "Exchange buffer for port "+this->name, Annotation::Type::OBJECT);
    }

    //Interaction with Software Ports
    void process_input() {
        if (this->size * sizeof(double) > this->buffer_slot.size)
            throw_error("PortArrayEmu::process_input() with array bigger than computer buffer");
        
        this->computer.memory.write_memory(this->buffer_slot.start_address, this->size * sizeof(double), (uchar*)(this->data.data()));
        this->computer.func_call->set_params_64(this->buffer_slot.start_address, (ulong)this->size);
        this->computer.call(this->emu_function, this->name.c_str());
    }
    void process_output() {
        throw_error("TODO implement.");
    }

private:
    MemoryRange buffer_slot;
    Computer& computer;
    //Getter/Setter function of the DynamicSoftware
    uint64_t emu_function;
};

/*
    Specifies how to exchange arrays with Native Software
*/
template<typename PortTypeInfo>
struct PortArrayDirect : public PortArray<PortTypeInfo> {
    PortArrayDirect(const PortInformation& info, DirectSoftwareSimulator& sim) : PortArray<PortTypeInfo>(info.name)
    {
        sim.resolve_real(info.main_accessor_function_name, this->direct_function);
    }

    //Interaction with Software Ports
    void process_input() {
        //TODO standard array structure
        ((typename PortTypeInfo::ArrayInputFunc)(this->direct_function))(this->data.data(), this->size);
    }
    void process_output() {
        throw_error("TODO implement");
    }
private:
    //Getter/Setter function of the DynamicSoftware
    void* direct_function;
};

/*
    Emulator/Native PortArrays for different types
*/
using PortIntArrayEmu = PortArrayEmu<PortTypeInfoInt>;
using PortIntArrayDirect = PortArrayDirect<PortTypeInfoInt>;
using PortDoubleArrayEmu = PortArrayEmu<PortTypeInfoDouble>;
using PortDoubleArrayDirect = PortArrayDirect<PortTypeInfoDouble>;