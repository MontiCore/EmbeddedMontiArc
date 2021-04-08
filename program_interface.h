#pragma once

#include <vector>
#include <string>
#include <memory>
#include "network.h"

#ifdef USE_DDC
#include "ddc/cppwrapper/ddcwrapper.h"
using namespace dsa::cpp::ddc;
#endif

struct DataType {
    virtual ~DataType() {}

    virtual std::string to_string() = 0;

    // virtual int deserialize(char *buffer, int size, void *port_data) = 0;
    // virtual int serialize(char *buffer, void *port_data) = 0; // Returns number of written bytes

#ifdef USE_DDC
    virtual void packet_to_ddc(DdcWrapper &ddc, PacketReader &packet, int &slot_id) const = 0;
    virtual void ddc_to_packet(DdcWrapper &ddc, PacketWriter &packet, int &slot_id) const = 0;
#endif
};

struct BasicType : public DataType {
    enum Type {
        Q,
        Z,
        N1,
        N,
        C,
        BOOLEAN,
        VEC2,
        VEC3,
        EMPTY
    };
    Type type;

    std::string to_string();

    // int deserialize(char *buffer, int size, void *port_data);
    // int serialize(char *buffer, void *port_data);
#ifdef USE_DDC
    void packet_to_ddc(DdcWrapper &ddc, PacketReader &packet, int &slot_id) const;
    void ddc_to_packet(DdcWrapper &ddc, PacketWriter &packet, int &slot_id) const;
#endif
};

struct VectorType : public DataType {
    std::unique_ptr<DataType> base_type;
    uint32_t size;
    
    std::string to_string();

    // int deserialize(char *buffer, int size, void *port_data);
    // int serialize(char *buffer, void *port_data);
#ifdef USE_DDC
    void packet_to_ddc(DdcWrapper &ddc, PacketReader &packet, int &slot_id) const;
    void ddc_to_packet(DdcWrapper &ddc, PacketWriter &packet, int &slot_id) const;
#endif
};

struct MatrixType : public DataType {
    std::unique_ptr<DataType> base_type;
    uint32_t rows;
    uint32_t columns;
    
    std::string to_string();

    // int deserialize(char *buffer, int size, void *port_data);
    // int serialize(char *buffer, void *port_data);
#ifdef USE_DDC
    void packet_to_ddc(DdcWrapper &ddc, PacketReader &packet, int &slot_id) const;
    void ddc_to_packet(DdcWrapper &ddc, PacketWriter &packet, int &slot_id) const;
#endif
};

struct StructType : public DataType {
    std::string name;
    std::vector<std::unique_ptr<DataType>> field_types;
    std::vector<std::string> field_names;
    
    std::string to_string();

    // int deserialize(char *buffer, int size, void *port_data);
    // int serialize(char *buffer, void *port_data);
#ifdef USE_DDC
    void packet_to_ddc(DdcWrapper &ddc, PacketReader &packet, int &slot_id) const;
    void ddc_to_packet(DdcWrapper &ddc, PacketWriter &packet, int &slot_id) const;
#endif
};

struct SimplePacketType : public DataType {
    std::unique_ptr<DataType> payload_type;
    
    std::string to_string();

#ifdef USE_DDC
    void packet_to_ddc(DdcWrapper &ddc, PacketReader &packet, int &slot_id) const;
    void ddc_to_packet(DdcWrapper &ddc, PacketWriter &packet, int &slot_id) const;
#endif
};

enum PortDirection {
    INPUT,
    OUTPUT,
    IO
};
enum PortType {
    DATA,
    SOCKET
};
struct PortInformation {
    std::string name;
    std::unique_ptr<DataType> data_type;
    PortType port_type;
    PortDirection dir;
    bool allows_multiple_inputs;
    bool optional;

    bool is_input() {
        return dir == INPUT || dir == IO;
    }
    bool is_output() {
        return dir == OUTPUT || dir == IO;
    }
};
struct ProgramInterface {
    std::string name;
    std::string version;
    std::vector<PortInformation> ports;

    std::string to_string();
};

void parse_interface(const char* interface, ProgramInterface &target);

class ProgramInterfaceException : public std::exception {
    std::string description;
public:
    ProgramInterfaceException(const std::string& description) : description("ProgramInterface Exception:\n\t" + description) {}
    virtual const char* what() const throw()
    {
        return description.c_str();
    }
};
