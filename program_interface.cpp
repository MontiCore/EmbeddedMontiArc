#include "program_interface.h"
#include "json.h"
#include "utils.h"
#include <iostream>

using namespace std;

DataType *parse_type(JsonTraverser &j);
DataType *parse_basic_type(JsonTraverser &j, ObjectIterator &it, ObjectIterator &end);
DataType *parse_vector_type(JsonTraverser &j, ObjectIterator &it, ObjectIterator &end);
DataType *parse_matrix_type(JsonTraverser &j, ObjectIterator &it, ObjectIterator &end);
DataType *parse_struct_type(JsonTraverser &j, ObjectIterator &it, ObjectIterator &end);
DataType *parse_simple_packet_type(JsonTraverser &j, ObjectIterator &it, ObjectIterator &end);

void parse_interface(const char* interface, ProgramInterface &target) {
    JsonTraverser j{interface};

    for (auto e : j.stream_object()) {
        if (e.equals("name")) {
            target.name = j.get_string().get_json_string();
        } else if (e.equals("version")) {
            target.version = j.get_string().get_json_string();
        } else if (e.equals("ports")) {
            for (auto t : j.stream_array()) {
                target.ports.emplace_back();
                auto& port = target.ports[target.ports.size() - 1];
                for (auto e2 : j.stream_object()) {
                    if (e2.equals("name")) {
                        port.name = j.get_string().get_json_string();
                    } else if (e2.equals("data_type")) {
                        port.data_type = unique_ptr<DataType>(parse_type(j));
                    } else if (e2.equals("direction")) {
                        auto v = j.get_string();
                        if (v.equals("INPUT")) {
                            port.dir = INPUT;
                        } else if (v.equals("OUTPUT")) {
                            port.dir = OUTPUT;
                        } else if (v.equals("IO")) {
                            port.dir = IO;
                        } else {
                            throw ProgramInterfaceException("Unknown 'PortDirection': " + v.get_json_string());
                        }
                    } else if (e2.equals("port_type")) {
                        auto v = j.get_string();
                        if (v.equals("DATA")) {
                            port.port_type = DATA;
                        } else if (v.equals("SOCKET")) {
                            port.port_type = SOCKET;
                        } else {
                            throw ProgramInterfaceException("Unknown 'PortType': " + v.get_json_string());
                        }
                    } else if (e2.equals("allows_multiple_inputs")) {
                        port.allows_multiple_inputs = j.get_bool();
                    } else if (e2.equals("optional")) {
                        port.optional = j.get_bool();
                    } else if (e2.equals("tags")) {
                        for (auto e : j.stream_array()) {
                            port.tags.emplace_back(j.get_string().get_json_string());
                        }
                    } else {
                        throw ProgramInterfaceException("Unknown 'PortInformation' json entry: " + e2.get_json_string());
                    }
                }
            }
        } else {
            throw ProgramInterfaceException("Unknown 'ProgramInterface' json entry: " + e.get_json_string());
        }
    }
}

DataType *parse_type(JsonTraverser &j) {
    bool has_type = false;
    auto stream = j.stream_object();
    auto it = stream.begin();
    auto end = stream.end();
    if (it != end) {
        auto e = *it;
        if (e.equals("type")) {
            auto type = j.get_string();
            ++it;
            if (type.equals("basic")) {
                return parse_basic_type(j, it, end);
            } else if (type.equals("vector")) {
                return parse_vector_type(j, it, end);
            } else if (type.equals("matrix")) {
                return parse_matrix_type(j, it, end);
            } else if (type.equals("struct")) {
                return parse_struct_type(j, it, end);
            } else if (type.equals("simple_packet")) {
                return parse_simple_packet_type(j, it, end);
            } else {
                throw ProgramInterfaceException("Unknown DataType: " + type.get_json_string());
            }
        } else {
            throw ProgramInterfaceException("Expected 'type' entry for DataType but got: " + e.get_json_string());
        }
    } else {
        throw ProgramInterfaceException("Expected 'type' entry for DataType but got nothing.");
    }
}

DataType *parse_basic_type(JsonTraverser &j, ObjectIterator &it, ObjectIterator &end) {
    auto type = make_unique<BasicType>();

    for (; it != end; ++it) {
        auto e = *it;
        if (e.equals("base_type")) {
            auto tn = j.get_string();
            if (tn.equals("Q") || tn.equals("double")) {
                type->type = BasicType::Q;
            } else if (tn.equals("Z") || tn.equals("int")) {
                type->type = BasicType::Z;
            } else if (tn.equals("N")) {
                type->type = BasicType::N;
            } else if (tn.equals("N1")) {
                type->type = BasicType::N1;
            } else if (tn.equals("C")) {
                type->type = BasicType::C;
            } else if (tn.equals("boolean")) {
                type->type = BasicType::BOOLEAN;
            } else if (tn.equals("void")) {
                type->type = BasicType::EMPTY;
            } else if (tn.equals("vec2")) {
                type->type = BasicType::VEC2;
            } else if (tn.equals("vec3")) {
                type->type = BasicType::VEC3;
            } else {
                throw ProgramInterfaceException("Unknown 'BasicType' type: " + tn.get_json_string());
            }
        } else {
            throw ProgramInterfaceException("Unknown 'BasicType' json entry: " + e.get_json_string());
        }
    }

    return type.release();
}

DataType *parse_vector_type(JsonTraverser &j, ObjectIterator &it, ObjectIterator &end) {
    auto type = make_unique<VectorType>();

    for (; it != end; ++it) {
        auto e = *it;
        if (e.equals("base_type")) {
            type->base_type = unique_ptr<DataType>(parse_type(j));
        } else if (e.equals("size")) {
            type->size = j.get_long();
        } else {
            throw ProgramInterfaceException("Unknown 'VectorType' json entry: " + e.get_json_string());
        }
    }

    return type.release();
}

DataType *parse_matrix_type(JsonTraverser &j, ObjectIterator &it, ObjectIterator &end) {
    auto type = make_unique<MatrixType>();

    for (; it != end; ++it) {
        auto e = *it;
        if (e.equals("base_type")) {
            type->base_type = unique_ptr<DataType>(parse_type(j));
        } else if (e.equals("rows")) {
            type->rows = j.get_long();
        } else if (e.equals("columns")) {
            type->columns = j.get_long();
        } else {
            throw ProgramInterfaceException("Unknown 'MatrixType' json entry: " + e.get_json_string());
        }
    }

    return type.release();
}

DataType *parse_struct_type(JsonTraverser &j, ObjectIterator &it, ObjectIterator &end) {
    auto type = make_unique<StructType>();

    for (; it != end; ++it) {
        auto e = *it;
        if (e.equals("name")) {
            type->name = j.get_string().get_json_string();
        } else if (e.equals("fields")) {
            for (auto t : j.stream_object()) {
                type->field_names.push_back(t.get_json_string());
                type->field_types.emplace_back(parse_type(j));
            }
        } else {
            throw ProgramInterfaceException("Unknown 'StructType' json entry: " + e.get_json_string());
        }
    }

    return type.release();
}


DataType *parse_simple_packet_type(JsonTraverser &j, ObjectIterator &it, ObjectIterator &end) {
    auto type = make_unique<SimplePacketType>();

    for (; it != end; ++it) {
        auto e = *it;
        if (e.equals("payloadType")) {
            type->payload_type = unique_ptr<DataType>(parse_type(j));
        } else {
            throw ProgramInterfaceException("Unknown 'SimplePacketType' json entry: " + e.get_json_string());
        }
    }

    return type.release();
}

std::string BasicType::to_string() {
    switch (type) {
        case Q: return "Q";
        case Z: return "Z";
        case N1: return "N1";
        case N: return "N";
        case BOOLEAN: return "B";
        case C: return "C";
        case VEC2: return "Vec2";
        case VEC3: return "Vec3";
        case EMPTY:  return "void";
        default:
            throw ProgramInterfaceException("Missing Case");
    }
}

std::string VectorType::to_string() {
    return '[' + ::to_string(size)+"; "+base_type->to_string()+']';
}

std::string MatrixType::to_string() {
    return '[' + ::to_string(rows)+'x'+::to_string(columns)+"; "+base_type->to_string()+']';
}

std::string StructType::to_string() {
    return "struct "+ name + " { ... }";
}

std::string SimplePacketType::to_string() {
    return "simple_packet { " + payload_type->to_string() + " }";
}

std::string ProgramInterface::to_string() {
    std::string res = "ProgramInterface of '"+name+"' (v"+version+"):\n";
    for (auto &p : ports) {
        res += '\t';
        if (p.dir == INPUT) res += "I ";
        else if (p.dir == OUTPUT) res += "O ";
        else res += "IO";
        if (p.port_type == SOCKET) res += " [socket]";
        res += p.name +": "+p.data_type->to_string()+'\n';
    }
    return res;
}

/*
    PACKET / DDC conversions
*/

#ifdef USE_DDC
void BasicType::packet_to_ddc(DdcWrapper &ddc, PacketReader &packet, int &slot_id) const {
    switch (type) {
        case Q:
            ddc.saveDataDouble(slot_id, packet.read_f64());
            ++slot_id;
        break;
        case Z:
        case N1:
        case N:
            ddc.saveDataInt32(slot_id, packet.read_u32());
            ++slot_id;
        break;
        case BOOLEAN:
            ddc.saveDataBool(slot_id, packet.read_u8() != 0);
            ++slot_id;
        break;
        case C:
        case VEC2:
            ddc.saveDataDouble(slot_id, packet.read_f64());
            ddc.saveDataDouble(slot_id+1, packet.read_f64());
            slot_id+=2;
        break;
        case VEC3:
            ddc.saveDataDouble(slot_id, packet.read_f64());
            ddc.saveDataDouble(slot_id+1, packet.read_f64());
            ddc.saveDataDouble(slot_id+2, packet.read_f64());
            slot_id+=3;
        break;
        case EMPTY: break;
        default:
            throw ProgramInterfaceException("Missing Case");
    }
}

void BasicType::ddc_to_packet(DdcWrapper &ddc, PacketWriter &packet, int &slot_id) const {
    switch (type) {
        case Q: {
            double v = 0;
            //cout << "Reading from DDC (slot " << slot_id << ")" << endl;
            ddc.readDataDouble(slot_id, v);
            //cout << "Writing to packet" << endl;
            packet.write_f64(v);
            ++slot_id;
        } break;
        case Z:
        case N1:
        case N: {
            int32_t v = 0;
            ddc.readDataInt32(slot_id, v);
            packet.write_u32(v);
            ++slot_id;
        } break;
        case BOOLEAN: {
            bool v = false;
            ddc.readDataBool(slot_id, v);
            packet.write_u8(v ? 1 : 0);
            ++slot_id;
        } break;
        case C:
        case VEC2: {
            double v = 0;
            ddc.readDataDouble(slot_id, v);
            packet.write_f64(v);
            ddc.readDataDouble(slot_id+1, v);
            packet.write_f64(v);
            slot_id += 2;
        } break;
        case VEC3: {
            double v = 0;
            ddc.readDataDouble(slot_id, v);
            packet.write_f64(v);
            ddc.readDataDouble(slot_id+1, v);
            packet.write_f64(v);
            ddc.readDataDouble(slot_id+2, v);
            packet.write_f64(v);
            slot_id += 3;
        } break;
        case EMPTY: break;
        default:
            throw ProgramInterfaceException("Missing Case");
    }
}



void VectorType::packet_to_ddc(DdcWrapper &ddc, PacketReader &packet, int &slot_id) const {
    for (int i = 0; i < size; ++i) {
        base_type->packet_to_ddc(ddc, packet, slot_id);
    }
}

void VectorType::ddc_to_packet(DdcWrapper &ddc, PacketWriter &packet, int &slot_id) const {
    for (int i = 0; i < size; ++i) {
        base_type->ddc_to_packet(ddc, packet, slot_id);
    }
}



void MatrixType::packet_to_ddc(DdcWrapper &ddc, PacketReader &packet, int &slot_id) const {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < columns; ++j) {
            packet_to_ddc(ddc, packet, slot_id);
        }
    }
}

void MatrixType::ddc_to_packet(DdcWrapper &ddc, PacketWriter &packet, int &slot_id) const {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < columns; ++j) {
            ddc_to_packet(ddc, packet, slot_id);
        }
    }
}



void StructType::packet_to_ddc(DdcWrapper &ddc, PacketReader &packet, int &slot_id) const {
    auto count = field_types.size();
    for (int i = 0; i < count; ++i) {
        field_types[i]->packet_to_ddc(ddc, packet, slot_id);
    }
}
void StructType::ddc_to_packet(DdcWrapper &ddc, PacketWriter &packet, int &slot_id) const {
    auto count = field_types.size();
    for (int i = 0; i < count; ++i) {
        field_types[i]->ddc_to_packet(ddc, packet, slot_id);
    }
}

void SimplePacketType::packet_to_ddc(DdcWrapper &ddc, PacketReader &packet, int &slot_id) const {
    // Sender string
    std::string sender = packet.read_str();
    ddc.saveDataString(slot_id, sender);
    ++slot_id;
    
    // Payload
    payload_type->packet_to_ddc(ddc, packet, slot_id);
}
void SimplePacketType::ddc_to_packet(DdcWrapper &ddc, PacketWriter &packet, int &slot_id) const {
    // Sender string
    std::string target;
    ddc.readDataString(slot_id, target);
    ++slot_id;
    packet.write_str(target);
    
    // Payload
    payload_type->ddc_to_packet(ddc, packet, slot_id);
}
#endif

/*
    SERIALIZATION
*/



// int BasicType::deserialize(char *buffer, int size, void *port_data) {
//     switch (type) {
//         case Q: {
//             if (size < 8) {
//                 cerr << "double data too small" << endl;
//                 return 0;
//             }
//             *((double*) port_data) = read_f64(buffer);
//             return 8;
//         }
//         case VEC2:{
//             if (size < 16) {
//                 cerr << "vec2 data too small" << endl;
//                 return 0;
//             }
//             vec2f64 *target = (vec2f64*) port_data;
//             target->x = read_f64(buffer);
//             target->y = read_f64(buffer);
//             return 16;
//         }
//         default:
//         cerr << "Unimplemented BasicType::deserialize" << endl;
//         return 0;
//     }
// }
// int BasicType::serialize(char *buffer, void *port_data) {
//     switch (type) {
//         case Q: {
//             write_f64(buffer, *((double*) port_data));
//             return 8;
//         }
//         default:
//         cerr << "Unimplemented BasicType::serialize" << endl;
//         return 0;
//     }
// }


// int VectorType::deserialize(char *buffer, int size, void *port_data) {
//     if (size < 2) {
//         cerr << "VectorType data too small" << endl;
//         return 0;
//     }
//     auto vec_size = read_u16(buffer);
//     if (size < vec_size + 2){
//         cerr << "VectorType data too small" << endl;
//         return 0;
//     }
//     auto basic_type = dynamic_cast<BasicType*>(base_type.get());
//     if (basic_type == nullptr) {
//         cerr << "Unimplemented VectorType::deserialize" << endl;
//         return 0;
//     }
//     if (basic_type->type != BasicType::Q) {
//         cerr << "Unimplemented VectorType::deserialize" << endl;
//         return 0;
//     }
//     std::vector<double> &target = *(std::vector<double>*)port_data;
//     for (int i = 0; i < vec_size; ++i) {
//         target[i] = read_f64(buffer);
//     }
//     return 2 + 8 * vec_size;
// }
// int VectorType::serialize(char *buffer, void *port_data) {
//     cerr << "Unimplemented VectorType::serialize" << endl;
//     return 0;
// }


// int MatrixType::deserialize(char *buffer, int size, void *port_data) {
//     cerr << "Unimplemented MatrixType::deserialize" << endl;
//     return 0;
// }
// int MatrixType::serialize(char *buffer, void *port_data) {
//     cerr << "Unimplemented MatrixType::serialize" << endl;
//     return 0;
// }


// int StructType::deserialize(char *buffer, int size, void *port_data) {
//     cerr << "Unimplemented StructType::deserialize" << endl;
//     return 0;
// }
// int StructType::serialize(char *buffer, void *port_data) {
//     cerr << "Unimplemented StructType::serialize" << endl;
//     return 0;
// }