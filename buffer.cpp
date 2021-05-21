#include "buffer.h"
#include <iostream>
#include <cstring>

int get_socket_id(const std::string &ip, int max_count) {
    if (ip.find("2::") != 0) {
        std::cerr << "Unsupported IP: '"<< ip << "' (only supports 'N-to-N' IPs with '2::' prefix)" << std::endl;
    }
    int id = strtol(ip.c_str()+3, nullptr, 10)-1;
    if (id >= max_count) {
        std::cerr << "'N-to-N' IP '" << ip << "' outside of range [1:"<< (max_count) << "]. Ignoring packet." << std::endl;
        return -1;
    }
    return id;
}

void DynamicBuffer::check_size(int size) {
    if (size >= buffer_size) {
        int32_t new_size = ((size / START_BUFFER_SIZE) + 1) * START_BUFFER_SIZE;
        char *new_buff = new char[new_size];
        for (int32_t i = 0; i < pos; ++i){
            new_buff[i] = buffer[i];
        }
        delete[] buffer;
        buffer = new_buff;
        buffer_size = new_size;
    }
}


uint64_t BinaryReader::read_u64() {
    check_range(8, "u64");
    uint64_t t = 0;
    for (int i = 0; i < 8; ++i) {
        t <<= 8;
        t |= ((uint8_t*)(pos))[i];
    }
    pos += 8;
    return t;
}
uint32_t BinaryReader::read_u32() {
    check_range(4, "u32");
    auto buff = (uint8_t*)pos;
    pos += 4;
    return (((uint32_t)buff[0]) << 24) | (((uint32_t)buff[1]) << 16) | (((uint32_t)buff[2]) << 8) | (uint32_t)buff[3];
}
uint16_t BinaryReader::read_u16() {
    check_range(2, "u16");
    auto buff = (uint8_t*)pos;
    pos += 2;
    return (((uint16_t)buff[0]) << 8) | (uint16_t)buff[1];
}
uint8_t BinaryReader::read_u8() {
    check_range(1, "u8");
    auto buff = (uint8_t*)pos;
    pos += 1;
    return *buff;
}
double BinaryReader::read_f64() {
    check_range(8, "f64");
    auto t = read_u64();
    return *((double*)&t);
}
std::string BinaryReader::read_str() {
    if (pos >= end) throw BufferException("str read past buffer content.");
    auto start = pos;
    while (pos < end && *pos != '\0') ++pos;
    ++pos; // Go after '\0'
    return std::string(start, pos-start-1);
}

void BinaryReader::check_range(int bytes, const char* type) {
    if (pos + bytes > end) throw BufferException(std::string(type) + " read past buffer content.");
}




void BinaryWriter::write_u64(uint64_t value) {
    auto pos = buffer.push_slot(8);
    for (int i = 0; i < 8; ++i) {
        ((uint8_t*)pos)[7-i] = value & 0xFF;
        value >>= 8;
    }
}
void BinaryWriter::write_u32(uint32_t value) {
    auto pos = buffer.push_slot(4);
    ((uint8_t*)pos)[0] = value >> 24;
    ((uint8_t*)pos)[1] = value >> 16;
    ((uint8_t*)pos)[2] = value >> 8;
    ((uint8_t*)pos)[3] = value;
}
void BinaryWriter::write_u16(uint16_t value) {
    auto pos = buffer.push_slot(2);
    ((uint8_t*)pos)[0] = (uint8_t) (value >> 8);
    ((uint8_t*)pos)[1] = (uint8_t) value;
}
void BinaryWriter::write_u8(uint8_t value) {
    auto pos = buffer.push_slot(1);
    *((uint8_t*)pos) = value;
}
void BinaryWriter::write_f64(double value) {
    uint64_t t = *((uint64_t*)&value);
    write_u64(t);
}
void BinaryWriter::write_str(const std::string &str) {
    int size = str.size() + 1; // Include terminating char
    auto pos = buffer.push_slot(size);
    ::memcpy(pos, str.c_str(), size);
}