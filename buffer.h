#pragma once
#include <stdint.h>
#include <string>
#include <exception>

struct DynamicBuffer {
    static constexpr int32_t START_BUFFER_SIZE = 1024;
    char *buffer = nullptr;
    char* pos = nullptr;
    int32_t buffer_size = 0;

    DynamicBuffer() {
        buffer = new char[START_BUFFER_SIZE];
        buffer_size = START_BUFFER_SIZE;
        reset();
    }

    void reset() {
        pos = buffer;
    }

    
    void append(uint8_t c) {
        append((char) c);
    }
    void append(const char c) {
        *pos = c;
        ++pos;
        check_size(position());
    }
    void append(const char* str) {
        while (*str){
            append(*str); ++str;
        }
    }

    // Must be a valid index
    void go_to(int index) {
        this->pos = buffer + index;
    }

    const char *as_terminated_string() {
        *pos = '\0';
        return buffer;
    }

    // Returns the current position in the buffer and increases its content (pos) by "bytes"
    char *push_slot(int bytes) {
        auto t = pos;
        pos += bytes;
        check_size(position());
        return t;
    }

    int position() {
        return pos-buffer;
    }

    void reserve(int size) {
        check_size(size);
    }

    ~DynamicBuffer(){
        if (buffer != nullptr) delete[] buffer;
    }

private:
    void check_size(int size);

};

struct BinaryReader {
    const char* data;
    const char* end;
    const char* pos;

    BinaryReader(const char* data, int size) : data(data), end(data+size), pos(data) {}

    uint64_t read_u64();
    uint32_t read_u32();
    uint16_t read_u16();
    uint8_t read_u8();
    double read_f64();
    std::string read_str();
private:
    void check_range(int bytes, const char* type);
};


struct BinaryWriter {
    DynamicBuffer &buffer;

    BinaryWriter(DynamicBuffer &buffer) : buffer(buffer) {}

    void write_u64(uint64_t value);
    void write_u32(uint32_t value);
    void write_u16(uint16_t value);
    void write_u8(uint8_t value);
    void write_f64(double value);
    void write_str(const std::string &str);
};


class BufferException : public std::exception {
    std::string description;
public:
    BufferException(const std::string& description) : description("BufferException:\n\t" + description) {}
    
    virtual const char* what() const throw()
    {
        return description.c_str();
    }
};

int get_socket_id(BinaryReader &br, int max_count);