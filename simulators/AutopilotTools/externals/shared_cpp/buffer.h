#pragma once
#include <stdint.h>
#include <string>



// Used for "snprintf" target
constexpr int32_t LOCAL_BUFFER_SIZE = 4096;
extern char LOCAL_BUFFER[LOCAL_BUFFER_SIZE];

struct DynamicBuffer {

    DynamicBuffer() {
        buffer = new char[START_BUFFER_SIZE];
        buffer_size = START_BUFFER_SIZE;
        pos = 0;
    }

    void reset() {
        pos = 0;
    }

    
    void append(uint8_t c) {
        append((char) c);
    }
    void append(const char c) {
        check_size(pos);
        buffer[pos] = c;
        ++pos;
    }
    void append(const char* str) {
        while (*str){
            append(*str); ++str;
        }
    }

    // Must be a valid index
    void go_to(int index) {
        this->pos = index;
    }

    const char *as_terminated_string() {
        buffer[pos] = '\0';
        return buffer;
    }

    // Returns the current position in the buffer and increases its content (pos) by "bytes"
    char *push_slot(int bytes) {
        auto t = pos;
        auto new_pos = pos+bytes;
        check_size(new_pos);
        pos = new_pos;
        return buffer+t;
    }

    int position() {
        return pos;
    }

    void reserve(int size) {
        check_size(size);
    }

    char *get_buffer() {
        return buffer;
    }

    ~DynamicBuffer(){
        if (buffer != nullptr) delete[] buffer;
    }

private:

    static constexpr int32_t START_BUFFER_SIZE = 1024;
    char *buffer;
    int32_t pos;
    int32_t buffer_size;

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

#ifdef NO_ERR_OUT

class BufferException : public std::exception {
    std::string description;
public:
    BufferException(const std::string& description) : description("BufferException:\n\t" + description) {}
    
    virtual const char* what() const throw()
    {
        return description.c_str();
    }
};

#endif

int get_socket_id(const std::string &ip, int32_t max_count);