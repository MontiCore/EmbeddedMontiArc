#pragma once

#include <stdint.h>
#include <string>
#include <exception>
#include <vector>


constexpr int BACKLOG = 10;

// The threads/functions receiving the socket as part of the session_func callback are responsible to call 'close_sock()' when done.
void session_server(char* port, void (*session_func)(int socket), bool& run_flag);
void close_sock(int sock);

class Socket {
    int socket;
public:
    Socket(int socket) : socket(socket) {}
    void write_s(char *buffer, int count) const;
    void read_s(char *buffer, int count) const;
};


/*
    Packet structure:
    byte: packet ID
    uint16_t: packet payload length
    payload: (*length* bytes)
*/
struct Packet {
    int id;
    int size; // Total size of packet
    char *buffer;
    int buffer_size;

    Packet(char *buffer, int buffer_size) : buffer(buffer), buffer_size(buffer_size), id(-1), size(0) {}
    Packet(char *buffer, int buffer_size, int id) : buffer(buffer), buffer_size(buffer_size), id(id), size(0) {}
};

struct PacketReader {
    Packet packet;
    char *pos;
    
    // Directly reads the packet from the socket
    PacketReader(char *buffer, int buffer_size, const Socket &socket);
    PacketReader(std::vector<char>& buffer, const Socket& socket) : PacketReader(buffer.data(), buffer.size(), socket) {}


    // Skips reading/writing *count* bytes, returns the current read/write position for later use
    char *skip_bytes(int count) {
        auto t = pos;
        pos += count;
        return t;
    }
    void go_to(char* pos) {
        this->pos = pos;
    }
    
    uint64_t read_u64();
    uint32_t read_u32();
    uint16_t read_u16();
    uint8_t read_u8();
    double read_f64();
    std::string read_str();


    private:
    void check_range(int bytes, const char* type);
    int size() {
        return (int) (pos - packet.buffer);
    }
    int payload_size() {
        return size() - 3;
    }
};

struct PacketWriter {
    Packet packet;
    char *pos;

    // Init the packet with 'id' constructor
    PacketWriter(char *buffer, int buffer_size, int packet_id) : packet(buffer, buffer_size, packet_id), pos(buffer) {
        write_u8(packet_id);
        skip_bytes(2); // Write pos at the end
    }
    // Init the packet with 'id' constructor
    PacketWriter(std::vector<char> &buffer, int packet_id) : packet(buffer.data(), buffer.size(), packet_id), pos(buffer.data()) {
        write_u8(packet_id);
        skip_bytes(2); // Write pos at the end
    }

    void send(const Socket &socket);

    // Skips reading/writing *count* bytes, returns the current read/write position for later use
    char *skip_bytes(int count) {
        auto t = pos;
        pos += count;
        return t;
    }
    
    void write_u64(uint64_t value);
    void write_u32(uint32_t value);
    void write_u16(uint16_t value);
    void write_u8(uint8_t value);
    void write_f64(double value);
    void write_str(const std::string &str);

private:
    void check_range(int bytes, const char* type);
    int size() {
        return (int) (pos - packet.buffer);
    }
    int payload_size() {
        return size() - 3;
    }
};



std::string get_last_error_str();
std::string get_last_wsa_error_str();


class NetworkException : public std::exception {
    std::string description;
public:
    NetworkException(const std::string& description) : description("NetworkException:\n\t" + description) {}
    static NetworkException last_err() {
        return NetworkException(get_last_error_str());
    }
    static NetworkException last_err(const std::string& context) {
        return NetworkException(context + ".\n\t\tLast Error: " + get_last_error_str());
    }
    static void throw_network_err(const std::string& context);

    virtual const char* what() const throw()
    {
        return description.c_str();
    }
};
