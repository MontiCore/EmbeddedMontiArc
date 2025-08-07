#pragma once

#include <stdint.h>
#include <string>
#include <exception>
#include <vector>

#include "buffer.h"

constexpr int BACKLOG = 10;

// The threads/functions receiving the socket as part of the session_func callback are responsible to call 'close_sock()' when done.
void session_server(char* port, void (*session_func)(int socket), bool& run_flag);
void close_sock(int sock);

class Socket {
    int socket;
    int64_t max_msg_size;
public:
    Socket(int socket) : socket(socket) {
        get_max_msg_size();
    }
    void get_max_msg_size();
    void write_s(char *buffer, int count) const;
    void read_s(char *buffer, int count) const;
};


/*
    Packet structure:
    byte: packet ID
    uint16_t: packet payload length
    payload: (*length* bytes)
*/


struct PacketReader {
    int id;
    int size; // Total size of packet

    DynamicBuffer &buffer;
    const Socket &socket;

    PacketReader(DynamicBuffer &buffer, const Socket &socket) : buffer(buffer), id(-1), size(0), socket(socket) {}
    void receive();
    BinaryReader getReader() {
        return {buffer.get_buffer()+3, payload_size()};
    }
    const char* getPayload() {
        buffer.get_buffer()[size] = '\0';
        return buffer.get_buffer() + 3;
    }
    
    int payload_size() {
        return size - 3;
    }
};



struct PacketWriter {

    DynamicBuffer &buffer;
    BinaryWriter bw;

    PacketWriter(DynamicBuffer &buffer, int packet_id) : buffer(buffer), bw(buffer) {
        buffer.reset();
        bw.write_u8(packet_id);
        bw.write_u16(0); // Write payload size at the end
    }

    bool has_payload() {
        return buffer.position() > 3;
    }
    void send(const Socket &socket);

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
