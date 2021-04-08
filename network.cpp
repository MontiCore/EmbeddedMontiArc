#include "network.h"
#include <iostream>
#include <cstring>

//using namespace std;

#if defined _WIN32 || defined _WIN64
#define IS_WIN
#else
#endif

#undef UNICODE

#ifdef IS_WIN

#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

#else

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>

#endif

using namespace std;

void *get_in_addr(struct sockaddr *sa);

void Socket::write_s(char *buffer, int count) const {
    TODO handle too long packets; // https://docs.microsoft.com/en-us/windows/win32/api/winsock2/nf-winsock2-send
    auto res = ::send(socket, buffer, count, 0);
    if (res != count) {
        auto msg = "Could not write " + to_string(count) + " bytes";
        if (res < 0) {
            NetworkException::throw_network_err(msg);
        } else {
            throw NetworkException(msg);
        }
    }
}
void Socket::read_s(char *buffer, int count) const {
    auto res = ::recv(socket, buffer, count, MSG_WAITALL);
    if (res != count) {
        auto msg = "Could not read " + to_string(count) + " bytes";
        if (res < 0) {
            NetworkException::throw_network_err(msg);
        } else {
            throw NetworkException(msg);
        }
    }
}


// Reads the packet from the socket and sets the reader to the start of the payload
PacketReader::PacketReader(char *buffer, int buffer_size, const Socket &socket) : packet(buffer, buffer_size), pos(buffer) {
    //cout << "Waiting for packet header " << endl;
    packet.size = 3;
    socket.read_s(packet.buffer, 3);
    packet.id = read_u8();
    auto length = read_u16();
    //cout << "Got packet header: id=" << id << ", size=" << length << endl;

    packet.size = length + 3;
    if (packet.size > packet.buffer_size) throw NetworkException("Packet (id=" + to_string(packet.id) + ") payload too big for buffer: " + to_string(length) + " > " + to_string(packet.buffer_size - 3));

    // Read data
    socket.read_s(pos, length);
}

uint64_t PacketReader::read_u64() {
    check_range(8, "u64");
    uint64_t t = 0;
    for (int i = 0; i < 8; ++i) {
        t <<= 8;
        t |= ((uint8_t*)pos)[i];
    }
    pos += 8;
    return t;
}
uint32_t PacketReader::read_u32() {
    check_range(4, "u32");
    auto buff = (uint8_t*)pos;
    pos += 4;
    return (((uint32_t)buff[0]) << 24) | (((uint32_t)buff[1]) << 16) | (((uint32_t)buff[2]) << 8) | (uint32_t)buff[3];
}
uint16_t PacketReader::read_u16() {
    check_range(2, "u16");
    auto buff = (uint8_t*)pos;
    pos += 2;
    return (((uint16_t)buff[0]) << 8) | (uint16_t)buff[1];
}
uint8_t PacketReader::read_u8() {
    check_range(1, "u8");
    auto buff = (uint8_t*)pos;
    pos += 1;
    return *buff;
}
double PacketReader::read_f64() {
    check_range(8, "f64");
    auto t = read_u64();
    return *((double*)&t);
}
std::string PacketReader::read_str() {
    auto max_pos = packet.buffer + packet.buffer_size;
    if (pos >= max_pos) throw NetworkException("str read past packet payload.");
    auto start = pos;
    while (pos < max_pos && *pos != '\0') ++pos;
    ++pos; // Go after '\0'
    return string(start, pos-start-1);
}

void PacketReader::check_range(int bytes, const char* type) {
    if (size() + bytes > packet.size) throw NetworkException(string(type) + " read past packet payload (packet: id="+to_string(packet.id)+" payload_length="+to_string(packet.size-3)+")");
}





void PacketWriter::send(const Socket &socket) {
    // Packet id and payload are already in 'buffer'
    auto payload = payload_size();
    packet.size = size();
    pos = packet.buffer + 1;
    write_u16(payload);
    socket.write_s(packet.buffer, packet.size);
}


void PacketWriter::write_u64(uint64_t value) {
    check_range(8, "u64");
    for (int i = 0; i < 8; ++i) {
        ((uint8_t*)pos)[7-i] = value & 0xFF;
        value >>= 8;
    }
    pos += 8;
}
void PacketWriter::write_u32(uint32_t value) {
    check_range(4, "u32");
    ((uint8_t*)pos)[0] = value >> 24;
    ((uint8_t*)pos)[1] = value >> 16;
    ((uint8_t*)pos)[2] = value >> 8;
    ((uint8_t*)pos)[3] = value;
    pos += 4;
}
void PacketWriter::write_u16(uint16_t value) {
    check_range(2, "u16");
    ((uint8_t*)pos)[0] = (uint8_t) (value >> 8);
    ((uint8_t*)pos)[1] = (uint8_t) value;
    pos += 2;
}
void PacketWriter::write_u8(uint8_t value) {
    check_range(1, "u8");
    *((uint8_t*)pos) = value;
    pos += 1;
}
void PacketWriter::write_f64(double value) {
    check_range(8, "f64");
    uint64_t t = *((uint64_t*)&value);
    write_u64(t);
}
void PacketWriter::write_str(const string &str) {
    int size = str.size() + 1; // Include terminating char
    check_range(size, "str");
    memcpy(pos, str.c_str(), size);
    pos += size;
}
void PacketWriter::check_range(int bytes, const char* type) {
    if (size() + bytes > packet.buffer_size) throw NetworkException(string(type) + " write past buffer size (packet: id="+to_string(packet.id)+")");
}











void close_sock(int sock) {
    #ifdef IS_WIN
        closesocket(sock);
    #else
        close(sock);
    #endif
}


void session_server(char *port, void (*session_func)(int socket), bool &run_flag) {
#ifdef IS_WIN
    WSADATA wsaData;
    int iResult;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0) {
        NetworkException::throw_network_err("WSAStartup failed");
    }

#endif

    // Setup server
    uint64_t sockfd, new_fd;  // listen on sock_fd, new connection on new_fd
    struct addrinfo hints, *servinfo, *p;
    struct sockaddr_storage their_addr; // connector's address information
    socklen_t sin_size;
    int yes=1;
    char s[INET6_ADDRSTRLEN];
    int rv;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC; // Ipv6 or 6
    hints.ai_socktype = SOCK_STREAM; // TCP
    hints.ai_flags = AI_PASSIVE; // use my IP

    if ((rv = getaddrinfo(NULL, port, &hints, &servinfo)) != 0) {
        #ifdef IS_WIN
        NetworkException::throw_network_err("getaddrinfo()");
        #else 
        throw NetworkException(string("getaddrinfo: ") + gai_strerror(rv));
        #endif
    }

    // loop through all the results and bind to the first we can
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
                p->ai_protocol)) == -1) {
            perror("server: socket");
            continue;
        }

        if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (char*)&yes,
                sizeof(int)) == -1) {
            perror("setsockopt");
            exit(1);
        }

        if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
            close_sock(sockfd);
            perror("server: bind");
            continue;
        }

        break;
    }

    freeaddrinfo(servinfo); // all done with this structure

    if (p == NULL)  {
        throw NetworkException("Failed to bind socket.");
    }

    if (listen(sockfd, BACKLOG) == -1) {
        NetworkException::throw_network_err("listen() failed");
    }


    // Wait for connection (single connection handled)
    while(run_flag) {  // main accept() loop
        cout << "server: waiting for incoming connection..." << endl;
        sin_size = sizeof their_addr;
        new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &sin_size);
        if (new_fd == -1) {
            perror("accept");
            continue;
        }

        inet_ntop(their_addr.ss_family,
            get_in_addr((struct sockaddr *)&their_addr),
            s, sizeof s);
        cout << "server: got connection from " << s << endl;

        session_func(new_fd);
    }

    close_sock(sockfd);
#ifdef IS_WIN
    WSACleanup();
#endif
}
















#ifdef IS_WIN
string get_last_error_str() {
    TCHAR buff[1024];
    DWORD dw = GetLastError();

    auto c = FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL,
        dw, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        buff, 1024, NULL);
    
    if (c == 0) throw NetworkException("Error getting last error string");

    return string(buff);
}
string get_last_wsa_error_str() {
    TCHAR buff[1024];
    DWORD dw = WSAGetLastError();

    auto c = FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL,
        dw, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        buff, 1024, NULL);

    if (c == 0) throw NetworkException::last_err("Error getting last WSA error string");

    return string(buff);
}
void NetworkException::throw_network_err(const std::string& context) {
    throw NetworkException(context + ".\n\t\tLast WSA Error: " + get_last_wsa_error_str());
}
#else
std::string get_last_error_str() {
    auto msg = strerror(errno);
    return std::string(msg);
}
void NetworkException::throw_network_err(const std::string& context) {
    throw NetworkException::last_err(context);
}
#endif

// get sockaddr, IPv4 or IPv6:
void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}
