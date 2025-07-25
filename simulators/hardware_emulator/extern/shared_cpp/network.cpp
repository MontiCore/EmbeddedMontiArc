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
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>

#endif

using namespace std;

void *get_in_addr(struct sockaddr *sa);

#ifdef IS_WIN
void Socket::get_max_msg_size() {
    DWORD target = 0;
    int size = sizeof(DWORD);
    if (getsockopt(socket, SOL_SOCKET, SO_MAX_MSG_SIZE, (char*)&target, &size) != 0) {
        NetworkException::throw_network_err("Error getting the max packet length");
    }
    max_msg_size = target;
}
#else
void Socket::get_max_msg_size() {
    // TODO the max segment length is not the max message length? => remove max tcp header-trailer length?
    // int64_t target = 0;
    // socklen_t size = sizeof(int64_t);
    // if (getsockopt(socket, IPPROTO_TCP, TCP_MAXSEG, (char*)&target, &size) != 0) {
    //     NetworkException::throw_network_err("Error getting the max packet length");
    // }

    // TCP on linux has no max size?
    max_msg_size = INT64_MAX;
}
#endif

void Socket::write_s(char *buffer, int count) const {
    while (count > max_msg_size) {
        auto res = ::send(socket, buffer, max_msg_size, 0);
        if (res != max_msg_size) {
            auto msg = "Could not write " + to_string(max_msg_size) + " bytes";
            if (res < 0) {
                NetworkException::throw_network_err(msg);
            } else {
                throw NetworkException(msg);
            }
        }
        buffer += max_msg_size;
        count -= max_msg_size;
    }
    if (count <= 0) return;
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
void PacketReader::receive() {
    //cout << "Waiting for packet header " << endl;
    id = -1;
    size = 0;
    BinaryReader br = {buffer.get_buffer(), 3};

    buffer.reserve(3);
    size = 3;
    socket.read_s(buffer.get_buffer(), 3);
    id = br.read_u8();
    auto length = br.read_u16();
    //cout << "Got packet header: id=" << id << ", size=" << length << endl;

    size = length + 3;
    buffer.reserve(size);

    // Read data
    socket.read_s(buffer.get_buffer()+3, length);
}



void PacketWriter::send(const Socket &socket) {
    int size = buffer.position();
    auto payload_size = size-3;
    buffer.go_to(1);
    bw.write_u16(payload_size);
    socket.write_s(buffer.get_buffer(), size);
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
string get_last_error_str_net() {
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
