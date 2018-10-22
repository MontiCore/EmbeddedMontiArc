#ifndef Dynamic_Connect_Helper_h
#define Not_h

#include <queue>
#include <vector>

#ifndef CONNECTION_H
#define CONNECTION_H
template<typename T>
struct connection {
    T *from;
    T *to;
};
#endif


int dynamicconnect(int numPorts, bool* connected){
    int port = -1;
    for (port = 0; port < numPorts; ++port) {
        if (!connected[port]) {
            break;
        }
    }
    if (port >= numPorts) {
        //no free ports
        return -1;
    }
    connected[port] = true;
    return port;
}

int dynamicconnect(int numPorts, bool* connected, std::queue<int>* request){
    int port = dynamicconnect(numPorts, connected);
    if(port >= 0){
        request->push(port);
    }
    return port;
}


#endif /* Dynamic_Connect_Helper_h */