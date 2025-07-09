<#-- (c) https://github.com/MontiCore/monticore -->

#ifndef DynamicHelper_h
#define DynamicHelper_h

#include <queue>
#include <vector>

#ifndef CONNECTION_H
#define CONNECTION_H
template<typename T>
struct connection {
void *beforeComponent;
T *source;
T *target;
};
//template<typename T, typename T2>
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

template<typename T>
void dynamicconnect_remove(std::vector<connection<T>>* vec, void* ac, T* source, T* target){
for(long i = vec->size()-1; i >= 0; --i){
connection<T> c = vec->at(i);
if( (c.beforeComponent == ac) && (c.source == source) && (c.target == target)){
vec->erase(vec->begin()+i);
}
}
}

#endif /* DynamicHelper_h */
