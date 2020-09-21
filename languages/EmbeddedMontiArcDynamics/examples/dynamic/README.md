<!-- (c) https://github.com/MontiCore/monticore -->
# Dynamic (Simple) Example

A simple example with dynamic request and connection of ports. 

TODO:
- free port connection
- free an instance (when & how)

## EMAD Files:

### NotAdapter.emad

```
package test1;

component NotAdapter {
    ports 
        dynamic in B in1[0:32],
        dynamic out B out1[0:32];
       
    instance Not dynamic notInstance [0:42];
     
    @ in1::connect && out1::connect {
        
        connect in1[?] -> notInstance.in1;
        connect notInstance.out1 -> out1[?];
    }
    
}
```

### Not.emad
As an alternative to the math behaviour we can use Not from simple example with events.
```
package test1;

component Not {
    ports 
        in B in1,
        out B out1;
    
    @ in1::value( false ) {
        connect 1 -> out1;
    }
    @ in1::value( true ){
        connect 0 -> out1;
    }
}
```

## CPP Files:

### Not.h
```cpp
#ifndef Not_h
#define Not_h

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "PortValueCheck.h"

class Not{
protected:
    PortValueCheck<bool, 1> pvc_in1_1;
    PortValueCheck<bool, 1> pvc_in1_2;
    
    void next(){
        this->pvc_in1_1.next();
        this->pvc_in1_2.next();
    }
    
    bool eventhandler_1_condition(){
        return pvc_in1_1.check();
    }
    void eventhandler_1_outputs(){
        if(eventhandler_1_condition()){
            out1 = CONSTANTPORT1;
        }
    }
    
    bool eventhandler_2_condition(){
        return pvc_in1_2.check();
    }
    void eventhandler_2_outputs(){
        if(eventhandler_2_condition()){
            out1 = CONSTANTPORT2;
        }
    }
    
    void event_outputs(){
        eventhandler_1_outputs();
        eventhandler_2_outputs();
    }
public:
    bool in1;
    bool out1;
    bool CONSTANTPORT1;
    bool CONSTANTPORT2;
    
    void init()
    {
        //init ports
        this->CONSTANTPORT1 = false;
        this->CONSTANTPORT2 = true;
        
        this->pvc_in1_1.setPortReference(&in1);
        this->pvc_in1_1.setEqualsTest(0, false, true);
        
        this->pvc_in1_2.setPortReference(&in1);
        this->pvc_in1_2.setEqualsTest(0, true, false);
        
    }
    void execute()
    {
        next();
        
        //subcomponents write inports
        // subcomponents -> execute
        
        //Write Outports:
        event_outputs();
    }
    
};

#endif /* Not_h */
```

### Not Adapter.h
```cpp
#ifndef NotAdapter_h
#define NotAdapter_h

#include <queue>          // std::queue
#include "Not.h"

#ifndef CONNECTION_H
#define CONNECTION_H
template<typename T>
struct connection {
    T *from;
    T *to;
};
#endif


class NotAdapter {
    
    Not notInstance[42];
    bool notInstance_instances_connected[42] = {};
    
    bool in1_connected[32] = {};
    std::queue<int> in1_connect_request;

    bool out1_connected[32]= {};
    std::queue<int> out1_connect_request;

    
    std::vector<connection<bool>> dynamic_bool_connection_IN;
    std::vector<connection<bool>> dynamic_bool_connection_OUT;

    void dynamic_input_connections(){
        for (std::vector<connection<bool>>::iterator it = dynamic_bool_connection_IN.begin(); it < dynamic_bool_connection_IN.end(); ++it) {
            *(*it).to = *(*it).from;
        }
    }
    
    void dynamic_output_connections(){
        for (std::vector<connection<bool>>::iterator it = dynamic_bool_connection_OUT.begin(); it < dynamic_bool_connection_OUT.end(); ++it) {
            *(*it).to = *(*it).from;
        }
    }
    
    int connect(int numPorts, bool* connected){
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
    
    int connect(int numPorts, bool* connected, std::queue<int>* request){
        
        int port = connect(numPorts, connected);
        
        if(port >= 0){
            request->push(port);
        }
        
        return port;
    }
    
    bool eventhandler_3_condition(){
        return !in1_connect_request.empty() && !out1_connect_request.empty();
    }
    void eventhandler_3_inputs(){
        if(eventhandler_3_condition()){
            //get connected input ports
            int portID_in1 = in1_connect_request.front(); in1_connect_request.pop();
            
            //get connected output ports
            int portID_out1 = this->out1_connect_request.front(); out1_connect_request.pop();
            
            int portID_notInstance = connect(42, notInstance_instances_connected);
            
            //check portID_*

            //workaround that in1 will be passed through
            notInstance[portID_notInstance].in1 = in1[portID_in1];
            
            //dynamic input connections
            dynamic_bool_connection_IN.push_back({&in1[portID_in1], &notInstance[portID_notInstance].in1});
            
            //dynamic output connections
            dynamic_bool_connection_OUT.push_back({ &notInstance[portID_notInstance].out1, &out1[portID_out1] });
        }
    }
    
    void event_inputs(){
        //eventhandler_X_inputes
        eventhandler_3_inputs();
    }
    
    void dynamic_executes(){
        for(int i =0; i < 42; ++i){
            if(notInstance_instances_connected[i]){
                notInstance[i].execute();
            }
        }
    }
    
public:
    bool in1[32] = {};
    bool out1[32] = {};

    void init(){
        
        //init subcomponents
        for (int i = 0; i < 42; ++i){
            notInstance[i].init();
        }
    }
    
    void execute(){
        //
        dynamic_input_connections();
        event_inputs();
        
        dynamic_executes();
        
        dynamic_output_connections();
    }
    
    //interfaces of eventhandler in1::connect && out1::connect
    bool connect_in1_out1(int* in1PortRef, int* out1PortRef){
        *in1PortRef = connect(32, in1_connected, &in1_connect_request);
        *out1PortRef = connect(32, out1_connected, &out1_connect_request);
        if(*in1PortRef < 0 || *out1PortRef < 0){
            return false;
        }
        
        return true;
        
    }
    
};
#endif /* NotAdapter_h */
```

## Demo to run (test) the code above
```cpp
#include "NotAdapter.h"

using namespace std;

int main()
{
	NotAdapter na;
        na.init();
        int a_in = -1;
        int a_out = -1;
        na.connect_in1_out1(&a_in, &a_out);
        na.in1[a_in] = false;
        
        na.execute();
        cout << na.out1[a_out] << endl;
        
        
        int b_in =-1;
        int b_out =-1;
        na.connect_in1_out1(&b_in, &b_out);
        
        na.in1[a_in] = true;
        na.in1[b_in] = false;
        na.execute();
        cout << na.out1[a_out] << "\t" << na.out1[b_out] << endl;
        
        return 0;
}
``` 

## PortValueCheck
```cpp
#ifndef PortValueCheck_h
#define PortValueCheck_h

#include <iostream>
using namespace std;

template <typename T>
class TestsInterface {
public:
    virtual bool check(){
        return false;
    }
};

template <typename T>
class EqualsTest: public TestsInterface<T>{
protected:
    T value;
    T* refValue;
public:
    EqualsTest(T val, T* rVal){
        value = val;
        refValue = rVal;
    }
    bool check(){
        return value == *refValue;
    }
};



template < typename T, std::size_t N >
class PortValueCheck {
    T* portReference;
    T valueHistory[N];
    TestsInterface<T>* checkers[N];
    
    bool isChecked = false;
    bool checkedValue = false;
    
public:
    
    void setPortReference(T* ref){
        this->portReference = ref;
    }
    
    void setEqualsTest(int index, T initValue, T value){
        checkers[index] = new EqualsTest<T>(value, &valueHistory[index]);
        valueHistory[index] = initValue;
    }
    
    void next(){
        for(int i = 0; i < (N-1); ++i){
            valueHistory[i] = valueHistory[i+1];
        }
        valueHistory[N-1] = *portReference;
        isChecked = false;
    }
    
    bool check(){
        if(isChecked){
            return checkedValue;
        }
        
        isChecked = true;
        for(int i = 0; i < N; ++i){
            if(!checkers[i]->check()){
                checkedValue = false;
                return false;
            }
        }
        checkedValue = true;
        return true;
    }
};

#endif /* PortValueCheck_h */
```
