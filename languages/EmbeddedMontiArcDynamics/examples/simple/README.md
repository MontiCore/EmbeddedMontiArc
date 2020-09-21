<!-- (c) https://github.com/MontiCore/monticore -->
# Simple Example

A simple example for a not component. 
This is the easiest model, for now.
There is no dynamic behavior, except the conditioned execution of the two event handling parts.

## EMAD Files:

### Not.emad
```
package test1;

component Not {
    port
        in B in1,
        out B out1;
        

    @ in1::value( true ) {
        connect false -> out1;
    }
    
    @ in1::value( false ) {
        connect true -> out1;
    }
    
}
```

## CPP:

### not.h
```
#ifndef TEST1_NOT
#define TEST1_NOT

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "armadillo.h"

using namespace arma;

class test1_not{
    protected:
        PortValueChecker pvc_in1_1;
        PortValueChecker pvc_in1_2;
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
            
            this->pvc_in1_1 = new PortValueChecker(&this->in1, true);
            this->pvc_in1_2 = new PortValueChecker(&this->in1, false);
        }
        void execute()
        {
            //subcomponents write inports
            // subcomponents -> execute
            
            //Write Outports:
            if(this->pvc_in1_1.active()){
                out1 = CONSTANTPORT1;
            }
            if(this->pvc_in1_2.active()){
                out1 = CONSTANTPORT2;
            }
            
            //set next as time passes by
            this->pvc_in1_1.next();
            this->pvc_in1_2.next();
        }

};
#endif
```
