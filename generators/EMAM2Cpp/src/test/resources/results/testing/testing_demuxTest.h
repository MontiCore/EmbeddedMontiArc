/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_DEMUXTEST
#define TESTING_DEMUXTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class testing_demuxTest{
public:
bool address[3];
double demux_input;
double demux_output[3];
void init()
{
}
void execute()
{
for( auto i=1;i<=3;i+=1){
if((address[i-1] == 1)){
demux_output[i-1] = demux_input;
}
}
}

};
#endif
