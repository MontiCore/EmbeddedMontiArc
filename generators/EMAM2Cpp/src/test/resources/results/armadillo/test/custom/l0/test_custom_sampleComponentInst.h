/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_CUSTOM_SAMPLECOMPONENTINST
#define TEST_CUSTOM_SAMPLECOMPONENTINST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "test_custom_sampleComponentInst_inst.h"
using namespace arma;
class test_custom_sampleComponentInst{
public:
test_custom_sampleComponentInst_inst inst;
void init()
{
inst.init();
}
void execute()
{
inst.execute();
}

};
#endif
