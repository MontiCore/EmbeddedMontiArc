/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_CUSTOM_SAMPLECOMPONENTINST_INST
#define TEST_CUSTOM_SAMPLECOMPONENTINST_INST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_custom_sampleComponentInst_inst{
const int tilesMaxX = 16;
const int tilesMaxY = 16;
public:
int posX;
int posY;
imat world;
void init()
{
world=imat(tilesMaxX,tilesMaxY);
}
void execute()
{
int zvalue = world(posX-1, posY-1);
}

};
#endif
