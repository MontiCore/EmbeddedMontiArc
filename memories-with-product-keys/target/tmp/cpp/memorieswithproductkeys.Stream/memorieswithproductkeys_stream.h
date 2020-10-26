#ifndef MEMORIESWITHPRODUCTKEYS_STREAM
#define MEMORIESWITHPRODUCTKEYS_STREAM
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class memorieswithproductkeys_stream{
public:
int hundreds;
int tens;
int ones;
int number;
void init()
{
}
void execute()
{
number = ones+10*tens+100*hundreds;
}

};
#endif
