#include "de_monticore_lang_monticar_semantics_loops_oscillationAsODESymbol.h"
#include "ExecutionStepper.h"
int main(int argc, char** argv)
{
    de_monticore_lang_monticar_semantics_loops_oscillationAsODESymbol instance;
    int i;
    for (i=1; i<atof(argv[1]); i++) {
        double realResult = 2*cos((sqrt((10))*getCurrentTime()/5));
        instance.execute();
        std::cout << "t = " << getCurrentTime() << ":\t " << instance.x << " \t error: \t " << realResult - instance.x << "\n";
    }
    return 0;
}