#include "de_monticore_lang_monticar_semantics_loops_simpleLoop.h"
#include "ExecutionStepper.h"
int main(int argc, char** argv)
{
    de_monticore_lang_monticar_semantics_loops_simpleLoop instance;
    instance.init();
    int i;
    for (i=1; i<argc; i++) {
        instance.input = atof(argv[i]);
        instance.execute();
        std::cout << "t = " << getCurrentTime() << ":\t " << instance.output << "\n";
    }
    return 0;
}