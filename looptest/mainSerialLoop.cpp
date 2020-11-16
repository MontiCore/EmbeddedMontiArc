#include "de_monticore_lang_monticar_semantics_loops_serialLoop.h"
int main(int argc, char** argv)
{
    de_monticore_lang_monticar_semantics_loops_serialLoop instance;
    instance.init();
    int i;
    for (i=1; i<argc; i++) {
        instance.in1 = atof(argv[i]);
        instance.execute();
        std::cout << instance.out1 << "\n";
    }
    return 0;
}