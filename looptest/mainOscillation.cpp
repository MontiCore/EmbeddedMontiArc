#include "de_monticore_lang_monticar_semantics_loops_oscillation.h"
int main(int argc, char** argv)
{
    de_monticore_lang_monticar_semantics_loops_oscillation instance;
    instance.init();
    int i;
    for (i=1; i<atof(argv[1]); i++) {
        instance.execute();
        std::cout << instance.out1 << "\n";
    }
    return 0;
}