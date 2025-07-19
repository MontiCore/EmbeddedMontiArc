/* (c) https://github.com/MontiCore/monticore */
#ifndef BA_INTERSECTION_INTERSECTIONCONTROLLER_TRAJECTORYTOSTOP_CONFLICTTOSTOPLOOKUP
#define BA_INTERSECTION_INTERSECTIONCONTROLLER_TRAJECTORYTOSTOP_CONFLICTTOSTOPLOOKUP
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class ba_intersection_intersectionController_trajectoryToStop_conflictToStopLookup{
const int n = 3;
const int x = 6;
public:
sword indexLookup[6];
void init()
{
}
void execute()
{
    imat counter = imat(n,1);
    for(int i = 1; i <= n; ++i){
        counter[i-1,1-1] = 0;
    }

    sword k = 1;
    //TODO: remove once generator can handle i = n-1:n
    sword maxI = n - 1;
    for(sword i = 1; i <= maxI;++i){
        sword minJ = i + 1;
        for(sword j = minJ; j <= n;++j){
            counter(i-1,1-1) = counter(i-1,1-1) + 1;
            counter(j-1,1-1) = counter(j-1,1-1) + 1;

            if(counter(i-1,1-1) <= counter(j-1,1-1)){
                indexLookup[k-1] = i;
            }else{
                indexLookup[k-1] = j;
            }
            k = k + 1;
        }
    }
}

};
#endif
