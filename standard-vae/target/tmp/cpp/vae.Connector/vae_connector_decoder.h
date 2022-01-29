#ifndef VAE_CONNECTOR_DECODER
#define VAE_CONNECTOR_DECODER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class vae_connector_decoder{
public:
colvec encoding;
cube data;
void init()
{
encoding=colvec(2);
data = cube(1, 28, 28);
}
void execute()
{
}

};
#endif
