#include <iostream>
#include <map>
#include <string>
#include "mxnet-cpp/MxNetCpp.h"
// Allow IDE to parse the types
#include "../include/mxnet-cpp/op.h"

using namespace std;
using namespace mxnet::cpp;


Symbol createModel() {

    input = Symbol::Variable("data");

<#list layers as layer>
    ${tc.include(layer)}
</#list>

}

void trainModel()

int main(int argc, char const *argv[]) {

}
