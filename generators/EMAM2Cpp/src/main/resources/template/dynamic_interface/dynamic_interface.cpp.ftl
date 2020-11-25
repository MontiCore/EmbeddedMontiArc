/** (c) https://github.com/MontiCore/monticore */
#include "dynamic_interface.h"
#include "json.h"

#include "./${mainModelName}.h"

${mainModelName} program_instance;
JsonWriter writer;

extern "C" {

EXPORT const char* DI__get_interface() {
    return R"(
${interfaceDescription}
    )";
}
EXPORT void DI__set_port(int i, const char* data) {
    JsonTraverser traverser;
    traverser.init(data);
    switch(i){
<#list setPortCases as setPortCase>
${setPortCase}
</#list>
    }
}
EXPORT const char* DI__get_port(int i) {
    writer.init();
    switch (i){
<#list getPortCases as getPortCase>
${getPortCase}
</#list>
    }
    return writer.get_string();
}


// Methods
EXPORT void DI__init() {
    writer.format = false;
    srand(time(NULL));
    program_instance.init();
}
EXPORT void DI__execute(double delta_sec) {
    // TODO set "delta_sec" port ?
    program_instance.execute();
}

}