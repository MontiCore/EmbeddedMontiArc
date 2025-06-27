#include "program.h"
#include <inttypes.h>
#include "./${mainModelName}.h"

#include "err_out.h"

static constexpr auto N_TO_N_BROADCAST_ADDR = "ff02::2";
static constexpr auto N_TO_N_PREFIX = "2::";

namespace autopilot {
DynamicBuffer buffer;

${mainModelName} program_instance;

void set_port_json(int32_t i, const char* data) {
    JsonTraverser reader{data};
    switch(i){
<#list setPortJsonCases as i>${i}</#list>
        default:
            throw_error("PortIO", "[set_port_json()] Invalid input port ID: '%PRIi32'");
    }
}
void get_port_json(int32_t i, JsonWriter &writer) {
    switch (i){
<#list getPortJsonCases as i>${i}</#list>
        default:
            throw_error("PortIO", "[get_port_json()] Invalid output port ID: '%PRIi32'");
    }
}

void set_port_binary(int32_t i, BinaryReader &reader) {
    switch (i){
<#list setPortBinaryCases as i>${i}</#list>
        default:
            throw_error("PortIO", "[set_port_binary()] Invalid output port ID: '%PRIi32'");
    }
}
void get_port_binary(int32_t i, BinaryWriter &writer) {
    switch (i){
<#list getPortBinaryCases as i>${i}</#list>
        default:
            throw_error("PortIO", "[get_port_binary()] Invalid output port ID: '%PRIi32'");
    }
}


// Methods
void init() {
    srand(time(NULL));
    program_instance.init();

// TODO: Should be handled in the component init() code
<#list initDataCalls as initDataCall>${initDataCall}</#list>
}
void execute(double delta_sec) {
    // TODO set "delta_sec" port ?
    program_instance.execute();
}

const int32_t PORT_COUNT = ${portCount};

const bool IS_SOCKET[] = {
    ${isSocket}
};

const bool IS_OUTPUT[] = {
    ${isOutput}
};

const char* PROGRAM_INTERFACE = R"(
${interfaceDescription}
)";

}
