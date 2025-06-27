/** (c) https://github.com/MontiCore/monticore */
#include "library_interface.h"
#include "program.h"
#include "json.h"

extern "C" {

EXPORT const char* DI__get_interface() {
    return autopilot::PROGRAM_INTERFACE;
}

EXPORT void DI__set_port(int i, const char* data, int is_json) {
    if (is_json == 0) {
        BinaryReader br{data, 4};
        auto payload_size = br.read_u32();
        br = BinaryReader(data+4, payload_size);
        autopilot::set_port_binary(i, br);
    } else {
        autopilot::set_port_json(i, data);
    }
}
EXPORT const char* DI__get_port(int i, int is_json) {
    autopilot::buffer.reset();
    if (is_json == 0) {
        BinaryWriter bw{autopilot::buffer};
        bw.write_u32(0); // Write size later
        autopilot::get_port_binary(i,bw);
        auto payload_size = bw.buffer.position() - 4;
        bw.buffer.go_to(0);
        bw.write_u32(payload_size);
        return bw.buffer.get_buffer();
    } else {
        JsonWriter writer {autopilot::buffer};
        autopilot::get_port_json(i, writer);
        return writer.get_string();
    }
}


// Methods
EXPORT void DI__init() {
    autopilot::init();
}
EXPORT void DI__execute(double delta_sec) {
    autopilot::execute(delta_sec);
}

}